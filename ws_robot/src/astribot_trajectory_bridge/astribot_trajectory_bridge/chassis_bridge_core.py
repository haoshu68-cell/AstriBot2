#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘桥接的控制核心（状态机）。**不依赖 rclpy、不依赖厂商 SDK。**

依赖全部通过端口注入（见 ports.py），所以使能/leash/闭环/异常这些分支可以在
离线环境里全部跑通 —— 这是本模块不必等真机就能被验证的前提。

状态机
======
::

    DISABLED ──enable()──▶ ENABLED ──leash 触发──▶ LEASH_TRIPPED
       ▲                    │  ▲                        │
       │                    │  └───reset_leash()────────┘
       │                    │
       │                    └──位姿丢失且 require_slam──▶ STOPPED_NO_POSE
       │                                                     │
       └──────────── disable() / 需重新 enable ───────────────┘

设计要点（每一条都对应一个已知风险）
==================================
* **积分种子每次使能重取**（202:44 / 203:43）。复用旧 pos_cmd 的后果：两次使能
  之间机器人被推动或自行漂移，积分起点与真实位置有偏差，一使能就是一个**阶跃
  位置指令** → 底盘猛冲。
* **leash 触发时必须同时冻结积分与校正**。只冻结积分而让外环继续推，校正会持续
  把指令往外拽，等于 leash 没起作用。
* **异常绝不吞**。任何 SDK 调用失败都转成状态位，由调用方上报；桥接不因单次
  失败退出（退出会让 /cmd_vel 彻底断流，比继续上报更糟）。
* **看门狗只把速度置零，不改状态**。cmd_vel 短暂中断是正常工况（Nav2 到点后
  就不发了），不该因此进入故障态。
"""

from astribot_trajectory_bridge.chassis_integrator import (
    ChassisConfigError,
    IDX_THETA,
    clamp_velocity,
    integrate_step,
    slew_limit_velocity,
    to_local_velocity,
    wrap_angle,
)
from astribot_trajectory_bridge.chassis_feedback import (
    POSE_SOURCE_GROUND_TRUTH,
    advance_desired_pose,
    check_leash,
    compute_correction,
    detect_pose_jump,
    effective_thresholds,
    is_correction_degenerate,
    leash_recover_command,
    odom_drift,
    slice_correction,
    validate_correction_config,
    validate_leash_config,
)

# 状态
ST_DISABLED = 'DISABLED'
ST_ENABLED = 'ENABLED'
ST_LEASH_TRIPPED = 'LEASH_TRIPPED'
ST_STOPPED_NO_POSE = 'STOPPED_NO_POSE'

# 上报用的状态位名（与 BridgeStatus.msg 的枚举同名，由节点层映射成数字）
S_OK = 'OK'
S_NOT_ENABLED = 'NOT_ENABLED'
S_SDK_CALL_FAILED = 'SDK_CALL_FAILED'
S_CMD_VEL_TIMEOUT = 'CMD_VEL_TIMEOUT'
S_LEASH_TRIPPED = 'LEASH_TRIPPED'
S_SLAM_UNAVAILABLE_OPEN_LOOP = 'SLAM_UNAVAILABLE_OPEN_LOOP'
S_SLAM_STALE = 'SLAM_STALE'
S_SLAM_RELOCALIZED = 'SLAM_RELOCALIZED'
S_ODOM_DRIFT_HIGH = 'ODOM_DRIFT_HIGH'
S_CORRECTION_DEGENERATE = 'CORRECTION_DEGENERATE'
S_SLAM_LOST_STOPPED = 'SLAM_LOST_STOPPED'
S_POSE_PORT_FAILED = 'POSE_PORT_FAILED'


class StatusEvent:
    """一条待上报的状态。核心不直接发话题，只产出事件，由节点层转成消息。"""

    def __init__(self, code, detail='', metric_1=0.0, metric_2=0.0):
        self.code = code
        self.detail = detail
        self.metric_1 = float(metric_1)
        self.metric_2 = float(metric_2)

    def __repr__(self):
        return 'StatusEvent(%s, %r, %.4f, %.4f)' % (
            self.code, self.detail, self.metric_1, self.metric_2)


class ChassisBridgeConfig:
    """底盘桥接配置。构造时即做合法性校验 —— 非法参数不允许被带进运行期。"""

    def __init__(self, part_name='astribot_chassis', freq=250.0,
                 input_frame='body', theta_reference='at_enable',
                 cmd_vel_timeout_sec=0.3,
                 max_vel_xy=1.0, max_vel_theta=2.0,
                 max_accel_xy=2.5, max_accel_theta=3.2,
                 leash_xy_m=0.25, leash_theta_rad=0.35,
                 require_manual_reset=True,
                 enable_slam_correction=True, pose_source='slam',
                 map_frame='map', base_frame='astribot_torso_base',
                 outer_rate=10.0, slam_max_age_sec=0.5,
                 slam_jump_threshold_m=0.30,
                 kp_xy=0.35, kp_theta=0.40,
                 max_corr_vel_xy=0.10, max_corr_vel_theta=0.20,
                 require_slam_to_enable=False, slam_loss_grace_sec=2.0,
                 odom_drift_window_sec=2.0, odom_drift_warn_m=0.15):
        self.part_name = part_name
        self.freq = float(freq)
        self.input_frame = input_frame
        self.theta_reference = theta_reference
        self.cmd_vel_timeout_sec = float(cmd_vel_timeout_sec)
        self.max_vel_xy = float(max_vel_xy)
        self.max_vel_theta = float(max_vel_theta)
        self.max_accel_xy = float(max_accel_xy)
        self.max_accel_theta = float(max_accel_theta)
        self.leash_xy_m = float(leash_xy_m)
        self.leash_theta_rad = float(leash_theta_rad)
        self.require_manual_reset = bool(require_manual_reset)
        self.enable_slam_correction = bool(enable_slam_correction)
        self.pose_source = pose_source
        # 节点层建 TfPosePort 要用；核心自身不查 TF（由 PosePort 注入）
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.outer_rate = float(outer_rate)
        self.slam_max_age_sec = float(slam_max_age_sec)
        self.kp_xy = float(kp_xy)
        self.kp_theta = float(kp_theta)
        self.max_corr_vel_xy = float(max_corr_vel_xy)
        self.max_corr_vel_theta = float(max_corr_vel_theta)
        self.require_slam_to_enable = bool(require_slam_to_enable)
        self.slam_loss_grace_sec = float(slam_loss_grace_sec)
        self.odom_drift_window_sec = float(odom_drift_window_sec)

        if self.cmd_vel_timeout_sec <= 0.0:
            raise ChassisConfigError(
                'cmd_vel_timeout_sec=%r 必须为正：看门狗是 cmd_vel 断流时的'
                '唯一止损，不允许关闭。' % (self.cmd_vel_timeout_sec,))
        validate_leash_config(self.leash_xy_m, self.leash_theta_rad)
        validate_correction_config(self.kp_xy, self.kp_theta,
                                   self.max_corr_vel_xy, self.max_corr_vel_theta,
                                   self.outer_rate, self.freq)
        # 按位姿源自动失效没有意义的阈值（见 chassis_feedback.effective_thresholds）
        self.slam_jump_threshold_m, self.odom_drift_warn_m = effective_thresholds(
            self.pose_source, float(slam_jump_threshold_m), float(odom_drift_warn_m))
        # 提前算一次，避免每帧重复校验 input_frame
        to_local_velocity((0.0, 0.0, 0.0), self.input_frame, 0.0)


class ChassisBridgeCore:
    """底盘控制核心。节点层只负责：喂 twist、按频率调 tick、把事件转成话题。"""

    def __init__(self, cfg, session, pose_port, clock):
        self.cfg = cfg
        self.session = session
        self.pose = pose_port
        self.clock = clock

        self.state = ST_DISABLED
        self.pos_cmd = None
        self.theta_ref = 0.0
        self.corr_per_tick = (0.0, 0.0, 0.0)
        self.corr_frozen = True

        self._last_twist = (0.0, 0.0, 0.0)
        self._last_twist_time = None
        self._prev_vel_out = (0.0, 0.0, 0.0)

        self._p_des_map = None
        self._p_slam_prev = None
        self._pose_lost_since = None
        # 外环用：累积"上一次外环以来的本体位移"，供 advance_desired_pose 推进
        self._body_disp_accum = [0.0, 0.0]
        self._dtheta_accum = 0.0
        # 漂移诊断窗口：(时刻, sdk_xy, slam_xy)
        self._drift_window = []

        self.events = []

    # ---------------- 事件 ----------------

    def _emit(self, code, detail='', m1=0.0, m2=0.0):
        self.events.append(StatusEvent(code, detail, m1, m2))

    def drain_events(self):
        out = self.events
        self.events = []
        return out

    # ---------------- 使能/停用 ----------------

    def enable(self):
        """使能。返回 (ok, detail)。

        每次使能都**重取积分种子**，绝不复用旧 pos_cmd（202:44 / 203:43）。
        """
        try:
            seed = self.session.get_desired_joints_position([self.cfg.part_name])[0]
        except Exception as exc:      # noqa: BLE001 —— 禁止吞异常，转状态位
            self._emit(S_SDK_CALL_FAILED, '使能时取积分种子失败：%s' % exc)
            return (False, 'get_desired_joints_position 失败：%s' % exc)

        if len(seed) != 3:
            self._emit(S_SDK_CALL_FAILED,
                       '底盘自由度=%d，期望 3。ROBOT_TYPE 未设为 S1 时是 2'
                       '（astribot_base.py:36-38）' % len(seed))
            return (False, '底盘自由度不是 3')

        pose, stamp = self._lookup_pose_checked()
        if pose is None and self.cfg.require_slam_to_enable:
            self._emit(S_SLAM_UNAVAILABLE_OPEN_LOOP,
                       'require_slam_to_enable=true 且位姿源不可用，拒绝使能')
            return (False, '位姿源不可用且 require_slam_to_enable=true')

        self.pos_cmd = [seed[0], seed[1], wrap_angle(seed[2])]
        self.theta_ref = (self.pos_cmd[IDX_THETA]
                          if self.cfg.theta_reference == 'at_enable' else 0.0)
        self._prev_vel_out = (0.0, 0.0, 0.0)
        self._last_twist = (0.0, 0.0, 0.0)
        self._last_twist_time = None
        self.corr_per_tick = (0.0, 0.0, 0.0)
        self._p_slam_prev = None
        self._pose_lost_since = None
        self._body_disp_accum = [0.0, 0.0]
        self._dtheta_accum = 0.0
        self._drift_window = []

        if pose is not None and self.cfg.enable_slam_correction:
            self._p_des_map = list(pose)
            self.corr_frozen = False
        else:
            self._p_des_map = None
            self.corr_frozen = True
            if pose is None:
                self._emit(S_SLAM_UNAVAILABLE_OPEN_LOOP,
                           '位姿源不可用，退化为纯开环 + leash')

        if is_correction_degenerate(self.cfg.pose_source,
                                    self.cfg.enable_slam_correction):
            # 防止把 ground_truth 下的漂亮数字误当成"闭环有效性已验收"
            self._emit(S_CORRECTION_DEGENERATE,
                       'pose_source=ground_truth：map->odom 是恒等静态 TF，'
                       '外环误差恒≈0，闭环不产生实际校正。仅可用于验证代码路径。')

        self.state = ST_ENABLED
        return (True, 'enabled')

    def disable(self):
        self.state = ST_DISABLED
        self.corr_frozen = True
        self.corr_per_tick = (0.0, 0.0, 0.0)
        self._emit(S_NOT_ENABLED, '已停用')
        return (True, 'disabled')

    def reset_leash(self):
        """leash 复位。只在 LEASH_TRIPPED 态有效，且会重取种子。"""
        if self.state != ST_LEASH_TRIPPED:
            return (False, '当前状态 %s 不是 LEASH_TRIPPED，无需复位' % self.state)
        return self.enable()

    # ---------------- 输入 ----------------

    def submit_twist(self, vx, vy, wz):
        self._last_twist = (float(vx), float(vy), float(wz))
        self._last_twist_time = self.clock.now()

    # ---------------- 位姿 ----------------

    def _lookup_pose_checked(self):
        """查位姿并做龄期检查。返回 (pose_or_None, stamp_or_None)。

        龄期检查在两种 pose_source 下都生效：虽然 ground_truth 的 map->odom 是
        静态 TF，但 odom->base 始终动态，合成变换的时间戳由动态那段决定。
        """
        try:
            pose, stamp = self.pose.lookup()
        except Exception as exc:      # noqa: BLE001
            # !!! 这里报 POSE_PORT_FAILED，不是 SDK_CALL_FAILED !!!
            # 位姿查询不是 SDK 调用；报成 SDK 故障会把诊断引向机器人/SDK，
            # 而真实故障在 TF/位姿源。也不折进 SLAM_UNAVAILABLE_OPEN_LOOP ——
            # lookup() 的契约是"查不到返回 (None, None) 而不抛"，抛了就是
            # 端口实现有缺陷，不能伪装成正常降级工况。
            self._emit(S_POSE_PORT_FAILED,
                       '位姿源实现抛异常（契约要求返回 (None, None) 而非抛）：%s' % exc)
            return (None, None)
        if pose is None:
            return (None, None)
        age = self.clock.now() - stamp
        if age > self.cfg.slam_max_age_sec:
            self._emit(S_SLAM_STALE,
                       '位姿龄期 %.3fs 超过阈值 %.3fs' % (age, self.cfg.slam_max_age_sec),
                       age, self.cfg.slam_max_age_sec)
            return (None, None)
        return (pose, stamp)

    # ---------------- 内环 ----------------

    def inner_tick(self):
        """内环一拍（按 cfg.freq 调用）。返回本拍是否真的下发了指令。"""
        if self.state in (ST_DISABLED, ST_LEASH_TRIPPED, ST_STOPPED_NO_POSE):
            return False

        # 看门狗：只把速度置零，不改状态（cmd_vel 短暂中断是正常工况）
        vel_in = self._last_twist
        if self._last_twist_time is None:
            vel_in = (0.0, 0.0, 0.0)
        else:
            idle = self.clock.now() - self._last_twist_time
            if idle > self.cfg.cmd_vel_timeout_sec:
                vel_in = (0.0, 0.0, 0.0)
                self._emit(S_CMD_VEL_TIMEOUT,
                           '/cmd_vel 已 %.3fs 无输入，速度置零' % idle,
                           idle, self.cfg.cmd_vel_timeout_sec)

        vel = clamp_velocity(vel_in, self.cfg.max_vel_xy, self.cfg.max_vel_theta)
        dt = 1.0 / self.cfg.freq
        vel = slew_limit_velocity(vel, self._prev_vel_out,
                                  self.cfg.max_accel_xy, self.cfg.max_accel_theta, dt)
        self._prev_vel_out = vel

        v_local = to_local_velocity(vel, self.cfg.input_frame,
                                    self.pos_cmd[IDX_THETA] - self.theta_ref)
        self.pos_cmd = integrate_step(self.pos_cmd, v_local, self.cfg.freq)

        # 供外环推进 p_des_map 用的本体位移累积
        self._body_disp_accum[0] += v_local[0] * dt
        self._body_disp_accum[1] += v_local[1] * dt
        self._dtheta_accum += v_local[2] * dt

        # 叠加外环校正（已被切成每拍小量）
        if not self.corr_frozen:
            self.pos_cmd = [self.pos_cmd[0] + self.corr_per_tick[0],
                            self.pos_cmd[1] + self.corr_per_tick[1],
                            wrap_angle(self.pos_cmd[2] + self.corr_per_tick[2])]

        # leash（快环，判据 = SDK 实际位置）
        try:
            actual = self.session.get_current_joints_position([self.cfg.part_name])[0]
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '读实际位置失败：%s' % exc)
            return False

        leash = check_leash(self.pos_cmd, actual,
                            self.cfg.leash_xy_m, self.cfg.leash_theta_rad)
        if leash.tripped:
            # 冻结积分 **和** 校正 —— 只冻结积分等于 leash 没起作用
            self.state = ST_LEASH_TRIPPED
            self.corr_frozen = True
            self.corr_per_tick = (0.0, 0.0, 0.0)
            self.pos_cmd = leash_recover_command(actual)
            self._prev_vel_out = (0.0, 0.0, 0.0)
            self._emit(S_LEASH_TRIPPED, leash.reason, leash.err_xy, leash.err_theta)
            return False

        try:
            self.session.set_joints_position(
                [self.cfg.part_name], [list(self.pos_cmd)])
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '下发位置失败：%s' % exc)
            return False
        return True

    # ---------------- 外环 ----------------

    def outer_tick(self):
        """外环一拍（按 cfg.outer_rate 调用）。"""
        if self.state != ST_ENABLED:
            return
        if not self.cfg.enable_slam_correction:
            return

        pose, stamp = self._lookup_pose_checked()
        if pose is None:
            self.corr_frozen = True
            self.corr_per_tick = (0.0, 0.0, 0.0)
            if self._pose_lost_since is None:
                self._pose_lost_since = self.clock.now()
            lost = self.clock.now() - self._pose_lost_since
            if (self.cfg.require_slam_to_enable
                    and lost > self.cfg.slam_loss_grace_sec):
                # 商业化配置：丢位姿超宽限期 → 主动停车，但不冻结 leash、不杀节点
                self.state = ST_STOPPED_NO_POSE
                self._prev_vel_out = (0.0, 0.0, 0.0)
                self._emit(S_SLAM_LOST_STOPPED,
                           '位姿源丢失 %.2fs 超过宽限 %.2fs，已停车'
                           % (lost, self.cfg.slam_loss_grace_sec),
                           lost, self.cfg.slam_loss_grace_sec)
            return

        self._pose_lost_since = None

        # 跳变检测：重定位事件必须重新对齐，不能当误差施加
        jumped, jump = detect_pose_jump(pose, self._p_slam_prev,
                                        self.cfg.slam_jump_threshold_m)
        self._p_slam_prev = list(pose)
        if jumped:
            self._p_des_map = list(pose)
            self.corr_per_tick = (0.0, 0.0, 0.0)
            self._reset_accum()
            self._emit(S_SLAM_RELOCALIZED,
                       '位姿跳变 %.4fm 超过阈值 %.4fm，已重新对齐期望位姿'
                       % (jump, self.cfg.slam_jump_threshold_m),
                       jump, self.cfg.slam_jump_threshold_m)
            return

        if self._p_des_map is None:
            # 之前一直没有位姿（开环），现在恢复了 -> 以当前位姿重新对齐
            self._p_des_map = list(pose)
            self._reset_accum()
            self.corr_frozen = False
            return

        # 用 SLAM 的绝对朝向把本体位移推进到 map 系
        self._p_des_map = advance_desired_pose(
            self._p_des_map, tuple(self._body_disp_accum),
            self._dtheta_accum, pose[IDX_THETA])
        self._reset_accum()

        corr = compute_correction(self._p_des_map, pose,
                                  self.cfg.kp_xy, self.cfg.kp_theta,
                                  self.cfg.max_corr_vel_xy,
                                  self.cfg.max_corr_vel_theta,
                                  self.cfg.outer_rate)
        self.corr_per_tick = slice_correction(corr, self.cfg.freq, self.cfg.outer_rate)
        self.corr_frozen = False

        self._update_drift(pose)

    def _reset_accum(self):
        self._body_disp_accum = [0.0, 0.0]
        self._dtheta_accum = 0.0

    def _update_drift(self, pose):
        """打滑诊断：窗口内两个位移源的模长差，frame 无关（见 odom_drift 说明）。"""
        try:
            sdk = self.session.get_current_joints_position([self.cfg.part_name])[0]
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '漂移诊断读实际位置失败：%s' % exc)
            return
        now = self.clock.now()
        self._drift_window.append((now, (sdk[0], sdk[1]), (pose[0], pose[1])))
        cutoff = now - self.cfg.odom_drift_window_sec
        while len(self._drift_window) > 1 and self._drift_window[0][0] < cutoff:
            self._drift_window.pop(0)
        if len(self._drift_window) < 2:
            return
        t0, sdk0, slam0 = self._drift_window[0]
        _, sdk1, slam1 = self._drift_window[-1]
        drift = odom_drift((sdk1[0] - sdk0[0], sdk1[1] - sdk0[1]),
                           (slam1[0] - slam0[0], slam1[1] - slam0[1]))
        if drift > self.cfg.odom_drift_warn_m:
            moved = ((slam1[0] - slam0[0]) ** 2 + (slam1[1] - slam0[1]) ** 2) ** 0.5
            self._emit(S_ODOM_DRIFT_HIGH,
                       '%.2fs 窗口内里程计与位姿源位移差 %.4fm 超过阈值 %.4fm'
                       % (now - t0, drift, self.cfg.odom_drift_warn_m),
                       drift, moved)
