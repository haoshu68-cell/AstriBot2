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

import collections

from astribot_trajectory_bridge.chassis_integrator import (
    ChassisConfigError,
    IDX_THETA,
    clamp_velocity,
    integrate_step_dt,
    measure_tick_dt,
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
#: 内环步长被钳位。metric_1=钳位前的实测步长(s)，metric_2=上限(s)。
#: 偶发说明调度抖动，持续出现说明内环真的跟不上，两种都必须可见 ——
#: 步长直接乘在速度上，静默钳位等于静默改变底盘速度。
S_TICK_DT_CLAMPED = 'TICK_DT_CLAMPED'

#: :meth:`ChassisBridgeCore.tick_stats` 的返回值。
#: ``rate_hz`` 是 count / 墙钟时长，**无偏** —— 与只在超阈时上报的
#: LOOP_OVERRUN 不同，后者的周期均值是截尾样本，不能当平均周期用。
#: ``live``：统计窗口是否**正在累积**（核心处于使能态）。停用后 count/rate 会
#: 停在上一段使能期间的值上，若不带这个标志，陈旧读数与当前读数长得一模一样。
TickStats = collections.namedtuple(
    'TickStats', 'count mean_dt rate_hz clamp_count clamp_ratio live')


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
                 odom_drift_window_sec=2.0, odom_drift_warn_m=0.15,
                 max_tick_dt_sec=0.04):
        self.part_name = part_name
        self.freq = float(freq)
        self.input_frame = input_frame
        self.theta_reference = theta_reference
        self.cmd_vel_timeout_sec = float(cmd_vel_timeout_sec)
        self.max_vel_xy = float(max_vel_xy)
        self.max_vel_theta = float(max_vel_theta)
        self.max_accel_xy = float(max_accel_xy)
        self.max_accel_theta = float(max_accel_theta)
        # 内环步长上限。默认 0.04s = 标称 4ms 的 10 倍。
        #
        # 怎么定的：实测超时周期最大 14.9ms（≈3.7 倍），取 10 倍留足余量，
        # 正常抖动不会被钳（钳了就等于没修这个缺陷）。上界的物理含义是
        # **单拍最大位移** = max_vel_xy * max_tick_dt_sec = 1.0 * 0.04 = 0.04m，
        # 必须远小于 leash_xy_m=0.25 —— 否则一次调度停顿就能把 leash 撞开。
        self.max_tick_dt_sec = float(max_tick_dt_sec)
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
        # 步长上限的两条硬约束，任一不满足都直接拒绝启动：
        #   ① 不能小于标称步长 —— 否则连正常拍都被钳，积分恒等于钳位值；
        #   ② 单拍最大位移必须远小于 leash —— 否则一次停顿就能把 leash 撞开，
        #      而 leash 是这条开环位置链路上唯一的硬保护。
        nominal_dt = 1.0 / self.freq if self.freq > 0.0 else float('inf')
        if self.max_tick_dt_sec < nominal_dt:
            raise ChassisConfigError(
                'max_tick_dt_sec=%r 小于标称步长 1/freq=%r：那样每一拍都会被钳位，'
                '积分恒等于钳位值，等于没修"按标称频率积分"这个缺陷。'
                % (self.max_tick_dt_sec, nominal_dt))
        max_tick_disp = self.max_vel_xy * self.max_tick_dt_sec
        if max_tick_disp >= self.leash_xy_m:
            raise ChassisConfigError(
                'max_vel_xy=%r × max_tick_dt_sec=%r = %.4fm，已达到 leash_xy_m=%r。'
                '单拍位移必须远小于 leash，否则一次调度停顿就能把 leash 撞开 —— '
                'leash 是开环位置链路上唯一的硬保护。请减小 max_tick_dt_sec。'
                % (self.max_vel_xy, self.max_tick_dt_sec, max_tick_disp,
                   self.leash_xy_m))
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

        # ---- 内环步长与拍率统计 ----
        # `_tick_count` / `_tick_dt_sum` 是**无偏**计数：每一拍都计，不像
        # LOOP_OVERRUN 只在超阈时才报。LOOP_OVERRUN 的周期均值是截尾样本的均值，
        # 拿它反推拍率会算出 91Hz，而时间账反解的下界是 ≥157Hz —— 差 1.7 倍。
        # 真实拍率至今没有无偏测量值，这两个字段就是为了补上它。
        self._prev_tick_time = None
        self._tick_first_time = None
        self._tick_count = 0
        self._tick_dt_sum = 0.0
        self._tick_clamp_count = 0

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
        # !!! 必须清掉上一次使能留下的时间戳 !!!
        # 不清的话，disable 停了一段时间再 enable，第一拍算出来的 dt 就是那整段
        # 停机时长；乘上速度就是一次巨大的位置阶跃。虽然会被 max_tick_dt_sec
        # 钳住，但那属于"靠钳位兜住逻辑错误"，不是设计意图。置 None 走首拍分支。
        self._prev_tick_time = None
        # 拍率统计窗口也必须一起重开 —— 2026-09-01 实机踩到的：
        # 原来只清 _prev_tick_time 而留着 _tick_first_time，于是
        # rate = count / (_prev_tick_time - _tick_first_time) 的**分母**把
        # disabled 的那段时长也算进去了。实测表现是 65 拍报成 165.4Hz
        # （真值 ~238Hz），停得越久报得越低，而且看不出异常。
        self._reset_tick_window()
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

        # ---- 步长用**实测**值，不用 1/freq ----
        # 底盘是位置接口：物理速度 = 每拍增量 × 墙钟拍率。按 1/freq 积分而实际
        # 拍率更低时，底盘只有指令速度的 (实际拍率/freq)。实测拍率下界 ≥157Hz，
        # 即最多只跑到指令的 63%。改用实测 dt 后拍率快慢不再影响速度。
        # 钳位是必需的：dt 直接乘在速度上，一次停顿就是一次位置阶跃。
        tick = measure_tick_dt(self.clock.now(), self._prev_tick_time,
                               1.0 / self.cfg.freq, self.cfg.max_tick_dt_sec)
        now_tick = self.clock.now()
        if self._tick_first_time is None:
            self._tick_first_time = now_tick
        self._prev_tick_time = now_tick
        dt = tick.dt
        self._tick_count += 1
        self._tick_dt_sum += dt
        if tick.clamped:
            self._tick_clamp_count += 1
            self._emit(S_TICK_DT_CLAMPED, tick.reason,
                       tick.raw if tick.raw is not None else 0.0,
                       self.cfg.max_tick_dt_sec)

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
        # !!! 行为变化 !!! 用实测 dt 后，加速度限幅按 a*dt 放行，每秒允许的
        # 速度变化回到配置的 max_accel；此前按 1/250 计算，实际只放行了
        # (实际拍率/250) 倍，也就是一直比配置**更保守**。这是有意修正，
        # 但它会让加速更快 —— 属运动行为变化，必须上机验。
        vel = slew_limit_velocity(vel, self._prev_vel_out,
                                  self.cfg.max_accel_xy, self.cfg.max_accel_theta, dt)
        self._prev_vel_out = vel

        v_local = to_local_velocity(vel, self.cfg.input_frame,
                                    self.pos_cmd[IDX_THETA] - self.theta_ref)
        self.pos_cmd = integrate_step_dt(self.pos_cmd, v_local, dt)

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

    def tick_stats(self):
        """内环拍率的**无偏**统计。

        ════════════════ 为什么必须是无偏的 ════════════════
        此前唯一能看到拍率的东西是 `LOOP_OVERRUN`，而它**只在周期 > 阈值时上报** ——
        是截尾样本。拿它的周期均值（9.26ms）反推拍率会得到 91Hz，而由超时**发生率**
        （11.4~18.0 次/秒）做时间账反解，下界是 **≥157Hz** —— 差 1.7 倍。
        据前者算出的"底盘只有指令速度的 36%"是错的。

        这里每一拍都计入，所以 `count / 墙钟时长` 就是真实平均拍率，无需推断。

        ════════════════ live 字段：为什么读数必须自带有效性 ════════════════
        `inner_tick` 在 `ST_DISABLED` 下提前返回，所以停用期间计数**不再增长**
        —— 这是对的。但那意味着停用后再读到的 count/rate 全是**上一段使能期间的
        陈旧值**。2026-09-01 我就被这个骗了两次：先据此断言"内环停了"，
        又据此断言"桥接仍是 enabled"，两次都错。

        这与 map_odom_tf 那个"陈旧 TF 被 lookup_transform(Time()) 当成最新返回"
        是**同一类缺陷**：一个不再更新的量，被下游当成当前值。
        所以这里把"窗口是否还在累积"做成返回值的一部分，调用方拿不到一个
        看起来正常、实则过期的数字。

        Returns:
            :class:`TickStats`，含 ``live``：True 表示统计窗口正在累积
            （核心处于使能态），False 表示读到的是陈旧值。
            未跑过任何一拍时全 0 且 ``live=False``。
        """
        live = self.state == ST_ENABLED
        if self._tick_count == 0 or self._tick_first_time is None:
            return TickStats(0, 0.0, 0.0, self._tick_clamp_count, 0.0, live)
        elapsed = self._prev_tick_time - self._tick_first_time
        # 只有一拍时 elapsed=0，此时算不出速率，报 0 而不是除零。
        rate = (self._tick_count / elapsed) if elapsed > 0.0 else 0.0
        return TickStats(
            self._tick_count,
            self._tick_dt_sum / self._tick_count,
            rate,
            self._tick_clamp_count,
            self._tick_clamp_count / self._tick_count,
            live)

    def _reset_tick_window(self):
        """重开拍率统计窗口。``enable()`` 必须调 —— 否则速率分母会含停机时长。"""
        self._prev_tick_time = None
        self._tick_first_time = None
        self._tick_count = 0
        self._tick_dt_sum = 0.0
        self._tick_clamp_count = 0

    def reset_tick_stats(self):
        """重置拍率统计窗口。诊断时想看"当前"拍率而非整段使能的均值时用。"""
        self._reset_tick_window()

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
