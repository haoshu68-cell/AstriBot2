#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘桥接的控制核心（状态机）。**不依赖 rclpy、不依赖厂商 SDK。**

依赖全部通过端口注入（见 ports.py），所以使能/leash/观测/异常这些分支可以在
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
* **leash 触发时必须同时冻结积分**。位姿反馈控制由上层 Nav2 负责，
  桥接不额外叠加位置修正。
* **异常绝不吞**。任何 SDK 调用失败都转成状态位，由调用方上报；桥接不因单次
  失败退出（退出会让 /cmd_vel 彻底断流，比继续上报更糟）。
* **看门狗只把速度置零，不改状态**。cmd_vel 短暂中断是正常工况（Nav2 到点后
  就不发了），不该因此进入故障态。
"""

import collections
import math

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
    check_leash,
    detect_pose_jump,
    effective_thresholds,
    leash_recover_command,
    odom_drift,
    validate_leash_config,
)

ST_DISABLED = 'DISABLED'
ST_ENABLED = 'ENABLED'
ST_LEASH_TRIPPED = 'LEASH_TRIPPED'
ST_STOPPED_NO_POSE = 'STOPPED_NO_POSE'
ST_STOPPED_STALE_SCAN = 'STOPPED_STALE_SCAN'

S_OK = 'OK'
S_NOT_ENABLED = 'NOT_ENABLED'
S_SDK_CALL_FAILED = 'SDK_CALL_FAILED'
S_CMD_VEL_TIMEOUT = 'CMD_VEL_TIMEOUT'
S_LEASH_TRIPPED = 'LEASH_TRIPPED'
S_SLAM_UNAVAILABLE_OPEN_LOOP = 'SLAM_UNAVAILABLE_OPEN_LOOP'
S_SLAM_STALE = 'SLAM_STALE'
S_SLAM_RELOCALIZED = 'SLAM_RELOCALIZED'
S_ODOM_DRIFT_HIGH = 'ODOM_DRIFT_HIGH'
S_SLAM_LOST_STOPPED = 'SLAM_LOST_STOPPED'
S_POSE_PORT_FAILED = 'POSE_PORT_FAILED'
S_SCAN_STALE = 'SCAN_STALE'
S_SCAN_LOST_STOPPED = 'SCAN_LOST_STOPPED'
S_SCAN_NEVER_RECEIVED = 'SCAN_NEVER_RECEIVED'
S_TICK_DT_CLAMPED = 'TICK_DT_CLAMPED'

TickStats = collections.namedtuple(
    'TickStats', 'count mean_dt rate_hz clamp_count clamp_ratio live max_dt')

TickWindowGap = collections.namedtuple('TickWindowGap', 'max_dt at over_count ticks')

VelTrace = collections.namedtuple(
    'VelTrace',
    'ticks wall in_peak clamped_peak slewed_peak local_peak '
    'clamp_bit slew_bit zeroed_ticks '
    'cmd_path cmd_net act_net dtheta_cmd dtheta_act corr_path')

VEL_BIT_EPS = 1e-9


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
                 max_accel_xy_up=None,
                 leash_xy_m=0.25, leash_theta_rad=0.35,
                 require_manual_reset=True,
                 enable_slam_correction=False, pose_source='slam',
                 map_frame='map', base_frame='astribot_torso_base',
                 outer_rate=10.0, slam_max_age_sec=0.5,
                 slam_jump_threshold_m=0.30,
                 require_slam_to_enable=False, slam_loss_grace_sec=2.0,
                 odom_drift_window_sec=2.0, odom_drift_warn_m=0.15,
                 max_tick_dt_sec=0.04,
                 require_fresh_scan=True, scan_max_age_sec=0.5,
                 scan_loss_grace_sec=2.0):
        self.part_name = part_name
        self.freq = float(freq)
        self.input_frame = input_frame
        self.theta_reference = theta_reference
        self.cmd_vel_timeout_sec = float(cmd_vel_timeout_sec)
        self.max_vel_xy = float(max_vel_xy)
        self.max_vel_theta = float(max_vel_theta)
        self.max_accel_xy = float(max_accel_xy)
        self.max_accel_theta = float(max_accel_theta)
        self.max_accel_xy_up = (None if max_accel_xy_up is None
                                else float(max_accel_xy_up))
        self.max_tick_dt_sec = float(max_tick_dt_sec)
        self.leash_xy_m = float(leash_xy_m)
        self.leash_theta_rad = float(leash_theta_rad)
        self.require_manual_reset = bool(require_manual_reset)
        if enable_slam_correction:
            raise ChassisConfigError(
                "enable_slam_correction 已退役：定位反馈控制由上层 Nav2 负责")
        self.pose_source = pose_source
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.outer_rate = float(outer_rate)
        self.slam_max_age_sec = float(slam_max_age_sec)
        self.require_slam_to_enable = bool(require_slam_to_enable)
        self.slam_loss_grace_sec = float(slam_loss_grace_sec)
        self.odom_drift_window_sec = float(odom_drift_window_sec)

        self.require_fresh_scan = bool(require_fresh_scan)
        self.scan_max_age_sec = float(scan_max_age_sec)
        self.scan_loss_grace_sec = float(scan_loss_grace_sec)
        if self.require_fresh_scan and self.scan_max_age_sec <= 0.0:
            raise ChassisConfigError(
                'scan_max_age_sec=%r 必须为正。要关闭这道联锁请显式设 '
                'require_fresh_scan=False —— 用非正阈值"顺便"关掉它，'
                '会让配置读起来像开着。' % (self.scan_max_age_sec,))
        if self.require_fresh_scan and self.scan_loss_grace_sec < self.scan_max_age_sec:
            raise ChassisConfigError(
                'scan_loss_grace_sec=%r 小于 scan_max_age_sec=%r：宽限期比判陈旧的'
                '阈值还短，等于跳过"置零"直接闩锁，短暂抖动就要人工复位。'
                % (self.scan_loss_grace_sec, self.scan_max_age_sec))

        if self.cmd_vel_timeout_sec <= 0.0:
            raise ChassisConfigError(
                'cmd_vel_timeout_sec=%r 必须为正：看门狗是 cmd_vel 断流时的'
                '唯一止损，不允许关闭。' % (self.cmd_vel_timeout_sec,))
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
        if (self.max_accel_xy_up is not None
                and self.max_accel_xy_up > self.max_accel_xy):
            raise ChassisConfigError(
                'max_accel_xy_up=%r 大于 max_accel_xy=%r：这个参数的用途是把'
                '**加速**方向限得比减速更紧（底盘真实加速度只有 ~0.39m/s²，'
                '开环位置链在加速段积下的欠账永不归还）。给成更大的值等于'
                '悄悄放宽加速，与它存在的理由正相反。'
                % (self.max_accel_xy_up, self.max_accel_xy))
        validate_leash_config(self.leash_xy_m, self.leash_theta_rad)
        if not (math.isfinite(self.freq) and math.isfinite(self.outer_rate) and
                0 < self.outer_rate <= self.freq):
            raise ChassisConfigError('要求 0 < outer_rate <= freq，且频率为有限值')
        self.slam_jump_threshold_m, self.odom_drift_warn_m = effective_thresholds(
            self.pose_source, float(slam_jump_threshold_m), float(odom_drift_warn_m))
        to_local_velocity((0.0, 0.0, 0.0), self.input_frame, 0.0)


class ChassisBridgeCore:
    """底盘控制核心。节点层只负责：喂 twist、按频率调 tick、把事件转成话题。"""

    def __init__(self, cfg, session, pose_port, clock):
        self.cfg = cfg
        self.session = session
        self.pose = pose_port
        self.clock = clock

        self.state = ST_DISABLED
        self.last_stop = None
        self.pos_cmd = None
        self.theta_ref = 0.0

        self._last_twist = (0.0, 0.0, 0.0)
        self._last_twist_time = None
        self._prev_vel_out = (0.0, 0.0, 0.0)
        self._last_scan_time = None
        self._scan_stale_since = None

        self._prev_tick_time = None
        self._tick_first_time = None
        self._tick_count = 0
        self._tick_dt_sum = 0.0
        self._tick_clamp_count = 0
        self._tick_max_raw = 0.0
        self._tick_win_max_raw = 0.0
        self._tick_win_max_at = None
        self._tick_win_over_count = 0
        self._tick_win_ticks = 0

        self._vel_win = None
        self._reset_vel_window()

        self._p_slam_prev = None
        self._pose_lost_since = None
        self._drift_window = []

        self.events = []


    def _emit(self, code, detail='', m1=0.0, m2=0.0):
        self.events.append(StatusEvent(code, detail, m1, m2))

    def _note_stop(self, state, reason, m1=0.0, m2=0.0):
        """记下"因何掉出使能态"，见 ``self.last_stop`` 的说明。

        刻意**不**复用 ``_emit``：事件会被 ``drain_events()`` 取空，
        而周期日志要的是一个**可反复读**的最近原因。同一个事实需要两种
        生命周期的载体，合并成一个就等于周期日志永远读到 None。
        """
        self.last_stop = (state, reason, m1, m2, self.clock.now())

    def drain_events(self):
        out = self.events
        self.events = []
        return out


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
        self._prev_tick_time = None
        self._reset_tick_window()
        self._reset_vel_window()
        self._p_slam_prev = None
        self._pose_lost_since = None
        self._scan_stale_since = None
        self._drift_window = []

        self.state = ST_ENABLED
        return (True, 'enabled')

    def disable(self):
        self.state = ST_DISABLED
        self._emit(S_NOT_ENABLED, '已停用')
        self._note_stop(ST_DISABLED, '外部调用 ~/disable 主动停用')
        return (True, 'disabled')

    def reset_leash(self):
        """leash 复位。只在 LEASH_TRIPPED 态有效，且会重取种子。"""
        if self.state != ST_LEASH_TRIPPED:
            return (False, '当前状态 %s 不是 LEASH_TRIPPED，无需复位' % self.state)
        return self.enable()


    def submit_twist(self, vx, vy, wz):
        self._last_twist = (float(vx), float(vy), float(wz))
        self._last_twist_time = self.clock.now()

    def submit_scan_seen(self, stamp=None):
        """上报"刚收到一帧 /scan"。节点层在 /scan 订阅回调里调用。

        :param stamp: 该帧的 header 时间戳（秒）。None 表示用当前时刻。

        !!! 为什么默认用**接收时刻**而不是 header.stamp !!!
        上游 pointcloud_slice_scan_node 的 hold_last 策略会把上一帧的几何
        **配上 now() 的新时间戳**重发。也就是说 header.stamp 在保持期间是"新"的，
        拿它判龄期会被骗过去 —— 这正是这道联锁要防的那件事。
        接收时刻至少能反映"话题还在动"，配合上游的 hold_last_max_frames=5
        上限（超过就真的停止输出），两者合起来才封住整个窗口：
            · 上游保持 <=0.5s：header 是新的、接收也在动 → 本联锁不触发（有意）
            · 上游超过上限停发：接收时刻不再更新 → 本联锁在 0.5s 后触发
        传 stamp 的用法留给"就是要按 header 判"的场景，但要清楚它可以被骗。
        """
        self._last_scan_time = self.clock.now() if stamp is None else float(stamp)

    def _scan_age(self):
        """返回 (龄期秒, 是否从未收到过)。"""
        if self._last_scan_time is None:
            return (float('inf'), True)
        return (self.clock.now() - self._last_scan_time, False)


    def _lookup_pose_checked(self):
        """查位姿并做龄期检查。返回 (pose_or_None, stamp_or_None)。

        龄期检查在两种 pose_source 下都生效：虽然 ground_truth 的 map->odom 是
        静态 TF，但 odom->base 始终动态，合成变换的时间戳由动态那段决定。
        """
        try:
            pose, stamp = self.pose.lookup()
        except Exception as exc:      # noqa: BLE001
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


    def inner_tick(self):
        """内环一拍（按 cfg.freq 调用）。返回本拍是否真的下发了指令。"""
        if self.state in (ST_DISABLED, ST_LEASH_TRIPPED, ST_STOPPED_NO_POSE,
                          ST_STOPPED_STALE_SCAN):
            return False

        tick = measure_tick_dt(self.clock.now(), self._prev_tick_time,
                               1.0 / self.cfg.freq, self.cfg.max_tick_dt_sec)
        now_tick = self.clock.now()
        if self._tick_first_time is None:
            self._tick_first_time = now_tick
        self._prev_tick_time = now_tick
        dt = tick.dt
        self._tick_count += 1
        self._tick_dt_sum += dt
        if tick.raw is not None and tick.raw > 0.0:
            self._tick_win_ticks += 1
            if tick.raw > self._tick_max_raw:
                self._tick_max_raw = tick.raw
            if tick.raw > self._tick_win_max_raw:
                self._tick_win_max_raw = tick.raw
                self._tick_win_max_at = now_tick
            if tick.raw > 2.0 / self.cfg.freq:
                self._tick_win_over_count += 1
        if tick.clamped:
            self._tick_clamp_count += 1
            self._emit(S_TICK_DT_CLAMPED, tick.reason,
                       tick.raw if tick.raw is not None else 0.0,
                       self.cfg.max_tick_dt_sec)

        raw_in = self._last_twist
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

        if self.cfg.require_fresh_scan:
            age, never = self._scan_age()
            if age > self.cfg.scan_max_age_sec:
                vel_in = (0.0, 0.0, 0.0)
                if self._scan_stale_since is None:
                    self._scan_stale_since = self.clock.now()
                stale_for = self.clock.now() - self._scan_stale_since
                if never:
                    self._emit(S_SCAN_NEVER_RECEIVED,
                               '从未收到 /scan（已等 %.2fs），速度置零。'
                               '先查话题名与 QoS —— BEST_EFFORT 发布配 RELIABLE '
                               '订阅会一帧都收不到且只有一条 WARNING'
                               % stale_for,
                               stale_for, self.cfg.scan_max_age_sec)
                else:
                    self._emit(S_SCAN_STALE,
                               '/scan 龄期 %.3fs 超过 %.3fs，速度置零'
                               % (age, self.cfg.scan_max_age_sec),
                               age, self.cfg.scan_max_age_sec)
                if stale_for > self.cfg.scan_loss_grace_sec:
                    self.state = ST_STOPPED_STALE_SCAN
                    self._prev_vel_out = (0.0, 0.0, 0.0)
                    self._emit(S_SCAN_LOST_STOPPED,
                               '/scan 已持续陈旧 %.2fs 超过宽限 %.2fs，已停车。'
                               '感知失效期间继续行走等于拿旧障碍图开车'
                               % (stale_for, self.cfg.scan_loss_grace_sec),
                               stale_for, self.cfg.scan_loss_grace_sec)
                    self._note_stop(
                        ST_STOPPED_STALE_SCAN,
                        '/scan 已持续陈旧 %.2fs 超过宽限 %.2fs'
                        % (stale_for, self.cfg.scan_loss_grace_sec),
                        stale_for, self.cfg.scan_loss_grace_sec)
                    return False
            else:
                self._scan_stale_since = None

        vel = clamp_velocity(vel_in, self.cfg.max_vel_xy, self.cfg.max_vel_theta)
        vel_clamped = vel
        vel = slew_limit_velocity(vel, self._prev_vel_out,
                                  self.cfg.max_accel_xy, self.cfg.max_accel_theta, dt,
                                  self.cfg.max_accel_xy_up)
        self._prev_vel_out = vel

        v_local = to_local_velocity(vel, self.cfg.input_frame,
                                    self.pos_cmd[IDX_THETA] - self.theta_ref)
        pos_before = list(self.pos_cmd)
        self.pos_cmd = integrate_step_dt(self.pos_cmd, v_local, dt)


        try:
            actual = self.session.get_current_joints_position([self.cfg.part_name])[0]
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '读实际位置失败：%s' % exc)
            return False

        leash = check_leash(self.pos_cmd, actual,
                            self.cfg.leash_xy_m, self.cfg.leash_theta_rad)

        self._record_vel(raw_in, vel_in, vel_clamped, vel, v_local, dt,
                         actual, pos_before)

        if leash.tripped:
            self.state = ST_LEASH_TRIPPED
            self.pos_cmd = leash_recover_command(actual)
            self._prev_vel_out = (0.0, 0.0, 0.0)
            self._emit(S_LEASH_TRIPPED, leash.reason, leash.err_xy, leash.err_theta)
            self._note_stop(ST_LEASH_TRIPPED, leash.reason,
                            leash.err_xy, leash.err_theta)
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
            return TickStats(0, 0.0, 0.0, self._tick_clamp_count, 0.0, live, 0.0)
        elapsed = self._prev_tick_time - self._tick_first_time
        rate = (self._tick_count / elapsed) if elapsed > 0.0 else 0.0
        return TickStats(
            self._tick_count,
            self._tick_dt_sum / self._tick_count,
            rate,
            self._tick_clamp_count,
            self._tick_clamp_count / self._tick_count,
            live,
            self._tick_max_raw)

    def consume_tick_window_gap(self):
        """取走并清零**本窗**的拍间隔极值。

        为什么是 consume 而不是只读：这个量的意义就是"上一条日志到现在"，
        只读会让每条日志都印同一个历史最大值 —— 那正是均值查不出瞬时停顿的
        同一类错误（一个不再更新的量被当成当前值，见 :meth:`tick_stats` 的
        ``live`` 那段）。所以调用方每印一次就必须清一次，名字里写明是 consume。

        Returns:
            :class:`TickWindowGap`。本窗一拍都没跑时全 0 / ``at=None``。
        """
        gap = TickWindowGap(self._tick_win_max_raw, self._tick_win_max_at,
                            self._tick_win_over_count, self._tick_win_ticks)
        self._tick_win_max_raw = 0.0
        self._tick_win_max_at = None
        self._tick_win_over_count = 0
        self._tick_win_ticks = 0
        return gap

    def _reset_tick_window(self):
        """重开拍率统计窗口。``enable()`` 必须调 —— 否则速率分母会含停机时长。"""
        self._prev_tick_time = None
        self._tick_first_time = None
        self._tick_count = 0
        self._tick_dt_sum = 0.0
        self._tick_clamp_count = 0
        self._tick_max_raw = 0.0
        self._tick_win_max_raw = 0.0
        self._tick_win_max_at = None
        self._tick_win_over_count = 0
        self._tick_win_ticks = 0

    def reset_tick_stats(self):
        """重置拍率统计窗口。诊断时想看"当前"拍率而非整段使能的均值时用。"""
        self._reset_tick_window()


    def _reset_vel_window(self):
        """重开速度链路统计窗口。"""
        self._vel_win = {
            'ticks': 0, 't0': None, 't1': None,
            'in_peak': 0.0, 'clamped_peak': 0.0, 'slewed_peak': 0.0,
            'local_peak': 0.0,
            'clamp_bit': 0, 'slew_bit': 0, 'zeroed': 0,
            'cmd_path': 0.0, 'corr_path': 0.0,
            'cmd_xy0': None, 'cmd_th0': None, 'cmd_xy1': None, 'cmd_th1': None,
            'act_xy0': None, 'act_th0': None, 'act_xy1': None, 'act_th1': None,
        }

    def _record_vel(self, raw_in, vel_in, vel_clamped, vel_slewed,
                    v_local, dt, actual, pos_before):
        """记一拍速度链路。纯累加，不打日志（内环 250Hz）。"""
        w = self._vel_win
        now = self.clock.now()
        w['ticks'] += 1
        if w['t0'] is None:
            w['t0'] = now
        w['t1'] = now

        raw_n = math.hypot(raw_in[0], raw_in[1])
        in_n = math.hypot(vel_in[0], vel_in[1])
        cl_n = math.hypot(vel_clamped[0], vel_clamped[1])
        sl_n = math.hypot(vel_slewed[0], vel_slewed[1])
        lo_n = math.hypot(v_local[0], v_local[1])
        w['in_peak'] = max(w['in_peak'], in_n)
        w['clamped_peak'] = max(w['clamped_peak'], cl_n)
        w['slewed_peak'] = max(w['slewed_peak'], sl_n)
        w['local_peak'] = max(w['local_peak'], lo_n)

        if raw_n > VEL_BIT_EPS and in_n <= VEL_BIT_EPS:
            w['zeroed'] += 1
        if any(abs(a - b) > VEL_BIT_EPS
               for a, b in zip(vel_in, vel_clamped)):
            w['clamp_bit'] += 1
        if any(abs(a - b) > VEL_BIT_EPS
               for a, b in zip(vel_clamped, vel_slewed)):
            w['slew_bit'] += 1

        w['cmd_path'] += lo_n * dt
        if w['cmd_xy0'] is None:
            w['cmd_xy0'] = (pos_before[0], pos_before[1])
            w['cmd_th0'] = pos_before[IDX_THETA]
            w['act_xy0'] = (actual[0], actual[1])
            w['act_th0'] = actual[IDX_THETA]
        w['cmd_xy1'] = (self.pos_cmd[0], self.pos_cmd[1])
        w['cmd_th1'] = self.pos_cmd[IDX_THETA]
        w['act_xy1'] = (actual[0], actual[1])
        w['act_th1'] = actual[IDX_THETA]

    def consume_vel_trace(self):
        """取走并清零**本窗**的速度链路取样。

        和 :meth:`consume_tick_window_gap` 同一个理由必须是 consume：这些量的
        意义是"上一条日志到现在"。只读不清会让每条日志印同一段历史 —— 一个不再
        更新的读数被当成当前值，是本项目反复踩到的那一类错误。

        Returns:
            :class:`VelTrace`。本窗一拍都没跑时 ``ticks=0``，调用方应据此跳过打印
            （而不是打一行全 0 —— 全 0 会被读成"链路上确实全是零"）。
        """
        w = self._vel_win
        wall = 0.0
        if w['t0'] is not None and w['t1'] is not None:
            wall = max(0.0, w['t1'] - w['t0'])

        def _net(p0, p1):
            if p0 is None or p1 is None:
                return 0.0
            return math.hypot(p1[0] - p0[0], p1[1] - p0[1])

        def _dth(a0, a1):
            if a0 is None or a1 is None:
                return 0.0
            return wrap_angle(a1 - a0)

        trace = VelTrace(
            ticks=w['ticks'], wall=wall,
            in_peak=w['in_peak'], clamped_peak=w['clamped_peak'],
            slewed_peak=w['slewed_peak'], local_peak=w['local_peak'],
            clamp_bit=w['clamp_bit'], slew_bit=w['slew_bit'],
            zeroed_ticks=w['zeroed'],
            cmd_path=w['cmd_path'],
            cmd_net=_net(w['cmd_xy0'], w['cmd_xy1']),
            act_net=_net(w['act_xy0'], w['act_xy1']),
            dtheta_cmd=_dth(w['cmd_th0'], w['cmd_th1']),
            dtheta_act=_dth(w['act_th0'], w['act_th1']),
            corr_path=w['corr_path'])
        self._reset_vel_window()
        return trace


    def outer_tick(self):
        """低频位姿健康检查和漂移诊断；不修改位置指令。"""
        if self.state != ST_ENABLED:
            return

        pose, stamp = self._lookup_pose_checked()
        if pose is None:
            if self._pose_lost_since is None:
                self._pose_lost_since = self.clock.now()
            lost = self.clock.now() - self._pose_lost_since
            if (self.cfg.require_slam_to_enable
                    and lost > self.cfg.slam_loss_grace_sec):
                self.state = ST_STOPPED_NO_POSE
                self._prev_vel_out = (0.0, 0.0, 0.0)
                self._emit(S_SLAM_LOST_STOPPED,
                           '位姿源丢失 %.2fs 超过宽限 %.2fs，已停车'
                           % (lost, self.cfg.slam_loss_grace_sec),
                           lost, self.cfg.slam_loss_grace_sec)
                self._note_stop(
                    ST_STOPPED_NO_POSE,
                    '位姿源丢失 %.2fs 超过宽限 %.2fs'
                    % (lost, self.cfg.slam_loss_grace_sec),
                    lost, self.cfg.slam_loss_grace_sec)
            return

        self._pose_lost_since = None

        jumped, jump = detect_pose_jump(pose, self._p_slam_prev,
                                        self.cfg.slam_jump_threshold_m)
        self._p_slam_prev = list(pose)
        if jumped:
            self._drift_window = []
            self._emit(S_SLAM_RELOCALIZED,
                       '位姿跳变 %.4fm，重置诊断窗口；桥接不校正位置指令' % jump,
                       jump, self.cfg.slam_jump_threshold_m)
            return
        self._update_drift(pose)

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
