#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂轨迹执行核心。**不依赖 rclpy、不依赖厂商 SDK。**

方案 B（周期流式 set_joints_position）为主实现，方案 A（move_joints_waypoints）
作为保留接口单独提供。选型理由见设计文档：MoveIt 的 FollowJointTrajectory 语义
要求支持 cancel 与 feedback（本工程的抢占式重规划依赖它），而方案 A 是阻塞调用、
期间不可取消，所以 A 不能承担"回退实现"的角色，只能关在一个明确标注的接口里。

为什么执行过程做成"外部驱动的状态机"而不是内部 while 循环
====================================================
内部 while 循环需要真实时钟和真实线程才能测，等于把 cancel/超时/跟踪误差这些
分支全部推到在线阶段 —— 而在线阶段需要真机，节拍还不可控。
做成 ``start() / step() / 结果`` 的形式后，测试可以用假时钟一拍一拍推进，
把每条分支都走到。
"""

from astribot_trajectory_bridge.arm_traj_math import (
    ArmConfigError,
    INTERP_CUBIC,
    assert_limit_order,
    check_time_monotonic,
    check_within_limits,
    cross_check_limits,
    drop_non_positive_time_points,
    interpolate_trajectory,
    is_settled,
    max_abs_error,
)

PH_IDLE = 'IDLE'
PH_STREAMING = 'STREAMING'      # 正在按时间轴下发
PH_SETTLING = 'SETTLING'        # 末点已发完，等实际位置收敛
PH_HOLDING = 'HOLDING'          # 取消后持续重发保持位置，等运动停住
PH_DONE = 'DONE'
PH_ABORTED = 'ABORTED'
PH_CANCELED = 'CANCELED'

EC_SUCCESSFUL = 0
EC_INVALID_GOAL = -1
EC_INVALID_JOINTS = -2
EC_OLD_HEADER_TIMESTAMP = -3
EC_PATH_TOLERANCE_VIOLATED = -4
EC_GOAL_TOLERANCE_VIOLATED = -5

S_OK = 'OK'
S_SDK_CALL_FAILED = 'SDK_CALL_FAILED'
S_LIMIT_VIOLATION = 'LIMIT_VIOLATION'
S_LIMIT_SOURCE_MISMATCH = 'LIMIT_SOURCE_MISMATCH'
S_TRACKING_ERROR_EXCEEDED = 'TRACKING_ERROR_EXCEEDED'
S_SETTLE_TIMEOUT = 'SETTLE_TIMEOUT'


class ArmBridgeConfig:
    """机械臂桥接配置。"""

    def __init__(self, part_name='astribot_arm_left', joint_names=None,
                 stream_freq=250.0, control_way='direct', use_wbc=False,
                 add_default_torso=False, interp=INTERP_CUBIC,
                 max_traj_duration_sec=60.0, limit_margin_rad=0.0,
                 cross_check_urdf=True, limit_cross_check_tol_rad=0.01,
                 strict_limit_check=False,
                 max_tracking_error_rad=0.10, abort_on_tracking_error=True,
                 settle_tolerance_rad=0.02, settle_timeout_sec=2.0,
                 hold_still_epsilon_rad=0.001, hold_still_ticks_required=25,
                 hold_timeout_sec=2.0,
                 allow_recovery_from_oob=True, oob_start_tolerance_rad=0.05):
        self.part_name = part_name
        self.joint_names = list(joint_names or [])
        self.stream_freq = float(stream_freq)
        self.control_way = control_way
        self.use_wbc = bool(use_wbc)
        self.add_default_torso = bool(add_default_torso)
        self.interp = interp
        self.max_traj_duration_sec = float(max_traj_duration_sec)
        self.limit_margin_rad = float(limit_margin_rad)
        self.cross_check_urdf = bool(cross_check_urdf)
        self.limit_cross_check_tol_rad = float(limit_cross_check_tol_rad)
        self.strict_limit_check = bool(strict_limit_check)
        self.max_tracking_error_rad = float(max_tracking_error_rad)
        self.abort_on_tracking_error = bool(abort_on_tracking_error)
        self.settle_tolerance_rad = float(settle_tolerance_rad)
        self.settle_timeout_sec = float(settle_timeout_sec)
        self.hold_still_epsilon_rad = float(hold_still_epsilon_rad)
        self.hold_still_ticks_required = int(hold_still_ticks_required)
        self.hold_timeout_sec = float(hold_timeout_sec)
        self.allow_recovery_from_oob = bool(allow_recovery_from_oob)
        self.oob_start_tolerance_rad = float(oob_start_tolerance_rad)

        if self.stream_freq <= 0.0:
            raise ArmConfigError('stream_freq=%r 必须为正' % (self.stream_freq,))
        if self.settle_tolerance_rad <= 0.0:
            raise ArmConfigError(
                'settle_tolerance_rad=%r 必须为正：收敛判据关掉之后，'
                '"控制器报完成但手臂还在动"这个已实测的坑就会回来。'
                % (self.settle_tolerance_rad,))
        if self.settle_timeout_sec <= 0.0:
            raise ArmConfigError('settle_timeout_sec=%r 必须为正'
                                 % (self.settle_timeout_sec,))
        if self.max_tracking_error_rad <= 0.0:
            raise ArmConfigError('max_tracking_error_rad=%r 必须为正'
                                 % (self.max_tracking_error_rad,))
        if self.hold_still_epsilon_rad <= 0.0:
            raise ArmConfigError(
                'hold_still_epsilon_rad=%r 必须为正：取消后的"停稳"判据关掉，'
                '取消就会在关节还在动的时候落终态。' % (self.hold_still_epsilon_rad,))
        if self.hold_still_ticks_required < 1:
            raise ArmConfigError('hold_still_ticks_required=%r 必须 >= 1'
                                 % (self.hold_still_ticks_required,))
        if self.hold_timeout_sec <= 0.0:
            raise ArmConfigError('hold_timeout_sec=%r 必须为正'
                                 % (self.hold_timeout_sec,))
        if self.oob_start_tolerance_rad <= 0.0:
            raise ArmConfigError(
                'oob_start_tolerance_rad=%r 必须为正' % (self.oob_start_tolerance_rad,))
        if self.oob_start_tolerance_rad > 0.5:
            raise ArmConfigError(
                'oob_start_tolerance_rad=%r 过大：它是"路点 0 是否确实等于当前实测'
                '位置"的判据，给大了就等于放行任意越限起点。' % (self.oob_start_tolerance_rad,))
        if self.add_default_torso:
            raise ArmConfigError(
                'add_default_torso=True：SDK 会隐式附加躯干默认位姿，导致'
                '"只规划了手臂却发现躯干在动"。MoveIt 的躯干由规划器另行管理，'
                '桥接应传 False。若确实需要，请改本校验并写明理由。')


def _oob_amount(q, lower, upper):
    """越限总量（各关节超出边界的绝对值之和）。0 表示完全合法。"""
    total = 0.0
    for v, a, b in zip(q, lower, upper):
        if v < a:
            total += a - v
        elif v > b:
            total += v - b
    return total


class StatusEvent:
    def __init__(self, code, detail='', metric_1=0.0, metric_2=0.0):
        self.code = code
        self.detail = detail
        self.metric_1 = float(metric_1)
        self.metric_2 = float(metric_2)

    def __repr__(self):
        return 'StatusEvent(%s, %r)' % (self.code, self.detail)


class Feedback:
    """一拍的执行反馈，节点层转成 Action feedback。"""

    def __init__(self, t, desired, actual, error):
        self.t = t
        self.desired = list(desired)
        self.actual = list(actual)
        self.error = float(error)


class ArmTrajExecutor:
    """单条轨迹的流式执行器（方案 B）。

    用法（节点层）::

        ok, ec, detail = ex.start(joint_names, times, positions, velocities)
        # 判"是否终态"，**不要**白名单枚举进行中的相位 —— 新增中间相位
        # （比如取消后的 HOLDING）会让白名单写法直接跳出循环，那个相位
        # 一拍都不会被推进，而且不报任何错。
        while ex.phase not in (PH_DONE, PH_CANCELED, PH_ABORTED):
            ex.step()                 # 按 stream_freq 调用
        # ex.phase / ex.error_code / ex.detail 即最终结果

    相位流转::

        IDLE --start--> STREAMING --末点发完--> SETTLING --收敛--> DONE
                            |                      |
                          cancel                 cancel
                            v                      v
                          HOLDING <----------------+
                            |
                    停稳/超时 v
                          CANCELED
    """

    def __init__(self, cfg, session, clock):
        self.cfg = cfg
        self.session = session
        self.clock = clock

        self.phase = PH_IDLE
        self.error_code = EC_SUCCESSFUL
        self.detail = ''
        self.events = []
        self.feedbacks = []

        self._lower = None
        self._upper = None
        self._times = None
        self._positions = None
        self._velocities = None
        self._t0 = None
        self._settle_start = None
        self._cancel_requested = False
        self._last_desired = None
        self._hold_target = None
        self._hold_start = None
        self._hold_last_actual = None
        self._hold_still_ticks = 0


    def _emit(self, code, detail='', m1=0.0, m2=0.0):
        self.events.append(StatusEvent(code, detail, m1, m2))

    def drain_events(self):
        out = self.events
        self.events = []
        return out

    def drain_feedbacks(self):
        out = self.feedbacks
        self.feedbacks = []
        return out


    def load_limits(self, urdf_lower=None, urdf_upper=None):
        """取 SDK 限位并自检。返回 (ok, detail)。

        顺序是 **(lower, upper)** —— astribot_client.py:141。
        examples/100:49 的解包顺序是反的（厂商样例 bug），不以它为准。
        """
        try:
            lower, upper = self.session.get_joints_position_limit([self.cfg.part_name])
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '读限位失败：%s' % exc)
            return (False, 'get_joints_position_limit 失败：%s' % exc)

        res = assert_limit_order(lower[0], upper[0])
        if not res.ok:
            self._emit(S_LIMIT_SOURCE_MISMATCH, res.reason)
            return (False, res.reason)
        self._lower, self._upper = list(lower[0]), list(upper[0])

        if self.cfg.cross_check_urdf and urdf_lower is not None:
            ok, worst, desc = cross_check_limits(
                self._lower, self._upper, urdf_lower, urdf_upper,
                self.cfg.limit_cross_check_tol_rad)
            if not ok:
                self._emit(S_LIMIT_SOURCE_MISMATCH, desc, worst,
                           self.cfg.limit_cross_check_tol_rad)
                if self.cfg.strict_limit_check:
                    return (False, desc)
        return (True, '')


    def start(self, joint_names, times, positions, velocities=None):
        """校验并进入 STREAMING。返回 (ok, error_code, detail)。"""
        self.phase = PH_IDLE
        self.error_code = EC_SUCCESSFUL
        self.detail = ''
        self._cancel_requested = False
        self._settle_start = None
        self._hold_target = None
        self._hold_start = None
        self._hold_last_actual = None
        self._hold_still_ticks = 0

        if self._lower is None:
            return self._reject(EC_INVALID_GOAL,
                                '限位未加载，必须先成功调用 load_limits()')
        if not times or not positions:
            return self._reject(EC_INVALID_GOAL, '空轨迹')
        if len(times) != len(positions):
            return self._reject(EC_INVALID_GOAL,
                                'times 长度 %d != positions 长度 %d'
                                % (len(times), len(positions)))
        if self.cfg.joint_names and list(joint_names) != self.cfg.joint_names:
            return self._reject(
                EC_INVALID_JOINTS,
                '关节名或顺序与本组不符。收到 %s，期望 %s'
                % (list(joint_names), self.cfg.joint_names))
        mono_ok, mono_desc = check_time_monotonic(times)
        if not mono_ok:
            return self._reject(EC_INVALID_GOAL, 'time_from_start 非单调：' + mono_desc)
        if times[-1] > self.cfg.max_traj_duration_sec:
            return self._reject(
                EC_INVALID_GOAL,
                '轨迹总时长 %.3fs 超过上限 %.3fs'
                % (times[-1], self.cfg.max_traj_duration_sec))
        allow_start_oob = False
        actual_now = self._read_actual()
        for i, q in enumerate(positions):
            ok, desc = check_within_limits(q, self._lower, self._upper,
                                           self.cfg.limit_margin_rad)
            if ok:
                continue
            if i == 0 and actual_now is not None and self.cfg.allow_recovery_from_oob:
                near = max_abs_error(q, actual_now)
                if near <= self.cfg.oob_start_tolerance_rad:
                    allow_start_oob = True
                    self._emit(S_LIMIT_VIOLATION,
                               '起点越限但等于当前实测位置（偏差 %.4f rad），'
                               '按恢复轨迹放行：%s' % (near, desc))
                    continue
                self._emit(S_LIMIT_VIOLATION,
                           '路点 0 越限且与当前实测位置不符（偏差 %.4f rad > %.4f），'
                           '拒绝：%s'
                           % (near, self.cfg.oob_start_tolerance_rad, desc))
                return self._reject(
                    EC_INVALID_GOAL,
                    '路点 0 越限且不等于当前位置（偏差 %.4f rad），'
                    '不是恢复轨迹：%s' % (near, desc))
            self._emit(S_LIMIT_VIOLATION, '路点 %d：%s' % (i, desc))
            return self._reject(EC_INVALID_GOAL, '路点 %d 越限：%s' % (i, desc))

        if allow_start_oob:
            worse = _oob_amount(positions[-1], self._lower, self._upper) \
                > _oob_amount(positions[0], self._lower, self._upper)
            if worse:
                return self._reject(
                    EC_INVALID_GOAL,
                    '起点越限，且这条轨迹的终点越限更严重 —— 不是恢复轨迹，拒绝。')

        self._times = list(times)
        self._positions = [list(p) for p in positions]
        self._velocities = ([list(v) for v in velocities]
                            if velocities else None)
        self._t0 = self.clock.now()
        self._last_desired = list(self._positions[0])
        self.phase = PH_STREAMING
        return (True, EC_SUCCESSFUL, '')

    def _reject(self, error_code, detail):
        self.phase = PH_ABORTED
        self.error_code = error_code
        self.detail = detail
        return (False, error_code, detail)

    def request_cancel(self):
        """请求取消。方案 B 支持 —— 停发新点即可，天然可取消。"""
        self._cancel_requested = True


    def step(self):
        """执行一拍。按 cfg.stream_freq 调用。返回当前 phase。"""
        if self.phase == PH_STREAMING:
            return self._step_streaming()
        if self.phase == PH_SETTLING:
            return self._step_settling()
        if self.phase == PH_HOLDING:
            return self._step_holding()
        return self.phase

    def _read_actual(self):
        try:
            return self.session.get_current_joints_position([self.cfg.part_name])[0]
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '读实际关节位置失败：%s' % exc)
            return None

    def _step_streaming(self):
        if self._cancel_requested:
            if self._hold_current():
                self.phase = PH_HOLDING
                self.detail = '收到取消请求，正在保持当前位置'
                return self._step_holding()
            self.phase = PH_CANCELED
            self.detail = '收到取消请求，但读不到实际位置、无法保持'
            return self.phase

        t = self.clock.now() - self._t0
        q = interpolate_trajectory(self._times, self._positions,
                                   self._velocities, t, self.cfg.interp)
        self._last_desired = list(q)

        try:
            self.session.set_joints_position(
                [self.cfg.part_name], [q],
                control_way=self.cfg.control_way,
                use_wbc=self.cfg.use_wbc,
                add_default_torso=self.cfg.add_default_torso)
        except Exception as exc:      # noqa: BLE001 —— 禁止吞异常
            self._emit(S_SDK_CALL_FAILED, '下发关节位置失败：%s' % exc)
            self.phase = PH_ABORTED
            self.error_code = EC_PATH_TOLERANCE_VIOLATED
            self.detail = 'SDK 下发失败：%s' % exc
            return self.phase

        actual = self._read_actual()
        if actual is None:
            self.phase = PH_ABORTED
            self.error_code = EC_PATH_TOLERANCE_VIOLATED
            self.detail = '无法读取实际关节位置'
            return self.phase

        err = max_abs_error(actual, q)
        self.feedbacks.append(Feedback(t, q, actual, err))
        if err > self.cfg.max_tracking_error_rad:
            self._emit(S_TRACKING_ERROR_EXCEEDED,
                       '跟踪误差 %.4frad 超过阈值 %.4frad'
                       % (err, self.cfg.max_tracking_error_rad),
                       err, self.cfg.max_tracking_error_rad)
            if self.cfg.abort_on_tracking_error:
                self.phase = PH_ABORTED
                self.error_code = EC_PATH_TOLERANCE_VIOLATED
                self.detail = '跟踪误差超阈：%.4frad' % err
                return self.phase

        if t >= self._times[-1]:
            self.phase = PH_SETTLING
            self._settle_start = self.clock.now()
        return self.phase

    def _step_settling(self):
        if self._cancel_requested:
            if self._hold_current():
                self.phase = PH_HOLDING
                self.detail = '收敛期间收到取消请求，正在保持当前位置'
                return self._step_holding()
            self.phase = PH_CANCELED
            self.detail = '收敛期间收到取消请求，但读不到实际位置'
            return self.phase

        target = self._positions[-1]
        actual = self._read_actual()
        if actual is None:
            self.phase = PH_ABORTED
            self.error_code = EC_GOAL_TOLERANCE_VIOLATED
            self.detail = '收敛判断时无法读取实际关节位置'
            return self.phase

        try:
            self.session.set_joints_position(
                [self.cfg.part_name], [list(target)],
                control_way=self.cfg.control_way,
                use_wbc=self.cfg.use_wbc,
                add_default_torso=self.cfg.add_default_torso)
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '收敛期下发失败：%s' % exc)
            self.phase = PH_ABORTED
            self.error_code = EC_GOAL_TOLERANCE_VIOLATED
            self.detail = 'SDK 下发失败：%s' % exc
            return self.phase

        if is_settled(actual, target, self.cfg.settle_tolerance_rad):
            self.phase = PH_DONE
            self.error_code = EC_SUCCESSFUL
            self.detail = 'settled'
            return self.phase

        waited = self.clock.now() - self._settle_start
        if waited > self.cfg.settle_timeout_sec:
            resid = max_abs_error(actual, target)
            self._emit(S_SETTLE_TIMEOUT,
                       '等待 %.3fs 仍未收敛，残余误差 %.4frad（容差 %.4frad）'
                       % (waited, resid, self.cfg.settle_tolerance_rad),
                       resid, self.cfg.settle_tolerance_rad)
            self.phase = PH_ABORTED
            self.error_code = EC_GOAL_TOLERANCE_VIOLATED
            self.detail = '收敛超时，残余 %.4frad' % resid
        return self.phase

    def _hold_current(self):
        """进入 HOLDING：记下要保持的位置，由 ``_step_holding`` 持续重发。

        !!! 不能只 dispatch 一次 !!!
        Gate 1-f 实测（MuJoCo 后端，取消一条正在执行的轨迹）：

          * 只发一次 -> 实际位置继续漂 **0.045 ~ 0.37 rad** 然后停在别处，
            而 SDK 侧的 desired 明明已经等于我们发的值 —— 也就是说指令登记上了，
            但**一次指令抓不住一个正在运动的关节**；
          * 以 stream_freq 持续重发 -> 漂移 **0.0000 rad**，稳稳抓住。

        为什么 DONE 路径没暴露这个问题：``_step_settling`` 本来就在"持续把末点
        发下去"（见那里的注释），到 DONE 时关节已经静止，所以停发只掉 0.0047 rad。
        取消路径少了这一步，而取消恰恰发生在关节**带着速度**的时候 —— 于是同一个
        SDK 特性在正常路径上无害、在安全路径上有害。

        取消是安全操作，抓不住等于取消无效，所以这里必须与 settling 对称。
        """
        actual = self._read_actual()
        if actual is None:
            self._emit(S_SDK_CALL_FAILED, '取消时读不到实际位置，无法保持')
            return False
        self._hold_target = list(actual)
        self._hold_start = self.clock.now()
        self._hold_last_actual = list(actual)
        return True

    def _step_holding(self):
        """持续重发保持位置，直到运动停住或超时。

        判据是"实际位置不再变化"，不是"实际位置等于保持目标"——
        实测里保持目标与最终停住的位置本来就差 0.0047 rad 量级的稳态误差，
        用"等于目标"当判据会永远等不到。
        """
        if self._hold_target is None:
            self.phase = PH_CANCELED
            return self.phase

        try:
            self.session.set_joints_position(
                [self.cfg.part_name], [list(self._hold_target)],
                control_way=self.cfg.control_way,
                use_wbc=self.cfg.use_wbc,
                add_default_torso=self.cfg.add_default_torso)
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '保持期下发失败：%s' % exc)
            self.phase = PH_CANCELED
            self.detail = '取消后保持失败：%s' % exc
            return self.phase

        actual = self._read_actual()
        if actual is None:
            self.phase = PH_CANCELED
            self.detail = '取消后保持期读不到实际位置'
            return self.phase

        moved = max_abs_error(actual, self._hold_last_actual)
        self._hold_last_actual = list(actual)
        elapsed = self.clock.now() - self._hold_start

        if moved < self.cfg.hold_still_epsilon_rad:
            self._hold_still_ticks += 1
        else:
            self._hold_still_ticks = 0

        if self._hold_still_ticks >= self.cfg.hold_still_ticks_required:
            self.phase = PH_CANCELED
            self.detail = ('已保持并停稳（保持 %.3fs，最终偏离保持点 %.4f rad）'
                           % (elapsed, max_abs_error(actual, self._hold_target)))
            return self.phase

        if elapsed > self.cfg.hold_timeout_sec:
            self._emit(S_SETTLE_TIMEOUT,
                       '取消后保持 %.3fs 仍未停稳，最近一拍位移 %.4f rad'
                       % (elapsed, moved), elapsed, self.cfg.hold_timeout_sec)
            self.phase = PH_CANCELED
            self.detail = '取消后保持超时（仍在运动）'
            return self.phase

        return self.phase


class WaypointDispatcher:
    """方案 A（保留接口）。**阻塞、期间不可取消。**

    单独成类而不是塞进 ArmTrajExecutor：让"不可取消"这个缺陷留在一个边界清晰的
    地方，不污染 Action 语义。
    """

    def __init__(self, cfg, session, enabled=False):
        self.cfg = cfg
        self.session = session
        self.enabled = bool(enabled)
        self.events = []

    def drain_events(self):
        out = self.events
        self.events = []
        return out

    def dispatch(self, waypoints, time_list, lower=None, upper=None):
        """返回 (ok, error_code_name, detail, dispatched, dropped)。

        error_code_name 用字符串，由节点层映射成 DispatchWaypoints.srv 的常量。
        """
        if not self.enabled:
            return (False, 'DISABLED_BY_CONFIG',
                    'enable_waypoints_service=false，方案 A 未启用', 0, 0)
        try:
            kept_wp, kept_t, dropped = drop_non_positive_time_points(
                waypoints, time_list)
        except ArmConfigError as exc:
            return (False, 'SHAPE_MISMATCH', str(exc), 0, 0)
        if not kept_wp:
            return (False, 'NO_POINTS_AFTER_DROP',
                    '丢弃 t<=0 的点后没有路点剩下（examples/206：t=0 由当前位置隐含）',
                    0, dropped)
        mono_ok, mono_desc = check_time_monotonic(kept_t)
        if not mono_ok:
            return (False, 'TIME_NOT_MONOTONIC', mono_desc, 0, dropped)
        if lower is not None:
            for i, q in enumerate(kept_wp):
                ok, desc = check_within_limits(q, lower, upper,
                                               self.cfg.limit_margin_rad)
                if not ok:
                    self.events.append(
                        StatusEvent(S_LIMIT_VIOLATION, '路点 %d：%s' % (i, desc)))
                    return (False, 'LIMIT_VIOLATION',
                            '路点 %d 越限：%s' % (i, desc), 0, dropped)
        try:
            self.session.move_joints_waypoints(
                [self.cfg.part_name], [kept_wp], kept_t,
                use_wbc=self.cfg.use_wbc,
                add_default_torso=self.cfg.add_default_torso)
        except Exception as exc:      # noqa: BLE001
            self.events.append(StatusEvent(S_SDK_CALL_FAILED, str(exc)))
            return (False, 'SDK_CALL_FAILED',
                    'move_joints_waypoints 失败：%s' % exc, 0, dropped)
        return (True, 'SUCCESS', '', len(kept_wp), dropped)
