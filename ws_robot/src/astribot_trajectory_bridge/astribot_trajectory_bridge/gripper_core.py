#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪服务的核心逻辑。**不依赖 rclpy**，只依赖 SessionPort。

为什么又是"核心 + 薄节点"
=====================
夹爪服务的分支几乎全是**拒绝路径**：越界、未知夹爪、并发、SDK 抛异常、
配置未启用。这些恰恰是最需要覆盖、又最难在真机上按需复现的部分
（真机上怎么可靠地造出"SDK 抛异常"？）。所以逻辑放在这里离线测，
节点层只做 ROS 类型转换。

错误码用**字符串常量**而不是 import SetGripper.Response，理由与其它核心一致：
让本模块在**没有编译 msgs** 的环境下也能单测。节点层负责映射成数字，
映射的一致性由 test_status_code_map 锁住。
"""

import threading

from astribot_trajectory_bridge.gripper_math import (
    CMD_CLOSED,
    CMD_OPEN,
    GripperConfigError,
    cmd_to_rad,
    describe_cmd,
    opening_fraction_to_cmd,
    validate_grasp_cmd,
)

# 结果码名。必须与 SetGripper.srv 里的常量**同名**。
EC_SUCCESS = 'SUCCESS'
EC_DISABLED_BY_CONFIG = 'DISABLED_BY_CONFIG'
EC_UNKNOWN_GRIPPER = 'UNKNOWN_GRIPPER'
EC_VALUE_OUT_OF_RANGE = 'VALUE_OUT_OF_RANGE'
EC_SDK_CALL_FAILED = 'SDK_CALL_FAILED'
EC_WRITE_GATE_DENIED = 'WRITE_GATE_DENIED'
EC_BUSY = 'BUSY'

# 上报到 BridgeStatus 的状态位名
S_SDK_CALL_FAILED = 'SDK_CALL_FAILED'
# 中间开度在超时内没到位。不静默 —— 上层可能正靠这个开度去抓东西。
S_MID_OPENING_TIMEOUT = 'SETTLE_TIMEOUT'


class GripperConfig:
    """夹爪服务配置。全部可配、不硬编码。"""

    def __init__(self, gripper_names=None, enable_service=True,
                 default_duration_sec=1.0, default_max_force_n=0.0,
                 settle_extra_sec=0.0, stream_freq=250.0,
                 mid_stream_tolerance=0.5, mid_stream_timeout_sec=3.0):
        # 合法夹爪名白名单。**必须显式给**，不从 SDK 现场问 ——
        # 现场问意味着 SDK 挂掉时"未知夹爪"和"SDK 故障"会混成一种错误。
        self.gripper_names = list(gripper_names or [])
        self.enable_service = bool(enable_service)
        self.default_duration_sec = float(default_duration_sec)
        # <=0 表示不设力，沿用 SDK 默认 48N（109 样例注释）
        self.default_max_force_n = float(default_max_force_n)
        # open/close 返回后额外等待的时间。SDK 的调用是阻塞的且尊重 duration，
        # 实测返回时已到位，所以默认 0 —— 保留这个旋钮是为了真机上万一有尾巴。
        self.settle_extra_sec = float(settle_extra_sec)
        # 中间开度必须**持续重发**才到位（单次下发会冲到完全闭合，实测）。
        # 这三个参数控制那个重发过程，全部可配、不硬编码。
        self.stream_freq = float(stream_freq)
        # 到位容差，单位是命令空间（0-100）。实测持续重发的稳态误差 < 0.011，
        # 所以 0.5 已经很宽松。
        self.mid_stream_tolerance = float(mid_stream_tolerance)
        self.mid_stream_timeout_sec = float(mid_stream_timeout_sec)

        if not self.gripper_names:
            raise GripperConfigError(
                'gripper_names 为空：没有白名单就无法区分"未知夹爪"与"SDK 故障"，'
                '拒绝启动。')
        if self.default_duration_sec <= 0.0:
            raise GripperConfigError(
                'default_duration_sec=%r 必须为正' % (self.default_duration_sec,))
        if self.settle_extra_sec < 0.0:
            raise GripperConfigError(
                'settle_extra_sec=%r 不能为负' % (self.settle_extra_sec,))
        if self.stream_freq <= 0.0:
            raise GripperConfigError('stream_freq=%r 必须为正' % (self.stream_freq,))
        if self.mid_stream_tolerance <= 0.0:
            raise GripperConfigError(
                'mid_stream_tolerance=%r 必须为正：容差为 0 时永远判不到位，'
                '中间开度会每次都走到超时。' % (self.mid_stream_tolerance,))
        if self.mid_stream_timeout_sec <= 0.0:
            raise GripperConfigError(
                'mid_stream_timeout_sec=%r 必须为正' % (self.mid_stream_timeout_sec,))


class GripperResult:
    """一次夹爪操作的结果。字段与 SetGripper.Response 一一对应。"""

    def __init__(self, ok, error_code, detail='', dispatched_cmd=0.0,
                 dispatched_rad=0.0, actual_cmd=0.0, force_applied=False):
        self.ok = ok
        self.error_code = error_code
        self.detail = detail
        self.dispatched_cmd = float(dispatched_cmd)
        self.dispatched_rad = float(dispatched_rad)
        self.actual_cmd = float(actual_cmd)
        self.force_applied = bool(force_applied)

    def __repr__(self):
        return ('GripperResult(ok=%s, %s, cmd=%.2f, rad=%.4f, actual=%.2f, '
                'force_applied=%s)'
                % (self.ok, self.error_code, self.dispatched_cmd,
                   self.dispatched_rad, self.actual_cmd, self.force_applied))


class StatusEvent:
    def __init__(self, code, detail=''):
        self.code = code
        self.detail = detail

    def __repr__(self):
        return 'StatusEvent(%s, %r)' % (self.code, self.detail)


class GripperController:
    """夹爪开合的执行者。

    并发策略：**拒绝而不排队**。
    ``open/close_effector`` 是阻塞调用（实测 duration=1.0 阻塞满 1.0s），
    并发调用同一只夹爪会让两次动作互相覆盖 —— 排队会让调用方等一个不确定的时长
    且拿不到"你的请求被推迟了"这个事实，直接拒绝更诚实。
    """

    def __init__(self, cfg, session, sleep_fn=None, clock_fn=None,
                 in_simulation=True):
        self.cfg = cfg
        self.session = session
        # 注入 sleep/clock 便于测试：中间开度的重发循环有超时，
        # 用真时钟测就得真等 3 秒，而且没法确定性地造出"超时"这个分支。
        self._sleep = sleep_fn if sleep_fn is not None else _noop_sleep
        self._clock = clock_fn if clock_fn is not None else _default_clock
        # 后端是否是仿真。决定 force_applied 能否为 True ——
        # 仿真下 set_effector_max_force 是空操作，谎报 True 会让人
        # 把仿真里的夹持行为当成"力限已验收"。
        self.in_simulation = bool(in_simulation)
        self.events = []
        self._busy = set()
        self._lock = threading.Lock()

    def _emit(self, code, detail=''):
        self.events.append(StatusEvent(code, detail))

    def drain_events(self):
        out = self.events
        self.events = []
        return out

    # ---------------- 解析请求 ----------------

    def resolve_names(self, name):
        """把请求里的 name 解析成部件名列表。空串 = 全部。

        返回 (names, err_or_None)。
        """
        if not name:
            return (list(self.cfg.gripper_names), None)
        if name not in self.cfg.gripper_names:
            return (None,
                    '未知夹爪 %r，合法取值：%s（空字符串表示全部）'
                    % (name, self.cfg.gripper_names))
        return ([name], None)

    def resolve_cmd(self, opening_fraction, use_raw_cmd, raw_cmd):
        """把请求解析成裸命令值。返回 (cmd, err_or_None)。

        越界**显式报错**，不静默夹 —— 越界往往正是极性搞错的症状，
        静默夹会让"我要求 150"变成"实际 100"而无人知晓。
        """
        if use_raw_cmd:
            try:
                cmd = validate_grasp_cmd(raw_cmd, 'raw_cmd')
            except GripperConfigError as exc:
                return (None, str(exc))
            return (cmd, None)

        try:
            f = float(opening_fraction)
        except (TypeError, ValueError):
            return (None, 'opening_fraction=%r 不是数值' % (opening_fraction,))
        if f != f:
            return (None, 'opening_fraction 是 NaN')
        if not (0.0 <= f <= 1.0):
            return (None,
                    'opening_fraction=%r 越出 [0, 1]。'
                    '提醒：1.0 = 全张开、0.0 = 全闭合；'
                    '若想直接给厂商的 0-100 裸命令值请置 use_raw_cmd=true'
                    '（注意那个空间是 0=张开、100=闭合）。' % (f,))
        return (opening_fraction_to_cmd(f), None)

    # ---------------- 执行 ----------------

    def execute(self, name='', opening_fraction=1.0, duration=0.0,
                use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
                write_allowed=True):
        """执行一次夹爪操作。返回 GripperResult。**不抛异常。**"""
        if not self.cfg.enable_service:
            return GripperResult(
                False, EC_DISABLED_BY_CONFIG,
                '夹爪服务未启用（gripper.enable_service=false）。'
                '显式拒绝而不静默成功 —— 静默成功会让调用方以为夹爪动了。')

        if not write_allowed:
            return GripperResult(
                False, EC_WRITE_GATE_DENIED,
                '写通路准入未通过，拒绝操作夹爪。')

        names, err = self.resolve_names(name)
        if err:
            return GripperResult(False, EC_UNKNOWN_GRIPPER, err)

        cmd, err = self.resolve_cmd(opening_fraction, use_raw_cmd, raw_cmd)
        if err:
            return GripperResult(False, EC_VALUE_OUT_OF_RANGE, err)

        dur = float(duration) if float(duration) > 0.0 \
            else self.cfg.default_duration_sec

        # 并发保护：整组一起占用。部分占用会出现"左手在动、右手请求被放过"
        # 却又共享同一个阻塞 SDK 调用的情况。
        with self._lock:
            clash = [n for n in names if n in self._busy]
            if clash:
                return GripperResult(
                    False, EC_BUSY,
                    '夹爪 %s 正在执行中。open/close_effector 是阻塞调用，'
                    '并发会让两次动作互相覆盖，所以拒绝而不排队。' % (clash,))
            self._busy.update(names)

        try:
            return self._do(names, cmd, dur, max_force)
        finally:
            with self._lock:
                self._busy.difference_update(names)

    def _stream_to(self, names, cmd):
        """把中间开度**持续重发**到实测到位。返回 (是否到位, 最后读到的值)。

        !!! 单次下发不只是"到不了位"，而是会冲到完全闭合 !!!
        实测（MuJoCo 后端，从张开位单次下发 cmd=50）：

            t≈0.1s -> 91.5      t≈0.5s -> 99.998      t≈2.0s -> 99.998

        要求半开、结果**全夹紧**。真机上手指之间有东西时这就是压碎。
        以 stream_freq 持续重发则：25 -> 25.011、50 -> 50.007、75 -> 75.003
        （误差 < 0.011）。到位后停止重发漂移 **0.000** —— 所以只有"接近过程"
        需要重发，稳定后不需要。

        本函数存在的直接原因是我在这里写过一句**没有取证的推测**：
        "夹爪到位后由机械结构自持，不需要像手臂那样持续重发"。
        那句话是错的，而且它当时被写成了事实的语气。
        """
        period = 1.0 / self.cfg.stream_freq
        deadline = self._clock() + self.cfg.mid_stream_timeout_sec
        last = float('nan')
        while True:
            self.session.set_joints_position(
                list(names), [[cmd] for _ in names],
                control_way='direct', use_wbc=False,
                add_default_torso=False)
            try:
                got = self.session.get_current_joints_position(list(names))
                vals = [float(g[0]) if isinstance(g, (list, tuple)) else float(g)
                        for g in got]
                last = max(vals, key=lambda v: abs(v - cmd))
                if all(abs(v - cmd) <= self.cfg.mid_stream_tolerance
                       for v in vals):
                    return (True, last)
            except Exception:      # noqa: BLE001
                # 读不到就没法判到位。不在这里上报（外层会因超时上报），
                # 也不提前返回成功 —— 提前返回会把"不知道"当成"到位了"。
                pass
            if self._clock() >= deadline:
                return (False, last)
            self._sleep(period)

    def _do(self, names, cmd, dur, max_force):
        force_applied = False
        want_force = float(max_force) > 0.0 or self.cfg.default_max_force_n > 0.0
        force_val = (float(max_force) if float(max_force) > 0.0
                     else self.cfg.default_max_force_n)

        if want_force:
            try:
                self.session.set_effector_max_force(
                    list(names), [force_val] * len(names))
                # !!! 仿真下绝不报 True !!!
                # astribot_client.py:1139 在仿真下直接 return，力根本没设上。
                # 谎报 True 会让人把仿真里的夹持行为当成"力限已验收"。
                force_applied = not self.in_simulation
            except Exception as exc:      # noqa: BLE001
                self._emit(S_SDK_CALL_FAILED, '设夹持力失败：%s' % exc)
                return GripperResult(
                    False, EC_SDK_CALL_FAILED,
                    '设夹持力失败（未执行开合）：%s' % exc,
                    dispatched_cmd=cmd, dispatched_rad=cmd_to_rad(cmd))

        # 走 open/close_effector 而不是 set_joints_position：
        # 前者是厂商为夹爪提供的语义化接口，且会尊重 duration 做平滑到位。
        # 只有端点值才有对应的语义化接口，中间值必须走位置指令。
        try:
            if cmd <= CMD_OPEN:
                self.session.open_effector(list(names), duration=dur)
            elif cmd >= CMD_CLOSED:
                self.session.close_effector(list(names), duration=dur)
            else:
                # 中间开度没有语义化接口，只能用位置指令 —— 而位置指令**必须
                # 持续重发**，见下面 _stream_to 的说明。
                reached, last = self._stream_to(names, cmd)
                if not reached:
                    self._emit(S_MID_OPENING_TIMEOUT,
                               '中间开度未到位：目标 %.2f，%.2fs 后仍在 %.2f'
                               % (cmd, self.cfg.mid_stream_timeout_sec, last))
        except Exception as exc:      # noqa: BLE001
            self._emit(S_SDK_CALL_FAILED, '夹爪开合失败：%s' % exc)
            return GripperResult(
                False, EC_SDK_CALL_FAILED, '夹爪开合失败：%s' % exc,
                dispatched_cmd=cmd, dispatched_rad=cmd_to_rad(cmd),
                force_applied=force_applied)

        if self.cfg.settle_extra_sec > 0.0:
            self._sleep(self.cfg.settle_extra_sec)

        actual = 0.0
        try:
            got = self.session.get_current_joints_position(list(names))
            if got and isinstance(got[0], (list, tuple)):
                actual = float(got[0][0])
            elif got:
                actual = float(got[0])
        except Exception as exc:      # noqa: BLE001
            # 读回失败**不判失败**：动作已经下发出去了。但必须上报，
            # 否则 actual_cmd=0 会被读成"夹爪在全张开位置"。
            self._emit(S_SDK_CALL_FAILED, '读夹爪实际位置失败：%s' % exc)
            return GripperResult(
                True, EC_SUCCESS,
                '%s；但读回实际位置失败：%s（actual_cmd 不可信）'
                % (describe_cmd(cmd), exc),
                dispatched_cmd=cmd, dispatched_rad=cmd_to_rad(cmd),
                actual_cmd=float('nan'), force_applied=force_applied)

        detail = describe_cmd(cmd)
        if want_force and not force_applied:
            detail += '；力限未生效（仿真下 set_effector_max_force 是空操作）'
        return GripperResult(True, EC_SUCCESS, detail,
                             dispatched_cmd=cmd, dispatched_rad=cmd_to_rad(cmd),
                             actual_cmd=actual, force_applied=force_applied)


def _default_clock():
    import time as _t
    return _t.monotonic()


def _noop_sleep(_seconds):
    """默认不真的睡。节点层会注入真实 sleep。"""
    return None
