#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪服务核心的分支测试。**离线，无 SDK、无 rclpy、无仿真。**

覆盖重点是**拒绝路径**
==================
夹爪服务的分支几乎全是拒绝：越界、未知夹爪、并发、SDK 抛异常、配置未启用。
这些在真机上很难按需复现（怎么可靠地让 SDK 抛异常？），所以必须离线穷尽。

另外两条是"防止错误结论通过"的断言：
  · 仿真下 force_applied 恒为 false —— 否则会把仿真里的夹持行为
    误当成"力限已验收"；
  · 极性：opening_fraction=1.0 必须映射到 cmd=0。
"""

import pytest

from astribot_trajectory_bridge.gripper_core import (
    EC_BUSY,
    EC_DISABLED_BY_CONFIG,
    EC_SDK_CALL_FAILED,
    EC_SUCCESS,
    EC_UNKNOWN_GRIPPER,
    EC_VALUE_OUT_OF_RANGE,
    EC_WRITE_GATE_DENIED,
    GripperConfig,
    GripperController,
)
from astribot_trajectory_bridge.gripper_math import GripperConfigError
from astribot_trajectory_bridge.ports import FakeSession

GL = 'astribot_gripper_left'
GR = 'astribot_gripper_right'


class TickClock:
    """可控时钟。每次读取推进一小步，让"超时"这个分支确定且瞬间可达。

    !!! 必须注入，不能用真时钟 !!!
    中间开度的重发循环带 3s 超时，且测试里 sleep 是空操作 ——
    用真时钟意味着每个走到超时的用例都要**真忙等 3 秒**。
    最初没注入时整组测试从 0.11s 涨到 3.11s，就是这个原因。
    """

    def __init__(self, step=0.01):
        self.t = 0.0
        self.step = float(step)

    def __call__(self):
        self.t += self.step
        return self.t


def build(follow_ratio=1.0, in_simulation=True, clock=None, **cfgkw):
    session = FakeSession(
        desired={GL: [0.0], GR: [0.0]}, current={GL: [0.0], GR: [0.0]},
        limits={GL: ([0.0], [100.0]), GR: ([0.0], [100.0])},
        dofs={GL: 1, GR: 1}, follow_ratio=follow_ratio,
        in_simulation=in_simulation)
    kw = dict(gripper_names=[GL, GR])
    kw.update(cfgkw)
    cfg = GripperConfig(**kw)
    return (GripperController(cfg, session, clock_fn=clock or TickClock(),
                              in_simulation=in_simulation),
            session)


def codes(ctrl):
    return [e.code for e in ctrl.drain_events()]


class TestConfigValidation:

    def test_empty_names_rejected(self):
        """没有白名单就无法区分"未知夹爪"与"SDK 故障"。"""
        with pytest.raises(GripperConfigError):
            GripperConfig(gripper_names=[])

    def test_nonpositive_duration_rejected(self):
        with pytest.raises(GripperConfigError):
            GripperConfig(gripper_names=[GL], default_duration_sec=0.0)

    def test_negative_settle_rejected(self):
        with pytest.raises(GripperConfigError):
            GripperConfig(gripper_names=[GL], settle_extra_sec=-1.0)

    def test_defaults_accepted(self):
        GripperConfig(gripper_names=[GL])


class TestPolarity:
    """!!! 极性 !!! 服务对外是 opening_fraction（1=全张开），内部翻转成 cmd。"""

    def test_full_open_dispatches_zero(self):
        ctrl, session = build()
        r = ctrl.execute(name=GL, opening_fraction=1.0)
        assert r.ok and r.dispatched_cmd == pytest.approx(0.0)
        assert session.effector_calls[-1][0] == 'open'

    def test_full_close_dispatches_hundred(self):
        ctrl, session = build()
        r = ctrl.execute(name=GL, opening_fraction=0.0)
        assert r.ok and r.dispatched_cmd == pytest.approx(100.0)
        assert session.effector_calls[-1][0] == 'close'

    def test_full_open_is_not_hundred(self):
        """反向锁：谁把"全张开"实现成 cmd=100 就会失败。"""
        ctrl, _ = build()
        r = ctrl.execute(name=GL, opening_fraction=1.0)
        assert r.dispatched_cmd != pytest.approx(100.0)

    def test_rad_is_closing_angle(self):
        """弧度是闭合角：全闭合时 0.93（关节上限），全张开时 0。"""
        ctrl, _ = build()
        assert ctrl.execute(name=GL, opening_fraction=0.0).dispatched_rad == \
            pytest.approx(0.93)
        assert ctrl.execute(name=GL, opening_fraction=1.0).dispatched_rad == \
            pytest.approx(0.0)

    def test_midpoint_uses_position_command(self):
        """中间开度没有语义化接口，必须走位置指令。"""
        ctrl, session = build()
        n0 = len(session.effector_calls)
        r = ctrl.execute(name=GL, opening_fraction=0.5)
        assert r.ok and r.dispatched_cmd == pytest.approx(50.0)
        assert len(session.effector_calls) == n0, '中间值不该走 open/close'
        assert session.set_position_calls[-1][1] == [[50.0]]


class TestMidOpeningStreaming:
    """!!! 中间开度必须持续重发 !!!

    实测（MuJoCo，从张开位**单次**下发 cmd=50）：
        t≈0.1s -> 91.5    t≈0.5s -> 99.998    t≈2.0s -> 99.998
    要求半开、结果**全夹紧** —— 真机上手指间有东西就是压碎。
    持续重发则 25 -> 25.011、50 -> 50.007、75 -> 75.003（误差 < 0.011），
    且到位后停发漂移 0.000。

    这个缺陷之所以存在，是因为我在实现里写过一句**没有取证的推测**
    （"夹爪到位后自持、不需持续重发"）并且用了事实的语气。
    """

    def test_streams_more_than_once_before_reaching(self):
        """跟随比例 <1 时需要多拍才到位 —— 必须真的发了多次。"""
        ctrl, session = build(follow_ratio=0.35)
        n0 = len(session.set_position_calls)
        r = ctrl.execute(name=GL, opening_fraction=0.5)
        assert r.ok
        assert len(session.set_position_calls) - n0 > 1, (
            '只发了一次 —— 单次下发实测会冲到完全闭合')

    def test_stops_when_reached(self):
        """到位就停，不无限重发。"""
        ctrl, session = build(follow_ratio=1.0)
        ctrl.execute(name=GL, opening_fraction=0.5)
        n = len(session.set_position_calls)
        assert n <= 3, '到位后仍在重发（%d 次）' % n

    def test_timeout_reports_and_still_returns_ok(self):
        """超时未到位：**上报**，但动作确实下发过，所以不判整体失败。

        判失败会让上层以为"什么都没发生"，而实际夹爪已经在动了。
        """
        ctrl, session = build(follow_ratio=0.0,      # 永远不动
                             mid_stream_timeout_sec=0.05)
        r = ctrl.execute(name=GL, opening_fraction=0.5)
        assert r.ok
        assert 'SETTLE_TIMEOUT' in codes(ctrl)

    def test_timeout_detail_carries_numbers(self):
        ctrl, session = build(follow_ratio=0.0, mid_stream_timeout_sec=0.05)
        ctrl.execute(name=GL, opening_fraction=0.5)
        ev = [e for e in ctrl.drain_events() if e.code == 'SETTLE_TIMEOUT']
        assert ev and '目标 50.00' in ev[0].detail

    def test_readback_failure_does_not_fake_success(self):
        """读不到实际位置时不许把"不知道"当成"到位了"。"""
        ctrl, session = build(mid_stream_timeout_sec=0.05)
        session.fail_after('get_current_joints_position', 0)
        ctrl.execute(name=GL, opening_fraction=0.5)
        assert 'SETTLE_TIMEOUT' in codes(ctrl), (
            '读回失败却判成到位了 —— 那是把"不知道"当成"成功"')

    def test_endpoints_do_not_stream(self):
        """端点走语义化接口，不该进重发循环。"""
        ctrl, session = build(follow_ratio=0.0)
        n0 = len(session.set_position_calls)
        ctrl.execute(name=GL, opening_fraction=0.0)
        ctrl.execute(name=GL, opening_fraction=1.0)
        assert len(session.set_position_calls) == n0

    def test_tolerance_zero_rejected(self):
        """容差 0 会让每次中间开度都走到超时。"""
        with pytest.raises(GripperConfigError):
            GripperConfig(gripper_names=[GL], mid_stream_tolerance=0.0)


class TestRangeChecking:
    """越界**显式报错**，不静默夹。"""

    @pytest.mark.parametrize('frac', [-0.01, 1.01, 5.0, -3.0])
    def test_fraction_out_of_range(self, frac):
        ctrl, _ = build()
        r = ctrl.execute(name=GL, opening_fraction=frac)
        assert not r.ok and r.error_code == EC_VALUE_OUT_OF_RANGE

    def test_fraction_nan(self):
        ctrl, _ = build()
        r = ctrl.execute(name=GL, opening_fraction=float('nan'))
        assert not r.ok and r.error_code == EC_VALUE_OUT_OF_RANGE

    def test_error_message_mentions_polarity(self):
        """越界往往正是极性搞错的症状，报错要提醒。"""
        ctrl, _ = build()
        r = ctrl.execute(name=GL, opening_fraction=100.0)
        assert '全张开' in r.detail and 'use_raw_cmd' in r.detail

    @pytest.mark.parametrize('raw', [-1.0, 100.1, 150.0])
    def test_raw_cmd_out_of_range(self, raw):
        ctrl, _ = build()
        r = ctrl.execute(name=GL, use_raw_cmd=True, raw_cmd=raw)
        assert not r.ok and r.error_code == EC_VALUE_OUT_OF_RANGE

    def test_raw_cmd_accepted_in_range(self):
        ctrl, _ = build()
        r = ctrl.execute(name=GL, use_raw_cmd=True, raw_cmd=100.0)
        assert r.ok and r.dispatched_cmd == pytest.approx(100.0)

    def test_raw_cmd_ignored_without_flag(self):
        """raw_cmd 不置 use_raw_cmd 时必须被忽略，走 opening_fraction。"""
        ctrl, _ = build()
        r = ctrl.execute(name=GL, opening_fraction=1.0,
                         use_raw_cmd=False, raw_cmd=100.0)
        assert r.ok and r.dispatched_cmd == pytest.approx(0.0)


class TestNameResolution:

    def test_empty_name_means_all(self):
        ctrl, session = build()
        r = ctrl.execute(name='', opening_fraction=0.0)
        assert r.ok
        assert session.effector_calls[-1][1] == [GL, GR]

    def test_unknown_gripper_rejected(self):
        ctrl, _ = build()
        r = ctrl.execute(name='astribot_gripper_middle', opening_fraction=1.0)
        assert not r.ok and r.error_code == EC_UNKNOWN_GRIPPER
        assert 'astribot_gripper_middle' in r.detail

    def test_error_lists_valid_names(self):
        ctrl, _ = build()
        r = ctrl.execute(name='bogus', opening_fraction=1.0)
        assert GL in r.detail and GR in r.detail

    def test_arm_name_is_not_a_gripper(self):
        """白名单必须挡住手臂名 —— 拿手臂当夹爪会下发 1 个值给 7 自由度部件。"""
        ctrl, _ = build()
        r = ctrl.execute(name='astribot_arm_left', opening_fraction=1.0)
        assert not r.ok and r.error_code == EC_UNKNOWN_GRIPPER


class TestForceApplied:
    """!!! 仿真下 force_applied 必须恒为 false !!!

    否则会让"仿真里夹持力已验收"这个错误结论通过 —— 而真机上力限是安全相关的。
    """

    def test_sim_never_reports_force_applied(self):
        ctrl, _ = build(in_simulation=True)
        r = ctrl.execute(name=GL, opening_fraction=0.0, max_force=40.0)
        assert r.ok and r.force_applied is False

    def test_sim_detail_says_force_not_effective(self):
        ctrl, _ = build(in_simulation=True)
        r = ctrl.execute(name=GL, opening_fraction=0.0, max_force=40.0)
        assert '力限未生效' in r.detail

    def test_real_reports_force_applied(self):
        ctrl, _ = build(in_simulation=False)
        r = ctrl.execute(name=GL, opening_fraction=0.0, max_force=40.0)
        assert r.ok and r.force_applied is True

    def test_no_force_requested_means_not_applied(self):
        ctrl, session = build(in_simulation=False)
        r = ctrl.execute(name=GL, opening_fraction=0.0, max_force=0.0)
        assert r.ok and r.force_applied is False
        assert session.effector_force_calls == []

    def test_default_force_from_config(self):
        ctrl, session = build(in_simulation=False, default_max_force_n=35.0)
        r = ctrl.execute(name=GL, opening_fraction=0.0)
        assert r.force_applied is True
        assert session.effector_force_calls[-1][1] == [35.0]

    def test_request_force_overrides_default(self):
        ctrl, session = build(in_simulation=False, default_max_force_n=35.0)
        ctrl.execute(name=GL, opening_fraction=0.0, max_force=55.0)
        assert session.effector_force_calls[-1][1] == [55.0]

    def test_force_failure_aborts_before_moving(self):
        """设力失败必须**在开合之前**中止 —— 否则会用错误的力去夹。"""
        ctrl, session = build()
        session.fail_after('set_effector_max_force', 0)
        n0 = len(session.effector_calls)
        r = ctrl.execute(name=GL, opening_fraction=0.0, max_force=40.0)
        assert not r.ok and r.error_code == EC_SDK_CALL_FAILED
        assert len(session.effector_calls) == n0, '设力失败后仍然执行了开合'
        assert '未执行开合' in r.detail


class TestGating:

    def test_disabled_by_config(self):
        ctrl, _ = build(enable_service=False)
        r = ctrl.execute(name=GL, opening_fraction=0.0)
        assert not r.ok and r.error_code == EC_DISABLED_BY_CONFIG

    def test_disabled_does_not_dispatch(self):
        ctrl, session = build(enable_service=False)
        ctrl.execute(name=GL, opening_fraction=0.0)
        assert session.effector_calls == []

    def test_write_gate_denied(self):
        ctrl, session = build()
        r = ctrl.execute(name=GL, opening_fraction=0.0, write_allowed=False)
        assert not r.ok and r.error_code == EC_WRITE_GATE_DENIED
        assert session.effector_calls == []


class TestSdkFailures:

    def test_open_failure_reported(self):
        ctrl, session = build()
        session.fail_after('open_effector', 0)
        r = ctrl.execute(name=GL, opening_fraction=1.0)
        assert not r.ok and r.error_code == EC_SDK_CALL_FAILED
        assert S_CODE in codes(ctrl)

    def test_close_failure_reported(self):
        ctrl, session = build()
        session.fail_after('close_effector', 0)
        r = ctrl.execute(name=GL, opening_fraction=0.0)
        assert not r.ok and r.error_code == EC_SDK_CALL_FAILED

    def test_midpoint_failure_reported(self):
        ctrl, session = build()
        session.fail_after('set_joints_position', 0)
        r = ctrl.execute(name=GL, opening_fraction=0.5)
        assert not r.ok and r.error_code == EC_SDK_CALL_FAILED

    def test_readback_failure_still_succeeds_but_reports(self):
        """读回失败不判失败（动作已下发），但必须上报且 actual_cmd 标为不可信。

        否则 actual_cmd=0 会被读成"夹爪在全张开位置"。
        """
        ctrl, session = build()
        session.fail_after('get_current_joints_position', 0)
        r = ctrl.execute(name=GL, opening_fraction=0.0)
        assert r.ok and r.error_code == EC_SUCCESS
        assert r.actual_cmd != r.actual_cmd      # NaN
        assert '不可信' in r.detail
        assert S_CODE in codes(ctrl)

    def test_no_exception_escapes(self):
        """execute() 承诺不抛异常 —— 逐个方法注入故障验证。"""
        for m in ('open_effector', 'close_effector', 'set_joints_position',
                  'get_current_joints_position', 'set_effector_max_force'):
            ctrl, session = build()
            session.fail_after(m, 0)
            for frac in (0.0, 0.5, 1.0):
                ctrl.execute(name=GL, opening_fraction=frac, max_force=40.0)


class TestConcurrency:
    """并发**拒绝而不排队**：阻塞调用并发会让两次动作互相覆盖。"""

    def test_busy_rejected(self):
        ctrl, _ = build()
        ctrl._busy.add(GL)      # noqa: SLF001 —— 直接造出"正在执行"
        r = ctrl.execute(name=GL, opening_fraction=0.0)
        assert not r.ok and r.error_code == EC_BUSY

    def test_busy_detail_names_the_gripper(self):
        ctrl, _ = build()
        ctrl._busy.add(GL)      # noqa: SLF001
        r = ctrl.execute(name=GL, opening_fraction=0.0)
        assert GL in r.detail

    def test_all_request_blocked_by_one_busy(self):
        """整组请求与单只占用冲突时也要拒 —— 它们共享同一个阻塞调用。"""
        ctrl, _ = build()
        ctrl._busy.add(GR)      # noqa: SLF001
        r = ctrl.execute(name='', opening_fraction=0.0)
        assert not r.ok and r.error_code == EC_BUSY

    def test_other_gripper_not_blocked(self):
        ctrl, _ = build()
        ctrl._busy.add(GR)      # noqa: SLF001
        r = ctrl.execute(name=GL, opening_fraction=0.0)
        assert r.ok

    def test_busy_released_after_success(self):
        ctrl, _ = build()
        assert ctrl.execute(name=GL, opening_fraction=0.0).ok
        assert ctrl.execute(name=GL, opening_fraction=1.0).ok

    def test_busy_released_after_failure(self):
        """失败路径也必须释放占用，否则一次失败会永久锁死夹爪。"""
        ctrl, session = build()
        session.fail_after('close_effector', 0)
        assert not ctrl.execute(name=GL, opening_fraction=0.0).ok
        session.fail_on.clear()
        assert ctrl.execute(name=GL, opening_fraction=0.0).ok


class TestDuration:

    def test_default_used_when_nonpositive(self):
        ctrl, session = build(default_duration_sec=1.5)
        ctrl.execute(name=GL, opening_fraction=0.0, duration=0.0)
        assert session.effector_calls[-1][2] == pytest.approx(1.5)

    def test_request_duration_honored(self):
        ctrl, session = build(default_duration_sec=1.5)
        ctrl.execute(name=GL, opening_fraction=0.0, duration=3.0)
        assert session.effector_calls[-1][2] == pytest.approx(3.0)


class TestDetailText:
    """日志里必须带文字说明：看到裸的 100 极容易误读成"全开"。"""

    def test_closed_detail(self):
        ctrl, _ = build()
        assert '闭合' in ctrl.execute(name=GL, opening_fraction=0.0).detail

    def test_open_detail(self):
        ctrl, _ = build()
        assert '张开' in ctrl.execute(name=GL, opening_fraction=1.0).detail

    def test_detail_includes_rad(self):
        ctrl, _ = build()
        assert '0.9300' in ctrl.execute(name=GL, opening_fraction=0.0).detail


S_CODE = 'SDK_CALL_FAILED'
