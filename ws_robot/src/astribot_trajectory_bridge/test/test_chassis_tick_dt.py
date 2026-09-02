#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""内环步长（dt）改用实测值的回归测试。

锁的是什么缺陷
============
底盘是**位置**接口：物理速度 = 每拍位置增量 × 墙钟拍率。原实现每拍加
``v / cfg.freq``（freq=250 是**配置值**），而实测拍率下界只有 ≥157Hz ——
底盘最多只跑到指令速度的 63%，且没有任何告警。

这里的断言分两类：
  · **速度守恒**：同一段墙钟时间内，无论拍率多少，积出的位移都应等于 v*T；
  · **钳位安全**：dt 直接乘在速度上，一次调度停顿不能变成位置阶跃。

本文件不 import rclpy、不 import msgs，任何环境都能跑。
"""

import pytest

from astribot_trajectory_bridge.chassis_integrator import (
    ChassisConfigError,
    integrate_step,
    integrate_step_dt,
    measure_tick_dt,
)


NOMINAL = 1.0 / 250.0
MAX_DT = 0.04


class TestVelocityIsPreservedRegardlessOfTickRate:
    """★ 缺陷本身：按标称频率积分时，拍率越低走得越少。"""

    @pytest.mark.parametrize('actual_rate', [250.0, 157.0, 91.0, 500.0])
    def test_measured_dt_gives_correct_displacement(self, actual_rate):
        """用实测 dt：任意拍率下，1 秒积出的位移都等于 v*1s。"""
        v = (0.5, 0.0, 0.0)
        dt = 1.0 / actual_rate
        pos = [0.0, 0.0, 0.0]
        ticks = int(round(actual_rate))          # 跑满 1 秒
        for _ in range(ticks):
            pos = integrate_step_dt(pos, v, dt)
        assert pos[0] == pytest.approx(0.5, abs=1e-9), (
            '拍率 %.0fHz 下 1 秒应走 0.5m，实际 %.6f' % (actual_rate, pos[0]))

    def test_nominal_freq_under_integrates_when_rate_is_low(self):
        """反证：按标称频率积分时，157Hz 只能走到 63% —— 这就是原来的行为。"""
        v = (0.5, 0.0, 0.0)
        actual_rate = 157.0
        pos = [0.0, 0.0, 0.0]
        for _ in range(int(round(actual_rate))):
            pos = integrate_step(pos, v, 250.0)   # 旧口径：用配置的 250
        ratio = pos[0] / 0.5
        assert ratio == pytest.approx(157.0 / 250.0, abs=0.01)
        assert ratio < 0.65, '这条测试自己写错了：旧口径就该明显偏小'

    def test_both_paths_agree_when_rate_equals_nominal(self):
        """拍率恰好等于标称时，新旧口径必须给出同一个结果（无回归）。"""
        v = (0.3, -0.2, 0.7)
        a = integrate_step([1.0, 2.0, 0.5], v, 250.0)
        b = integrate_step_dt([1.0, 2.0, 0.5], v, NOMINAL)
        assert a == pytest.approx(b)


class TestMeasureTickDtNormalCases:

    def test_first_tick_uses_nominal(self):
        """首拍没有参照，用标称值，且不算钳位（不是异常）。"""
        r = measure_tick_dt(now=100.0, prev=None,
                            nominal_dt=NOMINAL, max_dt=MAX_DT)
        assert r.dt == NOMINAL
        assert r.clamped is False
        assert r.raw is None

    @pytest.mark.parametrize('raw', [0.004, 0.006, 0.0149, 0.039])
    def test_normal_periods_pass_through_unclamped(self, raw):
        """实测超时周期最高 14.9ms，必须**原值通过** —— 钳了就等于没修。"""
        r = measure_tick_dt(now=10.0 + raw, prev=10.0,
                            nominal_dt=NOMINAL, max_dt=MAX_DT)
        assert r.dt == pytest.approx(raw)
        assert r.clamped is False


class TestMeasureTickDtClamping:
    """钳位是安全边界：dt 乘在速度上，停顿不能变成位置阶跃。"""

    def test_long_stall_is_clamped(self):
        r = measure_tick_dt(now=10.5, prev=10.0,
                            nominal_dt=NOMINAL, max_dt=MAX_DT)
        assert r.dt == MAX_DT
        assert r.clamped is True
        assert r.raw == pytest.approx(0.5)
        assert '钳位' in r.reason

    def test_clamped_step_displacement_stays_far_below_leash(self):
        """★ 钳位后单拍位移必须远小于 leash，否则一次停顿就撞开唯一的硬保护。"""
        max_vel_xy, leash_xy = 1.0, 0.25
        r = measure_tick_dt(now=100.0, prev=10.0,     # 90 秒的极端停顿
                            nominal_dt=NOMINAL, max_dt=MAX_DT)
        assert max_vel_xy * r.dt < leash_xy / 2.0, (
            '单拍最大位移 %.4fm 相对 leash %.2fm 余量不足'
            % (max_vel_xy * r.dt, leash_xy))

    @pytest.mark.parametrize('raw', [0.0, -0.001, -5.0])
    def test_non_advancing_clock_falls_back_to_nominal(self, raw):
        """rclpy 定时器积压时会背靠背触发，dt≈0；时钟倒退也要兜住。

        用 0 积分等于这一拍白丢（速度出现凹口），退回标称值保持连续。
        """
        r = measure_tick_dt(now=10.0 + raw, prev=10.0,
                            nominal_dt=NOMINAL, max_dt=MAX_DT)
        assert r.dt == NOMINAL
        assert r.clamped is True
        assert '时钟未前进' in r.reason

    def test_clamping_is_never_silent(self):
        """凡钳位必带原因文本 —— 静默钳位等于静默改变底盘速度。"""
        for now, prev in [(10.5, 10.0), (10.0, 10.0), (9.0, 10.0)]:
            r = measure_tick_dt(now, prev, NOMINAL, MAX_DT)
            assert r.clamped is True
            assert r.reason, '钳位了却没有原因文本'


class TestMeasureTickDtConfigGuards:
    """非法配置必须当场抛，不能带着跑。"""

    def test_max_dt_below_nominal_is_rejected(self):
        """max_dt < nominal 时每拍都被钳，积分恒等于钳位值 —— 等于没修。"""
        with pytest.raises(ChassisConfigError, match='小于'):
            measure_tick_dt(1.0, 0.0, nominal_dt=0.004, max_dt=0.001)

    @pytest.mark.parametrize('bad', [0.0, -1.0])
    def test_non_positive_nominal_rejected(self, bad):
        with pytest.raises(ChassisConfigError):
            measure_tick_dt(1.0, 0.0, nominal_dt=bad, max_dt=MAX_DT)

    @pytest.mark.parametrize('bad', [0.0, -1.0])
    def test_non_positive_max_rejected(self, bad):
        with pytest.raises(ChassisConfigError):
            measure_tick_dt(1.0, 0.0, nominal_dt=NOMINAL, max_dt=bad)


class TestIntegrateStepDtGuards:

    @pytest.mark.parametrize('bad', [0.0, -0.004])
    def test_non_positive_dt_rejected(self, bad):
        with pytest.raises(ChassisConfigError, match='dt'):
            integrate_step_dt([0.0, 0.0, 0.0], (1.0, 0.0, 0.0), bad)

    def test_wrong_dof_rejected(self):
        with pytest.raises(ChassisConfigError, match='长度'):
            integrate_step_dt([0.0, 0.0], (1.0, 0.0, 0.0), NOMINAL)

    def test_does_not_mutate_input(self):
        pos = [1.0, 2.0, 0.3]
        integrate_step_dt(pos, (1.0, 1.0, 1.0), NOMINAL)
        assert pos == [1.0, 2.0, 0.3]

    def test_theta_is_wrapped(self):
        import math
        out = integrate_step_dt([0.0, 0.0, math.pi - 0.001], (0.0, 0.0, 1.0), 0.01)
        assert -math.pi <= out[2] <= math.pi


class TestMixedRateSequence:
    """把实测到的周期分布直接喂进去，验证总位移只由墙钟决定。"""

    def test_displacement_matches_wall_clock_not_tick_count(self):
        # 实测 LOOP_OVERRUN 报出的周期（秒），混上一批正常拍
        periods = [0.0060, 0.0062, 0.0066, 0.0073, 0.0079,
                   0.0101, 0.0110, 0.0112, 0.0114, 0.0149]
        periods += [0.0030] * 40
        vx = 0.4
        pos = [0.0, 0.0, 0.0]
        t = 0.0
        prev = None
        for p in periods:
            t += p
            r = measure_tick_dt(t, prev, NOMINAL, MAX_DT)
            prev = t
            pos = integrate_step_dt(pos, (vx, 0.0, 0.0), r.dt)
        wall = sum(periods)
        # 首拍用标称值代替了它的真实周期，差额就是那一拍的偏差
        expected = vx * (wall - periods[0] + NOMINAL)
        assert pos[0] == pytest.approx(expected, abs=1e-9)
        # 与"按标称频率积分"相比，新口径明显更接近墙钟真值
        old = vx * len(periods) * NOMINAL
        assert abs(pos[0] - vx * wall) < abs(old - vx * wall)


# ═══════════════════════════════════════════════════════════════════════════
# 下面这组直接驱动**真实的 inner_tick**，不是测纯函数。
#
# 为什么必须有这一组：上面 TestVelocityIsPreservedRegardlessOfTickRate 全绿时，
# 把 inner_tick 里的 dt 换回 `1.0 / cfg.freq`（也就是原缺陷本身），
# 481 条测试**一条都不响** —— 纯函数测试证明不了"内环真的用了实测 dt"。
# 这个空洞是变异测试测出来的，不是想出来的。
# ═══════════════════════════════════════════════════════════════════════════

from astribot_trajectory_bridge.chassis_bridge_core import (      # noqa: E402
    ChassisBridgeConfig,
    ChassisBridgeCore,
)
from astribot_trajectory_bridge.ports import (                    # noqa: E402
    FakeClock,
    FakePose,
    FakeSession,
)

PART = 'astribot_chassis'


def _build(**overrides):
    clock = FakeClock(100.0)
    session = FakeSession(desired={PART: [0.0, 0.0, 0.0]},
                          current={PART: [0.0, 0.0, 0.0]},
                          follow_ratio=1.0)
    pose = FakePose(clock, pose=[0.0, 0.0, 0.0])
    cfg = ChassisBridgeConfig(enable_slam_correction=False, **overrides)
    core = ChassisBridgeCore(cfg, session, pose, clock)
    return core, clock


def _run(core, clock, vx, period, ticks):
    """以固定周期 period 跑 ticks 拍，返回 x 方向总位移。

    每拍都调 ``submit_scan_seen()``：/scan 时效性联锁默认开启，不喂就会在
    0.5s 后把速度置零，本文件测的 dt 结论就全被它压成 0 了。
    联锁自身的测试在 test_chassis_scan_interlock.py。

    !!! 这里有一条值得记住的观测 !!!
    加联锁后，本文件的 test_halved_tick_rate_still_travels_the_same_distance
    **依然通过** —— 因为它比较 A、B 两个位移是否相等，而两者都被压成了 0.0，
    0 == 0 成立。真正抓到问题的是 test_displacement_tracks_wall_clock_at_any_rate
    （它比的是绝对值）。相等性断言在"两边同时失效"时会假通过。
    """
    core.enable()
    x0 = core.pos_cmd[0]
    for _ in range(ticks):
        core.submit_twist(vx, 0.0, 0.0)
        core.submit_scan_seen()
        clock.advance(period)
        core.inner_tick()
    return core.pos_cmd[0] - x0


class TestInnerTickUsesMeasuredDt:
    """★ 直接盯住内环：拍率减半，同样墙钟时长内的位移必须不变。"""

    def test_halved_tick_rate_still_travels_the_same_distance(self):
        vx = 0.2                       # 远低于限幅，避免被 slew 影响结论
        # A：250Hz 跑 1 秒        B：125Hz 跑 1 秒（拍数减半、周期加倍）
        core_a, clock_a = _build()
        dist_a = _run(core_a, clock_a, vx, 1.0 / 250.0, 250)
        core_b, clock_b = _build()
        dist_b = _run(core_b, clock_b, vx, 1.0 / 125.0, 125)
        assert dist_b == pytest.approx(dist_a, rel=0.02), (
            '拍率减半后位移变了（%.6f vs %.6f）——说明内环没用实测 dt。'
            '底盘是位置接口，位移只该由墙钟决定。' % (dist_b, dist_a))

    @pytest.mark.parametrize('rate', [250.0, 157.0, 125.0])
    def test_displacement_tracks_wall_clock_at_any_rate(self, rate):
        """任意拍率下，1 秒都应走 vx*1s（留 5% 给 slew 起步的那几拍）。"""
        vx = 0.2
        core, clock = _build()
        dist = _run(core, clock, vx, 1.0 / rate, int(round(rate)))
        assert dist == pytest.approx(vx * 1.0, rel=0.05), (
            '拍率 %.0fHz 下 1 秒走了 %.4fm，应约 %.4fm' % (rate, dist, vx))

    def test_stats_report_the_actual_rate_not_the_configured_one(self):
        """无偏拍率统计必须报实测值 —— 这是原来完全没有的观测量。"""
        core, clock = _build()
        _run(core, clock, 0.2, 1.0 / 125.0, 125)
        st = core.tick_stats()
        assert st.count == 125
        assert st.rate_hz == pytest.approx(125.0, rel=0.02), (
            '统计报的是 %.1fHz，配置是 250 —— 报了配置值就等于没测' % st.rate_hz)
        assert st.clamp_count == 0

    def test_long_stall_inside_inner_tick_is_clamped_and_reported(self):
        """内环里遇到停顿：位移被钳住，且必须发出 TICK_DT_CLAMPED 事件。"""
        core, clock = _build()
        core.enable()
        core.submit_twist(1.0, 0.0, 0.0)
        clock.advance(1.0 / 250.0)
        core.inner_tick()
        core.drain_events()
        x_before = core.pos_cmd[0]

        core.submit_twist(1.0, 0.0, 0.0)
        clock.advance(0.5)                       # 500ms 停顿
        core.inner_tick()
        step = core.pos_cmd[0] - x_before

        assert step <= core.cfg.max_tick_dt_sec * core.cfg.max_vel_xy + 1e-9, (
            '单拍走了 %.4fm，超过钳位上界' % step)
        assert step < core.cfg.leash_xy_m / 2.0, (
            '单拍位移 %.4fm 相对 leash %.2fm 余量不足' % (step, core.cfg.leash_xy_m))
        assert 'TICK_DT_CLAMPED' in [e.code for e in core.drain_events()], (
            '钳位了却没上报 —— 静默钳位等于静默改变底盘速度')

    def test_reenable_after_pause_does_not_integrate_the_idle_time(self):
        """★ disable 一段时间再 enable，首拍不能把停机时长当 dt。

        判据用"不该上报 TICK_DT_CLAMPED"，**不用位移大小** ——
        位移这条判不出来：重新使能时 `_prev_vel_out` 归零，加速度限幅
        (2.5 * 0.04 = 0.1m/s) 把速度压得很小，位移只有 0.004m，
        把钳位完全盖住了。这个空洞是变异测试（enable 时不清 _prev_tick_time
        仍全绿）暴露出来的。

        顺带确认了一件好事：slew 限幅是停顿阶跃的**第二道防线**。
        """
        core, clock = _build()
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        clock.advance(1.0 / 250.0)
        core.inner_tick()
        core.disable()

        clock.advance(60.0)                      # 停机一分钟
        core.enable()
        core.drain_events()                      # 清掉 enable/disable 自身的事件
        core.submit_twist(0.5, 0.0, 0.0)
        clock.advance(1.0 / 250.0)
        core.inner_tick()

        assert 'TICK_DT_CLAMPED' not in [e.code for e in core.drain_events()], (
            '重新使能后首拍报了钳位 —— 说明 60 秒停机时长被当成 dt 拿去测量了。'
            '重新使能不是调度异常，不该走钳位分支。')


class TestConfigRejectsUnsafeClampBound:
    """配置层的硬校验：单拍位移必须远小于 leash。"""

    def test_max_tick_dt_that_can_trip_leash_is_rejected(self):
        with pytest.raises(ChassisConfigError, match='leash'):
            ChassisBridgeConfig(max_vel_xy=1.0, leash_xy_m=0.25,
                                max_tick_dt_sec=0.3)

    def test_max_tick_dt_below_nominal_is_rejected(self):
        with pytest.raises(ChassisConfigError, match='标称步长'):
            ChassisBridgeConfig(freq=250.0, max_tick_dt_sec=0.001)

    def test_default_config_is_accepted(self):
        cfg = ChassisBridgeConfig()
        assert cfg.max_vel_xy * cfg.max_tick_dt_sec < cfg.leash_xy_m / 2.0


# ═══════════════════════════════════════════════════════════════════════════
# 拍率统计窗口的两个缺陷（2026-09-01 实机暴露，离线复现后修）
#
# Bug 1  enable() 只清 _prev_tick_time，留着 _tick_first_time
#        -> rate = count / (last - first) 的分母含 disabled 时长
#        -> 实测 65 拍被报成 165.4Hz（真值 ~238Hz），停得越久报得越低
#
# Bug 2  停用后 count/rate 冻在上一段的值上，而日志照打
#        -> 与 map_odom_tf 那个"陈旧 TF 被当成最新"同类
#        -> 我因此误判两次："内环停了" / "桥接仍是 enabled"
# ═══════════════════════════════════════════════════════════════════════════


class TestTickStatsWindow:

    def _core(self):
        clock = FakeClock(100.0)
        session = FakeSession(desired={PART: [0.0, 0.0, 0.0]},
                              current={PART: [0.0, 0.0, 0.0]},
                              follow_ratio=1.0)
        cfg = ChassisBridgeConfig(enable_slam_correction=False)
        return ChassisBridgeCore(cfg, session, FakePose(clock, pose=[0.0]*3),
                                 clock), clock

    def _tick(self, core, clock, n, period=1.0 / 250.0):
        for _ in range(n):
            clock.advance(period)
            core.inner_tick()

    def test_rate_is_not_diluted_by_disabled_time(self):
        """★ Bug 1：停机时长绝不能进速率的分母。"""
        core, clock = self._core()
        core.enable()
        core.drain_events()
        self._tick(core, clock, 65)
        core.disable()
        core.drain_events()
        clock.advance(300.0)                     # 停 5 分钟
        core.enable()
        core.drain_events()
        self._tick(core, clock, 200)
        st = core.tick_stats()
        assert st.rate_hz > 200.0, (
            '停 5 分钟后再使能，速率报 %.1fHz —— 分母把停机时长算进去了' % st.rate_hz)

    def test_enable_reopens_the_window(self):
        """每次 enable 重开窗口：count 是**本段**的，不累加上一段。"""
        core, clock = self._core()
        core.enable()
        core.drain_events()
        self._tick(core, clock, 65)
        core.disable()
        core.drain_events()
        core.enable()
        core.drain_events()
        self._tick(core, clock, 30)
        assert core.tick_stats().count == 30, (
            'count=%d，应为本段的 30' % core.tick_stats().count)

    def test_clamp_count_also_reopens(self):
        """钳位计数同样属于窗口，不能跨段累加。"""
        core, clock = self._core()
        core.enable()
        core.drain_events()
        clock.advance(1.0 / 250.0)
        core.inner_tick()
        clock.advance(5.0)                               # 制造一次钳位
        core.inner_tick()
        assert core.tick_stats().clamp_count >= 1
        core.disable()
        core.drain_events()
        core.enable()
        core.drain_events()
        self._tick(core, clock, 10)
        assert core.tick_stats().clamp_count == 0, '钳位计数跨段累加了'

    def test_live_is_true_while_enabled(self):
        core, clock = self._core()
        core.enable()
        core.drain_events()
        self._tick(core, clock, 20)
        assert core.tick_stats().live is True

    def test_live_is_false_after_disable(self):
        """★ Bug 2：停用后读数必须自带"这是陈旧值"。"""
        core, clock = self._core()
        core.enable()
        core.drain_events()
        self._tick(core, clock, 20)
        core.disable()
        core.drain_events()
        st = core.tick_stats()
        assert st.live is False, (
            '停用后 live 仍为 True —— 调用方会把冻结的 %.1fHz 当成当前拍率'
            % st.rate_hz)
        assert st.count == 20, '停用不该清掉历史值，只该标记为非 live'

    def test_live_is_false_before_first_enable(self):
        core, _ = self._core()
        st = core.tick_stats()
        assert st.live is False and st.count == 0

    def test_count_does_not_grow_while_disabled(self):
        """inner_tick 在 ST_DISABLED 提前返回 —— 这是对的，锁住它。"""
        core, clock = self._core()
        core.enable()
        core.drain_events()
        self._tick(core, clock, 20)
        core.disable()
        core.drain_events()
        self._tick(core, clock, 100)
        assert core.tick_stats().count == 20
