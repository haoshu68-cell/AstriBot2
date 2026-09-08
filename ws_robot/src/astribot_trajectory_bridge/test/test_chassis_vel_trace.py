#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""桥接内部速度链路取样（VelTrace / consume_vel_trace）的测试。

锁的是什么缺陷
============
整条速度链路上，桥接**内部**这一段原本一个数都不落盘。
path_tracking_diagnostics_node 看到的最后一段是 /cmd_vel，而进了桥接之后还有
五段变换，其中两段在任何速度话题上都不可见：

  · 看门狗 / scan 联锁把速度**置零** —— 在 /cmd_vel 上和"上游根本没发"
    长得一模一样，而处置完全不同（查感知 vs 查规划器）；
  · 外环 SLAM 校正**直接加在位置上** —— 它贡献的位移不经过任何速度量。

于是"底盘为什么没按指令走"在这一层是全黑的。上一轮实机排查就是卡在这里。

这里的断言分三类
==============
  · **口径**：cmd_path 是路径长、cmd_net/act_net 是净位移。拿路径长和净位移
    相比是错的，测试必须把这个区别钉死（旋转就是它们分道扬镳的地方）。
  · **窗口**：consume 必须取走即清零，且 enable() 必须重开 —— 陈旧读数被当成
    当前值是本项目反复踩到的那一类错误。
  · **不说谎**：一拍都没跑时不能报出一行全 0（会被读成"链路上确实全是零"）；
    每一段"咬没咬住"必须逐轴判（clamp 改模长、slew 可能只改 wz）。

本文件不 import rclpy、不 import msgs，任何环境都能跑。
"""

import math

import pytest

from astribot_trajectory_bridge.chassis_bridge_core import (
    ChassisBridgeConfig,
    ChassisBridgeCore,
)
from astribot_trajectory_bridge.ports import FakeClock, FakePose, FakeSession

PART = 'astribot_chassis'
DT = 1.0 / 250.0


def build(follow_ratio=1.0, **overrides):
    clock = FakeClock(100.0)
    session = FakeSession(desired={PART: [0.0, 0.0, 0.0]},
                          current={PART: [0.0, 0.0, 0.0]},
                          follow_ratio=follow_ratio)
    pose = FakePose(clock, pose=[0.0, 0.0, 0.0])
    core = ChassisBridgeCore(ChassisBridgeConfig(**overrides),
                             session, pose, clock)
    core.submit_scan_seen()
    return (core, session, clock)


def run(core, clock, seconds, twist=None, dt=DT, feed_scan=True):
    """跑一段时间。

    !!! twist 每拍重发 !!! 不重发的话 cmd_vel 看门狗（0.3s）会先把速度置零，
    测出来的东西就与本文件要测的对象无关了 —— 这个坑在
    test_chassis_scan_interlock.py 里已经踩过一次。
    """
    n = int(round(seconds / dt))
    for _ in range(n):
        clock.advance(dt)
        if feed_scan:
            core.submit_scan_seen()
        if twist is not None:
            core.submit_twist(*twist)
        core.inner_tick()


class TestPathVersusNetDisplacement:
    """★ 口径：路径长和净位移不是一回事，混用就会造出假的跟踪误差。"""

    def test_straight_line_path_equals_net(self):
        # 直线走：两个口径应当**逐位**一致，直度=1。这是唯一允许把 act_net 和
        # cmd_net 直接相比的情形。
        core, _, clock = build()
        assert core.enable()[0]
        run(core, clock, 1.0, twist=(0.2, 0.0, 0.0))
        tr = core.consume_vel_trace()
        # 期望值把加速斜坡算进去，不用一个宽容差糊过去：
        #   斜坡耗时 v/a = 0.2/2.5 = 0.08s，这段少走的距离 v²/(2a) = 0.008m，
        #   所以 1 秒的路径长是 0.2×1.0 − 0.008 = 0.192m，**不是** 0.2m。
        # 原来写 approx(0.2, abs=5e-3) 是错的：它把一个真实的物理效应当成噪声，
        # 而 0.008 比那个容差还大 —— 容差一放宽就再也测不出斜坡有没有回归。
        v, a = 0.2, 2.5
        expected = v * 1.0 - v * v / (2.0 * a)
        assert tr.cmd_path == pytest.approx(expected, abs=2e-3), (
            'cmd_path=%.6f，按 v=%.2f a=%.2f 算应为 %.6f'
            % (tr.cmd_path, v, a, expected))
        assert tr.cmd_net == pytest.approx(tr.cmd_path, rel=1e-9), (
            '直线走时路径长与净位移必须相等（差一拍就说明窗口起点取在积分之后）：'
            'path=%.9f net=%.9f' % (tr.cmd_path, tr.cmd_net))

    def test_reversing_separates_path_from_net(self):
        """★ 这条是口径断言的核心。

        前进半窗再后退半窗：路径长一直在涨，净位移回到 0。若报告里把 cmd_path
        当净位移去和 act_net 比，就会凭空报出一个"跟踪误差"，而机器人其实完全
        跟住了。

        刻意用「反向」而不是「转圈」来构造这个分离：转圈的几何取决于
        pos_cmd 到底是本体系还是世界系 —— input_frame='body' 时
        to_local_velocity 原样返回、integrate_step_dt 直接把本体速度累加到
        pos_cmd 上，于是"转圈"在指令空间里其实是条直线。那个语义是**尚未定论
        的开放问题**（要靠实机 90° 旋转后纯 vx 的实验来定），拿它当这条测试的
        前提，就是把一个未验证的假设写进断言里。反向不依赖任何坐标系语义。
        """
        core, _, clock = build()
        assert core.enable()[0]
        run(core, clock, 0.5, twist=(0.2, 0.0, 0.0))
        run(core, clock, 0.5, twist=(-0.2, 0.0, 0.0))
        tr = core.consume_vel_trace()
        assert tr.cmd_path > 0.15, '路径长应该实打实地涨了：%.6f' % tr.cmd_path
        assert tr.cmd_net < tr.cmd_path * 0.2, (
            '来回走之后净位移必须远小于路径长，否则这条测试没测到东西：'
            'path=%.6f net=%.6f' % (tr.cmd_path, tr.cmd_net))

    def test_actual_side_is_net_only_never_summed_absolutes(self):
        """actual 侧刻意不累加逐拍 |δ|：那会把抖动整流成单向偏置。

        这里把"抖动"做成实际位置每拍来回跳，净位移为 0。若实现累加了逐拍
        绝对值，act_net 会随拍数线性增长（250Hz 下极显著）。
        """
        core, session, clock = build()
        assert core.enable()[0]
        # follow_ratio=1.0 会让 current 跟住指令；这里指令为零、手工抖 current。
        for i in range(250):
            clock.advance(DT)
            core.submit_scan_seen()
            core.submit_twist(0.0, 0.0, 0.0)
            jitter = 1e-4 if i % 2 == 0 else -1e-4
            session._current[PART] = [jitter, 0.0, 0.0]
            core.inner_tick()
        tr = core.consume_vel_trace()
        assert tr.act_net < 1e-3, (
            'act_net=%.6f —— 逐拍绝对值被累加了，1e-4 的抖动被整流成了 '
            '%.1f 倍的假位移' % (tr.act_net, tr.act_net / 1e-4))


class TestStagesAreVisibleSeparately:
    """每一段"有没有咬住"都要单独可见，否则查不出是哪一段改了速度。"""

    def test_clamp_bite_is_counted(self):
        core, _, clock = build(max_vel_xy=0.1)
        assert core.enable()[0]
        run(core, clock, 0.5, twist=(0.5, 0.0, 0.0))   # 远超限幅
        tr = core.consume_vel_trace()
        assert tr.clamp_bit > 0, 'clamp 明显咬住了却没计数'
        assert tr.in_peak == pytest.approx(0.5, abs=1e-6)
        assert tr.clamped_peak == pytest.approx(0.1, abs=1e-6), (
            'clamped_peak=%.6f，应被限到 max_vel_xy=0.1' % tr.clamped_peak)

    def test_slew_bite_on_angular_only_is_still_counted(self):
        """★ 逐轴判、不按模长判。

        只限住 wz 时 xy 模长一点没变 —— 若实现拿模长比较，这一段就永远
        报"没咬住"，而它实际上正在限制运动。
        """
        core, _, clock = build(max_accel_theta=0.5)
        assert core.enable()[0]
        run(core, clock, 0.1, twist=(0.0, 0.0, 2.0))   # 只给角速度
        tr = core.consume_vel_trace()
        assert tr.slew_bit > 0, (
            'slew 只限住了 wz（xy 模长恒为 0）就没被计数 —— 说明判据用的是模长')

    def test_interlock_zeroing_is_distinguishable_from_silence(self):
        """★ 联锁置零 vs 上游没发：这两件事必须分得开。"""
        core, _, clock = build(require_fresh_scan=True, scan_max_age_sec=0.2)
        assert core.enable()[0]
        # 先跑到 /scan 陈旧为止，然后**清一次窗口** —— 不清的话窗口里会同时含
        # 新鲜期（速度原样通过）和陈旧期，in_peak 取的是全窗最大值，自然还是
        # 0.2，断言"置零之后"就假失败了。我第一版就是这么写错的。
        run(core, clock, 0.3, twist=(0.2, 0.0, 0.0), feed_scan=False)
        assert core.consume_vel_trace().zeroed_ticks > 0, (
            '联锁把速度扔了却没记 —— 日志上会和"上游根本没发"一模一样')
        # 这一段全程陈旧，窗口里只有被置零的拍。
        run(core, clock, 0.2, twist=(0.2, 0.0, 0.0), feed_scan=False)
        tr = core.consume_vel_trace()
        assert tr.zeroed_ticks == tr.ticks, (
            '全程陈旧，%d 拍里只有 %d 拍被记成置零' % (tr.ticks, tr.zeroed_ticks))
        assert tr.in_peak == pytest.approx(0.0, abs=1e-9), (
            'in_peak=%.6f —— 它必须是置零**之后**的值' % tr.in_peak)

    def test_silence_upstream_is_not_counted_as_zeroed(self):
        # 反证：上游本来就没发时，zeroed_ticks 必须是 0，
        # 否则这个计数恒为正 —— 恒真的判据没有信息量。
        core, _, clock = build()
        assert core.enable()[0]
        run(core, clock, 0.5, twist=(0.0, 0.0, 0.0))
        tr = core.consume_vel_trace()
        assert tr.zeroed_ticks == 0


class TestOuterCorrectionIsAccountedSeparately:
    """外环校正直接加在位置上，不经过速度 —— 必须单独有个量。"""

    def test_correction_displacement_is_reported(self):
        core, _, clock = build()
        assert core.enable()[0]
        core.corr_frozen = False
        core.corr_per_tick = (1e-4, 0.0, 0.0)
        run(core, clock, 0.4, twist=(0.0, 0.0, 0.0))
        tr = core.consume_vel_trace()
        assert tr.corr_path > 0.0, (
            '外环校正贡献的位移没被记 —— 它在任何速度量里都查不到，'
            '不在这里记就彻底不可见')
        # 指令速度全程为零，但位置确实动了：这正是"速度是 0 却在移动"的机制。
        assert tr.cmd_path == pytest.approx(0.0, abs=1e-9)
        assert tr.cmd_net > 0.0, (
            'cmd_net=%.9f —— 校正加进 pos_cmd 了，净位移必须看得见' % tr.cmd_net)

    def test_frozen_correction_contributes_nothing(self):
        core, _, clock = build()
        assert core.enable()[0]
        core.corr_frozen = True
        core.corr_per_tick = (1e-4, 0.0, 0.0)
        run(core, clock, 0.4, twist=(0.0, 0.0, 0.0))
        tr = core.consume_vel_trace()
        assert tr.corr_path == pytest.approx(0.0, abs=1e-12), (
            '冻结期间还在记校正位移 —— 那个数是假的')


class TestWindowHygiene:
    """陈旧读数被当成当前值，是本项目反复踩到的那一类错误。"""

    def test_consume_clears_the_window(self):
        core, _, clock = build()
        assert core.enable()[0]
        run(core, clock, 0.5, twist=(0.2, 0.0, 0.0))
        first = core.consume_vel_trace()
        assert first.ticks > 0
        second = core.consume_vel_trace()
        assert second.ticks == 0, (
            '第二次取还有 %d 拍 —— 窗口没清，每条日志都会印同一段历史'
            % second.ticks)
        assert second.cmd_path == pytest.approx(0.0, abs=1e-12)
        assert second.in_peak == pytest.approx(0.0, abs=1e-12)

    def test_enable_reopens_the_window(self):
        """★ 不重开的话，act_net 会跨越停机期，凭空造出一个跟踪误差。"""
        core, session, clock = build()
        assert core.enable()[0]
        run(core, clock, 0.5, twist=(0.2, 0.0, 0.0))
        core.disable()
        # 停机期间实际位置被外力挪走（人推 / 厂商侧自己动）
        session._current[PART] = [5.0, 3.0, 0.0]
        session._desired[PART] = [5.0, 3.0, 0.0]
        clock.advance(30.0)
        assert core.enable()[0]
        run(core, clock, 0.2, twist=(0.0, 0.0, 0.0))
        tr = core.consume_vel_trace()
        assert tr.act_net < 0.01, (
            'act_net=%.4f —— 窗口起点还是停机前那个位置，'
            '这 %.1fm 是口径错误造出来的，不是跟踪误差' % (tr.act_net, tr.act_net))

    def test_no_ticks_reports_zero_ticks_not_a_row_of_zeros(self):
        """一拍都没跑时 ticks 必须是 0，调用方据此跳过打印。

        报一行全 0 会被读成"链路上确实全是零"（上游在发但被吃掉了），
        而真相是内环压根没跑（停用 / 联锁停车）—— 两件事处置完全不同。
        """
        core, _, clock = build()
        tr = core.consume_vel_trace()          # 从未 enable
        assert tr.ticks == 0
        assert tr.wall == pytest.approx(0.0)

    def test_wall_time_is_measured_not_derived_from_freq(self):
        """窗口时长必须是实测墙钟：拿 ticks/freq 反推会在拍率偏低时算错均速。

        这正是内环 dt 那个缺陷的同型错误（按标称频率算，157Hz 下只有 63%）。
        """
        core, _, clock = build(freq=250.0)
        assert core.enable()[0]
        slow_dt = 1.0 / 100.0                  # 实际只有 100Hz
        run(core, clock, 1.0, twist=(0.2, 0.0, 0.0), dt=slow_dt)
        tr = core.consume_vel_trace()
        assert tr.wall == pytest.approx(1.0, abs=0.02), (
            'wall=%.4f，应为实测的 ~1.0s；若按 ticks/freq=%.4f 反推就错了'
            % (tr.wall, tr.ticks / 250.0))
        assert tr.cmd_path / tr.wall == pytest.approx(0.2, abs=0.01), (
            '均速应等于指令速度，与拍率无关')


class TestSlipIsVisible:
    """打滑 = 指令在涨、实际不跟。这是这行日志最该抓到的东西之一。"""

    def test_full_slip_shows_command_without_motion(self):
        core, _, clock = build(follow_ratio=0.0, leash_xy_m=10.0)
        assert core.enable()[0]
        run(core, clock, 0.5, twist=(0.2, 0.0, 0.0))
        tr = core.consume_vel_trace()
        assert tr.cmd_net > 0.05, '指令位移应该实打实地涨了'
        assert tr.act_net == pytest.approx(0.0, abs=1e-9), (
            '完全打滑时实际净位移必须是 0，实测 %.6f' % tr.act_net)

    def test_perfect_following_matches(self):
        core, _, clock = build(follow_ratio=1.0)
        assert core.enable()[0]
        run(core, clock, 0.5, twist=(0.2, 0.0, 0.0))
        tr = core.consume_vel_trace()
        assert tr.act_net == pytest.approx(tr.cmd_net, rel=0.02), (
            '理想跟随下两者应当吻合：cmd_net=%.6f act_net=%.6f'
            % (tr.cmd_net, tr.act_net))
