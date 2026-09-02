#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/scan 时效性联锁的测试。

锁的是什么缺陷
============
2026-09-01 21:26 按急停后实测到的完整链条：

    厂商控制驱动停 → state_bridge 退出 → /joint_states 0Hz
      → robot_state_publisher 停发连杆 TF
        → 自滤逐连杆等满超时（36×50ms）
          → /scan 掉到 0.56Hz、数据龄期 2.03s
            → nav2 照常规划、照常发 /cmd_vel，**零告警**

也就是说：上游感知已经瞎了，而链路上没有任何一环拒绝执行。

为什么必须加在写通路，不能靠 nav2
================================
nav2 的 obstacle_layer 设了 ``expected_update_rate`` 之后**只会告警**。
实测 ``controller_server`` 二进制里没有任何检查 costmap currency 的字符串，
陈旧时它照样继续发速度。写通路是最后一个能说"不"的地方。

为什么现有两道保护都盖不住（都实测过）
====================================
· leash：指令与实测都在动、偏差正常，**不会** trip
· cmd_vel 看门狗：nav2 一直在发指令，``_last_twist_time`` 不为 None，
  走不到"无输入置零"那条分支

本文件不 import rclpy、不 import msgs，任何环境都能跑。
"""

import pytest

from astribot_trajectory_bridge.chassis_bridge_core import (
    ChassisBridgeConfig,
    ChassisBridgeCore,
    ChassisConfigError,
    ST_ENABLED,
    ST_STOPPED_STALE_SCAN,
    S_SCAN_LOST_STOPPED,
    S_SCAN_NEVER_RECEIVED,
    S_SCAN_STALE,
)
from astribot_trajectory_bridge.ports import FakeClock, FakePose, FakeSession

PART = 'astribot_chassis'
DT = 1.0 / 250.0


def build(feed_scan=True, **overrides):
    clock = FakeClock(100.0)
    session = FakeSession(desired={PART: [0.0, 0.0, 0.0]},
                          current={PART: [0.0, 0.0, 0.0]},
                          follow_ratio=1.0)
    pose = FakePose(clock, pose=[0.0, 0.0, 0.0])
    core = ChassisBridgeCore(ChassisBridgeConfig(**overrides),
                             session, pose, clock)
    if feed_scan:
        core.submit_scan_seen()
    return (core, session, pose, clock)


def codes(core):
    return [e.code for e in core.drain_events()]


def run(core, clock, seconds, feed_scan=False, dt=DT, twist=None):
    """跑一段时间。feed_scan=True 时每拍都喂一帧新 /scan。

    !!! twist 默认每拍重发 !!! 这不是图方便，是还原事故场景：
    nav2 在感知失效期间**一直在发** /cmd_vel。若不重发，cmd_vel 看门狗
    （timeout=0.3s）会先把速度置零，于是测出来的"停住了"其实是看门狗的功劳，
    与本联锁无关 —— 我第一版就是这么写的，两条测试因此假失败。
    传 twist=False 可显式模拟"上游也停了"。
    """
    n = int(round(seconds / dt))
    for _ in range(n):
        clock.advance(dt)
        if feed_scan:
            core.submit_scan_seen()
        if twist:
            core.submit_twist(*twist)
        core.inner_tick()


class TestConfigValidation:
    """配置层就该拦住"看着像开着、其实关了"的写法。"""

    def test_defaults_have_interlock_on(self):
        cfg = ChassisBridgeConfig()
        assert cfg.require_fresh_scan is True, '联锁必须默认开启'
        assert cfg.scan_max_age_sec > 0.0

    def test_non_positive_age_rejected_when_enabled(self):
        # 用 <=0 的阈值"顺便"关掉联锁 → 配置读起来像开着，实际永不触发。
        with pytest.raises(ChassisConfigError) as e:
            ChassisBridgeConfig(scan_max_age_sec=0.0)
        assert 'require_fresh_scan' in str(e.value)

    def test_explicit_disable_allows_any_age(self):
        ChassisBridgeConfig(require_fresh_scan=False, scan_max_age_sec=0.0)

    def test_grace_shorter_than_age_rejected(self):
        # 宽限期比判陈旧的阈值还短 = 跳过"置零"直接闩锁，抖一下就要人工复位。
        with pytest.raises(ChassisConfigError) as e:
            ChassisBridgeConfig(scan_max_age_sec=0.5, scan_loss_grace_sec=0.2)
        assert '宽限期' in str(e.value)

    def test_age_matches_upstream_hold_last_bound(self):
        # 0.5s 不是随手取的：上游 pointcloud_slice_scan_node 的
        # hold_last_max_frames=5 @10Hz 保证最多重发 0.5s 后停止输出，
        # 也就是"最长可能隐身时间"。两处必须一致，改一处要改两处。
        assert ChassisBridgeConfig().scan_max_age_sec == pytest.approx(0.5)


class TestFreshScanDoesNotInterfere:
    """★ 反向断言：/scan 正常时联锁必须完全不介入。

    只测"故障时会停"是不够的 —— 一道会误触发的联锁等于让机器人不能用，
    而且现场会被直接关掉。
    """

    def test_stays_enabled_and_moves(self):
        core, _, _, clock = build()
        core.enable()
        core.submit_twist(0.3, 0.0, 0.0)
        run(core, clock, 2.0, feed_scan=True, twist=(0.3, 0.0, 0.0))
        assert core.state == ST_ENABLED
        assert core.pos_cmd[0] > 0.1, '/scan 新鲜时底盘必须照常走'

    def test_no_scan_events_emitted(self):
        core, _, _, clock = build()
        core.enable()
        core.submit_twist(0.3, 0.0, 0.0)
        run(core, clock, 2.0, feed_scan=True, twist=(0.3, 0.0, 0.0))
        got = codes(core)
        for bad in (S_SCAN_STALE, S_SCAN_LOST_STOPPED, S_SCAN_NEVER_RECEIVED):
            assert bad not in got, '不该在 /scan 正常时报 %s' % bad

    def test_brief_gap_within_threshold_is_tolerated(self):
        # 掉一两帧是正常工况（实测 9.88~10.04Hz 本身就在抖）。
        core, _, _, clock = build()
        core.enable()
        core.submit_twist(0.3, 0.0, 0.0)
        clock.advance(0.3)          # < scan_max_age_sec=0.5
        core.inner_tick()
        assert core.state == ST_ENABLED
        assert S_SCAN_STALE not in codes(core)


class TestStaleScanZeroesVelocity:
    """第一层：超阈立刻置零，但不改状态（可自动恢复）。"""

    def test_velocity_zeroed_after_threshold(self):
        core, _, _, clock = build()
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        run(core, clock, 0.4, feed_scan=True, twist=(0.5, 0.0, 0.0))
        assert core.pos_cmd[0] > 0.0

        # 先跑过"容许窗口"(scan_max_age_sec=0.5)。这段里底盘**本来就该继续走** ——
        # 见 test_blind_distance_is_bounded_and_quantified 对这段的量化。
        run(core, clock, 0.6, feed_scan=False, twist=(0.5, 0.0, 0.0))
        after_threshold = core.pos_cmd[0]

        # 阈值之后：这一段才是联锁该压住的。
        # 判据按**物理量**写，不是拍一个小数字：置零要经 slew 限幅收下去
        # （直接把速度打成 0 是位置阶跃，比多走几毫米更糟），
        # 所以理论上界 = 减速距离 v²/(2a) = 0.5²/(2×2.5) = 0.05m。
        # 实测 0.011m，远小于该上界 —— 说明置零发生得比阈值点更早（阈值判定
        # 在超阈那一拍就生效，减速在容许窗口末尾已经开始）。
        cfg = ChassisBridgeConfig()
        brake_bound = 0.5 ** 2 / (2.0 * cfg.max_accel_xy)
        run(core, clock, 0.8, feed_scan=False, twist=(0.5, 0.0, 0.0))
        crept = core.pos_cmd[0] - after_threshold
        assert crept < brake_bound, \
            '超阈后位移 %.4fm 应小于减速距离上界 %.4fm' % (crept, brake_bound)

    def test_blind_distance_is_bounded_and_quantified(self):
        """★ 把这道联锁的**固有代价**钉成数字，而不是假装它不存在。

        阈值 0.5s 意味着：陈旧发生后的头 0.5s 内底盘仍按旧障碍图行走。
        代价 = scan_max_age_sec × 当前速度：
            0.5 m/s → 约 0.25m
        这段距离**无法靠这道联锁消除**，只能靠调小阈值（会误触发，因为上游
        hold_last 保证的隐身上限就是 0.5s）或降低速度。写成测试是为了让将来
        改阈值的人立刻看到它换来/赔掉多少距离。
        """
        v = 0.5
        core, _, _, clock = build()
        core.enable()
        core.submit_twist(v, 0.0, 0.0)
        run(core, clock, 0.3, feed_scan=True, twist=(v, 0.0, 0.0))
        start = core.pos_cmd[0]
        run(core, clock, 1.5, feed_scan=False, twist=(v, 0.0, 0.0))
        blind = core.pos_cmd[0] - start
        expected = v * ChassisBridgeConfig().scan_max_age_sec
        # 上界留 1.5 倍余量给 slew 收敛尾巴；关键是它必须是 0.25m 这个量级，
        # 不是 0（那会掩盖代价）也不是 0.75m（那说明联锁根本没生效）
        assert 0.5 * expected < blind < 1.5 * expected, \
            '盲走距离 %.4fm 应在 %.4fm 量级' % (blind, expected)

    def test_emits_scan_stale(self):
        core, _, _, clock = build()
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        run(core, clock, 0.8, feed_scan=False, twist=(0.5, 0.0, 0.0))
        assert S_SCAN_STALE in codes(core)

    def test_cmd_vel_watchdog_would_not_have_caught_it(self):
        """★ 证明这道联锁不是冗余的。

        持续发 cmd_vel（看门狗永不触发）+ 不喂 scan，
        如果没有本联锁，底盘会一路走下去。
        """
        core, _, _, clock = build(require_fresh_scan=False)
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        for _ in range(int(1.0 / DT)):
            clock.advance(DT)
            core.submit_twist(0.5, 0.0, 0.0)   # 看门狗一直被喂饱
            core.inner_tick()
        got = codes(core)
        assert 'CMD_VEL_TIMEOUT' not in got, '看门狗确实没触发'
        assert core.pos_cmd[0] > 0.3, \
            '关掉联锁时底盘会带着陈旧感知一路走 —— 这就是缺陷本身'
        assert core.state == ST_ENABLED

    def test_second_brief_gap_still_gets_full_grace(self):
        """★ 变异测试逼出来的漏洞：恢复新鲜时必须清掉陈旧计时器。

        不清的话，第二次**短暂**陈旧会立刻闩锁 —— 因为 stale_for 是从
        第一次陈旧算起的。实测该变异的后果：
            健康实现：第二次 0.9s 陈旧后 state=ENABLED
            忘记清零：第二次 0.9s 陈旧后 state=STOPPED_STALE_SCAN
        这是"连续 vs 累计"那类错误的又一次现身 —— 本项目在重试上限、
        hold_last 连击上都栽过同一个坑。
        """
        core, _, _, clock = build()
        core.enable()
        run(core, clock, 0.9, feed_scan=False, twist=(0.3, 0.0, 0.0))
        assert core.state == ST_ENABLED
        run(core, clock, 1.0, feed_scan=True, twist=(0.3, 0.0, 0.0))
        run(core, clock, 0.9, feed_scan=False, twist=(0.3, 0.0, 0.0))
        assert core.state == ST_ENABLED, \
            '第二次短暂陈旧必须重新享有完整宽限期，不能因为第一次的计时残留就闩锁'

    def test_grace_timer_restarts_after_recovery(self):
        """同一件事的另一个角度：恢复后再陈旧，闩锁时刻应由**新**计时决定。"""
        core, _, _, clock = build()
        core.enable()
        run(core, clock, 1.8, feed_scan=False, twist=(0.3, 0.0, 0.0))   # 逼近宽限 2.0
        assert core.state == ST_ENABLED
        run(core, clock, 0.5, feed_scan=True, twist=(0.3, 0.0, 0.0))    # 恢复
        run(core, clock, 1.0, feed_scan=False, twist=(0.3, 0.0, 0.0))   # 再陈旧 1.0s
        assert core.state == ST_ENABLED, \
            '1.8s + 1.0s 不该累加成超过 2.0s 的宽限'

        core, _, _, clock = build()
        core.enable()
        core.submit_twist(0.4, 0.0, 0.0)
        run(core, clock, 0.9, feed_scan=False, twist=(0.4, 0.0, 0.0))  # 陈旧但未超宽限
        assert core.state == ST_ENABLED, '未超宽限期不该闩锁'
        run(core, clock, 1.0, feed_scan=True, twist=(0.4, 0.0, 0.0))   # scan 回来
        assert core.state == ST_ENABLED
        before = core.pos_cmd[0]
        run(core, clock, 0.5, feed_scan=True, twist=(0.4, 0.0, 0.0))
        assert core.pos_cmd[0] > before, '恢复后必须能继续走，不需要人工干预'


class TestPersistentStaleLatchesStop:
    """第二层：持续陈旧超宽限 → 闩锁停车，要人介入。"""

    def test_latches_after_grace(self):
        core, _, _, clock = build()
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        run(core, clock, 3.0, feed_scan=False, twist=(0.5, 0.0, 0.0))      # > scan_loss_grace_sec=2.0
        assert core.state == ST_STOPPED_STALE_SCAN

    def test_emits_lost_stopped(self):
        core, _, _, clock = build()
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        run(core, clock, 3.0, feed_scan=False, twist=(0.5, 0.0, 0.0))
        assert S_SCAN_LOST_STOPPED in codes(core)

    def test_no_dispatch_while_latched(self):
        core, session, _, clock = build()
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        run(core, clock, 3.0, feed_scan=False, twist=(0.5, 0.0, 0.0))
        assert core.state == ST_STOPPED_STALE_SCAN
        frozen = list(core.pos_cmd)
        # scan 恢复也不自动解锁 —— 闩锁的意义就是要人看到
        run(core, clock, 1.0, feed_scan=True, twist=(0.5, 0.0, 0.0))
        assert core.state == ST_STOPPED_STALE_SCAN
        assert core.pos_cmd == frozen, '闩锁后不得再积分'

    def test_enable_clears_the_latch(self):
        core, _, _, clock = build()
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        run(core, clock, 3.0, feed_scan=False, twist=(0.5, 0.0, 0.0))
        assert core.state == ST_STOPPED_STALE_SCAN
        core.submit_scan_seen()
        ok, _ = core.enable()
        assert ok and core.state == ST_ENABLED

    def test_grace_timer_does_not_span_disabled_period(self):
        """★ 与 _tick_first_time 那个坑同型：计时器跨越停机期会算出错的时长。

        enable() 必须重开陈旧计时。判据不能只看"enable 后 scan 正常时没闩锁" ——
        变异测试实证：把 enable() 里那行删掉，那样的判据照样通过（因为 scan
        新鲜时根本走不到闩锁分支）。真正能观测到差别的是**enable 之后再次陈旧**：
        计时残留会让它立刻闩锁，而不是重新享有完整宽限。
        """
        core, _, _, clock = build()
        core.enable()
        run(core, clock, 1.5, feed_scan=False, twist=(0.3, 0.0, 0.0))  # 已陈旧但未闩锁
        core.disable()
        clock.advance(3600.0)                      # 停机一小时
        core.submit_scan_seen()                    # scan 是好的
        core.enable()
        assert core.state == ST_ENABLED
        # 关键一步：再次短暂陈旧，宽限必须是重新开始算的
        run(core, clock, 0.9, feed_scan=False, twist=(0.3, 0.0, 0.0))
        assert core.state == ST_ENABLED, \
            'enable 后陈旧计时必须重开，否则停机时长被算进"持续陈旧"而立刻闩锁'

    def test_reported_duration_excludes_disabled_period(self):
        """★ 变异测试逼出来的第二条：enable() 清计时器**只影响上报数字**。

        为什么必须单独测：把 enable() 里那行删掉，上面那条测试照样通过 ——
        因为 enable 之后只要有一拍 /scan 是新鲜的，else 分支就会把计时器清掉，
        把这个疏漏掩盖过去。真正暴露它的场景是 **enable 时 /scan 就已经陈旧、
        且一直没恢复**（运维在感知没起来时按了使能，完全会发生）。
        那时 stale_for 从上一次使能期算起，会报出"已持续陈旧 3600s"这种跨越
        停机期的数字，把排查引向错误的时间范围。

        这条**不是**安全问题：陈旧时 vel_in 每拍独立置零，与本计时器无关。
        钉住它是因为错的诊断数字会让人查错方向 —— 本项目已因
        "65 拍报成 165Hz"（分母含停机时长）栽过同型的坑。
        """
        core, _, _, clock = build(feed_scan=False)
        core.enable()
        run(core, clock, 1.0, feed_scan=False, twist=(0.3, 0.0, 0.0))
        core.drain_events()
        core.disable()
        clock.advance(3600.0)
        core.enable()          # 使能时 /scan 依然陈旧（从未收到过）
        run(core, clock, 0.8, feed_scan=False, twist=(0.3, 0.0, 0.0))
        durations = [e.metric_1 for e in core.drain_events()
                     if e.code == S_SCAN_NEVER_RECEIVED]
        assert durations, '应当报出陈旧事件'
        assert max(durations) < 10.0, \
            '上报持续时长 %.1fs 跨越了停机期，enable() 必须重开计时' % max(durations)


class TestNeverReceivedIsReportedSeparately:
    """"从未收到"与"收到过但变旧了"必须分开报 —— 排查方向完全不同。"""


    def test_never_received_code(self):
        core, _, _, clock = build(feed_scan=False)
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        run(core, clock, 0.8, feed_scan=False, twist=(0.5, 0.0, 0.0))
        got = codes(core)
        assert S_SCAN_NEVER_RECEIVED in got
        assert S_SCAN_STALE not in got, '从未收到不该报成 STALE'

    def test_never_received_mentions_qos(self):
        """报错文本要直接指向最可能的真因。

        本项目已因 BEST_EFFORT 发布 + RELIABLE 订阅（一帧收不到、只有一条
        WARNING）浪费过一轮排查，这个提示是为那个场景留的。
        """
        core, _, _, clock = build(feed_scan=False)
        core.enable()
        run(core, clock, 0.8, feed_scan=False, twist=(0.5, 0.0, 0.0))
        msgs = [e.detail for e in core.drain_events()
                if e.code == S_SCAN_NEVER_RECEIVED]
        assert msgs and 'QoS' in msgs[0]

    def test_never_received_still_blocks_motion(self):
        core, _, _, clock = build(feed_scan=False)
        core.enable()
        core.submit_twist(0.5, 0.0, 0.0)
        run(core, clock, 0.8, feed_scan=False, twist=(0.5, 0.0, 0.0))
        assert core.pos_cmd[0] < 0.02, '从未收到 /scan 时不得放行运动'


class TestInterlockCanBeDisabledExplicitly:
    """台架场景（没有雷达）要能显式关掉，但必须是显式的。"""

    def test_disabled_interlock_never_trips(self):
        core, _, _, clock = build(feed_scan=False, require_fresh_scan=False)
        core.enable()
        core.submit_twist(0.4, 0.0, 0.0)
        run(core, clock, 5.0, feed_scan=False, twist=(0.4, 0.0, 0.0))
        assert core.state == ST_ENABLED
        got = codes(core)
        for bad in (S_SCAN_STALE, S_SCAN_LOST_STOPPED, S_SCAN_NEVER_RECEIVED):
            assert bad not in got
