#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cmd_vel_body_to_world_node 姿态止损判定的纯函数单测。

这一份存在的理由是一次实机排查（2026-09-02）：那个节点的姿态监控里叠了
两个缺陷，而它们**互相掩盖**，所以在实机上表现为"一切正常"：

  缺陷 A：/odom 订阅用默认 QoS(RELIABLE)，而实机 /odom 发布者是 BEST_EFFORT
          -> 回调一帧都没执行过（实测 RELIABLE 0 帧 / BEST_EFFORT 704 帧@50Hz）
  缺陷 B：normal_height=0.134 是仿真值；实机 /odom 是轮式里程计，
          z/roll/pitch 恒等于 0 -> |0-0.134|=0.134 > 0.06 -> 判为异常姿态

A 掩盖了 B。谁把 A "顺手修好"（一个看起来完全正确的修法），节点就会在
收到第一帧 /odom 时永久停车，而 safety_tripped 没有复位路径。

所以本文件里最重要的不是"阈值算得对不对"，而是
TestDegeneracyMustBeCheckedFirst —— 它钉住"数据源退化必须先于超限判定"。
"""

import pytest

from astribot_s1_navigation.posture_monitor_policy import (
    ACT_COLLECTING,
    ACT_DISABLED,
    ACT_DISABLE_DEGENERATE,
    ACT_PASS,
    ACT_TRIP,
    evaluate_posture,
    DEGENERACY_EPS,
    MIN_SAMPLES_FOR_DEGENERACY,
    describe_monitor_state,
    is_degenerate_attitude_source,
    posture_out_of_bounds,
)

# 节点里的默认值。刻意在这里重复一遍并加断言（见 test_defaults_match_node），
# 而不是 import 节点模块 —— 那会把 rclpy 拖进来，离线测就跑不了。
SIM_NORMAL_H = 0.134
SIM_MAX_DEV = 0.06
SIM_MAX_TILT = 0.12


class TestPostureBounds:

    def test_level_at_normal_height_is_ok(self):
        bad, why = posture_out_of_bounds(
            SIM_NORMAL_H, 0.0, 0.0, SIM_NORMAL_H, SIM_MAX_DEV, SIM_MAX_TILT)
        assert bad is False and why is None

    def test_height_just_inside_band(self):
        z = SIM_NORMAL_H + SIM_MAX_DEV - 1e-6
        bad, _ = posture_out_of_bounds(
            z, 0.0, 0.0, SIM_NORMAL_H, SIM_MAX_DEV, SIM_MAX_TILT)
        assert bad is False

    def test_height_just_outside_band(self):
        z = SIM_NORMAL_H + SIM_MAX_DEV + 1e-6
        bad, why = posture_out_of_bounds(
            z, 0.0, 0.0, SIM_NORMAL_H, SIM_MAX_DEV, SIM_MAX_TILT)
        assert bad is True
        assert '高度偏差' in why

    def test_roll_and_pitch_each_trip_independently(self):
        for roll, pitch, token in ((SIM_MAX_TILT + 1e-6, 0.0, '横滚'),
                                   (0.0, SIM_MAX_TILT + 1e-6, '俯仰')):
            bad, why = posture_out_of_bounds(
                SIM_NORMAL_H, roll, pitch,
                SIM_NORMAL_H, SIM_MAX_DEV, SIM_MAX_TILT)
            assert bad is True
            assert token in why, (
                '原实现把 z/roll/pitch 三个量一起报，读日志的人得自己比对三个'
                '阈值才知道是哪一项触发的。原因字符串必须指明触发项。')

    def test_negative_tilt_also_trips(self):
        """符号不能漏 —— 往一边倒和往另一边倒都是倾倒。"""
        bad, _ = posture_out_of_bounds(
            SIM_NORMAL_H, -(SIM_MAX_TILT + 1e-6), 0.0,
            SIM_NORMAL_H, SIM_MAX_DEV, SIM_MAX_TILT)
        assert bad is True

    def test_hardware_zero_odom_trips_under_sim_thresholds(self):
        """**这就是缺陷 B 本身**：实机 z=0 在仿真阈值下必然被判为异常。

        这条测试刻意断言"会触发"，不是断言"应该触发"。
        它的作用是把那个事实钉在测试里：一旦有人把 normal_height 改到能容纳 0，
        这条测试会失败，从而强制他读到下面这段说明 ——
        让阈值容纳 0 只是让监控永不触发，**检测不出真的倾倒**，是假的安全感。
        正确做法是换 IMU 数据源，或按实机显式禁用。
        """
        bad, why = posture_out_of_bounds(
            0.0, 0.0, 0.0, SIM_NORMAL_H, SIM_MAX_DEV, SIM_MAX_TILT)
        assert bad is True
        assert '高度偏差' in why


class TestDegeneracyDetection:

    def test_all_constant_zero_is_degenerate(self):
        s = [(0.0, 0.0, 0.0)] * MIN_SAMPLES_FOR_DEGENERACY
        assert is_degenerate_attitude_source(s) is True

    def test_constant_nonzero_is_also_degenerate(self):
        """判据是"恒定不变"，不是"等于 0"。

        换一台机器人/换一个 odom 源，那个常数就可能不是 0；
        "恒定"才是"这个源不携带信息"的本质。
        """
        s = [(0.1292, 0.0, 0.0)] * MIN_SAMPLES_FOR_DEGENERACY
        assert is_degenerate_attitude_source(s) is True

    def test_any_variation_is_not_degenerate(self):
        for idx in range(3):
            s = []
            for i in range(MIN_SAMPLES_FOR_DEGENERACY):
                v = [0.0, 0.0, 0.0]
                v[idx] = 1e-3 * i
                s.append(tuple(v))
            assert is_degenerate_attitude_source(s) is False, (
                '第 %d 列有变化就不该判退化' % idx)

    def test_too_few_samples_is_never_degenerate(self):
        """样本不足时宁可不下结论 —— 凭 2 帧宣布数据源坏了是错的。"""
        for k in range(MIN_SAMPLES_FOR_DEGENERACY):
            s = [(0.0, 0.0, 0.0)] * k
            assert is_degenerate_attitude_source(s) is False, (
                '%d 帧就判退化，样本数下限没起作用' % k)
        s = [(0.0, 0.0, 0.0)] * MIN_SAMPLES_FOR_DEGENERACY
        assert is_degenerate_attitude_source(s) is True

    def test_variation_below_eps_still_degenerate(self):
        """浮点噪声级别的抖动不算携带信息。"""
        s = [(DEGENERACY_EPS / 4.0 * (i % 2), 0.0, 0.0)
             for i in range(MIN_SAMPLES_FOR_DEGENERACY)]
        assert is_degenerate_attitude_source(s) is True

    def test_min_samples_covers_less_than_a_second(self):
        """样本窗必须短于任何真实姿态变化，否则退化判定会先被超限判定抢跑。

        实机 /odom 实测 50Hz（2026-09-02：601 帧/12s）。
        """
        odom_hz = 50.0
        assert MIN_SAMPLES_FOR_DEGENERACY / odom_hz < 1.0, (
            '退化判定窗 %.2fs 太长' % (MIN_SAMPLES_FOR_DEGENERACY / odom_hz))


class TestDegeneracyMustBeCheckedFirst:
    """**本文件最重要的一组。**

    实机 /odom 的 z=0 同时满足两件事：
      · 在仿真阈值下"超限"（缺陷 B）
      · 是一个不携带姿态信息的退化源（真相）
    两个判定都会命中，所以**顺序决定行为**：
      · 先判超限 -> 永久停车，且日志说"检测到异常姿态"（错误结论）
      · 先判退化 -> 停用监控，并说明数据源不携带姿态信息（正确结论）
    """

    HW = [(0.0, 0.0, 0.0)] * MIN_SAMPLES_FOR_DEGENERACY

    def test_hardware_odom_satisfies_both_predicates(self):
        """先证明这两个判定在实机数据上确实**同时**成立。

        否则下面那条"顺序有意义"的测试就是空转的。
        """
        assert is_degenerate_attitude_source(self.HW) is True
        bad, _ = posture_out_of_bounds(
            0.0, 0.0, 0.0, SIM_NORMAL_H, SIM_MAX_DEV, SIM_MAX_TILT)
        assert bad is True, (
            '如果实机 z=0 在当前阈值下**不**超限，那顺序就无所谓了，'
            '本组测试也就失去意义 —— 说明阈值被改过，请重新评估。')

    def test_degenerate_verdict_is_not_a_trip(self):
        """退化和止损是两种不同的状态，措辞必须能区分。"""
        deg = describe_monitor_state(True, True, False)
        trip = describe_monitor_state(True, False, True)
        assert deg != trip
        assert '不携带姿态信息' in deg
        assert '止损' in trip
        assert '永久' in deg or '误判' in deg, (
            '退化的说明必须点出"继续按阈值判会永久停车"，'
            '否则读者不知道这条日志避免了什么')


class TestMonitorStateDescription:

    def test_four_states_are_all_distinct(self):
        """原实现里"监控开着但收不到数据"与"监控开着且一切正常"在日志上
        **完全无法区分** —— 两种都只有那一条启动 INFO，之后再无输出。"""
        states = {
            describe_monitor_state(False, False, False),
            describe_monitor_state(True, True, False),
            describe_monitor_state(True, False, True),
            describe_monitor_state(True, False, False),
        }
        assert len(states) == 4

    def test_disabled_explains_why_not_just_that(self):
        msg = describe_monitor_state(False, False, False)
        for token in ('轮式里程计', 'IMU'):
            assert token in msg, (
                '禁用说明必须写清"为什么不能用这个数据源"和"真要做该用什么"，'
                '缺 %r' % token)

    def test_online_state_is_not_falsely_reassuring(self):
        """只有真的在线时才允许说"在线"。"""
        assert '在线' in describe_monitor_state(True, False, False)
        for msg in (describe_monitor_state(False, False, False),
                    describe_monitor_state(True, True, False),
                    describe_monitor_state(True, False, True)):
            assert '在线' not in msg


def test_defaults_match_node():
    """本文件里抄的三个默认值必须和节点源码一致，否则上面的断言在验别的东西。"""
    import os
    import re
    here = os.path.dirname(os.path.abspath(__file__))
    node = os.path.join(here, '..', 'astribot_s1_navigation',
                        'cmd_vel_body_to_world_node.py')
    node = os.path.abspath(node)
    if not os.path.isfile(node):
        pytest.skip('找不到节点源码: %s' % node)
    src = open(node, encoding='utf-8').read()
    want = {'normal_height': SIM_NORMAL_H,
            'max_height_deviation': SIM_MAX_DEV,
            'max_tilt_rad': SIM_MAX_TILT}
    for name, expect in want.items():
        m = re.search(r"declare_parameter\(\s*'%s'\s*,\s*([0-9.]+)" % name, src)
        assert m, '节点里找不到 %s 的 declare_parameter' % name
        assert abs(float(m.group(1)) - expect) < 1e-12, (
            '节点里 %s=%s，本测试文件抄的是 %s —— 不一致，'
            '上面的断言在验一个不存在的配置' % (name, m.group(1), expect))


def test_posture_monitor_param_exists_and_defaults_true():
    """实机靠这个开关关掉监控；默认必须是 true 以保持仿真行为不变。"""
    import os
    import re
    here = os.path.dirname(os.path.abspath(__file__))
    node = os.path.abspath(os.path.join(
        here, '..', 'astribot_s1_navigation', 'cmd_vel_body_to_world_node.py'))
    if not os.path.isfile(node):
        pytest.skip('找不到节点源码')
    src = open(node, encoding='utf-8').read()
    m = re.search(r"declare_parameter\(\s*'enable_posture_monitor'\s*,\s*(\w+)",
                  src)
    assert m, '节点里没有 enable_posture_monitor 参数'
    assert m.group(1) == 'True', (
        'enable_posture_monitor 默认值是 %s，应为 True —— '
        '默认关掉会静默改变仿真行为' % m.group(1))


def test_odom_subscription_uses_best_effort():
    """QoS 回归钉子。

    这条测试防的是"把 QoS 改回默认 RELIABLE"——那会让实机上的 /odom 回调
    一帧都收不到（实测 0 帧 vs BEST_EFFORT 704 帧@50Hz），
    而节点活着、发布者数正常、日志无异常，按 pub>0 写的判据一条都发现不了。
    """
    import os
    import re
    here = os.path.dirname(os.path.abspath(__file__))
    node = os.path.abspath(os.path.join(
        here, '..', 'astribot_s1_navigation', 'cmd_vel_body_to_world_node.py'))
    if not os.path.isfile(node):
        pytest.skip('找不到节点源码')
    src = open(node, encoding='utf-8').read()
    m = re.search(r'create_subscription\(\s*Odometry\s*,\s*[\w.]+\s*,'
                  r'\s*[\w.]+\s*,\s*([^)]+)\)', src)
    assert m, '找不到 Odometry 的 create_subscription'
    qos = m.group(1).strip()
    assert 'sensor_data' in qos or 'BEST_EFFORT' in qos, (
        '/odom 订阅的 QoS 是 %r —— 纯数字(depth)意味着默认 RELIABLE，'
        '而实机 /odom 发布者是 BEST_EFFORT，单向不兼容会让回调一帧都收不到。' % qos)


# ================================================ 判定顺序（本文件的核心）

class TestEvaluatePostureOrdering:
    """钉住"退化判定必须先于超限判定"。

    这一组之所以必须存在：判定顺序原先写在节点的 callback 里，
    实测"把顺序调回去"这个变异在前 20 条测试下**全部存活** ——
    因为那 20 条测的都是纯函数各自的行为，没有一条测组合顺序。
    """

    TH = (SIM_NORMAL_H, SIM_MAX_DEV, SIM_MAX_TILT)
    HW = [(0.0, 0.0, 0.0)] * MIN_SAMPLES_FOR_DEGENERACY

    def test_hardware_odom_disables_not_trips(self):
        """**这是本文件最重要的一条断言。**

        实机数据必须得到"停用监控"，而不是"止损停车"。
        顺序反了这条就会拿到 ACT_TRIP。
        """
        act, _ = evaluate_posture(True, self.HW, 0.0, 0.0, 0.0, *self.TH)
        assert act == ACT_DISABLE_DEGENERATE, (
            '实机 /odom(z/roll/pitch 恒为 0) 应判为数据源退化并停用监控，'
            '实际得到 %r。若是 ACT_TRIP，说明超限判定跑在了退化判定前面 —— '
            '后果是永久停车 + 日志给出"检测到异常姿态"这个错误结论。' % act)

    def test_disabled_short_circuits_everything(self):
        """显式禁用时，连退化判定都不该跑（不该打那条 error）。"""
        act, why = evaluate_posture(False, self.HW, 0.0, 0.0, 0.0, *self.TH)
        assert act == ACT_DISABLED
        assert why is None

    def test_collecting_before_enough_samples(self):
        """样本不够时必须放行，且**不做超限判定**。

        少了这一步：第一帧就触发止损，退化检测永远等不到它需要的样本数。
        用一个"有变化因此非退化、但超限"的序列来测。
        """
        samples = [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0)]
        assert len(samples) < MIN_SAMPLES_FOR_DEGENERACY
        act, _ = evaluate_posture(True, samples, 0.5, 0.0, 0.0, *self.TH)
        assert act == ACT_COLLECTING, (
            '样本不足时应为 ACT_COLLECTING，实际 %r。'
            '若是 ACT_TRIP，退化检测就永远攒不到样本了。' % act)

    def test_real_tipping_still_trips(self):
        """真的倾倒必须仍然止损 —— 退化检测不能把真故障也吞掉。

        构造：roll 在窗口内**有变化**（所以非退化），且末值超过阈值。
        """
        samples = [(SIM_NORMAL_H, 0.01 * i, 0.0)
                   for i in range(MIN_SAMPLES_FOR_DEGENERACY)]
        z, roll, pitch = SIM_NORMAL_H, SIM_MAX_TILT + 0.05, 0.0
        samples.append((z, roll, pitch))
        act, why = evaluate_posture(True, samples, z, roll, pitch, *self.TH)
        assert act == ACT_TRIP, (
            '姿态源是活的且真的倾倒时必须止损，实际 %r' % act)
        assert '横滚' in why

    def test_healthy_varying_source_passes(self):
        samples = [(SIM_NORMAL_H + 0.001 * i, 0.001 * i, 0.0)
                   for i in range(MIN_SAMPLES_FOR_DEGENERACY + 5)]
        z, roll, pitch = samples[-1]
        act, why = evaluate_posture(True, samples, z, roll, pitch, *self.TH)
        assert act == ACT_PASS
        assert why is None

    def test_node_uses_the_pure_decision_not_its_own_ordering(self):
        """回归钉子：节点必须调 evaluate_posture，不许把顺序再抄回 callback。

        抄回去的后果不是功能错误，而是**顺序重新变得测不到** ——
        这正是本组测试要防的那件事。
        """
        import os
        here = os.path.dirname(os.path.abspath(__file__))
        node = os.path.abspath(os.path.join(
            here, '..', 'astribot_s1_navigation',
            'cmd_vel_body_to_world_node.py'))
        if not os.path.isfile(node):
            pytest.skip('找不到节点源码')
        src = open(node, encoding='utf-8').read()
        assert 'evaluate_posture(' in src, (
            '节点没有调用 evaluate_posture —— 判定顺序又回到了 callback 里，'
            '那里没有任何测试能钉住它')
        body = src[src.find('def odom_callback'):src.find('def cmd_vel_callback')]
        assert 'posture_out_of_bounds(' not in body, (
            'odom_callback 里直接调了 posture_out_of_bounds —— '
            '顺序判断又被搬回节点，测试覆盖不到')
