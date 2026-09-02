#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回调组布局与执行器线程数的回归测试。

这条测试锁的是 2026-08-31 实机故障：nav2 一切正常、`/cmd_vel` 2708 帧非零、
机器人一动不动。根因是**两个数字不一致** —— 底盘节点有 4 个互斥回调组，
容器按 `len(nodes) * 2 + 2 = 6` 开线程；互斥组只保证组内串行，组间抢不到线程
照样排队，`cmd_vel` 回调一次都没执行到。

当时那个 bug 没有任何一条测试能拦住它，因为：
  · 节点模块 import `ros_ports` -> import `astribot_bridge_msgs`，
    msgs 未编译的环境里连 import 都做不到；
  · 线程数是容器里的一个字面量，没有任何地方对它做断言。
所以把声明和算术挪进无依赖的 `callback_layout`，在这里离线锁住。

本文件**不 import rclpy、不 import msgs**，任何环境都能跑。
"""

import pytest

from astribot_trajectory_bridge.callback_layout import (
    ARM_GROUPS,
    CHASSIS_GROUPS,
    THREAD_HEADROOM,
    CallbackLayoutError,
    check_group_names,
    executor_thread_count,
    make_groups,
)


class TestChassisGroupDeclaration:
    """底盘那 4 个组的语义约束。"""

    def test_cmd_and_inner_are_separate_groups(self):
        """★ 这就是 2026-08-31 的故障本身：cmd 与 inner 同组则机器人不动。"""
        assert 'cmd' in CHASSIS_GROUPS
        assert 'inner' in CHASSIS_GROUPS
        assert CHASSIS_GROUPS.index('cmd') != CHASSIS_GROUPS.index('inner'), (
            'cmd_vel 订阅与内环定时器必须是两个不同的回调组。'
            '共用互斥组时内环会把组占满，cmd_vel 回调一次都执行不到，'
            '而 leash（偏差恒 0）和看门狗（_last_twist_time is None 走静默分支）'
            '都不会报 —— 表现为 nav2 正常但机器人静止。')

    def test_all_four_groups_present(self):
        """少任何一个都意味着某类回调被塞进了别人的组。"""
        assert set(CHASSIS_GROUPS) == {'inner', 'outer', 'srv', 'cmd'}

    def test_no_duplicate_names(self):
        check_group_names(CHASSIS_GROUPS, 'chassis')


class TestArmGroupDeclaration:

    def test_exec_group_declared(self):
        """exec 组必须在册 —— 它可重入，但仍要占线程才能与别的组真并发。"""
        assert 'exec' in ARM_GROUPS

    def test_all_three_groups_present(self):
        assert set(ARM_GROUPS) == {'exec', 'srv', 'grip'}

    def test_no_duplicate_names(self):
        check_group_names(ARM_GROUPS, 'arm')


class TestExecutorThreadCount:
    """线程数必须 >= 回调组总数。这条不成立时拆组等于没拆。"""

    def test_threads_cover_every_group_two_node_config(self):
        """生产构型：底盘 + 机械臂。"""
        n = executor_thread_count([CHASSIS_GROUPS, ARM_GROUPS])
        total_groups = len(CHASSIS_GROUPS) + len(ARM_GROUPS)
        assert n >= total_groups, (
            '%d 个线程盖不住 %d 个回调组' % (n, total_groups))
        assert n == total_groups + THREAD_HEADROOM

    def test_the_exact_regression_number(self):
        """★ 旧公式 len(nodes)*2+2 在这个构型下给 6，而组是 7 个 —— 就差这一个。"""
        old_formula = 2 * 2 + 2
        total_groups = len(CHASSIS_GROUPS) + len(ARM_GROUPS)
        assert total_groups == 7
        assert old_formula < total_groups, '这条测试自己写错了'
        assert executor_thread_count([CHASSIS_GROUPS, ARM_GROUPS]) > old_formula

    @pytest.mark.parametrize('node_lists', [
        [CHASSIS_GROUPS],
        [ARM_GROUPS],
        [CHASSIS_GROUPS, ARM_GROUPS],
        [CHASSIS_GROUPS, ARM_GROUPS, ARM_GROUPS],
    ])
    def test_threads_cover_groups_for_any_node_mix(self, node_lists):
        """任何节点组合都要满足下界 —— 只验生产构型会漏掉单节点启动。"""
        n = executor_thread_count(node_lists)
        assert n >= sum(len(g) for g in node_lists)

    def test_adding_a_group_grows_the_thread_count(self):
        """加组必须自动加线程。这正是旧硬编码公式做不到的事。"""
        before = executor_thread_count([CHASSIS_GROUPS])
        after = executor_thread_count([tuple(CHASSIS_GROUPS) + ('extra',)])
        assert after == before + 1

    def test_empty_node_list_is_rejected(self):
        with pytest.raises(CallbackLayoutError):
            executor_thread_count([])

    def test_negative_headroom_is_rejected(self):
        with pytest.raises(CallbackLayoutError):
            executor_thread_count([CHASSIS_GROUPS], headroom=-1)


class TestCheckGroupNames:
    """非法声明必须当场抛，不能带着跑 —— 组数错了没有任何运行期告警。"""

    def test_duplicate_names_rejected(self):
        with pytest.raises(CallbackLayoutError, match='重复'):
            check_group_names(('inner', 'cmd', 'inner'))

    def test_empty_list_rejected(self):
        with pytest.raises(CallbackLayoutError):
            check_group_names(())

    @pytest.mark.parametrize('bad', [('', 'cmd'), (None, 'cmd'), (1, 'cmd')])
    def test_non_string_names_rejected(self, bad):
        with pytest.raises(CallbackLayoutError):
            check_group_names(bad)

    def test_returns_tuple_unchanged(self):
        assert check_group_names(['a', 'b']) == ('a', 'b')


class TestMakeGroups:
    """节点侧建组走的就是这个函数，用 object 当 factory 即可离线验。"""

    def test_one_distinct_object_per_name(self):
        g = make_groups(CHASSIS_GROUPS, object)
        assert set(g) == set(CHASSIS_GROUPS)
        assert len({id(v) for v in g.values()}) == len(CHASSIS_GROUPS), (
            '每个名字必须拿到**独立**的组对象；共用一个等于没分组')

    def test_duplicate_declaration_rejected(self):
        with pytest.raises(CallbackLayoutError):
            make_groups(('inner', 'inner'), object)


class TestNodeClassesDeclareTheirGroups:
    """节点类的 CALLBACK_GROUPS 必须与 callback_layout 一致。

    需要 msgs 已编译（节点模块 import ros_ports -> astribot_bridge_msgs）。
    importorskip 放在**函数体内** —— 放模块级会中止整个 collection，
    见 test_status_code_map.py 顶部说明。
    """

    def test_chassis_node_declares_chassis_groups(self):
        pytest.importorskip('astribot_bridge_msgs.msg',
                            reason='需要先 colcon build astribot_bridge_msgs')
        from astribot_trajectory_bridge.chassis_cmd_bridge_node import (
            ChassisCmdBridgeNode)
        assert ChassisCmdBridgeNode.CALLBACK_GROUPS == CHASSIS_GROUPS

    def test_arm_node_declares_arm_groups(self):
        pytest.importorskip('astribot_bridge_msgs.msg',
                            reason='需要先 colcon build astribot_bridge_msgs')
        from astribot_trajectory_bridge.arm_traj_bridge_node import (
            ArmTrajBridgeNode)
        assert ArmTrajBridgeNode.CALLBACK_GROUPS == ARM_GROUPS
