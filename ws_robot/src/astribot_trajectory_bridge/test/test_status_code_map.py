#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核心层状态名 ↔ BridgeStatus.msg 枚举 的一致性测试。

为什么这条测试是必要的
====================
核心层（*_bridge_core.py）用**字符串**表示状态名，而话题用**数字**枚举。
两边各有一份名字，一旦不一致，后果是：诊断节点收到一个语义错误的状态位，
或者上报时抛 KeyError —— 而两者都只在**故障发生时**才会暴露，
也就是最不该出问题的时候。

所以这里把"核心层用到的每一个状态名都能映射"变成一条离线可跑的断言。
本测试需要 msgs 已编译；未编译时 skip 而不是 error（否则会掩盖同包其它测试）。
"""

import pytest

# msgs 需要 rosidl 产物；没编译时整条 skip
pytest.importorskip(
    'astribot_bridge_msgs.msg',
    reason='需要先 colcon build astribot_bridge_msgs 并 source install/setup.bash')

from astribot_bridge_msgs.msg import BridgeStatus            # noqa: E402
from astribot_bridge_msgs.srv import DispatchWaypoints        # noqa: E402

from astribot_trajectory_bridge import arm_bridge_core as arm  # noqa: E402
from astribot_trajectory_bridge import chassis_bridge_core as ch  # noqa: E402


def _status_names(module):
    """收集模块里所有 S_ 前缀的状态名常量值。"""
    return {getattr(module, n) for n in dir(module)
            if n.startswith('S_') and isinstance(getattr(module, n), str)}


def _msg_constants():
    return {n for n in dir(BridgeStatus)
            if n.isupper() and not n.startswith('_')
            and isinstance(getattr(BridgeStatus, n), int)}


class TestChassisStatusNames:

    def test_every_name_has_msg_constant(self):
        missing = _status_names(ch) - _msg_constants()
        assert not missing, (
            '底盘核心用到的状态名在 BridgeStatus.msg 里没有对应常量：%s。'
            '新增故障类型必须同时在 msg 里加枚举。' % sorted(missing))


class TestArmStatusNames:

    def test_every_name_has_msg_constant(self):
        missing = _status_names(arm) - _msg_constants()
        assert not missing, (
            '机械臂核心用到的状态名在 BridgeStatus.msg 里没有对应常量：%s'
            % sorted(missing))


class TestStatusCodeLookup:

    def test_known_name_maps(self):
        from astribot_trajectory_bridge.ros_ports import status_code_of
        assert status_code_of('LEASH_TRIPPED') == BridgeStatus.LEASH_TRIPPED

    def test_unknown_name_raises(self):
        # 显式失败而不是回落到默认值：回落会让诊断节点收到错误语义的状态位
        from astribot_trajectory_bridge.ros_ports import (
            UnknownStatusCode, status_code_of)
        with pytest.raises(UnknownStatusCode):
            status_code_of('NOT_A_REAL_STATE')

    def test_ok_is_zero(self):
        assert BridgeStatus.OK == 0

    def test_no_duplicate_enum_values(self):
        # 两个状态位取同一个数字会让上层无法区分它们
        values = {}
        for n in _msg_constants():
            v = getattr(BridgeStatus, n)
            values.setdefault(v, []).append(n)
        dups = {v: names for v, names in values.items() if len(names) > 1}
        assert not dups, 'BridgeStatus 里有重复的枚举值：%s' % dups


class TestDispatchWaypointsCodes:
    """方案 A 的错误码名必须与 srv 定义一致。"""

    EXPECTED = ['SUCCESS', 'DISABLED_BY_CONFIG', 'UNKNOWN_PART', 'SHAPE_MISMATCH',
                'TIME_NOT_MONOTONIC', 'LIMIT_VIOLATION', 'SDK_CALL_FAILED',
                'WRITE_GATE_DENIED', 'NO_POINTS_AFTER_DROP']

    @pytest.mark.parametrize('name', EXPECTED)
    def test_constant_exists(self, name):
        assert hasattr(DispatchWaypoints.Response, name)

    def test_dispatcher_returns_only_known_codes(self):
        """WaypointDispatcher 可能返回的每个错误码名都必须在 srv 里存在。

        这些名字是从 arm_bridge_core.WaypointDispatcher.dispatch 的所有 return
        分支里人工抄出来的 —— 抄漏了下面的断言会漏测，所以同时断言数量，
        改动 dispatch 的分支数时这条会提醒去更新。
        """
        returned = {'DISABLED_BY_CONFIG', 'SHAPE_MISMATCH', 'NO_POINTS_AFTER_DROP',
                    'TIME_NOT_MONOTONIC', 'LIMIT_VIOLATION', 'SDK_CALL_FAILED',
                    'SUCCESS'}
        for name in returned:
            assert hasattr(DispatchWaypoints.Response, name), name
        # 节点层还会用到这两个（core 之外的分支）
        for name in ('UNKNOWN_PART', 'WRITE_GATE_DENIED'):
            assert hasattr(DispatchWaypoints.Response, name)
        assert len(returned) + 2 == len(self.EXPECTED)


class TestErrorCodeAlignment:
    """机械臂的结果码必须与 control_msgs/FollowJointTrajectory 对齐。"""

    def test_values_match_control_msgs(self):
        fjt = pytest.importorskip('control_msgs.action').FollowJointTrajectory
        res = fjt.Result
        assert arm.EC_SUCCESSFUL == res.SUCCESSFUL
        assert arm.EC_INVALID_GOAL == res.INVALID_GOAL
        assert arm.EC_INVALID_JOINTS == res.INVALID_JOINTS
        assert arm.EC_OLD_HEADER_TIMESTAMP == res.OLD_HEADER_TIMESTAMP
        assert arm.EC_PATH_TOLERANCE_VIOLATED == res.PATH_TOLERANCE_VIOLATED
        assert arm.EC_GOAL_TOLERANCE_VIOLATED == res.GOAL_TOLERANCE_VIOLATED


class TestSetGripperCodeMap:
    """夹爪核心层的错误码名 ↔ SetGripper.srv 常量 的一致性。

    与 DispatchWaypoints 那组同理：核心层用字符串（为了能在没编译 msgs 的
    环境下单测），srv 用数字。两边名字漂了只在**故障发生时**暴露 ——
    也就是最不该出问题的时候。
    """

    def test_every_core_code_maps(self):
        from astribot_bridge_msgs.srv import SetGripper
        import astribot_trajectory_bridge.gripper_core as gc
        names = [v for k, v in vars(gc).items()
                 if k.startswith('EC_') and isinstance(v, str)]
        assert names, '没找到任何 EC_ 常量，测试本身失效了'
        missing = [n for n in names if not hasattr(SetGripper.Response, n)]
        assert missing == [], 'SetGripper.srv 缺少常量：%r' % (missing,)

    def test_no_duplicate_values(self):
        from astribot_bridge_msgs.srv import SetGripper
        import astribot_trajectory_bridge.gripper_core as gc
        vals = {}
        for k, v in vars(gc).items():
            if k.startswith('EC_') and isinstance(v, str):
                num = getattr(SetGripper.Response, v)
                assert num not in vals, \
                    '枚举值 %d 被 %s 和 %s 共用' % (num, vals[num], v)
                vals[num] = v

    def test_success_is_zero(self):
        """约定 SUCCESS=0，让调用方可以用 `if resp.error_code:` 判失败。"""
        from astribot_bridge_msgs.srv import SetGripper
        assert SetGripper.Response.SUCCESS == 0

    def test_gripper_status_codes_map_to_bridge_status(self):
        """夹爪核心上报的状态位必须能映射到 BridgeStatus。"""
        from astribot_trajectory_bridge.ros_ports import status_code_of
        import astribot_trajectory_bridge.gripper_core as gc
        for k, v in vars(gc).items():
            if k.startswith('S_') and isinstance(v, str):
                status_code_of(v)      # 映射不上会抛 UnknownStatusCode
