#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪会话接口的契约测试。**离线，用 FakeSession。**

这组测试锁的是"替身与真 SDK 语义一致"
==================================
本项目已经吃过一次替身语义与真 SDK 不符的苦：``FakeSession.set_joints_position``
早期只更新 desired 不更新 current，于是"理想跟随"实际表现成"完全打滑"，
每个移动测试都在 leash 处失败、而失败信息指向业务逻辑。

夹爪这里同样有两个容易写反的语义，必须钉住：
  1. **极性**：open -> 0.0，close -> 100.0（不是反过来）
  2. **仿真下 set_effector_max_force 是空操作** —— 若替身"真的设上了力"，
     就会让"仿真已验收夹持力"这个**错误结论**通过测试
"""

import pytest

from astribot_trajectory_bridge.ports import FakeSession, SdkCallFailure

GL = 'astribot_gripper_left'
GR = 'astribot_gripper_right'


def build(**kw):
    return FakeSession(desired={GL: [0.0], GR: [0.0]},
                       current={GL: [0.0], GR: [0.0]},
                       limits={GL: ([0.0], [100.0]), GR: ([0.0], [100.0])},
                       dofs={GL: 1, GR: 1}, **kw)


class TestPolarity:
    """极性写反的后果是抓取时张开、放置时闭合，且两边都不报错。"""

    def test_open_commands_zero(self):
        s = build()
        s.open_effector([GL])
        assert s.get_desired_joints_position([GL])[0] == pytest.approx([0.0])

    def test_close_commands_hundred(self):
        s = build()
        s.close_effector([GL])
        assert s.get_desired_joints_position([GL])[0] == pytest.approx([100.0])

    def test_close_is_not_zero(self):
        """反向锁：谁把 close 实现成 0 就会失败。"""
        s = build()
        s.close_effector([GL])
        assert s.get_desired_joints_position([GL])[0] != pytest.approx([0.0])

    def test_both_grippers(self):
        s = build()
        s.close_effector([GL, GR])
        for g in (GL, GR):
            assert s.get_desired_joints_position([g])[0] == pytest.approx([100.0])


class TestBlockingSemantics:
    """open/close 是**阻塞**的，且尊重 duration。"""

    def test_returns_sdk_string(self):
        """返回值必须与真 SDK 一致 —— 上层可能在判这个字符串。"""
        s = build()
        assert s.open_effector([GL]) == 'move to joint position success'
        assert s.close_effector([GL]) == 'move to joint position success'

    def test_duration_recorded(self):
        s = build()
        s.close_effector([GL], duration=2.5)
        assert s.effector_calls[-1] == ('close', [GL], 2.5)

    def test_calls_accumulate_in_order(self):
        s = build()
        s.close_effector([GL], duration=1.0)
        s.open_effector([GL], duration=0.5)
        assert [c[0] for c in s.effector_calls] == ['close', 'open']


class TestMaxForceIsNoopInSim:
    """!!! 关键 !!! 仿真下设力是空操作，替身不许"真的设上"。

    若替身真的设上了力，"仿真里夹持力已验收"这个错误结论就会通过测试 ——
    而真机上力限是安全相关的。
    """

    def test_returns_none_in_sim(self):
        s = build()          # in_simulation 默认 True
        assert s.set_effector_max_force([GL], [40]) is None

    def test_force_not_applied_in_sim(self):
        s = build()
        s.set_effector_max_force([GL], [40])
        assert s.effector_max_force is None, (
            '仿真下不该真的设上力 —— 否则会让"仿真已验收夹持力"的错误结论通过')

    def test_attempt_is_still_recorded(self):
        """空操作但要留痕：测试要能断言"上层确实尝试设了力"。"""
        s = build()
        s.set_effector_max_force([GL, GR], [40, 40])
        assert s.effector_force_calls[-1] == ([GL, GR], [40, 40])

    def test_real_semantics_does_apply(self):
        """显式声明真机语义时才真的设上 —— 两种后端语义可区分。"""
        s = build(in_simulation=False)
        s.set_effector_max_force([GL], [60])
        assert s.effector_max_force == [60]


class TestFollowRatioAndFaults:

    def test_follow_ratio_applies_to_gripper(self):
        """follow_ratio 也要作用在夹爪上，否则夹爪测不了"没夹到"。"""
        s = build(follow_ratio=0.0)
        s.close_effector([GL])
        assert s.get_current_joints_position([GL])[0] == pytest.approx([0.0])
        assert s.get_desired_joints_position([GL])[0] == pytest.approx([100.0])

    def test_fault_injection_open(self):
        s = build()
        s.fail_after('open_effector', 0)
        with pytest.raises(SdkCallFailure):
            s.open_effector([GL])

    def test_fault_injection_close(self):
        s = build()
        s.fail_after('close_effector', 0)
        with pytest.raises(SdkCallFailure):
            s.close_effector([GL])

    def test_fault_injection_max_force(self):
        s = build()
        s.fail_after('set_effector_max_force', 0)
        with pytest.raises(SdkCallFailure):
            s.set_effector_max_force([GL], [40])


def _unoverridden(cls):
    """列出 cls **没有覆盖**的端口方法。

    !!! 不能用 hasattr !!! 两个实现都继承 SessionPort，而基类为每个方法都提供了
    一个 raise NotImplementedError 的实现 —— 所以 ``hasattr`` **永远为真**，
    哪怕子类根本没实现。这个坑是实测出来的：先写成 hasattr 版本，
    然后从 AstribotSession 里删掉 close_effector（确认 1 -> 0 真的删掉了），
    测试**照样通过** —— 一条完全空转的断言。

    正确判据是"函数对象是否与基类的同名函数相同"：相同即未覆盖。
    """
    from astribot_trajectory_bridge.ports import SessionPort
    out = []
    for n in dir(SessionPort):
        if n.startswith('_'):
            continue
        base = getattr(SessionPort, n, None)
        if not callable(base):
            continue
        impl = getattr(cls, n, None)
        if impl is None or impl is base:
            out.append(n)
    return out


class TestPortContract:
    """真实适配器与抽象端口的方法集必须一致，否则运行期才 AttributeError。

    这条测试是有来由的：``AstribotSession.get_robot_mode`` 曾经调了
    ``Astribot`` 上**不存在**的属性，296 条离线测试全绿 —— 因为 FakeSession
    实现了它，只有真会话才暴露。方法**名字**层面的缺失可以离线抓到；
    方法体里调错属性抓不到（那需要真后端）。
    """

    def test_fake_overrides_all_port_methods(self):
        assert _unoverridden(FakeSession) == []

    def test_real_session_overrides_all_port_methods(self):
        """只检查**类上是否覆盖**，不构造真会话（那需要后端）。"""
        from astribot_trajectory_bridge.sdk_session import AstribotSession
        assert _unoverridden(AstribotSession) == []

    def test_detector_itself_works(self):
        """自检：探测器必须真的能发现"未覆盖"。

        没有这一条，上面两条可能又是空转的 —— 前一版就是。
        """
        from astribot_trajectory_bridge.ports import SessionPort

        class Half(SessionPort):
            def get_dof(self, names=None):
                return [1]

        missing = _unoverridden(Half)
        assert 'close_effector' in missing
        assert 'get_dof' not in missing
