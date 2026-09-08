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

!!! 不要把 pytest.importorskip 放回模块级 !!!
────────────────────────────────────────────
pytest 6.2.5（Humble 自带版本）下，**模块级**的 importorskip 抛出的 Skipped
会中止整个 collection —— 不是只跳过本文件，而是让 `pytest test/` 一条测试都收不到：

    pytest test/                                    → 1 skipped
    pytest test/ --ignore=test_status_code_map.py    → 430 passed, 4 skipped

这正是本文件原本想避免的"掩盖同包其它测试"，选的机制却干了同一件事。
而且症状伪装得很好：输出是干净的 `1 skipped`，看不出 430 条测试消失了。
（我据此错判过一次"桥接单测全 skip、修复没有回归覆盖"。）

正确写法是模块级 try/except 置标志 + `skipif`（见下），
或者像 test_chassis_bridge_core.py:573 那样把 importorskip 放进**函数体**。

!!! 那个 skipif 曾经让本文件最重要的一条断言从未执行过 !!!
─────────────────────────────────────────────────────
上面这个修复只解决了"collection 被中止"，却保留了"需要编译 msgs"这个前提。
于是在开发机上 `pytest test/test_status_code_map.py` 的输出是 **21 skipped** ——
干净、无报错、看不出任何问题。而实机上 msgs 是编译好的、测试却不在实机上跑。
结果 `SCAN_STALE` / `SCAN_LOST_STOPPED` / `SCAN_NEVER_RECEIVED` 三个状态名
一路出厂，直到实机上 /scan 陈旧那一刻上报路径抛 UnknownStatusCode、
异常穿过定时器回调打死 bridge_container，底盘写通路当场消失。

关键认识：**这条交叉核对从来不需要"编译后的 msg"，它只需要"枚举名的集合"，
而那个集合就在同级包的 .msg 源文件里躺着。** 所以现在改成直接解析 .msg 文本，
零构建、任何机器上都跑。需要编译产物的那几条（数值对齐、srv 常量）才留 skipif。
"""

import pathlib
import re

import pytest

# 模块级只做 try/except 置标志，**绝不**在这里 skip —— 见上方说明。
try:
    from astribot_bridge_msgs.msg import BridgeStatus
    from astribot_bridge_msgs.srv import DispatchWaypoints
    _MSGS_AVAILABLE = True
except ImportError:                                  # pragma: no cover
    BridgeStatus = None
    DispatchWaypoints = None
    _MSGS_AVAILABLE = False

# !!! 不要把这个 mark 提回模块级 pytestmark !!!
# 那会把下面那条"零构建也能跑"的交叉核对一起跳掉 —— 那正是缺陷出厂的原因。
_needs_msgs = pytest.mark.skipif(
    not _MSGS_AVAILABLE,
    reason='需要先 colcon build astribot_bridge_msgs 并 source install/setup.bash')

from astribot_trajectory_bridge import arm_bridge_core as arm  # noqa: E402
from astribot_trajectory_bridge import chassis_bridge_core as ch  # noqa: E402
from astribot_trajectory_bridge import gripper_core as gp  # noqa: E402


def _status_names(module):
    """收集模块里所有 S_ 前缀的状态名常量值。"""
    return {getattr(module, n) for n in dir(module)
            if n.startswith('S_') and isinstance(getattr(module, n), str)}


def _msg_constants():
    return {n for n in dir(BridgeStatus)
            if n.isupper() and not n.startswith('_')
            and isinstance(getattr(BridgeStatus, n), int)}


# ---------------------------------------------------------------------------
# 零构建的交叉核对：直接读 .msg 源文件
# ---------------------------------------------------------------------------

def _bridge_status_msg_path():
    """向上找同级包 astribot_bridge_msgs 里的 BridgeStatus.msg。

    不写死层数：colcon 在不同构建模式下的 rootdir 不一样，写死会变成"找不到就
    skip"，而"找不到就 skip"正是本文件要根除的那个失效模式。
    """
    here = pathlib.Path(__file__).resolve()
    rel = pathlib.Path('astribot_bridge_msgs') / 'msg' / 'BridgeStatus.msg'
    for parent in here.parents:
        candidate = parent / rel
        if candidate.is_file():
            return candidate
    return None


def _msg_constants_from_source():
    """从 .msg 文本解析枚举名 → 数值。"""
    path = _bridge_status_msg_path()
    if path is None:
        # 刻意 fail 而不是 skip：守卫跑不起来必须响亮，
        # 否则又回到"干净的 skipped 掩盖掉真实缺陷"那条老路上。
        pytest.fail(
            '找不到 BridgeStatus.msg（从 %s 向上找 astribot_bridge_msgs/msg/）。'
            '这条守卫不允许静默跳过 —— 它上一次静默跳过的代价是实机上'
            '桥接进程被打死。' % pathlib.Path(__file__).resolve())
    text = path.read_text(encoding='utf-8')
    consts = {m.group(1): int(m.group(2)) for m in
              re.finditer(r'^\s*uint8\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*$',
                          text, re.M)}
    # 正则一个都没匹配到会让下面所有断言变成恒真 —— 本项目已被"0 个匹配"
    # 伪装成"没有问题"骗过一次。所以先给解析结果本身设个下界。
    assert len(consts) >= 20, (
        '只从 %s 解析出 %d 个 uint8 常量，正则大概率没匹配上（文件 %d 字节）。'
        '这条守卫在解析失败时会全部变成恒真断言。'
        % (path, len(consts), len(text)))
    return consts


@pytest.mark.parametrize('module,label', [
    (ch, '底盘'), (arm, '机械臂'), (gp, '夹爪'),
])
def test_every_core_status_name_exists_in_msg_source(module, label):
    """★ 本文件最重要的一条：不需要编译 msgs，任何机器上都跑。

    每一处 ``_emit()`` 的状态名都是模块级 ``S_*`` 常量（已逐处核对，没有任何
    一处是拼出来的字符串），所以反射出的集合就是运行期可能出现的名字全集，
    这条断言因此是完备的 —— 不是抽样。
    """
    consts = _msg_constants_from_source()
    names = _status_names(module)
    assert names, '%s核心一个 S_* 状态名都没反射到，这条测试失效了' % label
    missing = sorted(names - set(consts))
    assert not missing, (
        '%s核心用到的状态名在 BridgeStatus.msg 里没有对应常量：%s。'
        '必须先在 msg 里加枚举再重建 astribot_bridge_msgs —— 否则该故障'
        '一触发，上报路径就会抛 UnknownStatusCode 打死桥接进程。'
        % (label, missing))


def test_msg_source_has_no_duplicate_values():
    """重复枚举值会让上层无法区分两个状态位。源文件级也要查一遍。"""
    consts = _msg_constants_from_source()
    by_value = {}
    for name, value in consts.items():
        by_value.setdefault(value, []).append(name)
    dups = {v: sorted(ns) for v, ns in by_value.items() if len(ns) > 1}
    assert not dups, 'BridgeStatus.msg 里有重复的枚举值：%s' % dups


@_needs_msgs
def test_source_parse_matches_compiled_msg():
    """解析结果必须与编译产物逐位一致。

    上面那条守卫拿源文件当真值源，所以必须钉住"源文件 == 编译产物"；
    否则解析器一漂，守卫就在核对一个不存在的东西。
    """
    from_source = _msg_constants_from_source()
    compiled = {n: getattr(BridgeStatus, n) for n in _msg_constants()}
    assert from_source == compiled, (
        '.msg 源文件解析结果与编译产物不一致。只差名字说明 install 陈旧'
        '（重新 colcon build astribot_bridge_msgs）；数值也不同说明解析器有问题。'
        '仅在源中=%s 仅在编译产物中=%s'
        % (sorted(set(from_source) - set(compiled)),
           sorted(set(compiled) - set(from_source))))


def test_scan_interlock_states_are_present():
    """★ 回归钉子：三个 /scan 联锁状态位。

    这三个名字的缺失曾经让 /scan 一陈旧就打死底盘写通路 —— 一个用来保证行车
    安全的联锁，在它触发的那一刻摧毁了它所保护的东西。上面的通用守卫已经能
    覆盖它，这里再按名字钉一遍：通用守卫是"核心层用到的都在"，
    而这条是"这三个具体的联锁状态位必须一直在"，删掉任一个都要当场失败。
    """
    consts = _msg_constants_from_source()
    for name in ('SCAN_STALE', 'SCAN_LOST_STOPPED', 'SCAN_NEVER_RECEIVED'):
        assert name in consts, (
            'BridgeStatus.msg 缺少 %s。/scan 时效性联锁的三个状态位'
            '（陈旧 / 闩锁停车 / 从未收到）必须各有独立枚举：'
            '前两者是上游故障，后者几乎总是话题名或 QoS 配错，排查方向不同。'
            % name)


class TestChassisStatusNames:

    @_needs_msgs
    def test_every_name_has_msg_constant(self):
        missing = _status_names(ch) - _msg_constants()
        assert not missing, (
            '底盘核心用到的状态名在 BridgeStatus.msg 里没有对应常量：%s。'
            '新增故障类型必须同时在 msg 里加枚举。' % sorted(missing))


class TestArmStatusNames:

    @_needs_msgs
    def test_every_name_has_msg_constant(self):
        missing = _status_names(arm) - _msg_constants()
        assert not missing, (
            '机械臂核心用到的状态名在 BridgeStatus.msg 里没有对应常量：%s'
            % sorted(missing))


@_needs_msgs
class TestStatusCodeLookup:

    def test_known_name_maps(self):
        from astribot_trajectory_bridge.ros_ports import status_code_of
        assert status_code_of('LEASH_TRIPPED') == BridgeStatus.LEASH_TRIPPED

    def test_unknown_name_raises(self):
        # 显式失败而不是回落到默认值：回落会让诊断节点收到错误语义的状态位。
        # 注意这里测的是 status_code_of 本身 —— 它必须继续抛。
        # 容错边界在 StatusReporter.publish 那一层，不在这里（见下面那组测试）。
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

    def test_all_core_names_mappable_passes(self):
        """启动期校验在当前代码上必须通过，且必须真的核对了一批名字。

        只断言"不抛"是不够的：反射一个名字都没找到时它也不抛。
        """
        from astribot_trajectory_bridge.ros_ports import (
            assert_all_core_names_mappable, core_status_names)
        checked = assert_all_core_names_mappable()
        assert checked >= 20, (
            '启动期校验只核对了 %d 个状态名，反射大概率漏了模块 —— '
            '核对到的是 %s' % (checked, sorted(core_status_names())))

    def test_core_names_reflection_covers_all_core_modules(self):
        """反射必须覆盖到三个核心层模块，漏一个就等于那个模块不受保护。"""
        from astribot_trajectory_bridge.ros_ports import core_status_names
        sources = {src.split('.')[0]
                   for srcs in core_status_names().values() for src in srcs}
        for mod in ('chassis_bridge_core', 'arm_bridge_core', 'gripper_core'):
            assert mod in sources, (
                '%s 的状态名没被 core_status_names() 反射到，'
                '它的状态名不受启动期校验保护' % mod)


@_needs_msgs
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


@_needs_msgs
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
