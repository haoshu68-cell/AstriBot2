#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回调组布局：每个桥接节点有哪些回调组，以及执行器该开几个线程。

════════════════ 为什么值得单独一个模块 ════════════════
2026-08-31 的实机故障（nav2 一切正常、路径也规划出来了、`/cmd_vel` 有 2708 帧
非零、机器人一动不动）根因是**两个数字不一致**：

  · 底盘节点有 4 个互斥回调组（inner / outer / srv / cmd）
  · 容器按 `len(nodes) * 2 + 2 = 6` 开线程

`MutuallyExclusiveCallbackGroup` 只保证**组内**串行，不保证组间能并发 ——
组分开了但抢不到线程，照样串行。当时 `cmd_vel` 回调一次都没执行到，
`_last_twist` 恒为 `(0, 0, 0)`，而 leash（指令与实测都不动，偏差恒 0）
和看门狗（`_last_twist_time is None` 走静默分支）都不会报。

把"节点声明自己有哪些组"和"线程数怎么算"收进同一个模块，是为了让这条算术
能被离线单测锁住。**本模块刻意不 import rclpy、不 import 任何 msgs** ——
节点模块 import 了 `ros_ports`（依赖 `astribot_bridge_msgs`），在 msgs 未编译的
环境里连 import 都做不到，那样的测试等于没有。

════════════════ 加组时必须做的事 ════════════════
在下面的元组里加一个名字就够了 —— 线程数会自动跟着长。
**不要**在节点里直接 `MutuallyExclusiveCallbackGroup()` 而不登记到这里：
那样组数会悄悄超过线程数，回到 2026-08-31 那个故障。
"""

CHASSIS_GROUPS = ('inner', 'outer', 'srv', 'cmd')

ARM_GROUPS = ('exec', 'srv', 'grip')

THREAD_HEADROOM = 2


class CallbackLayoutError(ValueError):
    """回调组布局非法。属开发期错误，必须当场抛而不是带着跑。"""


def check_group_names(names, where=''):
    """校验一组回调组名合法。返回原元组，方便链式调用。

    Args:
        names: 回调组名序列。
        where: 出错信息里用的位置说明。

    Raises:
        CallbackLayoutError: 名字为空、重名，或不是字符串。
    """
    names = tuple(names)
    if not names:
        raise CallbackLayoutError('%s回调组列表不能为空' % (where and where + ' 的 '))
    for n in names:
        if not isinstance(n, str) or not n:
            raise CallbackLayoutError(
                '%s回调组名必须是非空字符串，收到 %r' % (where and where + ' 的 ', n))
    if len(set(names)) != len(names):
        dup = sorted({n for n in names if list(names).count(n) > 1})
        raise CallbackLayoutError(
            '%s回调组名重复：%s。重名会让两个逻辑上要并发的回调落进同一个组，'
            '正是 2026-08-31 故障的形态。' % (where and where + ' 的 ', dup))
    return names


def make_groups(names, factory, where=''):
    """按声明的名字建回调组，返回 ``{名字: 组对象}``。

    ``factory`` 由调用方传入（本模块不 import rclpy），节点侧传
    ``MutuallyExclusiveCallbackGroup``，测试侧可以传 ``object``。
    """
    return {n: factory() for n in check_group_names(names, where)}


def executor_thread_count(node_group_lists, headroom=THREAD_HEADROOM):
    """算执行器线程数：所有节点的回调组总数 + 余量。

    Args:
        node_group_lists: 每个节点的回调组名序列组成的可迭代对象。
        headroom: 额外余量，默认 :data:`THREAD_HEADROOM`。

    Returns:
        线程数。保证 ``>= 回调组总数``，即每个组都能拿到线程。

    Raises:
        CallbackLayoutError: 没有任何节点，或 headroom 为负。
    """
    lists = [check_group_names(g, '节点#%d' % i)
             for i, g in enumerate(node_group_lists)]
    if not lists:
        raise CallbackLayoutError('没有任何节点，不该建执行器')
    if headroom < 0:
        raise CallbackLayoutError('headroom=%r 不能为负' % (headroom,))
    return sum(len(g) for g in lists) + headroom
