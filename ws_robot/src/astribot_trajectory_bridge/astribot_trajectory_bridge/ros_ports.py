#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 侧端口实现（TF 位姿源、ROS 时钟）与状态上报。

这三样是节点层唯一需要接触 rclpy 的地方；控制逻辑全在 *_bridge_core 里，
不依赖 rclpy，所以分支能离线测。
"""

import math

from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from astribot_bridge_msgs.msg import BridgeStatus
from astribot_trajectory_bridge.ports import ClockPort, PosePort


class RosClock(ClockPort):
    """节点时钟。用节点自己的 clock 而不是 time.time()，这样 use_sim_time 生效。"""

    def __init__(self, node):
        self._node = node

    def now(self):
        return self._node.get_clock().now().nanoseconds * 1e-9


class TfPosePort(PosePort):
    """从 TF 取 ``map -> base`` 位姿。

    !!! 用 Time() 取最新、**绝不带 timeout** !!!
    项目笔记记过一个坑：带 timeout 的**动态** TF 查询在非专用线程里必然失败，
    而 **static** TF 却能成功。放到本模块的语境里就是：
    ``pose_source=ground_truth``（map->odom 是静态 TF）会"看着正常"，而
    ``slam`` 模式却查不到 —— 同一份代码在两种配置下表现相反，极难归因。
    另外带 timeout 会阻塞回调，250Hz 的内环根本受不了。

    查不到时返回 ``(None, None)`` 而不是抛异常：位姿源短暂不可用是正常工况
    （外环会冻结校正并上报状态），不该让调用方用异常处理常规分支。
    """

    def __init__(self, node, map_frame, base_frame):
        self._node = node
        self._map_frame = map_frame
        self._base_frame = base_frame
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, node)
        self.lookup_failures = 0

    def lookup(self):
        try:
            tf = self._buffer.lookup_transform(
                self._map_frame, self._base_frame, Time())
        except TransformException:
            self.lookup_failures += 1
            return (None, None)
        t = tf.transform.translation
        r = tf.transform.rotation
        yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y),
                         1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        return ([t.x, t.y, yaw], stamp)


# ---------------------------------------------------------------------------
# 状态码映射
# ---------------------------------------------------------------------------

def _build_code_map():
    """把核心层用的**字符串**状态名映射成 BridgeStatus 的数字常量。

    为什么核心层用字符串而不直接用消息常量
    ----------------------------------
    核心层要能在**没有编译 msgs** 的环境下被单测（纯逻辑测试不该依赖 rosidl
    产物，否则测试就跑不动了）。代价是两边有一份名字要对齐 —— 所以配一个
    ``test_status_code_map`` 测试，保证核心层用到的每个名字都能映射，
    映射不上就当场失败，而不是运行期悄悄发一个错误的状态位。
    """
    out = {}
    for name in dir(BridgeStatus):
        if name.isupper() and not name.startswith('_'):
            value = getattr(BridgeStatus, name)
            if isinstance(value, int):
                out[name] = value
    return out


STATUS_CODE_MAP = _build_code_map()


class UnknownStatusCode(KeyError):
    """核心层给出的状态名在 BridgeStatus 里不存在。

    这必须是**显式失败**而不是回落到某个默认值：回落会让诊断节点收到一个
    语义错误的状态位，比收不到更糟。
    """


def status_code_of(name):
    if name not in STATUS_CODE_MAP:
        raise UnknownStatusCode(
            '状态名 %r 在 BridgeStatus.msg 里没有对应常量。'
            '新增故障类型必须同时在 msg 里加枚举 —— 不允许用 detail 字符串'
            '携带新故障类型，那样上层没法可靠地分支。' % (name,))
    return STATUS_CODE_MAP[name]


#: 会通过 StatusReporter 上报状态的核心层模块。
#: 新增核心层模块必须加进来，否则它的状态名不受启动期校验保护。
_CORE_MODULES = ('chassis_bridge_core', 'arm_bridge_core', 'gripper_core')


def core_status_names():
    """核心层运行期可能上报的状态名**全集** → 定义处。

    靠反射而不是手抄清单：每一处 ``_emit()`` / ``publish()`` 的状态名都是模块级
    ``S_*`` 常量（已逐处核对，没有任何一处是拼出来的字符串），所以反射出的集合
    就是全集，能支撑下面那条"启动期校验"的完备性。
    """
    import importlib
    out = {}
    for mod_name in _CORE_MODULES:
        mod = importlib.import_module('astribot_trajectory_bridge.' + mod_name)
        for attr in dir(mod):
            if attr.startswith('S_'):
                value = getattr(mod, attr)
                if isinstance(value, str):
                    out.setdefault(value, []).append('%s.%s' % (mod_name, attr))
    return out


def assert_all_core_names_mappable():
    """启动期一次性校验：核心层每个状态名都能映射成 msg 常量。

    !!! 这条校验存在的理由 !!!
    ────────────────────────
    实机上出过一次：``SCAN_STALE`` 没在 msg 里加枚举，于是 /scan 一陈旧，
    上报路径抛 UnknownStatusCode，异常穿过定时器回调打死整个 bridge_container，
    **底盘写通路当场消失**。也就是说，一个用来保护行车安全的联锁，
    在它触发的那一刻摧毁了它所保护的东西 —— 而在此之前跟踪得好好的
    （最后一秒：指令 0.2016m / 实际 0.2019m）。

    映射不一致本质上是**开发期**错误，就该在开发期炸；把它挪到启动期炸，
    最坏情况也只是机器人根本起不来（响亮、立刻、且此时机器人还没在动），
    而不是跑到一半突然没了写通路。
    """
    names = core_status_names()
    if not names:
        raise RuntimeError(
            '反射没找到任何核心层状态名 —— 校验本身失效了。'
            '检查 _CORE_MODULES=%r 与 S_* 命名约定。' % (_CORE_MODULES,))
    missing = {n: src for n, src in names.items() if n not in STATUS_CODE_MAP}
    if missing:
        raise UnknownStatusCode(
            '这些核心层状态名在 BridgeStatus.msg 里没有对应常量：%s。'
            '必须先在 msg 里加枚举再重建 astribot_bridge_msgs —— '
            '否则该故障一触发就会打死桥接进程。'
            % '; '.join('%s(%s)' % (n, ','.join(src))
                        for n, src in sorted(missing.items())))
    return len(names)


class StatusReporter:
    """把核心层产出的 StatusEvent 转成 BridgeStatus 发出去。

    做成独立类而不是散在节点里：底盘桥和机械臂桥共用同一个话题与同一套映射，
    重复实现会出现"两个桥的状态位含义不一致"。上报的**容错边界也只有这一处**，
    所以调用方无论从哪个回调调进来都受保护 —— 节点里有 7 处 publish 调用点，
    逐处包 try 必然漏（实机那次就是漏在 ``_inner_tick`` 里紧跟着
    ``except Exception`` 的下一行）。
    """

    def __init__(self, node, topic='/astribot/bridge/status', node_name=None):
        self._node = node
        self._name = node_name or node.get_name()
        # 启动期校验：映射不全就在这里炸，绝不留到故障触发时才炸。
        assert_all_core_names_mappable()
        self._pub = node.create_publisher(BridgeStatus, topic, 10)
        #: 上报本身失败的累计次数。非零即说明有上报被丢了，日志里有详情。
        self.publish_failures = 0

    def publish(self, code_name, detail='', metric_1=0.0, metric_2=0.0):
        """发一条状态。

        !!! 这个方法不允许把异常抛给调用方 !!!
        调用方是 250Hz 内环、外环和几个服务回调。异常从定时器回调穿出去会打死
        executor，也就是**上报故障反而消灭了写通路**（实机已发生过一次）。
        状态上报是观测手段，观测手段坏了不能拖着被观测的控制回路一起死。

        代价是失败会变安静，而这个话题存在的理由恰恰是"日志会被 SDK 的 fd 级
        重定向吞掉"。所以失败要计数（``publish_failures``）而不只是打日志。
        正常路径上这里恒不失败：名字映射已由 ``__init__`` 的启动期校验兜住。
        """
        try:
            msg = BridgeStatus()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.node_name = self._name
            msg.state = status_code_of(code_name)
            msg.detail = detail
            msg.metric_1 = float(metric_1)
            msg.metric_2 = float(metric_2)
            self._pub.publish(msg)
        except Exception as exc:      # noqa: BLE001
            self.publish_failures += 1
            self._node.get_logger().error(
                '状态上报失败（累计 %d 次），状态位 %r 已丢弃：%s'
                % (self.publish_failures, code_name, exc),
                throttle_duration_sec=5.0)

    def publish_events(self, events):
        """批量上报核心层 drain 出来的事件。

        单条失败不影响后续事件：容错边界在 :meth:`publish` 里，
        所以这里的循环不会被中间某一条打断（否则一条坏事件会吃掉同一拍
        后面所有的好事件）。
        """
        for ev in events:
            self.publish(ev.code, ev.detail, ev.metric_1, ev.metric_2)
