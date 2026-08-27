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


class StatusReporter:
    """把核心层产出的 StatusEvent 转成 BridgeStatus 发出去。

    做成独立类而不是散在节点里：底盘桥和机械臂桥共用同一个话题与同一套映射，
    重复实现会出现"两个桥的状态位含义不一致"。
    """

    def __init__(self, node, topic='/astribot/bridge/status', node_name=None):
        self._node = node
        self._name = node_name or node.get_name()
        self._pub = node.create_publisher(BridgeStatus, topic, 10)

    def publish(self, code_name, detail='', metric_1=0.0, metric_2=0.0):
        msg = BridgeStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.node_name = self._name
        msg.state = status_code_of(code_name)
        msg.detail = detail
        msg.metric_1 = float(metric_1)
        msg.metric_2 = float(metric_2)
        self._pub.publish(msg)

    def publish_events(self, events):
        """批量上报核心层 drain 出来的事件。

        单条事件映射失败**不吞**：让它抛出去，因为那意味着核心层与 msg 定义
        已经不一致，属于开发期错误，必须当场暴露。
        """
        for ev in events:
            self.publish(ev.code, ev.detail, ev.metric_1, ev.metric_2)
