#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`map_odom_tf_node` 的**接线**测试 —— 不只测纯函数。

════════════════ 为什么必须单独测接线 ════════════════
本仓库有过一次教训：底盘 dt 缺陷的纯函数测试 45 条全绿，但把节点里的
`inner_tick` 改回用标称 dt，**全部测试照样通过** —— 因为没有一条测试
真的驱动过节点里那条路径。纯函数正确 ≠ 节点用了它。

所以这一组直接构造真节点、喂假 TF 缓冲，验证：
  A2：陈旧的源变换必须让本节点**停止发布** map→odom（宁可断，不发错的）
  A3：map→camera_init 必须在**构造时**就发、走**静态**广播器
      （原缺陷：挂在 _tick 里且只发一次动态 TF，里程计没起时这条边根本不存在，
        而晚启动的消费者永远收不到）
"""

import pytest

# !!! 不用模块级 pytest.importorskip !!!
# pytest 6.2.5 下模块级 importorskip 会让**整个 collection** 被打断：
# 实测本目录在 rclpy 不可用时，`pytest test/` 只报 "1 skipped, 1 error"，
# 另外 197 条测试静默消失 —— 一个"干净的 1 skipped"比报错更危险。
# 正确写法是 try/except + pytestmark，跳过范围限制在本文件内。
try:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from astribot_s1_perception import map_odom_tf_node as mod
    _ROS_AVAILABLE = True
except ImportError:                                  # pragma: no cover
    rclpy = None
    TransformStamped = None
    mod = None
    _ROS_AVAILABLE = False

pytestmark = pytest.mark.skipif(
    not _ROS_AVAILABLE,
    reason='需要 rclpy / tf2_ros / geometry_msgs（本文件测的是节点接线）')


class Recorder:
    """冒充广播器，只记不发。"""

    def __init__(self, *_a, **_kw):
        self.sent = []

    def sendTransform(self, tf):        # noqa: N802  (tf2 的驼峰接口)
        self.sent.append(tf)


class FakeBuffer:
    """冒充 tf2 Buffer。`stamps` 决定每条边返回的戳，None 表示抛缺失异常。"""

    def __init__(self):
        self.stamps = {}
        self.pose = (0.0, 0.0, 0.0)     # x, y, yaw 的 (z, w) 用恒等
        self.calls = []

    def lookup_transform(self, target, source, _time, timeout=None):
        key = (target, source)
        self.calls.append(key)
        if key not in self.stamps:
            raise mod.LookupException(f'假缓冲里没有 {target}→{source}')
        tf = TransformStamped()
        sec = self.stamps[key]
        tf.header.stamp.sec = int(sec)
        tf.header.stamp.nanosec = int(round((sec - int(sec)) * 1e9))
        tf.header.frame_id = target
        tf.child_frame_id = source
        tf.transform.translation.x = self.pose[0]
        tf.transform.translation.y = self.pose[1]
        tf.transform.rotation.w = 1.0
        return tf


NOW = 1000.0


@pytest.fixture
def node(monkeypatch):
    """真节点 + 假缓冲 + 假广播器 + 冻结时钟。"""
    monkeypatch.setattr(mod, 'StaticTransformBroadcaster', Recorder)
    monkeypatch.setattr(mod, 'TransformBroadcaster', Recorder)
    monkeypatch.setattr(mod, 'TransformListener', lambda *a, **k: None)
    monkeypatch.setattr(mod, 'Buffer', FakeBuffer)

    if not rclpy.ok():
        rclpy.init()
    n = mod.MapOdomTfNode()
    monkeypatch.setattr(n, '_now', lambda: NOW)
    yield n
    n.destroy_node()


def feed(n, slam_age, odom_age):
    n.tf_buffer.stamps = {
        ('camera_init', 'aft_mapped'): NOW - slam_age,
        ('odom', 'astribot_torso_base'): NOW - odom_age,
    }


# ── A3：恒等静态边 ─────────────────────────────────────────────────────
class TestMapToSlamWorldEdge:

    def test_sent_at_construction_before_any_tick(self, node):
        # 关键回归：原来这条边要等第一次 _tick 里 slam+odom 都取到才发。
        assert len(node.static_broadcaster.sent) == 1

    def test_sent_on_the_static_broadcaster_not_the_dynamic_one(self, node):
        # /tf 不是 latched，只发一次等于晚启动的消费者永远收不到。
        assert node.static_broadcaster.sent, 'map→camera_init 没走静态广播器'
        assert node.tf_broadcaster.sent == [], '恒等边不该出现在动态 /tf 上'

    def test_edge_direction_and_identity(self, node):
        tf = node.static_broadcaster.sent[0]
        assert tf.header.frame_id == 'map'
        assert tf.child_frame_id == 'camera_init'
        assert tf.transform.translation.x == 0.0
        assert tf.transform.translation.y == 0.0
        assert tf.transform.rotation.w == 1.0

    def test_survives_odom_never_arriving(self, node):
        # /map 的 frame_id 是 camera_init。里程计没起时这条边仍必须在，
        # 否则以 map 为全局系的消费者整个显示不出地图。
        node.tf_buffer.stamps = {('camera_init', 'aft_mapped'): NOW}
        for _ in range(5):
            node._tick()
        assert len(node.static_broadcaster.sent) == 1
        assert node.tf_broadcaster.sent == []


# ── A2：陈旧源必须停发 ────────────────────────────────────────────────
class TestStaleSourceStopsPublishing:

    def test_fresh_sources_publish(self, node):
        feed(node, 0.0, 0.0)
        node._tick()
        assert len(node.tf_broadcaster.sent) == 1
        tf = node.tf_broadcaster.sent[0]
        assert (tf.header.frame_id, tf.child_frame_id) == ('map', 'odom')

    def test_stale_slam_does_not_publish(self, node):
        feed(node, slam_age=30.0, odom_age=0.0)
        node._tick()
        assert node.tf_broadcaster.sent == [], \
            'SLAM 冻结时仍在发 map→odom —— 这正是那个会静默错定位的缺陷'
        assert node._stale['slam'] == 1

    def test_stale_odom_does_not_publish(self, node):
        feed(node, slam_age=0.0, odom_age=30.0)
        node._tick()
        assert node.tf_broadcaster.sent == []
        assert node._stale['odom'] == 1

    def test_the_dangerous_case_slam_frozen_while_odom_moves(self, node):
        """SLAM 冻结 + 里程计在动 = 反向漂移的 map→odom。必须一条都不发。"""
        for i in range(10):
            node.tf_buffer.pose = (0.1 * i, 0.0, 0.0)   # 里程计一直在动
            feed(node, slam_age=5.0, odom_age=0.0)      # SLAM 戳不再前进
            node._tick()
        assert node.tf_broadcaster.sent == []
        assert node.decomposer.stats.updates == 0

    def test_recovers_when_source_becomes_fresh_again(self, node):
        feed(node, slam_age=30.0, odom_age=0.0)
        node._tick()
        assert node.tf_broadcaster.sent == []
        feed(node, slam_age=0.0, odom_age=0.0)
        node._tick()
        assert len(node.tf_broadcaster.sent) == 1, '恢复后必须继续发，不能一坏永坏'

    def test_slam_checked_before_odom_is_looked_up(self, node):
        # SLAM 陈旧时不该再去查里程计 —— 省一次查询，也让日志只指一个方向。
        feed(node, slam_age=30.0, odom_age=0.0)
        node._tick()
        assert ('odom', 'astribot_torso_base') not in node.tf_buffer.calls

    def test_missing_and_stale_are_counted_separately(self, node):
        # 缺失=边不存在；陈旧=边有但停更。查的地方完全不同，混在一起会白查一轮。
        node.tf_buffer.stamps = {}
        node._tick()
        assert node._miss['slam'] == 1 and node._stale['slam'] == 0
        feed(node, slam_age=30.0, odom_age=0.0)
        node._tick()
        assert node._miss['slam'] == 1 and node._stale['slam'] == 1

    def test_age_check_can_be_disabled(self, node):
        node.max_source_age = 0.0
        feed(node, slam_age=999.0, odom_age=999.0)
        node._tick()
        assert len(node.tf_broadcaster.sent) == 1
