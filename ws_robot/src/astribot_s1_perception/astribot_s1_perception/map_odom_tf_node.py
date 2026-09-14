#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""发布 `map→odom`：把外部 SLAM 的全局位姿与 SDK 里程计做 REP-105 分解。

════════════════ 为什么单独一个节点 ════════════════
`map→odom` 既不属于"地图中继"也不属于"里程计发布"，它是两者的**函数**：

    SLAM        camera_init → aft_mapped      （全局，带 GBA 修正会跳）
    SDK 里程计   odom → astribot_torso_base    （局部连续，允许漂不允许跳）
    本节点       map → odom = (map→base) ∘ (odom→base)⁻¹

塞进任何一边都会让那一边在对方没起来时发出错的变换。而 `slam_adapter_node`
之前就是这么错的：它在拿不到 odom 时发**单位变换**，等于宣称"odom 原点就是
map 原点" —— 只在开机即建图那一种情形下成立，不成立时地图与激光整体错位，
且**不会有任何报错**。那段已经删掉，改由本节点负责。

════════════════ "以 aft_mapped 为准"是本节点的前提 ════════════════
分解成立的必要条件是：`aft_mapped` 与 `astribot_torso_base` 指同一个物理点
（底盘中心）。这是**决策给定的**，不是本节点能验证的。

若这个前提不成立（两个"底盘中心"差一个固定偏移），后果是地图与机器人
系统性偏移一个常量 —— 症状看起来像"定位有固定误差"。本节点会在启动日志里
把这条前提写出来，好让排查时第一眼就能怀疑到它。

════════════════ 跳变是正常的 ════════════════
回环修正**应该**体现在 `map→odom` 上 —— 这正是 REP-105 分解的目的：
让跳变留在 map→odom，odom→base 保持连续（local_costmap 用 odom，
它不能瞬移）。所以本节点检测到跳变时**上报但照常发布**，绝不平滑掉。
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformListener,
)

from astribot_s1_perception.map_odom_decompose import (
    DEFAULT_MAX_SOURCE_AGE_SEC,
    DecompositionError,
    MapOdomDecomposer,
    Pose2D,
    check_source_age,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


class MapOdomTfNode(Node):
    """数学全在 `map_odom_decompose`（纯逻辑、25 项离线测试含 400 组随机穷举）。"""

    def __init__(self):
        super().__init__('map_odom_tf')

        self.declare_parameter('slam_world_frame', 'camera_init')
        self.declare_parameter('slam_base_frame', 'aft_mapped')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'astribot_torso_base')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('jump_report_m', 0.30)
        self.declare_parameter('max_tilt_rad', 0.10)
        self.declare_parameter('publish_map_to_slam_world', True)
        self.declare_parameter('source_timeout_sec', 60.0)
        self.declare_parameter('report_period_sec', 10.0)
        self.declare_parameter('max_source_age_sec', DEFAULT_MAX_SOURCE_AGE_SEC)

        self.slam_world = str(self.get_parameter('slam_world_frame').value)
        self.slam_base = str(self.get_parameter('slam_base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self.source_timeout = float(self.get_parameter('source_timeout_sec').value)
        self.max_source_age = float(self.get_parameter('max_source_age_sec').value)

        rate = float(self.get_parameter('publish_rate').value)
        if not rate > 0.0:
            raise DecompositionError(f'publish_rate={rate} 必须为正')

        self.decomposer = MapOdomDecomposer(
            jump_report_m=float(self.get_parameter('jump_report_m').value),
            max_tilt_rad=float(self.get_parameter('max_tilt_rad').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self._miss = {'slam': 0, 'odom': 0}
        self._stale = {'slam': 0, 'odom': 0}
        self.exit_code = 0
        self._start_sec = self._now()

        if bool(self.get_parameter('publish_map_to_slam_world').value):
            self._send_map_to_slam_world()

        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(2.0, self._tick_watchdog)
        report = float(self.get_parameter('report_period_sec').value)
        if report > 0.0:
            self.create_timer(report, self._tick_report)

        self.get_logger().info(
            f'map_odom_tf 启动：\n'
            f'  SLAM 侧  {self.slam_world} → {self.slam_base}\n'
            f'  我们这侧  {self.odom_frame} → {self.base_frame}\n'
            f'  发出      {self.map_frame} → {self.odom_frame}'
            f'  ({rate:.0f}Hz)\n'
            f'  ⚠️ 前提（决策给定，本节点无法验证）：'
            f'{self.slam_base} 与 {self.base_frame} 指同一个物理点。\n'
            f'     不成立时地图与机器人会系统性偏移一个常量，'
            f'症状像"定位有固定误差"。')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        slam = self._lookup(self.slam_world, self.slam_base, 'slam')
        if slam is None:
            return
        odom = self._lookup(self.odom_frame, self.base_frame, 'odom')
        if odom is None:
            return

        map_to_odom, jumped, jump_m = self.decomposer.update(slam, odom)
        stamp = self.get_clock().now().to_msg()

        self._send(map_to_odom, stamp)

        if jumped:
            self.get_logger().info(
                f'map→odom 跳变 {jump_m:.3f}m（累计 '
                f'{self.decomposer.stats.jumps} 次，最大 '
                f'{self.decomposer.stats.max_jump_m:.3f}m）。\n'
                f'  这**不是故障**：SLAM 回环修正本来就该落在 map→odom 上，'
                f'这正是 REP-105 分解的目的（跳变留给 map→odom，'
                f'odom→base 保持连续，因为 local_costmap 用 odom 不能瞬移）。\n'
                f'  但 global_costmap 会整体平移，若此刻正在导航，'
                f'全局路径会被重规划。')

    def _lookup(self, target, source, kind):
        """查一条变换并转成 Pose2D。失败返回 None 并按需 WARN。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self._miss[kind] += 1
            n = self._miss[kind]
            if n <= 3 or n % 100 == 0:
                self.get_logger().warning(
                    f'取不到 {target}→{source}（第 {n} 次）：{exc}\n'
                    + (f'  → 外部 SLAM 没在发 TF。确认 Voxel-SLAM 进程在跑，'
                       f'且它真的收到了雷达数据'
                       f'（mid360.yaml 的 lidar_type=0 要 CustomMsg，'
                       f'而厂商驱动 xfer_format=0 发 PointCloud2 —— 类型不匹配就是零数据）。'
                       if kind == 'slam' else
                       f'  → chassis_odom_node 没在发。它需要厂商本体运动服务已启动，'
                       f'否则 SDK 报 "No simulation or real robot is started" 并非零退出。'))
            return None

        stamp = tf.header.stamp
        age = check_source_age(
            self._now(), stamp.sec + stamp.nanosec * 1e-9,
            self.max_source_age, f'{target}→{source}')
        if age.stale:
            self._stale[kind] += 1
            n = self._stale[kind]
            if n <= 3 or n % 100 == 0:
                self.get_logger().warning(f'（第 {n} 次陈旧）{age.reason}')
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        tilt = self.decomposer.check_planar(q.x, q.y, f'{target}→{source}')
        if tilt is not None:
            n = self.decomposer.stats.rejected_tilt
            if n <= 3 or n % 100 == 0:
                self.get_logger().warning(tilt)
        try:
            return Pose2D(x=float(t.x), y=float(t.y),
                          theta=yaw_from_quaternion(q.z, q.w))
        except DecompositionError as exc:
            self.get_logger().error(f'{target}→{source} 的数值非法：{exc}')
            return None

    def _send(self, pose, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = pose.x
        tf.transform.translation.y = pose.y
        z, w = quaternion_from_yaw(pose.theta)
        tf.transform.rotation.z = z
        tf.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(tf)

    def _send_map_to_slam_world(self):
        """`map → camera_init` 恒等变换，把两套命名接起来。

        走 **static** 广播器（`/tf_static`，TRANSIENT_LOCAL 即 latched）：
        这条边永远不变，且必须让**晚启动的**消费者也能拿到。
        早先这里用动态广播器且只发一次，后果是 nav2 / rviz / cloud_to_grid
        只要启动得比这一刻晚，就永远看不到这条边。

        仍然由本节点独占这条边（不用 static_transform_publisher），
        以免出现"静态和动态两个源"。恒等意味着 map 与 camera_init 数值上
        同一个系，只是名字不同 —— camera_init 的原点是**底盘启动位姿**，
        所以这等于宣称"开机点即地图原点"。
        """
        if self.map_frame == self.slam_world:
            return          # 同名就不用接
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.slam_world
        tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf)
        self.get_logger().info(
            f'已发 {self.map_frame}→{self.slam_world} 恒等静态变换。'
            f'含义：开机点即地图原点（{self.slam_world} 的原点是底盘启动位姿）。\n'
            f'  这条边**不等**里程计就绪就发 —— /map 的 frame_id 是 '
            f'{self.slam_world}，缺这条边时以 {self.map_frame} 为全局系的'
            f'消费者会整个显示不出地图。')

    def _tick_watchdog(self):
        if self.decomposer.stats.updates > 0:
            return
        if self._now() - self._start_sec < self.source_timeout:
            return
        self.get_logger().error(
            f'{self.source_timeout}s 内一次都没算出 map→odom，退出。\n'
            f'  缺失次数：SLAM 侧 {self._miss["slam"]}，'
            f'里程计侧 {self._miss["odom"]}。\n'
            f'  陈旧次数：SLAM 侧 {self._stale["slam"]}，'
            f'里程计侧 {self._stale["odom"]}。\n'
            f'  **先分清是哪一类**：缺失=那条边根本没有；'
            f'陈旧=有但已停更（发布进程死了或卡住），查的地方完全不同。\n'
            f'  不静默等待是刻意的：静默等待会让 nav2 一直等 TF，'
            f'症状是"导航卡在启动"而原因在这一层。')
        self.exit_code = 1
        raise SystemExit(1)

    def _tick_report(self):
        s = self.decomposer.stats
        last = self.decomposer.last
        pose = (f'({last.x:.3f}, {last.y:.3f}, {last.theta:.3f})'
                if last is not None else '(尚无)')
        self.get_logger().info(
            f'map→odom={pose} 更新={s.updates} 跳变={s.jumps}'
            f'(最大 {s.max_jump_m:.3f}m) 倾角超限={s.rejected_tilt} '
            f'缺失 slam={self._miss["slam"]} odom={self._miss["odom"]} '
            f'陈旧 slam={self._stale["slam"]} odom={self._stale["odom"]}')


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = MapOdomTfNode()
        rclpy.spin(node)
    except DecompositionError as exc:
        print(f'[map_odom_tf] 参数被拒绝：{exc}', file=sys.stderr)
        code = 2
    except SystemExit as exc:
        code = int(exc.code or 0)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            code = code or node.exit_code
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
