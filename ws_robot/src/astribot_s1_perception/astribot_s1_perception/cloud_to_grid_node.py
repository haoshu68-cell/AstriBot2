#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 LIO 型 SLAM 的点云投影成 `nav_msgs/OccupancyGrid` 并发布 `/map`。

════════════════ 为什么必须有这一层 ════════════════
`/home/astribot/SLAM` 跑的是 **Voxel-SLAM**，实机源码扫描已确认它的接口：

    输出话题  /map_cmap /map_pmap /map_scan /map_init /map_test /map_path /map_true
              —— 全部 sensor_msgs/PointCloud2
    OccupancyGrid 引用数  **0**        ← 它根本不产生栅格图
    TF        camera_init → aft_mapped （IMUST 状态，即 map → IMU 体系）
    pub_odom_func 只发 TF，连 nav_msgs/Odometry 话题都没有

而 nav2 / 探索协调器 / `map_start_cell_check` 全都硬依赖 `OccupancyGrid` 的 `/map`。
所以 `slam_adapter_node`（做话题改名 + QoS 归一化 + 心跳）**接不上 Voxel-SLAM**，
中间必须先有这一层投影。判定与数学全在 `cloud_to_grid.py`（纯逻辑、48 项离线测试，
含真实规模的性能回归），本文件只做 ROS 粘合。

════════════════ 与 slam_adapter_node 的分工 ════════════════
    Voxel-SLAM ──PointCloud2──▶ 本节点 ──OccupancyGrid──▶ /map
                                          （已是下游要的契约）

两条路线，用 `publish_directly` 参数选：
  · true （默认）本节点直接发 `/map`，不再经 adapter —— 少一跳，少一处静默故障源
  · false        发到 `intermediate_map_topic`，由 `slam_adapter_node` 做契约校验
                 + 心跳重发。**上游长时间静止时需要它**（协调器按本地到达时间
                 判 /map 超时）

⚠️ 本节点自己也带心跳（`republish_period_sec`），所以默认路线不缺心跳。
   走 adapter 只在"想让契约校验单独成一层"时才有意义。
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from astribot_s1_perception.cloud_to_grid import GridConfig, GridConfigError, project


def latched_qos(depth=1):
    """`/map` 的 QoS。下游（nav2、协调器）按 TRANSIENT_LOCAL 订阅，不一致就零消息。"""
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def cloud_qos(depth=2):
    """点云入口用 BEST_EFFORT。

    Voxel-SLAM 用的是 `create_publisher<PointCloud2>(topic, 100)`，即默认 QoS
    （RELIABLE + VOLATILE）。我们用 BEST_EFFORT **订阅** RELIABLE 发布方是
    兼容的（订阅方要求更弱）；反过来才不兼容。depth 取 2 是刻意的：
    地图点云可能很大，堆积比丢帧更糟 —— 投影一帧要几秒，堆着只会越拖越久。
    """
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)


class CloudToGridNode(Node):
    """PointCloud2 → OccupancyGrid。数学全在 `cloud_to_grid`，本类只搬数据。"""

    def __init__(self):
        super().__init__('cloud_to_grid')

        # -- 入 --
        # Voxel-SLAM 的候选话题。/map_cmap 是"当前地图"（cmap = current map），
        # 是最合理的默认；/map_pmap 是历史/持久地图，会包含已加载的旧会话。
        # ⚠️ 上线前用 `ros2 topic hz` 逐个核对哪个真有数据、点数多少。
        self.declare_parameter('cloud_topic', '/map_cmap')
        self.declare_parameter('cloud_frame_override', '')   # 空 = 用消息自带 frame_id

        # -- 出 --
        self.declare_parameter('publish_directly', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('intermediate_map_topic', '/slam/map')
        self.declare_parameter('map_frame', 'map')

        # -- 投影参数（逐项对应 GridConfig）--
        self.declare_parameter('resolution', 0.05)
        # z 切片：默认 [-0.05, 0.60]。下界略低于地面是刻意的 —— 地面在
        # z≈-0.095（相对 astribot_torso_base），但点云的 z 原点取决于 SLAM 的
        # camera_init，两者不是同一个基准，所以这个区间**必须在实机上核对**。
        self.declare_parameter('z_min', -0.05)
        self.declare_parameter('z_max', 0.60)
        self.declare_parameter('min_points_per_cell', 2)
        self.declare_parameter('padding_m', 1.0)
        self.declare_parameter('max_cells', 4_000_000)

        # -- 空闲空间雕刻 --
        # 不雕的话整张图只有"占据"和"未知"，没有"空闲"，nav2 无法规划，
        # 而探索协调器会把每个前沿候选都判为不可站（前沿必须在自由空间边上）。
        self.declare_parameter('carve_free_space', True)
        self.declare_parameter('carve_max_range_m', 8.0)
        self.declare_parameter('carve_n_rays', 360)
        # 传感器位置从 TF 取；取不到就退化为不雕并 WARN（不静默）
        self.declare_parameter('sensor_frame', 'aft_mapped')
        self.declare_parameter('tf_timeout_sec', 0.5)

        # -- 行为 --
        self.declare_parameter('republish_period_sec', 5.0)
        self.declare_parameter('source_timeout_sec', 60.0)
        self.declare_parameter('report_period_sec', 10.0)
        # 投影一帧大图要秒级。这个上限防止回调堆积把节点拖死。
        self.declare_parameter('min_interval_sec', 2.0)

        self.cloud_topic = str(self.get_parameter('cloud_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.sensor_frame = str(self.get_parameter('sensor_frame').value)
        self.do_carve = bool(self.get_parameter('carve_free_space').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self.min_interval = float(self.get_parameter('min_interval_sec').value)
        self.source_timeout = float(self.get_parameter('source_timeout_sec').value)

        # GridConfig 自己会在构造期校验并抛 GridConfigError（由 main 兜住并以
        # 退出码 2 结束）。这里刻意不 try —— 捕获后原样 raise 没有任何作用，
        # 只会让读代码的人以为这里做了额外处理。
        self.config = GridConfig(
            resolution=float(self.get_parameter('resolution').value),
            z_min=float(self.get_parameter('z_min').value),
            z_max=float(self.get_parameter('z_max').value),
            min_points_per_cell=int(self.get_parameter('min_points_per_cell').value),
            padding_m=float(self.get_parameter('padding_m').value),
            max_cells=int(self.get_parameter('max_cells').value),
            carve_max_range_m=float(self.get_parameter('carve_max_range_m').value),
            carve_n_rays=int(self.get_parameter('carve_n_rays').value))

        topic = (str(self.get_parameter('map_topic').value)
                 if bool(self.get_parameter('publish_directly').value)
                 else str(self.get_parameter('intermediate_map_topic').value))
        self.map_pub = self.create_publisher(OccupancyGrid, topic, latched_qos())
        self.out_topic = topic

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PointCloud2, self.cloud_topic, self._on_cloud, cloud_qos())

        self.exit_code = 0
        self._last_grid = None
        self._last_stamp = None
        self._last_project_sec = None
        self._start_sec = self._now()
        self._stats = {'received': 0, 'projected': 0, 'skipped': 0,
                       'carve_failed': 0, 'published': 0}

        period = float(self.get_parameter('republish_period_sec').value)
        if period > 0.0:
            self.create_timer(0.5, self._tick_heartbeat)
        self._republish_period = period
        self.create_timer(2.0, self._tick_watchdog)
        report = float(self.get_parameter('report_period_sec').value)
        if report > 0.0:
            self.create_timer(report, self._tick_report)

        self.get_logger().info(
            f'cloud_to_grid 启动：\n'
            f'  入 {self.cloud_topic} (BEST_EFFORT)\n'
            f'  出 {self.out_topic} (transient_local+reliable) frame={self.map_frame}\n'
            f'  分辨率 {self.config.resolution}  z 切片 '
            f'[{self.config.z_min}, {self.config.z_max}]\n'
            f'  雕刻空闲={"开" if self.do_carve else "关"} '
            f'(传感器 frame={self.sensor_frame}, 半径 {self.config.carve_max_range_m}m)\n'
            f'  最小投影间隔 {self.min_interval}s  心跳 {period}s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- 点云回调 ---------------------------------------------------------
    def _on_cloud(self, msg):
        self._stats['received'] += 1
        now = self._now()

        # 限流。投影一帧真实规模的图要秒级（实测 4.84s / 3367 位姿），
        # 不限流的话回调会堆积，越拖越久最后完全跟不上。
        if (self._last_project_sec is not None
                and now - self._last_project_sec < self.min_interval):
            self._stats['skipped'] += 1
            return

        points = self._read_points(msg)
        if not points:
            self.get_logger().warning(
                f'{self.cloud_topic} 收到 {msg.width}x{msg.height} 的点云但解析出 0 个点。'
                f'字段: {[f.name for f in msg.fields]}。'
                f'Livox 的 PointCloud2 是 PointXYZRTL 布局，'
                f'必须含 x/y/z 三个 FLOAT32 字段。')
            return

        sensor_xy = self._lookup_sensor_xy(msg.header) if self.do_carve else None
        try:
            grid = project(points, self.config, sensor_xy=sensor_xy)
        except GridConfigError as exc:
            # 通常是 max_cells 被撑爆：点云范围远超预期
            self.get_logger().error(
                f'投影失败: {exc}\n'
                f'  点数={len(points)}。若是 max_cells 超限，说明点云范围远超预期 ——'
                f'先确认 z 切片是否把天花板/地面也算进来了。')
            return

        self._last_project_sec = self._now()
        self._stats['projected'] += 1
        self._last_grid = grid
        self._last_stamp = msg.header.stamp
        self._publish(grid, msg.header.stamp)

        self.get_logger().info(
            f'投影完成 {grid.width}x{grid.height} '
            f'占据={grid.occupied_cells} 空闲={grid.free_cells} '
            f'未知={grid.unknown_cells} '
            f'用点={grid.points_used} 切片外={grid.points_out_of_slab} '
            f'耗时={self._last_project_sec - now:.2f}s')

        # 全是未知/占据、一个空闲格都没有 —— 这种图 nav2 无法规划，
        # 而症状会表现成 "Starting point in lethal space" 之类，离根因很远。
        if grid.free_cells == 0:
            self.get_logger().warning(
                '空闲格为 0：nav2 无法规划，探索会把每个前沿都判为不可站。'
                + ('  传感器位姿取不到，雕刻被跳过 —— 先修 TF。'
                   if sensor_xy is None
                   else f'  已雕刻但仍为 0，检查 z 切片 '
                        f'[{self.config.z_min}, {self.config.z_max}] 是否覆盖了地面。'))

    def _read_points(self, msg):
        """从 PointCloud2 取 (x, y, z)。

        用 `read_points` 而不是手工解 buffer：Livox 的布局是 PointXYZRTL
        （多出 reflectivity/tag/line 三个字段），手工按 x/y/z 连续假设去解会错位。
        """
        try:
            return [(float(p[0]), float(p[1]), float(p[2]))
                    for p in pc2.read_points(
                        msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        except Exception as exc:      # noqa: BLE001
            self.get_logger().error(
                f'解析点云失败: {exc}。字段: {[f.name for f in msg.fields]}')
            return []

    def _lookup_sensor_xy(self, header):
        """从 TF 取传感器在地图系的位置，供雕刻用。

        取不到就返回 None → 不雕刻 + WARN。**不静默**：没有雕刻的图没有空闲格，
        而那个后果（nav2 不能规划）离"TF 查不到"这个原因隔了好几层。
        """
        target = self.map_frame
        source = self.sensor_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, header.stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            t = tf.transform.translation
            return [(float(t.x), float(t.y))]
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self._stats['carve_failed'] += 1
            # 只在前几次和每 20 次报一次，避免刷屏掩盖别的信息
            if self._stats['carve_failed'] <= 3 or self._stats['carve_failed'] % 20 == 0:
                self.get_logger().warning(
                    f'取不到 {target}→{source} 的变换（第 '
                    f'{self._stats["carve_failed"]} 次），本帧不雕刻空闲空间：{exc}\n'
                    f'  后果：地图只有"占据"和"未知"，没有"空闲" → nav2 无法规划。\n'
                    f'  Voxel-SLAM 发的是 camera_init→aft_mapped，'
                    f'若 map_frame 不是 camera_init，需要一条 '
                    f'{target}→camera_init 的静态 TF 把两者接起来。')
            return None

    # -- 发布 -------------------------------------------------------------
    def _publish(self, grid, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info.resolution = grid.resolution
        msg.info.width = grid.width
        msg.info.height = grid.height
        msg.info.origin.position.x = grid.origin_x
        msg.info.origin.position.y = grid.origin_y
        # origin 的朝向必须是单位四元数：下游所有 world↔cell 换算都假定轴对齐
        # （`cloud_to_grid.world_to_cell` 就是纯平移）。带旋转不报错，只让路径偏移。
        msg.info.origin.orientation.w = 1.0
        msg.data = list(grid.data)
        self.map_pub.publish(msg)
        self._stats['published'] += 1

    # -- 定时器 -----------------------------------------------------------
    def _tick_heartbeat(self):
        """按周期重发最后一帧。

        协调器按**本地到达时间**判 /map 超时（不看 header.stamp）。场景静止时
        Voxel-SLAM 可能长时间不发新点云 —— 那是它的正常行为，但在协调器看来
        就是"地图断了"。stamp 刻意保留原值，不改成"现在"：改了会掩盖上游停更。
        """
        if self._last_grid is None or self._republish_period <= 0.0:
            return
        if self._last_project_sec is None:
            return
        if self._now() - self._last_project_sec < self._republish_period:
            return
        # 借用 _last_project_sec 做心跳计时：重发也算一次"发出"
        self._last_project_sec = self._now()
        self._publish(self._last_grid, self._last_stamp)

    def _tick_watchdog(self):
        """源超时就非零退出，不静默等待。

        静默等待会让 nav2 一直等 /map，症状是"导航卡在启动"，而原因在这一层。
        """
        if self._last_grid is not None:
            return
        if self._now() - self._start_sec < self.source_timeout:
            return
        self.get_logger().error(
            f'{self.source_timeout}s 内没有成功投影任何一帧，退出。\n'
            f'  收到点云 {self._stats["received"]} 帧。\n'
            f'  依次查：\n'
            f'  ① Voxel-SLAM 是否在跑（ros2 node list | grep -i voxel）\n'
            f'  ② 话题名是否真是 {self.cloud_topic}'
            f'（候选：/map_cmap /map_pmap /map_scan /map_true）\n'
            f'  ③ **Voxel-SLAM 现在很可能收不到雷达数据**：config/mid360.yaml 里\n'
            f'     lidar_type=0(LIVOX) 意味着它订阅 livox_ros_driver2/CustomMsg，\n'
            f'     而厂商驱动 xfer_format=0 发的是 PointCloud2 —— 类型不匹配，零数据。\n'
            f'  ④ ROS_DOMAIN_ID 是否一致（实机是 25，不是 42）')
        self.exit_code = 1
        raise SystemExit(1)

    def _tick_report(self):
        s = self._stats
        self.get_logger().info(
            f'收={s["received"]} 投影={s["projected"]} 限流跳过={s["skipped"]} '
            f'发布={s["published"]} 雕刻失败={s["carve_failed"]}')


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = CloudToGridNode()
        rclpy.spin(node)
    except GridConfigError as exc:
        print(f'[cloud_to_grid] 投影配置被拒绝：{exc}', file=sys.stderr)
        code = 2
    except SystemExit as exc:
        code = int(exc.code or 0)
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
