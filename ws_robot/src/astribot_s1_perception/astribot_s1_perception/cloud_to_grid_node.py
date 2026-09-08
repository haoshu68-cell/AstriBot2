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
from rclpy.executors import ExternalShutdownException
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
        # Voxel-SLAM 的 /map_scan_filtered 是**给导航用的实时障碍点云**：
        #   · 已在 SLAM 侧做过高度 ROI（nav_scan_z_min=0.05 / max=1.63，
        #     见 voxelslam.cpp:63 与 mid360.yaml:26-27）
        #   · frame_id = camera_init，坐标已是**世界系**
        #   · 点类型 pcl::PointXYZINormal 经 pcl::toROSMsg 直转，
        #     除 x/y/z 外的 intensity/normal_*/curvature **均未赋值**，忽略
        #
        # ⚠️ 不要用 /map_cmap：那是全量地图（含 previous_map 加载的旧会话），
        #    不是实时障碍，而且**没有**做高度过滤。
        self.declare_parameter('cloud_topic', '/map_scan_filtered')
        self.declare_parameter('cloud_frame_override', '')   # 空 = 用消息自带 frame_id

        # -- 出 --
        self.declare_parameter('publish_directly', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('intermediate_map_topic', '/slam/map')
        # camera_init 是 Voxel-SLAM 的世界系（原点 = 底盘启动位姿），
        # 而 aft_mapped 是**底盘中心**（SLAM 内部已用 chassis_extrinsic_* 从
        # IMU 位姿换算过，见 voxelslam.cpp:38-42 + mid360.yaml:19-20）。
        # 默认直接用 camera_init 当 map，省掉一条恒等静态 TF；
        # 若上层要求 frame 名必须叫 map，就把这里设成 map 并补一条
        # map→camera_init 的静态 TF —— 但**不能只改这里**，否则查不到变换。
        self.declare_parameter('map_frame', 'camera_init')

        # -- 投影参数（逐项对应 GridConfig）--
        self.declare_parameter('resolution', 0.05)
        # ⚠️⚠️ z 切片默认**放通**（-inf, +inf），刻意如此。
        #
        # /map_scan_filtered 已经在 SLAM 侧按 [0.05, 1.63] 过滤过。我们再切一刀
        # 就是**两层过滤串联取交集**：本仓库踩过同形态的坑（两个包各一层限速，
        # 0.50×0.15=0.075，只改一个看不到效果）。
        #
        # 具体危害：若这里保留旧默认 [-0.05, 0.60]，交集变成 [0.05, 0.60]，
        # **0.60~1.63m 的障碍全部被静默丢掉** —— 那正是人体躯干、桌面、
        # 台面高度的障碍。地图上看不出来，机器人会直接撞过去。
        #
        # 什么时候才该收紧：只有当你订阅的是**未过滤**的话题（如 /map_scan），
        # 或者需要比 SLAM 更严的区间时。改之前先确认 SLAM 侧的 nav_scan_z_* 值。
        self.declare_parameter('z_min', float('-inf'))
        self.declare_parameter('z_max', float('inf'))
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
        self._warn_if_double_filtering()

    def _warn_if_double_filtering(self):
        """本节点的 z 切片若比上游更严，响亮警告。

        为什么单列一个方法：**串联过滤取交集是静默的**。上游
        /map_scan_filtered 已按 [nav_scan_z_min, nav_scan_z_max] = [0.05, 1.63]
        过滤，我们再切一刀，交集会悄悄砍掉一段高度 —— 地图上看不出来，
        代价是漏掉那一段的障碍。本仓库踩过同形态的坑（两个包各一层限速，
        0.50×0.15=0.075，只改一个看不到效果，根因在另一个包里）。
        """
        # 上游的过滤区间（Voxel-SLAM 的默认值，见 voxelslam.cpp:902 / mid360.yaml:26-27）
        upstream_min, upstream_max = 0.05, 1.63
        if 'scan_filtered' not in self.cloud_topic:
            return          # 订阅的不是已过滤话题，我们自己切是应该的
        problems = []
        if self.config.z_min > upstream_min:
            problems.append(
                f'z_min={self.config.z_min} > 上游 {upstream_min}')
        if self.config.z_max < upstream_max:
            problems.append(
                f'z_max={self.config.z_max} < 上游 {upstream_max}，'
                f'会丢掉 {self.config.z_max}~{upstream_max}m 的障碍'
                f'（人体躯干/桌面/台面高度）')
        if problems:
            self.get_logger().warning(
                f'⚠️ 双层高度过滤：{self.cloud_topic} 已在 SLAM 侧按 '
                f'[{upstream_min}, {upstream_max}] 过滤过，而本节点又切了 '
                f'[{self.config.z_min}, {self.config.z_max}]。\n'
                + '\n'.join(f'  · {p}' for p in problems)
                + '\n  两层串联取交集，**丢掉的障碍在地图上看不出来**。'
                  '确认这是你要的；否则把 z_min/z_max 设为 -inf/inf 放通。')

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
                f'上游是 pcl::PointXYZINormal 经 toROSMsg 直转，'
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
        """从 PointCloud2 取 (x, y, z)。坐标**已是世界系**(camera_init),无需变换。

        上游是 `pcl::PointXYZINormal` 经 `pcl::toROSMsg` 直转，字段顺序为
        x/y/z/intensity/normal_x/normal_y/curvature —— 除 x/y/z 外**均未赋值**。
        所以必须用 `field_names=('x','y','z')` 按名字取，不能按偏移假设连续。

        `is_dense` 上游未显式设置，因此 `skip_nans=True` 是必需的：
        NaN 会让后面的 min/max 全变 NaN，症状是"地图尺寸算出来是 0 或天文数字"。
        """
        try:
            return [(float(p[0]), float(p[1]), float(p[2]))
                    for p in pc2.read_points(
                        msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        except Exception as exc:      # noqa: BLE001
            self.get_logger().error(
                f'解析点云失败: {exc}\n'
                f'  实际字段: {[f.name for f in msg.fields]}\n'
                f'  期望含 x/y/z（FLOAT32）。上游是 PointXYZINormal，'
                f'字段应为 x/y/z/intensity/normal_x/normal_y/normal_z/curvature。')
            return []

    def _lookup_sensor_xy(self, header):
        """从 TF 取传感器在地图系的位置，供雕刻用。

        取不到就返回 None → 不雕刻 + WARN。**不静默**：没有雕刻的图没有空闲格，
        而那个后果（nav2 不能规划）离"TF 查不到"这个原因隔了好几层。

        ⚠️ 用 `Time()`（= 最新可用）而不是 `header.stamp`，这是实机实测逼出来的：
        Voxel-SLAM 的 TF stamp 取 `rclcpp::Clock().now()`（**发布时刻**，
        见 voxelslam.cpp:26），而点云 stamp 略晚于它。拿点云 stamp 去查 TF
        就成了"查未来"，实测差 **0.0003 秒**就抛
        `Lookup would require extrapolation into the future`。

        而 `timeout` 对这种情况**无效** —— 它只能等"还没到的数据"，
        不能等"已经过去但被判为需要外推"的时刻。所以加大 timeout 没有用，
        必须改查询时刻。

        用最新变换的代价：雕刻用的位姿与点云可能差最多一个 TF 周期（50ms）。
        底盘速度上限 0.5m/s 时误差 ≤2.5cm，小于一个栅格（5cm），可接受。
        """
        target = self.map_frame
        source = self.sensor_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
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
                    f'  后果：地图只有"占据"和"未知"，没有"空闲" → nav2 无法规划，'
                    f'探索会把每个前沿都判为不可站。\n'
                    f'  Voxel-SLAM 只发 camera_init→aft_mapped 这一条边'
                    f'（aft_mapped 是**底盘中心**，不是 IMU 系）。\n'
                    f'  · map_frame={target} 若不是 camera_init，需要一条 '
                    f'{target}→camera_init 的静态 TF，光改 map_frame 不够。\n'
                    f'  · 也确认 SLAM 真的在发 TF：'
                    f'ros2 topic echo /tf --once | grep -A2 aft_mapped')
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

    def _describe_publishers(self):
        """把上游话题**实际的**类型与 QoS 报出来，不让人靠猜。

        2026-09-03 的教训：厂商把 `/map_scan_filtered` 从 PointCloud2 改成了
        `nav_msgs/OccupancyGrid`。此时话题名完全正确、发布者也在以 10Hz 发，
        而本节点按 PointCloud2 订阅 —— 回调一次都不执行，`收=0`。
        原来的自查清单第 ② 条让人去查**话题名**，于是排查会卡在
        "话题名对的呀" 上；第 ③ 条讲的是 SLAM 收不到雷达，方向也不对。
        所以这里直接把实测的类型/QoS 打出来，一眼就能看出是不是类型变了。
        """
        try:
            infos = self.get_publishers_info_by_topic(self.cloud_topic)
        except Exception as exc:                     # noqa: BLE001
            return f'    （查询发布者失败：{exc}）\n'
        if not infos:
            return ('    实测：该话题**没有任何发布者** —— '
                    '上游没起来，或 domain/DDS profile 不一致。\n')
        lines = []
        for i in infos:
            q = i.qos_profile
            lines.append(
                f'    实测发布者 {i.node_name}：类型 {i.topic_type}'
                f'  reliability={str(q.reliability).split(".")[-1]}'
                f'  durability={str(q.durability).split(".")[-1]}\n')
        return ''.join(lines)

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
            f'  本节点按 sensor_msgs/PointCloud2 订阅 {self.cloud_topic}。\n'
            f'{self._describe_publishers()}'
            f'  依次查：\n'
            f'  ① **话题类型是否被上游改了**（收=0 且上面实测有发布者时，'
            f'几乎一定是这一条）。\n'
            f'     2026-09-03 实测：厂商把 /map_scan_filtered 改成了 '
            f'nav_msgs/OccupancyGrid，\n'
            f'     话题名没变、10Hz 照发，而本节点的回调一次都不执行。\n'
            f'     此时不要改本节点的订阅类型 —— 那份栅格**没有未知区(-1)**，\n'
            f'     用它当 /map 会让前沿检测失效、自主探索立刻判"完成"。\n'
            f'     可行替代是改吃 /map_scan（仍是 PointCloud2），\n'
            f'     但必须同时把 z 切片补成 [0.05, 1.63]：实测 /map_scan 有大量\n'
            f'     2.6~4.3m 的天花板点，放通会把天花板投影成地面障碍。\n'
            f'  ② Voxel-SLAM 是否在跑（ros2 node list | grep -i voxel）\n'
            f'  ③ 话题名是否真是 {self.cloud_topic}'
            f'（候选：/map_cmap /map_pmap /map_scan /map_true）\n'
            f'  ④ **Voxel-SLAM 很可能收不到雷达数据**：mid360.yaml 的\n'
            f'     lidar_type=0(LIVOX) 意味着它订阅 livox_ros_driver2/CustomMsg，\n'
            f'     而厂商驱动 xfer_format=0 发 PointCloud2 —— 类型不匹配，零数据。\n'
            f'     先确认厂商雷达驱动在跑：ros2 topic hz /livox/lidar_front\n'
            f'  ⑤ ROS_DOMAIN_ID 是否一致（实机是 25，不是 42）')
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
    except ExternalShutdownException:
        # SIGTERM（launch 关停 / systemd stop）的正常表现，不是故障。
        # 不接住的话 rclpy 会把它抛成一串栈回溯，看起来像崩溃 ——
        # 而那会掩盖真正的错误，也让 launch 的退出处理器难以区分正常与异常。
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
