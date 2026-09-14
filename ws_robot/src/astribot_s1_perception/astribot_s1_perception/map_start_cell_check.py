#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""出生点栅格校验：换到外部地图之后，机器人是不是出生在墙里。

============================ 为什么必须有这一步 ============================
`localization: ground_truth` 下，机器人在地图里的位置完全由
`map_to_odom_xyz_yaw` 这一条静态变换决定。这个数填错的后果**不会**在启动时
报错，而是等到规划阶段才以一个误导性的症状暴露：

    [planner_server] GridBased ... Starting point in lethal space!

我们已经栽过一次同类问题（当时是夹爪没进点云自滤，机器人把指尖当障碍，
SLAM 把幻影烙进地图，于是机器人所在格被膨胀成 INSCRIBED_INFLATED）。
从症状到根因隔着三层，排查成本极高。换外部地图之后，坐标填错会**必然**
复现同一个症状，所以必须在启动时就拦住，并且把实际栅格值打出来。

判据刻意与探索协调器的目标校验对齐：出生点必须是**已知且自由**，
且给定半径内不允许有占据栅格。
    · "未知"也算不通过 —— 未知区域下面可能是墙，代价地图会按未知处理，
      规划器同样拒绝从那里出发。
    · 半径默认 0.25m（与 xy_goal_tolerance 同口径）。**不要**用
      robot_radius 0.42：那个口径等于把足迹重复计一遍，实测会把绝大多数
      合法站位也判成不可站。
===========================================================================

用法（launch 里作为一次性校验节点，不通过就非零退出，带着整个 launch 一起停）：
    ros2 run astribot_s1_perception map_start_cell_check --ros-args \
        --params-file <map_source.yaml> -p map_topic:=/map
"""

import math
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


_OCCUPIED_THRESHOLD = 65


class StartCellVerdict:
    """校验结论。刻意不用异常：调用方要的是"为什么不通过"，而不是栈回溯。"""

    def __init__(self, ok, reason, cell_value=None, grid_xy=None, world_xy=None):
        self.ok = ok
        self.reason = reason
        self.cell_value = cell_value
        self.grid_xy = grid_xy
        self.world_xy = world_xy

    def __str__(self):
        parts = [('通过' if self.ok else '不通过'), self.reason]
        if self.world_xy is not None:
            parts.append(f'世界坐标=({self.world_xy[0]:.3f}, {self.world_xy[1]:.3f})')
        if self.grid_xy is not None:
            parts.append(f'栅格=({self.grid_xy[0]}, {self.grid_xy[1]})')
        if self.cell_value is not None:
            parts.append(f'栅格值={self.cell_value}')
        return ' | '.join(str(p) for p in parts)


def check_start_cell(info_width, info_height, resolution, origin_x, origin_y,
                     data, world_x, world_y, clearance_m):
    """纯函数版校验，不依赖 ROS，便于单测。

    参数刻意摊平成基本类型而不是收 OccupancyGrid：单测里造一张小地图
    就不需要构造 ROS 消息，断言也能写得死。
    """
    if resolution <= 0.0:
        return StartCellVerdict(False, f'地图分辨率非法({resolution})')
    if info_width <= 0 or info_height <= 0:
        return StartCellVerdict(False, f'地图尺寸非法({info_width}x{info_height})')
    if len(data) != info_width * info_height:
        return StartCellVerdict(
            False,
            f'地图数据长度{len(data)}与尺寸{info_width}x{info_height}不符')

    gx = int(math.floor((world_x - origin_x) / resolution))
    gy = int(math.floor((world_y - origin_y) / resolution))
    world_xy = (world_x, world_y)

    if not (0 <= gx < info_width and 0 <= gy < info_height):
        return StartCellVerdict(
            False, '出生点落在地图范围之外', grid_xy=(gx, gy), world_xy=world_xy)

    value = data[gy * info_width + gx]
    if value < 0:
        return StartCellVerdict(
            False, '出生点所在栅格是**未知**区域（未知区下面可能是墙，'
                   '代价地图与规划器都会拒绝从这里出发）',
            cell_value=value, grid_xy=(gx, gy), world_xy=world_xy)
    if value >= _OCCUPIED_THRESHOLD:
        return StartCellVerdict(
            False, '出生点所在栅格是**占据**的（机器人出生在墙里）',
            cell_value=value, grid_xy=(gx, gy), world_xy=world_xy)

    if clearance_m > 0.0:
        radius_cells = int(math.ceil(clearance_m / resolution))
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if math.hypot(dx, dy) * resolution > clearance_m:
                    continue
                nx, ny = gx + dx, gy + dy
                if not (0 <= nx < info_width and 0 <= ny < info_height):
                    continue
                neighbour = data[ny * info_width + nx]
                if neighbour >= _OCCUPIED_THRESHOLD:
                    return StartCellVerdict(
                        False,
                        f'出生点净空不足：半径{clearance_m:.2f}m内存在占据栅格'
                        f'（在({nx}, {ny})，值={neighbour}）',
                        cell_value=value, grid_xy=(gx, gy), world_xy=world_xy)

    return StartCellVerdict(
        True, '出生点已知、自由、净空满足', cell_value=value,
        grid_xy=(gx, gy), world_xy=world_xy)


class MapStartCellCheck(Node):
    """订阅一次 /map，校验出生点，然后退出。"""

    def __init__(self):
        super().__init__('map_start_cell_check')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_to_odom_xyz_yaw', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('start_cell_clearance_m', 0.25)
        self.declare_parameter('map_wait_sec', 30.0)

        self._topic = self.get_parameter('map_topic').value
        pose = list(self.get_parameter('map_to_odom_xyz_yaw').value or [])
        if len(pose) != 4:
            self.get_logger().error(
                f'map_to_odom_xyz_yaw 必须是 4 个数 [x, y, z, yaw]，收到 {len(pose)} 个')
            self.verdict = StartCellVerdict(False, 'map_to_odom_xyz_yaw 配置非法')
            self.done = True
            return

        self._world_x = float(pose[0])
        self._world_y = float(pose[1])
        self._clearance = float(self.get_parameter('start_cell_clearance_m').value)
        self.verdict = None
        self.done = False

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self._topic, self._on_map, qos)
        self.get_logger().info(
            f'等待 {self._topic}，校验出生点 '
            f'({self._world_x:.3f}, {self._world_y:.3f}) 净空 {self._clearance:.2f}m')

    def _on_map(self, msg):
        if self.done:
            return
        info = msg.info
        self.verdict = check_start_cell(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            msg.data, self._world_x, self._world_y, self._clearance)
        self.get_logger().info(
            f'地图指纹: {info.width}x{info.height} res={info.resolution:.3f} '
            f'origin=({info.origin.position.x:.3f}, {info.origin.position.y:.3f})')
        if self.verdict.ok:
            self.get_logger().info(f'出生点校验{self.verdict}')
        else:
            self.get_logger().error(f'出生点校验{self.verdict}')
            self.get_logger().error(
                '拒绝启动。要么改 map_source.yaml 的 map_to_odom_xyz_yaw，'
                '要么确认加载的是正确的地图。'
                '若继续启动，规划器会报 "Starting point in lethal space!"，'
                '那个症状离根因很远。')
        self.done = True


def main(argv=None):
    rclpy.init(args=argv)
    node = MapStartCellCheck()
    wait_sec = 30.0
    try:
        wait_sec = float(node.get_parameter('map_wait_sec').value)
    except Exception:
        pass

    import time
    started = time.time()
    while rclpy.ok() and not node.done and time.time() - started < wait_sec:
        rclpy.spin_once(node, timeout_sec=0.2)

    verdict = node.verdict
    if verdict is None:
        node.get_logger().error(
            f'{wait_sec:.0f}s 内没收到 {node._topic}。'
            '地图源没起来，或 QoS 不匹配（latched 话题必须 TRANSIENT_LOCAL 订阅）。')
        exit_code = 1
    else:
        exit_code = 0 if verdict.ok else 1

    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
