#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试可视化/诊断脚本（按约束，Python 在本包内仅用于调试可视化）。

用途：不开 RViz 也能在终端里快速判断两个模块是否健康，以及问题出在哪一环。
这是排查「为什么 costmap 里有幽灵障碍物」「为什么探索不出目标」时的第一站。

它做四件事：
  1. 统计 /scan_from_cloud 的帧率、有效障碍束占比、最近障碍距离；
  2. 把扫描按扇区打印成 ASCII 环形图，直观看出哪个方向有东西；
  3. 检查各切片层 Marker 的点数分布，确认「多层切片是不是真的每层都有数据」；
  4. 跟踪 /explore/status 与 /explore/goal_pose，打印探索状态与目标变化。

用法：
    ros2 run astribot_s1_autonomy scan_slice_debug.py
    ros2 run astribot_s1_autonomy scan_slice_debug.py --ros-args \\
        -p scan_topic:=/scan -p duration_sec:=30.0
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray


class ScanSliceDebug(Node):
    """订阅感知/探索模块的输出，做统计并周期性打印诊断报告。"""

    def __init__(self):
        super().__init__('scan_slice_debug')

        self.declare_parameter('scan_topic', '/scan_from_cloud')
        self.declare_parameter('marker_topic',
                               '/pointcloud_slice_scan_node/debug_markers')
        self.declare_parameter('status_topic', '/explore/status')
        self.declare_parameter('goal_topic', '/explore/goal_pose')
        self.declare_parameter('report_period_sec', 5.0)
        self.declare_parameter('duration_sec', 0.0)   # 0 = 一直跑
        self.declare_parameter('sector_count', 16)

        self._scan_topic = self.get_parameter('scan_topic').value
        self._sector_count = int(self.get_parameter('sector_count').value)
        self._duration = float(self.get_parameter('duration_sec').value)

        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, sensor_qos)
        self.create_subscription(
            MarkerArray, self.get_parameter('marker_topic').value,
            self._on_markers, QoSProfile(depth=1))
        self.create_subscription(
            String, self.get_parameter('status_topic').value,
            self._on_status, QoSProfile(depth=5))
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal, QoSProfile(depth=5))
        # 完成标志是 transient_local，必须用匹配 QoS 否则收不到
        self.create_subscription(
            Bool, '/explore/complete', self._on_complete, latched_qos)

        self._scan_count = 0
        self._last_scan = None
        self._slice_point_counts = {}
        self._status_text = '(未收到)'
        self._complete = None
        self._goal_count = 0
        self._last_goal = None
        self._goal_moves = []

        self._start = self.get_clock().now()
        self.create_timer(
            float(self.get_parameter('report_period_sec').value), self._report)

        self.get_logger().info(
            f'诊断脚本已启动，监听 scan={self._scan_topic}。'
            f'{"持续运行" if self._duration <= 0 else f"运行 {self._duration:.0f}s 后退出"}')

    # ---------------- 回调 ----------------
    def _on_scan(self, msg):
        self._scan_count += 1
        self._last_scan = msg

    def _on_markers(self, msg):
        counts = {}
        for marker in msg.markers:
            if marker.ns.startswith('slice_') or marker.ns == 'self_filtered':
                counts[marker.ns] = len(marker.points)
        if counts:
            self._slice_point_counts = counts

    def _on_status(self, msg):
        self._status_text = msg.data

    def _on_complete(self, msg):
        self._complete = msg.data

    def _on_goal(self, msg):
        self._goal_count += 1
        new = (msg.pose.position.x, msg.pose.position.y)
        if self._last_goal is not None:
            moved = math.hypot(new[0] - self._last_goal[0], new[1] - self._last_goal[1])
            self._goal_moves.append(moved)
        self._last_goal = new

    # ---------------- 报告 ----------------
    def _scan_stats(self):
        """返回 (有效束数, 总束数, 最近距离, 各扇区最近距离)。"""
        scan = self._last_scan
        if scan is None or not scan.ranges:
            return None

        total = len(scan.ranges)
        sectors = [math.inf] * self._sector_count
        valid = 0
        closest = math.inf
        for i, r in enumerate(scan.ranges):
            # 「无障碍」有两种表示：inf 或 等于 range_max（取决于 no_return_mode）
            if not math.isfinite(r) or r >= scan.range_max - 1e-6:
                continue
            if r < scan.range_min:
                continue
            valid += 1
            closest = min(closest, r)
            angle = scan.angle_min + (i * scan.angle_increment)
            # 把 [-pi, pi) 映射到扇区下标，0 号扇区正对车头
            norm = (angle + math.pi) % (2.0 * math.pi)
            idx = int(norm / (2.0 * math.pi) * self._sector_count) % self._sector_count
            sectors[idx] = min(sectors[idx], r)
        return valid, total, closest, sectors

    def _report(self):
        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed <= 0.0:
            return

        print('\n' + '=' * 68)
        print(f'  运行 {elapsed:5.1f}s   感知帧率 {self._scan_count / elapsed:5.2f} Hz'
              f'   累计 {self._scan_count} 帧')
        print('=' * 68)

        stats = self._scan_stats()
        if stats is None:
            print('  [感知] 尚未收到任何 LaserScan —— 检查：')
            print('         · 输入点云话题是否有数据 (ros2 topic hz /livox/fused_points)')
            print('         · TF 是否可用 (节点会打 WARN 并丢帧)')
            print('         · use_sim_time 是否为 true')
        else:
            valid, total, closest, sectors = stats
            ratio = (valid / total * 100.0) if total else 0.0
            closest_text = (f'最近障碍 {closest:5.2f} m' if math.isfinite(closest)
                            else '全向无障碍')
            print(f'  [感知] 有效障碍束 {valid}/{total} ({ratio:5.1f}%)   {closest_text}')
            # 扇区最近距离：0° 为车头，逆时针递增
            print('         扇区最近距离(m):')
            line = '         '
            for i, d in enumerate(sectors):
                tag = f'{int(i * 360 / self._sector_count):3d}°'
                cell = f'{d:5.2f}' if math.isfinite(d) else '  -- '
                line += f'{tag}={cell}  '
                if (i + 1) % 4 == 0:
                    print(line)
                    line = '         '
            if line.strip():
                print(line)

        if self._slice_point_counts:
            print('  [切片] 各层点数（确认多层切片每层都有数据，而不是只有一层在干活）:')
            for ns in sorted(self._slice_point_counts):
                count = self._slice_point_counts[ns]
                bar = '#' * min(40, count // 20)
                name = ns.replace('slice_', '') if ns != 'self_filtered' else '（自身剔除）'
                print(f'         {name:>14s} {count:6d} {bar}')
            if all(v == 0 for k, v in self._slice_point_counts.items()
                   if k.startswith('slice_')):
                print('         !! 所有切片层都是 0 点：z 区间可能配错了。')
                print('            注意 z 是 astribot_torso_base 系下的高度，地面约 -0.129m。')
        else:
            print('  [切片] 未收到 Marker（publish_markers 是否为 true？话题名是否对？）')

        complete_text = ('(未收到)' if self._complete is None
                         else ('是' if self._complete else '否'))
        print(f'  [探索] 状态: {self._status_text}')
        print(f'         探索完成标志: {complete_text}')
        print(f'         已输出目标 {self._goal_count} 个', end='')
        if self._last_goal:
            print(f'   最新目标 ({self._last_goal[0]:.2f}, {self._last_goal[1]:.2f})')
        else:
            print()
        if self._goal_moves:
            avg_move = sum(self._goal_moves) / len(self._goal_moves)
            print(f'         相邻目标平均跳变 {avg_move:.2f} m', end='')
            # 平均跳变极小说明目标在原地反复输出 —— 震荡抑制没起作用
            if avg_move < 0.1:
                print('  !! 目标几乎不动，疑似震荡，检查 max_same_goal_count')
            else:
                print()

        if self._duration > 0.0 and elapsed >= self._duration:
            print('\n达到设定时长，退出。')
            # 不在回调里调 rclpy.shutdown()（会让 spin 卡住），
            # 改为置标志由主循环退出。
            raise SystemExit(0)


def main():
    rclpy.init()
    node = ScanSliceDebug()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
