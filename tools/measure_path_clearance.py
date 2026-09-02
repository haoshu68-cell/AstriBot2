#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实测「全局路径是否居中」。

这是验收 inflation_radius 0.65 -> 0.75 的**直接证据**。
不看参数、不看代价剖面，只看一件事：
    /plan 上每个路径点，到最近障碍物的实测距离是多少。

为什么必须实测：代价剖面只证明"规划器有往中间靠的梯度"，
不证明"它真的靠了" —— cost_travel_multiplier 决定它多在意那个梯度，
而那一项是待标定的。看着像居中和真的居中是两件事。

距离用**欧氏距离变换**在 /map 上算（不是在代价地图上）：
代价地图的膨胀值是我们刚改的量，用它衡量会变成自证。
/map 是 SLAM 的原始占据栅格，与本次改动无关。

用法:
  ROS_DOMAIN_ID=25 python3 tools/measure_path_clearance.py --samples 40
"""
import argparse
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:              # 没有 scipy 就退化成暴力最近邻
    distance_transform_edt = None


def latched_qos(depth=1):
    """/map 是 transient_local（SLAM/map_server 都这么发），用默认 QoS 收不到。"""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def volatile_qos(depth=10):
    """/plan 必须用 VOLATILE。

    !!! 这里踩过一次 !!!
    起初 /plan 也用了 latched_qos(TRANSIENT_LOCAL)，而 nav2_planner 发布
    /plan 用的是默认 VOLATILE —— **durability 不兼容**，订阅方一条都收不到，
    且现象是"话题有发布者、就是没消息"，与"规划器没出路径"完全同形。
    当时脚本报「没有采到任何 /plan」，我差点去查规划器。
    QoS 请求比提供的更强 ⇒ 不匹配，这是 DDS 的规则，不是 bug。
    """
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE)


class ClearanceMeter(Node):

    def __init__(self, want_samples, occupied_threshold):
        super().__init__('path_clearance_meter')
        self.want = want_samples
        self.occ_thr = occupied_threshold
        self.grid = None
        self.dist_m = None          # 每格到最近占据格的距离(m)
        self.samples = []           # 每条路径一条记录
        self.seen_paths = 0

        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched_qos())
        self.create_subscription(Path, '/plan', self.on_path, volatile_qos(10))

    def on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        data = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        # 只把「确定占据」当障碍。未知(-1) 不算 —— 否则未探明区边界会被
        # 当成墙，路径净空被系统性低估。
        occupied = data >= self.occ_thr
        free_like = ~occupied
        if distance_transform_edt is not None:
            # EDT 给的是「到最近 False 的距离」，所以传 free_like
            self.dist_m = distance_transform_edt(free_like) * msg.info.resolution
        else:
            self.get_logger().error('缺少 scipy，无法算距离变换')
            self.dist_m = None
        self.grid = msg.info
        self.get_logger().info(
            f'收到地图 {w}x{h} @{msg.info.resolution}m，占据格 {int(occupied.sum())} 个')

    def on_path(self, msg):
        if self.dist_m is None or self.grid is None:
            return
        if len(msg.poses) < 2:
            return
        g = self.grid
        clears = []
        for ps in msg.poses:
            mx = int((ps.pose.position.x - g.origin.position.x) / g.resolution)
            my = int((ps.pose.position.y - g.origin.position.y) / g.resolution)
            if 0 <= mx < g.width and 0 <= my < g.height:
                clears.append(float(self.dist_m[my, mx]))
        if len(clears) < 2:
            return
        self.seen_paths += 1
        self.samples.append({
            'n': len(clears),
            'min': min(clears),
            'mean': sum(clears) / len(clears),
            'p10': float(np.percentile(clears, 10)),
        })
        if self.seen_paths % 5 == 0:
            self.get_logger().info(f'已采 {self.seen_paths}/{self.want} 条路径')

    def done(self):
        return self.seen_paths >= self.want


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', type=int, default=30, help='采多少条 /plan')
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('--occupied-threshold', type=int, default=65)
    # 判据：路径点到墙的最小净空。1.5m 通道居中 => 0.75m；
    # 但真实地图有比 1.5m 更窄的通道，那里物理上不可能有 0.75m。
    # 所以判据只能压在「比内切半径明显富余」这个量级上，具体值由实测定标。
    ap.add_argument('--expect-mean-min', type=float, default=0.0,
                    help='>0 时对 min 净空的均值做断言')
    args = ap.parse_args()

    if distance_transform_edt is None:
        print('需要 scipy: pip install scipy', file=sys.stderr)
        return 2

    rclpy.init()
    node = ClearanceMeter(args.samples, args.occupied_threshold)
    t0 = node.get_clock().now()
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.5)
        if (node.get_clock().now() - t0).nanoseconds / 1e9 > args.timeout:
            node.get_logger().warn('超时，用已采到的样本出结论')
            break

    s = node.samples
    print()
    print('=' * 62)
    if not s:
        print('没有采到任何 /plan —— 无法给出结论（不要把"没数据"当成"合格"）')
        rclpy.shutdown()
        return 1
    mins = [x['min'] for x in s]
    means = [x['mean'] for x in s]
    p10s = [x['p10'] for x in s]
    print(f'样本: {len(s)} 条路径')
    print(f'  每条路径的**最小净空**: 均值 {sum(mins)/len(mins):.3f}m  '
          f'最小 {min(mins):.3f}m  最大 {max(mins):.3f}m')
    print(f'  每条路径的**平均净空**: 均值 {sum(means)/len(means):.3f}m')
    print(f'  每条路径的 P10 净空  : 均值 {sum(p10s)/len(p10s):.3f}m')
    print()
    print('参考量: 八边形内切 0.388m / 外接 0.420m / 1.5m 通道居中 = 0.750m')
    print('=' * 62)
    rc = 0
    if args.expect_mean_min > 0.0:
        got = sum(mins) / len(mins)
        ok = got >= args.expect_mean_min
        print(f'判定: 最小净空均值 {got:.3f}m '
              f'{">=" if ok else "<"} 期望 {args.expect_mean_min:.3f}m -> '
              f'{"PASS" if ok else "FAIL"}')
        rc = 0 if ok else 1
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
