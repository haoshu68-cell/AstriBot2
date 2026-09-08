#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
感知链硬门禁：逐跳数**实收帧数**，任一跳为 0 就把断点打出来并以非 0 退出。

为什么要它（run8 的教训，逐条都是实测）：
  栈起来后 Gazebo 在步进（/clock 868Hz）、rviz 进程在、18 个节点全在、
  nav2 全部 active —— 但 /livox/fused_points 起整条链 0 帧，空转了 900 秒，
  **没有任何一处报错**。表现只有"rviz 一片空白 + 探索 dispatched=0"。

为什么判据只能是"实收帧数"，三个更省事的判据都骗过我：
  · 进程存活：livox_fusion_node 活着、12 个线程、fd 数与工作节点逐字相同，
    但它在 DDS 上三个端点(2 订阅 + 1 发布)一个都不存在，30s 只烧 1 个 CPU tick。
  · 发布者数：/livox/fused_points 只有 SUB 没有 PUB，照样出现在 `ros2 topic list` 里。
  · `ros2 topic hz`：加 --qos-reliability best_effort 后，本机实测它对 /odom(47Hz)、
    /livox/lidar_left(19Hz) 一律报 0 —— 工具本身在这台机器上不可用。

订阅一律用 BEST_EFFORT：它对 RELIABLE 和 BEST_EFFORT 发布者**都兼容**，
所以不会因为 QoS 把"发布端没发"和"我这边收不到"混成同一个读数。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage

# 链条按数据流顺序排列，报错时报**第一个**断点，这样直接指向责任节点。
# (话题, 类型, 该跳的上游节点名 —— 断在这一跳时要查的就是它)
CHAIN = [
    ('/clock',                      None,           'gazebo (ign gazebo)'),
    ('/odom',                       Odometry,       'omni_effort_drive_node / gz bridge'),
    ('/livox/lidar_left',           PointCloud2,    'parameter_bridge (gz->ros)'),
    ('/livox/lidar_right',          PointCloud2,    'parameter_bridge (gz->ros)'),
    ('/livox/left/cloud_filtered',  PointCloud2,    'livox_preprocess_left'),
    ('/livox/right/cloud_filtered', PointCloud2,    'livox_preprocess_right'),
    ('/livox/fused_points',         PointCloud2,    'livox_fusion_node'),
    ('/livox/cloud_self_filtered',  PointCloud2,    'pointcloud_slice_scan_node'),
    ('/scan_from_cloud',            LaserScan,      'pointcloud_slice_scan_node'),
    ('/scan',                       LaserScan,      'pointcloud_to_laserscan'),
    ('/map',                        OccupancyGrid,  'slam_toolbox'),
]

# /clock 单独处理（rosgraph_msgs 不在上面的 import 里，避免这个文件被当成消息包依赖）
from rosgraph_msgs.msg import Clock  # noqa: E402
CHAIN[0] = ('/clock', Clock, 'gazebo (ign gazebo)')

# map 这一跳还不够：slam 建了图不等于 TF 里有 map frame，而 rviz 的 Fixed Frame
# 是 map —— frame 不在，**整个场景包括机器人模型都渲染不出来**（用户直接看到的现象）。
# 所以额外单独校验 TF 里的 map。


class ChainProbe:
    """
    全链探针：**节点只建一次**，每个测量窗口之间只清计数器。

    ⚠️ 这里原来是每轮 `Node(...)` + `destroy_node()`，那个写法有 bug 且**极其容易误判**：
    第一个窗口读数正常，从第二个窗口起**全链同时归零**（连 /clock 都是 0）。
    run9 实测就是这样：
        [  8s] 未通，缺: /livox/fused_points          <- 只缺一个，其余全 OK
        [ 16s] 未通，缺: /clock /odom ...（全部 11 条）<- 突然全 0
    而同一时刻栈是完全健康的：手工单次跑同一份判据，fused_points 7.37Hz、map frame 在、
    协调器已派发目标并在导航。也就是说这个 bug 会把一个**正常的栈**报成"感知链全断"。

    怎么区分"探针掉线"和"链真的断了"（这两者代价完全不同，必须分清）：
      · 全链同时为 0、且 /clock 也为 0  => 探针自己的问题。/clock 由 Gazebo 直发、
        不经过感知链任何一跳，它归零说明问题在订阅侧而不在链上。
      · 部分为 0，且断点上游有帧、下游无帧 => 链真的断了。run8 的融合节点就是这种：
        上游 276 帧、它自己输出 0 帧，且 /proc 里 30s 只烧 1 个 CPU tick
        （CPU 读数不经过 DDS，所以不会被探针问题污染 —— 交叉验证要用这种独立量）。
    """

    def __init__(self):
        self.node = Node('chain_gate_probe')
        self.counts = {t: 0 for t, _, _ in CHAIN}
        self.frames = set()

        def mk(topic):
            def cb(_msg):
                self.counts[topic] += 1
            return cb

        for topic, msg_type, _ in CHAIN:
            self.node.create_subscription(
                msg_type, topic, mk(topic), qos_profile_sensor_data)

        def tf_cb(msg):
            for tr in msg.transforms:
                self.frames.add(tr.header.frame_id)
                self.frames.add(tr.child_frame_id)

        self.node.create_subscription(TFMessage, '/tf', tf_cb, qos_profile_sensor_data)
        self.node.create_subscription(
            TFMessage, '/tf_static', tf_cb, qos_profile_sensor_data)

    def measure(self, window_sec: float):
        """跑一个测量窗口。计数器清零，但 TF frame 集合**不清** —— /tf_static
        是 TRANSIENT_LOCAL 之外的一次性广播，清掉会让后续窗口看不到静态 frame。"""
        for k in self.counts:
            self.counts[k] = 0
        t0 = time.time()
        while time.time() - t0 < window_sec:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self.counts), set(self.frames), time.time() - t0

    def destroy(self):
        self.node.destroy_node()


def report(counts, frames, elapsed):
    print('  %-32s %8s %10s' % ('话题', '帧数', 'Hz'))
    for topic, _, owner in CHAIN:
        n = counts[topic]
        mark = 'OK ' if n > 0 else '🔴 '
        print('  %s%-30s %8d %9.2f   %s' % (mark, topic, n, n / elapsed, '' if n else '<= 断点，查 ' + owner))
    print('  %s%-30s %8s %9s   %s' % (
        'OK ' if 'map' in frames else '🔴 ', 'TF:map frame',
        len(frames), '-', '' if 'map' in frames else '<= rviz Fixed Frame 解析不了，整个场景空白'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=240.0, help='门禁等待上限（秒）')
    ap.add_argument('--window', type=float, default=8.0, help='每次测量窗口（秒）')
    args = ap.parse_args()

    rclpy.init()
    t_start = time.time()
    attempt = 0
    counts = frames = None
    elapsed = 1.0
    seen_clock = False   # /clock 是否曾经出过数（见下面全零分支的判据说明）
    probe = ChainProbe()
    try:
        while time.time() - t_start < args.timeout:
            attempt += 1
            counts, frames, elapsed = probe.measure(args.window)
            all_ok = all(counts[t] > 0 for t, _, _ in CHAIN) and 'map' in frames
            waited = int(time.time() - t_start)
            if all_ok:
                print('\n✅ 感知链全通（等待 %ds，第 %d 次测量）' % (waited, attempt))
                report(counts, frames, elapsed)
                return 0
            broken = [t for t, _, _ in CHAIN if counts[t] == 0]
            # "全链同时为 0（含 /clock）"这个启发式**只有在 /clock 曾经出过数之后才成立**：
            #   · 启动初期全零是正常的 —— Gazebo 世界还没加载完，一个发布者都还没有。
            #     run10 实测第 8 秒就是这种情况，早期版本在这里打了一条"探针异常"的假诊断。
            #   · 而 /clock 已经出过数、之后又全零，才是探针/DDS 侧掉线的特征
            #     （/clock 由 Gazebo 直发、不经感知链任何一跳，它归零说明问题在订阅侧）。
            if counts['/clock'] > 0:
                seen_clock = True
            if len(broken) == len(CHAIN):
                if seen_clock:
                    print('  [%3ds] ⚠️ 全链同时归零，但 /clock 之前出过数 —— 这是探针/DDS 侧'
                          '掉线的特征，不是感知链断裂；继续重试' % waited, flush=True)
                else:
                    print('  [%3ds] 栈还没开始发数（/clock 尚未出现），仿真在加载世界，正常'
                          % waited, flush=True)
            else:
                print('  [%3ds] 未通，缺: %s' % (waited, ' '.join(broken) or 'TF:map'), flush=True)
        print('\n🔴 门禁超时（%.0fs）。逐跳实测如下，第一个 🔴 就是断点：' % args.timeout)
        if counts is not None:
            report(counts, frames, elapsed)
        return 1
    finally:
        probe.destroy()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
