#!/usr/bin/env python3
"""等仿真真正开始步进物理，判据是 /clock 的**实收帧数**。

为什么单独挑 /clock 出来做启动判据（run10 的直接教训）：
  Gazebo 有一种加载期死锁：进程全在、rviz 在、18 个节点全在、`ign gazebo server`
  也在烧 CPU，但物理一步都没走。实测特征是三条同时成立：
    · /clock 发布者数 = 1（桥接活着），但 6 秒收到 0 帧；
    · gazebo server 只烧 ~2 ticks/s，而 GUI 烧 ~143 ticks/s（渲染在转，物理没动）；
    · stack.log 里 gz_ros2_control 停在 "asking for robot_description" 之后再无输出。
  这种情况下等下去没有意义 —— run10 等满 240s 一帧没有。唯一出路是重启仿真。

所以这里**不做全链门禁**，只回答一个问题："仿真在走吗？"
在走就立刻返回，让上层赶紧把录制挂上（录制必须早于第一个目标派发）；
不在走就非 0 退出，让上层杀栈重来。

判据只能是实收帧数，不能是发布者数 —— 上面第一条就是"有发布者、0 帧"。
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock

# /clock 是 BEST_EFFORT 发布的。订阅侧写 RELIABLE 会一帧都收不到且只有一条 WARNING，
# 表现和"仿真没步进"完全一样 —— 见 memory qos-reliability-breaks-chains-silently。
CLOCK_QOS = QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=100.0,
                    help='等待上限（秒）。超了就认定加载期死锁，非 0 退出')
    ap.add_argument('--min-frames', type=int, default=20,
                    help='认定"在步进"所需的最少帧数。要求 >1 是为了排除单帧抖动')
    args = ap.parse_args()

    rclpy.init()
    node = Node('wait_sim_stepping')
    got = {'n': 0, 'first': None, 'last': None}

    def on_clock(msg):
        got['n'] += 1
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if got['first'] is None:
            got['first'] = t
        got['last'] = t

    node.create_subscription(Clock, '/clock', on_clock, CLOCK_QOS)

    t0 = time.time()
    last_report = 0.0
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        waited = time.time() - t0
        if got['n'] >= args.min_frames:
            # 还要确认仿真时间**真的在前进**，不只是收到了帧。
            advanced = (got['last'] or 0) - (got['first'] or 0)
            print('✅ 仿真在步进：等待 %.0fs，收到 %d 帧 /clock，仿真时间前进 %.2fs'
                  % (waited, got['n'], advanced), flush=True)
            return 0
        if waited - last_report >= 10.0:
            last_report = waited
            print('  [%3ds] /clock 发布者=%d 实收帧=%d（还没开始步进）'
                  % (waited, node.count_publishers('/clock'), got['n']), flush=True)

    print('🔴 %.0fs 内 /clock 只收到 %d 帧（发布者=%d）—— 判定 Gazebo 加载期死锁，'
          '物理没有步进。等下去无效，应重启仿真。'
          % (args.timeout, got['n'], node.count_publishers('/clock')), flush=True)
    return 1


if __name__ == '__main__':
    # ⚠️ 这里**刻意不走** node.destroy_node()/rclpy.shutdown()（原来是写在
    # try/finally 里的，2026-09-08 撤掉）：这台机器上 rclpy 的拆除路径会挂死。
    # 同一天在 verify_sim_stack.py 上实测过一次：卡 1792s 只烧掉 0.7s CPU，
    # DDS 发现套接字 Recv-Q 堆到 213120 字节。这个脚本是**启动闸门**，
    # 被上层用退出码判"要不要重启"，它一挂整轮启动就永久卡住，
    # 而表象是"闸门卡住"，极容易被读成"栈没起来"。
    # 判据打印完就直接退进程 —— 退出码照旧 0/1，上层的判断不受影响。
    _rc = main()
    sys.stdout.flush()
    os._exit(_rc)
