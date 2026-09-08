#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""感知/TF 链闸门：在开始任何测量之前，判定"进程活着但在 DDS 图里不存在"这一类
启动期偶发故障。判据一律是**窗口内实测帧数**与**实测 TF 龄期/推进**。

═══ 为什么必须有这道闸门（2026-09-08 实测，代价是 22 分钟）═══
那一轮 `/clock` 步进判据**通过了**（3s 收到 20 帧、仿真时间前进 0.02s），
于是脚本照常起 rviz、进入分层验证。而实际上三条链整个是死的：

  进程                      状态          1200s 累计 CPU   应发布           实测
  robot_state_publisher     活着,URDF已解析  3.7s = 0.3%    整棵机器人 TF     0 条边
  livox_fusion_node         活着,打了横幅    10s  = 0.8%    /livox/fused_points  0 帧
  velocity_smoother         活着,已 Configuring  39s        nav2 末跳         —

三个进程在 45s 窗口里**一次都没出现在 DDS 图**（同期累计发现 42 个节点）。
"只是忙"被算术否掉：`/joint_states` 实测 96.6Hz，RSP 消费 1200s 不可能只花 3.7s CPU。
下游全部对得上：`/scan`、`/scan_from_cloud` 双 0Hz -> slam 无输入 -> 无 `/map`、
无 `map->odom` -> rviz 里 Fixed Frame=map 根本不存在（用户看到的"机器人无坐标"）,
stack.log 里 6261 条 costmap "observation buffer has not been updated"。
**根因未定**；`tools/run_five_round_exploration.sh` 头部记的是同一类故障
（连续三次启动各有一个不同节点中招）。现有对策只有：闸住 + 重启。

═══ 判据为什么是这三条 ═══
`/clock` 只证明 Gazebo 在走，与 RSP/感知链完全无关 —— 上面那轮就是反例。
这里挑的三条正好各自覆盖一段独立的进程与话题：

  1. RSP 链   TF 边 astribot_torso_base -> astribot_torso_link_1
     **必须挑一条 revolute 边**（这条来自 torso_wheel.xacro:267 的转动关节），
     因为它只可能由 RSP 消费 /joint_states 之后动态发出。
     不能拿 odom->astribot_torso_base 当判据：那条由 omni_effort_drive 发，
     上面那轮它 48Hz 活得很好而 RSP 是死的 —— 用它判会假阳性。
     也不能只判"查得到"：tf2 buffer 会留 10s，冻结的一帧照样查得到
     （见 memory frozen-counter-read-as-current-value）。所以还要求**戳在前进**。
  2. 融合链   /livox/fused_points 帧数（左右两路预处理 -> livox_fusion_node）
  3. 出口     scan 话题帧数（slice_scan 档是 /scan_from_cloud，laserscan 档是 /scan）

左右两路 cloud_filtered 只**报数不设门**：它们是 fused 的上游，
坏在哪一跳要靠它们区分（上面那轮左右两路都是 9.7Hz/28%CPU，坏的是 fusion 本身）。
把它们设成必过项等于凭空多一个可能误杀的门。

═══ 两条实现上的硬约束 ═══
· 点云订阅一律 raw=True：只数帧、不反序列化。Python 反序列化两万点跟不上 10Hz
  会静默丢 60~80% 帧，闸门自己就成了假阴性的来源
  （见 memory python-cannot-keep-up-with-livox-customsg）。
· 退出走 os._exit()，绝不 destroy_node()/rclpy.shutdown()/destroy_subscription()：
  这台机器上拆除路径会挂死。同一天实测过一次：卡 1792s 只烧掉 0.7s CPU、
  DDS 发现套接字 Recv-Q 堆到 213120 字节，表象是"闸门卡住"，
  极容易被读成"栈没起来"。判据打印完就走，不给关闭路径任何机会。
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

BASE_FRAME = 'astribot_torso_base'
# RSP 专属的动态边（revolute）。换 URDF 时这个名字要跟着改，
# 改错的表现是闸门恒不过 —— 比恒过安全，但仍然要在这里留下依据：
# ws_robot/src/astribot_s1_description/urdf/astribot_s1_torso_wheel.xacro:267-268
RSP_CHILD_FRAME = 'astribot_torso_link_1'

FUSED_TOPIC = '/livox/fused_points'
SCAN_TOPIC = {'slice_scan': '/scan_from_cloud', 'laserscan': '/scan'}
# 只报数、不设门的上游（用于把"坏在哪一跳"定出来）
INFO_TOPICS = ['/livox/left/cloud_filtered', '/livox/right/cloud_filtered']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=120.0,
                    help='等待上限（秒）。超了判为启动期偶发故障，非 0 退出让上层重启')
    ap.add_argument('--scan-source', default='slice_scan',
                    choices=sorted(SCAN_TOPIC))
    ap.add_argument('--window', type=float, default=5.0,
                    help='单个计数窗口长度（秒）。三条判据必须在**同一个**窗口内同时成立')
    ap.add_argument('--min-frames', type=int, default=3,
                    help='窗口内每条必过话题所需的最少帧数。>1 是为了排除单帧抖动/latch')
    ap.add_argument('--min-warmup', type=float, default=45.0,
                    help='本进程自己的 DDS 发现收敛下界（秒）。--timeout 不得小于它，'
                         '否则失败归因会是假的 —— 见下面 argparse 之后那段说明')
    args = ap.parse_args()

    # ═══ --timeout 不得短于发现收敛窗口（2026-09-08 自测撞出来的，改的是**闸门自己**）═══
    # 拿同一个已知坏栈连跑两次闸门，第一次四个窗口把上游左右两路全读成 0.0Hz，
    # 归因打印成"上游也断了，往预处理/gz 侧查"；第二次（--timeout 16）第一个窗口
    # 就读到 5.0/6.8Hz、随后稳定 9.6Hz，归因变成"上游有数据，断点就在融合节点本身"。
    # 后者才是对的（那两个预处理进程 27.4% CPU 活得很好）。差别只是**本进程自己的
    # DDS 发现有没有收敛** —— 本仓库已实测过新 participant 要 30~50s 才收敛
    # （memory dds-discovery-needs-40s-window-and-endpoint-can-lag）。
    # 也就是说：短 timeout 下这个闸门会把"我还没发现"讲成"它没在发"，
    # 而这条归因是要拿去指挥排查方向的。宁可不给归因，不能给错的归因。
    if args.timeout < args.min_warmup:
        print('🔴 --timeout=%.0fs 短于发现收敛下界 --min-warmup=%.0fs。'
              '这个组合下的失败归因不可信（实测：同一个坏栈，短 timeout 把 9.6Hz 的'
              '上游读成 0.0Hz 并指错排查方向），所以直接拒跑而不是给你一个假结论。'
              % (args.timeout, args.min_warmup), flush=True)
        sys.stdout.flush()
        os._exit(2)

    scan_topic = SCAN_TOPIC[args.scan_source]

    rclpy.init()
    node = Node('wait_perception_chain')
    buf = Buffer()
    TransformListener(buf, node)

    counts = {}

    def counter(key):
        counts[key] = 0

        def cb(_msg):
            counts[key] += 1
        return cb

    # raw=True：回调只拿到序列化字节，不反序列化点云（见文件头）。
    # msg_type 仍必须是**真的**消息类 —— rclpy 要用它取 typesupport；
    # 传 bytes 之类的占位类型会在 create_subscription 里直接抛异常。
    node.create_subscription(PointCloud2, FUSED_TOPIC, counter(FUSED_TOPIC),
                             qos_profile_sensor_data, raw=True)
    node.create_subscription(LaserScan, scan_topic, counter(scan_topic),
                             qos_profile_sensor_data, raw=True)
    for t in INFO_TOPICS:
        node.create_subscription(PointCloud2, t, counter(t),
                                 qos_profile_sensor_data, raw=True)

    def probe_tf():
        """返回 (查到了吗, 该边的戳秒). timeout 一律 0 —— 带 timeout 的
        lookup_transform 在非专用线程里对**动态** TF 会失败而静态照常成功，
        那种失败看起来像"TF 坏了"（见 memory ros2-tf-timeout-needs-dedicated-thread）。
        这里靠 spin 循环自己轮询，不依赖 tf2 的内部等待。"""
        try:
            tr = buf.lookup_transform(BASE_FRAME, RSP_CHILD_FRAME, Time())
        except Exception:
            return False, None
        return True, tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9

    t0 = time.time()
    last_line = '（一个计数窗口都没跑完 —— --timeout 比 --window 还短）'
    # 必须在循环外先初始化：--timeout < --window 时循环体一次都不执行，
    # 而下面的失败分支要读这三个标志。不初始化就是 NameError —— 闸门本该报
    # "哪一跳断了"，却变成一个跟栈毫无关系的 traceback。
    tf_ok = fused_ok = scan_ok = False
    tf_adv = 0.0
    while time.time() - t0 < args.timeout:
        for k in counts:
            counts[k] = 0
        tf_first = tf_last = None
        w0 = time.time()
        while time.time() - w0 < args.window:
            rclpy.spin_once(node, timeout_sec=0.05)
            ok, stamp = probe_tf()
            if ok:
                if tf_first is None:
                    tf_first = stamp
                tf_last = stamp

        dt = time.time() - w0
        # TF 判据两项都要：查得到 **且** 戳在窗口内前进（排除冻结的一帧）。
        tf_adv = (tf_last - tf_first) if (tf_first is not None) else 0.0
        tf_ok = tf_first is not None and tf_adv > 0.0
        fused_ok = counts[FUSED_TOPIC] >= args.min_frames
        scan_ok = counts[scan_topic] >= args.min_frames

        last_line = ('  [%3ds] RSP的TF %s->%s: %s(戳前进%.2fs)   %s=%.1fHz   %s=%.1fHz'
                     '   | 上游 左=%.1fHz 右=%.1fHz'
                     % (time.time() - t0, BASE_FRAME, RSP_CHILD_FRAME,
                        '有' if tf_first is not None else '**无**', tf_adv,
                        FUSED_TOPIC, counts[FUSED_TOPIC] / dt,
                        scan_topic, counts[scan_topic] / dt,
                        counts[INFO_TOPICS[0]] / dt, counts[INFO_TOPICS[1]] / dt))
        print(last_line, flush=True)

        if tf_ok and fused_ok and scan_ok:
            print('✅ 感知/TF 链就绪（三条判据在同一窗口内同时成立，等待 %.0fs）'
                  % (time.time() - t0), flush=True)
            sys.stdout.flush()
            os._exit(0)

    # ---- 不过。把"坏在哪一跳"写清楚，不要只说"闸门不过"。
    print('🔴 %.0fs 内感知/TF 链没有就绪。逐跳最后一次读数：' % args.timeout, flush=True)
    print(last_line, flush=True)

    # ⚠️ 归因之前先判"我这个观察者自己是不是什么都没看见"。
    # 全 0（连不设门的上游两路也是 0）+ TF 一条边都没有 = 这个读数与
    # "本进程的 DDS 发现从未收敛"**不可区分**。这种情况下逐跳归因是无意义的，
    # 说"栈坏了"也是越权 —— 必须如实说"分不出来"。
    saw_nothing = (not counts[INFO_TOPICS[0]] and not counts[INFO_TOPICS[1]]
                   and not counts[FUSED_TOPIC] and not counts[scan_topic]
                   and tf_first is None)
    if saw_nothing:
        print('   ⚠️ 本进程在最后一个窗口里**一条消息、一条 TF 边都没收到**（连不设门的'
              '上游两路也是 0）。这与"本观察者的 DDS 发现从未收敛"不可区分，'
              '所以下面不给逐跳归因 —— 给了就是猜的。', flush=True)
        print('   要分开这两种可能，只有一条路：查 ROS_DOMAIN_ID 是否与栈进程一致'
              '（本仓库被 warehouse_sim.launch.py 钉成 25），再看栈日志里有没有'
              ' process has died。', flush=True)
    else:
        broken = []
        if not tf_ok:
            broken.append('robot_state_publisher（整棵机器人 TF 缺失 -> rviz 里没有坐标；'
                          '注意 odom->%s 由 omni_effort_drive 发，它正常不代表 RSP 正常）'
                          % BASE_FRAME)
        if not fused_ok:
            up = counts[INFO_TOPICS[0]] + counts[INFO_TOPICS[1]]
            broken.append('livox_fusion_node（上游左右两路合计 %d 帧 -> %s）'
                          % (up, '上游也断了，往预处理/gz 侧查' if up == 0
                             else '**上游有数据，断点就在融合节点本身**'))
        if not scan_ok:
            broken.append('scan 出口 %s（fused 有数据的话断点在切片/投影节点）' % scan_topic)
        for b in broken:
            print('   · 断点候选: %s' % b, flush=True)
    print('   这是已知的启动期偶发故障（进程活着、日志正常、但 participant 在 DDS 图里'
          '不出现），**根因未定**，现有对策只有重启。', flush=True)
    sys.stdout.flush()
    os._exit(1)


if __name__ == '__main__':
    main()
