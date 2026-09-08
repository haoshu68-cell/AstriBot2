#!/usr/bin/env python3
"""实机侧：开录之前检查指标采集的输入是不是真的在出数。

**两类话题必须用两种判据** —— 这是这个脚本最容易写错的地方，我第一版就写错了：

  · 流式话题（/tf、/odom、/scan…）周期发布，判据是**实收帧数/拍率**。
    这里绝不能用发布者数：本项目实测过五个话题全部 pub=1 但 0Hz
    （BEST_EFFORT 发布者遇上 RELIABLE 订阅者，全场只有一条 WARNING），
    也实测过节点发现到了而话题 pub 恒 0。链上每个节点都有发布者，
    所以 count_publishers 判据恒为真。
  · 事件式话题（/exploration/current_goal、/plan、/cmd_vel）只在事件发生时
    发一次。**窗口内 0 帧完全正常** —— 协调器 PAUSED、goal_in_flight=0 时
    就是一帧都没有。对它们判据只能是"有没有发布者"，帧数只作参考。
    第一版拿帧数判它们，于是把一个活得很好的协调器报成了"没在跑"。

订阅端 QoS 一律 BEST_EFFORT + VOLATILE：订阅端放宽才能同时兼容两种发布者，
反过来（RELIABLE 订阅）会静默收不到 BEST_EFFORT 的数据。

不用 `ros2 topic hz`：实机的 ros2 **只有 7 个子命令**
（bag/daemon/extension_points/extensions/node/param/service），
`ros2 topic`/`run`/`launch` 全部报 invalid choice。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from action_msgs.msg import GoalStatusArray
from tf2_msgs.msg import TFMessage

STREAM = 'stream'   # 判据：实收拍率 > 0
EVENT = 'event'     # 判据：有发布者（帧数只作参考，0 帧是正常的）

# (话题, 类型, 种类, 是否必需, 说明)
CHECKS = [
    ('/tf',                        TFMessage,      STREAM, True,
     '底盘位姿的唯一来源。厂商栈不发布任何 TF，整棵树是我们这侧的责任'),
    ('/odom',                      Odometry,       STREAM, True,
     '速度与里程；TF 断掉时的兜底'),
    ('/exploration/state',         String,         STREAM, True,
     '相位、失败原因、各类计数'),
    ('/global_costmap/costmap',    OccupancyGrid,  STREAM, False,
     '净空判据。注意它是 0~100 而不是 0~255，253 会映射成 99'),
    ('/scan_from_cloud',           LaserScan,      STREAM, False,
     '感知链末端；实机是 livox 点云转出来的'),
    ('/exploration/current_goal',  PoseStamped,    EVENT,  True,
     '**轮次靠它切分**。没有发布者 = 协调器没在跑 = 一轮都采不到'),
    ('/plan',                      Path,           EVENT,  False,
     '横偏（cross-track）的参考线'),
    ('/cmd_vel',                   Twist,          EVENT,  False,
     '实发速度。机器人静止时本来就没有'),
    ('/compute_path_to_pose/_action/status', GoalStatusArray, EVENT, False,
     '规划耗时的唯一正经口径（隐藏话题）'),
]


class Probe(Node):
    def __init__(self):
        super().__init__('metrics_input_probe')
        self.counts = {t: 0 for t, _, _, _, _ in CHECKS}
        self.last_state = None
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        for topic, msg_type, _, _, _ in CHECKS:
            self.create_subscription(
                msg_type, topic,
                lambda m, t=topic: self._on(t, m),
                qos)

    def _on(self, topic, msg):
        self.counts[topic] += 1
        if topic == '/exploration/state':
            self.last_state = msg.data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--window', type=float, default=10.0,
                    help='计帧窗口（秒）')
    ap.add_argument('--rounds', type=int, default=5,
                    help='最多重试几个窗口。实机 DDS 发现实测要 30~50s 收敛，'
                         '所以是多轮重试而不是把单个窗口拉长')
    ap.add_argument('--base-frame', default='astribot_torso_base')
    ap.add_argument('--map-frame', default='map')
    args = ap.parse_args()

    rclpy.init()
    node = Probe()

    # TF 要真查一次 —— /tf 有帧不代表 map->base 这一条链是通的。
    # 用 Time()（取最新）而不是带 timeout 的 lookup：带 timeout 的查询在
    # 非专用线程里对**动态 TF** 会失败，而静态 TF 照样查得到，看起来一切正常。
    import tf2_ros
    buf = tf2_ros.Buffer()
    _listener = tf2_ros.TransformListener(buf, node)  # noqa: F841

    ok = False
    for rnd in range(1, args.rounds + 1):
        for t in node.counts:
            node.counts[t] = 0
        t0 = time.time()
        while time.time() - t0 < args.window:
            rclpy.spin_once(node, timeout_sec=0.1)

        tf_ok, tf_note = False, ''
        try:
            tr = buf.lookup_transform(args.map_frame, args.base_frame,
                                      rclpy.time.Time())
            age = (node.get_clock().now() - rclpy.time.Time.from_msg(
                tr.header.stamp)).nanoseconds / 1e9
            tf_ok = age < 2.0
            tf_note = 'age=%.2fs' % age
            if not tf_ok:
                tf_note += ' —— 陈旧！冻结的读数不能当当前值'
        except Exception as exc:                       # noqa: BLE001
            tf_note = type(exc).__name__

        print('---- 第 %d/%d 个 %.0fs 窗口 ----' % (rnd, args.rounds, args.window))
        missing = []
        for topic, _, kind, req, why in CHECKS:
            n = node.counts[topic]
            try:
                pub = node.count_publishers(topic)
            except Exception:                          # noqa: BLE001
                pub = -1
            if kind is STREAM:
                good = n > 0
                shown = '%5d 帧 %6.1f Hz' % (n, n / args.window)
            else:
                # 事件式：有发布者就算通，帧数只是"这个窗口里发生了几次"
                good = pub > 0
                shown = 'pub=%d  本窗 %d 次' % (pub, n)
            if req and not good:
                missing.append(topic)
            mark = '✅' if good else ('🔴' if req else '⚪')
            print('  %s %-40s %-22s %s%s'
                  % (mark, topic, shown, '[必需] ' if req else '', why))
        print('  %s %-40s %-22s [必需] map->base'
              % ('✅' if tf_ok else '🔴',
                 'TF %s->%s' % (args.map_frame, args.base_frame), tf_note))

        ok = (not missing) and tf_ok
        if node.last_state:
            print('  探索状态: %s' % node.last_state)
        if ok:
            break
        if rnd < args.rounds:
            print('  必需项还缺: %s%s —— 再等一个窗口（DDS 发现可能还没收敛）'
                  % (' '.join(missing), '' if tf_ok else ' TF'))

    # 通过之后仍要把"会少哪几列"讲清楚，不能让它看起来一切完美。
    if ok:
        st = node.last_state or ''
        if 'state=PAUSED' in st or 'state=IDLE' in st:
            print()
            print('⚠️  协调器当前不在派发（%s）。'
                  % st.split(' ')[0] if st else '')
            print('    话题都通、录制会正常开始，但只有真的派发出目标才会产生轮次。')
            print('    要么在另一个终端恢复探索，要么手动遥控（那种情况下轮次为 0，'
                  'bag 仍然完整可用）。')
        if node.counts['/cmd_vel'] == 0:
            print('⚠️  /cmd_vel 本窗 0 次：机器人此刻静止。这是正常的，'
                  '不代表写通路有问题。')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
