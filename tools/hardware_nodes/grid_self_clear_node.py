#!/usr/bin/env python3
"""把厂商 SLAM 栅格里"机器人自己"这团假障碍清掉，再转发给 nav2 与探索调度器。

## 为什么必须有这一步

厂商的 nav_prob_grid_node 用 /voxel_slam/keyframe_submap 的**原始**关键帧点云
合成占据栅格，链路上没有任何自滤。实测（2026-09-03，机器人静止）：

    /map_scan_filtered 里水平距离 <0.45m 的点共 5 个，
    z 分位 min=0.505 p50=0.546 max=0.554，水平距离 0.122~0.196m
    —— 位置与高度都正好落在机器人自己的躯干柱上。

后果是两处同时被堵死：
  1. /map_scan_filtered_prob 在机器人所在格 = 100，
     探索调度器的红线判据("obstacle 层已占据 ⇒ 物理真堵，禁止脱困")当场触发，
     状态机在 VALIDATING -> PAUSED 之间无限循环，一次目标都发不出去；
  2. nav2 全局图的 static_layer 继承这团，机器人所在格原始代价 = 254，
     规划器必报 "Starting point in lethal space"。

而这团**跟着机器人走**：每个关键帧都在自己脚下重新投 +6 票
（nav_prob_grid.yaml: hit_delta=6 / miss_delta=1），所以它不是一次性的开机残留，
每到一个新位置都会重新长出来。

nav2 自己的 obstacle_layer.footprint_clearing_enabled 实测已经是 True，但救不了：
ObstacleLayer 的 updateCosts 用 updateWithMax 合并，静态层的 254 恒胜。
所以只能在栅格这一层清。

## 清除判据为什么安全

只清"机器人当前(以及刚刚)自身占据的那一小块"。机器人身体所在的格子里不可能有
外部障碍物 —— 如果有，机器人此刻已经在碰撞里了。这不是放宽红线：红线要防的是
"真障碍挡死了还硬要脱困"，而这团根本不是障碍物，是机器人自己。

只清当前位姿不够：机器人往前挪 0.3m 后，刚才那团就变成紧贴屁股的障碍，
把自己关在里面。所以按最近一段轨迹清一条走廊(trail_window_sec)。

半径默认 0.25m —— 比足迹任何方向的半宽都小(正方形四向半宽均 0.31)，
只够盖住实测 0.196m 的自身回波，不会吃掉真实障碍。
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException

FREE = 0


class GridSelfClear(Node):
    def __init__(self):
        super().__init__('grid_self_clear_node')
        self.declare_parameter('input_topic', '/map_scan_filtered_prob')
        self.declare_parameter('output_topic', '/map_nav')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'astribot_torso_base')
        self.declare_parameter('clear_radius_m', 0.25)
        self.declare_parameter('trail_window_sec', 20.0)
        self.declare_parameter('trail_sample_dist_m', 0.10)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('log_period_sec', 10.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.radius = float(self.get_parameter('clear_radius_m').value)
        self.window = float(self.get_parameter('trail_window_sec').value)
        self.sample_d = float(self.get_parameter('trail_sample_dist_m').value)
        self.occ_th = int(self.get_parameter('occupied_threshold').value)
        log_period = float(self.get_parameter('log_period_sec').value)

        self.trail = []          # [(t, x, y)]
        self.buf = Buffer()
        self.tf_listener = TransformListener(self.buf, self)
        self.tf_warned = False
        self.n_in = 0
        self.n_cleared_last = 0
        self.n_cleared_total = 0

        # 输出 QoS 刻意与输入一致(RELIABLE + VOLATILE)：
        # nav2 静态层那侧必须配 map_subscribe_transient_local: False，
        # 探索调度器那侧必须配 map_transient_local: false。两处口径要与这里一致。
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('output_topic').value, q)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('input_topic').value, self.cb, q)
        self.create_timer(log_period, self.log_tick)

    def robot_xy(self):
        try:
            tr = self.buf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            if not self.tf_warned:
                self.get_logger().warn(
                    '%s->%s 暂时查不到(%s)，此前的帧原样转发；此告警只打一次'
                    % (self.map_frame, self.base_frame, type(e).__name__))
                self.tf_warned = True
            return None
        return (tr.transform.translation.x, tr.transform.translation.y)

    def cb(self, msg):
        self.n_in += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        xy = self.robot_xy()
        if xy is not None:
            if not self.trail or math.hypot(xy[0] - self.trail[-1][1],
                                            xy[1] - self.trail[-1][2]) >= self.sample_d:
                self.trail.append((now, xy[0], xy[1]))
            else:
                # 原地不动也要把时间戳续上，否则站久了自己那格会过期变回障碍
                self.trail[-1] = (now, self.trail[-1][1], self.trail[-1][2])
            self.trail = [s for s in self.trail if now - s[0] <= self.window]

        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        data = list(msg.data)

        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        w, h = msg.info.width, msg.info.height
        rad_cells = int(math.ceil(self.radius / res)) if res > 0.0 else 0
        cleared = 0
        for (_, px, py) in self.trail:
            cx = int((px - ox) / res)
            cy = int((py - oy) / res)
            for j in range(cy - rad_cells, cy + rad_cells + 1):
                if j < 0 or j >= h:
                    continue
                for i in range(cx - rad_cells, cx + rad_cells + 1):
                    if i < 0 or i >= w:
                        continue
                    # 圆形而不是方框：方框的角比半径远，会多吃 41% 的面积
                    if math.hypot((i - cx) * res, (j - cy) * res) > self.radius:
                        continue
                    k = j * w + i
                    # 只把"占据"改成"自由"。未知(-1)保持未知 ——
                    # 把未知改成自由会凭空吃掉前沿，探索会少走地方。
                    if data[k] >= self.occ_th:
                        data[k] = FREE
                        cleared += 1
        out.data = data
        self.n_cleared_last = cleared
        self.n_cleared_total += cleared
        self.pub.publish(out)

    def log_tick(self):
        self.get_logger().info(
            '转发 %d 帧；最近一帧清掉自身假障碍 %d 格，累计 %d 格；轨迹样本 %d 个(窗口 %.0fs, 半径 %.2fm)'
            % (self.n_in, self.n_cleared_last, self.n_cleared_total,
               len(self.trail), self.window, self.radius))


def main():
    rclpy.init()
    n = GridSelfClear()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
