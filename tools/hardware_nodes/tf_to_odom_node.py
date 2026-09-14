#!/usr/bin/env python3
"""从 TF 反推 /odom —— 探索链路里 /odom 的唯一实机来源。

为什么不用 astribot_trajectory_bridge 的 chassis_odom_node：
  那个节点跑在 SDK 会话里，而 SDK 初始化会停在"控制权"交互提示上
  （实测另有用户持有控制权，进程能起但一帧不发）。而 /odom 在探索链路里
  只有两个用途：调度器判"驻留稳定"的速度，以及 nav2 行为树的速度反馈 ——
  两者都不需要轮式里程计的原始精度，SLAM 位姿求差分足够。

坐标口径：odom 与 map 在本机是单位静态变换（见 TF 方案甲），
  所以直接把 map->astribot_torso_base 当成 odom->base 发出去，
  child_frame_id 用真实的 base frame（本机没有 base_link）。

速度是位姿差分，不是测量值：静止时会有 SLAM 抖动量级的噪声，
  调度器的 settle_speed=0.05 阈值远大于它。
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException


class TfToOdom(Node):
    def __init__(self):
        super().__init__('tf_to_odom_node')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('base_frame', 'astribot_torso_base')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('odom_topic', '/odom')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.src = self.get_parameter('source_frame').value
        self.base = self.get_parameter('base_frame').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Odometry, self.get_parameter('odom_topic').value, q)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.prev = None          # (t, x, y, yaw)
        self.warned = False
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        try:
            tr = self.buf.lookup_transform(self.src, self.base, rclpy.time.Time())
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            if not self.warned:
                self.get_logger().warn('%s->%s 暂时查不到(%s)，TF 树建立后自动恢复；此告警只打一次'
                                       % (self.src, self.base, type(e).__name__))
                self.warned = True
            return

        t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        qq = tr.transform.rotation
        yaw = math.atan2(2.0 * (qq.w * qq.z + qq.x * qq.y),
                         1.0 - 2.0 * (qq.y * qq.y + qq.z * qq.z))

        m = Odometry()
        m.header.stamp = tr.header.stamp
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.position.z = tr.transform.translation.z
        m.pose.pose.orientation = qq

        if self.prev is not None:
            dt = t - self.prev[0]
            if dt > 1e-3:
                dx, dy = x - self.prev[1], y - self.prev[2]
                c, s = math.cos(-yaw), math.sin(-yaw)
                m.twist.twist.linear.x = (dx * c - dy * s) / dt
                m.twist.twist.linear.y = (dx * s + dy * c) / dt
                dyaw = math.atan2(math.sin(yaw - self.prev[3]), math.cos(yaw - self.prev[3]))
                m.twist.twist.angular.z = dyaw / dt
        self.prev = (t, x, y, yaw)
        self.pub.publish(m)


def main():
    rclpy.init()
    n = TfToOdom()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
