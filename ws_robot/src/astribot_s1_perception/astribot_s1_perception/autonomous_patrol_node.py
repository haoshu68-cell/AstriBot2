#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：自主巡游建图节点——不依赖 Nav2 全套（本环境虽然装了 nav2-bringup，
但要接入完整代价地图/规划器/行为树需要额外一大批参数配置，超出"让机器人动起来把
仓库探索一遍建好图"这个具体需求的范围，这里用一个轻量级的反应式漫游策略：

    每个控制周期读一次融合后的 360° /scan，找到"最开阔的方向"（该方向上激光测距最远），
    因为本机器人是 X 型布局全向轮底盘，不需要像差速轮那样先转向再前进——
    直接把 linear.x/y 设成指向那个最开阔方向的分量即可，同时叠加一个缓慢的持续自转
    （扩大扫到的角度范围、帮助建图），前方太近有障碍物时降速/纯自转避让。

    这不是频率意义上的"最优路径探索"（不会像 Nav2 explore_lite 那样跟踪未知区域边界），
    但对"让机器人动起来、把仓储环境逛一遍、让 SLAM Toolbox 有更多视角的扫描数据"这个
    目标来说足够了，且不需要额外安装/调试 Nav2 costmap+planner+behavior tree 那一整套。
    如果之后想升级成真正的前沿探索(frontier exploration)，可以在这个节点的位置换成
    `nav2_bringup` + 一个 frontier exploration 包（apt 里没有 ros-humble-explore-lite，
    需要自己源码编译），当前实现里预留了同样的 /cmd_vel 输出接口，替换很容易。

    !!! 实测踩坑记录（安全阀，非常重要）!!!：底盘实际驱动用的是 gz-sim 的
    VelocityControl 系统插件（直接对模型本体设定线速度/角速度，绕开了小尺寸轮子摩擦力学
    在本机物理引擎下的数值失效问题，见 astribot_s1.gazebo.xacro 里的详细说明），
    但这种"直接设定速度"的方式有个副作用：它不太理会真实的接触碰撞力——一旦机器人
    在巡游过程中撞上仓储货架之类的障碍物（雷达只装在躯干高度，可能没探测到伸出去的
    机械臂在别的高度蹭到货架），插件仍然按指令强行输出速度，和碰撞求解器的反作用力
    互相顶牛，实测出现过底盘位姿 z 坐标不断爬升、姿态四元数出现异常横滚/俯仰分量
    （相当于"飞起来/翻滚"）且不会自己恢复的情况。因为这是物理引擎数值层面的异常，
    仅靠 ROS 节点没法在事后把机器人"拽回地面"，这里加了一个安全监控：持续核对
    /odom 的 z 高度和姿态四元数，一旦明显偏离正常站立状态，立即持续下发零速度
    （防止巡游逻辑继续添乱、让情况更糟），并且大声报错提示需要人工重新
    `ros2 launch ...` 把机器人重新生成一遍——这不是"自动恢复"，是"止损"。
    同时把默认的巡游速度调低、安全距离调大，降低触发这类碰撞的概率
    （工程上的缓解措施，不是把根因彻底修复——更彻底的修复需要让避障逻辑感知机械臂的
    展开范围，或者把驱动机制换成"轮子摩擦力学真实生效"的方案，见 gazebo.xacro 里的
    後续优化建议，属于本方案范围之外的进一步工作）。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class AutonomousPatrolNode(Node):

    def __init__(self):
        super().__init__('autonomous_patrol_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('control_period', 0.2)
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('yaw_rate', 0.12)
        self.declare_parameter('front_safety_distance', 0.9)
        self.declare_parameter('front_arc_deg', 70.0)
        self.declare_parameter('cornered_distance', 1.1)
        self.declare_parameter('critical_stop_distance', 0.4)
        self.declare_parameter('max_target_range', 6.0)
        self.declare_parameter('smoothing_alpha', 0.25)
        self.declare_parameter('normal_height', 0.134)
        self.declare_parameter('max_height_deviation', 0.06)
        self.declare_parameter('max_tilt_rad', 0.12)  # 约7°，比早期版本大幅收紧

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_callback, qos)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        self.latest_scan = None
        self.vx_filt = 0.0
        self.vy_filt = 0.0
        self.wz_filt = 0.0
        self.safety_tripped = False
        self.current_yaw = 0.0  # 从 /odom 持续更新，见下面 world-frame 换算说明

        period = self.get_parameter('control_period').value
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            '自主巡游节点已启动：读取 %s，反应式漫游 + 缓慢自转扩大扫描覆盖范围，输出到 %s；'
            '同时监控 %s 做异常姿态/高度的安全止损。' %
            (self.get_parameter('scan_topic').value, self.get_parameter('cmd_vel_topic').value,
             self.get_parameter('odom_topic').value))

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if self.safety_tripped:
            return
        z = msg.pose.pose.position.z
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        normal_height = self.get_parameter('normal_height').value
        max_dev = self.get_parameter('max_height_deviation').value
        max_tilt = self.get_parameter('max_tilt_rad').value

        if abs(z - normal_height) > max_dev or abs(roll) > max_tilt or abs(pitch) > max_tilt:
            self.safety_tripped = True
            self.get_logger().error(
                '!!! 安全监控触发止损：检测到异常姿态(z=%.3f, roll=%.3f, pitch=%.3f)，'
                '大概率是巡游中撞上了障碍物导致仿真物理异常（不是自动能恢复的问题）。'
                '已停止自主巡游节点继续下发速度指令，请检查 Gazebo 画面，'
                '必要时重新执行 ros2 launch 把机器人重新生成一遍。' % (z, roll, pitch))

    def control_loop(self):
        if self.safety_tripped:
            self.cmd_pub.publish(Twist())
            return

        msg = self.latest_scan
        if msg is None:
            return

        ranges = np.array(msg.ranges, dtype=np.float64)
        n = ranges.shape[0]
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        if not np.any(valid):
            self._publish_smoothed(0.0, 0.0, self.get_parameter('yaw_rate').value)
            return

        max_target_range = self.get_parameter('max_target_range').value
        capped_ranges = np.where(valid, np.minimum(ranges, max_target_range), 0.0)
        best_idx = int(np.argmax(capped_ranges))
        best_range = capped_ranges[best_idx]
        best_angle = angles[best_idx]

        front_arc = math.radians(self.get_parameter('front_arc_deg').value) / 2.0
        angle_diff = np.mod(angles - best_angle + math.pi, 2 * math.pi) - math.pi
        travel_mask = valid & (np.abs(angle_diff) <= front_arc)
        travel_min = float(np.min(ranges[travel_mask])) if np.any(travel_mask) else float('inf')

        any_min = float(np.min(ranges[valid]))

        safety_distance = self.get_parameter('front_safety_distance').value
        cornered_distance = self.get_parameter('cornered_distance').value
        max_speed = self.get_parameter('max_linear_speed').value
        yaw_rate = self.get_parameter('yaw_rate').value
        critical_distance = self.get_parameter('critical_stop_distance').value

        if any_min < critical_distance:
            vx, vy, wz = 0.0, 0.0, yaw_rate
        elif best_range < cornered_distance:
            vx, vy, wz = 0.0, 0.0, yaw_rate
        else:
            speed = max_speed
            if travel_min < safety_distance:
                speed *= max(0.0, travel_min / safety_distance)
            world_angle = best_angle + self.current_yaw
            vx = math.cos(world_angle) * speed
            vy = math.sin(world_angle) * speed
            wz = yaw_rate * 0.5

        self._publish_smoothed(vx, vy, wz)

    def _publish_smoothed(self, vx, vy, wz):
        alpha = self.get_parameter('smoothing_alpha').value
        self.vx_filt = alpha * vx + (1 - alpha) * self.vx_filt
        self.vy_filt = alpha * vy + (1 - alpha) * self.vy_filt
        self.wz_filt = alpha * wz + (1 - alpha) * self.wz_filt

        cmd = Twist()
        cmd.linear.x = self.vx_filt
        cmd.linear.y = self.vy_filt
        cmd.angular.z = self.wz_filt
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop_cmd = Twist()
            node.cmd_pub.publish(stop_cmd)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
