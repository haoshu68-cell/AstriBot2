#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：Nav2 输出的车体系(base_frame, astribot_torso_base) cmd_vel → world 系转换，
是接入 Nav2 时**必须**存在的一个适配层，不是可选的美化步骤。

!!! 关键背景（不是纸面推演，是本 session 实测+反查二进制确认过的真实坑）!!!：
本机器人底盘运动完全由 gz-sim 的 `VelocityControl` 系统插件直接接管（见
`astribot_s1_description/urdf/astribot_s1.gazebo.xacro`），用 `strings` 反查
`libignition-gazebo6-velocity-control-system.so` 确认它内部的速度分量组件类型叫
`WorldLinearVelocityCmdTag`/`WorldAngularVelocityCmdTag`——也就是说这个插件把
`/cmd_vel` 的 `linear.x/y` **当 world 系分量直接用**，不是移动机器人 cmd_vel 常见的
车体系语义。而 Nav2 的标准控制器（RegulatedPurePursuit/MPPI/DWB）全部按 ROS 惯例，
在车体系(base_frame)下发布 Twist（这是 ROS 生态里几乎所有移动机器人 cmd_vel 的默认
约定，Nav2 没有例外）。如果把 Nav2 的输出直接接到真实 `/cmd_vel`，车身一边被 Nav2
持续要求转向(角速度非零)，一边把车体系速度向量误当 world 系向量使用，一整圈转下来
推力方向连续扫过整个圆周，效果就是"原地打转、几乎不挪窝"——这正是本 session 早前修复
`autonomous_patrol_node` 时踩过、也是用户最初反馈"底盘移动时机器人会倾倒/走不动"这个
问题背后的同一类根因。这里复用 `autonomous_patrol_node.py` 里验证过的换算模式：
`world_angle = body_angle + current_yaw`（yaw 从 /odom 持续更新），对 wz（绕Z轴角速度）
不需要转换——车体系和world系共享同一个Z轴，yaw角速度标量在两个系里数值相同。

同时复用 `autonomous_patrol_node.py` 里验证过的"基于/odom的姿态异常安全监控"：
z高度/roll/pitch超出阈值时，判定发生了仿真物理异常（大概率是撞到障碍物），
立即停止转发 Nav2 的速度指令、持续下发零速度，并报错——这样 Nav2 导航路径下机器人
享有跟自主巡游模式同等级别的安全网，不是"接了Nav2以后没人管姿态异常"。
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class CmdVelBodyToWorldNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_body_to_world_node')

        self.declare_parameter('input_topic', '/cmd_vel_nav_body')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        # ---- 安全监控阈值：与 autonomous_patrol_node.py 保持一致的判定标准 ----
        self.declare_parameter('normal_height', 0.134)
        self.declare_parameter('max_height_deviation', 0.06)
        self.declare_parameter('max_tilt_rad', 0.12)  # 约7°

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.current_yaw = 0.0
        self.safety_tripped = False

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(
            'cmd_vel_body_to_world_node 已启动：订阅 %s(车体系, Nav2输出) + %s(取航向角)，'
            '换算成 world 系后发到 %s（gz-sim VelocityControl 插件按 world 系解释速度，'
            '这个转换是必须的，不是可选优化）。同时监控 /odom 做异常姿态安全止损。' %
            (input_topic, odom_topic, output_topic))

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
        sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)

        normal_height = self.get_parameter('normal_height').value
        max_dev = self.get_parameter('max_height_deviation').value
        max_tilt = self.get_parameter('max_tilt_rad').value

        if abs(z - normal_height) > max_dev or abs(roll) > max_tilt or abs(pitch) > max_tilt:
            self.safety_tripped = True
            self.get_logger().error(
                '!!! 安全监控触发止损：检测到异常姿态(z=%.3f, roll=%.3f, pitch=%.3f)，'
                '停止转发 Nav2 的速度指令、持续下发零速度。请检查 Gazebo 画面，'
                '必要时重新执行 ros2 launch 把机器人重新生成一遍。' % (z, roll, pitch))
            self.cmd_pub.publish(Twist())

    def cmd_vel_callback(self, msg: Twist):
        if self.safety_tripped:
            self.cmd_pub.publish(Twist())
            return

        # msg.linear.x/y 是 Nav2 按车体系(astribot_torso_base)算出来的速度分量；
        # wz(角速度)不受坐标系影响，直接照抄。
        body_speed = math.hypot(msg.linear.x, msg.linear.y)
        if body_speed > 1e-6:
            body_angle = math.atan2(msg.linear.y, msg.linear.x)
            world_angle = body_angle + self.current_yaw
            vx = math.cos(world_angle) * body_speed
            vy = math.sin(world_angle) * body_speed
        else:
            vx = 0.0
            vy = 0.0

        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = msg.angular.z
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBodyToWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发一个零速度：VelocityControl 是"设定即保持"，不会因为没有新消息自动归零。
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
