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
import time

from rclpy.clock import Clock, ClockType

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from astribot_s1_navigation.posture_monitor_policy import (
    ACT_COLLECTING,
    ACT_TRIP,
    describe_monitor_state,
    evaluate_posture,
)


class CmdVelBodyToWorldNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_body_to_world_node')

        self.declare_parameter('input_topic', '/cmd_vel_nav_body')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('normal_height', 0.134)
        self.declare_parameter('max_height_deviation', 0.06)
        self.declare_parameter('max_tilt_rad', 0.12)  # 约7°
        self.declare_parameter('enable_body_to_world', False)
        self.declare_parameter('enable_posture_monitor', True)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('cmd_timeout_sec', 0.5)

        for key in ('odom_timeout_sec', 'cmd_timeout_sec'):
            value = float(self.get_parameter(key).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(key + ' must be finite and positive')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self._last_odom_received = None
        self._last_odom_stamp = None
        self._last_cmd_received = None
        self._odom_valid = False
        self.current_yaw = 0.0
        self.safety_tripped = False
        self.posture_enabled = self.get_parameter('enable_posture_monitor').value
        self._attitude_samples = []

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback,
                                 qos_profile_sensor_data)

        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._watchdog_timer = self.create_timer(0.05, self._watchdog, clock=self._watchdog_clock)

        self.get_logger().info(
            'cmd_vel_body_to_world_node 已启动：订阅 %s(车体系, Nav2输出) + %s，'
            '转发到 %s。body→world旋转=%s（力矩闭环底盘吃车体系，默认关闭旋转，'
            '详见cmd_vel_callback注释）。%s' %
            (input_topic, odom_topic, output_topic,
             'ON(VelocityControl架构)' if self.get_parameter('enable_body_to_world').value
             else 'OFF(直通)',
             describe_monitor_state(self.posture_enabled, False, False)))

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        values = (q.x, q.y, q.z, q.w, msg.pose.pose.position.z)
        norm = sum(v * v for v in values[:4])
        self._odom_valid = all(math.isfinite(v) for v in values) and abs(norm - 1.0) < 0.01
        self._last_odom_received = time.monotonic()
        self._last_odom_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if not self._odom_valid or not self._odom_ready():
            if self.posture_enabled or self.get_parameter('enable_body_to_world').value:
                self.cmd_pub.publish(Twist())
            return
        self.current_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if not self.posture_enabled:
            return
        if self.safety_tripped:
            return

        z = msg.pose.pose.position.z
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)

        if len(self._attitude_samples) < 64:
            self._attitude_samples.append((z, roll, pitch))
        action, reason = evaluate_posture(
            True, self._attitude_samples, z, roll, pitch,
            self.get_parameter('normal_height').value,
            self.get_parameter('max_height_deviation').value,
            self.get_parameter('max_tilt_rad').value)

        if action == ACT_COLLECTING:
            return
        if action == ACT_TRIP:
            self.safety_tripped = True
            self.get_logger().error(
                '%s 触发项：%s。（z=%.3f roll=%.3f pitch=%.3f）'
                'safety_tripped 没有复位路径，需重启本节点才能恢复转发。'
                % (describe_monitor_state(True, False, True), reason,
                   z, roll, pitch))
            self.cmd_pub.publish(Twist())

    def _odom_ready(self):
        if not (self.get_parameter('enable_body_to_world').value or self.posture_enabled):
            return True
        if not self._odom_valid or self._last_odom_received is None:
            return False
        timeout = self.get_parameter('odom_timeout_sec').value
        age = (self.get_clock().now() - self._last_odom_stamp).nanoseconds / 1e9
        return 0.0 <= age <= timeout and time.monotonic() - self._last_odom_received <= timeout

    def _watchdog(self):
        if self._last_cmd_received is None:
            return
        if (self.safety_tripped or not self._odom_ready() or
                time.monotonic() - self._last_cmd_received > self.get_parameter('cmd_timeout_sec').value):
            self.cmd_pub.publish(Twist())
            # Stop a lost command once; an idle adapter must not keep overriding other inputs.
            self._last_cmd_received = None

    def cmd_vel_callback(self, msg: Twist):
        self._last_cmd_received = time.monotonic()
        valid = all(math.isfinite(v) for v in (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z))
        if self.safety_tripped or not valid or not self._odom_ready():
            self.cmd_pub.publish(Twist())
            return

        if self.get_parameter('enable_body_to_world').value:
            body_speed = math.hypot(msg.linear.x, msg.linear.y)
            if body_speed > 1e-6:
                body_angle = math.atan2(msg.linear.y, msg.linear.x)
                world_angle = body_angle + self.current_yaw
                vx = math.cos(world_angle) * body_speed
                vy = math.sin(world_angle) * body_speed
            else:
                vx = 0.0
                vy = 0.0
        else:
            vx = msg.linear.x
            vy = msg.linear.y

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
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
