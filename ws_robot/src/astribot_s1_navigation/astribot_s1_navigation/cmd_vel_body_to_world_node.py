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
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from astribot_s1_navigation.posture_monitor_policy import (
    ACT_COLLECTING,
    ACT_DISABLE_DEGENERATE,
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
        # ---- 安全监控阈值：与 autonomous_patrol_node.py 保持一致的判定标准 ----
        self.declare_parameter('normal_height', 0.134)
        self.declare_parameter('max_height_deviation', 0.06)
        self.declare_parameter('max_tilt_rad', 0.12)  # 约7°
        # 见 cmd_vel_callback 里的详细说明：VelocityControl(world系语义)已被力矩闭环
        # (车体系语义)取代，默认不再做 body→world 旋转，本节点退化为
        # "直通转发 + 姿态安全监控"。换回VelocityControl架构时设回 true。
        self.declare_parameter('enable_body_to_world', False)
        # ---- 姿态监控总开关 ----
        # !!! 实机必须显式给 false !!! 理由（实测，不是推演）：
        # 实机 /odom 是轮式里程计、只暴露 3-DOF，z/roll/pitch **恒等于 0**，
        # 而 normal_height=0.134 是仿真值 -> |0-0.134|=0.134 > 0.06 -> 判为异常姿态。
        # 此前之所以没炸，是因为这个节点的 /odom 订阅用的是默认 RELIABLE QoS，
        # 而实机 /odom 发布者是 BEST_EFFORT，回调**一帧都没执行过**（实测
        # RELIABLE 0 帧 / BEST_EFFORT 704 帧 @50Hz）—— 两个缺陷互相掩盖。
        # 本次把 QoS 修成 sensor_data（对 RELIABLE 发布者同样兼容），
        # 于是缺陷 2 会暴露出来，所以必须同时有这个开关。
        # 默认保持 True 是为了不改变仿真行为。
        self.declare_parameter('enable_posture_monitor', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.current_yaw = 0.0
        self.safety_tripped = False
        #: 姿态监控运行态。degenerate=数据源不携带姿态信息（自动停用）。
        self.posture_enabled = self.get_parameter('enable_posture_monitor').value
        self.posture_degenerate = False
        self._attitude_samples = []

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_vel_callback, 10)
        # !!! /odom 必须用 sensor_data（BEST_EFFORT）!!!
        # 原先是 `..., 10)` 即默认 RELIABLE，而实机 /odom 发布者是 BEST_EFFORT ——
        # 单向不兼容，订阅者**一帧都收不到**，只有一条 WARNING，
        # 而节点活着、发布者数正常，按 pub>0 写的判据一条都发现不了。
        # BEST_EFFORT 订阅者对 RELIABLE 发布者同样兼容，所以仿真侧不受影响。
        self.create_subscription(Odometry, odom_topic, self.odom_callback,
                                 qos_profile_sensor_data)

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
        self.current_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if not self.posture_enabled or self.posture_degenerate:
            return
        if self.safety_tripped:
            return

        z = msg.pose.pose.position.z
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)

        # ---- 判定顺序在 evaluate_posture 里，刻意不写在这里 ----
        # 实机 z=0 同时满足"超限"和"数据源退化"两个判定，顺序直接决定行为。
        # 顺序留在 callback 里就没有任何测试能钉住它 —— 实测过：这里顺序写对了，
        # 但"把顺序调回去"这个变异在 20 条测试下**全部存活**。
        # 挪进纯函数后 TestEvaluatePostureOrdering 才真正拦得住。
        if len(self._attitude_samples) < 64:
            self._attitude_samples.append((z, roll, pitch))
        action, reason = evaluate_posture(
            True, self._attitude_samples, z, roll, pitch,
            self.get_parameter('normal_height').value,
            self.get_parameter('max_height_deviation').value,
            self.get_parameter('max_tilt_rad').value)

        if action == ACT_COLLECTING:
            return
        if action == ACT_DISABLE_DEGENERATE:
            self.posture_degenerate = True
            self.get_logger().error(describe_monitor_state(True, True, False))
            return
        if action == ACT_TRIP:
            self.safety_tripped = True
            self.get_logger().error(
                '%s 触发项：%s。（z=%.3f roll=%.3f pitch=%.3f）'
                'safety_tripped 没有复位路径，需重启本节点才能恢复转发。'
                % (describe_monitor_state(True, False, True), reason,
                   z, roll, pitch))
            self.cmd_pub.publish(Twist())

    def cmd_vel_callback(self, msg: Twist):
        if self.safety_tripped:
            self.cmd_pub.publish(Twist())
            return

        # !!! 力控重构方案后的关键变化（务必读完再改）!!!：
        # 这个 body→world 旋转当初存在的唯一理由是 gz-sim VelocityControl 插件
        # 按 **world 系** 解释速度指令。现在 VelocityControl 已经整体移除，
        # 底盘换成 astribot_s1_chassis_effort_drive 的力矩闭环，它的全向轮
        # 逆解吃的是 **车体系** (vx,vy,wz)——正好就是 Nav2 原生输出的坐标系。
        # 此时如果还做这个旋转，等于把 Nav2 要求的方向额外转了一个航向角 yaw：
        # 机器人正对 x 轴(yaw=0)时看起来正常，一旦转弯就会往错误方向走，导航必然失败。
        # 所以默认 enable_body_to_world=false（直通转发），只保留本节点的
        # z/roll/pitch 安全监控职责。若哪天换回 VelocityControl 那套架构，
        # 把这个参数设回 true 即可，转换代码原样保留、没有删。
        if self.get_parameter('enable_body_to_world').value:
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
        # 退出前发一个零速度做兜底（力矩闭环节点自己也有cmd_vel超时归零逻辑，
        # 但多发一个零速度没有坏处）。
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
