#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：双臂展开时给 Nav2 的 velocity_smoother 发限速指令（任务书"检测机械臂处于伸展
状态时限制最大运动速度"这一条安全逻辑的落地）。

!!! 如实说明这是什么、不是什么 !!!：这里用的是**启发式判定**——检查左右臂各7个关节的
当前角度和一个"收纳姿态"参考角度数组的最大偏差，超过阈值就认为"手臂展开了"，跟着降速。
这不是精确的几何碰撞检测（不知道机械臂末端在三维空间里到底伸到哪个位置、离最近障碍物
多远），只是一个"关节明显偏离收纳姿态 → 保守降速"的粗粒度安全阀，足够覆盖"双臂大幅张开
时降速慢慢挪"这个基本需求，但不能替代真正的全身碰撞检测。任务书里这一条本身也是标注成
"可选方案"，更精确的做法是用 `nav2_collision_monitor` 订阅机械臂末端 TF、动态改变机器人
的碰撞多边形，这里没有做，作为后续可扩展点写在 README 里，不假装当前方案已经是完整解。

Nav2 侧的落地机制：`nav2_velocity_smoother` 支持订阅 `speed_limit_topic`
（默认 `/speed_limit`，消息类型 `nav2_msgs/msg/SpeedLimit`，`percentage=true` 时
`speed_limit` 字段是百分比 0~100），这是 Nav2 官方现成机制，不需要自己改
velocity_smoother 的代码。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav2_msgs.msg import SpeedLimit


# 左右臂关节名，跟 astribot_s1_arm.xacro / arm_left_controller、arm_right_controller
# 里的关节列表完全一致（astribot_arm_{left,right}_joint_{1..7}）。
ARM_JOINT_NAMES = (
    [f'astribot_arm_left_joint_{i}' for i in range(1, 8)] +
    [f'astribot_arm_right_joint_{i}' for i in range(1, 8)]
)


class ArmSpeedLimiterNode(Node):

    def __init__(self):
        super().__init__('arm_speed_limiter_node')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        self.declare_parameter('check_period', 0.5)
        # "收纳姿态"参考角度（rad），跟 ARM_JOINT_NAMES 一一对应，默认全 0——
        # !!! 部署到真机/换了默认收纳姿态时必须重新核对这组数值，不要直接照抄 !!!
        self.declare_parameter('folded_reference_rad', [0.0] * len(ARM_JOINT_NAMES))
        # 任意一个手臂关节偏离收纳姿态超过这个角度(rad)，判定为"展开状态"。
        self.declare_parameter('extended_threshold_rad', 0.5)
        # 判定为展开状态时的限速百分比(0~100)，100=不限速。
        self.declare_parameter('extended_speed_limit_pct', 50.0)

        self.speed_pub = self.create_publisher(
            SpeedLimit, self.get_parameter('speed_limit_topic').value, 10)
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self.joint_state_callback, 10)

        self.last_extended = None  # 只在状态变化时发布+打日志，避免刷屏
        self.get_logger().info(
            'arm_speed_limiter_node 已启动：监控双臂关节角相对收纳姿态的偏差，'
            '超过 %.2f rad 判定为展开状态，限速到 %.0f%%（这是启发式判定，不是精确碰撞'
            '检测，详见文件头部说明）。' % (
                self.get_parameter('extended_threshold_rad').value,
                self.get_parameter('extended_speed_limit_pct').value))

    def joint_state_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        reference = self.get_parameter('folded_reference_rad').value
        threshold = self.get_parameter('extended_threshold_rad').value

        max_dev = 0.0
        for joint_name, ref in zip(ARM_JOINT_NAMES, reference):
            pos = name_to_pos.get(joint_name)
            if pos is None:
                continue  # 这一帧 joint_states 里没有这个关节(比如还没上线)，跳过
            max_dev = max(max_dev, abs(pos - ref))

        is_extended = max_dev > threshold
        if is_extended == self.last_extended:
            return  # 状态没变化，不重复发布(SpeedLimit 是"持续生效直到下一条"的语义)

        self.last_extended = is_extended
        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.percentage = True
        if is_extended:
            limit.speed_limit = self.get_parameter('extended_speed_limit_pct').value
            self.get_logger().warn(
                '检测到机械臂展开(最大关节偏差 %.2f rad > 阈值)，限速到 %.0f%%' %
                (max_dev, limit.speed_limit))
        else:
            limit.speed_limit = 0.0  # nav2约定：0.0 表示解除限速，恢复正常速度上限
            self.get_logger().info('机械臂已收纳，解除限速')
        self.speed_pub.publish(limit)


def main(args=None):
    rclpy.init(args=args)
    node = ArmSpeedLimiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
