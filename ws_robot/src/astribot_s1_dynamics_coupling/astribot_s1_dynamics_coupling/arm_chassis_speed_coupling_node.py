#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：臂-底盘动力学耦合动态调速节点——独立、无侵入的中间控制层，插在
"世界坐标系速度输出之后、gz-sim VelocityControl 插件接收之前"这个位置：

    Nav2/自主巡游 → [既有的 body→world 转换+安全监控节点] → (中间话题) →
    【本节点：根据双臂展开幅度/关节角速度算 0~1.0 动态限速系数，缩放+平滑】 →
    真正的 /cmd_vel → gz-sim VelocityControl 插件

不修改 VelocityControl 驱动架构、坐标变换/SLAM/雷达滤波/TF/Nav2核心逻辑、既有
安全监控与静态臂限速节点的任何代码——接入方式是在 launch 里把上游节点（比如
`cmd_vel_body_to_world_node`）本来就有的 `output_topic` 参数**覆盖**成一个中间
话题名，本节点订阅这个中间话题、发布到真正的 `/cmd_vel`，全程不touch既有文件。

!!! 关于"静默失效、保持原生全速运动"这一条要求的如实说明（不夸大能力边界）!!!：
本节点在**运行时逻辑层面**做到了"任何单次异常都不阻塞、不降速为0，直接按1.0（不
限速）放行"——每一步计算（关节数据解析、系数计算、平滑滤波、发布）都包在
try/except 里，任何一步出错就退回到"这一帧不缩放、原样转发"，不会让机器人卡死。
但如果本节点自己的**进程整体崩溃退出**，因为它是这条转发链路里"中间话题→真正
/cmd_vel"唯一的桥接者，进程一旦不在了，新的速度指令确实没法再送到真正的/cmd_vel
——这是"中间人"式无侵入架构在拓扑上无法回避的一个真实限制（除非直接改动上游节点
的输出目标，但那样就不是"无侵入"了）。这里用 launch 里的 `respawn=True` 做进程级
兜底（挂了自动重启，短暂中断后恢复，不是"零中断保持全速"），配合上面的运行时
"任何异常都不限速"逻辑，两层加起来是本方案实际能提供的"静默失效"保障，如实记录，
不假装是完美的、任何情况下都零感知的故障转移。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


# 跟 astribot_s1_navigation/arm_speed_limiter_node.py 里用的是同一套命名规范
# （astribot_arm_{left,right}_joint_{1..7}），但这是本包自己独立维护的一份列表，
# 不依赖、不import那个包，保持两个包完全解耦（一个是Nav2官方speed_limit机制的
# 粗粒度静态限速，一个是本包更精细的连续动态耦合限速，互相独立、不共享代码）。
ARM_JOINT_NAMES = (
    [f'astribot_arm_left_joint_{i}' for i in range(1, 8)] +
    [f'astribot_arm_right_joint_{i}' for i in range(1, 8)]
)


class ArmChassisSpeedCouplingNode(Node):

    def __init__(self):
        super().__init__('arm_chassis_speed_coupling_node')

        # ---- 话题 ----
        self.declare_parameter('input_topic', '/cmd_vel_pre_arm_coupling')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')

        # ---- 展开幅度限速曲线 ----
        # 双臂14个关节里，任意一个关节相对"收纳姿态"的偏差(rad)达到这个值，
        # 就认为展开幅度维度已经"满量程"（貢献1.0的活跃度）。
        self.declare_parameter('folded_reference_rad', [0.0] * len(ARM_JOINT_NAMES))
        self.declare_parameter('extension_full_rad', 1.2)

        # ---- 运动速率限速曲线 ----
        # 任意一个关节的角速度(rad/s)绝对值达到这个值，就认为速率维度已经"满量程"。
        self.declare_parameter('velocity_full_rad_s', 2.0)

        # ---- 限速系数计算/平滑 ----
        # 展开幅度和运动速率两个维度取更严格(更慢)的那个，不是加权平均——宁可
        # 保守，不要因为"平均出来还凑合"而放过一个真正剧烈的运动。
        self.declare_parameter('min_speed_scale', 0.15)  # 满量程活跃度时的限速系数下限
        # 系数本身做一阶低通平滑，避免关节角度抖动/传感器噪声导致底盘速度一顿一顿。
        self.declare_parameter('scale_smoothing_alpha', 0.25)

        # ---- 数据陈旧/异常降级 ----
        # /joint_states 超过这么久没更新，判定为"数据陈旧"（比如硬件掉线/节点没启动），
        # 自动降级为固定安全低速，而不是不限速地放行（也不是直接停机——不中断导航任务）。
        self.declare_parameter('joint_state_timeout_sec', 0.5)
        self.declare_parameter('degraded_scale', 0.3)

        # ---- 日志 ----
        self.declare_parameter('log_throttle_sec', 2.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_vel_callback, 10)
        self.create_subscription(JointState, joint_states_topic, self.joint_state_callback, 10)

        self.current_scale = 1.0          # 平滑后的动态限速系数，joint_state_callback里更新
        self.last_joint_state_time = None  # 用 self.get_clock() 的时间戳，不是wall clock
        self._last_log_time = None

        self.get_logger().info(
            'arm_chassis_speed_coupling_node 已启动：订阅 %s(车体真正要下发的world系'
            'Twist) + %s(双臂关节状态)，根据展开幅度(阈值%.2frad)/运动速率'
            '(阈值%.2frad/s)算连续0~1.0限速系数(下限%.2f)，平滑后缩放，发到 %s。'
            '/joint_states 超过%.1fs未更新自动降级到固定%.2f限速。' % (
                input_topic, joint_states_topic,
                self.get_parameter('extension_full_rad').value,
                self.get_parameter('velocity_full_rad_s').value,
                self.get_parameter('min_speed_scale').value,
                output_topic,
                self.get_parameter('joint_state_timeout_sec').value,
                self.get_parameter('degraded_scale').value))

    def joint_state_callback(self, msg: JointState):
        # !!! 任何单次异常都不能让这个回调整体崩溃/退出——按文件头部"静默失效"
        # 说明，出错时保持上一次已知的 current_scale 不变（不是重置成1.0，因为
        # 出错时机器人还是应该保持之前的判断，不能因为这一帧解析失败就突然放开
        # 限速；也不是直接判定成"最危险"，避免噪声/瞬时错误造成不必要的急停）。
        try:
            name_to_pos = dict(zip(msg.name, msg.position))
            has_velocity = len(msg.velocity) == len(msg.name)
            name_to_vel = dict(zip(msg.name, msg.velocity)) if has_velocity else {}

            reference = self.get_parameter('folded_reference_rad').value
            extension_full = self.get_parameter('extension_full_rad').value
            velocity_full = self.get_parameter('velocity_full_rad_s').value

            extension_ratio = 0.0
            velocity_ratio = 0.0
            for joint_name, ref in zip(ARM_JOINT_NAMES, reference):
                pos = name_to_pos.get(joint_name)
                if pos is not None and extension_full > 1e-6:
                    r = abs(pos - ref) / extension_full
                    extension_ratio = max(extension_ratio, min(1.0, r))
                vel = name_to_vel.get(joint_name)
                if vel is not None and velocity_full > 1e-6:
                    r = abs(vel) / velocity_full
                    velocity_ratio = max(velocity_ratio, min(1.0, r))

            # 两个维度取更严格(活跃度更高→限速更多)的那个，不做加权平均。
            activity = max(extension_ratio, velocity_ratio)
            min_scale = self.get_parameter('min_speed_scale').value
            raw_scale = 1.0 - activity * (1.0 - min_scale)
            raw_scale = max(min_scale, min(1.0, raw_scale))

            alpha = self.get_parameter('scale_smoothing_alpha').value
            self.current_scale = alpha * raw_scale + (1 - alpha) * self.current_scale
            self.last_joint_state_time = self.get_clock().now()

            self._throttled_log(activity, extension_ratio, velocity_ratio)
        except Exception as exc:  # noqa: BLE001 - 按文件头部说明，单帧异常绝不能扩散
            self.get_logger().error(
                '解析/joint_states出错，保持上一次限速系数不变：%s' % exc)

    def _throttled_log(self, activity, extension_ratio, velocity_ratio):
        now = self.get_clock().now()
        throttle = self.get_parameter('log_throttle_sec').value
        if (self._last_log_time is not None and
                (now - self._last_log_time).nanoseconds < throttle * 1e9):
            return
        self._last_log_time = now
        if activity > 0.05:
            self.get_logger().info(
                '机械臂活跃度=%.2f(展开%.2f/速率%.2f) → 限速系数=%.2f' %
                (activity, extension_ratio, velocity_ratio, self.current_scale))

    def cmd_vel_callback(self, msg: Twist):
        # !!! 核心"静默失效"逻辑：任何异常都 fall back 到 1.0（原样转发，不限速），
        # 不是 fall back 到 0（那样反而会让机器人在异常时突然停住，比"不限速"更危险，
        # 跟任务书"节点异常不影响基础运动能力"的要求相反）。
        try:
            scale = self._get_effective_scale()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('计算限速系数出错，本帧不限速直接转发：%s' % exc)
            scale = 1.0

        try:
            out = Twist()
            out.linear.x = msg.linear.x * scale
            out.linear.y = msg.linear.y * scale
            out.angular.z = msg.angular.z * scale
            self.cmd_pub.publish(out)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('发布缩放后的cmd_vel出错，原样转发本帧：%s' % exc)
            self.cmd_pub.publish(msg)

    def _get_effective_scale(self):
        """结合 /joint_states 陈旧检测，返回当前应该使用的限速系数。"""
        if self.last_joint_state_time is None:
            # 从没收到过关节状态(节点刚启动、或机械臂话题一直没上线)——按"异常场景
            # 处理规则"要求，降级到固定安全低速，不是不限速也不是完全停机。
            return self.get_parameter('degraded_scale').value

        timeout = self.get_parameter('joint_state_timeout_sec').value
        age_sec = (self.get_clock().now() - self.last_joint_state_time).nanoseconds / 1e9
        if age_sec > timeout:
            self.get_logger().warn(
                '/joint_states 已 %.2fs 未更新(超过阈值%.2fs)，降级到固定限速 %.2f' %
                (age_sec, timeout, self.get_parameter('degraded_scale').value),
                throttle_duration_sec=2.0)
            return self.get_parameter('degraded_scale').value

        return self.current_scale


def main(args=None):
    rclpy.init(args=args)
    node = ArmChassisSpeedCouplingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
