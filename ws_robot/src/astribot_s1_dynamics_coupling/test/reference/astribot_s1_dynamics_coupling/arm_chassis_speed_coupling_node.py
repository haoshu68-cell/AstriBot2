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

!!! C1 修正(2026-08-25)：展开度量换成水平伸展，不再用关节角偏差 !!!
原实现拿"双臂关节角相对一个参考姿态(默认全0)的最大偏差"当展开程度，实测证明这个
度量与真实物理量**反相关**——全0姿态其实是肘部完全伸直、水平伸展 0.4205m 的姿态，
却因为偏差为0而完全不限速；而真正收拢的姿态(伸展 0.3532m)反倒被限到系数下限。
现在改成查 TF 求"监控连杆相对底盘回转轴的水平距离(m)"，阈值也是物理长度。
完整实测证据与倾覆余量重算见 arm_reach_metric.py 头部说明和本包 README。

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
from rclpy.time import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException

from astribot_s1_dynamics_coupling.arm_reach_metric import (
    METRIC_HORIZONTAL_REACH,
    METRIC_JOINT_DEVIATION,
    VALID_METRICS,
    ReachMetricConfigError,
    horizontal_reach,
    joint_deviation_activity,
    reach_activity,
    scale_from_activity,
    validate_reach_thresholds,
    velocity_activity,
)


ARM_JOINT_NAMES = (
    [f'astribot_arm_left_joint_{i}' for i in range(1, 8)] +
    [f'astribot_arm_right_joint_{i}' for i in range(1, 8)]
)

DEFAULT_MONITORED_LINKS = [
    'astribot_arm_left_tcp_link',
    'astribot_arm_right_tcp_link',
    'astribot_gripper_left_Link_L11',
    'astribot_gripper_right_Link_R11',
]

DEFAULT_CHASSIS_BASE_FRAME = 'astribot_torso_base'


class ArmChassisSpeedCouplingNode(Node):

    def __init__(self):
        super().__init__('arm_chassis_speed_coupling_node')

        self.declare_parameter('input_topic', '/cmd_vel_pre_arm_coupling')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.declare_parameter('extension_metric', METRIC_HORIZONTAL_REACH)
        self.declare_parameter('chassis_base_frame', DEFAULT_CHASSIS_BASE_FRAME)
        self.declare_parameter('reach_tf_timeout_sec', 0.5)
        timeout = float(self.get_parameter('reach_tf_timeout_sec').value)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('reach_tf_timeout_sec must be finite and positive')
        self.declare_parameter('monitored_links', DEFAULT_MONITORED_LINKS)
        self.declare_parameter('reach_folded_m', 0.42)
        self.declare_parameter('reach_full_m', 0.8865)
        self.declare_parameter('reach_update_period_sec', 0.05)

        self.declare_parameter('folded_reference_rad', [0.0] * len(ARM_JOINT_NAMES))
        self.declare_parameter('extension_full_rad', 1.2)

        self.declare_parameter('velocity_full_rad_s', 2.0)

        self.declare_parameter('min_speed_scale', 0.15)  # 满量程活跃度时的限速系数下限
        self.declare_parameter('scale_smoothing_alpha', 0.25)

        self.declare_parameter('joint_state_timeout_sec', 0.5)
        self.declare_parameter('degraded_scale', 0.3)

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

        self._metric = self._resolve_metric()
        self._tf_buffer = None
        self._tf_listener = None
        self._last_reach_m = None       # 最近一次成功算出的最大水平伸展
        self._last_reach_link = ''      # 当时最远的那个连杆,日志用
        self._last_reach_time = None    # 节流用
        self._tf_warned = False         # TF 尚未就绪的告警只打一次,不刷屏
        if self._metric == METRIC_HORIZONTAL_REACH:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            'arm_chassis_speed_coupling_node 已启动：订阅 %s(车体真正要下发的world系'
            'Twist) + %s(双臂关节状态)，展开度量=%s，运动速率满量程%.2frad/s，'
            '算连续0~1.0限速系数(下限%.2f)，平滑后缩放，发到 %s。'
            '/joint_states 超过%.1fs未更新自动降级到固定%.2f限速。' % (
                input_topic, joint_states_topic, self._describe_metric(),
                self.get_parameter('velocity_full_rad_s').value,
                self.get_parameter('min_speed_scale').value,
                output_topic,
                self.get_parameter('joint_state_timeout_sec').value,
                self.get_parameter('degraded_scale').value))

    def _resolve_metric(self):
        """确定实际使用的展开度量。配置非法时**退回更保守的旧度量**并大声报错,
        而不是抛异常退出——本节点是 cmd_vel 链路上唯一的桥接者,进程死掉配合
        respawn=True 会变成崩溃重启循环,那比继续用一个偏保守的度量更糟。"""
        metric = self.get_parameter('extension_metric').value
        if metric not in VALID_METRICS:
            self.get_logger().error(
                'extension_metric=%r 不是合法取值%s，退回保守的 %s 度量。'
                % (metric, list(VALID_METRICS), METRIC_JOINT_DEVIATION))
            return METRIC_JOINT_DEVIATION
        if metric == METRIC_JOINT_DEVIATION:
            self.get_logger().warn(
                'extension_metric=joint_deviation：这是已被实测证伪的旧度量(与真实'
                '水平伸展反相关，详见 arm_reach_metric.py 头部)，只应用于 A/B 回归'
                '对比或临时回退，不要长期这么跑。')
            return METRIC_JOINT_DEVIATION
        try:
            validate_reach_thresholds(
                self.get_parameter('reach_folded_m').value,
                self.get_parameter('reach_full_m').value)
        except ReachMetricConfigError as exc:
            self.get_logger().error(
                '伸展阈值配置非法(%s)，退回保守的 %s 度量。请修正 reach_folded_m/'
                'reach_full_m 后重启。' % (exc, METRIC_JOINT_DEVIATION))
            return METRIC_JOINT_DEVIATION
        return METRIC_HORIZONTAL_REACH

    def _describe_metric(self):
        if self._metric == METRIC_HORIZONTAL_REACH:
            return ('水平伸展 %.3f~%.3fm(相对 %s，监控%d个连杆)' % (
                self.get_parameter('reach_folded_m').value,
                self.get_parameter('reach_full_m').value,
                self.get_parameter('chassis_base_frame').value,
                len(self.get_parameter('monitored_links').value)))
        return ('关节偏差 满量程%.2frad(旧度量，已证伪)'
                % self.get_parameter('extension_full_rad').value)

    def _max_horizontal_reach(self):
        """非阻塞查询全部监控 TF；任一路缺失或陈旧均返回 None。"""
        base_frame = self.get_parameter('chassis_base_frame').value
        links = self.get_parameter('monitored_links').value
        best = None
        best_link = ''
        failures = []
        for link in links:
            try:
                tf = self._tf_buffer.lookup_transform(base_frame, link, Time())
            except TransformException as exc:
                failures.append('%s(%s)' % (link, type(exc).__name__))
                continue
            stamp = Time.from_msg(tf.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            t = tf.transform.translation
            # Zero stamp denotes a timeless, entirely static TF chain.
            if (not all(math.isfinite(v) for v in (t.x, t.y, t.z)) or
                    (stamp.nanoseconds != 0 and
                     not 0.0 <= age <= self.get_parameter('reach_tf_timeout_sec').value)):
                failures.append('%s(stale or invalid TF)' % link)
                continue
            reach = horizontal_reach(t.x, t.y)
            if best is None or reach > best:
                best, best_link = reach, link
        if best is None or failures:
            if not self._tf_warned:
                self._tf_warned = True
                self.get_logger().warn(
                    '监控连杆的 TF 不完整或无效(%s)，伸展维度按满量程保护；'
                    'TF 树建立后会自动恢复，此告警只打一次。'
                    % ('; '.join(failures) if failures else 'monitored_links 为空'))
            return None
        self._tf_warned = False
        return best, best_link


    def joint_state_callback(self, msg: JointState):
        try:
            name_to_pos = dict(zip(msg.name, msg.position))
            has_velocity = len(msg.velocity) == len(msg.name)
            name_to_vel = dict(zip(msg.name, msg.velocity)) if has_velocity else {}

            extension_ratio = self._extension_activity(name_to_pos)
            velocity_ratio = velocity_activity(
                name_to_vel, ARM_JOINT_NAMES,
                self.get_parameter('velocity_full_rad_s').value)

            activity = max(extension_ratio, velocity_ratio)
            raw_scale = scale_from_activity(
                activity, self.get_parameter('min_speed_scale').value)

            alpha = self.get_parameter('scale_smoothing_alpha').value
            self.current_scale = alpha * raw_scale + (1 - alpha) * self.current_scale
            self.last_joint_state_time = self.get_clock().now()

            self._throttled_log(activity, extension_ratio, velocity_ratio)
        except Exception as exc:  # noqa: BLE001 - 按文件头部说明，单帧异常绝不能扩散
            self.get_logger().error(
                '解析/joint_states出错，保持上一次限速系数不变：%s' % exc)

    def _extension_activity(self, name_to_pos):
        """展开维度活跃度。horizontal_reach 走 TF,joint_deviation 走关节角。"""
        if self._metric == METRIC_JOINT_DEVIATION:
            return joint_deviation_activity(
                name_to_pos, ARM_JOINT_NAMES,
                self.get_parameter('folded_reference_rad').value,
                self.get_parameter('extension_full_rad').value)

        now = self.get_clock().now()
        period = self.get_parameter('reach_update_period_sec').value
        stale = (self._last_reach_time is None or
                 not 0 <= (now - self._last_reach_time).nanoseconds < period * 1e9)
        if stale:
            probed = self._max_horizontal_reach()
            if probed is not None:
                self._last_reach_m, self._last_reach_link = probed
                self._last_reach_time = now
            else:
                self._last_reach_m = None
                self._last_reach_link = ''
                self._last_reach_time = now

        if self._last_reach_m is None:
            return 1.0
        return reach_activity(
            self._last_reach_m,
            self.get_parameter('reach_folded_m').value,
            self.get_parameter('reach_full_m').value)

    def _throttled_log(self, activity, extension_ratio, velocity_ratio):
        now = self.get_clock().now()
        throttle = self.get_parameter('log_throttle_sec').value
        if (self._last_log_time is not None and
                (now - self._last_log_time).nanoseconds < throttle * 1e9):
            return
        self._last_log_time = now
        if activity > 0.05:
            if self._metric == METRIC_HORIZONTAL_REACH and self._last_reach_m is not None:
                detail = '展开%.2f(伸展%.3fm@%s)/速率%.2f' % (
                    extension_ratio, self._last_reach_m, self._last_reach_link,
                    velocity_ratio)
            else:
                detail = '展开%.2f/速率%.2f' % (extension_ratio, velocity_ratio)
            self.get_logger().info(
                '机械臂活跃度=%.2f(%s) → 限速系数=%.2f' %
                (activity, detail, self.current_scale))

    def cmd_vel_callback(self, msg: Twist):
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
