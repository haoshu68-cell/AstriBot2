#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：双臂展开时给 Nav2 的 velocity_smoother 发限速指令（任务书"检测机械臂处于伸展
状态时限制最大运动速度"这一条安全逻辑的落地）。

!!! C1 修正(2026-08-25)：判据从"关节角偏差 > 0.5rad"换成"水平伸展 > 0.42m" !!!
原判据是"双臂 14 个关节相对全 0 参考姿态的最大偏差超过阈值就算展开"。实测证明它与
真实伸展**反相关**（详见 arm_reach_metric.py 头部的采样数据）：全 0 姿态其实是肘部
完全伸直、水平伸展 0.4205m 的姿态，却因偏差为 0 被判"已收纳"；而真正收拢的姿态反倒
被判"展开"。后果是任何作业姿态都被恒定砍到 50%，再叠乘耦合节点的系数，实测合计
0.075，把 `desired_linear_vel: 0.5` 压到约 0.037 m/s。

现在改成查 TF 求"监控连杆相对底盘回转轴的水平距离"，阈值 `extended_reach_m` 默认
0.64 = 0.42(costmap 的 `robot_radius`) + 0.2163(支撑多边形倾覆力臂)：机械臂**伸出
足迹之外的那一段**长到跟底盘自己的倾覆力臂相当时，才动用这个粗粒度二值保护。
连续精细调速交给 `astribot_s1_dynamics_coupling` 的耦合节点（它从 0.42 就开始线性
介入），本节点定位是**backstop**——耦合节点被关掉或崩溃重启期间兜住真正危险的姿态。

!!! 如实说明这仍然是什么、不是什么 !!!：这仍然是**保守的单点判据**，只看几个监控
连杆原点离回转轴多远，不知道机械臂离最近障碍物多远，也不是精确的全身碰撞检测。
换度量修掉的是"判据与物理量反相关"这个错误，不是把它升级成了完整解。
另外要明确一点：限速**不会缩小机器人的碰撞包络**。实测机械臂水平伸展最大可达
0.8865m，比 `robot_radius: 0.42` 多伸出 0.47m，这部分是规划器完全看不见的真实碰撞
风险，正确机制是姿态相关的动态足迹（`nav2_collision_monitor`），本节点不解决它，
作为独立工作项（C1b）记录在 README，不假装已经覆盖。

Nav2 侧的落地机制：`nav2_velocity_smoother` 支持订阅 `speed_limit_topic`
（默认 `/speed_limit`，消息类型 `nav2_msgs/msg/SpeedLimit`，`percentage=true` 时
`speed_limit` 字段是百分比 0~100），这是 Nav2 官方现成机制，不需要自己改
velocity_smoother 的代码。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav2_msgs.msg import SpeedLimit
from tf2_ros import Buffer, TransformListener, TransformException

from astribot_s1_navigation.arm_reach_metric import (
    METRIC_HORIZONTAL_REACH,
    METRIC_JOINT_DEVIATION,
    VALID_METRICS,
    ReachMetricConfigError,
    horizontal_reach,
    is_extended_by_reach,
    max_joint_deviation,
    validate_extended_reach,
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


class ArmSpeedLimiterNode(Node):

    def __init__(self):
        super().__init__('arm_speed_limiter_node')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        self.declare_parameter('check_period', 0.5)

        self.declare_parameter('extension_metric', METRIC_HORIZONTAL_REACH)
        self.declare_parameter('chassis_base_frame', DEFAULT_CHASSIS_BASE_FRAME)
        self.declare_parameter('reach_tf_timeout_sec', 0.5)
        timeout = float(self.get_parameter('reach_tf_timeout_sec').value)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('reach_tf_timeout_sec must be finite and positive')
        self.declare_parameter('monitored_links', DEFAULT_MONITORED_LINKS)
        self.declare_parameter('extended_reach_m', 0.64)
        self.declare_parameter('extended_reach_hysteresis_m', 0.03)

        self.declare_parameter('folded_reference_rad', [0.0] * len(ARM_JOINT_NAMES))
        self.declare_parameter('extended_threshold_rad', 0.5)

        self.declare_parameter('extended_speed_limit_pct', 50.0)

        self.speed_pub = self.create_publisher(
            SpeedLimit, self.get_parameter('speed_limit_topic').value, 10)
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self.joint_state_callback, 10)

        self.last_extended = None
        period = float(self.get_parameter('check_period').value)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError('check_period must be positive')
        self._limit_timer = self.create_timer(period, self._republish_limit)
        self._metric = self._resolve_metric()
        self._tf_buffer = None
        self._tf_listener = None
        self._tf_warned = False
        if self._metric == METRIC_HORIZONTAL_REACH:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        if self._metric == METRIC_HORIZONTAL_REACH:
            criterion = ('监控连杆相对 %s 的水平伸展 > %.3fm(迟滞 %.3fm)' % (
                self.get_parameter('chassis_base_frame').value,
                self.get_parameter('extended_reach_m').value,
                self.get_parameter('extended_reach_hysteresis_m').value))
        else:
            criterion = ('关节偏差 > %.2frad(旧判据，已证伪)'
                         % self.get_parameter('extended_threshold_rad').value)
        self.get_logger().info(
            'arm_speed_limiter_node 已启动：判据为 %s，满足则限速到 %.0f%%'
            '（这是保守的单点判据，不是精确碰撞检测，也不缩小碰撞包络，'
            '详见文件头部说明）。' % (
                criterion, self.get_parameter('extended_speed_limit_pct').value))

    def _republish_limit(self):
        if self.last_extended is None:
            return
        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.percentage = True
        limit.speed_limit = (self.get_parameter('extended_speed_limit_pct').value
                             if self.last_extended else 0.0)
        self.speed_pub.publish(limit)

    def _resolve_metric(self):
        """确定实际使用的判据。配置非法时**退回更保守的旧判据**并大声报错，而不是
        抛异常退出——限速节点退出会让 /speed_limit 永远不再更新，等于静默解除限速，
        那比继续用一个偏保守的判据危险得多。"""
        metric = self.get_parameter('extension_metric').value
        if metric not in VALID_METRICS:
            self.get_logger().error(
                'extension_metric=%r 不是合法取值%s，退回保守的 %s 判据。'
                % (metric, list(VALID_METRICS), METRIC_JOINT_DEVIATION))
            return METRIC_JOINT_DEVIATION
        if metric == METRIC_JOINT_DEVIATION:
            self.get_logger().warn(
                'extension_metric=joint_deviation：这是已被实测证伪的旧判据(与真实'
                '水平伸展反相关，详见 arm_reach_metric.py 头部)，只应用于 A/B 回归'
                '对比或临时回退，不要长期这么跑。')
            return METRIC_JOINT_DEVIATION
        try:
            validate_extended_reach(
                self.get_parameter('extended_reach_m').value,
                self.get_parameter('extended_reach_hysteresis_m').value)
        except ReachMetricConfigError as exc:
            self.get_logger().error(
                '展开判据阈值非法(%s)，退回保守的 %s 判据。请修正 extended_reach_m/'
                'extended_reach_hysteresis_m 后重启。' % (exc, METRIC_JOINT_DEVIATION))
            return METRIC_JOINT_DEVIATION
        return METRIC_HORIZONTAL_REACH

    def _max_horizontal_reach(self):
        """查 TF 求监控连杆里相对底盘回转轴的最大水平伸展(m)，返回 (伸展, 连杆名)。

        任一路查不到或陈旧返回 None，由调用方决定策略(本节点按最保守处理，判"展开")。

        !!! lookup_transform 传 Time()(=最新可用)且不带 timeout !!!
        项目笔记记过：带 timeout 的动态 TF 查询在非专用线程里必然失败，而 static TF
        却能成功，看起来像正常。这里在执行器线程里跑，带 timeout 还会阻塞回调。
        """
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
                    '监控连杆的 TF 不完整或无效(%s)，按最保守处理(判为展开、限速)；'
                    'TF 树建立后自动恢复，此告警只打一次。'
                    % ('; '.join(failures) if failures else 'monitored_links 为空'))
            return None
        self._tf_warned = False
        return best, best_link

    def joint_state_callback(self, msg: JointState):
        if self._metric == METRIC_HORIZONTAL_REACH:
            probed = self._max_horizontal_reach()
            if probed is None:
                is_extended = True
                detail = 'TF 不可用，按最保守判定'
            else:
                reach, link = probed
                is_extended = is_extended_by_reach(
                    reach,
                    self.get_parameter('extended_reach_m').value,
                    self.get_parameter('extended_reach_hysteresis_m').value,
                    was_extended=bool(self.last_extended))
                detail = '最大水平伸展 %.3fm@%s' % (reach, link)
        else:
            name_to_pos = dict(zip(msg.name, msg.position))
            max_dev = max_joint_deviation(
                name_to_pos, ARM_JOINT_NAMES,
                self.get_parameter('folded_reference_rad').value)
            is_extended = max_dev > self.get_parameter('extended_threshold_rad').value
            detail = '最大关节偏差 %.2frad' % max_dev

        if is_extended == self.last_extended:
            return  # 定时器负责重发，状态变化日志只记录一次。

        self.last_extended = is_extended
        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.percentage = True
        if is_extended:
            limit.speed_limit = self.get_parameter('extended_speed_limit_pct').value
            self.get_logger().warn(
                '检测到机械臂展开(%s)，限速到 %.0f%%' % (detail, limit.speed_limit))
        else:
            limit.speed_limit = 0.0  # nav2约定：0.0 表示解除限速，恢复正常速度上限
            self.get_logger().info('机械臂在足迹以内(%s)，解除限速' % detail)
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
