#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""发布 `odom` 话题与 `odom → astribot_torso_base` TF。**纯只读，不含任何运动指令。**

════════════════ 它填的是一个真实缺口 ════════════════
实机实测：`/tf` 与 `/tf_static` 的**发布者数均为 0** —— 厂商栈完全不发 TF。
`odom → astribot_torso_base` 这条边因此没有任何来源，而五个下游同时依赖它：
nav2 的 costmap、bt_navigator、控制器、点云自滤、探索协调器。
缺这一条边的症状分散在各处（"Could not transform"、costmap 空、导航起不来），
离根因都很远。

════════════════ 只读边界 ════════════════
本节点只调用两个 SDK **读**接口：
    get_current_joints_position(['astribot_chassis'])
    get_current_joints_velocity(['astribot_chassis'])
会话经 `sdk_session.open_session()` 建立，其中 `high_control_rights` 硬编码 False
（那是只读方向的物理边界，不是可配置项）。**本文件不 import、不调用任何
set_* / move_* / open_effector 等会让机器人运动的接口。**

════════════════ 为什么不复用 chassis_cmd_bridge 的 _publish_odom ════════════════
那份实现 `frame_id='sdk_chassis'`、不发 TF、且只在成功 enable 之后才发
（写入闸门拒绝时 `_outer_tick` 直接提前返回）。也就是说"只读、手推机器人看
odom 变化"这件事用那份代码做不到。本节点必须独立于写通路。

⚠️ **不要和 Gazebo 的 OdometryPublisher 同时跑**：同一子帧两个父源不会报错，
只让位姿反复跳，症状看起来像"定位漂移"。仿真里请让本节点保持关闭。
"""

from astribot_logging import get_logger

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from astribot_trajectory_bridge.chassis_odom_source import (
    ChassisOdomSource,
    OdomSourceError,
)
from astribot_trajectory_bridge.sdk_session import SdkSessionError, open_session


def sensor_qos(depth=10):
    """里程计用 BEST_EFFORT。

    与 SDK 侧一致（实测厂商自己的 joint_space_command 就是 BEST_EFFORT，
    那条 incompatible QoS 告警两端都在 SDK 内部），而且 odom 丢一帧无所谓 ——
    下一帧 20ms 后就来。用 RELIABLE 反而会在网络抖动时堆积重传。
    """
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)


class ChassisOdomNode(Node):
    """ROS 粘合层。所有数学都在 `chassis_odom_source`（纯逻辑、离线可测）。"""

    def __init__(self):
        super().__init__('chassis_odom')

        self.declare_parameter('part_name', 'astribot_chassis')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'astribot_torso_base')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('jump_threshold_m', 0.30)
        self.declare_parameter('velocity_frame', 'body')
        self.declare_parameter('sdk_freq', 250.0)
        self.declare_parameter('report_period_sec', 10.0)
        self.declare_parameter('max_consecutive_read_errors', 25)

        self.part_name = str(self.get_parameter('part_name').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        rate = float(self.get_parameter('publish_rate').value)
        if not rate > 0.0:
            raise OdomSourceError(f'publish_rate={rate} 必须为正')

        if self.base_frame == 'base_link':
            self.get_logger().warning(
                'base_frame=base_link：本机器人**没有** base_link，'
                '根 frame 是 astribot_torso_base。'
                '这样发出去的 TF 会挂在一个没人认的 frame 上。')

        self.source = ChassisOdomSource(
            jump_threshold_m=float(self.get_parameter('jump_threshold_m').value),
            velocity_frame=str(self.get_parameter('velocity_frame').value))

        self.exit_code = 0
        self._read_errors = 0
        self._max_read_errors = int(
            self.get_parameter('max_consecutive_read_errors').value)

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), sensor_qos())
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if bool(self.get_parameter('publish_tf').value) else None)

        self.session = open_session(
            freq=float(self.get_parameter('sdk_freq').value),
            node_name='chassis_odom_reader',
            logger=self.get_logger())

        self.create_timer(1.0 / rate, self._tick)
        report_period = float(self.get_parameter('report_period_sec').value)
        if report_period > 0.0:
            self.create_timer(report_period, self._tick_report)

        self.get_logger().info(
            f'chassis_odom 启动（**只读**）：\n'
            f'  part={self.part_name}  {rate:.1f}Hz\n'
            f'  出 {self.get_parameter("odom_topic").value}  '
            f'TF {self.odom_frame}→{self.base_frame} '
            f'{"发" if self.tf_broadcaster else "不发"}\n'
            f'  跳变阈值 {self.source.jump_threshold_m}m  '
            f'速度系={self.source.velocity_frame}')

    def _tick(self):
        try:
            pos = self.session.get_current_joints_position([self.part_name])[0]
            vel = self.session.get_current_joints_velocity([self.part_name])[0]
        except Exception as exc:      # noqa: BLE001
            self._on_read_error(exc)
            return

        self._read_errors = 0
        try:
            sample = self.source.sample(pos, vel)
        except OdomSourceError as exc:
            self.get_logger().error(f'SDK 读数不符合约定，退出：{exc}')
            self.exit_code = 1
            raise SystemExit(1)

        stamp = self.get_clock().now().to_msg()
        self._publish_odom(sample, stamp)
        if self.tf_broadcaster is not None:
            self._publish_tf(sample, stamp)

        if sample.jumped:
            self.get_logger().warning(
                f'位姿跳变 {sample.jump_m:.3f}m（阈值 '
                f'{self.source.jump_threshold_m}m）：'
                f'odom 的契约是允许漂、**不允许跳**。'
                f'累计 {self.source.stats.jumps} 次 / '
                f'{self.source.stats.samples} 帧。'
                f'若持续出现，说明厂商内部在做重定位，'
                f'"把 SDK 位姿当 odom"这个方案不成立，'
                f'要改用外部 SLAM 的里程计或自己积分轮速。')

    def _on_read_error(self, exc):
        self._read_errors += 1
        if self._read_errors >= self._max_read_errors:
            self.get_logger().error(
                f'连续 {self._read_errors} 次读 SDK 失败，退出：{exc}\n'
                f'  不静默重试是刻意的：下游会一直等 odom，'
                f'症状变成"导航卡住"，而原因在这一层。\n'
                f'  依次查：① part 名字是否为 {self.part_name}'
                f'（与 chassis_bridge.yaml 一致）；'
                f'② SDK 后端是否还活着；'
                f'③ ROS_DOMAIN_ID 是否与 SDK 一致（实机是 25，不是 42）。')
            self.exit_code = 1
            raise SystemExit(1)
        self.get_logger().debug(f'读 SDK 失败（{self._read_errors}）：{exc}')

    def _publish_odom(self, sample, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = sample.x
        msg.pose.pose.position.y = sample.y
        z, w = sample.quaternion_zw
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        msg.twist.twist.linear.x = sample.vx_body
        msg.twist.twist.linear.y = sample.vy_body
        msg.twist.twist.angular.z = sample.wz
        self.odom_pub.publish(msg)

    def _publish_tf(self, sample, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = sample.x
        transform.transform.translation.y = sample.y
        z, w = sample.quaternion_zw
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(transform)

    def _tick_report(self):
        stats = self.source.stats
        self.get_logger().info(
            f'帧={stats.samples} 行程={stats.travelled_m:.3f}m '
            f'跳变={stats.jumps}（占比 {self.source.jump_ratio * 100:.2f}%，'
            f'最大 {stats.max_jump_m:.3f}m）')


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = ChassisOdomNode()
        rclpy.spin(node)
    except SdkSessionError as exc:
        get_logger('astribot.chassis_odom_node').error(f'[chassis_odom] SDK 会话建立失败：{exc}')
        code = 3
    except OdomSourceError as exc:
        get_logger('astribot.chassis_odom_node').error(f'[chassis_odom] 参数被拒绝：{exc}')
        code = 2
    except SystemExit as exc:
        code = int(exc.code or 0)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            code = code or node.exit_code
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
