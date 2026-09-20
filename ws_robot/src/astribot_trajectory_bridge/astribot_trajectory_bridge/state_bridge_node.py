#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把部件关节反馈映射为 /joint_states，沿用 bridge.yaml 的顺序和单位换算。

manufacturer 模式直接订阅厂家反馈，保留每个部件的源时间戳，不创建 SDK
会话。只发布主动关节，从动关节由 robot_state_publisher 的 mimic 关系生成。
sdk 模式保留原有读取路径；SDK 自身可能进行控制权交互。
"""

from astribot_logging import get_logger

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _ensure_sdk_does_not_mute_us():
    """必须在 import 厂商 SDK **之前**调用。

    !!! 实测：SDK 一被 import 就把整个进程的 fd 1/2 重定向到 /dev/null !!!
    `astribot_sdk/core/astribot_api/astribot_interface.py` 顶部有一段：

        quiet = os.getenv("ASTRIBOT_LOG", "").lower() not in ("1", "true", "on")
        if quiet:
            _fd1, _fd2 = os.dup(1), os.dup(2)
            null_fd = os.open(os.devnull, os.O_WRONLY)
            os.dup2(null_fd, 1); os.dup2(null_fd, 2)

    它是为了掩掉底层 C 库的刷屏，但 `os.dup2` 作用于**进程级文件描述符**，
    所以连带把本节点自己的日志一起吞了 —— 包括本该"响亮失败"的那些 ERROR。

    实测现象：不设 ASTRIBOT_LOG 时，节点连不上后端就静默退出（`Exited with
    failure 1` 之外一个字都没有）；设了 ASTRIBOT_LOG=1 才看到真正的错误原因。
    这会让"读不到状态就响亮失败"这条设计彻底失效 —— 失败了，但没人看得见。

    所以这里在 import 前把环境变量置上，让那段 quiet 分支根本不进，
    而不是依赖运维记得 export。依赖人记得设环境变量等于把坑留着。
    """
    import os
    if os.environ.get('ASTRIBOT_LOG', '').lower() not in ('1', 'true', 'on'):
        os.environ['ASTRIBOT_LOG'] = '1'


class StateBridge(Node):
    """把 SDK 的部件级状态展开成逐关节 /joint_states。"""

    def __init__(self):
        super().__init__('astribot_state_bridge')
        self._load_params()
        self._robot = None
        self._read_failures = 0
        self._flat_names = []
        for part in self._parts:
            self._flat_names.extend(self._joint_names[part])

        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info(
            '状态桥接：%d 个部件 / %d 个主动关节，发布频率 %.1f Hz'
            % (len(self._parts), len(self._flat_names), self._publish_rate))

    def _load_params(self):
        self.declare_parameter('bridge.publish_rate', 50.0)
        self.declare_parameter('bridge.frame_id', '')
        self.declare_parameter('bridge.sdk_freq', 250.0)
        self.declare_parameter('bridge.sdk_high_control_rights', False)
        self.declare_parameter('bridge.sdk_node_name', 'astribot_state_bridge')
        self.declare_parameter('bridge.feedback_source', 'sdk')
        self.declare_parameter('bridge.max_consecutive_read_failures', 25)
        self.declare_parameter('joint_map.parts', [''])

        self._publish_rate = float(self.get_parameter('bridge.publish_rate').value)
        self._frame_id = self.get_parameter('bridge.frame_id').value
        self._sdk_freq = float(self.get_parameter('bridge.sdk_freq').value)
        self._high_rights = bool(
            self.get_parameter('bridge.sdk_high_control_rights').value)
        self._sdk_node_name = self.get_parameter('bridge.sdk_node_name').value
        self._feedback_source = self.get_parameter('bridge.feedback_source').value
        if self._feedback_source not in ('sdk', 'manufacturer'):
            raise ValueError('bridge.feedback_source must be sdk or manufacturer')
        self._max_failures = int(
            self.get_parameter('bridge.max_consecutive_read_failures').value)

        if self._publish_rate <= 0.0:
            raise ValueError('bridge.publish_rate 必须为正，实际 %r' % self._publish_rate)
        if self._high_rights:
            raise ValueError(
                'bridge.sdk_high_control_rights=true 与只读状态桥接的语义冲突：'
                '读状态不需要控制权，拿了控制权只会让"绝不会动到机器人"这条边界失效。'
                'Gate 3 开写通路时再改。')

        self._parts = [p for p in self.get_parameter('joint_map.parts').value if p]
        if not self._parts:
            raise ValueError(
                'joint_map.parts 为空。必须用 --params-file 传入 bridge.yaml —— '
                '本节点不内置默认映射，内置默认值等于把需要人来断言的东西藏起来。')

        self._joint_names = {}
        self._scale = {}
        self._offset = {}
        for part in self._parts:
            key_j = 'joint_map.%s.joints' % part
            key_s = 'joint_map.%s.scale' % part
            key_o = 'joint_map.%s.offset' % part
            self.declare_parameter(key_j, [''])
            self.declare_parameter(key_s, 1.0)
            self.declare_parameter(key_o, 0.0)
            names = [n for n in self.get_parameter(key_j).value if n]
            if not names:
                raise ValueError('%s 没有配置关节名列表' % key_j)
            self._joint_names[part] = names
            self._scale[part] = float(self.get_parameter(key_s).value)
            self._offset[part] = float(self.get_parameter(key_o).value)

    def connect(self):
        """连 SDK。失败直接抛，由 main 变成"启动期响亮失败"。"""
        if self._feedback_source == 'manufacturer':
            from astribot_msgs.msg import RobotJointState
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            self._last_published = {}
            self._manufacturer_subscriptions = [self.create_subscription(
                RobotJointState, '/'+part+'/joint_space_states',
                lambda msg, part=part: self._on_manufacturer_state(part, msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                for part in self._parts]
            self.get_logger().info('直接订阅厂家关节反馈，保留源时间戳；不创建 SDK 会话。')
            return
        _ensure_sdk_does_not_mute_us()
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
        self.get_logger().info(
            '连接厂商 SDK（freq=%.1f, high_control_rights=%s）...'
            % (self._sdk_freq, self._high_rights))
        self._robot = Astribot(
            freq=self._sdk_freq, high_control_rights=self._high_rights,
            node_name=self._sdk_node_name)

        sdk_dofs = dict(zip(self._robot.whole_body_names, self._robot.whole_body_dofs))
        problems = []
        for part in self._parts:
            if part not in sdk_dofs:
                problems.append('%s 不在 SDK whole_body_names %s 里'
                                % (part, self._robot.whole_body_names))
                continue
            want = len(self._joint_names[part])
            if sdk_dofs[part] != want:
                problems.append('%s: SDK DOF=%d 而 bridge.yaml 给了 %d 个关节名'
                                % (part, sdk_dofs[part], want))
        if problems:
            raise ValueError(
                'bridge.yaml 的部件映射与 SDK 不一致，拒绝启动：\n  ' +
                '\n  '.join(problems))

        self.get_logger().info(
            'SDK 已连接。whole_body_names=%s dofs=%s'
            % (self._robot.whole_body_names, self._robot.whole_body_dofs))
        self.get_logger().warning(
            '本节点是**只读**桥接：未申请控制权，不接受轨迹、不下发任何指令。')

        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)

    def _on_manufacturer_state(self, part, source):
        stamp = source.header.stamp.sec * 10**9 + source.header.stamp.nanosec
        previous = self._last_published.get(part)
        if stamp <= 0 or (previous is not None and stamp-previous < 10**9/self._publish_rate):
            return
        values = source.position
        names = self._joint_names[part]
        if len(values) != len(names) or not all(math.isfinite(v) for v in values):
            raise ValueError(part+' 厂家关节反馈长度或数值无效')
        msg = JointState()
        msg.header.stamp = source.header.stamp
        msg.header.frame_id = self._frame_id
        msg.name = names
        msg.position = [self._scale[part]*v+self._offset[part] for v in values]
        # Publish only the updated part: a missing part must not acquire a new TF stamp.
        self._pub.publish(msg)
        self._last_published[part] = stamp

    def _read_positions(self):
        """按部件读位置，返回展平后的 URDF 量纲值；读不到返回 None。"""
        grouped = self._robot.get_current_joints_position(self._parts)
        if grouped is None or len(grouped) != len(self._parts):
            return None
        flat = []
        for part, values in zip(self._parts, grouped):
            names = self._joint_names[part]
            if values is None or len(values) != len(names):
                return None
            scale = self._scale[part]
            offset = self._offset[part]
            flat.extend(scale * float(v) + offset for v in values)
        return flat

    def _on_timer(self):
        try:
            positions = self._read_positions()
        except Exception as exc:  # noqa: BLE001 —— SDK 侧异常不能让定时器线程死掉
            self.get_logger().error('读状态抛异常: %s: %s' % (type(exc).__name__, exc))
            positions = None

        if positions is None:
            self._read_failures += 1
            if self._read_failures >= self._max_failures:
                self.get_logger().error(
                    '连续 %d 个周期读不到有效状态，停止发布并退出。'
                    '（宁可让上层看到"没有状态"，也不能给它一份过期的关节角当规划起点）'
                    % self._read_failures)
                raise SystemExit(1)
            self.get_logger().warning(
                '第 %d/%d 次读状态失败' % (self._read_failures, self._max_failures))
            return

        self._read_failures = 0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.name = self._flat_names
        msg.position = positions
        self._pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = StateBridge()
        node.connect()
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code) if exc.code is not None else 0
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        msg = '状态桥接启动失败: %s: %s' % (type(exc).__name__, exc)
        if node is not None:
            node.get_logger().error(msg)
        else:
            get_logger('astribot.state_bridge_node').error(msg)
        code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
