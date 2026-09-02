#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真值源。

**为什么要有这一层**
本方案最重要的一条规则是「不能用底盘自己的里程计验证底盘」——打滑时里程计和指令
一起错，误差为零，看着完美，而机器人其实没走到。但这条规则在两个环境里的结论**相反**：

* **仿真**：``/odom`` 来自 gz-sim 的 ``OdometryPublisher`` 插件，它**基于模型真实位姿**
  计算、与底盘运动学解耦（``astribot_s1.gazebo.xacro:62-64``，50 Hz）。
  打滑时它跟着真实位姿一起不动 —— 所以它**是**真值。
* **真机**：厂商栈不发布任何 TF；SDK 的 ``get_current_joints_position([chassis])``
  是底盘驱动自己的航迹推算 —— 它**不是**真值，只能当交叉参考。

所以这里用两个不同的类把区别写死，并且给航迹推算那个类起名叫
``DeadReckoningReference``、``is_ground_truth`` 恒为 False，让"顺手用它当真值"
在代码层面显眼到无法忽略。
"""

import math
import sys
import time

from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    """四元数 -> yaw。只取绕 Z 的分量。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_angle(a):
    """归一到 (-pi, pi]。"""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def unwrap_delta(a_to, a_from):
    """两个角度之差，取最短路径。"""
    return wrap_angle(a_to - a_from)


class Pose2D:
    __slots__ = ('x', 'y', 'yaw', 'stamp')

    def __init__(self, x, y, yaw, stamp):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.stamp = stamp     # 秒。仿真里是仿真时间，真机是墙钟

    def __repr__(self):
        return f'Pose2D(x={self.x:.4f}, y={self.y:.4f}, yaw={self.yaw:.4f}, t={self.stamp:.3f})'

    def as_dict(self):
        return {'x': self.x, 'y': self.y, 'yaw': self.yaw, 'stamp': self.stamp}


def displacement(p0, p1):
    """p0 -> p1 的位移，**表达在 p0 的车体系里**。

    返回 (前向, 侧向, 转角)。开环平移/旋转测试要的就是这个口径：
    "沿指令方向走了多少"和"垂直方向漂了多少"。
    """
    dx = p1.x - p0.x
    dy = p1.y - p0.y
    c, s = math.cos(-p0.yaw), math.sin(-p0.yaw)
    return (c * dx - s * dy, s * dx + c * dy, unwrap_delta(p1.yaw, p0.yaw))


class SimOdomTruth:
    """仿真真值：订阅 /odom（基于模型真实位姿，见模块头部说明）。"""

    is_ground_truth = True
    name = 'sim:/odom (OdometryPublisher, 基于模型真实位姿)'

    def __init__(self, node, topic='/odom', history_sec=5.0):
        self._node = node
        self._topic = topic
        self._history_sec = history_sec
        self._history = []          # [Pose2D, ...] 按时间递增
        self._msg_count = 0
        self._sub = node.create_subscription(Odometry, topic, self._cb, 50)

    def _cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = Pose2D(msg.pose.pose.position.x,
                   msg.pose.pose.position.y,
                   yaw_from_quaternion(msg.pose.pose.orientation),
                   stamp)
        self._history.append(p)
        self._msg_count += 1
        cutoff = stamp - self._history_sec
        while len(self._history) > 2 and self._history[0].stamp < cutoff:
            self._history.pop(0)

    @property
    def msg_count(self):
        return self._msg_count

    def wait_ready(self, timeout_sec=5.0):
        """等到收到第一条消息。超时抛错而不是静默返回零位姿。"""
        import rclpy
        end = time.monotonic() + timeout_sec
        while not self._history and time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.02)
        if not self._history:
            raise TimeoutError(
                f'{timeout_sec}s 内没收到 {self._topic}。仿真里它是真值源，'
                f'缺了整组测试都不能做。检查 ros_gz_bridge 与 OdometryPublisher 插件。')
        return self.pose()

    def pose(self):
        if not self._history:
            raise RuntimeError('还没有位姿数据，先调 wait_ready()')
        return self._history[-1]

    def velocity(self, window_sec=0.2):
        """由位姿在时间窗上差分求速度，返回 (vx_body, vy_body, wz)。

        **刻意不用 /odom 的 twist 字段**：那是插件自己 50 Hz 差分出来的，噪声大。
        window_sec 越大越平滑但越滞后；阶跃响应测量建议 0.1，稳态测量 0.3。
        """
        if len(self._history) < 2:
            return (0.0, 0.0, 0.0)
        latest = self._history[-1]
        target = latest.stamp - window_sec
        ref = self._history[0]
        for p in self._history:
            if p.stamp >= target:
                ref = p
                break
        dt = latest.stamp - ref.stamp
        if dt <= 1e-6:
            return (0.0, 0.0, 0.0)
        fwd, lat, dyaw = displacement(ref, latest)
        return (fwd / dt, lat / dt, dyaw / dt)


class DeadReckoningReference:
    """真机上的 SDK 航迹推算读数。**不是真值**，只用于交叉参考与 C 组接口测试。

    A 组的距离/角度判据一律不许用它——用 ManualTruth。
    """

    is_ground_truth = False
    name = 'real:SDK get_current_joints_position (航迹推算，非真值)'

    def __init__(self, astribot):
        self._astribot = astribot
        self._chassis = astribot.chassis_name

    def pose(self):
        q = self._astribot.get_current_joints_position([self._chassis])[0]
        return Pose2D(q[0], q[1], q[2], time.time())

    def desired_pose(self):
        q = self._astribot.get_desired_joints_position([self._chassis])[0]
        return Pose2D(q[0], q[1], q[2], time.time())


class ManualTruth:
    """真机真值：人工量测录入（地面贴标 + 卷尺 / 激光笔投墙）。

    交互式：每一次试验结束后提示操作者输入实测值。刻意做成**必须逐次录入**，
    不提供"批量补录"——事后凭记忆补的数不是数据。
    """

    is_ground_truth = True
    name = 'real:人工量测（地面贴标 + 卷尺 / 激光笔）'

    def __init__(self, stream=None):
        self._in = stream or sys.stdin

    def _ask_float(self, prompt, allow_blank=False):
        while True:
            print(prompt, end='', flush=True)
            line = self._in.readline()
            if line == '':
                raise EOFError('输入结束，测试中止（已录入的样本仍会写盘）')
            line = line.strip()
            if not line and allow_blank:
                return None
            try:
                return float(line)
            except ValueError:
                print('  请输入一个数字（单位见提示），或 Ctrl-D 中止。')

    def ask_translation(self, trial, heading_deg, commanded_m):
        """返回 (沿指令方向实测位移 m, 垂直方向偏移 m, 终态偏航变化 rad)。"""
        print(f'\n--- 试验 {trial}：指令沿 {heading_deg:g}° 走 {commanded_m:g} m ---')
        along = self._ask_float('  沿指令方向实测位移 (m)：')
        lateral = self._ask_float('  垂直于指令方向的偏移 (m，左正右负)：')
        dyaw = self._ask_float('  终态偏航变化 (rad，激光笔投墙换算；无则回车) ：',
                               allow_blank=True)
        return along, lateral, (0.0 if dyaw is None else dyaw)

    def ask_rotation(self, trial, commanded_rad):
        """返回 (实测转角 rad, xy 漂移 m)。"""
        print(f'\n--- 试验 {trial}：指令转 {math.degrees(commanded_rad):.1f}° ---')
        dyaw = self._ask_float('  实测转角 (rad，激光笔投墙换算)：')
        drift = self._ask_float('  xy 漂移距离 (m)：')
        return dyaw, drift

    def ask_distance(self, prompt):
        return self._ask_float(f'  {prompt} (m)：')


def make_truth(env, node=None, astribot=None):
    """按环境选真值源。真机必须给 ManualTruth，不给就报错而不是退化成 odom。"""
    if env == 'sim':
        if node is None:
            raise ValueError('sim 真值源需要一个 rclpy Node')
        return SimOdomTruth(node)
    if env == 'real':
        return ManualTruth()
    raise ValueError(f'未知环境 {env}')
