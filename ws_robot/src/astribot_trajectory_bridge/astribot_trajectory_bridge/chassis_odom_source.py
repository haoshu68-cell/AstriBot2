#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SDK 底盘位姿/速度 → odom 的**纯逻辑**层（不 import rclpy / 不 import 消息类型）。

填的是一个真实缺口：`odom → astribot_torso_base` 这条边在真机上**没有任何节点发布**
（实机实测 `/tf` 与 `/tf_static` 的发布者数均为 0，厂商栈完全不发 TF）。
缺这一条边会让五个下游同时失效，而症状分散在各处：nav2 报
"Could not transform"、costmap 空、bt_navigator 起不来 —— 都离根因很远。

════════════════ 为什么不复用 chassis_cmd_bridge 里的 _publish_odom ════════════════
那份实现 `frame_id='sdk_chassis'`、不发 TF、且**只在成功 enable 之后才发**
（`pos_cmd is None` 直接提前返回，写入闸门拒绝时 `_outer_tick` 也提前返回）。
也就是说"只读、手推机器人看 odom 变化"这件事用那份代码**做不到**。
本节点必须独立于写通路 —— 这正是它单独成节点的原因。

════════════════ 前提：odom 允许漂，不允许跳 ════════════════
odom 的契约只是**局部连续**：允许缓慢漂移，但**不允许跳变**。
若 SDK 的位姿会跳（厂商内部重定位），把它当 odom 就不成立，
必须退回"用外部 SLAM 的里程计"或"轮式里程计自己积分"。
所以本模块检测到跳变时 **WARN + 计数上报，绝不静默平滑掉** ——
平滑掉就把"这个方案不成立"这个事实藏了起来。
"""

import math
from dataclasses import dataclass, field

from astribot_trajectory_bridge.chassis_feedback import detect_pose_jump
from astribot_trajectory_bridge.chassis_integrator import wrap_angle


IDX_X, IDX_Y, IDX_THETA = 0, 1, 2
IDX_VX, IDX_VY, IDX_WZ = 0, 1, 2


class OdomSourceError(ValueError):
    """输入不符合约定。构造期/取样期抛出，不允许发出一个"凑合能用"的 odom。"""


@dataclass
class OdomSample:
    """一个 odom 采样点。纯数据，节点层负责搬进 Odometry 消息与 TF。

    twist 是**机体系**（child_frame_id 系）的速度 —— 这是 nav_msgs/Odometry 的
    规定：`twist` 在 child_frame 里表达，`pose` 在 header.frame_id 里表达。
    搞反了不会报错，只会让 nav2 的速度前瞻在机器人转向时系统性偏一个旋转。
    """

    x: float
    y: float
    theta: float
    vx_body: float
    vy_body: float
    wz: float
    jump_m: float = 0.0
    jumped: bool = False

    @property
    def quaternion_zw(self):
        """绕 z 的四元数 (z, w)。底盘只有 yaw，x/y 恒为 0。"""
        return (math.sin(self.theta * 0.5), math.cos(self.theta * 0.5))


@dataclass
class OdomStats:
    """可观测量。跳变次数是判断"这个方案成不成立"的唯一依据，必须能被读到。"""

    samples: int = 0
    jumps: int = 0
    max_jump_m: float = 0.0
    travelled_m: float = 0.0
    jump_history: list = field(default_factory=list)


MAX_JUMP_HISTORY = 32


class ChassisOdomSource:
    """SDK 底盘位姿/速度 → OdomSample。不含 rclpy，不读时钟。

    输入（每周期一次，两个 SDK **只读**调用）：
        pos = get_current_joints_position(['astribot_chassis'])[0]   # [x, y, theta]
        vel = get_current_joints_velocity(['astribot_chassis'])[0]   # [vx, vy, w]

    ⚠️ 本类不写任何 SDK 接口，也不含任何会让机器人运动的调用。
    """

    def __init__(self, jump_threshold_m=0.30, velocity_frame='body'):
        if not jump_threshold_m > 0.0:
            raise OdomSourceError(
                f'jump_threshold_m={jump_threshold_m} 必须为正。'
                f'置 0 等于"每一帧都算跳变"，日志会被刷满而真正的跳变被埋掉。')
        if velocity_frame not in ('body', 'world'):
            raise OdomSourceError(
                f'velocity_frame={velocity_frame!r} 非法，只能是 body|world。'
                f'这决定 SDK 给的 [vx, vy] 是机体系还是世界系 —— '
                f'搞反了不报错，只让 nav2 的速度前瞻在转向时偏一个旋转。')
        self.jump_threshold_m = float(jump_threshold_m)
        self.velocity_frame = velocity_frame
        self.stats = OdomStats()
        self._prev_pose = None

    def sample(self, pos, vel):
        """把一对 SDK 读数变成 OdomSample。

        pos / vel 必须各有 3 个数。长度不对就抛 —— 静默补零会让 odom
        看起来"能动但方向不对"，那比直接失败难查得多。
        """
        pose = self._as_triple(pos, 'pos')
        twist = self._as_triple(vel, 'vel')

        jumped, jump_m = detect_pose_jump(pose, self._prev_pose, self.jump_threshold_m)

        self.stats.samples += 1
        if jumped:
            self.stats.jumps += 1
            self.stats.max_jump_m = max(self.stats.max_jump_m, jump_m)
            if len(self.stats.jump_history) < MAX_JUMP_HISTORY:
                self.stats.jump_history.append(round(jump_m, 4))
        elif self._prev_pose is not None:
            self.stats.travelled_m += jump_m

        self._prev_pose = tuple(pose)

        vx_body, vy_body = self._to_body_velocity(twist, pose[IDX_THETA])
        return OdomSample(
            x=pose[IDX_X], y=pose[IDX_Y], theta=wrap_angle(pose[IDX_THETA]),
            vx_body=vx_body, vy_body=vy_body, wz=twist[IDX_WZ],
            jump_m=jump_m, jumped=jumped)

    def _to_body_velocity(self, twist, theta):
        """统一到机体系。Odometry.twist 按规定就在 child_frame 里表达。"""
        if self.velocity_frame == 'body':
            return (twist[IDX_VX], twist[IDX_VY])
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        return (cos_t * twist[IDX_VX] + sin_t * twist[IDX_VY],
                -sin_t * twist[IDX_VX] + cos_t * twist[IDX_VY])

    @staticmethod
    def _as_triple(values, what):
        if values is None:
            raise OdomSourceError(
                f'{what} 为 None：SDK 只读调用没拿到数据。'
                f'先确认 get_current_joints_{"position" if what == "pos" else "velocity"} '
                f'的 part 名字是 astribot_chassis（与 chassis_bridge.yaml 一致）。')
        triple = [float(v) for v in values]
        if len(triple) != 3:
            raise OdomSourceError(
                f'{what} 需要 3 个数 [x, y, theta] / [vx, vy, w]，'
                f'收到 {len(triple)} 个：{triple}。'
                f'静默补零会让 odom 看起来"能动但方向不对"，所以这里直接失败。')
        return triple

    @property
    def jump_ratio(self):
        """跳变帧占比。这个数不为 0 就意味着"SDK 位姿能当 odom"这个前提不成立。"""
        if self.stats.samples == 0:
            return 0.0
        return self.stats.jumps / self.stats.samples
