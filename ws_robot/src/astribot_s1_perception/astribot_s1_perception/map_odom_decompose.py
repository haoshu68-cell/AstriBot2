#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`map→odom` 的分解数学。**纯逻辑**，不 import rclpy、不 import 消息类型。

════════════════ 这一层解决什么问题 ════════════════
TF 要求每个子帧**恰好一个**父源。而我们有两个独立的位姿来源：

    外部 SLAM   camera_init → aft_mapped      （全局，会被 GBA 修正而跳变）
    SDK 里程计   odom → astribot_torso_base    （局部连续，允许漂但不跳）

两者说的是**同一个物理点**（底盘中心）—— 这一条是决策"以 aft_mapped 为准"
给定的前提，不是我推断的。于是标准 REP-105 分解成立：

    map → odom = (map → base)_SLAM  ∘  (odom → base)_SDK⁻¹

这样 TF 树是：
    map ──(本模块算)──▶ odom ──(chassis_odom_node)──▶ astribot_torso_base
无环、无双父源，而 `astribot_torso_base` 的最终位置恰好等于 SLAM 说的位置。

════════════════ 为什么不能反过来 ════════════════
不能直接把 SLAM 的变换当成 `map→odom` 发出去。odom 的契约是**局部连续、不跳变**，
而 Voxel-SLAM 带 Loop closure + GBA，它的位姿是 map 级、会跳。把会跳的量当 odom
会让 local_costmap（`global_frame: odom`）整体瞬移，而 nav2 不会报错，
只是局部规划突然失效 —— 症状离根因很远。

也不能把 `aft_mapped` 直接改名成 `astribot_torso_base`：下游硬依赖后者
（nav2 六处 `robot_base_frame`、探索协调器、点云自滤），而 URDF 的根 link
也是它 —— robot_state_publisher 要从它往下发各连杆，它必须只有一个父。

════════════════ 纯 2D ════════════════
底盘是平面运动，两个来源都只有 (x, y, yaw) 有意义。刻意不做 3D：
带上 pitch/roll 会让"地面不平引起的姿态噪声"漏进 map→odom，
表现成地图轻微摇摆，而那是假的。
"""

import math
from dataclasses import dataclass


class DecompositionError(ValueError):
    """输入不满足分解前提。不允许发一个"凑合能用"的变换出去。"""


def wrap_angle(theta):
    """归一化到 (-pi, pi]。不做的话角度会越界累积，日志里看着离谱。"""
    return math.atan2(math.sin(theta), math.cos(theta))


def yaw_from_quaternion(z, w):
    """只从 (z, w) 取 yaw。平面运动下 x/y 应当≈0，调用方负责校验。"""
    return wrap_angle(2.0 * math.atan2(float(z), float(w)))


def quaternion_from_yaw(theta):
    """返回 (z, w)。x/y 恒为 0 —— 平面运动没有 roll/pitch。"""
    half = 0.5 * float(theta)
    return (math.sin(half), math.cos(half))


@dataclass(frozen=True)
class Pose2D:
    """平面位姿。x/y 单位米，theta 弧度。"""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def __post_init__(self):
        for name in ('x', 'y', 'theta'):
            v = getattr(self, name)
            if not math.isfinite(v):
                raise DecompositionError(
                    f'Pose2D.{name}={v!r} 不是有限数。'
                    f'NaN/Inf 会让整条 TF 链变成 NaN，而 tf2 不会报错，'
                    f'只是所有 lookupTransform 都失败 —— 症状离根因很远。')

    def inverse(self):
        """逆变换。用于 (odom→base)⁻¹。"""
        cos_t, sin_t = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(
            x=-(cos_t * self.x + sin_t * self.y),
            y=-(-sin_t * self.x + cos_t * self.y),
            theta=wrap_angle(-self.theta))

    def compose(self, other):
        """self ∘ other：先施加 other，再施加 self（左乘）。"""
        cos_t, sin_t = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(
            x=self.x + cos_t * other.x - sin_t * other.y,
            y=self.y + sin_t * other.x + cos_t * other.y,
            theta=wrap_angle(self.theta + other.theta))

    def translation_distance_to(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)


#: `map→odom` 单次更新的位移上限（米）。超过就是 SLAM 发生了回环修正。
#: 那**不是**故障 —— 回环修正本来就该体现在 map→odom 上（这正是 REP-105
#: 分解的目的：让跳变留在 map→odom，odom→base 保持连续）。但要**上报**，
#: 因为它会让 global_costmap 整体平移，而这对上层是可见事件。
DEFAULT_JUMP_REPORT_M = 0.30


@dataclass
class DecomposeStats:
    """可观测量。故障时先看这几个数。"""

    updates: int = 0
    jumps: int = 0
    max_jump_m: float = 0.0
    rejected_tilt: int = 0


class MapOdomDecomposer:
    """(map→base)_SLAM 与 (odom→base)_SDK  →  map→odom。纯逻辑，不读时钟。

    时间一律由调用方传入。本仓库有过"用墙钟时间戳让测试结论作废"的先例，
    所以这里刻意不给它读时钟的能力。
    """

    def __init__(self, jump_report_m=DEFAULT_JUMP_REPORT_M, max_tilt_rad=0.10):
        if not jump_report_m > 0.0:
            raise DecompositionError(
                f'jump_report_m={jump_report_m} 必须为正；'
                f'置 0 会把每次更新都报成跳变，日志刷满、真跳变被埋掉')
        self.jump_report_m = float(jump_report_m)
        self.max_tilt_rad = float(max_tilt_rad)
        self.stats = DecomposeStats()
        self._last = None

    def update(self, map_to_base, odom_to_base):
        """算一次 map→odom。返回 (Pose2D, 本次是否跳变, 跳变距离)。"""
        result = map_to_base.compose(odom_to_base.inverse())

        jumped = False
        jump_m = 0.0
        if self._last is not None:
            jump_m = self._last.translation_distance_to(result)
            jumped = jump_m > self.jump_report_m
            if jumped:
                self.stats.jumps += 1
                self.stats.max_jump_m = max(self.stats.max_jump_m, jump_m)

        self._last = result
        self.stats.updates += 1
        return (result, jumped, jump_m)

    def check_planar(self, qx, qy, label):
        """校验四元数确实是平面旋转（x/y≈0）。返回违规原因或 None。

        为什么要查：本模块按 2D 处理。若上游真的有 pitch/roll（地面不平、
        标定有倾角），静默丢掉它会让 map→odom 与真实位姿差一个倾角，
        表现成地图与激光对不上一个小角度 —— 那种偏差很容易被误当成"定位精度不够"。
        """
        tilt = math.hypot(float(qx), float(qy))
        if tilt > self.max_tilt_rad:
            self.stats.rejected_tilt += 1
            return (f'{label} 的四元数 x={qx:.4f} y={qy:.4f}（合成 {tilt:.4f}）'
                    f'超过平面容差 {self.max_tilt_rad}：本模块按 2D 分解，'
                    f'静默丢掉 pitch/roll 会让 map→odom 差一个倾角，'
                    f'表现成"地图与激光差一个小角度"，容易被误判成定位精度问题。')
        return None

    @property
    def last(self):
        return self._last
