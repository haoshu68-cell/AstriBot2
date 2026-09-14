#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SDK 指令位置误差 leash 与只读位姿诊断。

Nav2 负责基于定位的轨迹反馈控制。桥接只对其积分位置指令与 SDK 实际位置
执行 leash 检查，避免驱动阻塞时位置指令持续超前；不再实现 SLAM 校正器。
"""

import math

from astribot_trajectory_bridge.chassis_integrator import (
    ChassisConfigError,
    IDX_THETA,
    IDX_X,
    IDX_Y,
    error_magnitude,
    pose_error,
    wrap_angle,
)


POSE_SOURCE_SLAM = 'slam'
POSE_SOURCE_GROUND_TRUTH = 'ground_truth'
VALID_POSE_SOURCES = (POSE_SOURCE_SLAM, POSE_SOURCE_GROUND_TRUTH)


class LeashState:
    """leash 的判定结果。tripped 为 True 时调用方必须冻结积分。"""

    def __init__(self, tripped, err_xy, err_theta, reason=''):
        self.tripped = tripped
        self.err_xy = err_xy
        self.err_theta = err_theta
        self.reason = reason


def validate_leash_config(leash_xy_m, leash_theta_rad):
    """leash 阈值校验。<=0 等于关闭保护，必须显式拒绝而不是当成"不限制"。"""
    if leash_xy_m <= 0.0:
        raise ChassisConfigError(
            'leash_xy_m=%r 必须为正。<=0 等于关闭 leash 保护，'
            '而 leash 是位置积分开环链路上唯一的硬保护，不允许关闭。' % (leash_xy_m,))
    if leash_theta_rad <= 0.0:
        raise ChassisConfigError(
            'leash_theta_rad=%r 必须为正，理由同 leash_xy_m。' % (leash_theta_rad,))


def check_leash(pos_cmd, sdk_actual, leash_xy_m, leash_theta_rad):
    """leash 判定。判据是 **SDK 实际位置**，不是 SLAM。

    用 SDK 实际而不是 SLAM 的理由：leash 要抓的是"指令跑飞"，需要快（250 Hz 可读）
    且与下发量同源（同一个 frame，不需要任何对齐）。SLAM 低频且跨 frame，
    用它做 leash 会引入 frame 失配这个与打滑无关的误差来源。
    """
    err = pose_error(pos_cmd, sdk_actual)
    err_xy, err_theta = error_magnitude(err)
    if err_xy > leash_xy_m:
        return LeashState(True, err_xy, err_theta,
                          'xy 偏差 %.4fm > 阈值 %.4fm' % (err_xy, leash_xy_m))
    if err_theta > leash_theta_rad:
        return LeashState(True, err_xy, err_theta,
                          'theta 偏差 %.4frad > 阈值 %.4frad' % (err_theta, leash_theta_rad))
    return LeashState(False, err_xy, err_theta)


def leash_recover_command(sdk_actual):
    """leash 触发后把指令拽回实际位置。

    返回 SDK 实际位置的副本作为新的 pos_cmd —— 不是置零、也不是保持原值：
    保持原值会让下次解冻时立刻又是一个大阶跃；置零则会命令底盘回原点。
    """
    return [sdk_actual[IDX_X], sdk_actual[IDX_Y], wrap_angle(sdk_actual[IDX_THETA])]


def effective_thresholds(pose_source, slam_jump_threshold_m, odom_drift_warn_m):
    """按位姿源自动失效那些没有意义的阈值。

    ``ground_truth`` 下 ``map->odom`` 是恒等静态 TF：
      * 位姿不会跳变 -> 跳变检测无意义
      * SLAM 位姿与 SDK 位姿同源 -> 漂移比对恒为 0，告警无意义

    自动失效而不是让人手工改，是因为手工改容易漏，漏了会得到一堆假告警或假静默。
    注意 ``slam_max_age_sec`` **两种源下都保留** —— 虽然 map->odom 是静态的，
    但 odom->base 始终是动态的，合成变换的时间戳由动态那一段决定，
    所以龄期检查照常能发现"里程计停了"。
    """
    if pose_source not in VALID_POSE_SOURCES:
        raise ChassisConfigError(
            'pose_source=%r 非法，必须显式声明为 %s 之一'
            % (pose_source, list(VALID_POSE_SOURCES)))
    if pose_source == POSE_SOURCE_GROUND_TRUTH:
        return (float('inf'), float('inf'))
    return (slam_jump_threshold_m, odom_drift_warn_m)


def detect_pose_jump(pose_now, pose_prev, jump_threshold_m):
    """位姿跳变检测（重定位事件）。

    返回 (是否跳变, 跳变距离)。跳变时调用方必须**重新对齐期望位姿**而不是
    施加这个巨大误差 —— 扫描匹配的一次修正可能有几十厘米，当成误差喂进去
    就是一个几十厘米的位置阶跃。
    """
    if pose_prev is None:
        return (False, 0.0)
    jump = math.hypot(pose_now[IDX_X] - pose_prev[IDX_X],
                      pose_now[IDX_Y] - pose_prev[IDX_Y])
    return (jump > jump_threshold_m, jump)


def odom_drift(disp_sdk_xy, disp_slam_xy):
    """里程计漂移：同一时间窗内两个位移源的**模长差**，frame 无关。

    为什么用模长差而不是"两个 frame 里的绝对位置相减"
    ----------------------------------------------
    SDK 的 [x, y, theta] 在驱动器自己的 frame（原点是驱动器启动位置），
    SLAM 在 map frame（原点是建图起点），两者差一个刚体变换。直接相减需要先对齐，
    而对齐关系会随时间失效（若 SDK 位姿本身是里程计积分）。

    位移的**模长是旋转不变量**，所以比较模长不需要知道 frame 之间的关系。
    物理含义也更干净：**"轮子以为走了多远"与"世界里实际走了多远"之差**，
    这正是打滑的定义，且不掺任何 frame 失配成分。
    """
    d_sdk = math.hypot(disp_sdk_xy[0], disp_sdk_xy[1])
    d_slam = math.hypot(disp_slam_xy[0], disp_slam_xy[1])
    return abs(d_sdk - d_slam)
