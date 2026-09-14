#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘 leash（位置牵引绳）与 SLAM 位姿反馈外环的纯函数核心。

为什么必须有 leash
================
``set_joints_position`` 收的是**位置**指令，而积分是开环的：轮子打滑时，指令
位置持续超前实际位置，**没有任何反馈会阻止它**。误差单调累积，一旦恢复附着力，
底盘会以最大能力冲向那个跑飞的位置。leash 是这条开环链路上唯一的硬保护。

leash 与 SLAM 外环的分工（不能混）
================================
底盘有三个位置来源，角色必须钉死，否则会互相打架：

===============  ==========================================  ==================
来源              特性                                        角色
===============  ==========================================  ==================
pos_cmd          桥接自己积分，无噪声但累积漂移                 下发量
SDK 实际          get_current_joints_position，快但含打滑误差    **leash 判据**（快环）
SLAM 位姿         TF map->base，绝对但低频且会跳变              **漂移校正**（慢环）
===============  ==========================================  ==================

为什么外环不能直接塞进 250 Hz 内环
--------------------------------
slam_toolbox 是异步的，位姿更新只有 5~20 Hz，且扫描匹配会产生**离散跳变**。
把跳变直接当误差喂进 250 Hz 位置指令 = 下发阶跃 = 底盘猛冲。
所以外环低速率 + 限幅 + 切片下发是硬要求，三者缺一不可。

稳定性：这是位置伺服套在位置伺服外面
--------------------------------
SDK 内部本身是位置控制，外环再给位置校正，很容易振荡。设计上用三条约束换稳定，
宁可校正慢：``kp <= KP_HARD_MAX``、校正**速度**限幅、校正量切片到内环周期。
其中"校正速度限幅"是最重要的一条 —— 它保证即使位姿源完全发疯，校正也只能让
底盘以 max_corr_vel 蠕动，不会猛冲。
"""

import math

from astribot_trajectory_bridge.chassis_integrator import (
    ChassisConfigError,
    IDX_THETA,
    IDX_X,
    IDX_Y,
    error_magnitude,
    pose_error,
    rot_z,
    rotate_vec2_transposed,
    wrap_angle,
)

KP_HARD_MAX = 0.5

POSE_SOURCE_SLAM = 'slam'
POSE_SOURCE_GROUND_TRUTH = 'ground_truth'
VALID_POSE_SOURCES = (POSE_SOURCE_SLAM, POSE_SOURCE_GROUND_TRUTH)


class LeashState:
    """leash 的判定结果。tripped 为 True 时调用方必须同时冻结积分与校正。"""

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


def validate_correction_config(kp_xy, kp_theta, max_corr_vel_xy,
                               max_corr_vel_theta, outer_rate, inner_freq):
    """外环参数校验。

    kp 超上限时抛错由调用方决定回退，因为"回退到保守值继续跑"比"拒绝启动"更合适
    —— 增益偏大只是可能振荡，不是立即危险；但必须大声报出来。
    """
    if outer_rate <= 0.0:
        raise ChassisConfigError('outer_rate=%r 必须为正' % (outer_rate,))
    if inner_freq <= 0.0:
        raise ChassisConfigError('inner_freq=%r 必须为正' % (inner_freq,))
    if inner_freq < outer_rate:
        raise ChassisConfigError(
            'inner_freq=%r 小于 outer_rate=%r：外环比内环还快，校正无法切片下发'
            % (inner_freq, outer_rate))
    if max_corr_vel_xy <= 0.0 or max_corr_vel_theta <= 0.0:
        raise ChassisConfigError(
            'max_corr_vel_xy=%r / max_corr_vel_theta=%r 必须为正。'
            '这是位姿源发疯时的最后防线，不允许关闭。'
            % (max_corr_vel_xy, max_corr_vel_theta))
    if kp_xy <= 0.0 or kp_theta <= 0.0:
        raise ChassisConfigError(
            'kp_xy=%r / kp_theta=%r 必须为正' % (kp_xy, kp_theta))
    if kp_xy > KP_HARD_MAX or kp_theta > KP_HARD_MAX:
        raise ChassisConfigError(
            'kp_xy=%r / kp_theta=%r 超过硬上限 %.2f。外环 %.1fHz 下增益过大会过冲：'
            '这是位置伺服套位置伺服的结构，宁可校正慢。'
            % (kp_xy, kp_theta, KP_HARD_MAX, outer_rate))


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


def is_correction_degenerate(pose_source, enable_correction):
    """闭环是否处于退化模式（跑得动但不产生实际校正）。

    ``ground_truth`` 下 map->odom 恒等、odom->base 又来自同一套里程计，
    所以外环误差恒≈0、校正量恒≈0。代码路径在跑，但闭环没有作用。

    这个函数存在的唯一目的：**防止把 ground_truth 下的漂亮数字误当成
    "闭环有效性已验收"**。调用方必须据此上报 CORRECTION_DEGENERATE 状态位，
    而不是 OK。
    """
    return bool(enable_correction) and pose_source == POSE_SOURCE_GROUND_TRUTH


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


def advance_desired_pose(p_des_map, local_disp_xy, dtheta, theta_slam):
    """在 map 系推进"应该在哪"。

    用 SLAM 的 theta 而不是 p_des_map 自己的 theta 做旋转：SLAM 的朝向是绝对的，
    而 p_des_map 的朝向本身也可能已经漂了，用漂了的朝向去旋转会让 xy 误差
    与 theta 误差互相污染。
    """
    world_xy = (rot_z(theta_slam)[0][0] * local_disp_xy[0]
                + rot_z(theta_slam)[0][1] * local_disp_xy[1],
                rot_z(theta_slam)[1][0] * local_disp_xy[0]
                + rot_z(theta_slam)[1][1] * local_disp_xy[1])
    return [p_des_map[IDX_X] + world_xy[0],
            p_des_map[IDX_Y] + world_xy[1],
            wrap_angle(p_des_map[IDX_THETA] + dtheta)]


def compute_correction(p_des_map, p_slam, kp_xy, kp_theta,
                       max_corr_vel_xy, max_corr_vel_theta, outer_rate):
    """外环一次校正量，返回**本体系**增量 (dx, dy, dtheta)。

    为什么结果是本体系：``pos_cmd`` 只吃本体系增量（见 chassis_integrator 里
    202 模式的说明），所以 map 系误差必须先转回本体系。

    !!! 这里不需要任何 map<->SDK 的 frame 对齐 !!!
    校正链路全程只产生本体系增量，``pos_cmd`` 从不需要知道自己在 map 里的绝对
    位置。所以两个 frame 之间那个刚体变换在控制上是不需要的 —— 这也是为什么
    本模块没有 T_map_sdk：留着一个会随时间失效、又只服务于告警的变换是净负担。

    双重限幅：
      1. kp 比例 -> 误差越大校正越强，但增益有硬上限
      2. **校正速度**限幅 -> 校正引起的运动速度不得超过 max_corr_vel，
         这保证位姿源发疯时底盘最多蠕动
    """
    err_map = pose_error(p_des_map, p_slam)
    err_body_xy = rotate_vec2_transposed(rot_z(p_slam[IDX_THETA]),
                                         (err_map[0], err_map[1]))
    corr_xy = (kp_xy * err_body_xy[0], kp_xy * err_body_xy[1])
    corr_theta = kp_theta * err_map[2]

    max_step_xy = max_corr_vel_xy / outer_rate
    norm_xy = math.hypot(corr_xy[0], corr_xy[1])
    if norm_xy > max_step_xy:
        scale = max_step_xy / norm_xy
        corr_xy = (corr_xy[0] * scale, corr_xy[1] * scale)
    max_step_theta = max_corr_vel_theta / outer_rate
    corr_theta = max(-max_step_theta, min(max_step_theta, corr_theta))
    return (corr_xy[0], corr_xy[1], corr_theta)


def slice_correction(correction, inner_freq, outer_rate):
    """把外环校正量切成内环每周期的小量，避免阶跃。

    外环 10 Hz 算出的校正量若在一个内环周期（4 ms）内一次性加上，就是一个位置
    阶跃；摊到 25 个内环周期上则是平滑的。

    ════════════════ 已知遗留：这里仍用**标称**频率 ════════════════
    切片份数按 ``inner_freq / outer_rate`` = 25 算，但内环实测拍率只有下界
    ≥157Hz —— 两次外环之间真实只跑了 15.7~25 拍，所以校正量最多只施加了 63%。
    内环积分已于本次改用实测 dt（见 chassis_integrator.measure_tick_dt），
    **这里刻意没有一起改**，理由：

    · 少施加校正是**保守**方向（收敛慢），多施加才会过冲；
    · 一次改两条控制通路会让上机验证分不清是哪条的效果；
    · 正确的修法是把校正表达成**速度**、和位移用同一个 dt 积分，
      那是更大的重构，不该混在这次里。

    改它之前先想清楚：拍率估计偏高会让校正过冲，而这条链路上
    ``max_corr_vel_xy`` 是唯一的兜底。
    """
    ticks = max(1, int(round(inner_freq / outer_rate)))
    return (correction[0] / ticks, correction[1] / ticks, correction[2] / ticks)


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
