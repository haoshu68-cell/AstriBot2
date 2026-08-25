#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文件用途：机械臂"展开程度"活跃度度量的纯函数实现(不依赖 rclpy/TF,便于单测)。

!!! 为什么要换度量(C1 修正,基于实测证据,不是拍脑袋) !!!
原实现用"双臂 14 个关节相对一个参考姿态(默认全 0)的最大角度偏差"当展开程度,
实测证明这个度量与真实物理量**反相关**,而且几乎不携带伸展信息:

  * 全 0 姿态(原来当作"收纳基准")的实际水平伸展是 0.4205 m,已经顶到规划足迹
    半径 0.42 m —— 它根本不是收拢姿态(肘部完全伸直),却因为偏差为 0 而完全不限速;
  * 肘部折回的真实收纳姿态水平伸展只有 0.3532 m、臂质心水平偏移 0.0025 m,
    却因为关节偏差达到 2.4 rad 而被限到系数下限;
  * 全工作空间随机采样 4000 次:最大伸展 0.8865 m 时关节偏差 3.062,
    最小伸展 0.1997 m 时关节偏差 3.079 —— 偏差几乎相同,伸展差 4.4 倍。

所以换成"监控连杆相对底盘回转轴的水平距离(伸展量,单位 m)",阈值也随之变成物理
长度。连杆位姿从 TF(即 URDF/robot_state_publisher)读,本文件不含任何 DH 参数、
连杆长度或质量常数。

阈值定标依据(全部由 URDF 实测,详见包 README 的倾覆余量评估):
  * reach_folded_m 默认取 0.42 = Nav2 的 robot_radius —— 机械臂还在底盘足迹以内
    时,规划器已经把它算进去了,不需要额外限速;
  * reach_full_m 默认取 0.8865 = 实测全工作空间最大水平伸展,让活跃度铺满真实
    动态范围,而不是铺满一个凭感觉取的角度区间。
"""

# 度量方式取值。joint_deviation 是被证伪的旧度量,保留下来只为 A/B 回归对比和
# 一键回退,不是推荐值。
METRIC_HORIZONTAL_REACH = 'horizontal_reach'
METRIC_JOINT_DEVIATION = 'joint_deviation'
VALID_METRICS = (METRIC_HORIZONTAL_REACH, METRIC_JOINT_DEVIATION)


class ReachMetricConfigError(ValueError):
    """伸展度量阈值配置非法(比如 folded >= full 会导致除零/量程反向)。"""


def validate_reach_thresholds(reach_folded_m, reach_full_m):
    """校验伸展阈值区间。非法配置显式抛错,不做静默夹紧——静默夹紧会让一个
    配错的阈值表现成"限速好像失效了",比直接报错难查得多。"""
    if reach_folded_m < 0.0:
        raise ReachMetricConfigError(
            'reach_folded_m=%r 不能为负(它是一个到回转轴的水平距离)' % (reach_folded_m,))
    if reach_full_m <= reach_folded_m:
        raise ReachMetricConfigError(
            'reach_full_m=%r 必须严格大于 reach_folded_m=%r,否则活跃度区间为空/反向'
            % (reach_full_m, reach_folded_m))


def horizontal_reach(x, y):
    """连杆原点相对底盘回转轴的水平距离。只取 xy——竖直方向的抬高不增加倾覆
    力臂,把 z 也算进去会让"手臂举高但贴着身体"被误判成大幅展开。"""
    return (float(x) ** 2 + float(y) ** 2) ** 0.5


def reach_activity(max_reach_m, reach_folded_m, reach_full_m):
    """把最大水平伸展换算成 0~1.0 活跃度。

    低于 reach_folded_m 记 0(足迹以内,规划器已覆盖),高于 reach_full_m 封顶 1.0。
    调用前必须已经 validate_reach_thresholds 过。
    """
    span = reach_full_m - reach_folded_m
    if span <= 0.0:
        raise ReachMetricConfigError(
            'reach 区间非法(full=%r <= folded=%r)' % (reach_full_m, reach_folded_m))
    ratio = (float(max_reach_m) - reach_folded_m) / span
    return max(0.0, min(1.0, ratio))


def joint_deviation_activity(name_to_pos, joint_names, reference_rad, extension_full_rad):
    """旧度量:关节角相对参考姿态的最大偏差 / 满量程。仅供回归对比,已被证伪。

    joint_states 里缺失的关节直接跳过(比如机械臂话题还没上线),不当成 0 偏差——
    当成 0 会伪造出"手臂已收拢"的假象。
    """
    if extension_full_rad <= 1e-6:
        return 0.0
    activity = 0.0
    for joint_name, ref in zip(joint_names, reference_rad):
        pos = name_to_pos.get(joint_name)
        if pos is None:
            continue
        activity = max(activity, min(1.0, abs(pos - ref) / extension_full_rad))
    return activity


def velocity_activity(name_to_vel, joint_names, velocity_full_rad_s):
    """速率维度活跃度:任意关节角速度绝对值 / 满量程。

    这一维跟"参考姿态"无关,C1 没有改动它——手臂高速运动时的反作用力矩是独立且
    合理的约束。
    """
    if velocity_full_rad_s <= 1e-6:
        return 0.0
    activity = 0.0
    for joint_name in joint_names:
        vel = name_to_vel.get(joint_name)
        if vel is None:
            continue
        activity = max(activity, min(1.0, abs(vel) / velocity_full_rad_s))
    return activity


def scale_from_activity(activity, min_speed_scale):
    """活跃度 → 限速系数。activity=0 不限速,activity=1 降到 min_speed_scale。"""
    raw = 1.0 - float(activity) * (1.0 - min_speed_scale)
    return max(min_speed_scale, min(1.0, raw))
