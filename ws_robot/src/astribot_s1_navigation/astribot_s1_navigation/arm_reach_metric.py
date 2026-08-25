#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文件用途：机械臂"展开程度"度量的纯函数实现(不依赖 rclpy/TF，便于单测)。

!!! 本文件与 astribot_s1_dynamics_coupling/arm_reach_metric.py 是**故意重复**的两份 !!!
跟本包 ARM_JOINT_NAMES 那份重复同一个理由：静态限速节点(走 Nav2 官方 /speed_limit
机制、二值判断)和耦合包的连续调速节点是两套完全独立、互不 import 的保护，两个包
之间不建立依赖。两边的度量口径必须一致(都是"监控连杆相对底盘回转轴的水平距离")，
但阈值各自独立维护——改一边的时候要记得看另一边。

!!! 为什么从"关节角偏差"换成"水平伸展"(C1 修正，基于实测证据) !!!
原实现用"双臂 14 个关节相对全 0 参考姿态的最大角度偏差 > 0.5rad"判"展开"，实测
证明这个判据与真实伸展**反相关**：
  * 全 0 姿态实际水平伸展 0.4205 m(肘部完全伸直)，偏差为 0 → 判"已收纳"、不限速；
  * 肘部折回的真实收纳姿态伸展只有 0.3532 m，偏差 2.4 rad → 判"展开"、限速 50%；
  * 全工作空间采样 4000 次：最大伸展 0.8865 m 时偏差 3.062，最小伸展 0.1997 m 时
    偏差 3.079——偏差几乎相同，伸展差 4.4 倍。
后果是任何作业姿态都被判"展开"、恒定砍到 50%，再乘上耦合节点的系数(实测触底
0.15)，合计 0.075，把 desired_linear_vel 0.5 压到约 0.037 m/s。

阈值改成物理长度后，默认 extended_reach_m = 0.42 = 本包 costmap 的 robot_radius：
机械臂伸出规划足迹之外才判"展开"，这个判据是可解释的。
"""

METRIC_HORIZONTAL_REACH = 'horizontal_reach'
METRIC_JOINT_DEVIATION = 'joint_deviation'
VALID_METRICS = (METRIC_HORIZONTAL_REACH, METRIC_JOINT_DEVIATION)


class ReachMetricConfigError(ValueError):
    """展开判据阈值配置非法。"""


def horizontal_reach(x, y):
    """连杆原点相对底盘回转轴的水平距离。只取 xy——竖直抬高不增加倾覆力臂。"""
    return (float(x) ** 2 + float(y) ** 2) ** 0.5


def validate_extended_reach(extended_reach_m, hysteresis_m=0.0):
    """校验展开判据阈值。非法值显式抛错，不静默取默认——静默取默认会让一个配错的
    阈值表现成"限速好像没生效"，比直接报错难查得多。"""
    if extended_reach_m <= 0.0:
        raise ReachMetricConfigError(
            'extended_reach_m=%r 必须为正(它是一个到回转轴的水平距离)'
            % (extended_reach_m,))
    if hysteresis_m < 0.0:
        raise ReachMetricConfigError(
            'extended_reach_hysteresis_m=%r 不能为负' % (hysteresis_m,))
    if hysteresis_m >= extended_reach_m:
        raise ReachMetricConfigError(
            'extended_reach_hysteresis_m=%r 不能 >= extended_reach_m=%r，'
            '否则解除阈值会落到 0 或负数，判据永真'
            % (hysteresis_m, extended_reach_m))


def is_extended_by_reach(max_reach_m, extended_reach_m,
                         hysteresis_m=0.0, was_extended=False):
    """水平伸展判据(带迟滞)：最大伸展超过阈值 → 判"展开"。

    !!! 为什么需要迟滞 !!!
    实测全 0 姿态的水平伸展是 0.4205 m，而默认阈值 extended_reach_m = 0.42
    (= costmap 的 robot_radius)——两者只差 0.5 mm。这是个纯巧合，但后果很实际：
    双臂静置时伸展就压在阈值上，关节的微小抖动会让二值判据在"展开/收纳"之间来回
    翻转，而本节点每次翻转都会发一条 SpeedLimit，velocity_smoother 的速度上限就会
    在 50% 和 100% 之间反复跳。所以一旦判定为展开，要求伸展降到
    (extended_reach_m - hysteresis_m) 以下才解除。

    was_extended 是上一次的判定结果，首次调用传 False(即按"未展开"起步，随后
    第一帧就会按真实伸展给出正确判定)。
    """
    reach = float(max_reach_m)
    if was_extended:
        # 已经在限速中：要低于解除阈值才松开
        return reach > (float(extended_reach_m) - float(hysteresis_m))
    return reach > float(extended_reach_m)


def max_joint_deviation(name_to_pos, joint_names, reference_rad):
    """旧判据用的最大关节偏差。仅供回归对比，已被证伪。

    joint_states 里缺失的关节跳过，不当成 0 偏差——当成 0 会伪造"已收拢"的假象。
    """
    max_dev = 0.0
    for joint_name, ref in zip(joint_names, reference_rad):
        pos = name_to_pos.get(joint_name)
        if pos is None:
            continue
        max_dev = max(max_dev, abs(pos - ref))
    return max_dev
