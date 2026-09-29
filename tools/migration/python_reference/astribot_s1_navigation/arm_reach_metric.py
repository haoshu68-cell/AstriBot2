#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂水平伸展度量，不依赖 rclpy/TF。

与 dynamics_coupling 中的实现保持相同度量口径，但两包独立部署、阈值独立维护。
修改时需核对两份实现。水平伸展取监控连杆到回转轴的 xy 距离；
关节角偏差不能可靠表示伸展程度，旧接口仅供回归对比。
"""

import os

if os.environ.get('ASTRIBOT_BRIDGE_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
    try:
        from astribot_trajectory_bridge_native import _chassis_math_native as _native
    except ImportError:  # pragma: no cover
        _native = None
else:
    _native = None

METRIC_HORIZONTAL_REACH = 'horizontal_reach'
METRIC_JOINT_DEVIATION = 'joint_deviation'
VALID_METRICS = (METRIC_HORIZONTAL_REACH, METRIC_JOINT_DEVIATION)


class ReachMetricConfigError(ValueError):
    """展开判据阈值配置非法。"""


def horizontal_reach(x, y):
    """连杆原点相对底盘回转轴的水平距离。只取 xy——竖直抬高不增加倾覆力臂。"""
    return _native.horizontal_reach(float(x), float(y)) if _native is not None \
        else (float(x) ** 2 + float(y) ** 2) ** 0.5


def validate_extended_reach(extended_reach_m, hysteresis_m=0.0):
    """校验正值阈值及 0 <= hysteresis_m < extended_reach_m；非法值显式抛错。"""
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
    """伸展超过阈值时置位；降至阈值减迟滞量时解除，避免边界抖动。

    was_extended 为上一次结果，首次调用传 False。
    """
    reach = float(max_reach_m)
    if _native is not None:
        return _native.is_extended_by_reach(
            reach, float(extended_reach_m), float(hysteresis_m), bool(was_extended))
    if was_extended:
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
