#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""姿态阈值判定。数据源是否支持姿态由配置指定，不能从静止样本推断。"""

import math

MIN_SAMPLES_FOR_DEGENERACY = 20

DEGENERACY_EPS = 1e-9


def posture_out_of_bounds(z, roll, pitch,
                          normal_height, max_height_deviation, max_tilt_rad):
    """姿态是否超出允许范围。返回 (是否超限, 原因字符串或 None)。

    刻意返回原因：原实现只打一行"检测到异常姿态(z=.. roll=.. pitch=..)"，
    三个量一起报，读日志的人得自己比对三个阈值才知道是哪一项触发的。
    """
    if not all(math.isfinite(v) for v in (z, roll, pitch)):
        return True, '姿态含非有限数值'
    if abs(z - normal_height) > max_height_deviation:
        return True, ('高度偏差 |%.4f - %.4f| = %.4f 超过 %.4f'
                      % (z, normal_height, abs(z - normal_height),
                         max_height_deviation))
    if abs(roll) > max_tilt_rad:
        return True, ('横滚 |%.4f| 超过 %.4f' % (roll, max_tilt_rad))
    if abs(pitch) > max_tilt_rad:
        return True, ('俯仰 |%.4f| 超过 %.4f' % (pitch, max_tilt_rad))
    return False, None


def is_degenerate_attitude_source(samples, eps=DEGENERACY_EPS,
                                 min_samples=MIN_SAMPLES_FOR_DEGENERACY):
    """仅检查样本是否恒定；恒定不能证明数据源缺少姿态能力。"""
    if len(samples) < min_samples:
        return False
    for idx in range(3):
        col = [s[idx] for s in samples]
        if max(col) - min(col) > eps:
            return False
    return True


def describe_monitor_state(enabled, degenerate, tripped):
    """把三个布尔量翻译成一句人能读的状态，供启动日志与止损日志共用。

    存在的理由：原实现里"监控开着但收不到数据"与"监控开着且一切正常"
    在日志上**完全无法区分** —— 两种情况都是那一条启动 INFO，之后再无输出。
    """
    if not enabled:
        return '姿态监控已由配置显式禁用。'
    if degenerate:
        return '姿态数据不可用。'
    if tripped:
        return '姿态监控**已止损**：检测到异常姿态，持续下发零速度。'
    return '姿态监控已启用：等待有效 /odom，采样窗口结束后执行阈值判定。'



ACT_PASS = 'pass'
ACT_COLLECTING = 'collecting'
ACT_DISABLE_DEGENERATE = 'disable_degenerate'
ACT_TRIP = 'trip'
ACT_DISABLED = 'disabled'


def evaluate_posture(enabled, samples, z, roll, pitch,
                     normal_height, max_height_deviation, max_tilt_rad,
                     min_samples=MIN_SAMPLES_FOR_DEGENERACY):
    """保留启动采样窗口及原有阈值；静止不会停用监控。"""
    if not enabled:
        return ACT_DISABLED, None
    if len(samples) < min_samples:
        return ACT_COLLECTING, None
    tripped, why = posture_out_of_bounds(
        z, roll, pitch, normal_height, max_height_deviation, max_tilt_rad)
    return (ACT_TRIP, why) if tripped else (ACT_PASS, None)
