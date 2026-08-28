#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`chassis_odom_source` 的离线测试。不需要 ROS / SDK / 真机 / 仿真。

这一层填的是 `odom → astribot_torso_base` 那条真机上**没人发布**的边，
而它的前提（"SDK 位姿允许漂、不允许跳"）只能靠跳变计数来取证 ——
所以跳变相关的行为是本文件的重点。
"""

import math

import pytest

from astribot_trajectory_bridge.chassis_odom_source import (
    ChassisOdomSource,
    OdomSourceError,
    OdomSample,
)


# ---------------------------------------------------------------------------
# 一、构造期校验
# ---------------------------------------------------------------------------
@pytest.mark.parametrize('bad', [0.0, -0.1])
def test_non_positive_jump_threshold_rejected(bad):
    with pytest.raises(OdomSourceError) as excinfo:
        ChassisOdomSource(jump_threshold_m=bad)
    # 必须说出后果：每帧都算跳变会把日志刷满、真正的跳变被埋掉
    assert '刷满' in str(excinfo.value)


def test_unknown_velocity_frame_rejected():
    with pytest.raises(OdomSourceError) as excinfo:
        ChassisOdomSource(velocity_frame='map')
    assert 'body|world' in str(excinfo.value)


# ---------------------------------------------------------------------------
# 二、输入健壮性：宁可失败也不补零
# ---------------------------------------------------------------------------
@pytest.mark.parametrize('bad', [[], [0.0], [0.0, 0.0], [0.0] * 4])
def test_wrong_length_rejected(bad):
    source = ChassisOdomSource()
    with pytest.raises(OdomSourceError) as excinfo:
        source.sample(bad, [0.0, 0.0, 0.0])
    assert '3 个数' in str(excinfo.value)


def test_none_input_names_the_sdk_call_to_check():
    source = ChassisOdomSource()
    with pytest.raises(OdomSourceError) as excinfo:
        source.sample(None, [0.0, 0.0, 0.0])
    # 报错要指向可执行的下一步，而不只是说"没数据"
    assert 'astribot_chassis' in str(excinfo.value)


def test_velocity_length_checked_too():
    source = ChassisOdomSource()
    with pytest.raises(OdomSourceError):
        source.sample([0.0, 0.0, 0.0], [0.0, 0.0])


def test_integer_input_accepted():
    """SDK 有可能给回 int；不该因为类型就失败。"""
    sample = ChassisOdomSource().sample([0, 0, 0], [0, 0, 0])
    assert sample.x == 0.0


# ---------------------------------------------------------------------------
# 三、位姿与四元数
# ---------------------------------------------------------------------------
def test_pose_passed_through():
    sample = ChassisOdomSource().sample([1.5, -2.5, 0.3], [0.0, 0.0, 0.0])
    assert (sample.x, sample.y) == (1.5, -2.5)
    assert sample.theta == pytest.approx(0.3)


def test_theta_wrapped_into_pi_range():
    """不 wrap 的话四元数仍然对，但下游打印出来的角度会越界，误导排查。"""
    sample = ChassisOdomSource().sample([0.0, 0.0, 4.0 * math.pi + 0.5],
                                        [0.0, 0.0, 0.0])
    assert -math.pi <= sample.theta <= math.pi
    assert sample.theta == pytest.approx(0.5)


@pytest.mark.parametrize('theta', [0.0, 0.5, -0.5, math.pi / 2, 3.0])
def test_quaternion_matches_theta(theta):
    z, w = OdomSample(x=0, y=0, theta=theta, vx_body=0, vy_body=0, wz=0).quaternion_zw
    assert z == pytest.approx(math.sin(theta / 2))
    assert w == pytest.approx(math.cos(theta / 2))
    # 单位四元数（底盘只有 yaw，x/y 恒 0，所以 z²+w²=1）
    assert z * z + w * w == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# 四、速度坐标系
#
# Odometry.twist 按规定在 child_frame（机体系）里表达。搞反不报错，
# 只让 nav2 的速度前瞻在转向时系统性偏一个旋转。
# ---------------------------------------------------------------------------
def test_body_frame_velocity_passed_through():
    source = ChassisOdomSource(velocity_frame='body')
    sample = source.sample([0.0, 0.0, math.pi / 2], [1.0, 0.0, 0.2])
    assert (sample.vx_body, sample.vy_body) == (1.0, 0.0)
    assert sample.wz == 0.2


def test_world_frame_velocity_rotated_into_body():
    """朝向 +90° 时，世界系 +x 的速度在机体系里是 -y。"""
    source = ChassisOdomSource(velocity_frame='world')
    sample = source.sample([0.0, 0.0, math.pi / 2], [1.0, 0.0, 0.0])
    assert sample.vx_body == pytest.approx(0.0, abs=1e-9)
    assert sample.vy_body == pytest.approx(-1.0)


def test_world_rotation_is_identity_at_zero_heading():
    source = ChassisOdomSource(velocity_frame='world')
    sample = source.sample([0.0, 0.0, 0.0], [0.3, -0.4, 0.0])
    assert sample.vx_body == pytest.approx(0.3)
    assert sample.vy_body == pytest.approx(-0.4)


def test_world_rotation_preserves_speed_magnitude():
    """旋转不该改变速度大小 —— 这条能拦住把旋转矩阵写转置反了的错。"""
    for theta in (0.0, 0.7, -1.3, 2.9):
        source = ChassisOdomSource(velocity_frame='world')
        sample = source.sample([0.0, 0.0, theta], [0.3, -0.4, 0.0])
        assert math.hypot(sample.vx_body, sample.vy_body) == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# 五、跳变检测 —— 本文件的重点
#
# "SDK 位姿能当 odom"这个前提**只能**靠跳变计数取证。所以：
#   · 第一帧不能算跳变（没有前一帧可比）
#   · 跳变要照常发布（藏起来比让人看见更糟）
#   · 跳变不计入行程（一次重定位几十厘米，会污染漂移速率的度量）
# ---------------------------------------------------------------------------
def test_first_sample_is_never_a_jump():
    """没有前一帧可比。若第一帧就算跳变，每次启动都会误报一次。"""
    sample = ChassisOdomSource(jump_threshold_m=0.3).sample([99.0, 99.0, 0.0],
                                                           [0.0, 0.0, 0.0])
    assert sample.jumped is False
    assert sample.jump_m == 0.0


def test_small_motion_is_not_a_jump():
    source = ChassisOdomSource(jump_threshold_m=0.30)
    source.sample([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    sample = source.sample([0.29, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert sample.jumped is False
    assert source.stats.jumps == 0


def test_large_step_is_flagged_as_a_jump():
    source = ChassisOdomSource(jump_threshold_m=0.30)
    source.sample([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    sample = source.sample([5.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert sample.jumped is True
    assert sample.jump_m == pytest.approx(5.0)
    assert source.stats.jumps == 1
    assert source.stats.max_jump_m == pytest.approx(5.0)


def test_jump_is_still_published():
    """跳变照常发布：藏起来会让下游拿不到真实位姿，比让它看到跳变更糟。"""
    source = ChassisOdomSource(jump_threshold_m=0.30)
    source.sample([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    sample = source.sample([5.0, 3.0, 0.0], [0.0, 0.0, 0.0])
    assert (sample.x, sample.y) == (5.0, 3.0)     # 未被平滑、未被夹住


def test_jump_excluded_from_travelled_distance():
    """一次重定位能有几十厘米。计进行程就把漂移速率的度量污染了。"""
    source = ChassisOdomSource(jump_threshold_m=0.30)
    for x in (0.0, 0.1, 0.2):                      # 正常走 0.2m
        source.sample([x, 0.0, 0.0], [0.0, 0.0, 0.0])
    source.sample([10.0, 0.0, 0.0], [0.0, 0.0, 0.0])   # 一次跳变
    source.sample([10.1, 0.0, 0.0], [0.0, 0.0, 0.0])   # 跳变后继续走 0.1m
    assert source.stats.travelled_m == pytest.approx(0.3)
    assert source.stats.jumps == 1


def test_jump_ratio_is_zero_for_a_clean_run():
    """这个数不为 0 就意味着"SDK 位姿能当 odom"的前提不成立。"""
    source = ChassisOdomSource(jump_threshold_m=0.30)
    for i in range(50):
        source.sample([i * 0.01, 0.0, 0.0], [0.5, 0.0, 0.0])
    assert source.jump_ratio == 0.0
    assert source.stats.samples == 50


def test_jump_ratio_before_any_sample_is_zero_not_a_crash():
    assert ChassisOdomSource().jump_ratio == 0.0


def test_slow_drift_is_not_reported_as_jumps():
    """odom 的契约是允许漂、不允许跳。持续小步漂移必须一次都不报跳变。"""
    source = ChassisOdomSource(jump_threshold_m=0.30)
    x = 0.0
    for _ in range(500):
        x += 0.05                                   # 每帧 5cm，累计 25m
        source.sample([x, 0.0, 0.0], [1.0, 0.0, 0.0])
    assert source.stats.jumps == 0
    assert source.stats.travelled_m == pytest.approx(24.95)


def test_jump_history_is_bounded():
    """节点要长跑。无界增长的记录本身就是隐患（长跑退化那次的教训）。"""
    from astribot_trajectory_bridge.chassis_odom_source import MAX_JUMP_HISTORY
    source = ChassisOdomSource(jump_threshold_m=0.30)
    for i in range(MAX_JUMP_HISTORY * 3):
        source.sample([i * 10.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert len(source.stats.jump_history) == MAX_JUMP_HISTORY
    assert source.stats.jumps > MAX_JUMP_HISTORY     # 计数本身不被截断
