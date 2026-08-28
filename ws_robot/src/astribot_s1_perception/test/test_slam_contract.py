#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""外部 SLAM 接入契约的离线测试。不需要 ROS / 仿真 / 真机 / 外部 SLAM。

用普通 dataclass 冒充 nav_msgs/OccupancyGrid（被测代码是鸭子类型的），
这样每一条契约违规都能被**构造**出来 —— 在线要触发一次坏数据得靠运气。
"""

from dataclasses import dataclass, field

import pytest

from astribot_s1_perception.slam_contract import (
    CANONICAL_CELL_VALUES,
    COORDINATOR_MAP_TIMEOUT_SEC,
    ContractViolation,
    MapContract,
    SlamAdapter,
)


# --- 假消息 ----------------------------------------------------------------
@dataclass
class Quat:
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Pose:
    orientation: Quat = field(default_factory=Quat)


@dataclass
class Info:
    resolution: float = 0.05
    width: int = 4
    height: int = 3
    origin: Pose = field(default_factory=Pose)


@dataclass
class Stamp:
    sec: int = 100
    nanosec: int = 0


@dataclass
class Header:
    stamp: Stamp = field(default_factory=Stamp)
    frame_id: str = 'map'


@dataclass
class Grid:
    info: Info = field(default_factory=Info)
    header: Header = field(default_factory=Header)
    data: list = field(default_factory=lambda: [0] * 12)


def good_grid(**kwargs):
    grid = Grid()
    for key, value in kwargs.items():
        setattr(grid, key, value)
    return grid


# ---------------------------------------------------------------------------
# 一、构造期校验：心跳必须快于协调器超时
#
# 这个错误在线只表现为"探索偶发停住"，而 /map 一直有数据 —— 所以必须在
# 构造期就挡住，不能留到运行时。
# ---------------------------------------------------------------------------
def test_default_contract_accepts_a_good_grid():
    assert MapContract().validate_grid(good_grid(), now_sec=100.0) == []


@pytest.mark.parametrize('period', [10.0, 10.1, 60.0])
def test_republish_slower_than_coordinator_timeout_rejected(period):
    with pytest.raises(ContractViolation) as excinfo:
        MapContract(republish_period_sec=period)
    message = str(excinfo.value)
    assert 'map_timeout_sec' in message
    # 必须说出后果，否则排查会停在"它报错了"
    assert '探索偶发停住' in message


def test_republish_just_under_timeout_accepted():
    period = COORDINATOR_MAP_TIMEOUT_SEC - 0.1
    assert MapContract(republish_period_sec=period).republish_period_sec == period


def test_non_positive_resolution_expected_rejected():
    with pytest.raises(ContractViolation):
        MapContract(resolution_expected=0.0)


# ---------------------------------------------------------------------------
# 二、逐项契约违规
# ---------------------------------------------------------------------------
@pytest.mark.parametrize('width,height', [(0, 3), (4, 0), (0, 0)])
def test_empty_grid_rejected(width, height):
    grid = good_grid(info=Info(width=width, height=height), data=[])
    problems = MapContract().validate_grid(grid)
    assert len(problems) == 1
    assert 'width/height 为 0' in problems[0]


def test_data_length_mismatch_rejected_with_both_numbers():
    grid = good_grid(data=[0] * 11)          # 4x3 应为 12
    problems = MapContract().validate_grid(grid)
    assert any('11' in p and '12' in p for p in problems)
    assert any('行主序' in p for p in problems)


def test_wrong_resolution_rejected():
    grid = good_grid(info=Info(resolution=0.10))
    problems = MapContract().validate_grid(grid)
    assert any('resolution=0.1' in p for p in problems)


def test_non_positive_resolution_rejected():
    grid = good_grid(info=Info(resolution=0.0))
    assert MapContract().validate_grid(grid)


@pytest.mark.parametrize('quat', [
    Quat(w=0.707, z=0.707),      # 绕 z 转 90°
    Quat(w=1.0, x=1e-3),         # 微小倾斜也要拦：不会报错，只是路径系统性偏移
])
def test_rotated_origin_rejected(quat):
    grid = good_grid(info=Info(origin=Pose(orientation=quat)))
    problems = MapContract().validate_grid(grid)
    assert any('单位四元数' in p for p in problems)


def test_intermediate_cell_values_rejected_in_strict_mode():
    grid = good_grid(data=[0] * 11 + [55])
    problems = MapContract().validate_grid(grid)
    assert any('55' in p for p in problems)
    # 必须给出放宽的开关名，否则用户只知道被拒不知道怎么办
    assert any('strict_cell_values' in p for p in problems)


def test_intermediate_cell_values_accepted_when_relaxed():
    grid = good_grid(data=[0] * 11 + [55])
    assert MapContract(strict_cell_values=False).validate_grid(grid) == []


def test_out_of_range_rejected_even_when_relaxed():
    grid = good_grid(data=[0] * 11 + [200])
    problems = MapContract(strict_cell_values=False).validate_grid(grid)
    assert any('越界' in p for p in problems)


def test_all_canonical_values_accepted():
    data = [CANONICAL_CELL_VALUES[i % 3] for i in range(12)]
    assert MapContract().validate_grid(good_grid(data=data)) == []


# ---------------------------------------------------------------------------
# 三、时间戳：只查超前，不查滞后
#
# 滞后是正常的（SLAM 有延迟，而且我们自己就在重发旧帧）。超前说明时钟没对齐，
# 实机走 PTP、开机时钟是 1970，这个坑真实存在。
# ---------------------------------------------------------------------------
def test_stamp_ahead_of_local_clock_rejected():
    grid = good_grid(header=Header(stamp=Stamp(sec=105)))
    problems = MapContract().validate_grid(grid, now_sec=100.0)
    assert any('超前' in p and 'PTP' in p for p in problems)


def test_stamp_within_tolerance_accepted():
    grid = good_grid(header=Header(stamp=Stamp(sec=100, nanosec=500_000_000)))
    assert MapContract().validate_grid(grid, now_sec=100.0) == []


def test_lagging_stamp_is_not_a_violation():
    """重发旧帧就必然滞后，若查滞后会把正常心跳判成违规。"""
    grid = good_grid(header=Header(stamp=Stamp(sec=1)))
    assert MapContract().validate_grid(grid, now_sec=100.0) == []


def test_stamp_check_skipped_when_no_clock_given():
    grid = good_grid(header=Header(stamp=Stamp(sec=99999)))
    assert MapContract().validate_grid(grid, now_sec=None) == []


# ---------------------------------------------------------------------------
# 四、转发 / 心跳状态机
#
# 时间全部由测试传入，不 sleep、不读墙钟 —— 本仓库有过"用墙钟时间戳让测试结论
# 作废"的先例（docs/ros2-test-harness-pitfalls）。
# ---------------------------------------------------------------------------
def test_good_map_is_forwarded_and_counted():
    adapter = SlamAdapter()
    grid = good_grid()
    assert adapter.on_source_map(grid, now_sec=100.0) is grid
    assert adapter.stats.received == 1
    assert adapter.stats.forwarded == 1
    assert adapter.stats.rejected == 0
    assert adapter.has_map is True


def test_bad_map_is_not_forwarded_and_reasons_are_kept():
    adapter = SlamAdapter()
    assert adapter.on_source_map(good_grid(data=[0] * 5), now_sec=100.0) is None
    assert adapter.stats.rejected == 1
    assert adapter.stats.forwarded == 0
    assert adapter.stats.last_reject_reasons  # 拒绝原因必须留下来
    assert adapter.has_map is False


def test_bad_map_does_not_clobber_the_last_good_one():
    """收到坏帧后**不能**把缓存清掉，否则心跳跟着断，症状变成"地图超时"。"""
    adapter = SlamAdapter()
    good = good_grid()
    adapter.on_source_map(good, now_sec=100.0)
    adapter.on_source_map(good_grid(data=[0] * 5), now_sec=101.0)
    assert adapter.has_map is True
    assert adapter.republish(now_sec=106.0) is good


def test_reject_reasons_cleared_after_a_good_frame():
    adapter = SlamAdapter()
    adapter.on_source_map(good_grid(data=[0] * 5), now_sec=100.0)
    adapter.on_source_map(good_grid(), now_sec=101.0)
    assert adapter.stats.last_reject_reasons == ()


def test_no_republish_before_period_elapses():
    adapter = SlamAdapter()
    adapter.on_source_map(good_grid(), now_sec=100.0)
    assert adapter.due_for_republish(104.9) is False
    assert adapter.republish(104.9) is None
    assert adapter.stats.republished == 0


def test_republish_at_exactly_the_period():
    adapter = SlamAdapter()
    grid = good_grid()
    adapter.on_source_map(grid, now_sec=100.0)
    assert adapter.due_for_republish(105.0) is True
    assert adapter.republish(105.0) is grid
    assert adapter.stats.republished == 1
    # 重发后计时重置：下一次要再等一个周期
    assert adapter.due_for_republish(105.1) is False


def test_never_republishes_before_any_map_arrives():
    """没有可发的帧时不能拿空帧填坑 —— 空 /map 会让 costmap 无法初始化。"""
    adapter = SlamAdapter()
    assert adapter.due_for_republish(1e6) is False
    assert adapter.republish(1e6) is None


def test_fresh_source_map_resets_the_heartbeat():
    """上游自己在发时不该额外重发 —— 否则 /map 上出现两倍流量。"""
    adapter = SlamAdapter()
    adapter.on_source_map(good_grid(), now_sec=100.0)
    adapter.on_source_map(good_grid(), now_sec=104.0)
    assert adapter.due_for_republish(105.0) is False
    assert adapter.due_for_republish(109.0) is True


def test_heartbeat_gap_always_stays_under_coordinator_timeout():
    """不变量：任意时刻两次发出之间的间隔 < 协调器超时。

    这条比"周期等于 5.0"更有价值：它是下游真正依赖的性质，
    改 republish_period_sec 时若不小心越过 10.0，这里会红。
    """
    adapter = SlamAdapter()
    # 起点用 100.0 而不是 0.0：假消息的默认 stamp 是 sec=100，
    # 从 0 起会先被"stamp 超前"拒掉，一帧都进不去缓存。
    adapter.on_source_map(good_grid(), now_sec=100.0)
    emitted = [100.0]
    now = 100.0
    while now < 200.0:                       # 上游此后完全静默
        now += 0.5
        if adapter.republish(now) is not None:
            emitted.append(now)
    gaps = [b - a for a, b in zip(emitted, emitted[1:])]
    assert gaps, '整个 100s 里一次都没重发'
    assert max(gaps) < COORDINATOR_MAP_TIMEOUT_SEC
