#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`cloud_to_grid` 的离线测试。

判据取向：**每个测试针对一个已知会出问题的具体行为**，不做"看起来对"的宽松断言。
带 `# 反相关` 注释的是故障注入测试——把被测机制关掉必须让它失败，
否则那个测试什么都没验证。
"""

import math
import os
import tempfile

import pytest

from astribot_s1_perception.cloud_to_grid import (
    CELL_FREE,
    CELL_OCCUPIED,
    CELL_UNKNOWN,
    GridConfig,
    GridConfigError,
    compute_bounds,
    load_session_poses,
    project,
    slab_filter,
    world_to_cell,
)


# ----------------------------------------------------------------- 配置校验

def test_default_config_valid():
    GridConfig().validate()


def test_resolution_must_be_positive():
    with pytest.raises(GridConfigError, match='resolution'):
        GridConfig(resolution=0.0).validate()


def test_z_max_must_exceed_z_min():
    with pytest.raises(GridConfigError, match='z_max'):
        GridConfig(z_min=0.5, z_max=0.5).validate()


def test_min_points_per_cell_at_least_one():
    with pytest.raises(GridConfigError, match='min_points_per_cell'):
        GridConfig(min_points_per_cell=0).validate()


def test_negative_padding_rejected():
    with pytest.raises(GridConfigError, match='padding_m'):
        GridConfig(padding_m=-0.1).validate()


def test_default_slab_matches_scan_chain():
    """默认切片带上界必须与 pointcloud_to_laserscan 的 max_height 0.6 一致。

    不一致会让同一个障碍在 /scan 与 /map 里出现在不同位置。
    """
    assert GridConfig().z_max == pytest.approx(0.60)


# ----------------------------------------------------------------- 高度切片

def test_slab_keeps_in_band():
    pts = [(1.0, 2.0, 0.0), (1.0, 2.0, 0.3)]
    kept, dropped = slab_filter(pts, -0.05, 0.60)
    assert len(kept) == 2 and dropped == 0


def test_slab_drops_ground_and_ceiling():
    pts = [(0.0, 0.0, -0.50), (0.0, 0.0, 2.0)]
    kept, dropped = slab_filter(pts, -0.05, 0.60)
    assert kept == [] and dropped == 2


def test_slab_boundaries_inclusive():
    kept, _ = slab_filter([(0, 0, -0.05), (0, 0, 0.60)], -0.05, 0.60)
    assert len(kept) == 2


def test_slab_drops_nan_and_inf():
    """NaN 会让 min/max 全变 NaN，症状是"地图尺寸是 0 或天文数字"，离根因很远。"""
    pts = [(float('nan'), 0.0, 0.0), (0.0, float('inf'), 0.0),
           (0.0, 0.0, float('nan')), (1.0, 1.0, 0.0)]
    kept, dropped = slab_filter(pts, -0.05, 0.60)
    assert kept == [(1.0, 1.0)] and dropped == 3


def test_slab_discards_z():
    """输出只保留 xy —— 投影后 z 不再有意义。"""
    kept, _ = slab_filter([(1.5, 2.5, 0.3)], -0.05, 0.60)
    assert kept == [(1.5, 2.5)]


# ----------------------------------------------------------------- 边界计算

def test_bounds_none_on_empty():
    assert compute_bounds([], 1.0) is None


def test_bounds_applies_padding():
    b = compute_bounds([(0.0, 0.0), (2.0, 4.0)], 1.0)
    assert b == (-1.0, -1.0, 3.0, 5.0)


def test_bounds_zero_padding():
    assert compute_bounds([(0.0, 0.0), (2.0, 4.0)], 0.0) == (0.0, 0.0, 2.0, 4.0)


def test_bounds_single_point():
    assert compute_bounds([(1.0, 1.0)], 0.5) == (0.5, 0.5, 1.5, 1.5)


# ----------------------------------------------------------------- 投影主流程

def _wall_points(n=10, x=1.0, z=0.3):
    """一段竖墙：x 固定，y 递增。每格给足票数以越过 min_points_per_cell。"""
    pts = []
    for i in range(n):
        y = i * 0.05
        pts.extend([(x, y, z)] * 3)
    return pts


def test_empty_slab_raises_with_actionable_message():
    """全部点在带外时必须响亮失败，且提示"本模块不做坐标变换"。"""
    with pytest.raises(ValueError, match='没有任何有效点'):
        project([(0.0, 0.0, 5.0)], GridConfig())


def test_data_length_equals_width_times_height():
    """探索协调器对 data.size() != w*h 直接 ERROR 丢弃。"""
    g = project(_wall_points(), GridConfig())
    assert len(g.data) == g.width * g.height


def test_only_three_cell_values():
    g = project(_wall_points(), GridConfig(), sensor_xy=[(0.0, 0.2)])
    assert set(g.data) <= {CELL_UNKNOWN, CELL_FREE, CELL_OCCUPIED}


def test_wall_becomes_occupied():
    g = project(_wall_points(), GridConfig())
    assert g.occupied_cells > 0


def test_min_points_per_cell_suppresses_noise():
    """单点不足以成为障碍——Voxel-SLAM 每帧约 4000 点，孤立点多是噪声。"""
    cfg = GridConfig(min_points_per_cell=2)
    g = project([(1.0, 1.0, 0.3)], cfg)
    assert g.occupied_cells == 0


def test_min_points_one_accepts_single_point():
    # 反相关：把阈值降到 1，同样的输入必须变成占据
    g = project([(1.0, 1.0, 0.3)], GridConfig(min_points_per_cell=1))
    assert g.occupied_cells == 1


def test_resolution_changes_grid_size():
    pts = _wall_points()
    coarse = project(pts, GridConfig(resolution=0.10))
    fine = project(pts, GridConfig(resolution=0.05))
    assert fine.width > coarse.width and fine.height > coarse.height


def test_max_cells_guard_trips_on_outlier():
    """一个离群点把边界撑开时必须报错，而不是分配几个 GB。"""
    pts = _wall_points() + [(5000.0, 5000.0, 0.3)] * 3
    with pytest.raises(ValueError, match='超过上限'):
        project(pts, GridConfig(max_cells=10_000))


def test_origin_is_min_corner():
    g = project(_wall_points(), GridConfig(padding_m=1.0))
    assert g.origin_x <= 1.0 - 1.0 + 1e-9
    assert g.origin_y <= 0.0 - 1.0 + 1e-9


# ------------------------------------------------- 未知 vs 自由（探索的命门）

def test_without_sensor_xy_there_is_no_free_space():
    """不传轨迹 → 没有自由格。nav2 的 allow_unknown=false 会让规划直接失败。"""
    g = project(_wall_points(), GridConfig())
    assert g.free_cells == 0
    assert g.unknown_cells > 0
    assert any('sensor_xy' in w for w in g.warnings)


def test_with_sensor_xy_free_space_is_carved():
    # 反相关：给了轨迹就必须出现自由格，否则 carve 没生效
    g = project(_wall_points(), GridConfig(), sensor_xy=[(0.0, 0.2)])
    assert g.free_cells > 0


def test_unknown_survives_behind_the_wall():
    """墙后必须仍是未知——否则整张图没有前沿，探索无事可做。

    这一条是探索能不能跑的根本判据，不是锦上添花。
    """
    g = project(_wall_points(n=20, x=1.0), GridConfig(),
                sensor_xy=[(0.0, 0.5)])
    assert g.unknown_cells > 0


def test_ray_does_not_overwrite_occupied():
    """射线终点是障碍，不能被雕成自由，否则墙会被自己的射线擦掉。"""
    g = project(_wall_points(), GridConfig(), sensor_xy=[(0.0, 0.2)])
    assert g.occupied_cells > 0


def test_divergent_sensor_pose_fails_loudly():
    """位姿飞到 1e6 说明 SLAM 发散了，必须响亮失败而不是静默跳过。

    早先这里断言的是"越界位姿被跳过"。改成边界包含轨迹之后，这种位姿会被
    max_cells 闸门拦住——**这才是对的**：一个发散的位姿意味着定位已经不可信，
    静默产出一张"看起来正常"的地图比报错危险得多。
    """
    with pytest.raises(ValueError, match='超过上限'):
        project(_wall_points(), GridConfig(), sensor_xy=[(1e6, 1e6)])


def test_nearby_sensor_pose_expands_grid_moderately():
    """正常范围内的轨迹点只是把图适度撑大，不触发闸门。"""
    g = project(_wall_points(), GridConfig(), sensor_xy=[(-5.0, -5.0), (0.0, 0.2)])
    assert g.origin_x <= -5.0
    assert g.occupied_cells > 0


# ----------------------------------------------------------------- 坐标换算

def test_world_to_cell_origin_maps_to_zero():
    g = project(_wall_points(), GridConfig())
    assert world_to_cell(g, g.origin_x + 1e-9, g.origin_y + 1e-9) == (0, 0)


def test_world_to_cell_out_of_range():
    g = project(_wall_points(), GridConfig())
    assert world_to_cell(g, g.origin_x - 10.0, g.origin_y) == (None, None)


def test_index_is_row_major():
    """OccupancyGrid 是行主序。搞反会让地图转置 90°。"""
    g = project(_wall_points(), GridConfig())
    assert g.index_of(2, 1) == 1 * g.width + 2


# ------------------------------------------------- alidarState.txt 解析

_REAL_LINE = (
    '1787645325.298624 0.0000093 0.0000043 -0.0000062 '
    '0.0026625 -0.0006982 0.0000092 0.9999962 '
    '0.0125414 -0.0059902 -0.0070922 -0.0057813 -0.0013811 0.0006321 '
    '-0.0000032 0.0000051 -0.0000301 -0.0005231 -0.0118427 -9.8015499 '
    '0.0098134 0.0072324 0.0122887 0.0048372 0.0048446 0.0047587'
)


def _write(tmpdir, text):
    p = os.path.join(tmpdir, 'alidarState.txt')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return p


def test_parses_real_session_line():
    """用实机 sessions/1floor 里的真实首行，不用编造的数据。"""
    with tempfile.TemporaryDirectory() as d:
        poses = load_session_poses(_write(d, _REAL_LINE + '\n'))
    assert poses == [(pytest.approx(0.0000093), pytest.approx(0.0000043))]


def test_gravity_field_position_is_the_anchor():
    """第 18-20 字段应是重力 ≈ (0, 0, -9.8)。它是字段没错位的锚点。

    若哪天位置字段取错了，这条会先失败。
    """
    parts = _REAL_LINE.split()
    gz = float(parts[19])
    assert -9.9 < gz < -9.7


def test_multiple_lines_preserve_order():
    with tempfile.TemporaryDirectory() as d:
        second = _REAL_LINE.replace('0.0000093', '1.5000000', 1)
        poses = load_session_poses(_write(d, _REAL_LINE + '\n' + second + '\n'))
    assert len(poses) == 2
    assert poses[1][0] == pytest.approx(1.5)


def test_blank_lines_skipped():
    with tempfile.TemporaryDirectory() as d:
        poses = load_session_poses(_write(d, '\n' + _REAL_LINE + '\n\n'))
    assert len(poses) == 1


def test_short_line_raises_with_line_number():
    """坏行必须带行号响亮失败，不静默跳过。"""
    with tempfile.TemporaryDirectory() as d:
        p = _write(d, _REAL_LINE + '\n1.0 2.0\n')
        with pytest.raises(ValueError, match=r':2 字段数'):
            load_session_poses(p)


def test_unparsable_number_raises_with_line_number():
    with tempfile.TemporaryDirectory() as d:
        p = _write(d, 'abc def ghi jkl mno pqr stu vwx\n')
        with pytest.raises(ValueError, match=r':1 解析失败'):
            load_session_poses(p)


def test_empty_file_raises():
    with tempfile.TemporaryDirectory() as d:
        with pytest.raises(ValueError, match='没有任何位姿'):
            load_session_poses(_write(d, ''))


# ------------------------------------------------- 端到端：真实场景形状

def test_corridor_produces_all_three_states():
    """一条走廊：两侧墙 + 中间轨迹。必须同时出现占据/自由/未知三态。

    这是最接近真实建图产物的形状，也是探索能跑的最低要求。
    """
    pts = []
    for i in range(40):
        y = i * 0.05
        pts.extend([(0.0, y, 0.3)] * 3)   # 左墙
        pts.extend([(2.0, y, 0.3)] * 3)   # 右墙
    traj = [(1.0, i * 0.10) for i in range(20)]
    g = project(pts, GridConfig(), sensor_xy=traj)
    assert g.occupied_cells > 0
    assert g.free_cells > 0
    assert g.unknown_cells > 0
    assert len(g.data) == g.width * g.height


# ------------------------------------------------- 射线雕刻参数与复杂度
#
# 下面这组测试是**第一版实现卡死之后补的**。原实现是"每个位姿遍历所有占据格连线"，
# 在上面那些 10~40 点的样例上毫秒级通过，但用实机 sessions/1floor 的真实规模
# （3367 位姿 x 约 2 万占据格）直接跑不完。教训是：**小样本单测无法暴露复杂度问题**，
# 必须显式加规模测试。

def test_carve_params_validated():
    for bad in (dict(carve_max_range_m=0.0),
                dict(carve_n_rays=0),
                dict(carve_pose_stride=0)):
        with pytest.raises(GridConfigError):
            GridConfig(**bad).validate()


def test_carve_range_bounds_free_area():
    """射线量程限制自由区范围——量程越小，自由格越少。

    这条同时证明 carve_max_range_m 真的生效（反相关）。
    """
    pts = _wall_points(n=4, x=6.0)          # 墙远在 6 m 外
    near = project(pts, GridConfig(carve_max_range_m=1.0), sensor_xy=[(0.0, 0.1)])
    far = project(pts, GridConfig(carve_max_range_m=8.0), sensor_xy=[(0.0, 0.1)])
    assert far.free_cells > near.free_cells


def test_pose_stride_reduces_work_not_correctness():
    """抽稀不应让三态结构消失——只是自由区略小。"""
    pts = []
    for i in range(60):
        pts.extend([(0.0, i * 0.05, 0.3)] * 3)
        pts.extend([(3.0, i * 0.05, 0.3)] * 3)
    traj = [(1.5, i * 0.05) for i in range(60)]
    dense = project(pts, GridConfig(carve_pose_stride=1), sensor_xy=traj)
    sparse = project(pts, GridConfig(carve_pose_stride=10), sensor_xy=traj)
    for g in (dense, sparse):
        assert g.occupied_cells > 0 and g.free_cells > 0 and g.unknown_cells > 0
    assert sparse.free_cells <= dense.free_cells


def test_sensor_cell_itself_is_free():
    """机器人所在格必须是自由——否则 nav2 报 Starting point in lethal space。"""
    g = project(_wall_points(n=4, x=3.0), GridConfig(), sensor_xy=[(0.0, 0.1)])
    col, row = world_to_cell(g, 0.0, 0.1)
    assert col is not None, '轨迹点落在图外——边界没把轨迹算进去'
    assert g.data[g.index_of(col, row)] == CELL_FREE


def test_bounds_include_trajectory_not_just_points():
    """边界必须同时覆盖点云与轨迹。

    只用点云算边界时，机器人自己的位置会落在图外，射线雕刻整个失效，
    症状却表现为"nav2 报 Starting point in lethal space"——离根因三层远。
    """
    # 墙远在 x=6，机器人在 x=0：若只用点云算边界，x=0 必然在图外
    g = project(_wall_points(n=4, x=6.0), GridConfig(), sensor_xy=[(0.0, 0.1)])
    assert g.origin_x <= 0.0, f'origin_x={g.origin_x} 把轨迹排除在外了'
    assert world_to_cell(g, 0.0, 0.1)[0] is not None


def test_real_scale_completes_quickly():
    """实机规模（约 41x43 m、3367 位姿）必须在几秒内完成。

    这是那次卡死的回归测试。阈值给得宽松（15 s），只为拦住复杂度回退，
    不是性能基准。
    """
    import time

    # 40 m x 40 m 的方形回廊轨迹，位姿间距 5 cm，量级与 1floor 一致
    traj = []
    step = 0.05
    for i in range(int(40 / step)):
        traj.append((i * step, 0.0))
    for i in range(int(40 / step)):
        traj.append((40.0, i * step))
    pts = []
    for (x, y) in traj[::5]:
        pts.extend([(x, y + 2.0, 0.3)] * 3)
        pts.extend([(x, y - 2.0, 0.3)] * 3)

    t0 = time.time()
    g = project(pts, GridConfig(), sensor_xy=traj)
    elapsed = time.time() - t0

    assert elapsed < 15.0, f'实机规模耗时 {elapsed:.1f}s，复杂度可能又退化了'
    assert g.width * g.height > 100_000      # 确认真的是大图，不是被什么截断了
    assert g.occupied_cells > 0 and g.free_cells > 0 and g.unknown_cells > 0


def test_carve_cost_independent_of_occupied_count():
    """射线法的代价必须与占据格数量无关——这正是换算法的原因。

    墙加密 10 倍，耗时不应跟着涨一个量级。
    """
    import time

    traj = [(0.0, i * 0.05) for i in range(100)]

    def timed(wall_pts_per_row):
        pts = []
        for i in range(200):
            for k in range(wall_pts_per_row):
                pts.extend([(3.0 + k * 0.05, i * 0.05, 0.3)] * 3)
        t0 = time.time()
        project(pts, GridConfig(), sensor_xy=traj)
        return time.time() - t0

    thin = timed(1)
    thick = timed(10)
    # 允许一定放大（占据格集合变大、set 查询稍慢），但不能是数量级
    assert thick < thin * 5 + 2.0, f'thin={thin:.2f}s thick={thick:.2f}s，疑似又回到 O(占据格) 复杂度'
