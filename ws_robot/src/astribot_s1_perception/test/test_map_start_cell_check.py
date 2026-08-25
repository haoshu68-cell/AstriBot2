#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""出生点栅格校验的离线测试。

用合成小地图，因为它的每一格都能手写出来，断言才能写死。
"""

import pytest

from astribot_s1_perception.map_start_cell_check import check_start_cell


# 5x5 地图，分辨率 1m，原点 (0, 0)。
# 行 0 是 y=0（OccupancyGrid 的 data 是行优先、从 origin 那一行开始）。
#
#   y=4 |  0   0   0   0   0
#   y=3 |  0   0   0   0   0
#   y=2 |  0   0   0  100  0     ← (3,2) 占据
#   y=1 |  0   0   0   0   0
#   y=0 |  0  -1   0   0   0     ← (1,0) 未知
#        ----------------------
#         x=0  1   2   3   4
FREE = 0
OCC = 100
UNK = -1

GRID = [
    FREE, UNK,  FREE, FREE, FREE,   # y=0
    FREE, FREE, FREE, FREE, FREE,   # y=1
    FREE, FREE, FREE, OCC,  FREE,   # y=2
    FREE, FREE, FREE, FREE, FREE,   # y=3
    FREE, FREE, FREE, FREE, FREE,   # y=4
]
W = H = 5
RES = 1.0
OX = OY = 0.0


def check(x, y, clearance=0.0):
    return check_start_cell(W, H, RES, OX, OY, GRID, x, y, clearance)


def test_free_cell_passes():
    verdict = check(0.5, 1.5)          # (0, 1) 自由
    assert verdict.ok, str(verdict)
    assert verdict.grid_xy == (0, 1)


def test_occupied_cell_rejected_and_says_so():
    verdict = check(3.5, 2.5)          # (3, 2) 占据
    assert not verdict.ok
    assert '占据' in verdict.reason
    assert verdict.cell_value == OCC   # 必须把实测栅格值报出来，便于排查


def test_unknown_cell_rejected():
    """未知**必须**判不通过。

    这一条是本文件最关键的用例：未知区域下面可能是墙，代价地图按未知处理，
    规划器同样拒绝从那里出发。如果把这一支删掉（只否决占据），
    这个测试必须失败 —— 否则那条校验就是空跑的。
    """
    verdict = check(1.5, 0.5)          # (1, 0) 未知
    assert not verdict.ok
    assert '未知' in verdict.reason
    assert verdict.cell_value == UNK


def test_outside_map_rejected():
    assert not check(99.0, 99.0).ok
    assert not check(-5.0, 1.0).ok
    assert '范围之外' in check(99.0, 99.0).reason


def test_clearance_rejects_neighbour_occupied():
    """出生点自己是自由的，但净空半径内有占据 → 不通过。"""
    # (2, 2) 自由，右边 (3, 2) 是占据，距离 1m
    assert check(2.5, 2.5, clearance=0.0).ok          # 只查本格 -> 通过
    verdict = check(2.5, 2.5, clearance=1.5)          # 半径覆盖到 (3,2) -> 不通过
    assert not verdict.ok
    assert '净空不足' in verdict.reason


def test_clearance_allows_unknown_neighbour():
    """周围有未知格不否决 —— 地图边缘本来就有未知，否则几乎无处可站。"""
    # (0, 1) 自由，(1, 0) 是未知，距离 sqrt(2)
    verdict = check(0.5, 1.5, clearance=2.0)
    assert verdict.ok, str(verdict)


def test_map_edge_clearance_does_not_crash():
    """净空窗口越出地图边界时按跳过处理，不能索引越界。"""
    verdict = check(0.5, 0.5, clearance=3.0)
    assert verdict is not None


# ---------------------------------------------------------------------------
# 退化输入：给明确原因而不是异常/越界
# ---------------------------------------------------------------------------
@pytest.mark.parametrize('res', [0.0, -1.0])
def test_bad_resolution_reported(res):
    verdict = check_start_cell(W, H, res, OX, OY, GRID, 1.0, 1.0, 0.0)
    assert not verdict.ok
    assert '分辨率' in verdict.reason


def test_size_mismatch_reported():
    verdict = check_start_cell(W, H, RES, OX, OY, GRID[:-1], 1.0, 1.0, 0.0)
    assert not verdict.ok
    assert '长度' in verdict.reason


def test_zero_size_reported():
    verdict = check_start_cell(0, 0, RES, OX, OY, [], 1.0, 1.0, 0.0)
    assert not verdict.ok
    assert '尺寸' in verdict.reason


def test_negative_origin_offsets_grid_correctly():
    """原点为负时栅格换算不能算错 —— 真机地图的原点几乎总是负的。"""
    # 原点 (-2, -2)，世界 (0.5, 0.5) 应落在栅格 (2, 2)
    verdict = check_start_cell(W, H, RES, -2.0, -2.0, GRID, 0.5, 0.5, 0.0)
    assert verdict.grid_xy == (2, 2)
    assert verdict.ok
