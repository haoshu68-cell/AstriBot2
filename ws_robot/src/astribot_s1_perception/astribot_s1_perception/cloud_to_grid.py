#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""点云地图 → 占据栅格：纯逻辑，不含 rclpy，可离线单测。

============================ 为什么需要这一层 ============================
Voxel-SLAM（`/home/astribot/SLAM/vxlm-slam`）**只输出点云地图**，实测它的 7 个
地图话题全部是 `sensor_msgs/PointCloud2`：

    /map_cmap  /map_pmap  /map_scan  /map_init  /map_test  /map_path  /map_true

`grep create_publisher voxelslam.cpp` 零 `OccupancyGrid`、零 `nav_msgs`。
而 nav2 的 `global_costmap/static_layer`、探索协调器、前沿探索器**全部只吃
`nav_msgs/OccupancyGrid`**。所以必须有这一层投影。

============================ 投影口径（不是随便选的）============================
本模块的高度切片口径与既有的 `/scan` 链**刻意一致**，否则同一个障碍在
代价地图和 SLAM 地图里会出现在不同位置：

    pointcloud_to_laserscan_params.yaml: min_height 0.05 / max_height 0.6

地面在 `astribot_torso_base` 下方约 z ≈ -0.095（**不是** -0.129；odom/Gazebo 的
静态高度 0.1292 不是地面偏移）。所以默认切片带取 [-0.05, +0.60]：
下界略高于地面避免把地面点算成障碍，上界与 `/scan` 链一致。

============================ 三态语义 ============================
`OccupancyGrid.data` 用标准 ROS 编码，与 `exploration_coordinator_params.yaml`
的 `occupied_threshold: 65` / `free_threshold: 25` 对齐：

    -1  未知（从未被观测）
     0  自由
   100  占据

**未知 vs 自由的区分是探索的全部意义所在** —— 静态地图里没有未知栅格就没有前沿，
探索协调器无事可做。所以本模块**不能**把没有点的格子一律标成自由：
只有被"射线扫过但没打到东西"的格子才是自由。见 `carve_free_space`。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


# 与既有 /scan 链一致的默认切片带（见模块头）
DEFAULT_Z_MIN = -0.05
DEFAULT_Z_MAX = 0.60

# 标准 ROS 占据栅格三态
CELL_UNKNOWN = -1
CELL_FREE = 0
CELL_OCCUPIED = 100


class GridConfigError(ValueError):
    """配置不合法。刻意用异常而不是返回默认值——静默降级会让错误配置活很久。"""


@dataclass
class GridConfig:
    """投影参数。

    `resolution` 必须与 `nav2_params_rpp.yaml` 的 global_costmap `resolution: 0.05`
    一致，否则代价地图会做一次重采样、引入半格误差。
    """

    resolution: float = 0.05
    z_min: float = DEFAULT_Z_MIN
    z_max: float = DEFAULT_Z_MAX
    # 单个格子里累计到多少点才算占据。1 = 一个点就算，噪声敏感；
    # 实测 Voxel-SLAM 的 keyframe 点云每帧约 4000 点，建议 ≥ 2。
    min_points_per_cell: int = 2
    # 地图外扩边距（m）。留一圈未知，避免机器人贴着地图边界时
    # 代价地图取不到数据。
    padding_m: float = 1.0
    # 单张图的格子数上限，防止一个离群点把地图撑到几个 GB。
    max_cells: int = 4_000_000
    # --- 射线雕刻参数（见 carve_free_space 的复杂度说明）---
    # 单条射线最长距离（m）。取雷达有效观测的保守值，太大会把未观测区误标成自由。
    carve_max_range_m: float = 8.0
    # 每个位姿发射的射线条数。360 条 = 1°/条。
    carve_n_rays: int = 360
    # 位姿抽稀步长。实测相邻关键帧只差 4.7 cm，逐帧发射是纯浪费。
    carve_pose_stride: int = 10

    def validate(self) -> None:
        if not (self.resolution > 0.0):
            raise GridConfigError('resolution 必须 > 0，收到 %r' % (self.resolution,))
        if not (self.z_max > self.z_min):
            raise GridConfigError(
                'z_max(%r) 必须大于 z_min(%r)' % (self.z_max, self.z_min))
        if self.min_points_per_cell < 1:
            raise GridConfigError(
                'min_points_per_cell 必须 >= 1，收到 %r' % (self.min_points_per_cell,))
        if self.padding_m < 0.0:
            raise GridConfigError('padding_m 不能为负，收到 %r' % (self.padding_m,))
        if self.max_cells < 1:
            raise GridConfigError('max_cells 必须 >= 1，收到 %r' % (self.max_cells,))
        if not (self.carve_max_range_m > 0.0):
            raise GridConfigError(
                'carve_max_range_m 必须 > 0，收到 %r' % (self.carve_max_range_m,))
        if self.carve_n_rays < 1:
            raise GridConfigError(
                'carve_n_rays 必须 >= 1，收到 %r' % (self.carve_n_rays,))
        if self.carve_pose_stride < 1:
            raise GridConfigError(
                'carve_pose_stride 必须 >= 1，收到 %r' % (self.carve_pose_stride,))


@dataclass
class GridResult:
    """投影结果。字段名与 `nav_msgs/OccupancyGrid` 对齐，方便节点层直接搬。"""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: list  # 长度必须 == width*height，值域 {-1, 0, 100}
    # 诊断用，不进消息
    points_used: int = 0
    points_out_of_slab: int = 0
    occupied_cells: int = 0
    free_cells: int = 0
    unknown_cells: int = 0
    warnings: list = field(default_factory=list)

    def index_of(self, col: int, row: int) -> int:
        """行主序索引。OccupancyGrid 是 row-major，row 0 在 origin 处。"""
        return row * self.width + col


def slab_filter(points, z_min: float, z_max: float):
    """取出落在高度带内的点。

    `points` 是 (x, y, z) 三元组的可迭代对象。
    返回 (kept, dropped_count)。**不做任何坐标变换**——调用方必须先把点变换到
    目标 frame（通常是 map），本模块不碰 TF。
    """
    kept = []
    dropped = 0
    for p in points:
        x, y, z = p[0], p[1], p[2]
        # NaN/Inf 必须显式丢掉：它们会让后面的 min/max 全变成 NaN，
        # 而症状是"地图尺寸算出来是 0 或者天文数字"，离根因很远。
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            dropped += 1
            continue
        if z_min <= z <= z_max:
            kept.append((x, y))
        else:
            dropped += 1
    return kept, dropped


def compute_bounds(xy_points, padding_m: float):
    """算出包含所有点的轴对齐边界，外扩 padding。

    返回 (min_x, min_y, max_x, max_y)；空输入返回 None（调用方决定怎么办，
    本模块不替它编一个假的空地图）。
    """
    if not xy_points:
        return None
    min_x = min(p[0] for p in xy_points)
    max_x = max(p[0] for p in xy_points)
    min_y = min(p[1] for p in xy_points)
    max_y = max(p[1] for p in xy_points)
    return (min_x - padding_m, min_y - padding_m,
            max_x + padding_m, max_y + padding_m)


def carve_free_space(grid: GridResult, sensor_xy, occupied_cells: set,
                     max_range_m: float = 8.0, n_rays: int = 360,
                     pose_stride: int = 1) -> int:
    """从每个传感器位置发射一圈虚拟射线，把打到障碍之前的格子标成自由。

    **为什么必须区分自由与未知**：探索的定义是"走向未知区域的边界"。
    若把所有没点的格子标成自由，整张图就没有未知栅格 → 没有前沿 →
    探索协调器永远无事可做（`map_source.yaml` 的注释已记过这件事）。
    反过来若一律标未知，走过的空地也是未知，机器人会反复回到已探索区域。

    ============================ 算法选择（实测驱动）============================
    第一版实现是"对每个位姿遍历所有占据格连线"，在合成小样本上跑得很快，
    但用实机 `sessions/1floor` 的真实规模一跑就卡死：

        3367 个位姿 x 约 2 万个占据格 x 每条射线数十格 ≈ 10^8 次遍历

    单元测试用的是 10~40 个点，**完全暴露不出这个复杂度**。
    改成"每个位姿发 N 条定角射线、遇到占据格即停"之后是

        (3367/stride) x n_rays x (max_range/resolution)

    与占据格数量**无关**，这才是占据栅格建图的标准做法。

    `max_range_m`  单条射线最长距离。应取雷达有效量程的保守值——
                   Mid-360 标称 70 m，但室内建图有效观测远小于此，
                   取太大会把未观测区域误标成自由。
    `pose_stride`  位姿抽稀。实测相邻关键帧只差 4.7 cm（10 Hz、约 0.47 m/s），
                   逐帧发射是纯浪费；stride=10 即约 0.47 m 一次，足够密。
    """
    if max_range_m <= 0.0:
        raise GridConfigError('max_range_m 必须 > 0，收到 %r' % (max_range_m,))
    if n_rays < 1:
        raise GridConfigError('n_rays 必须 >= 1，收到 %r' % (n_rays,))
    if pose_stride < 1:
        raise GridConfigError('pose_stride 必须 >= 1，收到 %r' % (pose_stride,))

    carved = 0
    max_cells = int(math.ceil(max_range_m / grid.resolution))
    # 预算三角函数，避免在内层循环里反复算
    angles = [2.0 * math.pi * k / n_rays for k in range(n_rays)]
    dirs = [(math.cos(a), math.sin(a)) for a in angles]

    for i in range(0, len(sensor_xy), pose_stride):
        sx, sy = sensor_xy[i]
        s_col, s_row = world_to_cell(grid, sx, sy)
        if s_col is None:
            continue
        # 传感器所在格本身一定是自由的（机器人就站在那儿）
        idx = grid.index_of(s_col, s_row)
        if grid.data[idx] == CELL_UNKNOWN:
            grid.data[idx] = CELL_FREE
            carved += 1

        for (dx, dy) in dirs:
            for step in range(1, max_cells + 1):
                c = s_col + int(round(dx * step))
                r = s_row + int(round(dy * step))
                if c < 0 or c >= grid.width or r < 0 or r >= grid.height:
                    break
                if (c, r) in occupied_cells:
                    break          # 打到障碍，射线终止，障碍格保持占据
                idx = grid.index_of(c, r)
                if grid.data[idx] == CELL_UNKNOWN:
                    grid.data[idx] = CELL_FREE
                    carved += 1
    return carved


def world_to_cell(grid: GridResult, x: float, y: float):
    """map 系坐标 → (col, row)。越界返回 (None, None)。

    注意 `OccupancyGrid` 的 origin 是**栅格左下角**在 map 系里的位置，
    且本模块只支持 `origin.orientation` 为单位四元数——
    探索协调器实测**只读 `origin.position.x/y`、完全忽略旋转**，
    所以带旋转的 origin 会被静默算错。节点层必须保证发出去的 origin 无旋转。
    """
    col = int(math.floor((x - grid.origin_x) / grid.resolution))
    row = int(math.floor((y - grid.origin_y) / grid.resolution))
    if col < 0 or col >= grid.width or row < 0 or row >= grid.height:
        return (None, None)
    return (col, row)


def project(points, config: GridConfig, sensor_xy=None) -> GridResult:
    """点云 → 占据栅格。这是本模块的入口。

    `points`     : (x, y, z) 可迭代，**必须已经在 map 系下**
    `sensor_xy`  : 传感器/机器人轨迹在 map 系下的 (x, y) 序列。
                   给了就做射线雕刻（区分自由与未知）；不给则整张图
                   只有"占据"和"未知"两态 —— 那样探索能跑（有前沿），
                   但机器人会认为自己站在未知区域里，nav2 的
                   `allow_unknown: false` 会让规划直接失败。
                   **所以在线建图必须传轨迹。**

    抛 `GridConfigError`（配置不合法）或 `ValueError`（点云导致尺寸超限）。
    """
    config.validate()

    xy, dropped = slab_filter(points, config.z_min, config.z_max)

    # 边界必须**同时**包含点云和轨迹。
    #
    # 第一版只用点云算边界，结果机器人自己的位置可能落在图外 ——
    # `world_to_cell` 返回 None、射线雕刻整个不生效、自由格数为 0。
    # 症状是"地图看起来没问题但 nav2 报 Starting point in lethal space"，
    # 而本仓库已经记过这个坑离根因有三层远（见 map_source.yaml 的
    # validate_start_cell 注释）。所以这里把轨迹一起算进去。
    extent = list(xy)
    if sensor_xy:
        extent.extend((float(sx), float(sy)) for sx, sy in sensor_xy)

    bounds = compute_bounds(extent, config.padding_m)
    if bounds is None:
        raise ValueError(
            '高度带 [%.3f, %.3f] 内没有任何有效点（丢弃 %d 个）。'
            '要么切片带不对，要么点云不在 map 系下——'
            '本模块不做坐标变换，调用方必须先变换。'
            % (config.z_min, config.z_max, dropped))

    min_x, min_y, max_x, max_y = bounds
    width = int(math.ceil((max_x - min_x) / config.resolution))
    height = int(math.ceil((max_y - min_y) / config.resolution))
    width = max(width, 1)
    height = max(height, 1)

    if width * height > config.max_cells:
        raise ValueError(
            '栅格 %dx%d = %d 格超过上限 %d。通常是有离群点把边界撑开了——'
            '先查点云里的 NaN/远点，不要直接抬高 max_cells。'
            % (width, height, width * height, config.max_cells))

    grid = GridResult(
        resolution=config.resolution,
        width=width, height=height,
        origin_x=min_x, origin_y=min_y,
        data=[CELL_UNKNOWN] * (width * height),
    )
    grid.points_out_of_slab = dropped

    # 逐格计票，达到阈值才算占据
    counts = {}
    for (x, y) in xy:
        col, row = world_to_cell(grid, x, y)
        if col is None:
            continue
        key = (col, row)
        counts[key] = counts.get(key, 0) + 1
        grid.points_used += 1

    occupied = set()
    for (col, row), n in counts.items():
        if n >= config.min_points_per_cell:
            grid.data[grid.index_of(col, row)] = CELL_OCCUPIED
            occupied.add((col, row))

    if sensor_xy:
        carve_free_space(grid, sensor_xy, occupied,
                         max_range_m=config.carve_max_range_m,
                         n_rays=config.carve_n_rays,
                         pose_stride=config.carve_pose_stride)
    else:
        grid.warnings.append(
            '未提供 sensor_xy：整张图只有占据与未知两态。'
            'nav2 的 allow_unknown=false 会让规划在未知格上直接失败。')

    grid.occupied_cells = sum(1 for v in grid.data if v == CELL_OCCUPIED)
    grid.free_cells = sum(1 for v in grid.data if v == CELL_FREE)
    grid.unknown_cells = sum(1 for v in grid.data if v == CELL_UNKNOWN)

    # 契约自检：data 长度必须等于 width*height，否则探索协调器直接 ERROR 丢弃
    if len(grid.data) != grid.width * grid.height:
        raise ValueError(
            'data 长度 %d != width*height %d（内部错误）'
            % (len(grid.data), grid.width * grid.height))

    return grid


def load_session_poses(path: str):
    """读 Voxel-SLAM 的 `alidarState.txt`，返回 [(x, y)] 轨迹。

    实测格式（`sessions/1floor/alidarState.txt`，3367 行）每行空格分隔：

        t  px py pz  qx qy qz qw  vx vy vz  bgx bgy bgz  bax bay baz  gx gy gz  ...

    即时间戳 + 位置(3) + 四元数(4, xyzw) + 速度(3) + 陀螺零偏(3) + 加计零偏(3)
    + 重力(3, 实测约 (-0.0005, -0.0118, -9.8015) —— 这是判断字段顺序没错位的锚点)
    + 若干协方差项。

    只取 px, py。抛 ValueError 并带上行号，不静默跳过坏行。
    """
    poses = []
    with open(path, 'r', encoding='utf-8') as fh:
        for lineno, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 8:
                raise ValueError(
                    '%s:%d 字段数 %d < 8，不是预期的 alidarState 格式'
                    % (path, lineno, len(parts)))
            try:
                poses.append((float(parts[1]), float(parts[2])))
            except ValueError as exc:
                raise ValueError('%s:%d 解析失败：%s' % (path, lineno, exc)) from exc
    if not poses:
        raise ValueError('%s 里没有任何位姿' % path)
    return poses
