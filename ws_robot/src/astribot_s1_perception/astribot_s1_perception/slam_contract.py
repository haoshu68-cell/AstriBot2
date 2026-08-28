#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""外部 SLAM → 本仓库下游契约的**纯逻辑**层（不 import rclpy / 不 import 消息类型）。

为什么与 `slam_adapter_node.py` 分开：契约校验是这一层最容易出错、又最难在线复现的
部分（要真的跑一个外部 SLAM 才能触发一次坏数据），拆出来就能离线穷举。
这与本包 `cloud_to_grid.py` 的做法一致。

**为什么需要"契约"而不是直接转发**：这一层是新引入的中间层，而本仓库反复踩过
"配置静默不生效 / QoS 不兼容零消息 / 参数被覆盖" 这类**无报错的失败**
（见 docs 里 params_file 泄漏、节点名 remap、QoS 两端都在 SDK 内部三例）。
适配层若也静默，故障源就又多一个。所以：**不合格就响亮失败，绝不转发坏数据**。

鸭子类型说明：`validate_grid` 只要求传进来的对象长得像 `nav_msgs/OccupancyGrid`
（`.info.resolution` / `.info.width` / `.info.height` / `.info.origin.orientation`
/ `.data`）。测试里用普通 namedtuple 即可，不需要 ROS 环境。
"""

from dataclasses import dataclass


#: 下游硬依赖的取值集合。-1 未知 / 0 空闲 / 100 占据。
#: 与 `cloud_to_grid.py` 的 CELL_* 常量同一套语义，此处刻意不 import 它，
#: 避免纯逻辑模块之间产生方向不明的依赖。
CANONICAL_CELL_VALUES = (-1, 0, 100)

#: 协调器（autonomous_patrol_node）的 map_timeout_sec。心跳周期必须 **严格小于** 它。
COORDINATOR_MAP_TIMEOUT_SEC = 10.0


class ContractViolation(RuntimeError):
    """契约不满足。由节点层决定是非零退出还是降级，本模块只负责判定。"""


@dataclass(frozen=True)
class MapContract:
    """/map 必须满足的条件。每一条都对应一个已知的下游故障模式。"""

    resolution_expected: float = 0.05
    resolution_tol: float = 1e-6
    republish_period_sec: float = 5.0
    max_stamp_future_sec: float = 1.0
    #: True = data 里只允许出现 CANONICAL_CELL_VALUES；
    #: False = 放宽到 [-1, 100] 区间（某些 SLAM 用中间值表达占据概率）
    strict_cell_values: bool = True

    def __post_init__(self):
        """构造期就把"心跳比超时还慢"挡住 —— 这个错误在线只表现为地图偶发超时。"""
        if self.republish_period_sec >= COORDINATOR_MAP_TIMEOUT_SEC:
            raise ContractViolation(
                f'republish_period_sec={self.republish_period_sec} 必须 < 协调器的 '
                f'map_timeout_sec={COORDINATOR_MAP_TIMEOUT_SEC}。'
                f'否则协调器会在两次心跳之间判超时并停止派发前沿，'
                f'症状是"探索偶发停住"，而 /map 一直有数据，极难归因。')
        if self.resolution_expected <= 0.0:
            raise ContractViolation(
                f'resolution_expected={self.resolution_expected} 必须为正')

    def validate_grid(self, grid, now_sec=None):
        """返回违规项列表；空列表 = 合格。**不抛异常**，让调用方决定失败方式。

        now_sec 为 None 时跳过时间戳检查（离线测试与"上游不填 stamp"两种场景）。
        """
        problems = []
        problems.extend(self._check_shape(grid))
        problems.extend(self._check_resolution(grid))
        problems.extend(self._check_origin(grid))
        problems.extend(self._check_values(grid))
        if now_sec is not None:
            problems.extend(self._check_stamp(grid, now_sec))
        return problems

    # -- 逐项检查 ---------------------------------------------------------
    # 拆成小函数不是为了复用，是为了让每条违规的**措辞**能单独测：
    # 本仓库的经验是错误信息说不出后果，排查就会停在"它报错了"这一步。

    @staticmethod
    def _check_shape(grid):
        info = grid.info
        width, height = int(info.width), int(info.height)
        if width <= 0 or height <= 0:
            return [f'width/height 为 0（{width}x{height}）：'
                    f'空地图会让 nav2 的 costmap 无法初始化，'
                    f'且 map_start_cell_check 无从判定出生点']
        expected = width * height
        actual = len(grid.data)
        if actual != expected:
            return [f'data 长度 {actual} != width*height {expected}（{width}x{height}）：'
                    f'下游按行主序索引，长度不符会读到越界或错位的格子，'
                    f'表现为代价地图整体斜移']
        return []

    def _check_resolution(self, grid):
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return [f'resolution={resolution} 非正']
        if abs(resolution - self.resolution_expected) > self.resolution_tol:
            return [f'resolution={resolution} 与期望 {self.resolution_expected} 不符：'
                    f'本仓库多处距离阈值按 0.05 定过口径'
                    f'（前沿的 goal_unknown_clearance_radius 必须为 0 就是其一），'
                    f'分辨率一变那些阈值全部失去意义']
        return []

    @staticmethod
    def _check_origin(grid):
        """origin 的朝向必须是单位四元数。

        为什么单列一项：带旋转的 origin 在 RViz 里看着"地图歪了一点"，
        而 nav2 的 costmap 与本仓库所有 world↔cell 换算都假定轴对齐
        （`cloud_to_grid.world_to_cell` 就是纯平移）。歪了不会报错，
        只是路径系统性偏移，很容易被误判成定位漂移。
        """
        quaternion = grid.info.origin.orientation
        w, x, y, z = (float(quaternion.w), float(quaternion.x),
                      float(quaternion.y), float(quaternion.z))
        if abs(w - 1.0) > 1e-6 or max(abs(x), abs(y), abs(z)) > 1e-6:
            return [f'origin.orientation 不是单位四元数 '
                    f'(w={w} x={x} y={y} z={z})：'
                    f'下游所有 world↔cell 换算都假定轴对齐，'
                    f'带旋转不会报错，只会让路径系统性偏移']
        return []

    def _check_values(self, grid):
        if self.strict_cell_values:
            allowed = set(CANONICAL_CELL_VALUES)
            bad = sorted({int(v) for v in grid.data} - allowed)
            if bad:
                return [f'data 出现非法取值 {bad[:8]}'
                        f'{"..." if len(bad) > 8 else ""}：'
                        f'只允许 {CANONICAL_CELL_VALUES}。'
                        f'若上游用中间值表达占据概率，'
                        f'把 strict_cell_values 置 false 并确认 nav2 的 '
                        f'lethal_cost_threshold 与之相符']
            return []
        out_of_range = sorted({int(v) for v in grid.data if not -1 <= int(v) <= 100})
        if out_of_range:
            return [f'data 出现越界取值 {out_of_range[:8]}'
                    f'{"..." if len(out_of_range) > 8 else ""}：'
                    f'OccupancyGrid 的取值域是 [-1, 100]']
        return []

    def _check_stamp(self, grid, now_sec):
        """时间戳不得**超前**本地时钟。

        只查超前不查滞后是刻意的：滞后是正常的（SLAM 有处理延迟，而且我们自己
        每 republish_period_sec 就重发一次旧帧）；超前则说明两边时钟没对齐 ——
        实机走 PTP 且开机时钟是 1970，这个坑是真实存在的。超前的 stamp 会让
        tf2 的 lookupTransform 永远等不到"未来"的变换而超时。
        """
        stamp_sec = float(grid.header.stamp.sec) + float(grid.header.stamp.nanosec) * 1e-9
        ahead = stamp_sec - now_sec
        if ahead > self.max_stamp_future_sec:
            return [f'header.stamp 超前本地时钟 {ahead:.3f}s '
                    f'（上限 {self.max_stamp_future_sec}s）：'
                    f'两边时钟没对齐。实机走 PTP、开机时钟是 1970，'
                    f'先确认时间同步已收敛。超前的 stamp 会让 tf2 '
                    f'永远等不到"未来"的变换而超时']
        return []


# ---------------------------------------------------------------------------
# 心跳 / 转发状态机
#
# 为什么要心跳：协调器按**本地到达时间**判 /map 超时（不是按 header.stamp）。
# 外部 SLAM 在场景静止时可能长时间不发新地图 —— 这是它的正常行为，
# 但在协调器看来就是"地图断了"。所以适配层缓存最后一帧按周期重发。
# ---------------------------------------------------------------------------
@dataclass
class AdapterStats:
    """可观测量。故障时先看这几个数，能立刻分清"没收到"与"收到但被拒"。"""

    received: int = 0
    forwarded: int = 0
    rejected: int = 0
    republished: int = 0
    last_reject_reasons: tuple = ()


class SlamAdapter:
    """外部 SLAM → 我们的契约。纯逻辑：不发消息，只回答"现在该发什么"。

    三件事，各自独立可测：
      1) frame 改名：外部 frame 名 → 我们固定的 map / odom / astribot_torso_base
      2) 契约校验：不合格**不转发**（strict 下由节点层非零退出）
      3) 心跳重发：缓存最后一帧，按 republish_period_sec 重发

    时间一律由调用方以秒传入（`now_sec`），本类不读时钟 —— 这样心跳时序能用
    确定性的数列测出来，不需要 sleep。本仓库有过"用墙钟时间戳导致测试结论作废"
    的先例，所以这里刻意不给它读时钟的能力。
    """

    def __init__(self, contract=None, source_map_frame='map', strict=True):
        self.contract = contract or MapContract()
        self.source_map_frame = source_map_frame
        self.strict = strict
        self.stats = AdapterStats()
        self._last_good = None
        self._last_emit_sec = None

    # -- 入口 -------------------------------------------------------------
    def on_source_map(self, grid, now_sec):
        """收到一帧源地图。返回要转发的对象，或 None（被拒，不转发）。

        被拒时 stats.last_reject_reasons 里带着原因；节点层负责按 strict
        决定是 ERROR + 非零退出还是 WARN 后继续等下一帧。
        """
        self.stats.received += 1
        problems = self.contract.validate_grid(grid, now_sec)
        if problems:
            self.stats.rejected += 1
            self.stats.last_reject_reasons = tuple(problems)
            return None

        self.stats.last_reject_reasons = ()
        self._last_good = grid
        self._last_emit_sec = now_sec
        self.stats.forwarded += 1
        return grid

    def due_for_republish(self, now_sec):
        """到点了吗。没有可重发的帧时返回 False（不拿空帧填坑）。"""
        if self._last_good is None or self._last_emit_sec is None:
            return False
        return (now_sec - self._last_emit_sec) >= self.contract.republish_period_sec

    def republish(self, now_sec):
        """取出要重发的帧并记账。未到点/无缓存时返回 None。"""
        if not self.due_for_republish(now_sec):
            return None
        self._last_emit_sec = now_sec
        self.stats.republished += 1
        return self._last_good

    @property
    def has_map(self):
        return self._last_good is not None
