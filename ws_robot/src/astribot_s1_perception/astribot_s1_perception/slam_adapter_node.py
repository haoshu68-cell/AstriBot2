#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""外部 SLAM（`/home/astribot/SLAM`，Voxel-SLAM）→ 本仓库下游契约的接入节点。

职责三件，与 `slam_contract.py` 一一对应：
  1) **话题/frame 改名**：外部名 → 固定的 `/map` + `map`/`odom`/`astribot_torso_base`
  2) **QoS 归一化**：不论上游怎么发，下游一律 TRANSIENT_LOCAL + RELIABLE
  3) **心跳重发**：缓存最后一帧按周期重发，满足协调器"按本地到达时间判超时"

判定逻辑全在 `slam_contract.py`（纯逻辑、离线可测），本文件只做 ROS 粘合。

════════════════ 为什么这一层必须响亮失败 ════════════════
本仓库反复踩过**无报错的失败**：params_file 泄漏让 yaml 静默不生效、节点名 remap
让两份 yaml 全失效、QoS 不兼容导致零消息且两端都不报错。适配层是新引入的一层，
若它也静默，故障源就又多一个。所以：
  · 契约违规 → `strict_contract=true` 时 **非零退出**，不转发坏数据
  · `source_timeout_sec` 内收不到源地图 → **非零退出**，不静默等待
（后者照 `map_domain_relay.py` 的既有纪律。）

⚠️ **本节点不发布 `odom→astribot_torso_base`**。那条边由
`astribot_trajectory_bridge/chassis_odom_node.py` 负责。分开是因为 Voxel-SLAM 是
LIO 型、带 Loop closure + GBA，它的位姿是 **map 级**、会被全局优化修正而跳变；
odom 要求局部连续不跳变，两者不是同一个量，不能拿一个冒充另一个。

⚠️ **本节点不发布任何 TF。** `map→odom` 由 `map_odom_tf_node` 负责，
它做的是 REP-105 分解 `map→odom = (SLAM 的 map→base) ∘ (odom→base)⁻¹`。
本节点曾经在这里发一个**单位变换**占位，那等于宣称"odom 原点就是 map 原点" ——
只在开机即建图那一种情形下成立，不成立时地图与激光整体错位且零报错。已删除。
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid

from astribot_s1_perception.slam_contract import ContractViolation, MapContract, SlamAdapter


def latched_qos(depth=1):
    """latched 地图话题的 QoS。两侧必须一致，否则零消息。"""
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def volatile_qos(depth=5):
    """上游是普通 publisher 时的兜底 QoS。

    需要它是因为 TRANSIENT_LOCAL **订阅**端与 VOLATILE **发布**端不兼容：
    订阅方要求"历史可补发"而发布方给不了，DDS 直接不建连接 —— 零消息、零报错。
    """
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)


AUTO_QOS_FALLBACK_SEC = 8.0


class SlamAdapterNode(Node):
    """ROS 粘合层。所有判定都委托给 `slam_contract`，本类只管收发与记账。"""

    def __init__(self):
        super().__init__('slam_adapter')

        self.declare_parameter('source_map_topic', '/slam/map')
        self.declare_parameter('source_map_qos', 'auto')      # auto|transient_local|volatile
        self.declare_parameter('source_map_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('republish_period_sec', 5.0)
        self.declare_parameter('strict_contract', True)
        self.declare_parameter('source_timeout_sec', 30.0)
        self.declare_parameter('resolution_expected', 0.05)
        self.declare_parameter('strict_cell_values', True)
        self.declare_parameter('report_period_sec', 10.0)

        self.source_map_topic = self._str('source_map_topic')
        self.source_map_qos = self._str('source_map_qos').lower()
        self.source_map_frame = self._str('source_map_frame')
        self.map_topic = self._str('map_topic')
        self.map_frame = self._str('map_frame')
        self.odom_frame = self._str('odom_frame')
        self.strict = bool(self.get_parameter('strict_contract').value)
        self.source_timeout_sec = float(self.get_parameter('source_timeout_sec').value)

        if self.source_map_qos not in ('auto', 'transient_local', 'volatile'):
            raise ContractViolation(
                f'source_map_qos={self.source_map_qos!r} 非法，'
                f'只能是 auto|transient_local|volatile')

        self.contract = MapContract(
            resolution_expected=float(self.get_parameter('resolution_expected').value),
            republish_period_sec=float(self.get_parameter('republish_period_sec').value),
            strict_cell_values=bool(self.get_parameter('strict_cell_values').value))
        self.adapter = SlamAdapter(
            contract=self.contract,
            source_map_frame=self.source_map_frame,
            strict=self.strict)

        self.exit_code = 0

        self._setup_io()
        self._log_startup()

    def _str(self, name):
        return str(self.get_parameter(name).value)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _setup_io(self):
        self.map_pub = self.create_publisher(
            OccupancyGrid, self.map_topic, latched_qos())

        self._subs = []
        first_qos = (volatile_qos() if self.source_map_qos == 'volatile'
                     else latched_qos())
        self._subs.append(self.create_subscription(
            OccupancyGrid, self.source_map_topic, self._on_source_map, first_qos))

        self._start_sec = self._now_sec()
        self._fallback_added = False
        self.create_timer(1.0, self._tick_watchdog)
        self.create_timer(0.5, self._tick_heartbeat)
        report_period = float(self.get_parameter('report_period_sec').value)
        if report_period > 0.0:
            self.create_timer(report_period, self._tick_report)

    def _log_startup(self):
        self.get_logger().info(
            f'slam_adapter 启动：\n'
            f'  源  {self.source_map_topic} (qos={self.source_map_qos}) '
            f'frame={self.source_map_frame}\n'
            f'  出  {self.map_topic} (transient_local+reliable) frame={self.map_frame}\n'
            f'  心跳 {self.contract.republish_period_sec}s\n'
            f'  strict_contract={self.strict}  超时 {self.source_timeout_sec}s')
        self.get_logger().info(
            '本节点**不发任何 TF**。map→odom 由 map_odom_tf_node 负责'
            '（它做 REP-105 分解，需要 SLAM 的 map→base 与 chassis_odom_node '
            '的 odom→base 两者都在）。若 TF 树断在 map→odom，查那个节点，'
            '不要在这里找。')

    def _on_source_map(self, msg):
        forwarded = self.adapter.on_source_map(msg, self._now_sec())
        if forwarded is None:
            self._handle_rejection()
            return
        self._emit(forwarded)

    def _handle_rejection(self):
        reasons = self.adapter.stats.last_reject_reasons
        detail = '\n'.join(f'  · {r}' for r in reasons)
        if self.strict:
            self.get_logger().error(
                f'源地图违反契约，拒绝转发并退出（strict_contract=true）：\n{detail}\n'
                f'  要先看数据再改代码就把 strict_contract 置 false，'
                f'那样只 WARN 并继续等下一帧 —— 但**不会**转发坏数据。')
            self.exit_code = 1
            raise SystemExit(1)
        self.get_logger().warning(
            f'源地图违反契约，本帧不转发（strict_contract=false）：\n{detail}')

    def _emit(self, grid):
        """改 frame 名后发出。刻意**不改** header.stamp。

        为什么重发时也保留原 stamp：协调器按**本地到达时间**判超时，不看 stamp；
        而把 stamp 改成"现在"会让 nav2 以为这是新观测，掩盖上游已经停更的事实。
        原 stamp 老得离谱正是我们想让人看见的信号。
        """
        grid.header.frame_id = self.map_frame
        self.map_pub.publish(grid)


    def _tick_heartbeat(self):
        grid = self.adapter.republish(self._now_sec())
        if grid is not None:
            self._emit(grid)

    def _tick_watchdog(self):
        """两件事：auto QoS 退化，以及超时非零退出。"""
        elapsed = self._now_sec() - self._start_sec
        if self.adapter.has_map:
            return

        if (self.source_map_qos == 'auto' and not self._fallback_added
                and elapsed >= AUTO_QOS_FALLBACK_SEC):
            self._fallback_added = True
            self._subs.append(self.create_subscription(
                OccupancyGrid, self.source_map_topic,
                self._on_source_map, volatile_qos()))
            self.get_logger().warning(
                f'{AUTO_QOS_FALLBACK_SEC}s 内没收到 {self.source_map_topic}，'
                f'补一个 VOLATILE 订阅（source_map_qos=auto 的退化路径）。\n'
                f'  这通常说明上游是普通 publisher：TRANSIENT_LOCAL 订阅端与 '
                f'VOLATILE 发布端**不兼容**，DDS 不建连接，零消息且零报错。\n'
                f'  确认后请把 source_map_qos 显式设为 volatile，别依赖退化。')

        if elapsed >= self.source_timeout_sec:
            self.get_logger().error(
                f'{self.source_timeout_sec}s 内没收到任何 {self.source_map_topic}，退出。\n'
                f'  不静默等待是刻意的：静默等待会让下游一直等 /map，'
                f'症状是"nav2 卡在启动"，而真正的原因在这一层。\n'
                f'  依次查：① 外部 SLAM 进程是否在跑；'
                f'② 话题名是否真是 {self.source_map_topic}'
                f'（`ros2 topic list | grep -i map`）；'
                f'③ ROS_DOMAIN_ID 是否与外部 SLAM 一致（实机是 25，不是 42）；'
                f'④ 消息类型是否为 nav_msgs/OccupancyGrid —— '
                f'Voxel-SLAM 是 LIO 型，只出点云**不出** OccupancyGrid，'
                f'那种情况要先接 cloud_to_grid 投影，而不是接这个节点。')
            self.exit_code = 1
            raise SystemExit(1)

    def _tick_report(self):
        stats = self.adapter.stats
        self.get_logger().info(
            f'收={stats.received} 转发={stats.forwarded} '
            f'拒绝={stats.rejected} 重发={stats.republished}')


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = SlamAdapterNode()
        rclpy.spin(node)
    except ContractViolation as exc:
        print(f'[slam_adapter] 配置被拒绝：{exc}', file=sys.stderr)
        code = 2
    except SystemExit as exc:
        code = int(exc.code or 0)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            code = code or node.exit_code
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
