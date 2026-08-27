#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨机地图中继：把真机 domain 上的 /map 单向搬到本机 domain。

!!! 当前默认配置下本节点不启动 !!!
全栈已统一到 domain 25（与厂商 env.sh 一致），`map_source.yaml` 里
remote_domain_id == local_domain_id == 25，map_provider.launch.py 在同域时
**跳过**本节点 —— 同域下真机 /map 直接可见，中继只会回环。
本文件保留下来是因为它是"要回到跨域隔离"时唯一可用的实现，下面这段隔离依据
和实测记录都仍然有效，别删。

==================== 隔离靠 domain，不靠 ROS_LOCALHOST_ONLY ====================
最省事的做法是两台机器用同一个 domain 直连。**代价要清楚**：我们本机图里有
`/cmd_vel`、`/wheel_effort_controller/commands`、
`/gripper_{left,right}_controller/follow_joint_trajectory` —— 同域直连意味着
仿真里的控制指令有通路打到真机上去。当前统一 domain 就是接受了这个代价，
换来的是桥接能与 SDK 互相看见（真机上 SDK 后端的 domain 改不动）。

要恢复网络层隔离，就用**一个进程、两个 rclpy Context、各自不同 domain、
只搬一个话题、单向**，也就是本文件：

    真机 domain(remote)                          本机 domain(local)
     /map (slam_toolbox on real robot)
          │ 订阅 context_remote
          ▼
      map_domain_relay ──发布 context_local──▶ /map

安全性的依据（实测验证过，不是推测）：
  · 假真机在 domain 25 上的私有话题 `/fake_robot_only`，在 domain 42 上
    **两种 ROS_LOCALHOST_ONLY 取值下都不可见** —— 跨域确实互不可见。
  · 中继是唯一的跨域参与者，而且只**单向**搬 /map。本机的 /cmd_vel 之类
    没有任何通路能到真机。

!!! 最初的设计错了，这里记下来免得有人改回去 !!!
原来想的是"只让中继进程 ROS_LOCALHOST_ONLY=0，其余节点保持 1，这样只有中继能上网"。
实测证明行不通：

    domain 42 订阅端 ROS_LOCALHOST_ONLY=1 -> 收不到中继发的 /map
    domain 42 订阅端 ROS_LOCALHOST_ONLY=0 -> 收得到

`localhost_only=1` 与 `=0` 的 DDS 参与者**互相发现不了**，同机同域也不行。
保持 1 的后果是中继"成功发布"到没人听的地方，而 nav2 一直等 /map —— 静默失败。
所以 real_live 模式下**整条栈**都要 ROS_LOCALHOST_ONLY=0，
map_provider.launch.py 会在启动时检查并拒绝，不让它静默地跑成那样。

rclpy 支持按 context 指定 domain（Humble 实测 `rclpy.init` 签名里就有 `domain_id`）：
    rclpy.init(context=ctx, domain_id=N)

================================ 两条硬要求 ================================
1. **QoS 两侧都必须 TRANSIENT_LOCAL + RELIABLE**。/map 是 latched 的；
   用 VOLATILE 订阅 latched 话题的后果是**一条消息都收不到**，
   而且只有一行 QoS 不兼容的 WARN —— 极易被读成"远端没发地图"。
2. **超时必须报错退出，不许静默等待**。静默等待的表现是"nav2 一直等 /map 不动"，
   排查方向会完全跑偏（会去查 nav2、查 costmap，而问题在网络/domain）。

用法（**整条栈**都要 ROS_LOCALHOST_ONLY=0，见上面那段）：
    ROS_LOCALHOST_ONLY=0 ros2 run astribot_s1_perception map_domain_relay \
        --ros-args --params-file <map_source.yaml>
"""

import sys
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid


def latched_qos(depth=1):
    """latched 地图话题的 QoS。两侧必须一致，否则零消息。"""
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def map_fingerprint(msg):
    """地图指纹。

    打这个是为了让"仿真用的到底是不是真机那张图"可核对，而不是靠相信中继。
    已知栅格占比顺带能看出地图是不是只传了一半。
    """
    info = msg.info
    total = len(msg.data)
    if total == 0:
        return f'{info.width}x{info.height} res={info.resolution:.3f} 空数据'
    free = sum(1 for v in msg.data if v == 0)
    occupied = sum(1 for v in msg.data if v >= 65)
    unknown = sum(1 for v in msg.data if v < 0)
    known_ratio = 100.0 * (total - unknown) / total
    return (f'{info.width}x{info.height} res={info.resolution:.3f} '
            f'origin=({info.origin.position.x:.3f}, {info.origin.position.y:.3f}) '
            f'自由={free} 占据={occupied} 未知={unknown} 已知占比={known_ratio:.1f}%')


class _ParamReader(Node):
    """只用来读参数的临时节点。

    参数要在**本机** context 上读（--params-file 是给本进程的），
    所以不能等远端 context 建好再读。
    """

    def __init__(self, context):
        super().__init__('map_domain_relay_params', context=context)
        self.declare_parameter('remote_domain_id', 25)
        self.declare_parameter('local_domain_id', 25)
        self.declare_parameter('remote_map_topic', '/map')
        self.declare_parameter('local_map_topic', '/map')
        self.declare_parameter('relay_timeout_sec', 30.0)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv

    # ---- 先在本机 context 上读参数 ----
    local_ctx = Context()
    rclpy.init(args=argv, context=local_ctx)
    reader = _ParamReader(local_ctx)
    remote_domain = int(reader.get_parameter('remote_domain_id').value)
    local_domain = int(reader.get_parameter('local_domain_id').value)
    remote_topic = str(reader.get_parameter('remote_map_topic').value)
    local_topic = str(reader.get_parameter('local_map_topic').value)
    timeout_sec = float(reader.get_parameter('relay_timeout_sec').value)
    logger = reader.get_logger()

    if remote_domain == local_domain:
        logger.error(
            f'remote_domain_id 与 local_domain_id 相同({remote_domain})。'
            '同域下不需要中继 —— 真机的 /map 直接可见，直接订阅即可；'
            '起中继只会把同一张地图回环发布给自己。\n'
            '  注意：map_provider.launch.py 在同域时会**跳过**本节点，'
            '所以看到这条错误说明是直接 ros2 run 起的，或者 params 传错了。')
        reader.destroy_node()
        rclpy.shutdown(context=local_ctx)
        return 1

    # 本机 context 已经用默认 domain 初始化过了，要按参数指定 domain
    # 就得重建。先关掉再按 local_domain 重开。
    reader.destroy_node()
    rclpy.shutdown(context=local_ctx)

    local_ctx = Context()
    rclpy.init(args=argv, context=local_ctx, domain_id=local_domain)
    remote_ctx = Context()
    rclpy.init(args=argv, context=remote_ctx, domain_id=remote_domain)

    local_node = Node('map_domain_relay_local', context=local_ctx)
    remote_node = Node('map_domain_relay_remote', context=remote_ctx)
    logger = local_node.get_logger()

    publisher = local_node.create_publisher(
        OccupancyGrid, local_topic, latched_qos())

    state = {'count': 0, 'last_fingerprint': None}

    def on_remote_map(msg):
        state['count'] += 1
        fingerprint = map_fingerprint(msg)
        publisher.publish(msg)
        if fingerprint != state['last_fingerprint']:
            logger.info(
                f'中继第 {state["count"]} 张地图 '
                f'(domain {remote_domain}{remote_topic} -> '
                f'domain {local_domain}{local_topic}): {fingerprint}')
            state['last_fingerprint'] = fingerprint

    remote_node.create_subscription(
        OccupancyGrid, remote_topic, on_remote_map, latched_qos())

    logger.info(
        f'地图中继启动：远端 domain={remote_domain} 话题={remote_topic}，'
        f'本机 domain={local_domain} 话题={local_topic}，'
        f'等待远端地图，超时 {timeout_sec:.0f}s')

    local_exec = SingleThreadedExecutor(context=local_ctx)
    local_exec.add_node(local_node)
    remote_exec = SingleThreadedExecutor(context=remote_ctx)
    remote_exec.add_node(remote_node)

    stop = threading.Event()

    def spin_local():
        while not stop.is_set() and local_ctx.ok():
            local_exec.spin_once(timeout_sec=0.1)

    local_thread = threading.Thread(target=spin_local, daemon=True)
    local_thread.start()

    exit_code = 0
    try:
        # ---- 等第一张地图。超时必须响亮失败 ----
        started = time.time()
        while remote_ctx.ok() and state['count'] == 0:
            remote_exec.spin_once(timeout_sec=0.2)
            if time.time() - started > timeout_sec:
                logger.error(
                    f'{timeout_sec:.0f}s 内没在 domain {remote_domain} 上收到 '
                    f'{remote_topic}。逐条查：'
                    '(1) 真机侧的 slam_toolbox 起了吗；'
                    '(2) 本进程的 ROS_LOCALHOST_ONLY 是 0 吗（=1 时跨机永远发现不了）；'
                    '(3) 两台机器网络互通吗；'
                    '(4) 远端发布者的 QoS 是 TRANSIENT_LOCAL 吗'
                    '（VOLATILE 的话这里一条都收不到，且只有一行 WARN）。')
                exit_code = 1
                break

        # ---- 拿到第一张之后转入长期中继 ----
        while exit_code == 0 and remote_ctx.ok():
            remote_exec.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        local_thread.join(timeout=2.0)
        remote_node.destroy_node()
        local_node.destroy_node()
        if remote_ctx.ok():
            rclpy.shutdown(context=remote_ctx)
        if local_ctx.ok():
            rclpy.shutdown(context=local_ctx)

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
