#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""桥接容器进程（方案 S-1：单进程、单 SDK 会话、多 ROS 节点）。

!!! 为什么必须是单进程单会话 !!!
=============================
``Astribot()`` 构造会建 ROS 节点、等接口 alive，且 SDK 一被 import 就做进程级
fd 重定向。三个桥接各建一个实例 =
  * 三个 SDK 会话（行为未定义）；
  * 三次 fd 重定向；
  * 底盘的积分种子与机械臂的下发**看不见对方的 desired position** ——
    而它们操作的是同一台机器人。

代价是 GIL 下 250Hz 内环与 Action 回调同进程争线程。用
``MultiThreadedExecutor`` + 各自的 callback group 缓解，并让内环监控实际周期、
超阈上报 LOOP_OVERRUN —— 把抖动变成可观测量，而不是靠感觉。

!!! 环境变量必须在 import SDK 之前设置 !!!
本文件顶层**不 import** 任何 SDK 相关模块；``sdk_session.open_session()`` 内部
才做真实 import，且 import 之前先跑环境自检。
"""

import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor

from astribot_trajectory_bridge.arm_traj_bridge_node import ArmTrajBridgeNode
from astribot_trajectory_bridge.chassis_cmd_bridge_node import ChassisCmdBridgeNode
from astribot_trajectory_bridge.sdk_session import SdkSessionError, open_session


class _BootLogger:
    """会话建立之前还没有 Node，用它把自检结论打到 stderr。

    走 stderr 而不是 stdout：SDK 的 fd 重定向同时吞掉 1 和 2，但在 import 之前
    两者都还正常，而 stderr 不会被 ROS 的 stdout 缓冲吃掉顺序。
    """

    @staticmethod
    def info(msg):
        sys.stderr.write('[bridge_container] %s\n' % msg)

    @staticmethod
    def error(msg):
        sys.stderr.write('[bridge_container][ERROR] %s\n' % msg)

    warn = info


def main(args=None):
    # !!! rclpy.init 必须在 open_session **之前** !!!
    #
    # 实测（2026-08-26）：
    #   · 先 rclpy.init() 再 open_session  -> 正常，SDK 不会因重复 init 报错
    #   · 先 open_session 再 rclpy.init()  -> 抛
    #     `RuntimeError: Context.init() must only be called once`
    # 厂商 SDK 构造时自己会初始化 rclpy（它内部要建 ROS 节点）。顺序反了之后
    # 报错指向 rclpy，而真实原因是"SDK 已经初始化过了" —— 归因方向完全错。
    #
    # `if not rclpy.ok()` 这层保护是为了让本函数在**已初始化的进程里**
    # 也能被调用（测试、或将来被别的入口复用），而不是修顺序问题本身。
    if not rclpy.ok():
        rclpy.init(args=args)

    # 这一步会 setdefault ASTRIBOT_LOG / ROBOT_TYPE 并自检，然后才真正 import SDK
    try:
        session = open_session(freq=250.0, node_name='astribot_bridge_session',
                              logger=_BootLogger)
    except SdkSessionError as exc:
        # 会话建不起来就**响亮失败**，不进入"看起来在跑但什么都不做"的状态
        _BootLogger.error('SDK 会话建立失败，桥接容器退出：\n%s' % exc)
        rclpy.try_shutdown()
        return 1

    nodes = []
    try:
        chassis = ChassisCmdBridgeNode(session)
        nodes.append(chassis)
    except Exception as exc:      # noqa: BLE001
        _BootLogger.error('底盘桥接构造失败：%s' % exc)
    try:
        arm = ArmTrajBridgeNode(session)
        nodes.append(arm)
    except Exception as exc:      # noqa: BLE001
        _BootLogger.error('机械臂桥接构造失败：%s' % exc)

    if not nodes:
        _BootLogger.error('没有任何桥接节点构造成功，容器退出')
        rclpy.try_shutdown()
        return 2

    # 线程数 = 节点数 * 2（内外环/Action+服务）+ 2 余量。
    # 给足线程是必要的：不够时 cancel 回调会排在 execute 后面，取消就失效了。
    executor = MultiThreadedExecutor(num_threads=len(nodes) * 2 + 2)
    for n in nodes:
        executor.add_node(n)

    _BootLogger.info('桥接容器就绪：%d 个节点，%d 个执行线程'
                     % (len(nodes), len(nodes) * 2 + 2))
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
