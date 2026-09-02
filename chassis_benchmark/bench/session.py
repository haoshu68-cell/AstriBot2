#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运行前预检 + 运行上下文。

**这一层的全部目的是把「测出来的数字是假的」那几种情况拦在启动之前。**
本仓库真实踩过的、能在启动时就查出来的：

* 查询 shell 不带 ``ROS_DOMAIN_ID`` -> "节点全都 Node not found"，看着像系统没起来。
* 开环测试时 nav2 还在跑 -> ``arm_chassis_speed_coupling_node`` 也在发 ``/cmd_vel``，
  两个发布者抢同一个话题，测的是两者的叠加。
* 仿真里用墙钟时间戳 -> 时间轴全错。
* 长跑的仿真退化 -> 读数漂出硬限位、出现可复现的假尖峰。只能靠"记录启动时长 + 提醒重启"。

预检失败默认**拒绝运行**（``--force`` 可以强行继续，但会把这件事写进结果的 context，
事后能看出这份数据是在预检不通过的情况下取的）。
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

# 开环测试期间**不允许**存在的节点：它们会往 /cmd_vel 发东西，与测试脚本抢话题
CMD_VEL_COMPETITORS = (
    'arm_chassis_speed_coupling_node',
    'cmd_vel_body_to_world_node',
    'autonomous_patrol_node',
    'controller_server',
    'velocity_smoother',
)

EXPECTED_DOMAIN = {'sim': 42, 'real': 25}     # real 实测是 25，不是 42

DEFAULT_CMD_VEL_TOPIC = '/cmd_vel'
DEFAULT_ODOM_TOPIC = '/odom'
DRIVE_NODE_NAME = 'omni_effort_drive_node'


class PreflightError(RuntimeError):
    pass


class Session:
    """一次测试运行的 ROS 上下文。用 with 语句管理。"""

    def __init__(self, test_id, env, node_name=None, force=False,
                 need_open_loop=True, extra_context=None):
        self.test_id = test_id
        self.env = env
        self.force = force
        self.need_open_loop = need_open_loop
        self._node_name = node_name or f'bench_{test_id}'
        self.context = {'env': env, 'test_id': test_id}
        self.context.update(extra_context or {})
        self.warnings = []
        self.node = None
        self._clock_msgs = 0

    # ---------- 生命周期 ----------
    def __enter__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(self._node_name)
        try:
            self.preflight()
        except Exception:
            self.close()
            raise
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if rclpy.ok():
            rclpy.shutdown()

    def spin_for(self, seconds):
        """按**墙钟**等待并处理回调。仿真里的时长判据不要用这个，用 /clock。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    # ---------- 预检 ----------
    def preflight(self):
        problems = []

        # 1. ROS_DOMAIN_ID
        raw = os.environ.get('ROS_DOMAIN_ID')
        self.context['ROS_DOMAIN_ID'] = raw
        expected = EXPECTED_DOMAIN.get(self.env)
        # 空串与未设置是两回事但后果一样（都落到默认域 0），一并按"未设置"处理。
        # 非数字值也要拦——不能让 int() 抛 ValueError 变成一个看不懂的 traceback。
        if raw is None or raw.strip() == '':
            problems.append(
                f'ROS_DOMAIN_ID 未设置（或为空串，会落到默认域 0）。'
                f'{self.env} 环境期望 {expected}。'
                f'不设的话会看到"节点全都 Node not found"，误以为系统没起来。')
        else:
            try:
                domain = int(raw.strip())
            except ValueError:
                problems.append(f'ROS_DOMAIN_ID={raw!r} 不是整数，DDS 会拒绝或落到默认域。')
            else:
                self.context['ROS_DOMAIN_ID'] = domain
                if expected is not None and domain != expected:
                    self.warnings.append(
                        f'ROS_DOMAIN_ID={domain}，而 {self.env} 环境的常用值是 {expected}。'
                        f'若是有意为之请忽略。')

        # 2. 发现节点（顺带确认 DDS 通了）
        self.spin_for(2.0)
        names = [n for n, _ in self.node.get_node_names_and_namespaces()]
        self.context['nodes_seen'] = len(names)
        if len(names) <= 1:
            problems.append(
                f'只发现 {len(names)} 个节点。DDS 没通，或者仿真/机器人没起来。'
                f'先确认 ROS_DOMAIN_ID 与 launch 侧一致。')

        # 3. 驱动节点在不在（仿真）
        if self.env == 'sim':
            if DRIVE_NODE_NAME not in names:
                problems.append(
                    f'没发现 {DRIVE_NODE_NAME}。底盘完全靠它算力矩驱动，'
                    f'不在就是一点驱动力都没有（VelocityControl/MecanumDrive 已整体移除）。'
                    f'检查 warehouse_sim.launch.py 的 enable_effort_drive:=true。')

            # 4. /clock 在不在：仿真时间判据的前提
            self._check_clock()

        # 5. 开环测试：/cmd_vel 不能有别的发布者
        if self.need_open_loop:
            competitors = [n for n in names if n in CMD_VEL_COMPETITORS]
            n_pub = self.node.count_publishers(DEFAULT_CMD_VEL_TOPIC)
            self.context['cmd_vel_publishers_before_test'] = n_pub
            self.context['cmd_vel_competitors'] = competitors
            if competitors:
                problems.append(
                    f'检测到会抢 {DEFAULT_CMD_VEL_TOPIC} 的节点：{competitors}。'
                    f'开环测试必须只起 warehouse_sim.launch.py，不要起 nav2——'
                    f'否则测的是测试指令与它们输出的叠加。')
            elif n_pub > 0:
                self.warnings.append(
                    f'{DEFAULT_CMD_VEL_TOPIC} 已有 {n_pub} 个发布者但没识别出是谁。'
                    f'先 `ros2 topic info -v {DEFAULT_CMD_VEL_TOPIC}` 查清楚。')

        # 6. odom 在不在（仿真里它是真值源）
        if self.node.count_publishers(DEFAULT_ODOM_TOPIC) == 0:
            problems.append(
                f'{DEFAULT_ODOM_TOPIC} 没有发布者。仿真里它来自 OdometryPublisher 插件、'
                f'基于模型真实位姿，是本组测试的真值源，缺了就没法测。')

        self.context['preflight_warnings'] = self.warnings
        for w in self.warnings:
            self.node.get_logger().warning(f'[预检] {w}')

        if problems:
            msg = '预检未通过：\n' + '\n'.join(f'  {i+1}. {p}' for i, p in enumerate(problems))
            self.context['preflight_problems'] = problems
            if not self.force:
                raise PreflightError(msg + '\n\n确认无误可加 --force 强行继续'
                                           '（会记进结果 context）。')
            self.context['preflight_forced'] = True
            self.node.get_logger().error(msg + '\n\n--force 已指定，继续运行。'
                                               '这份数据的可信度自负。')

    def _check_clock(self):
        self._clock_msgs = 0

        def _cb(_msg):
            self._clock_msgs += 1

        sub = self.node.create_subscription(Clock, '/clock', _cb, 10)
        self.spin_for(1.5)
        self.node.destroy_subscription(sub)
        self.context['clock_msgs_in_1_5s'] = self._clock_msgs
        if self._clock_msgs == 0:
            self.warnings.append(
                '/clock 没有消息。仿真里所有时长判据都必须用仿真时间，'
                '拿墙钟算会得到错的时间轴。确认 use_sim_time 与 gz 的 clock 桥。')


def base_arg_parser(test_id, description):
    """所有测试脚本共用的命令行骨架。"""
    p = argparse.ArgumentParser(prog=test_id, description=description)
    p.add_argument('--env', choices=('sim', 'real'), required=True,
                   help='sim=Gazebo(真值取 /odom)  real=真机(真值靠人工量)')
    p.add_argument('--n', type=int, default=10, help='每个条件的重复次数')
    p.add_argument('--force', action='store_true',
                   help='预检不通过也继续（会记进结果 context）')
    p.add_argument('--results-dir', default=None, help='结果输出目录')
    return p


def results_dir(explicit=None):
    if explicit:
        return explicit
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'results')


def run_main(main_fn, argv=None):
    """所有测试脚本共用的入口包装。

    职责只有一个：**让失败以人能读的形式出现**。预检失败是最常见的失败，
    它应该给出一段"下一步该做什么"的文字，而不是一段 traceback ——
    traceback 会让人去查代码，而问题其实在环境。

    退出码：0 通过 / 1 未通过或失败 / 2 环境或用法问题 / 130 中断
    """
    import rclpy as _rclpy
    # 非数字的 ROS_DOMAIN_ID 会让 rclpy.init() 在 rcl 层抛 RCLError，
    # 那发生在 Session 的预检之前，所以这一条必须在 init 之前先查。
    raw = os.environ.get('ROS_DOMAIN_ID')
    if raw is not None and raw.strip() != '':
        try:
            int(raw.strip())
        except ValueError:
            print(f'\n[环境错误] ROS_DOMAIN_ID={raw!r} 不是整数，rcl 会直接拒绝初始化。\n'
                  f'          sim 期望 {EXPECTED_DOMAIN["sim"]}，'
                  f'real 期望 {EXPECTED_DOMAIN["real"]}。\n', file=sys.stderr)
            return 2
    try:
        return main_fn(argv)
    except PreflightError as exc:
        print(f'\n[预检失败]\n{exc}\n', file=sys.stderr)
        return 2
    except TimeoutError as exc:
        print(f'\n[超时] {exc}\n', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\n[中断]', file=sys.stderr)
        return 130
    finally:
        if _rclpy.ok():
            _rclpy.shutdown()
