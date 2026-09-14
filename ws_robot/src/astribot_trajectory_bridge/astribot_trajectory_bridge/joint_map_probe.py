#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节映射探针：验证 bridge.yaml 里「部件内关节顺序」这个断言。

为什么需要单独一个探针
====================
厂商 SDK 的 `get_current_joints_position(names)` 返回**按部件成组的裸数组**，
数组里第 i 个数对应哪个物理关节，SDK 没有任何地方声明；而 SDK 核心是编译好的
`astribot_function.so`，源码不可读 —— 读代码得不到答案。

顺序错了的后果很隐蔽：`/joint_states` 照样发得出来、话题里也有 22 个值、
RViz 里机器人也在动，只是**动的姿态是错的**。MoveIt 会拿这个错姿态当规划起点，
于是"规划成功但一执行就撞"，而日志里一切正常。

判据：拿限位向量做指纹，不依赖任何假设
====================================
SDK 有 `get_joints_position_limit(names)`（返回 `(lower, upper)`，D6 已定；
本探针同时用 SDK 源码复核了这一点 —— `astribot_interface.py:425` 是
`return joints_position_limit["lower"], joints_position_limit["upper"]`）。

臂的 7 个关节限位互不相同：
    -3.1/3.1、-1.53/0.46、±3.1、-0.06/2.61、±2.56、±0.76、±1.53
所以「按 bridge.yaml 的顺序从 URDF 取出的限位向量」必须与「SDK 返回的限位向量」
逐项相等。顺序错了就对不上。这是个**独立于我的假设**的判据：
两边的数据来源完全不同（一边是 URDF 展开，一边是 SDK 运行时），
而它们本该指向同一台机器。

注意这条判据的边界（不要过度解读）
--------------------------------
- 限位相同的关节之间无法区分。躯干四个关节里 joint_1(-0.04~1.5) /
  joint_2(-2.3~0.06) / joint_3(-0.4~2.3) / joint_4(±1.2) 互不相同，能区分；
  头部两个关节若限位相同则**区分不了**，本探针会明确报告"该部件不可判定"，
  而不是假装通过。
- 判据成立的前提是 URDF 限位已经与 SDK 同源。这一点由 Gate 1 的
  test_joint_limits_parity.py 保证（限位取自各部件 yaml 的 model: 字段）。
  所以两条测试是串起来的：Gate 1 保证"限位对"，本探针用"限位对"去反推"顺序对"。
"""

from astribot_logging import get_logger

import sys

import rclpy
from rclpy.node import Node


class JointMapProbe(Node):
    """连 SDK，把限位向量与 bridge.yaml 声明的顺序做比对。"""

    def __init__(self):
        super().__init__('joint_map_probe')
        self._declare_and_load()

    def _declare_and_load(self):
        self.declare_parameter('bridge.sdk_freq', 250.0)
        self.declare_parameter('bridge.sdk_node_name', 'astribot_joint_map_probe')
        self.declare_parameter('joint_map.parts', [''])
        self.sdk_freq = self.get_parameter('bridge.sdk_freq').value
        self.sdk_node_name = self.get_parameter('bridge.sdk_node_name').value
        self.parts = [p for p in self.get_parameter('joint_map.parts').value if p]
        if not self.parts:
            raise ValueError(
                'joint_map.parts 为空。必须用 --params-file 传入 bridge.yaml，'
                '本探针不内置任何默认映射（内置默认值等于把要验证的假设藏起来）。')
        self.joint_names = {}
        for part in self.parts:
            key = 'joint_map.%s.joints' % part
            self.declare_parameter(key, [''])
            names = [n for n in self.get_parameter(key).value if n]
            if not names:
                raise ValueError('%s 没有配置关节名列表' % key)
            self.joint_names[part] = names


def _urdf_limits(urdf_text, names):
    """从 URDF 文本里按给定顺序取出 (lower, upper)。缺关节直接抛，不静默跳过。"""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_text)
    table = {}
    for joint in root.findall('joint'):
        limit = joint.find('limit')
        if limit is None:
            continue
        table[joint.get('name')] = (
            float(limit.get('lower')), float(limit.get('upper')))
    out = []
    for name in names:
        if name not in table:
            raise KeyError('URDF 里找不到带 <limit> 的关节 %s' % name)
        out.append(table[name])
    return out


def _fingerprint_is_decidable(limits):
    """限位向量能否唯一定序：任意两个关节的 (lower,upper) 不同才可判定。"""
    seen = set()
    for pair in limits:
        key = (round(pair[0], 9), round(pair[1], 9))
        if key in seen:
            return False
        seen.add(key)
    return True


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    exit_code = 0
    try:
        node = JointMapProbe()
        logger = node.get_logger()

        node.declare_parameter('robot_description', '')
        urdf_text = node.get_parameter('robot_description').value
        if not urdf_text:
            logger.error(
                '缺 robot_description 参数。请随 --params-file 一起传入，'
                '或先起 robot_state_publisher 后用 -p robot_description:="$(xacro ...)"。')
            return 1

        import os
        if os.environ.get('ASTRIBOT_LOG', '').lower() not in ('1', 'true', 'on'):
            os.environ['ASTRIBOT_LOG'] = '1'
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
        logger.info('正在连接厂商 SDK（freq=%.1f, high_control_rights=False）...'
                    % node.sdk_freq)
        robot = Astribot(freq=node.sdk_freq, high_control_rights=False,
                         node_name=node.sdk_node_name)

        logger.info('SDK whole_body_names = %s' % (robot.whole_body_names,))
        logger.info('SDK whole_body_dofs  = %s' % (robot.whole_body_dofs,))

        ok = True
        sdk_dofs = dict(zip(robot.whole_body_names, robot.whole_body_dofs))
        for part in node.parts:
            if part not in sdk_dofs:
                logger.error('bridge.yaml 里的部件 %s 不在 SDK 的 whole_body_names 里' % part)
                ok = False
                continue
            want = len(node.joint_names[part])
            got = sdk_dofs[part]
            mark = 'OK ' if want == got else '不符'
            logger.info('[DOF] %-26s SDK=%d  bridge.yaml=%d  %s' % (part, got, want, mark))
            if want != got:
                ok = False
        if not ok:
            logger.error('部件清单/DOF 数就不一致，顺序无从谈起。先修 bridge.yaml。')
            return 1

        logger.info('---- 限位指纹比对（判据：URDF 按 bridge.yaml 顺序取出的限位'
                    '必须与 SDK 返回的逐项相等）----')
        undecidable = []
        for part in node.parts:
            names = node.joint_names[part]
            lower, upper = robot.get_joints_position_limit([part])
            sdk_lo, sdk_up = list(lower[0]), list(upper[0])
            urdf = _urdf_limits(urdf_text, names)

            if not _fingerprint_is_decidable(urdf):
                undecidable.append(part)
                logger.warning(
                    '[%s] 该部件内有关节限位完全相同，**限位指纹无法唯一定序** —— '
                    '本探针对该部件只能证明"限位集合一致"，不能证明"顺序正确"。'
                    '不要把它当成已验证。' % part)

            if len(sdk_lo) != len(names) or len(sdk_up) != len(names):
                logger.error('[%s] SDK 限位长度 %d/%d 与关节数 %d 不符'
                             % (part, len(sdk_lo), len(sdk_up), len(names)))
                ok = False
                continue

            for idx, name in enumerate(names):
                u_lo, u_up = urdf[idx]
                d_lo = abs(u_lo - sdk_lo[idx])
                d_up = abs(u_up - sdk_up[idx])
                good = d_lo < 1.0e-6 and d_up < 1.0e-6
                logger.info(
                    '  [%d] %-34s URDF(%+.4f,%+.4f)  SDK(%+.4f,%+.4f)  %s'
                    % (idx, name, u_lo, u_up, sdk_lo[idx], sdk_up[idx],
                       'OK' if good else '<<< 不符'))
                if not good:
                    ok = False

        logger.info('---- 当前关节位置（SDK 原始值，按部件）----')
        positions = robot.get_current_joints_position(node.parts)
        for part, values in zip(node.parts, positions):
            logger.info('  %-26s %s' % (part, [round(float(v), 5) for v in values]))

        if ok:
            if undecidable:
                logger.warning(
                    '限位指纹全部吻合，但以下部件**不可判定**（限位有重复）: %s。'
                    '这些部件的顺序仍是未验证的假设。' % undecidable)
            logger.info('结论：bridge.yaml 的部件划分与关节顺序通过限位指纹检验。')
        else:
            logger.error('结论：**不通过**。上面标了"不符"的行就是错位处。')
            exit_code = 1
    except Exception as exc:  # noqa: BLE001 —— 探针必须把任何异常变成清晰报告
        if node is not None:
            node.get_logger().error('探针失败: %s: %s' % (type(exc).__name__, exc))
        else:
            get_logger('astribot.joint_map_probe').error('探针失败（节点都没建起来）: %s: %s' % (type(exc).__name__, exc))
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
