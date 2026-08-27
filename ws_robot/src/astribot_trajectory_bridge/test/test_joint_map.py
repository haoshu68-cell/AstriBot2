#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bridge.yaml 的映射表与 URDF 的一致性测试（不需要 SDK 后端）。

本测试与 joint_map_probe 是**两件不同的事**，不要混：

- 本测试（离线，确定性）：验证 bridge.yaml 声明的关节名在 URDF 里真实存在、
  是主动关节而不是 mimic 从动关节、夹爪的 scale 与 URDF 限位自洽。
  也就是"我这边自己有没有说错话"。
- `joint_map_probe`（在线，需要后端）：验证部件内的**顺序**与 SDK 一致。
  也就是"我说的话与厂商是否指同一台机器"。

顺序这件事离线测不出来 —— 顺序的真值只存在于 SDK 的编译产物里。
所以本测试**不声称**验证了顺序，只保证顺序之外的一切都对。
把两者混为一谈会造成"测试全绿所以映射没问题"的错觉，而顺序恰恰是最容易错、
错了又最难发现的那一项（姿态全错但话题格式完全正常）。
"""

import os
import subprocess

import pytest
import yaml

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.abspath(os.path.join(_THIS_DIR, '..'))
_REPO_ROOT = os.path.abspath(os.path.join(_PKG_DIR, '..', '..', '..'))
_BRIDGE_YAML = os.path.join(_PKG_DIR, 'config', 'bridge.yaml')
_TOP_XACRO = os.path.join(
    _REPO_ROOT, 'ws_robot', 'src', 'astribot_s1_description', 'urdf', 'astribot_s1.xacro')

# SDK 的 whole_body_names/dofs（示例 208 与 get_info 文档串一致）：
#   torso 4 / arm_left 7 / gripper_left 1 / arm_right 7 / gripper_right 1 / head 2
# 合计 22，**不含底盘**（底盘在厂商侧是另外 3 个虚拟关节，本决策已砍掉导航）。
_EXPECTED_TOTAL_DOF = 22


@pytest.fixture(scope='module')
def bridge_cfg():
    with open(_BRIDGE_YAML, 'r', encoding='utf-8') as handle:
        raw = yaml.safe_load(handle)
    return raw['/**']['ros__parameters']


@pytest.fixture(scope='module')
def urdf_joints():
    """展开顶层 xacro，返回 {关节名: (type, mimic_of_or_None, lower, upper)}。"""
    result = subprocess.run(
        ['xacro', _TOP_XACRO, 'robot_name:=astribot_s1'],
        capture_output=True, text=True)
    if result.returncode != 0:
        # !!! 区分"环境没准备好"与"URDF 真的有问题" !!!
        # 顶层 xacro 内部用 $(find astribot_s1_description) 解析 mesh 路径，
        # 所以展开它**要求工作区已 build 且 install/setup.bash 已 source**。
        # 在裸 colcon test（只 source 了 /opt/ros/humble）下必然失败。
        #
        # 这种情况必须 skip 而不是 error：否则整个包的 colcon test 恒为红，
        # 把同包内其它测试的真实状态一起掩盖掉（实测就掩盖过一次）。
        # 判据取 PackageNotFoundError —— 只有"包找不到"才算环境问题，
        # 其它任何 xacro 错误仍然当失败报出来，不放过真正的 URDF 问题。
        if 'PackageNotFoundError' in result.stderr:
            pytest.skip(
                '需要先 build 工作区并 source install/setup.bash 才能展开 xacro'
                '（顶层 xacro 用 $(find astribot_s1_description) 解析路径）。'
                '本条不是失败，是环境未就绪。')
        raise AssertionError('xacro 展开失败:\n%s' % result.stderr)
    import xml.etree.ElementTree as ET
    root = ET.fromstring(result.stdout)
    out = {}
    for joint in root.findall('joint'):
        mimic = joint.find('mimic')
        limit = joint.find('limit')
        out[joint.get('name')] = (
            joint.get('type'),
            mimic.get('joint') if mimic is not None else None,
            float(limit.get('lower')) if limit is not None and limit.get('lower') else None,
            float(limit.get('upper')) if limit is not None and limit.get('upper') else None,
        )
    return out


def _mapped(cfg):
    """返回 [(part, [joint...])]，顺序与 parts 一致。"""
    jm = cfg['joint_map']
    return [(part, jm[part]['joints']) for part in jm['parts']]


def test_every_part_has_a_mapping(bridge_cfg):
    """parts 列表里的每个部件都必须有对应的映射块。"""
    jm = bridge_cfg['joint_map']
    for part in jm['parts']:
        assert part in jm, 'parts 里列了 %s 但没有它的映射块' % part
        assert jm[part].get('joints'), '%s 的 joints 为空' % part


def test_total_dof_matches_sdk_layout(bridge_cfg):
    """关节总数 = 22，与 SDK 的 whole_body_dofs 合计一致。

    这条挡的是"多列或漏列一个部件"。漏了某部件，MoveIt 会一直报
    complete state not known；多列了，桥接启动期就会因 DOF 不符被拒。
    """
    total = sum(len(joints) for _, joints in _mapped(bridge_cfg))
    assert total == _EXPECTED_TOTAL_DOF, (
        '映射表关节总数 %d != SDK 的 22（torso4+armL7+gripL1+armR7+gripR1+head2）'
        % total)


def test_no_chassis_in_mapping(bridge_cfg):
    """底盘不能进映射表。

    厂商侧底盘是 3 个虚拟关节 astribot_chassis_x/y/z_rot，走 SLAM 世界系的
    位置指令，不在 whole_body_names 里，也不是 /joint_states 该表达的东西。
    """
    for part, joints in _mapped(bridge_cfg):
        assert 'chassis' not in part, '部件 %s 看起来是底盘，不该进状态桥接' % part
        for name in joints:
            assert 'chassis' not in name and 'wheel' not in name.lower(), (
                '%s 里出现了底盘/轮子关节 %s' % (part, name))


def test_all_mapped_joints_exist_in_urdf(bridge_cfg, urdf_joints):
    """映射到的每个关节名都必须在 URDF 里真实存在。

    名字打错的后果很隐蔽：robot_state_publisher 会忽略它不认识的关节名，
    于是那个关节永远停在 0，而话题里明明有值。
    """
    for part, joints in _mapped(bridge_cfg):
        for name in joints:
            assert name in urdf_joints, (
                'bridge.yaml 的 %s 映射到了 URDF 里不存在的关节 %s' % (part, name))


def test_mapped_joints_are_active_not_mimic(bridge_cfg, urdf_joints):
    """只能映射主动关节；mimic 从动关节绝不能出现在这里。

    夹爪每侧 6 个关节里只有 joint_L1 是主动的。把 mimic 关节也发出去会造成
    同一自由度两个来源：桥接发一份、robot_state_publisher 按 mimic 又算一份，
    两边一旦有出入就出现无法解释的姿态抖动（已实测 Gazebo 侧的 mimic
    就有 7.8° 稳态误差，正是这类出入）。
    """
    for part, joints in _mapped(bridge_cfg):
        for name in joints:
            joint_type, mimic_of, _, _ = urdf_joints[name]
            assert mimic_of is None, (
                '%s 映射了 mimic 从动关节 %s（它 mimic 的是 %s）—— '
                '从动关节的值由 robot_state_publisher 算，桥接不该发'
                % (part, name, mimic_of))
            assert joint_type != 'fixed', '%s 映射了 fixed 关节 %s' % (part, name)


def test_gripper_scale_consistent_with_urdf_limit(bridge_cfg, urdf_joints):
    """夹爪 scale 必须与 URDF 限位自洽：scale * 100 == 主动关节上限。

    厂商侧夹爪指令是 0~100 的抽象量，URDF 侧是 0~0.93 rad。
    scale=0.0093 这个数来自 MuJoCo actuator 的伺服平衡点
    （gainprm 4.65 / biasprm -500 => len = 0.0093*ctrl）。
    这条测试把它与 URDF 的行程绑在一起：谁改了 gripper_drive_upper 而忘了改
    scale，桥接就会把 0~100 映射到错误的弧度区间 —— 夹爪开合幅度整体偏掉，
    而没有任何报错。
    """
    checked = 0
    for part, joints in _mapped(bridge_cfg):
        if 'gripper' not in part:
            continue
        assert len(joints) == 1, '夹爪部件 %s 应只映射 1 个主动关节' % part
        scale = float(bridge_cfg['joint_map'][part]['scale'])
        offset = float(bridge_cfg['joint_map'][part]['offset'])
        _, _, lower, upper = urdf_joints[joints[0]]
        assert offset == 0.0, '夹爪 offset 应为 0（0 = 张开，两侧原点对齐）'
        assert abs(scale * 0.0 + offset - lower) < 1e-9, (
            '%s: SDK 指令 0 应映射到 URDF 下限 %.4f' % (part, lower))
        assert abs(scale * 100.0 + offset - upper) < 1e-9, (
            '%s: SDK 指令 100 应映射到 URDF 上限 %.4f，实际映射到 %.6f。'
            'scale=%.6f 与 URDF 行程不自洽。' % (part, upper, scale * 100.0, scale))
        checked += 1
    assert checked == 2, '应检查左右两个夹爪，实际检查了 %d 个' % checked


def test_non_gripper_parts_are_identity_scaled(bridge_cfg):
    """手臂/躯干/头部必须是 scale=1 / offset=0（弧度对弧度）。

    这些部件两边都是弧度，任何非 1 的 scale 都是错误 —— 而且是那种
    "姿态整体缩放、看着像标定问题"的错误，极难定位。
    """
    for part, _ in _mapped(bridge_cfg):
        if 'gripper' in part:
            continue
        scale = float(bridge_cfg['joint_map'][part]['scale'])
        offset = float(bridge_cfg['joint_map'][part]['offset'])
        assert abs(scale - 1.0) < 1e-12, '%s 的 scale 应为 1.0，实际 %r' % (part, scale)
        assert abs(offset) < 1e-12, '%s 的 offset 应为 0.0，实际 %r' % (part, offset)


def test_no_duplicate_joint_names(bridge_cfg):
    """同一个关节不能被两个部件映射。

    重复的后果：JointState 里同名关节出现两次，消费方行为未定义
    （robot_state_publisher 取哪个取决于实现），是个不可靠的静默错误。
    """
    seen = {}
    for part, joints in _mapped(bridge_cfg):
        for name in joints:
            assert name not in seen, (
                '关节 %s 同时出现在 %s 和 %s 里' % (name, seen[name], part))
            seen[name] = part


def test_readonly_boundary_is_configured(bridge_cfg):
    """只读边界必须在配置里就是关的：不申请控制权。

    这是 Gate 2 的安全前提 —— 没有控制权，即使桥接有 bug 也动不了机器人。
    节点里也有一道同样的检查（配置为 true 就拒绝启动），两处都留是有意的：
    配置层面挡住"手滑改了 yaml"，代码层面挡住"参数从别处覆盖进来"。
    """
    assert bridge_cfg['bridge']['sdk_high_control_rights'] is False, (
        'Gate 2 是只读桥接，sdk_high_control_rights 必须为 false')
