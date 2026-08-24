#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节限位一致性测试：本包展开出的 URDF 必须与厂商 **per-part** 模型逐项一致。

为什么需要这个测试
==================
厂商 astribot_config 里 ship 了**四份互不一致**的模型：

    model/astribot_arm_left.urdf 等 per-part 文件   <- SDK 运行时真正加载的
    model/astribot_whole_body.urdf
    model/astribot_whole_body_with_wheel.urdf        <- 本包早期抄的那一份
    model/astribot_whole_body_with_head.urdf
    model/astribot_whole_body_dynamic_calib.urdf     <- 速度一律 20，标定占位值

判断依据是各部件 yaml 的 ``model:`` 字段 —— 它指向的才是 SDK 的运行模型，
``whole_body_*.urdf`` 一份都没被引用。所以"与真机对齐"只能以 per-band 文件为准。

本包早期从 ``astribot_whole_body_with_wheel.urdf`` 抄数值（抄得很忠实，
逐项核对 0 处差异），但那是一份 SDK 不加载的模型，于是产生了两个实质风险：

* 躯干速度限位 6.0 rad/s vs 真机 1.8 rad/s —— **宽 3.3 倍，方向不安全**。
  时间最优参数化跑满时躯干段速度可达真机限位 3 倍，而 Gazebo 会照 URDF
  老实执行、什么都不报。
* 左臂 joint_3 限位 −3.0~1.4 vs 真机 ±3.1 —— 白丢 1.7 rad 行程，
  可达域分析偏悲观。

这个测试把"以 per-part 为准"这条规则**机器化**，让上面那种漂移不可能再发生，
而不是靠注释提醒。详见 docs/sim_real_alignment.md 第 0 节。

覆盖范围
========
per-part 模型覆盖 arm_left / arm_right / torso / head 共 20 个可动关节。
轮子与夹爪不在 per-part 文件里（chassis / gripper 用 yaml 内联 model{} 块），
不由本测试覆盖。
"""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

# 本文件位于 <repo>/ws_robot/src/astribot_s1_description/test/
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
_VENDOR_MODEL_DIR = os.path.join(
    _REPO_ROOT, 'astribot_config', 'robot_config', 'astribot_s1', 'model')
_TOP_XACRO = os.path.abspath(os.path.join(_THIS_DIR, '..', 'urdf', 'astribot_s1.xacro'))

# 以哪些 per-part 文件为真值。与各部件 yaml 的 model: 字段一一对应。
_PER_PART_FILES = (
    'astribot_arm_left.urdf',
    'astribot_arm_right.urdf',
    'astribot_torso.urdf',
    'astribot_head.urdf',
)

# 数值比较容差。URDF 里都是十进制字面量，1e-9 足够吸收解析误差，
# 又不会把 0.06 和 0.0 这种真实差异放过去。
_TOL = 1e-9


def _parse_joints(root):
    """从 URDF 根节点提取 {关节名: {字段: 值}}，只取可动关节的限位与原点。"""
    joints = {}
    for joint in root.findall('joint'):
        name = joint.get('name')
        entry = {'type': joint.get('type')}
        limit = joint.find('limit')
        if limit is not None:
            for key in ('lower', 'upper', 'velocity', 'effort'):
                raw = limit.get(key)
                entry[key] = None if raw is None else float(raw)
        origin = joint.find('origin')
        if origin is not None:
            for key in ('xyz', 'rpy'):
                raw = origin.get(key)
                if raw is not None:
                    entry[key] = tuple(float(v) for v in raw.split())
        joints[name] = entry
    return joints


def _expand_top_xacro():
    """展开本包顶层 xacro。失败时把 stderr 原样抛出——xacro 的报错信息本身就是线索。"""
    result = subprocess.run(
        ['xacro', _TOP_XACRO, 'robot_name:=astribot_s1'],
        capture_output=True, text=True, timeout=180)
    if result.returncode != 0:
        raise AssertionError(
            '展开 %s 失败(returncode=%d):\n%s' % (_TOP_XACRO, result.returncode, result.stderr))
    return ET.fromstring(result.stdout)


@pytest.fixture(scope='module')
def ours():
    return _parse_joints(_expand_top_xacro())


@pytest.fixture(scope='module')
def vendor():
    """合并全部 per-part 文件。同名关节在多份文件里重复出现时视为配置错误。"""
    merged = {}
    for filename in _PER_PART_FILES:
        path = os.path.join(_VENDOR_MODEL_DIR, filename)
        assert os.path.exists(path), (
            '找不到厂商 per-part 模型: %s\n'
            '本测试要求本包位于 <astribot_sdk_ros2>/ws_robot/src/astribot_s1_description' % path)
        parsed = _parse_joints(ET.parse(path).getroot())
        for name, entry in parsed.items():
            assert name not in merged, '关节 %s 在多个 per-part 文件里重复定义' % name
            merged[name] = entry
    return merged


def test_vendor_per_part_models_exist(vendor):
    """先确认真值源本身可读，避免后面的测试因为路径问题假通过。"""
    assert len(vendor) >= 20, '厂商 per-part 模型只解析出 %d 个关节，太少' % len(vendor)


def test_joint_limits_match_per_part(ours, vendor):
    """逐关节比对 lower/upper/velocity/effort。"""
    mismatches = []
    checked = 0
    for name, want in vendor.items():
        if want.get('type') == 'fixed':
            continue
        if name not in ours:
            mismatches.append('%s: 本包 URDF 里不存在这个关节' % name)
            continue
        got = ours[name]
        checked += 1
        for key in ('lower', 'upper', 'velocity', 'effort'):
            expected, actual = want.get(key), got.get(key)
            if expected is None and actual is None:
                continue
            if expected is None or actual is None or abs(expected - actual) > _TOL:
                mismatches.append(
                    '%s.%s: 本包=%s 厂商per-part=%s' % (name, key, actual, expected))
    assert checked >= 20, '只比对到 %d 个可动关节，覆盖不足' % checked
    assert not mismatches, (
        '本包 URDF 的关节限位与厂商 per-part 模型不一致，共 %d 处：\n  %s\n\n'
        '真值源是各部件 yaml 的 model: 字段指向的 per-part 文件，'
        '不是 whole_body_*.urdf。详见 docs/sim_real_alignment.md 第 0 节。'
        % (len(mismatches), '\n  '.join(mismatches)))


def test_joint_origins_match_per_part(ours, vendor):
    """连带锁住关节原点。

    实测 per-part 与 whole_body 在关节原点上逐项一致（0 处差异），
    所以这一项现在就是通过的；把它写成测试是为了防止将来有人"顺手"改几何——
    几何一改，FK 就与真机不一致，而这种错误在仿真里完全看不出来。
    """
    mismatches = []
    for name, want in vendor.items():
        if name not in ours:
            continue
        got = ours[name]
        for key in ('xyz', 'rpy'):
            expected = want.get(key, (0.0, 0.0, 0.0))
            actual = got.get(key, (0.0, 0.0, 0.0))
            if len(expected) != len(actual) or any(
                    abs(a - b) > _TOL for a, b in zip(expected, actual)):
                mismatches.append('%s.%s: 本包=%s 厂商per-part=%s' % (name, key, actual, expected))
    assert not mismatches, (
        '关节原点与厂商 per-part 模型不一致，共 %d 处：\n  %s'
        % (len(mismatches), '\n  '.join(mismatches)))


def test_not_using_whole_body_limits(ours):
    """哨兵测试：明确挡住"退回 whole_body 限位"这个具体的历史错误。

    这几个值是 whole_body_with_wheel.urdf 的特征值。一旦它们重新出现，
    说明有人又从错误的那份模型抄了数值——上面两个测试也会失败，
    但这个测试的报错信息能直接指出根因。
    """
    sentinels = [
        ('astribot_torso_joint_1', 'velocity', 6.0, '躯干速度 6.0 是 whole_body 的值，真机是 1.8'),
        ('astribot_torso_joint_4', 'upper', 1.5, '躯干 joint_4 上限 1.5 是 whole_body 的值，真机是 1.2'),
        ('astribot_arm_left_joint_3', 'upper', 1.4, '左臂 joint_3 上限 1.4 是 whole_body 的值，真机是 3.1'),
        ('astribot_arm_left_joint_4', 'lower', 0.0, '左臂 joint_4 下限 0.0 是 whole_body 的值，真机是 -0.06'),
    ]
    regressions = []
    for name, key, bad_value, why in sentinels:
        entry = ours.get(name)
        if entry is None:
            continue
        actual = entry.get(key)
        if actual is not None and abs(actual - bad_value) <= _TOL:
            regressions.append('%s.%s = %s —— %s' % (name, key, actual, why))
    assert not regressions, (
        '检测到退回 whole_body 限位：\n  %s' % '\n  '.join(regressions))
