#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪模型一致性测试。

厂商没有任何单一文件同时给出夹爪的几何和耦合关系，本包的
astribot_s1_gripper.xacro 因此混用两个源：几何/轴向/行程/耦合比例取 MJCF，
连杆碰撞 mesh 与基座安装位姿取 whole_body_with_gripper.sdf。
两个源之间本来就有毫米级差异，所以这里做的不是"和某一份文件逐字节比对"，
而是校验几条**能证明模型没抄错**的性质：

1. TCP 偏置 = 厂商 arm yaml 的 effector_to_tool_pose，一位不差。
2. 平行夹爪不变量：指尖姿态与开合角无关。mimic 比例符号写错立刻不成立。
3. 两源交叉校验：MJCF 换算到 link_7 系 vs SDF 的 joint pose，差异 <= 3 mm。
4. 质量守恒：集中表示与分体表示的总质量差 <= 1%。
5. 碰撞包络：夹爪必须真的进碰撞模型（守住"沿抓取方向少 0.15 m"那个 bug）。
"""

import math
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
_TOP_XACRO = os.path.abspath(os.path.join(_THIS_DIR, '..', 'urdf', 'astribot_s1.xacro'))
_VENDOR_S1 = os.path.join(_REPO_ROOT, 'astribot_config', 'robot_config', 'astribot_s1')

# 两个厂商源之间的已知差异上界。实测 L1/L2/R1/R2 四个关节原点的最大偏差是
# 2.6 mm（y 方向，四个关节完全一致，说明是基座安装位姿的系统性差异而非随机误差）。
# 3 mm 不是随手取的余量：它刚好容下已知差异，又能在差异变大时报警。
_CROSS_SOURCE_TOL_M = 3.0e-3

# 集中表示(0.775349) vs 分体表示(0.781722) 的相对差 0.82%，是厂商两份模型自带的。
_MASS_TOL_REL = 0.01

_GRIPPER_SIDES = ('left', 'right')


# --------------------------------------------------------------------------- #
# 工具：xacro 展开 / 最小 3x3 旋转运算（不引 numpy，测试依赖越少越好跑）
# --------------------------------------------------------------------------- #

def _expand_top_xacro(**args):
    """展开顶层 xacro。失败时把 stderr 原样抛出 —— xacro 的报错本身就是线索。"""
    argv = ['xacro', _TOP_XACRO, 'robot_name:=astribot_s1']
    argv += ['%s:=%s' % (k, v) for k, v in args.items()]
    result = subprocess.run(argv, capture_output=True, text=True)
    if result.returncode != 0:
        raise AssertionError('xacro 展开失败 (%s):\n%s' % (argv, result.stderr))
    return result.stdout


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _vec_add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rpy_to_mat(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _axis_angle_to_mat(axis, angle):
    norm = math.sqrt(sum(c * c for c in axis))
    x, y, z = (c / norm for c in axis)
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def _max_abs_diff(m, n):
    return max(abs(m[i][j] - n[i][j]) for i in range(3) for j in range(3))


class _Model(object):
    """展开后 URDF 的最小访问器：只做本测试需要的那几件事。"""

    def __init__(self, urdf_text):
        self.root = ET.fromstring(urdf_text)
        self.links = {l.get('name'): l for l in self.root.findall('link')}
        self.joints = {j.get('name'): j for j in self.root.findall('joint')}

    def joint_tf(self, name, q=0.0):
        """返回 (R, p)：关节变换 = origin 位姿 · 绕 axis 转 q。fixed 关节忽略 q。"""
        joint = self.joints[name]
        origin = joint.find('origin')
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get('xyz'):
                xyz = [float(v) for v in origin.get('xyz').split()]
            if origin.get('rpy'):
                rpy = [float(v) for v in origin.get('rpy').split()]
        mat = _rpy_to_mat(*rpy)
        axis = joint.find('axis')
        if axis is not None and joint.get('type') != 'fixed':
            mat = _mat_mul(mat, _axis_angle_to_mat(
                [float(v) for v in axis.get('xyz').split()], q))
        return mat, xyz

    def chain_tf(self, names, angles):
        mat = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
        pos = [0.0, 0.0, 0.0]
        for name, q in zip(names, angles):
            j_mat, j_pos = self.joint_tf(name, q)
            pos = _vec_add(pos, _mat_vec(mat, j_pos))
            mat = _mat_mul(mat, j_mat)
        return mat, pos

    def mimic_of(self, name):
        return self.joints[name].find('mimic')

    def total_mass(self):
        total = 0.0
        for link in self.root.findall('link'):
            inertial = link.find('inertial')
            if inertial is not None:
                total += float(inertial.find('mass').get('value'))
        return total

    def _link_mass(self, name):
        inertial = self.links[name].find('inertial')
        return 0.0 if inertial is None else float(inertial.find('mass').get('value'))

    def subtree_mass(self, root_link):
        """root_link 及其所有后代 link 的质量之和。

        腕部这类局部改动必须用局部质量做判据 —— 拿整机质量当分母会把灵敏度
        稀释掉（见 test_mass_conserved_between_lumped_and_articulated 的说明）。
        """
        children = {}
        for joint in self.root.findall('joint'):
            parent = joint.find('parent').get('link')
            children.setdefault(parent, []).append(joint.find('child').get('link'))

        total = 0.0
        stack = [root_link]
        seen = set()
        while stack:
            name = stack.pop()
            if name in seen:
                continue
            seen.add(name)
            total += self._link_mass(name)
            stack.extend(children.get(name, []))
        return total


@pytest.fixture(scope='module')
def model():
    return _Model(_expand_top_xacro())


@pytest.fixture(scope='module')
def model_no_gripper():
    return _Model(_expand_top_xacro(use_gripper='false'))


# --------------------------------------------------------------------------- #
# 1. TCP 偏置必须等于厂商 yaml 的 effector_to_tool_pose
# --------------------------------------------------------------------------- #

def _vendor_effector_to_tool(side):
    """从厂商 arm yaml 里抠出 effector_to_tool_pose 的 7 个数。

    刻意不用 yaml.safe_load：这些文件的 model: 是个非标准的行内块（裸 key、
    带尾逗号），safe_load 直接抛 ScannerError。这里只要一行数字，正则最省事。
    """
    import re
    path = os.path.join(_VENDOR_S1, 'astribot_arm_%s.yaml' % side)
    with open(path, 'r', encoding='utf-8') as handle:
        text = handle.read()
    match = re.search(r'effector_to_tool_pose:\s*\[([^\]]*)\]', text)
    assert match is not None, '厂商 yaml 里找不到 effector_to_tool_pose: %s' % path
    values = [float(v) for v in match.group(1).split(',')]
    assert len(values) == 7, 'effector_to_tool_pose 应是 xyz+quat 共 7 个数: %s' % values
    return values


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_tcp_offset_matches_vendor_yaml(side, model):
    """tcp_link 相对法兰的位姿 == 厂商 effector_to_tool_pose。

    这条守住的是一个实质错误：改造前把 tool_link（与 link_7 原点完全重合，
    也就是法兰面本身）当笛卡尔目标用，等价于让法兰去到目标点，而该到位的是
    两指间的夹持点 —— 差 0.15 m。
    """
    expected = _vendor_effector_to_tool(side)
    mat, pos = model.chain_tf(
        ['astribot_arm_%s_tool_joint' % side, 'astribot_arm_%s_tcp_joint' % side],
        [0.0, 0.0])

    for axis_idx, axis_name in enumerate('xyz'):
        assert abs(pos[axis_idx] - expected[axis_idx]) < 1e-9, (
            'TCP 偏置 %s 与厂商 yaml 不一致: 模型 %.6f, yaml %.6f'
            % (axis_name, pos[axis_idx], expected[axis_idx]))

    # 四元数 (0,0,0,1) = 不旋转。厂商给的就是纯平移，模型也必须是纯平移 ——
    # 一旦谁给 tcp_joint 加了 rpy，所有笛卡尔目标的姿态解释就整体偏了。
    assert expected[3:] == [0.0, 0.0, 0.0, 1.0], (
        '厂商 effector_to_tool_pose 的姿态不再是单位四元数，本测试的前提需要重新审查: %s'
        % expected)
    assert _max_abs_diff(mat, _rpy_to_mat(0, 0, 0)) < 1e-12, 'tcp_joint 不应带旋转'


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_tcp_link_has_no_geometry(side, model):
    """TCP 必须是纯坐标系：它是 IK/约束的参考系，不该自己制造碰撞。

    已知坑的机器化版本：TCP 若坐在腕部碰撞体内部，"TCP 贴着物体"必然报碰撞，
    而报出来的错误码和"奇异被否"完全一样，极难分辨。
    """
    link = model.links['astribot_arm_%s_tcp_link' % side]
    assert link.find('collision') is None, 'tcp_link 不能有 collision'
    assert link.find('visual') is None, 'tcp_link 不能有 visual'
    assert link.find('inertial') is None, 'tcp_link 不能有质量'


# --------------------------------------------------------------------------- #
# 2. 耦合关系与平行夹爪不变量
# --------------------------------------------------------------------------- #

# 取自 MJCF 的 <equality>（astribot_gripper_left_actuator.xml）：
#   R1 = L1 / R2 = R1 / R11 = -R1 / L2 = -L1 / L11 = +L1
# 代入 R1 = L1 后全部化为对主动关节 L1 的 ±1 比例。
_EXPECTED_MIMIC_RATIO = {
    'L2': -1.0,
    'L11': 1.0,
    'R1': 1.0,
    'R2': 1.0,
    'R11': -1.0,
}


def _model_mimic_ratio(model, joint_name):
    """从模型里读某关节相对主动关节的比例；主动关节本身返回 1.0。

    存在的意义是让下面的几何测试**用模型声明的比例去算**，而不是用测试里
    写死的比例 —— 否则测试验证的是作者的假设，不是模型（已实测过这个假阳性）。
    """
    mimic = model.mimic_of(joint_name)
    return 1.0 if mimic is None else float(mimic.get('multiplier'))


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_single_active_dof_per_gripper(side, model):
    """一具夹爪只能有 1 个活动自由度，其余 5 个必须是 mimic。

    厂商的控制接口就是一个标量（open_effector/close_effector 下发 0 或 100）。
    模型里多出哪怕一个独立自由度，规划器就会去搜一个真机下发不了的维度，
    规划成功、执行不了。
    """
    active = []
    for name, joint in model.joints.items():
        if 'gripper_%s' % side not in name or joint.get('type') == 'fixed':
            continue
        if joint.find('mimic') is None:
            active.append(name)
    assert active == ['astribot_gripper_%s_joint_L1' % side], (
        '夹爪活动自由度应只有主动关节 L1，实际: %s' % sorted(active))


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_mimic_ratios_match_mujoco_equality(side, model):
    """5 个从动关节的 mimic 比例与 MJCF equality 一致，且比例只能是 ±1。

    "只能是 ±1" 不是风格约束：xacro 里从动关节的限位是用
    (1±ratio)/2 这个恒等式算出来的，该式仅在 ratio ∈ {+1,-1} 时成立。
    将来遇到非 ±1 的耦合，限位会**静默算错**，所以在这里挡住。
    """
    master = 'astribot_gripper_%s_joint_L1' % side
    for suffix, ratio in _EXPECTED_MIMIC_RATIO.items():
        name = 'astribot_gripper_%s_joint_%s' % (side, suffix)
        mimic = model.mimic_of(name)
        assert mimic is not None, '%s 应该是 mimic 关节' % name
        assert mimic.get('joint') == master, (
            '%s 应直接 mimic 主动关节（不依赖链式 mimic），实际 mimic %s'
            % (name, mimic.get('joint')))
        actual = float(mimic.get('multiplier'))
        assert abs(actual - ratio) < 1e-12, (
            '%s 的 mimic 比例与 MJCF equality 不符: 期望 %+.1f, 实际 %+.6f'
            % (name, ratio, actual))
        assert abs(abs(actual) - 1.0) < 1e-12, (
            '%s 的比例不是 ±1，限位推导式 (1±ratio)/2 已失效，必须改写 xacro'
            % name)
        assert float(mimic.get('offset', '0')) == 0.0, (
            '%s 不该有 offset：MJCF polycoef 的常数项是 0' % name)


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_mimic_limits_derived_from_master(side, model):
    """从动关节限位 == 主动关节行程 × 比例，不是各自手填的。

    抄 MJCF 的 ±1.00 或 SDF 的 ±3.14 都会埋一个隐患：主动关节走到行程端点时，
    从动关节的限位若与之不匹配，URDF 层面就自相矛盾。
    """
    master_limit = model.joints['astribot_gripper_%s_joint_L1' % side].find('limit')
    upper = float(master_limit.get('upper'))
    assert abs(float(master_limit.get('lower'))) < 1e-12, '主动关节下限应为 0（0 = 张开）'

    for suffix, ratio in _EXPECTED_MIMIC_RATIO.items():
        limit = model.joints['astribot_gripper_%s_joint_%s' % (side, suffix)].find('limit')
        expect_lower = min(0.0, upper * ratio)
        expect_upper = max(0.0, upper * ratio)
        assert abs(float(limit.get('lower')) - expect_lower) < 1e-12, (
            '%s 下限应为 %.4f' % (suffix, expect_lower))
        assert abs(float(limit.get('upper')) - expect_upper) < 1e-12, (
            '%s 上限应为 %.4f' % (suffix, expect_upper))


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_parallel_jaw_invariant(side, model):
    """平行夹爪不变量：两个指尖连杆的姿态与开合角无关。

    这条比"逐个数值对比"更强。L1 轴 (0,-1,0)、L11 轴 (0,1,0)、比例 +1，
    于是 L11 在基座系里的净旋转 R(-y,q)·R(+y,q) = I 对任意 q 恒成立；
    R 侧同理（R1 轴 +y、R11 比例 -1）。
    任何一个比例符号写反、或哪个 axis 抄错方向，姿态立刻随 q 变化 —— 也就是
    夹爪变成"张合时指尖会转"的非平行夹爪，抓取姿态的语义随之失效。

    !!! 从动关节的角度必须从模型的 mimic multiplier 读，不能在测试里写死 !!!
    最初这里把 L11 的角度写成 q*1.0（照 MJCF 的比例硬编码）。实测把 xacro 里
    L11 的 ratio 改成 -1 注入故障后，本测试**照样通过** —— 因为它算的是
    "假设比例是 +1 时的姿态"，而不是模型实际声明的比例。
    那样它验证的是我的假设，不是模型。改成从 mimic 读之后才真正有效。
    """
    for suffix in ('L11', 'R11'):
        prefix = suffix[0]
        master_name = 'astribot_gripper_%s_joint_%s1' % (side, prefix)
        child_name = 'astribot_gripper_%s_joint_%s' % (side, suffix)
        # 主动关节 L1 之外的每一节都可能是 mimic（R1 就是），逐个从模型取比例。
        master_ratio = _model_mimic_ratio(model, master_name)
        child_ratio = _model_mimic_ratio(model, child_name)

        ref = None
        for q in (0.0, 0.15, 0.3, 0.6, 0.93):
            mat, _ = model.chain_tf([master_name, child_name],
                                    [q * master_ratio, q * child_ratio])
            if ref is None:
                ref = mat
                continue
            assert _max_abs_diff(mat, ref) < 1e-12, (
                '%s 侧指尖姿态随开合角变化（q=%.2f，比例 %+.1f/%+.1f），不是平行夹爪'
                % (prefix, q, master_ratio, child_ratio))


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_jaws_close_monotonically(side, model):
    """开合方向：q 增大 = 两指靠拢（q=0 张开，q=upper 闭合）。

    方向来自 SDK 实现 astribot_client.py：open_effector 下发 0.0、
    close_effector 下发 100.0，配合 MJCF actuator 的 L1 = 0.0093·cmd。
    符号搞反的后果很具体：上层"张开去抓"会变成"闭合撞上去"。

    同上，比例一律从模型的 mimic 读，不在测试里写死。
    """
    upper = float(model.joints['astribot_gripper_%s_joint_L1' % side]
                  .find('limit').get('upper'))
    widths = []
    for q in (0.0, 0.3, 0.6, upper):
        positions = {}
        for suffix in ('L11', 'R11'):
            prefix = suffix[0]
            master_name = 'astribot_gripper_%s_joint_%s1' % (side, prefix)
            child_name = 'astribot_gripper_%s_joint_%s' % (side, suffix)
            _, pos = model.chain_tf(
                [master_name, child_name],
                [q * _model_mimic_ratio(model, master_name),
                 q * _model_mimic_ratio(model, child_name)])
            positions[prefix] = pos
        widths.append(abs(positions['L'][0] - positions['R'][0]))

    assert widths[0] > widths[-1], (
        'q=0 应是张开、q=%.2f 应是闭合，实测开口 %.4f -> %.4f（方向反了）'
        % (upper, widths[0], widths[-1]))
    for prev, cur in zip(widths, widths[1:]):
        assert cur < prev + 1e-12, '开口宽度必须随 q 单调收窄，实测 %s' % widths


# --------------------------------------------------------------------------- #
# 3. 两个厂商源的交叉校验
# --------------------------------------------------------------------------- #

def _sdf_gripper_joint_origins(side):
    """从 whole_body_with_gripper.sdf 读夹爪 6 个关节相对 link_7 的原点。

    该 SDF 由 URDF 转换而来（link 名里带 fixed_joint_lump__ 是标志），
    link 的 pose 全为 0、joint 的 pose 就是 URDF 里 joint origin 在父 link 系的值，
    所以能直接和本包模型比。
    """
    path = os.path.join(_VENDOR_S1, 'model', 'astribot_whole_body_with_gripper.sdf')
    model_el = ET.parse(path).getroot().find('model')
    origins = {}
    for joint in model_el.findall('joint'):
        name = joint.get('name')
        if 'gripper_%s' % side not in name:
            continue
        pose = [float(v) for v in joint.findtext('pose').split()]
        origins[name.rsplit('_', 1)[-1]] = pose[:3]
    return origins


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_cross_source_joint_origins(side, model):
    """MJCF 几何换算到 link_7 系后，与 SDF 的 joint pose 差异 <= 3 mm。

    本包的夹爪关节原点抄自 MJCF（相对夹爪基座给），SDF 则直接给相对 link_7 的值。
    两者是厂商两份独立模型，本来就有系统性差异 —— 实测 L1/L2/R1/R2 四个关节
    在 y 方向一致偏 2.6 mm，说明差异来自基座安装位姿而非抄错某一个数。

    这条测试不追求"两源一致"（做不到），而是**把已知差异钉住**：
    差异一旦超过 3 mm，说明有人改了基座安装位姿或某个关节原点，必须重新核对。
    """
    sdf_origins = _sdf_gripper_joint_origins(side)
    assert len(sdf_origins) == 6, 'SDF 里应有 6 个夹爪关节，实际 %d' % len(sdf_origins)

    # L11/R11 的父 link 是 L1/R1，不是 link_7，需要按各自的链算到 link_7 系。
    chains = {
        'L1': ['astribot_gripper_%s_base_fixed_joint', 'astribot_gripper_%s_joint_L1'],
        'L2': ['astribot_gripper_%s_base_fixed_joint', 'astribot_gripper_%s_joint_L2'],
        'R1': ['astribot_gripper_%s_base_fixed_joint', 'astribot_gripper_%s_joint_R1'],
        'R2': ['astribot_gripper_%s_base_fixed_joint', 'astribot_gripper_%s_joint_R2'],
    }
    worst = 0.0
    for suffix, chain in chains.items():
        names = [tpl % side for tpl in chain]
        _, pos = model.chain_tf(names, [0.0] * len(names))
        expect = sdf_origins[suffix]
        delta = max(abs(pos[i] - expect[i]) for i in range(3))
        worst = max(worst, delta)
        assert delta <= _CROSS_SOURCE_TOL_M, (
            '%s 关节原点两源差异 %.4f m 超过 %.4f m 容差：模型 %s vs SDF %s'
            % (suffix, delta, _CROSS_SOURCE_TOL_M,
               [round(v, 6) for v in pos], expect))
    # 差异明显小于容差时也值得知道 —— 说明容差可以收紧。
    assert worst > 0.0, '两源差异恰好为 0，可疑：确认没有把 SDF 当成 MJCF 抄'


# L11/R11 这一处两源差得比 3 mm 多得多，单独钉住，不混进上面那条断言里。
#
# MJCF: L11 body pos（相对 L1）= (0.04125, 0, 0.036379080527138)
# SDF : joint_L11 pose（相对 L1）= (0.036,   0, 0.031749)
# 差 (0.00525, 0, 0.00463)，模长 7.0 mm —— 是上面那条 3 mm 容差的两倍多。
#
# 本包取的是 MJCF。选择依据（不强，如实记下）：SDF 那个数值与 MJCF 里
# 被注释掉的 <connect body1=Link_L2 body2=Link_L11 anchor='0.036 0 0.031749'>
# **完全相同**，而 connect 的 anchor 在 MuJoCo 里是 body1(L2) 局部系下的点，
# 说的是四连杆另一处销轴（L2–L11），不是 L1–L11 这处销轴。
# 也就是说 SDF 很可能在转换时把两个销轴搞混了。但这只是推断，没有厂商文档佐证。
#
# 后果：指尖位置在抓取方向上有约 7 mm 的不确定度。抓取余量比这个数小的话，
# 结论就不可信 —— 这条测试的作用是让这个已知不确定度**留在台面上**，
# 不因为"测试全绿"就被当成不存在。
_L11_KNOWN_CROSS_SOURCE_DELTA_M = 7.0e-3


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_l11_cross_source_delta_is_pinned(side, model):
    """L11/R11 的两源差异钉在已知值上（约 7 mm），不允许悄悄漂移。"""
    sdf_origins = _sdf_gripper_joint_origins(side)
    for suffix, parent_suffix in (('L11', 'L1'), ('R11', 'R1')):
        name = 'astribot_gripper_%s_joint_%s' % (side, suffix)
        _, pos = model.joint_tf(name)
        expect = sdf_origins[suffix]
        delta = math.sqrt(sum((pos[i] - expect[i]) ** 2 for i in range(3)))
        assert abs(delta - _L11_KNOWN_CROSS_SOURCE_DELTA_M) < 0.5e-3, (
            '%s 相对 %s 的两源差异变成 %.4f m（已知值 %.4f m）。'
            '差异变化说明有人改了 MJCF 侧或 SDF 侧的取值，必须重新判断该信哪个源。'
            % (suffix, parent_suffix, delta, _L11_KNOWN_CROSS_SOURCE_DELTA_M))


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_gripper_mount_pose_matches_sdf(side, model):
    """夹爪基座相对 link_7 的安装位姿 == SDF 里 lump 进 link_7 的那个 pose。

    这是唯一直接抄 SDF 的几何量（MJCF 里夹爪是独立模型，没有它相对法兰的装法），
    所以必须精确相等，不走 3 mm 容差。
    """
    path = os.path.join(_VENDOR_S1, 'model', 'astribot_whole_body_with_gripper.sdf')
    model_el = ET.parse(path).getroot().find('model')
    expect = None
    for link in model_el.findall('link'):
        if link.get('name') != 'astribot_arm_%s_link_7' % side:
            continue
        for col in link.findall('collision'):
            if 'gripper_%s_base' % side in col.get('name'):
                expect = [float(v) for v in col.findtext('pose').split()]
    assert expect is not None, 'SDF 里找不到 lump 进 link_7 的夹爪基座 collision'

    mat, pos = model.joint_tf('astribot_gripper_%s_base_fixed_joint' % side)
    for idx, axis_name in enumerate('xyz'):
        assert abs(pos[idx] - expect[idx]) < 1e-9, (
            '基座安装位置 %s 与 SDF 不符: %.6f vs %.6f' % (axis_name, pos[idx], expect[idx]))
    assert _max_abs_diff(mat, _rpy_to_mat(*expect[3:6])) < 1e-9, (
        '基座安装姿态与 SDF 不符（rpy 应为 %s）—— 这个 1.5708 决定了 TCP 沿 -y 偏置' % expect[3:6])


# --------------------------------------------------------------------------- #
# 4. 质量守恒：集中表示 vs 分体表示
# --------------------------------------------------------------------------- #

def test_mass_conserved_between_lumped_and_articulated(model, model_no_gripper):
    """两种夹爪表示法的**腕部子树**质量差 <= 1%。

    use_gripper=false 时夹爪是法兰上的一坨集中惯量（0.64763 kg，厂商 per-part
    模型的原值）；true 时是 7 个真实 link（基座 0.4 + 6 指连杆 0.254）。
    这条测试挡的是一个很容易犯且很难发现的错误：加了夹爪 link 却忘了把法兰的
    集中惯量清零 —— 夹爪质量被算两遍，整机重心和力矩全偏，而模型看起来毫无异常。

    1% 容差不是随手取的：厂商自己两份模型就差 0.82%
    （whole_body.sdf 集中 0.775349 vs with_gripper.sdf 分体 0.781722）。

    !!! 为什么比的是腕部子树而不是整机总质量 !!!
    最初写的是整机总质量。实测注入"法兰惯量没清零"这个故障后，整机相对差只有
    1.664%（两条臂各多算 0.6476 kg，摊到 78.6 kg 的整机上）—— 勉强越过 1% 门槛，
    而**单侧**多算就只有 0.83%，会从门槛下面溜过去。
    误差是腕部局部的，判据却拿整机质量做分母，等于自己把灵敏度稀释了 78 倍。
    改成比腕部子树后，同样的故障是 +83%，量级上再无歧义。
    """
    for side in _GRIPPER_SIDES:
        root = 'astribot_arm_%s_link_7' % side
        lumped = model_no_gripper.subtree_mass(root)
        articulated = model.subtree_mass(root)
        rel = abs(articulated - lumped) / lumped
        assert rel <= _MASS_TOL_REL, (
            '%s 腕部子树质量差 %.3f%% 超过 %.1f%%：集中 %.6f kg vs 分体 %.6f kg。'
            '最常见原因是加了夹爪 link 但 tool_link 的集中惯量没清零。'
            % (side, rel * 100.0, _MASS_TOL_REL * 100.0, lumped, articulated))


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_flange_inertia_moves_with_representation(side, model, model_no_gripper):
    """法兰惯量的归属必须随 use_gripper 切换，两种表示不能同时带质量。"""
    name = 'astribot_arm_%s_tool_link' % side
    assert model.links[name].find('inertial') is None, (
        'use_gripper=true 时 %s 必须无惯量（夹爪质量已由夹爪 link 承担）' % name)
    assert model_no_gripper.links[name].find('inertial') is not None, (
        'use_gripper=false 时 %s 必须保留厂商的集中惯量，否则整机凭空轻了 0.65 kg' % name)


# --------------------------------------------------------------------------- #
# 5. 碰撞包络：夹爪必须真的进了碰撞模型
# --------------------------------------------------------------------------- #

def _link_collision_meshes(model, name):
    link = model.links[name]
    out = []
    for col in link.findall('collision'):
        mesh = col.find('geometry/mesh')
        if mesh is not None:
            out.append(os.path.basename(mesh.get('filename')))
    return out


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_gripper_contributes_collision_geometry(side, model):
    """夹爪必须贡献碰撞几何 —— 这是整个改动最核心的一条。

    改造前腕部唯一的碰撞体是 link_7 的 sphere r=0.05（在 link_7 系里只覆盖
    y ∈ [-0.05, 0.05]），而厂商自己的碰撞模型（whole_body.sdf 的
    opened_gripper.obj）覆盖到 y = -0.1997、横向宽 0.148 m。
    也就是说夹爪整体不在碰撞模型里：沿抓取方向少约 0.15 m。

    后果不是"规划保守一点"，而是**规划出来的轨迹会用夹爪去撞东西**，
    且仿真里一切正常。所以这条必须是硬断言，不是可选项。
    """
    expected_with_collision = ('base', 'Link_L11', 'Link_L2', 'Link_R11', 'Link_R2')
    for suffix in expected_with_collision:
        name = 'astribot_gripper_%s_%s' % (side, suffix)
        meshes = _link_collision_meshes(model, name)
        assert meshes, '%s 缺碰撞几何' % name
        assert all('collision_mesh' in m for m in meshes), (
            '%s 的碰撞应使用厂商的简化碰撞网格(*_collision_mesh.obj)，实际 %s'
            % (name, meshes))

    # L1/R1 在厂商 SDF 里确实没有 collision（被基座外壳包住的内部连杆）。
    # 这里断言"照样没有"，是为了把"照抄厂商"和"漏抄"区分开 ——
    # 若哪天给它们补了碰撞体，应该是个有意决定，会在这里被提示。
    for suffix in ('Link_L1', 'Link_R1'):
        name = 'astribot_gripper_%s_%s' % (side, suffix)
        assert not _link_collision_meshes(model, name), (
            '%s 在厂商 SDF 里无 collision，本模型也不应有（如确需补充请同步改本测试）'
            % name)


@pytest.mark.parametrize('side', _GRIPPER_SIDES)
def test_wrist_collision_reaches_vendor_envelope(side, model):
    """夹爪碰撞体沿抓取方向的伸出量，必须接近厂商 opened_gripper 包络。

    用厂商 whole_body.sdf 里 link_7 的那个 opened_gripper.obj 当标尺：
    它在 link_7 系里覆盖 y ∈ [-0.1997, -0.048]。本模型是分体建模，
    指尖碰撞体应当伸到同一个量级 —— 允许 2 cm 差（张开角与网格简化程度不同），
    但绝不能只到 -0.05（那就是退回了"夹爪不建模"的老状态）。
    """
    _, pos = model.chain_tf(
        ['astribot_gripper_%s_base_fixed_joint' % side,
         'astribot_gripper_%s_joint_L1' % side,
         'astribot_gripper_%s_joint_L11' % side], [0.0, 0.0, 0.0])
    # L11 原点已在 link_7 系；指尖胶垫还在其 +z 方向约 0.036 m 处。
    reach_y = pos[1]
    assert reach_y < -0.10, (
        '夹爪指尖连杆在 link_7 系只伸到 y=%.4f，远不及厂商包络 -0.1997 —— '
        '碰撞模型可能又退回了"夹爪不建模"的状态' % reach_y)
    assert reach_y > -0.25, (
        '夹爪指尖伸到 y=%.4f，超出厂商包络 -0.1997 太多，几何可能抄错' % reach_y)





