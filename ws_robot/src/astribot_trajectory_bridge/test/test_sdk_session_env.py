#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``sdk_session`` 里**不需要真机也能测**的那部分：环境自检与打包缺陷的绕行。

为什么这些分支值得单独测
======================
它们全部只在"SDK 起不来"的路径上执行，也就是**最容易被写错又最难被发现**的地方：
写错了不会有人察觉，直到某天真机上 import 失败、而错误提示把人指向错误的方向。
本项目已经为此付过一次代价 —— 曾长期把 pinocchio 版本共存误报成 ABI 冲突，
错结论扩散进 5 处注释和多份文档（见 memory: pinocchio-versions-coexist-by-soname）。

所以这里把两件事钉死：
1. 缺失 pip 包的探测必须**只报真缺的**，且不能因为 import 有副作用而误判；
2. 失败提示里**不许再出现 pinocchio ABI 冲突**这类已被证伪的说法。

``prewarm_robotics_library_py`` 依赖厂商 SDK 是否在场，所以只断言它的**契约**
（永不抛异常、返回 bool），不断言返回值 —— 那取决于环境。
"""

import os

import pytest

from astribot_trajectory_bridge.sdk_session import (
    _import_failure_hints,
    _sdk_root_guess,
    missing_pip_packages,
    prewarm_robotics_library_py,
)


# ---------------------------------------------------------------- 缺包探测

def test_missing_pip_packages_reports_only_absent_ones():
    """真实存在的包不许被报成缺失。"""
    # os / sys 一定在；用它们做"必然存在"的样本
    assert missing_pip_packages(('os', 'sys')) == []


def test_missing_pip_packages_detects_absent():
    fake = 'zz_definitely_not_installed_pkg_9f3a'
    assert missing_pip_packages((fake,)) == [fake]


def test_missing_pip_packages_preserves_order_and_filters():
    fake_a = 'zz_absent_a_9f3a'
    fake_b = 'zz_absent_b_9f3a'
    got = missing_pip_packages((fake_a, 'os', fake_b))
    assert got == [fake_a, fake_b]


def test_missing_pip_packages_empty_input():
    assert missing_pip_packages(()) == []


def test_filterpy_is_installed():
    """filterpy 是厂商 SDK import 链的硬依赖（util.py -> whole_body_control.py:3123）。

    缺它时报错是 ``No module named 'meta'`` —— 一个**完全误导**的名字，
    真因要再往里挖一层才看得到。所以这里直接把它钉成一条断言。
    """
    assert missing_pip_packages(('filterpy',)) == [], (
        '缺 filterpy。必须用 --no-deps 安装，否则会顶掉本项目钉住的 numpy 1.21.5：'
        'python3 -m pip install --no-deps filterpy')


def test_numpy_pin_is_intact():
    """numpy 必须停在 1.21.5：装 filterpy/opencv 时极易被依赖解析顶掉。"""
    numpy = pytest.importorskip('numpy')
    assert numpy.__version__ == '1.21.5', (
        'numpy 被顶到了 %s，本项目钉的是 1.21.5' % numpy.__version__)


# ------------------------------------------------------- 失败提示的内容约束

def test_hints_never_blame_pinocchio_abi():
    """!!! 回归锁 !!! 失败提示里不许再出现已被证伪的 pinocchio ABI 冲突说法。

    厂商 .so 的 DT_NEEDED 烧死的是 libpinocchio_default.so.3.7.0，仓库
    third_party 自带该版本；系统 apt 的 4.0.0 **soname 不同**，永不参与解析。
    两版共存不是冲突，`ldd` 全部解析成功、零个 not found。
    """
    text = _import_failure_hints()
    lowered = text.lower()
    assert 'abi' not in lowered, '提示里又出现了 ABI 冲突的说法：%s' % text
    # 允许出现 pinocchio 这个词（比如将来写"与 pinocchio 无关"），
    # 但不许把它和"冲突/阻塞"绑在一起。
    for bad in ('pinocchio 冲突', 'pinocchio abi', '主版本'):
        assert bad not in lowered, '提示里出现了被证伪的说法 %r：%s' % (bad, text)


def test_hints_are_actionable():
    """提示必须给出可执行动作，而不是只描述现象。"""
    text = _import_failure_hints()
    assert 'env.sh' in text
    # 缺包时必须带上 --no-deps 的告警（否则会顶掉 numpy 钉版）
    if missing_pip_packages():
        assert '--no-deps' in text


def test_hints_mention_core_common_when_absent_from_pythonpath(monkeypatch):
    """PYTHONPATH 缺 core/common 时必须点出来 —— 这是第一层遮蔽的直接原因。"""
    monkeypatch.setenv('ASTRIBOT_SDK_ROOT', '/fake/sdk/root')
    monkeypatch.setenv('PYTHONPATH', '/some/unrelated/path')
    text = _import_failure_hints()
    assert os.path.join('/fake/sdk/root', 'astribot_sdk', 'core', 'common') in text
    assert 'robotics_library_py' in text


def test_hints_skip_core_common_when_present(monkeypatch):
    """已经在 PYTHONPATH 上时不许再报 —— 假告警会让人怀疑正确的配置。"""
    root = '/fake/sdk/root'
    common = os.path.join(root, 'astribot_sdk', 'core', 'common')
    monkeypatch.setenv('ASTRIBOT_SDK_ROOT', root)
    monkeypatch.setenv('PYTHONPATH', os.pathsep.join([common, '/other']))
    text = _import_failure_hints()
    assert common not in text


def test_hints_survive_missing_sdk_root(monkeypatch):
    """ASTRIBOT_SDK_ROOT 没设时不许抛 —— 生成提示的过程本身不能再制造异常。"""
    monkeypatch.delenv('ASTRIBOT_SDK_ROOT', raising=False)
    monkeypatch.delenv('PYTHONPATH', raising=False)
    text = _import_failure_hints()
    assert isinstance(text, str) and text


def test_sdk_root_guess_defaults_to_empty(monkeypatch):
    monkeypatch.delenv('ASTRIBOT_SDK_ROOT', raising=False)
    assert _sdk_root_guess() == ''


def test_sdk_root_guess_reads_env(monkeypatch):
    monkeypatch.setenv('ASTRIBOT_SDK_ROOT', '/x/y')
    assert _sdk_root_guess() == '/x/y'


# --------------------------------------------------------------- 预热的契约

def test_prewarm_never_raises_and_returns_bool():
    """预热失败必须**静默返回 False**，不许抛。

    真正的报错要留给后面那次真实 import 去报（那里的上下文更完整）。
    这里抛异常会把"缺 pip 包"误报成"预热失败"，把人带偏一层。
    """
    got = prewarm_robotics_library_py()
    assert isinstance(got, bool)


def test_prewarm_is_idempotent():
    """会被节点重复调用（多次 open_session），必须可重入。"""
    a = prewarm_robotics_library_py()
    b = prewarm_robotics_library_py()
    assert a == b
