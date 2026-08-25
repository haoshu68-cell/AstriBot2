#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图来源配置的离线测试：组合矩阵 + 字段健壮性。

不需要 ROS 运行时、不需要仿真、不需要真机。
"""

import os
import tempfile

import pytest

from astribot_s1_perception.map_source_config import (
    MapSourceConfigError,
    check_live_transport_env,
    load_config,
    needs_slam_toolbox,
    require,
    resolve,
    validate_combination,
)


def write_config(**params):
    """写一份临时配置文件，返回路径。"""
    lines = ['map_provider:', '  ros__parameters:']
    for key, value in params.items():
        if isinstance(value, str):
            lines.append(f'    {key}: "{value}"')
        elif isinstance(value, bool):
            lines.append(f'    {key}: {"true" if value else "false"}')
        elif isinstance(value, (list, tuple)):
            lines.append(f'    {key}: [{", ".join(str(v) for v in value)}]')
        else:
            lines.append(f'    {key}: {value}')
    handle = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, encoding='utf-8')
    handle.write('\n'.join(lines) + '\n')
    handle.close()
    return handle.name


# ---------------------------------------------------------------------------
# 一、组合矩阵
#
# 这是整套配置最容易写错的地方，而错了的表现极隐蔽：
# sim_slam + ground_truth 会让 slam_toolbox 与静态 TF 同时发 map→odom，
# 症状是位姿反复跳，看起来像"定位漂移"。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize('map_source,localization', [
    ('sim_slam', 'slam'),
    ('real_file', 'ground_truth'),
    ('real_live', 'ground_truth'),
])
def test_valid_combinations_accepted(map_source, localization):
    assert validate_combination(map_source, localization) is None


def test_sim_slam_with_ground_truth_rejected():
    reason = validate_combination('sim_slam', 'ground_truth')
    assert reason is not None
    # 拒绝原因必须指出"两个父源"这个真实后果，而不只是说"不允许"
    assert '两个父源' in reason


@pytest.mark.parametrize('map_source', ['real_file', 'real_live'])
def test_real_with_slam_rejected_as_route_a(map_source):
    reason = validate_combination(map_source, 'slam')
    assert reason is not None
    assert '路线 A' in reason
    # 必须给出可执行的下一步，而不是只说不行
    assert 'ground_truth' in reason


@pytest.mark.parametrize('bad', ['', 'sim', 'SIM_SLAM', 'real', None])
def test_unknown_map_source_rejected(bad):
    assert validate_combination(bad, 'slam') is not None


@pytest.mark.parametrize('bad', ['', 'amcl', 'GROUND_TRUTH', None])
def test_unknown_localization_rejected(bad):
    assert validate_combination('sim_slam', bad) is not None


def test_needs_slam_toolbox_only_for_sim_slam():
    assert needs_slam_toolbox('sim_slam') is True
    assert needs_slam_toolbox('real_file') is False
    assert needs_slam_toolbox('real_live') is False


# ---------------------------------------------------------------------------
# 二、配置读取与覆盖
# ---------------------------------------------------------------------------
def test_load_config_reads_yaml():
    path = write_config(map_source='real_file', localization='ground_truth',
                        map_yaml_path='/tmp/x.yaml')
    try:
        _, params = load_config(path)
        assert params['map_source'] == 'real_file'
        assert params['map_yaml_path'] == '/tmp/x.yaml'
    finally:
        os.unlink(path)


def test_empty_override_does_not_clobber_yaml():
    """留空的覆盖项必须被忽略。

    这条规则是踩过坑的：给命令行参数设非空默认值时，它会无条件盖掉 yaml 里的
    正确值，而且从日志里看不出来是被盖了。
    """
    path = write_config(map_source='real_file', localization='ground_truth')
    try:
        _, params = load_config(path, overrides={'map_source': '', 'localization': None})
        assert params['map_source'] == 'real_file'
        assert params['localization'] == 'ground_truth'
    finally:
        os.unlink(path)


def test_non_empty_override_wins():
    path = write_config(map_source='sim_slam', localization='slam')
    try:
        _, params = load_config(path, overrides={'map_source': 'real_file',
                                                 'localization': 'ground_truth'})
        assert params['map_source'] == 'real_file'
        assert params['localization'] == 'ground_truth'
    finally:
        os.unlink(path)


def test_missing_file_reports_path():
    with pytest.raises(MapSourceConfigError) as excinfo:
        load_config('/definitely/not/here.yaml')
    assert '/definitely/not/here.yaml' in str(excinfo.value)


def test_wrong_structure_reports_expected_shape():
    handle = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, encoding='utf-8')
    handle.write('some_other_node:\n  ros__parameters:\n    map_source: "sim_slam"\n')
    handle.close()
    try:
        with pytest.raises(MapSourceConfigError) as excinfo:
            load_config(handle.name)
        assert 'map_provider' in str(excinfo.value)
    finally:
        os.unlink(handle.name)


# ---------------------------------------------------------------------------
# 三、require() 的类型与缺项报错要指向配置项，不能是 KeyError/TypeError
# ---------------------------------------------------------------------------
def test_require_missing_key_names_the_key():
    with pytest.raises(MapSourceConfigError) as excinfo:
        require({}, 'relay_timeout_sec', float)
    assert 'relay_timeout_sec' in str(excinfo.value)


def test_require_uses_default_when_absent():
    assert require({}, 'relay_timeout_sec', float, 30.0) == 30.0


def test_require_bad_type_names_the_key_and_value():
    with pytest.raises(MapSourceConfigError) as excinfo:
        require({'remote_domain_id': 'not_an_int'}, 'remote_domain_id', int)
    message = str(excinfo.value)
    assert 'remote_domain_id' in message
    assert 'not_an_int' in message


# ---------------------------------------------------------------------------
# 四、resolve() 端到端：非法组合必须在这一层就抛，不能返回"凑合能跑"的结果
# ---------------------------------------------------------------------------
def test_resolve_rejects_invalid_combination_with_config_path():
    path = write_config(map_source='sim_slam', localization='ground_truth')
    try:
        with pytest.raises(MapSourceConfigError) as excinfo:
            resolve(path)
        message = str(excinfo.value)
        assert path in message           # 要指出是哪份配置文件
        assert 'sim_slam' in message
        assert 'ground_truth' in message
    finally:
        os.unlink(path)


def test_resolve_returns_effective_values():
    path = write_config(map_source='real_file', localization='ground_truth',
                        map_yaml_path='/tmp/m.yaml')
    try:
        config_path, params, map_source, localization = resolve(path)
        assert config_path == path
        assert map_source == 'real_file'
        assert localization == 'ground_truth'
        assert params['map_yaml_path'] == '/tmp/m.yaml'
    finally:
        os.unlink(path)


# ---------------------------------------------------------------------------
# 五、real_live 对 ROS_LOCALHOST_ONLY 的要求
#
# 这条是拿实测打脸换来的：原设计以为"只让中继进程 =0，其余保持 1"就能既跨机
# 又隔离。实测 localhost_only=1 与 =0 的参与者互相发现不了（同机同域也不行），
# 于是中继会"成功发布"到没人听的地方，nav2 一直等 /map —— 静默失败。
# ---------------------------------------------------------------------------
def test_live_rejects_localhost_only_one():
    reason = check_live_transport_env('1')
    assert reason is not None
    # 必须解释"为什么"而不只是"不行"，否则下一个人会改回去
    assert '互相发现不了' in reason
    # 必须给出可执行的改法
    assert 'ROS_LOCALHOST_ONLY=0' in reason


@pytest.mark.parametrize('value', ['0', 0, None, ''])
def test_live_accepts_localhost_only_off_or_unset(value):
    assert check_live_transport_env(value) is None
