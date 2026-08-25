#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图来源配置的读取与校验。

为什么单独一个模块而不是写在 launch 里：这份逻辑有**两个** launch 需要
（`map_provider.launch.py` 要按它分发，`perception_slam_bringup.launch.py` 要按它
决定起不起 slam_toolbox）。抄两份的话，两边的判定迟早漂移 —— 而漂移的后果是
"slam_toolbox 和静态 TF 同时发 map→odom"，症状看起来像定位漂移，极难归因。

放在这里的另一个好处是能离线单测：组合矩阵与字段校验是这套配置最容易写错的地方，
而它们的错误都不会在启动时立刻可见。
"""

import os

import yaml


VALID_MAP_SOURCES = ('sim_slam', 'real_file', 'real_live')
VALID_LOCALIZATION = ('slam', 'ground_truth')

#: 需要 slam_toolbox 在线建图的地图来源
SLAM_TOOLBOX_SOURCES = ('sim_slam',)


class MapSourceConfigError(RuntimeError):
    """配置非法。在 launch 阶段抛出，让它带着明确原因停在启动前。"""


def default_config_path():
    """包内默认配置路径。放在函数里而不是模块常量，避免 import 时就要求包已安装。"""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('astribot_s1_perception'),
        'config', 'map_source.yaml')


def load_config(path=None, overrides=None):
    """读 yaml 并用**非空**的覆盖项覆盖。

    overrides 里值为 None 或空字符串的键一律忽略 —— 这条规则是踩过坑的：
    给命令行参数设非空默认值时，它会无条件盖掉 yaml 里的正确值，
    而且从日志里看不出来是被盖了。
    """
    path = path or default_config_path()
    if not os.path.isfile(path):
        raise MapSourceConfigError(f'找不到地图来源配置文件: {path}')

    with open(path, 'r', encoding='utf-8') as handle:
        raw = yaml.safe_load(handle) or {}
    try:
        params = dict(raw['map_provider']['ros__parameters'])
    except (KeyError, TypeError):
        raise MapSourceConfigError(
            f'{path} 结构不对：期望顶层 map_provider.ros__parameters')

    for key, value in (overrides or {}).items():
        if value not in (None, ''):
            params[key] = value

    return path, params


def require(params, key, cast, default=None):
    """取一个必填/可选项并转型，失败给出指向配置项的错误而不是 KeyError。"""
    if key not in params or params[key] is None:
        if default is not None:
            return default
        raise MapSourceConfigError(f'配置缺少必填项 {key}')
    try:
        return cast(params[key])
    except (TypeError, ValueError) as exc:
        raise MapSourceConfigError(
            f'配置项 {key} 类型不对: {params[key]!r} ({exc})')


def validate_combination(map_source, localization):
    """校验两个轴的组合。合法返回 None，否则返回拒绝原因字符串。

    合法组合只有三个：
        sim_slam  + slam          （今天的行为；探索算法只能在这个组合下开发）
        real_file + ground_truth  （默认；在真机地图上验证规划/导航/搬运）
        real_live + ground_truth  （同上，地图跟着真机实时更新）
    """
    if map_source not in VALID_MAP_SOURCES:
        return f'map_source={map_source!r} 非法，只能是 {VALID_MAP_SOURCES}'
    if localization not in VALID_LOCALIZATION:
        return f'localization={localization!r} 非法，只能是 {VALID_LOCALIZATION}'

    if map_source == 'sim_slam' and localization == 'ground_truth':
        return ('sim_slam + ground_truth 无意义且有害：既然在建图，map→odom 本来就由 '
                'slam_toolbox 发布；再加一条静态 TF 会让同一个子帧有两个父源，'
                '位姿反复跳，而症状看起来像"定位漂移"，极难归因。'
                '要用真值定位就把地图源改成 real_file / real_live。')

    if map_source in ('real_file', 'real_live') and localization == 'slam':
        return ('real_* + slam 是路线 A，暂未开放：扫描匹配要求**仿真几何与地图一致**，'
                '而当前 Gazebo 跑的是 AWS 仓库、地图来自真实场地，'
                '不匹配时定位必然发散（表现为位姿乱跳、代价地图错位）。'
                '需要它得先做"从占据栅格挤出 Gazebo 世界"的工具。'
                '现在请用 localization:=ground_truth。')

    return None


def resolve(path=None, overrides=None):
    """读配置 + 校验组合，返回 (配置路径, params, map_source, localization)。

    组合非法直接抛 MapSourceConfigError —— 不返回一个"凑合能跑"的结果。
    """
    config_path, params = load_config(path, overrides)
    map_source = require(params, 'map_source', str)
    localization = require(params, 'localization', str)

    reason = validate_combination(map_source, localization)
    if reason is not None:
        raise MapSourceConfigError(
            f'地图来源组合被拒绝（配置文件 {config_path}）：\n'
            f'  map_source={map_source} localization={localization}\n'
            f'  {reason}')

    return config_path, params, map_source, localization


def needs_slam_toolbox(map_source):
    """这个地图来源是否需要 slam_toolbox 在线建图。"""
    return map_source in SLAM_TOOLBOX_SOURCES


def check_live_transport_env(localhost_only_value):
    """`real_live` 模式对 ROS_LOCALHOST_ONLY 的要求。合法返回 None，否则返回原因。

    ================== 这条校验是拿实测打脸换来的 ==================
    最初的设计是"只让中继进程 ROS_LOCALHOST_ONLY=0，其余节点保持 1，
    这样只有中继能上网"。**实测证明这行不通**：

        domain 42 订阅端 ROS_LOCALHOST_ONLY=1 -> 收不到中继发的 /map
        domain 42 订阅端 ROS_LOCALHOST_ONLY=0 -> 收得到

    `localhost_only=1` 与 `=0` 的 DDS 参与者**互相发现不了**，即使同机同域。
    于是中继会"成功发布"到一个没人听的地方，而 nav2 一直等 /map ——
    典型的静默失败。

    修正后的隔离依据（同一轮实测顺带验证过的）：**隔离来自 domain，不来自
    localhost_only**。实测中假真机在 domain 25 上的私有话题 `/fake_robot_only`
    在 domain 42 上**两种 localhost_only 取值下都不可见**。
    所以 real_live 的安全性由两点保证：
      1. 真机在 domain 25，本机栈在 domain 42，跨域不可见；
      2. 中继是唯一的跨域参与者，而且只**单向**搬 /map ——
         本机的 /cmd_vel 之类没有任何通路能到真机。
    """
    if str(localhost_only_value) == '1':
        return ('map_source=real_live 需要 ROS_LOCALHOST_ONLY=0，当前是 1。\n'
                '  原因：localhost_only=1 与 =0 的 DDS 参与者互相发现不了'
                '（同机同域也不行，已实测）。保持 1 的后果是中继"成功发布"到'
                '没人听的地方，而 nav2 一直等 /map —— 静默失败。\n'
                '  安全性不依赖这个开关：真机在 remote_domain_id、本机栈在 '
                'local_domain_id，跨域本来就互不可见（已实测），而且中继只单向搬 /map，'
                '本机的 /cmd_vel 没有通路到真机。\n'
                '  改法：整条栈用 ROS_LOCALHOST_ONLY=0 启动，并确认 '
                'local_domain_id 与真机的 domain 不同。')
    return None
