#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨包**配置自洽性**测试（离线，不需要仿真）。

为什么需要这一份
================
三段式跟踪的纯函数层有 36 条测试 + 故障注入，但本轮实测踩到的四个坑
**全部在配置层**，那里此前没有任何自动校验：

  1. 内层 MPPI 只手抄了 15 行、漏掉 10 个 critics —— MPPI 的代价函数全部来自
     critics，漏了 PathFollowCritic/PathAlignCritic 就没有东西把机器人往路径上
     拉。现象：起步对齐正常、机器人贴在路径上，但指令 vx 均值 -0.021(往后)、
     剩余距离来回晃、Failed to make progress。
  2. 探索场景刻意跳过终点旋转，但 Nav2 的 SimpleGoalChecker **同时**卡位置和
     朝向。实测位置 0.165m 已满足 0.18，朝向却差 94.2° -> goal checker 永不满足
     -> FollowPath 不成功。只改了协调器的 check_yaw、漏了 Nav2 这一层。
  3. goal_checker_plugins 列两项时，FollowPath 的 goal_checker_id 端口**无默认值**，
     默认行为树传空字符串 -> 每次 FollowPath 直接 abort。现象是"机器人完全不跟踪
     路径 + 反复 spin/backup"，而真因只在 ~/.ros/log/controller_server_*.log 里。
  4. goal_clearance_radius 被按"必须等于 xy_goal_tolerance"向下调到 0.18/0.10，
     而 robot_radius=0.42 意味着距障碍 0.42m 内全是 253(inscribed lethal)，
     于是目标点必然落在致命带里，机器人开过去后再也规划不出来。
     实测 clearance=0.25 时 lethal 0 次，=0.18 时 2159 次。

这四条各对应下面一个测试类。全部只读 yaml/xml，不起节点。
"""

import glob
import math
import os
import re

import pytest
import yaml

# ---------------------------------------------------------------- 定位源码树

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, '..', '..'))     # .../ws_robot/src

NAV_CFG_DIR = os.path.join(_SRC, 'astribot_s1_navigation', 'config')
NAV_BT_DIR = os.path.join(_SRC, 'astribot_s1_navigation', 'behavior_trees')
AUTONOMY_CFG = os.path.join(
    _SRC, 'astribot_s1_autonomy', 'config', 'exploration_coordinator_params.yaml')
# 自举的两条关键耦合分别落在另外两个包里，必须从源头读，不能在测试里抄常数。
SLAM_MAPPER_CFG = os.path.join(
    _SRC, 'astribot_s1_perception', 'config', 'mapper_params_online_async.yaml')
CHASSIS_CFG = os.path.join(
    _SRC, 'astribot_s1_chassis_effort_drive', 'config', 'omni_effort_drive_params.yaml')
NAV_BRINGUP_LAUNCH = os.path.join(
    _SRC, 'astribot_s1_navigation', 'launch', 'nav2_full_bringup.launch.py')
COUPLING_NODE = os.path.join(
    _SRC, 'astribot_s1_dynamics_coupling', 'astribot_s1_dynamics_coupling',
    'arm_chassis_speed_coupling_node.py')
# 写通路的 /scan 联锁在另一个包里。costmap 的告警阈值必须比它宽松，
# 否则会出现"底盘已停车而 costmap 零告警"的组合，见
# TestObservationStalenessDetection.test_rate_slower_than_bridge_interlock。
BRIDGE_NODE = os.path.join(
    _SRC, 'astribot_trajectory_bridge', 'astribot_trajectory_bridge',
    'chassis_cmd_bridge_node.py')

NAV_PARAM_FILES = sorted(glob.glob(os.path.join(NAV_CFG_DIR, 'nav2_params_*.yaml')))

THREE_PHASE_PLUGIN = 'astribot_s1_path_tracking::ThreePhaseController'


def _load(path):
    with open(path, encoding='utf-8') as fh:
        return yaml.safe_load(fh)


def _controller_params(path):
    return _load(path)['controller_server']['ros__parameters']


def _coordinator_params():
    """协调器 yaml 用的是 `/**` 通配键，不是节点名。

    这一点本身是个坑：`/**:` 文件加载时不会报错，所以写错节点名时所有参数
    都静默回落到声明默认值。这里显式按通配键读，并断言它存在。
    """
    d = _load(AUTONOMY_CFG)
    assert '/**' in d, (
        f'{AUTONOMY_CFG} 的顶层键不是 `/**`，实际是 {list(d)}。'
        f'若改成了节点名，请同步改本测试；混用会让参数静默失效。')
    return d['/**']['ros__parameters']


def _robot_envelope_radius(path, which):
    """底盘外接半径。两份配置表示法不同：
      · rpp  用 robot_radius（标量）
      · mppi 用 footprint（多边形，取顶点到原点的最大距离）
    """
    p = _costmap_params(path, which)
    if p.get('robot_radius') is not None:
        return float(p['robot_radius'])
    fp = p.get('footprint')
    assert fp, f'{os.path.basename(path)} {which} 既没有 robot_radius 也没有 footprint'
    if isinstance(fp, str):
        fp = yaml.safe_load(fp)
    return max((x * x + y * y) ** 0.5 for x, y in fp)


def _inscribed_radius(path, which):
    """内切半径：膨胀层判 253 用的就是它。
      · rpp  圆形足迹 robot_radius -> 内切=外接=robot_radius
      · mppi 八边形 footprint      -> 各边到原点的最短距离
    两份文件几何不同，数值必然不同，所以不能抄常数。
    """
    p = _costmap_params(path, which)
    if p.get('robot_radius') is not None:
        return float(p['robot_radius'])
    fp = p.get('footprint')
    assert fp, f'{os.path.basename(path)} {which} 既无 robot_radius 也无 footprint'
    if isinstance(fp, str):
        fp = yaml.safe_load(fp)

    def edge_dist(a, b):
        ax, ay = a
        bx, by = b
        ex, ey = bx - ax, by - ay
        l2 = ex * ex + ey * ey
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, -(ax * ex + ay * ey) / l2))
        return math.hypot(ax + t * ex, ay + t * ey)

    return min(edge_dist(fp[i], fp[(i + 1) % len(fp)]) for i in range(len(fp)))


def _costmap_params(path, which):
    """which: 'global_costmap' | 'local_costmap'"""
    return _load(path)[which][which]['ros__parameters']


def _three_phase_instances(cp):
    """返回 {实例名: 参数字典}，只含 ThreePhaseController 的实例。"""
    out = {}
    for name in cp.get('controller_plugins', []):
        block = cp.get(name)
        if isinstance(block, dict) and block.get('plugin') == THREE_PHASE_PLUGIN:
            out[name] = block
    return out


def _chassis_cmd_vel_timeout():
    """底盘力矩闭环节点的 cmd_vel 超时(s)。自举的发送频率必须高于它的倒数。"""
    if not os.path.isfile(CHASSIS_CFG):
        return None
    for section in _load(CHASSIS_CFG).values():
        if isinstance(section, dict) and 'ros__parameters' in section:
            v = section['ros__parameters'].get('cmd_vel_timeout_sec')
            if v is not None:
                return float(v)
    return None


def _coupling_min_speed_scale():
    """臂-底盘耦合限速的下限系数。

    刻意从源码里读 declare_parameter 的默认值，而不是在测试里抄一个常数：
    抄常数意味着那个包改了默认值这条耦合会静默失效，而失效的表现是
    「自举有时静默转不动」——最难查的那一类间歇故障。
    """
    if not os.path.isfile(COUPLING_NODE):
        return None
    with open(COUPLING_NODE, encoding='utf-8') as fh:
        for line in fh:
            if 'min_speed_scale' in line and 'declare_parameter' in line:
                head = line.split('#')[0]
                start = head.find(',')
                end = head.rfind(')')
                if start < 0 or end < 0:
                    continue
                try:
                    return float(head[start + 1:end].strip())
                except ValueError:
                    return None
    return None


def _bridge_scan_max_age():
    """写通路 /scan 联锁的龄期阈值(s)。

    刻意从 chassis_cmd_bridge_node.py 的 declare_parameter 读，不在测试里抄常数：
    抄常数意味着那个包改了默认值后，这条耦合会**静默失效**，
    而失效表现是"底盘停车了但 costmap 一声不响"——最难查的那一类。
    """
    if not os.path.isfile(BRIDGE_NODE):
        return None
    with open(BRIDGE_NODE, encoding='utf-8') as fh:
        for line in fh:
            head = line.split('#')[0]
            if "'scan_max_age_sec'" not in head or 'd(' not in head:
                continue
            start = head.find(',')
            end = head.rfind(')')
            if start < 0 or end < 0:
                continue
            try:
                return float(head[start + 1:end].strip())
            except ValueError:
                return None
    return None


# 至少要能找到配置文件，否则后面所有测试都会"因为没数据而通过"。
def test_config_files_exist():
    assert NAV_PARAM_FILES, f'找不到任何 nav2_params_*.yaml，查找路径: {NAV_CFG_DIR}'
    assert os.path.isfile(AUTONOMY_CFG), f'找不到协调器配置: {AUTONOMY_CFG}'


def test_found_three_phase_instances():
    """哨兵：如果一个三段式实例都没找到，下面那些逐实例的测试会全部空转而"通过"。"""
    total = 0
    for f in NAV_PARAM_FILES:
        total += len(_three_phase_instances(_controller_params(f)))
    assert total > 0, (
        '没有在任何 nav2_params_*.yaml 里找到 ThreePhaseController 实例。'
        '若确实移除了三段式，请同时删掉本文件里对应的测试，不要留空转的测试。')


# ============================================================ 坑 1：inner critics

class TestInnerControllerCritics:
    """内层 MPPI 必须带齐 critics —— 它就是 MPPI 的代价函数。"""

    def test_inner_block_exists(self):
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            for name, block in _three_phase_instances(cp).items():
                assert 'inner' in block, (
                    f'{os.path.basename(f)} 的 {name} 没有 inner 段。'
                    f'三段式的跟踪段完全依赖内层控制器，缺了它只会原地转。')

    def test_inner_critics_present_and_nonempty(self):
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            for name, block in _three_phase_instances(cp).items():
                inner = block.get('inner', {})
                if 'mppi' not in str(inner.get('plugin', '')).lower():
                    continue                      # 非 MPPI 内层不适用本条
                critics = inner.get('critics')
                assert critics, (
                    f'{os.path.basename(f)} 的 {name}.inner 没有 critics。'
                    f'MPPI 的代价函数全部来自 critics，漏了它机器人会贴着路径原地漂移'
                    f'（实测：指令 vx 均值 -0.021、剩余距离来回晃、'
                    f'Failed to make progress）。')

    def test_inner_critics_match_outer_followpath(self):
        """内层 critics 必须与外层 FollowPath 同集合 —— 两处漂开时行为会不一致，
        而这种不一致只在切控制器那次才显形。"""
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            outer = cp.get('FollowPath', {})
            if 'mppi' not in str(outer.get('plugin', '')).lower():
                continue
            expected = set(outer.get('critics', []))
            assert expected, f'{os.path.basename(f)} 的 FollowPath 自己就没有 critics'
            for name, block in _three_phase_instances(cp).items():
                inner = block.get('inner', {})
                if 'mppi' not in str(inner.get('plugin', '')).lower():
                    continue
                got = set(inner.get('critics', []))
                assert got == expected, (
                    f'{os.path.basename(f)} 的 {name}.inner critics 与 FollowPath 不一致。\n'
                    f'  缺少: {sorted(expected - got)}\n'
                    f'  多出: {sorted(got - expected)}')

    def test_inner_critic_blocks_are_configured(self):
        """列进 critics 的每个 critic 都应有自己的参数块，否则它按默认值跑，
        而默认值与本项目调过的值可能差很远。"""
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            for name, block in _three_phase_instances(cp).items():
                inner = block.get('inner', {})
                for critic in inner.get('critics', []):
                    assert critic in inner, (
                        f'{os.path.basename(f)} 的 {name}.inner 列了 critic '
                        f'{critic} 但没有它的参数块')


# =================================================== 坑 2：跳过终点旋转 vs 朝向容差

class TestSkipGoalAlignImpliesLooseYaw:
    """align_goal_enabled=false 时，Nav2 的朝向容差必须宽到不约束。

    SimpleGoalChecker 同时卡位置和朝向；既然刻意不对齐终点姿态，
    朝向就永远差着（实测 94.2°），goal checker 永不满足。
    """

    LOOSE_YAW_MIN = 3.14        # ≈π，等于不约束

    def test_loose_yaw_when_any_instance_skips_goal_align(self):
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            skipping = [
                n for n, b in _three_phase_instances(cp).items()
                if b.get('align_goal_enabled') is False]
            if not skipping:
                continue
            checkers = cp.get('goal_checker_plugins', [])
            assert len(checkers) == 1, (
                '存在跳过终点对齐的实例时，本测试假定只有一个 goal checker；'
                '若改成多 checker + 显式 goal_checker_id，请更新本测试')
            gc = cp.get(checkers[0], {})
            yaw = gc.get('yaw_goal_tolerance')
            assert yaw is not None and yaw >= self.LOOSE_YAW_MIN, (
                f'{os.path.basename(f)}: {skipping} 跳过终点对齐，'
                f'但 {checkers[0]}.yaw_goal_tolerance = {yaw} 仍在卡朝向。\n'
                f'后果：位置达标而朝向永远不达标 -> goal checker 永不满足 -> '
                f'FollowPath 不成功 -> Failed to make progress -> 恢复行为 -> 超时。\n'
                f'实测过的现象就是「转完之后不跟路径、原地漂移」。')

    def test_coordinator_check_yaw_consistent_with_skipping(self):
        """协调器的 check_yaw 必须与"是否对齐终点姿态"一致。
        既然不对齐，就不能再去复核朝向 —— 否则每个目标都必然判失败。"""
        coord = _coordinator_params()
        any_skip = False
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            if any(b.get('align_goal_enabled') is False
                   for b in _three_phase_instances(cp).values()):
                any_skip = True
        if any_skip:
            assert coord.get('check_yaw') is False, (
                '存在跳过终点对齐的控制器实例，但协调器 check_yaw=true。'
                '既然不对齐终点姿态，复核朝向就会让每个探索目标都必然判失败。')


# ============================================= 坑 3：goal_checker_plugins 只能一项

class TestGoalCheckerSelection:
    """FollowPath 的 goal_checker_id 端口**无默认值**（controller_id 有）。

    列两项而调用方传空字符串时，controller_server 每次 FollowPath 直接 abort：
      FollowPath called with goal_checker name "" ... which does not exist.
    """

    def test_single_goal_checker_or_all_bts_specify_id(self):
        bt_files = sorted(glob.glob(os.path.join(NAV_BT_DIR, '*.xml')))
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            checkers = cp.get('goal_checker_plugins', [])
            assert checkers, f'{os.path.basename(f)} 没有配 goal_checker_plugins'
            if len(checkers) == 1:
                continue
            # 多个 checker 时，本仓库所有 BT 都必须显式传 goal_checker_id。
            # 注意这仍不能保证外部客户端（RViz 用默认树）不出问题。
            assert bt_files, (
                f'{os.path.basename(f)} 列了多个 goal checker '
                f'{checkers}，但仓库里没有任何自定义行为树 —— '
                f'默认树传空 goal_checker_id 会让每次 FollowPath 直接 abort')
            for bt in bt_files:
                text = open(bt, encoding='utf-8').read()
                if '<FollowPath' in text:
                    assert 'goal_checker_id' in text, (
                        f'{os.path.basename(bt)} 的 FollowPath 没有显式 goal_checker_id，'
                        f'而 {os.path.basename(f)} 配了多个 goal checker')

    def test_declared_checkers_have_param_blocks(self):
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            for name in cp.get('goal_checker_plugins', []):
                assert name in cp, (
                    f'{os.path.basename(f)} 声明了 goal checker {name} 但没有它的参数块')


# ==================================== 坑 4：净空半径的上下界（两侧都有悬崖）

class TestClearanceRadiusBounds:
    """xy_goal_tolerance <= goal_clearance_radius < robot_radius

    下界：容差圈内任何一点都可能是机器人的落点，那片区域必须自由。
    上界：robot_radius 以内的格子在 costmap 里是 253(inscribed lethal)，
          取到 robot_radius 会把几乎所有候选判成不可站（实测 6775/6784）。
    """


    # 实测下界。**不是**从几何推出来的，而是跑出来的：
    #   clearance = 0.25 -> lethal space 0 次
    #   clearance = 0.18 -> lethal space 2159 次（+67s 起），机器人开到目标后
    #                       再也规划不出来，探索停摆
    # 机制上说不出一个干净的解析下界：能否站得住取决于 validator 的
    # occupied_threshold 与 costmap 代价值的对应关系，而 253(inscribed) 那一档
    # 是否算"占据"随阈值而变。所以这里钉住实测安全值。
    # 上界仍是 robot_radius：取到它会把几乎所有候选判成不可站（实测 6775/6784）。
    MEASURED_SAFE_CLEARANCE = 0.25

    def test_clearance_not_below_measured_safe_value(self):
        clearance = _coordinator_params()['validator']['goal_clearance_radius']
        assert clearance >= self.MEASURED_SAFE_CLEARANCE, (
            f'goal_clearance_radius({clearance}) < 实测安全值'
            f'({self.MEASURED_SAFE_CLEARANCE})。\n'
            f'实测对比：0.25 -> lethal space 0 次；0.18 -> 2159 次。\n'
            f'原因：robot_radius=0.42 意味着距障碍 0.42m 内的格子是 253(inscribed '
            f'lethal)，净空要求太小的目标点会落在这条带里，机器人开过去后无法再规划。\n'
            f'要调低必须重新实测 lethal 计数，不能只按"等于 xy_goal_tolerance"推。')

    def test_clearance_within_bounds(self):
        coord = _coordinator_params()
        clearance = coord['validator']['goal_clearance_radius']
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            checkers = cp.get('goal_checker_plugins', [])
            xy = cp[checkers[0]]['xy_goal_tolerance']
            robot_radius = _robot_envelope_radius(f, 'global_costmap')
            assert clearance >= xy, (
                f'goal_clearance_radius({clearance}) < xy_goal_tolerance({xy}) '
                f'@{os.path.basename(f)}：容差圈内可能有不自由的点，机器人收敛不进去')
            assert clearance < robot_radius, (
                f'goal_clearance_radius({clearance}) >= robot_radius({robot_radius}) '
                f'@{os.path.basename(f)}：会把几乎所有候选判成不可站'
                f'（实测 6775/6784），探索一个目标都发不出去')

    def test_clearance_keeps_goal_out_of_inscribed_band(self):
        """记录用（不强制）：滑行量与净空半径同量级时，机器人**实际停住的位置**
        可能已经出了净空圈。实测发零速后仍蠕行 0.069~0.100m。
        这里只在净空半径明显小于实测滑行量时给出提示。"""
        coord = _coordinator_params()
        clearance = coord['validator']['goal_clearance_radius']
        measured_coast_max = 0.100
        if clearance < measured_coast_max:
            pytest.skip(
                f'goal_clearance_radius({clearance}) < 实测最大滑行量'
                f'({measured_coast_max})，机器人可能停在净空圈外。'
                f'这是已知权衡，不判失败，但换场地/换底盘后要重测。')


# ============================================== 其余既有联动（本轮之前就踩过的）

class TestExistingCouplings:

    def test_arrival_tolerance_not_tighter_than_controller(self):
        coord = _coordinator_params()
        arrival = coord['arrival_xy_tolerance']
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            xy = cp[cp['goal_checker_plugins'][0]]['xy_goal_tolerance']
            assert arrival >= xy, (
                f'arrival_xy_tolerance({arrival}) < xy_goal_tolerance({xy}) '
                f'@{os.path.basename(f)}：Nav2 报成功而协调器判超差，'
                f'会陷入「成功即失败」的反复重试')

    def test_unknown_clearance_is_zero(self):
        coord = _coordinator_params()
        v = coord['validator']['goal_unknown_clearance_radius']
        assert v == 0.0, (
            f'goal_unknown_clearance_radius={v}，必须为 0。'
            f'>= 地图分辨率会按定义否掉每一个前沿候选（实测 6775/6784），'
            f'dispatched 恒为 0、机器人永不移动')

    def test_inflation_exceeds_robot_radius(self):
        for f in NAV_PARAM_FILES:
            for which in ('global_costmap', 'local_costmap'):
                p = _costmap_params(f, which)
                rr = _robot_envelope_radius(f, which)
                infl = p['inflation_layer']['inflation_radius']
                assert infl > rr, (
                    f'{os.path.basename(f)} {which}: inflation_radius({infl}) '
                    f'<= robot_radius({rr})，路径会紧贴障碍物')

    def test_both_param_files_share_accuracy_caliber(self):
        """两份控制器配置的到位口径必须一致，否则切控制器会静默改变精度。"""
        seen = {}
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            gc = cp[cp['goal_checker_plugins'][0]]
            seen[os.path.basename(f)] = (
                gc['xy_goal_tolerance'], gc['yaw_goal_tolerance'])
        assert len(set(seen.values())) == 1, f'两份配置的到位容差不一致: {seen}'


# ========================================= 三段式实例自身的参数约束（与运行期一致）

class TestThreePhaseInstanceParams:
    """这些约束在插件 configure() 里会抛异常拒绝启动；这里离线先抓一遍，
    省得到了在线才发现（在线表现是节点起不来，日志在 controller_server 里）。"""

    def test_start_min_angle_not_below_align_tolerance(self):
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                tol = b.get('align_tolerance', 0.05)
                mn = b.get('start_min_angle', 0.20)
                assert mn >= tol, (
                    f'{os.path.basename(f)} {name}: start_min_angle({mn}) < '
                    f'align_tolerance({tol})，起步对齐段进去就立刻满足、形同虚设')

    def test_floor_not_above_max_angular_vel(self):
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                floor = b.get('align_floor_vel', 0.05)
                mx = b.get('align_max_vel', 0.6)
                assert floor <= mx, (
                    f'{os.path.basename(f)} {name}: align_floor_vel({floor}) > '
                    f'align_max_vel({mx})，配置自相矛盾')

    def test_new_goal_epsilon_positive(self):
        """置 0 会让每次周期性重规划都被判成新目标，退化回"每秒原地转一次"。
        实测未修时 86 次重规划里 12 次真的停下来转，最长 6.76s。"""
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                eps = b.get('new_goal_epsilon', 0.25)
                assert eps > 0.0, (
                    f'{os.path.basename(f)} {name}: new_goal_epsilon={eps} 必须 > 0')


# =============================== 窄通道贴边通行：yaml 必须落在运行期接受的范围内
#
# 下面每一条都对应 ThreePhaseController::declareAndLoadParams 里的一次 throw。
# 离线抓的价值：在线表现是 controller_server 起不来，而 nav2 的 lifecycle
# 握手会一直等下去（进程活着、话题在、就是不工作）—— 这个现象极难反推。

class TestNarrowPassageParams:

    # 足迹几何。八边形正好每 45° 复现一次横向包络，所以有利朝向周期是 pi/4。
    OCTAGON_PERIOD_RAD = 0.7853981634

    def test_narrow_block_present_in_every_instance(self):
        """漏配整块 = 全部回落到代码默认值。本项目已踩过一次同类问题
        （节点名 remap 让 yaml 键匹配不上，所有参数静默回落）。"""
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                assert 'narrow_enabled' in b, (
                    f'{os.path.basename(f)} {name}: 缺少窄通道配置块。'
                    f'漏配不会报错，只会静默使用代码默认值')

    def test_scan_step_resolves_the_target_band(self):
        """本档可通行区间只有 0.032m 宽（外接 0.42 − 内切 0.388），
        而栅格是 0.05m。步长过大就分辨不出「贴哪边能过」。"""
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                if not b.get('narrow_enabled', True):
                    continue
                step = b.get('narrow_scan_step', 0.01)
                half = b.get('narrow_scan_half_width', 0.30)
                assert 0.0 < step <= 0.025, (
                    f'{os.path.basename(f)} {name}: narrow_scan_step={step} '
                    f'必须在 (0, 0.025] —— 目标区间仅 0.032m 宽')
                assert step < half, (
                    f'{os.path.basename(f)} {name}: narrow_scan_step({step}) >= '
                    f'narrow_scan_half_width({half})，横向扫描只有一个样本')

    def test_yaw_gate_actually_gates(self):
        """闸门必须 < 周期/2。favorableYawError 的返回值上限就是周期/2，
        闸门比它还大 ⇒ 误差恒在闸门内 ⇒ 闸门永不生效。
        而这一档里朝向不对就是过不去，闸门失效等于带着错朝向往卡死里走。"""
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                if not b.get('narrow_enabled', True):
                    continue
                gate = b.get('narrow_yaw_gate', 0.12)
                period = b.get('narrow_favorable_period', self.OCTAGON_PERIOD_RAD)
                assert gate > 0.0, (
                    f'{os.path.basename(f)} {name}: narrow_yaw_gate={gate} 必须 > 0')
                assert gate < period / 2.0, (
                    f'{os.path.basename(f)} {name}: narrow_yaw_gate({gate}) >= '
                    f'narrow_favorable_period/2({period / 2.0})，朝向闸门形同虚设')

    def test_favorable_period_matches_footprint_symmetry(self):
        """有利朝向周期必须与足迹的旋转对称性一致。八边形配成 pi/2（四边形的周期）
        会让机器人绕远路去凑一个本来不必要的朝向。"""
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            n_vertices = None
            fp = _costmap_params(f, 'local_costmap').get('footprint')
            if fp:
                if isinstance(fp, str):
                    fp = yaml.safe_load(fp)
                n_vertices = len(fp)
            for name, b in _three_phase_instances(cp).items():
                if not b.get('narrow_enabled', True):
                    continue
                period = b.get('narrow_favorable_period', self.OCTAGON_PERIOD_RAD)
                assert period > 0.0
                if n_vertices:
                    expected = 2.0 * 3.141592653589793 / n_vertices
                    assert abs(period - expected) < 1e-3, (
                        f'{os.path.basename(f)} {name}: narrow_favorable_period={period} '
                        f'与 {n_vertices} 边形足迹的对称周期 {expected:.6f} 不符')

    def test_footprint_threshold_must_be_lethal_not_inflation(self):
        """足迹**多边形**的阈值必须是 254，不能是 253。

        ⚠️ 本条推翻了它的上一版。上一版断言 `thr < 254`，理由写的是
           "254 是真障碍，把它当触发条件就是让脱困逻辑去穿真障碍" ——
           **那个理由是错的**，而且这条测试当时正在把 bug 锁死。

        错在哪：253 的语义是"这一格离障碍不超过内切半径"（nav2
        InflationLayer::computeCost 逐字），而内切半径**就是底盘半宽**。
        足迹多边形已经把底盘尺寸表达了一遍，再要求外轮廓躲开 253 带，
        就是把底盘半宽算两遍。用 254 并不会"去穿真障碍" —— 恰恰相反，
        254 才是"轮廓压到障碍本体"这一唯一真实的碰撞条件。

        重复计的量级（实测数据）：需要 通道宽 > 4*侧向半宽
            八边形 侧向 0.3894 + 内切(含padding) 0.3992 -> 需 > 1.577m
        而本仿真环境最窄通道 0.65m 量级 ⇒ 恒为 253、零梯度。
        实测 2 轮共 1678s：足迹代价读数 51 次 253、仅 2 次 254，
        其中 26 次同时中心代价=0。909 次"连小足迹也过不去"全是假的。
        """
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                if not b.get('narrow_enabled', True):
                    continue
                thr = b.get('narrow_footprint_lethal_threshold', 254.0)
                assert 253.0 < thr <= 255.0, (
                    f'{os.path.basename(f)} {name}: '
                    f'narrow_footprint_lethal_threshold={thr} 必须 > 253。'
                    f'足迹多边形已经表达了底盘尺寸，用 253(膨胀内切带)当碰撞阈值'
                    f'等于重复计底盘半宽 —— 后果是窄于约 1.58m 的通道里恒为 253、'
                    f'零梯度。中心格查询才用 253。')

    def test_red_line_guards_are_all_armed(self):
        """用户明确的三条红线各自对应一个参数，任何一个 <= 0 都会让该红线失守：
          · narrow_timeout           所有脱困逻辑必须带超时
          · narrow_max_engagements   不能无限循环脱困
          · narrow_max_path_deviation 不得脱离全局参考路径太远
        """
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                if not b.get('narrow_enabled', True):
                    continue
                # 旧键必须**不存在**：它的语义已变（把"走得久"当成"卡住"，
                # 等于给窄通道设了 v_along x timeout 的长度上限，实测 2.5m
                # 连砍 3 次健康通行）。留着它会让人以为超时还按旧语义生效；
                # 代码侧也会拒绝启动。
                assert 'narrow_timeout' not in b, (
                    f'{os.path.basename(f)} {name}: narrow_timeout 已废弃且语义已变，'
                    f'必须改用 narrow_stall_timeout + narrow_stall_min_gain + '
                    f'narrow_hard_timeout')
                stall = b.get('narrow_stall_timeout', 6.0)
                gain = b.get('narrow_stall_min_gain', 0.05)
                hard = b.get('narrow_hard_timeout', 120.0)
                v_along = b.get('narrow_v_along', 0.10)
                assert stall > 0.0, (
                    f'{os.path.basename(f)} {name}: narrow_stall_timeout 必须 > 0'
                    f'（红线：必须带超时）')
                assert gain > 0.0, (
                    f'{os.path.basename(f)} {name}: narrow_stall_min_gain 必须 > 0，'
                    f'否则栅格噪声就能冒充进展、卡住判据形同虚设')
                assert hard > stall, (
                    f'{os.path.basename(f)} {name}: narrow_hard_timeout({hard}) 必须 > '
                    f'narrow_stall_timeout({stall})，否则绝对上限先触发'
                    f'＝退回"按时长判"那个缺陷')
                # 隐含的最低容忍速度不能超过实际行进速度，否则正常通行必被判卡住。
                implied = gain / stall
                assert implied < v_along, (
                    f'{os.path.basename(f)} {name}: 卡住判据隐含最低速度 '
                    f'{implied:.4f}m/s 已达到或超过 narrow_v_along({v_along})，'
                    f'正常通行也会被判成原地蹭')
                assert b.get('narrow_max_engagements', 3) >= 1, (
                    f'{os.path.basename(f)} {name}: narrow_max_engagements 必须 >= 1'
                    f'（红线：不能无限循环脱困）')
                dev = b.get('narrow_max_path_deviation', 0.50)
                half = b.get('narrow_scan_half_width', 0.30)
                assert dev > 0.0, (
                    f'{os.path.basename(f)} {name}: narrow_max_path_deviation 必须 > 0'
                    f'（红线：不得脱离参考路径）')
                assert dev >= half, (
                    f'{os.path.basename(f)} {name}: narrow_max_path_deviation({dev}) < '
                    f'narrow_scan_half_width({half})，横向扫描会选出立刻触发越界的目标')

    def test_speeds_are_restricted_not_normal_tracking(self):
        """贴边通行是受限通行。此处离墙不足 0.02m，高速下任何一拍的横向误差
        都直接变成撞墙。代码里 > 0.30 拒绝启动，这里用更紧的工程上限。"""
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                if not b.get('narrow_enabled', True):
                    continue
                v_along = b.get('narrow_v_along', 0.10)
                v_lat = b.get('narrow_v_lateral_max', 0.05)
                wz = b.get('narrow_wz_max', 0.20)
                assert 0.0 < v_along <= 0.30, (
                    f'{os.path.basename(f)} {name}: narrow_v_along={v_along} '
                    f'必须在 (0, 0.30] —— 贴边通行必须低速')
                assert 0.0 < v_lat <= v_along, (
                    f'{os.path.basename(f)} {name}: narrow_v_lateral_max({v_lat}) '
                    f'不应超过 narrow_v_along({v_along})，横向比前进还快说明配反了')
                assert 0.0 < wz <= 0.5, (
                    f'{os.path.basename(f)} {name}: narrow_wz_max={wz} 必须在 (0, 0.5]')

    def test_hysteresis_ticks_are_at_least_one(self):
        """clear_ticks=0 会让 narrowCleared 永不成立（见其实现），
        等于接管永不退出 —— 那是一个永久接管的控制器。"""
        for f in NAV_PARAM_FILES:
            for name, b in _three_phase_instances(_controller_params(f)).items():
                if not b.get('narrow_enabled', True):
                    continue
                assert b.get('narrow_trigger_ticks', 3) >= 1, (
                    f'{os.path.basename(f)} {name}: narrow_trigger_ticks 必须 >= 1')
                assert b.get('narrow_clear_ticks', 3) >= 1, (
                    f'{os.path.basename(f)} {name}: narrow_clear_ticks 必须 >= 1，'
                    f'0 会让接管永不退出')

    def test_instances_agree_on_narrow_config(self):
        """两个实例的差异只应在终点对齐上。窄通道是安全机制，
        两个场景各配一套必然漂开，而漂开后的现象是「搬运场景能过、探索场景卡住」。"""
        for f in NAV_PARAM_FILES:
            inst = _three_phase_instances(_controller_params(f))
            narrow = {
                name: {k: v for k, v in b.items() if k.startswith('narrow')}
                for name, b in inst.items()
            }
            if len(narrow) < 2:
                continue
            ref_name, ref = next(iter(narrow.items()))
            for name, blk in narrow.items():
                assert blk == ref, (
                    f'{os.path.basename(f)}: {name} 与 {ref_name} 的窄通道配置不一致，'
                    f'差异={set(blk.items()) ^ set(ref.items())}')


# ================================ 全局路径居中：膨胀梯度必须在中心有唯一最小值
#
# 需求：1.5m 通道里机器人中心距两侧墙各 0.75m。
#
# 这一组把 nav2 膨胀层的代价公式复算一遍，断言"中心是唯一最小代价点"。
# 为什么值得离线固化：0.65 -> 0.75 这个改动**在线现象极不明显** ——
# 路径看起来都"差不多在中间"，只有把横向剖面算出来才看得到 0.65 时
# 中心是一条 ±0.10m 的零代价平带（带内无梯度，规划器只挑最短路径）。

class TestGlobalPathCentering:

    TARGET_CORRIDOR_M = 2.0      # 需求口径已从 1.5m 改为 2.0m
    CSF_DEFAULT = 3.0

    @staticmethod
    def _inflation_cost(d, inscribed, inflation_radius, csf):
        """nav2_costmap_2d::InflationLayer 的代价函数。

        253 那一档由 inscribed 决定，**与 inflation_radius 无关** ——
        这正是"放大 inflation_radius 不缩小可行域"的原因。
        """
        if d <= inscribed:
            return 253.0
        if d > inflation_radius:
            return 0.0
        return 252.0 * math.exp(-csf * (d - inscribed))

    def _lateral_profile(self, f, width_m):
        """返回 [(偏离中心, 代价)]，代价取两侧墙的最大值（膨胀层取 max）。"""
        cp = _costmap_params(f, 'global_costmap')
        infl = cp['inflation_layer']['inflation_radius']
        csf = cp['inflation_layer'].get('cost_scaling_factor', self.CSF_DEFAULT)
        inscribed = _inscribed_radius(f, 'global_costmap')
        half = width_m / 2.0
        out = []
        y = 0.0
        while y <= half - inscribed + 1e-9:
            c = max(
                self._inflation_cost(half + y, inscribed, infl, csf),
                self._inflation_cost(half - y, inscribed, infl, csf))
            out.append((round(y, 3), c))
            y += 0.01
        return out

    def test_center_is_strict_minimum(self):
        """中心必须是**唯一**最小值，且代价随偏离单调上升。
        0.65 时中心与 ±0.05m 代价同为 0.0 —— 那不是最小值，是平带。"""
        for f in NAV_PARAM_FILES:
            prof = self._lateral_profile(f, self.TARGET_CORRIDOR_M)
            assert len(prof) >= 3, f'{os.path.basename(f)}: 剖面样本太少'
            center = prof[0][1]
            for y, c in prof[1:]:
                assert c > center, (
                    f'{os.path.basename(f)}: 偏离中心 {y}m 的代价 {c:.1f} '
                    f'不高于中心 {center:.1f} —— 中心不是唯一最小值，'
                    f'规划器没有"往中间靠"的梯度')

    def test_no_zero_cost_flat_band_at_center(self):
        """零代价平带宽度必须为 0。平带内无梯度，规划器只会挑最短路径、
        于是拐角切内弯贴到平带边缘。这就是 0.65 做不到居中的直接原因。"""
        for f in NAV_PARAM_FILES:
            cp = _costmap_params(f, 'global_costmap')
            infl = cp['inflation_layer']['inflation_radius']
            half = self.TARGET_CORRIDOR_M / 2.0
            flat = max(0.0, half - infl)
            assert flat <= 1e-9, (
                f'{os.path.basename(f)}: global inflation_radius={infl} < '
                f'半通道宽 {half} ⇒ 中心存在 ±{flat:.3f}m 零代价平带，'
                f'带内无梯度。inflation_radius 必须 >= {half}')

    def test_widening_inflation_does_not_shrink_free_band(self):
        """放大 inflation_radius 不得缩小"规划器可用带"。
        253 由 inscribed 决定；若这条失败说明有人把两者搞混了。"""
        for f in NAV_PARAM_FILES:
            inscribed = _inscribed_radius(f, 'global_costmap')
            half = self.TARGET_CORRIDOR_M / 2.0
            usable = half - inscribed
            assert usable > 0.0, (
                f'{os.path.basename(f)}: 内切半径 {inscribed} >= 半通道宽 {half}，'
                f'{self.TARGET_CORRIDOR_M}m 通道整条不可通行')
            # 可用带只跟 inscribed 有关，与 inflation_radius 无关
            for probe in (0.65, 0.75, 1.25):
                cp = _costmap_params(f, 'global_costmap')
                csf = cp['inflation_layer'].get('cost_scaling_factor', self.CSF_DEFAULT)
                edge = self._inflation_cost(inscribed + 1e-6, inscribed, probe, csf)
                assert edge < 253.0, (
                    f'{os.path.basename(f)}: inflation_radius={probe} 时'
                    f'内切边界外侧被判成 253，可用域被缩小了')

    def test_local_costmap_inflation_left_untouched(self):
        """局部代价地图必须维持原值 0.65。

        它被 MPPI 的 ObstaclesCritic/CostCritic 读取，抬高它的代价会改变
        已验证的跟踪行为。本次需求只要求全局路径居中。"""
        for f in NAV_PARAM_FILES:
            lp = _costmap_params(f, 'local_costmap')
            got = lp['inflation_layer']['inflation_radius']
            assert abs(got - 0.65) < 1e-9, (
                f'{os.path.basename(f)}: local_costmap inflation_radius={got}，'
                f'应保持 0.65 不变（只改全局）')

    # ---- 需求二：不要为抄近道钻窄通道 ----
    #
    # 规划器最小化「几何长度 x (1 + m * cost/252)」（node_2d.cpp 源码确认，
    # 启发式是纯欧氏距离、不含代价项）。所以窄通道要被避开，前提是它的
    # **惩罚因子 > 1.0**；等于 1.0 时它与宽路等价，A* 只剩"谁短走谁"。

    @staticmethod
    def _penalty_factor(cost, multiplier):
        """Node2D::getTraversalCost 的惩罚因子部分。"""
        return 1.0 + multiplier * (cost / 252.0)

    def _factor_for_width(self, f, width_m):
        cp = _costmap_params(f, 'global_costmap')
        infl = cp['inflation_layer']['inflation_radius']
        csf = cp['inflation_layer'].get('cost_scaling_factor', self.CSF_DEFAULT)
        inscribed = _inscribed_radius(f, 'global_costmap')
        m = _load(f)['planner_server']['ros__parameters']['GridBased'][
            'cost_travel_multiplier']
        c = self._inflation_cost(width_m / 2.0, inscribed, infl, csf)
        return self._penalty_factor(c, m)

    def test_target_width_corridor_is_actually_penalized(self):
        """口径宽度的通道必须有 >1 的惩罚因子。

        这是"钻窄通道"那个缺陷的直接判据：inflation_radius 0.75 时
        2m 通道中心代价为 0、因子恰为 1.000，与宽路完全等价。
        """
        for f in NAV_PARAM_FILES:
            fac = self._factor_for_width(f, self.TARGET_CORRIDOR_M)
            assert fac > 1.0 + 1e-9, (
                f'{os.path.basename(f)}: {self.TARGET_CORRIDOR_M}m 通道的惩罚因子'
                f'={fac:.3f}，等于宽路 ⇒ 规划器只会挑短的、必走窄通道。'
                f'需要 global inflation_radius >= {self.TARGET_CORRIDOR_M / 2.0}')

    def test_narrower_corridor_is_penalized_more(self):
        """必须单调：越窄的通道惩罚越重。否则"偏好宽路"没有方向性。"""
        widths = [1.0, 1.5, 2.0]
        for f in NAV_PARAM_FILES:
            facs = [self._factor_for_width(f, w) for w in widths]
            for a, b, wa, wb in zip(facs, facs[1:], widths, widths[1:]):
                assert a > b, (
                    f'{os.path.basename(f)}: {wa}m 通道因子 {a:.3f} 未高于 '
                    f'{wb}m 的 {b:.3f} —— 窄的没有被罚得更重')

    def test_inflation_radius_covers_target_half_width(self):
        """判据的显式形式，改小 inflation_radius 会在这里失败。"""
        need = self.TARGET_CORRIDOR_M / 2.0
        for f in NAV_PARAM_FILES:
            infl = _costmap_params(f, 'global_costmap')[
                'inflation_layer']['inflation_radius']
            assert infl >= need - 1e-9, (
                f'{os.path.basename(f)}: global inflation_radius={infl} < '
                f'{need}，宽于 {2 * infl}m 的通道中心代价恒为 0、彼此无法区分')

    def test_cost_weight_is_above_default(self):
        """仅有梯度不够：cost_travel_multiplier 太低时规划器仍会抄近道贴边。"""
        for f in NAV_PARAM_FILES:
            gb = _load(f)['planner_server']['ros__parameters']['GridBased']
            m = gb.get('cost_travel_multiplier', 2.0)
            assert m > 2.0, (
                f'{os.path.basename(f)}: cost_travel_multiplier={m} 仍是默认量级，'
                f'梯度存在但规划器不够在意它')


# ============================================================ 行为树与控制器对应


class TestBehaviorTrees:

    def test_bt_controller_ids_are_configured(self):
        """BT 里写的 controller_id 必须真的存在于 controller_plugins，
        否则每次 FollowPath 直接 abort（与坑 3 同一类错误）。"""
        import re
        bt_files = sorted(glob.glob(os.path.join(NAV_BT_DIR, '*.xml')))
        if not bt_files:
            pytest.skip('仓库内没有自定义行为树')
        declared = set()
        for f in NAV_PARAM_FILES:
            declared |= set(_controller_params(f).get('controller_plugins', []))
        for bt in bt_files:
            text = open(bt, encoding='utf-8').read()
            for cid in re.findall(r'controller_id="([^"]+)"', text):
                assert cid in declared, (
                    f'{os.path.basename(bt)} 用了 controller_id="{cid}"，'
                    f'但它不在任何 nav2_params 的 controller_plugins 里: {sorted(declared)}')

    def test_followpath_named_controller_exists(self):
        """默认行为树的 controller_id 端口默认值是 "FollowPath"，
        所以必须始终有一个叫这个名字的实例，否则默认树/RViz 全部失败。"""
        for f in NAV_PARAM_FILES:
            plugins = _controller_params(f).get('controller_plugins', [])
            assert 'FollowPath' in plugins, (
                f'{os.path.basename(f)} 的 controller_plugins 里没有 FollowPath。'
                f'默认行为树的 controller_id 默认值就是它')

# ================================== 需求1：follow_path 模式的配置自洽

class TestDispatchMode:
    """follow_path 模式把已校验路径直接交给控制器，绕开了 bt_navigator。
    因此 BT 里的 controller_id 不再生效，改由协调器的 follow_controller_id 决定 ——
    这一项写错会让每次 FollowPath 直接 abort（与坑 3 同类）。
    """

    def test_mode_is_recognised(self):
        mode = _coordinator_params().get('nav_dispatch_mode', 'follow_path')
        assert mode in ('follow_path', 'navigate_to_pose'), (
            f'nav_dispatch_mode={mode!r} 非法，只接受 follow_path / navigate_to_pose')

    def test_follow_controller_id_is_configured(self):
        coord = _coordinator_params()
        if coord.get('nav_dispatch_mode', 'follow_path') != 'follow_path':
            pytest.skip('未启用 follow_path 模式')
        cid = coord.get('follow_controller_id', 'FollowPathExplore')
        declared = set()
        for f in NAV_PARAM_FILES:
            declared |= set(_controller_params(f).get('controller_plugins', []))
        assert cid in declared, (
            f'follow_controller_id={cid!r} 不在任何 controller_plugins 里: '
            f'{sorted(declared)}。后果：每次 FollowPath 直接 abort，'
            f'现象是机器人完全不跟踪路径')

    def test_goal_checker_id_matches_checker_count(self):
        """留空只在「只有一个 goal checker」时安全；多项时留空必然 abort。"""
        coord = _coordinator_params()
        if coord.get('nav_dispatch_mode', 'follow_path') != 'follow_path':
            pytest.skip('未启用 follow_path 模式')
        gid = coord.get('follow_goal_checker_id', '')
        for f in NAV_PARAM_FILES:
            checkers = _controller_params(f).get('goal_checker_plugins', [])
            if len(checkers) > 1:
                assert gid, (
                    f'{os.path.basename(f)} 配了多个 goal checker {checkers}，'
                    f'但 follow_goal_checker_id 留空 —— 该端口无默认值，会 abort')
            if gid:
                assert gid in checkers, (
                    f'follow_goal_checker_id={gid!r} 不在 {os.path.basename(f)} 的 '
                    f'goal_checker_plugins {checkers} 里')

    def test_replan_period_not_negative(self):
        v = _coordinator_params().get('replan_period_sec', 1.0)
        assert v >= 0.0, f'replan_period_sec={v} 不能为负'

    def test_replan_faster_than_nav_timeout(self):
        """重规划周期必须远小于单目标超时，否则路径失效后来不及换就超时了。"""
        coord = _coordinator_params()
        if coord.get('nav_dispatch_mode', 'follow_path') != 'follow_path':
            pytest.skip('未启用 follow_path 模式')
        period = coord.get('replan_period_sec', 1.0)
        timeout = coord['nav_timeout_sec']
        if period == 0.0:
            pytest.skip('replan_period_sec=0：刻意不重规划，代价已在 yaml 注释里写明')
        assert period < timeout / 10.0, (
            f'replan_period_sec({period}) 相对 nav_timeout_sec({timeout}) 太大，'
            f'路径失效后几乎没有换路径的机会')


# ============ 坑 6：无条件周期重规划 = 没有一条路径被跟踪到位 ============

class TestReplanPolicy:
    """实测缺陷：replan_policy 只有 periodic 一种行为时，跟踪期**无条件**
    每 1.0s 重规划并重发 FollowPath。一轮运行 36 个目标下发了 393 次
    FollowPath —— 平均每个目标换 10.9 条路径、每条只被跟踪约 1.5s
    就被下一条抢占，没有任何一条被跟踪到位。

    这一组守住修复后的判据不被配置层重新破坏。
    """

    VALID = ('on_invalid', 'periodic')

    def test_policy_value_is_recognised(self):
        p = _coordinator_params().get('replan_policy', 'on_invalid')
        assert p in self.VALID, (
            f'replan_policy={p!r} 非法，只接受 {self.VALID}。'
            f'节点会拒绝启动而不是静默回落')

    def test_default_policy_is_on_invalid(self):
        """periodic 是回退配置，不该是长期生效的那个。"""
        p = _coordinator_params().get('replan_policy', 'on_invalid')
        assert p == 'on_invalid', (
            f'replan_policy={p!r}：periodic 会无条件换路径，'
            f'实测 36 个目标换了 393 条、无一条被跟踪到位。'
            f'它只保留作一键回退，正常运行必须是 on_invalid')

    def test_deviation_limit_exceeds_arrival_tolerance(self):
        """!!! 判据颠倒会把本次修复完全抵消 !!!

        偏离阈值若不大于抵达容差，收尾阶段的正常贴合误差就会被判成
        「已偏离路径」，于是每次快到目标时都换一条新路径 —— 现象与修复前一样。
        节点在 loadParameters 里也会拒绝启动，这里是配置层的第二道。
        """
        coord = _coordinator_params()
        dev = coord.get('path_deviation_limit_m', 0.6)
        arr = coord['arrival_xy_tolerance']
        assert dev > arr, (
            f'path_deviation_limit_m({dev}) <= arrival_xy_tolerance({arr})：'
            f'收尾阶段的正常贴合误差会被判成偏离路径、反复换路径')

    def test_deviation_limit_exceeds_controller_goal_tolerance(self):
        """同样不能小于控制器自己的 xy_goal_tolerance —— 那是收敛球半径。"""
        dev = _coordinator_params().get('path_deviation_limit_m', 0.6)
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            for name, ck in cp.items():
                if not isinstance(ck, dict) or 'xy_goal_tolerance' not in ck:
                    continue
                if name not in cp.get('goal_checker_plugins', []):
                    continue          # 只看真正生效的那个 checker
                tol = ck['xy_goal_tolerance']
                assert dev > tol, (
                    f'{os.path.basename(f)} {name}: path_deviation_limit_m({dev}) '
                    f'<= xy_goal_tolerance({tol})，收敛球内的正常误差会触发换路径')

    def test_path_max_age_is_disabled_or_generous(self):
        """路径寿命是兜底项。设成小值等于把 on_invalid 又变回周期重规划。"""
        coord = _coordinator_params()
        age = coord.get('path_max_age_sec', 0.0)
        assert age >= 0.0, f'path_max_age_sec={age} 不能为负'
        if age == 0.0:
            return                    # 0 = 不启用，这是推荐值
        period = coord.get('replan_period_sec', 1.0)
        assert age > 5 * max(period, 1.0), (
            f'path_max_age_sec({age}) 太小：这会让路径因为「旧」而被换掉，'
            f'等于绕过 on_invalid 判据退回周期重规划')

    def test_invalid_replan_attempts_bounded(self):
        """不得无限期跟踪一条已判死的路径。

        实测缺陷：「剩余段不可通行 -> 重规划 -> 新路径也不合法 -> 沿用当前路径」
        这条兜底会让机器人明知走不通还继续顶，实测原地顶了 17s，
        直到 progress checker 才救回来。所以必须有限次后放弃该目标。
        """
        n = _coordinator_params().get('max_invalid_replan_attempts', 3)
        assert n >= 1, (
            f'max_invalid_replan_attempts={n}：0 等于允许无限期跟踪已判死的路径')
        assert n <= 10, (
            f'max_invalid_replan_attempts={n} 太大：每次尝试至少一个检查节拍，'
            f'这么多次等于又回到「顶着走十几秒」')

    def test_invalid_replan_budget_fits_nav_timeout(self):
        """放弃目标必须发生在单目标超时之前，否则这条保护等于不存在。"""
        coord = _coordinator_params()
        n = coord.get('max_invalid_replan_attempts', 3)
        interval = coord.get('replan_min_interval_sec', 1.0)
        timeout = coord['nav_timeout_sec']
        assert n * max(interval, coord.get('replan_check_period_sec', 0.5)) < timeout, (
            f'{n} 次 x {interval}s 已经超过 nav_timeout_sec({timeout})，'
            f'「路径判死就放弃目标」这条保护永远来不及生效')

    def test_check_period_is_faster_than_min_interval_allows(self):
        """检查节拍必须比最小重规划间隔更密，否则「该换的时候还没看」。"""
        coord = _coordinator_params()
        chk = coord.get('replan_check_period_sec', 0.5)
        assert chk > 0.0, f'replan_check_period_sec={chk} 必须 > 0'
        interval = coord.get('replan_min_interval_sec', 1.0)
        assert chk <= interval, (
            f'replan_check_period_sec({chk}) > replan_min_interval_sec({interval})：'
            f'检查比允许换路径的频率还稀，防抖参数形同虚设')


# ============ 坑 7：冷启动自举必须真能转动、且不绕过安全链 ============

class TestBootstrap:
    """SLAM 冷启动死锁：slam_toolbox 只在移动超过 minimum_travel_heading
    之后才插入扫描 -> 冷启动地图无已知格 -> 协调器不下发目标 -> 机器人不动
    -> 地图不长。自举把「推一把」收进节点，但配置错了会静默无效：
    转了却没转够阈值，现象仍然是「机器人不动」，排查会绕回原点。
    """

    def test_mode_is_recognised(self):
        m = _coordinator_params().get('bootstrap_mode', 'rotate')
        assert m in ('rotate', 'disabled'), (
            f'bootstrap_mode={m!r} 非法，只接受 rotate / disabled')

    def test_min_yaw_delta_matches_slam_minimum_travel_heading(self):
        """!!! 这是本组最关键的一条 !!!

        自举的目的就是触发 slam_toolbox 插入新扫描。若 bootstrap_min_yaw_delta
        小于 SLAM 的 minimum_travel_heading，自举会「达标」但 SLAM 仍然不插入
        扫描，地图照样不长 —— 自举变成一个看起来成功的空操作。
        """
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        delta = coord.get('bootstrap_min_yaw_delta', 0.20)
        slam = _load(SLAM_MAPPER_CFG)
        heading = None
        for section in slam.values():
            if isinstance(section, dict) and 'ros__parameters' in section:
                heading = section['ros__parameters'].get('minimum_travel_heading')
                if heading is not None:
                    break
        assert heading is not None, (
            f'在 {SLAM_MAPPER_CFG} 里找不到 minimum_travel_heading，'
            f'无法校验自举阈值 —— 这条耦合必须可查')
        assert delta >= heading, (
            f'bootstrap_min_yaw_delta({delta}) < slam minimum_travel_heading({heading})：'
            f'自举会判"转够了"但 SLAM 不插入新扫描，地图照样不长')

    def test_rotation_budget_can_reach_threshold_unlimited(self):
        """即使完全不限速也转不到阈值 = 配置自相矛盾。"""
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        vel = coord.get('bootstrap_angular_vel', 0.40)
        dur = coord.get('bootstrap_duration_sec', 4.0)
        delta = coord.get('bootstrap_min_yaw_delta', 0.20)
        assert vel * dur >= delta, (
            f'bootstrap_angular_vel({vel}) * bootstrap_duration_sec({dur}) = '
            f'{vel * dur} < bootstrap_min_yaw_delta({delta})：一次自举必然判失败')

    def test_rotation_budget_survives_worst_case_coupling_scale(self):
        """必须按**最坏情况的下游限速**取时长，不是按名义角速度。

        臂-底盘耦合节点会按机械臂展开程度把速度连续缩放，下限是
        min_speed_scale。按名义值算够、按最坏值算不够，就会出现
        「有时能自举、有时静默转不动」这种最难查的间歇故障。
        """
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        scale = _coupling_min_speed_scale()
        if scale is None:
            pytest.skip('未找到耦合节点的 min_speed_scale 默认值')
        vel = coord.get('bootstrap_angular_vel', 0.40)
        dur = coord.get('bootstrap_duration_sec', 4.0)
        delta = coord.get('bootstrap_min_yaw_delta', 0.20)
        assert vel * scale * dur >= delta, (
            f'最坏限速下 {vel}*{scale}*{dur} = {vel * scale * dur}rad '
            f'< bootstrap_min_yaw_delta({delta})：机械臂展开时自举会静默转不动。'
            f'把 bootstrap_duration_sec 提到 >= {delta / (vel * scale):.1f}s')

    def test_cmd_rate_beats_chassis_cmd_vel_timeout(self):
        """发得比底盘 cmd_vel 超时还慢，速度会被反复归零。"""
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        rate = coord.get('bootstrap_cmd_rate_hz', 20.0)
        timeout = _chassis_cmd_vel_timeout()
        assert timeout is not None, (
            f'在 {CHASSIS_CFG} 里找不到 cmd_vel_timeout_sec，这条耦合必须可查')
        assert rate >= 2.0 / timeout, (
            f'bootstrap_cmd_rate_hz({rate}) 相对底盘 cmd_vel_timeout_sec({timeout}) '
            f'太低：至少要 {2.0 / timeout}Hz 才不会被反复超时归零')

    def test_yaw_frame_does_not_depend_on_slam(self):
        """!!! 自举量转角的 frame 不能依赖 SLAM !!!

        `map -> odom` 由 slam_toolbox 发布。真正的冷启动死锁下 SLAM 还没出图，
        用 map 系量转角会让自举被自己的前置条件挡死（实测连续 71 次
        「需要自举但取不到当前朝向」），而自举正是为破这个死锁存在的。
        """
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        frame = coord.get('bootstrap_yaw_frame', 'odom')
        map_frame = coord.get('map_frame', 'map')
        assert frame != map_frame, (
            f'bootstrap_yaw_frame={frame!r} 与 map_frame 相同：'
            f'{map_frame} -> odom 由 SLAM 发布，冷启动时不存在，'
            f'自举会被自己的前置条件挡死 —— 而那正是它要破的死锁')
        assert frame, 'bootstrap_yaw_frame 不能为空'

    def test_cmd_vel_topic_does_not_bypass_safety_chain(self):
        """自举必须发到 controller_server 的输出点，不能直发 /cmd_vel。

        直发 /cmd_vel 会一次绕过四层保护：velocity_smoother 的加减速限制、
        cmd_vel_body_to_world 的倾倒监控、臂-底盘耦合限速、以及最终的
        力矩闭环 leash。
        """
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        topic = coord.get('bootstrap_cmd_vel_topic', '/cmd_vel_nav_body_raw')
        assert topic.strip('/') != 'cmd_vel', (
            f'bootstrap_cmd_vel_topic={topic!r} 直发底盘终端话题，'
            f'绕过 smoother/倾倒监控/耦合限速/leash 四层保护')

    def test_min_clearance_matches_robot_envelope(self):
        """净空门限应当就是底盘外接半径：更小则八边形顶点可能已接触障碍。"""
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        clearance = coord.get('bootstrap_min_clearance_m', 0.42)
        for f in NAV_PARAM_FILES:
            for which in ('global_costmap', 'local_costmap'):
                r = _robot_envelope_radius(f, which)
                # 容差取 1mm：mppi 的 footprint 顶点 (0.297,0.297) 算出来是
                # 0.420021m，比标称 0.42 大 21μm —— 那是 yaml 里写坐标时的取整
                # 残差，不是安全裕度问题。用 1e-6 会让这条测试变成噪声。
                assert clearance >= r - 1e-3, (
                    f'bootstrap_min_clearance_m({clearance}) < '
                    f'{os.path.basename(f)} {which} 外接半径({r})：'
                    f'障碍已进到足迹以内还在原地转')

    def test_bootstrap_attempts_are_bounded(self):
        """禁止死循环重试：自举必须有次数上限。"""
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        n = coord.get('bootstrap_max_attempts', 6)
        assert 1 <= n <= 60, (
            f'bootstrap_max_attempts={n} 不合理：<1 等于关闭，过大等于无限重试')

    def test_trigger_wait_is_not_zero(self):
        """立刻自举等于抢在 SLAM 前面动 —— 启动瞬态地图本来就要几秒才长出来。"""
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        wait = coord.get('bootstrap_trigger_wait_sec', 6.0)
        assert wait > 0.0, (
            f'bootstrap_trigger_wait_sec={wait}：启动瞬态就会自举，'
            f'而那时地图本来就还没长出来')

    def test_scan_topic_is_the_one_nav2_uses(self):
        """安全门用的激光必须与 nav2 代价地图看的是同一条，否则两边判据不同源。

        注意**不能**从 nav2_params_*.yaml 里读 observation_sources 的 topic ——
        那里写的是 `/scan`，但 nav2_full_bringup 会按 scan_source 参数在
        launch 期把它覆盖成 `/scan_from_cloud`。yaml 里的值不是生效值，
        照着它断言会得出一个错误结论。真正的出处是 bringup launch。
        """
        coord = _coordinator_params()
        if coord.get('bootstrap_mode', 'rotate') == 'disabled':
            pytest.skip('自举已关闭')
        topic = coord.get('bootstrap_scan_topic', '/scan_from_cloud')

        with open(NAV_BRINGUP_LAUNCH, encoding='utf-8') as fh:
            launch_src = fh.read()
        # 从 launch 源码里取 scan_source 的默认值和它对应的话题表达式。
        m_default = re.search(
            r"'scan_source',\s*default_value='([a-z_]+)'", launch_src)
        assert m_default, (
            f'在 {os.path.basename(NAV_BRINGUP_LAUNCH)} 里找不到 scan_source 的默认值，'
            f'无法确定 Nav2 实际订阅哪条激光')
        default_source = m_default.group(1)
        # 取 scan_topic_expr 那一段里出现的两个 /topic，第一个对应 slice_scan 分支。
        m_block = re.search(
            r'scan_topic_expr\s*=\s*PythonExpression\(\[(.*?)\]\)', launch_src, re.S)
        assert m_block, (
            f'在 {os.path.basename(NAV_BRINGUP_LAUNCH)} 里找不到 scan_topic_expr，'
            f'这条耦合必须可查')
        topics = re.findall(r"'(/[\w/]+)'", m_block.group(1))
        assert len(topics) == 2, (
            f'scan_topic_expr 里解析出 {topics}，期望恰好两个候选话题')
        effective = topics[0] if default_source == 'slice_scan' else topics[1]
        assert topic.lstrip('/') == effective.lstrip('/'), (
            f'bootstrap_scan_topic={topic!r} 与 Nav2 实际订阅的 {effective!r} 不一致'
            f'(scan_source 默认 {default_source!r})：自举安全门与代价地图判据不同源')

    def test_min_known_cells_is_positive(self):
        """这条是 COMPLETED 判定的前置：设成 0 会让空地图被判成「探索完成」。"""
        n = _coordinator_params().get('min_known_cells_for_decision', 100)
        assert n >= 1, (
            f'min_known_cells_for_decision={n}：0 等于取消保护，'
            f'冷启动的空地图（前沿格也是 0）会被判成探索完成')


# ================== 坑 5：纯旋转段 vs progress checker（只量平移）

class TestProgressCheckerIsRotationAware:
    """三段式的 ALIGN 段是纯旋转(vx=vy=0)，SimpleProgressChecker 只量平移，
    旋转时间会被直接从 movement_time_allowance 里扣掉。

    实测算术：ALIGN 最长 5.55s(零平移) + FOLLOW 均速 0.111m/s 走 0.5m 需 4.50s
              = 10.05s > 10s -> 必然 Failed to make progress。
    """

    ROTATION_AWARE = 'PoseProgressChecker'

    def _has_three_phase(self, cp):
        return bool(_three_phase_instances(cp))

    def test_rotation_aware_checker_when_three_phase_enabled(self):
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            if not self._has_three_phase(cp):
                continue
            pc = cp.get('progress_checker', {})
            plugin = str(pc.get('plugin', ''))
            assert self.ROTATION_AWARE in plugin, (
                f'{os.path.basename(f)} 启用了三段式跟踪(含纯旋转段)，'
                f'但 progress_checker 是 {plugin!r}。\n'
                f'SimpleProgressChecker 只量平移，旋转期间平移恒为 0 而计时器在走，'
                f'实测 5.55s 旋转 + 4.50s 行驶 = 10.05s > 10s 必然失败。\n'
                f'应使用 nav2_controller::PoseProgressChecker（旋转也算进展）。')

    def test_required_movement_angle_present_and_sane(self):
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            pc = cp.get('progress_checker', {})
            if self.ROTATION_AWARE not in str(pc.get('plugin', '')):
                continue
            ang = pc.get('required_movement_angle')
            assert ang is not None, (
                f'{os.path.basename(f)} 用了 PoseProgressChecker 但没配 '
                f'required_movement_angle，会落到插件默认值')
            assert 0.05 < ang < 1.0, (
                f'required_movement_angle={ang} 不合理：太小会把里程计角度噪声'
                f'当成进展，太大则正常对齐转角触发不了重置')

    def test_angle_threshold_reachable_within_align_segment(self):
        """转角阈值必须小于「一次对齐实际能转过的角度」，否则等于没启用。

        对齐段至少要转过 start_min_angle 才会进入（小于它直接跳过），
        所以阈值必须明显小于 start_min_angle。
        """
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            pc = cp.get('progress_checker', {})
            ang = pc.get('required_movement_angle')
            if ang is None:
                continue
            for name, b in _three_phase_instances(cp).items():
                start_min = b.get('start_min_angle', 0.20)
                assert ang < start_min, (
                    f'{os.path.basename(f)} {name}: required_movement_angle({ang}) '
                    f'>= start_min_angle({start_min})。'
                    f'对齐段只在误差超过 start_min_angle 时才进入，'
                    f'阈值若不小于它，短对齐就触发不了计时重置')


# ==================================== 坑 9：陈旧检测只加在一份配置里

class TestObservationStalenessDetection:
    """两份 nav2_params_*.yaml 的 obstacle_layer 都必须开陈旧检测。

    起因：expected_update_rate 原先只加在 nav2_params_rpp.yaml 里。
    controller_plugin 是个 launch 参数，切到 mppi 就换了整份 yaml ——
    于是陈旧检测**静默消失**，而 costmap 照发、日志无异常、判据全过。
    2026-09-02 在实机上用探针读活节点的参数，才看到两个 costmap 都是 0.0。

    这类"一份文件加了、另一份忘了"的分叉，靠人工 review 是抓不住的：
    两份文件各 800+ 行、共 4 处 scan 块。所以必须由测试强制。

    !!! 边界要写清楚 !!!：这个参数只让 costmap **打告警**。
    实测 controller_server 二进制里没有任何检查 costmap currency 的字符串，
    陈旧时它照样发速度。真正的拒绝下发在 chassis_cmd_bridge 的 /scan 联锁。
    这条测试保的是"可观测性不会静默丢失"，不是"安全性有兜底"。
    """

    #: 两个雷达帧周期。健康态实测 /scan 9.9~10.04Hz —— 取 0.1 会因抖一帧误报。
    MIN_RATE = 0.11
    #: 秒级陈旧就是危险的（实测事故那次龄期 2.03s），所以上限压在 1s 内。
    MAX_RATE = 1.0

    def _scan_blocks(self, path):
        """返回 [(which, scan 块), ...]，覆盖 local 与 global 两个 costmap。"""
        out = []
        for which in ('local_costmap', 'global_costmap'):
            p = _costmap_params(path, which)
            ob = p.get('obstacle_layer')
            if not isinstance(ob, dict):
                continue
            for src in str(ob.get('observation_sources', 'scan')).split():
                blk = ob.get(src)
                if isinstance(blk, dict):
                    out.append(('%s/%s' % (which, src), blk))
        return out

    def test_found_scan_blocks(self):
        """哨兵：一个 scan 块都找不到时，下面的测试会空转而"通过"。"""
        total = sum(len(self._scan_blocks(f)) for f in NAV_PARAM_FILES)
        assert total >= 2 * len(NAV_PARAM_FILES), (
            '每份 nav2_params_*.yaml 都应至少有 local+global 两个 obstacle_layer '
            'scan 块，实际只找到 %d 个（文件数 %d）。'
            '若确实改了结构，请同时改这条哨兵，不要留空转的测试。'
            % (total, len(NAV_PARAM_FILES)))

    def test_expected_update_rate_set_everywhere(self):
        missing = []
        for f in NAV_PARAM_FILES:
            for label, blk in self._scan_blocks(f):
                if blk.get('expected_update_rate') is None:
                    missing.append('%s %s' % (os.path.basename(f), label))
        assert not missing, (
            '这些 obstacle_layer 没设 expected_update_rate，等于**关掉**陈旧检测'
            '（nav2 默认 0.0 使 isCurrent() 恒为 true，/scan 多旧都不告警）：\n  '
            + '\n  '.join(missing))

    def test_expected_update_rate_in_sane_band(self):
        for f in NAV_PARAM_FILES:
            for label, blk in self._scan_blocks(f):
                v = blk.get('expected_update_rate')
                if v is None:
                    continue        # 由上一条测试负责报错
                v = float(v)
                assert self.MIN_RATE <= v <= self.MAX_RATE, (
                    '%s %s 的 expected_update_rate=%r 不在 [%.2f, %.2f]。'
                    '太小(如 0.1)会因 /scan 抖一帧就误报——健康态实测 9.9~10.04Hz；'
                    '太大则秒级陈旧也不告警，而实测事故那次龄期就是 2.03s。'
                    % (os.path.basename(f), label, v,
                       self.MIN_RATE, self.MAX_RATE))

    def test_all_files_agree(self):
        """两份文件必须取同一个值。

        分开取值没有任何正当理由：数据源是同一个 /scan、同一台雷达。
        值不同只可能是改了一份忘了另一份，而那正是本类要防的事。
        """
        seen = {}
        for f in NAV_PARAM_FILES:
            for label, blk in self._scan_blocks(f):
                v = blk.get('expected_update_rate')
                if v is not None:
                    seen.setdefault(float(v), []).append(
                        '%s %s' % (os.path.basename(f), label))
        assert len(seen) <= 1, (
            'expected_update_rate 在不同文件/图层间取值不一致：%s。'
            '数据源是同一个 /scan，没有理由分开取值。' % (
                {k: v for k, v in seen.items()},))

    def test_rate_slower_than_bridge_interlock(self):
        """costmap 的告警阈值必须比写通路联锁**宽**。

        联锁(scan_max_age_sec)才是真正会拒绝下发的那一层。若 costmap 的阈值
        反而更宽松，就会出现"底盘已经因陈旧停车，而 costmap 一声不响"的组合，
        排查时看不到任何线索指向 /scan。
        """
        interlock = _bridge_scan_max_age()
        if interlock is None:
            pytest.skip('读不到 chassis_bridge 的 scan_max_age_sec 默认值')
        for f in NAV_PARAM_FILES:
            for label, blk in self._scan_blocks(f):
                v = blk.get('expected_update_rate')
                if v is None:
                    continue
                assert float(v) <= interlock, (
                    '%s %s 的 expected_update_rate=%r 比写通路联锁的 '
                    'scan_max_age_sec=%r 还宽松 —— 会出现"底盘已停车而 costmap '
                    '零告警"的组合，排查时没有任何线索指向 /scan。'
                    % (os.path.basename(f), label, v, interlock))


# ================================ 坑 10：限速覆盖的默认值会静默抬高 yaml 的限速

class TestSpeedCapOverride:
    """navigation.launch.py 的 max_linear_speed 一键限速。

    这个机制的**唯一隐患**是它的默认值：RewrittenYaml 的 param_rewrites 是
    无条件生效的，所以 launch 的默认值会**覆盖** yaml 里写的 vx_max。
    如果哪天有人把 yaml 的 vx_max 从 1.0 调低到 0.5 以求安全，
    而 launch 默认值还是 1.0，那么这个"安全"改动会被静默抬回 1.0 ——
    改的人看着 yaml 说"我限到 0.5 了"，实跑却是 1.0。

    所以这里强制两者相等。
    """

    NAV_LAUNCH = os.path.join(
        _SRC, 'astribot_s1_navigation', 'launch', 'navigation.launch.py')

    def _launch_default(self):
        with open(self.NAV_LAUNCH, encoding='utf-8') as fh:
            src = fh.read()
        m = re.search(r"'max_linear_speed'\s*,\s*default_value='([0-9.]+)'", src)
        return float(m.group(1)) if m else None

    def test_launch_arg_exists(self):
        assert os.path.isfile(self.NAV_LAUNCH), self.NAV_LAUNCH
        assert self._launch_default() is not None, (
            'navigation.launch.py 里找不到 max_linear_speed 的 '
            'DeclareLaunchArgument 默认值')

    def test_launch_default_matches_yaml(self):
        """**本类的核心断言。**"""
        default = self._launch_default()
        if default is None:
            pytest.skip('launch 参数不存在，由上一条测试报错')
        for f in NAV_PARAM_FILES:
            cp = _controller_params(f)
            fp = cp.get('FollowPath', {})
            vx = fp.get('vx_max')
            if vx is None:
                continue
            assert abs(float(vx) - default) < 1e-9, (
                '%s 的 FollowPath.vx_max=%s，而 navigation.launch.py 的 '
                'max_linear_speed 默认值是 %s。两者必须相等 —— '
                'RewrittenYaml 无条件生效，不相等时这个"默认"会静默把 yaml 里的'
                '限速**抬高**到 launch 的值，改 yaml 的人完全看不出来。'
                % (os.path.basename(f), vx, default))

    def test_linear_caps_are_rewritten(self):
        """只压 vx_max 是个漏洞：vx_min 仍是负的满量程，可以全速倒车。"""
        with open(self.NAV_LAUNCH, encoding='utf-8') as fh:
            src = fh.read()
        block = src[src.find('param_substitutions.update('):]
        block = block[:block.find('})') + 2]
        for key in ('vx_max', 'vy_max', 'vx_min'):
            assert ("'%s':" % key) in block, (
                'param_substitutions 里没有重写 %r —— 少压一个量就是一个漏洞。'
                '尤其 vx_min：只压 vx_max 时 MPPI 仍可全速倒车。' % key)

    def test_array_params_are_not_rewritten(self):
        """velocity_smoother 的 max_velocity/min_velocity **不得**在这里重写。

        它们是 double 数组，而 RewrittenYaml.convert() 只尝试 int/float/bool
        （已读 /opt/ros/humble 下的实现确认：三种都不匹配就原样返回字符串）。
        写进去的后果是 velocity_smoother 配置期抛
          parameter 'max_velocity' has invalid type: ... is of type {string}
        然后 lifecycle_manager "Aborting bringup" —— 整套 nav2 从未 activate，
        而 9 个进程全都活着、进程数判据照过。实测踩过一次。

        这条测试刻意断言"**不存在**"，是为了防止有人看到
        "第二层限速没压住"就把它加回来。要压第二层得换机制
        （例如直接改 yaml，或给 velocity_smoother 单独传 parameters）。
        """
        with open(self.NAV_LAUNCH, encoding='utf-8') as fh:
            src = fh.read()
        block = src[src.find('param_substitutions.update('):]
        block = block[:block.find('})') + 2]
        for key in ('max_velocity', 'min_velocity'):
            assert ("'%s':" % key) not in block, (
                'param_substitutions 重写了 %r —— 它是 double 数组，'
                'RewrittenYaml 会把它写成字符串，velocity_smoother 配置失败、'
                'nav2 整套无法 activate（而进程数判据完全看不出来）。' % key)

    def test_reverse_cap_is_negative(self):
        """vx_min 的重写必须产出**负值**。

        写成正数的后果很隐蔽：vx_min > 0 意味着 MPPI 被强制只能前进，
        它连倒车退让这个恢复行为都做不出来，而报出来的只是"规划不出轨迹"。
        """
        with open(self.NAV_LAUNCH, encoding='utf-8') as fh:
            src = fh.read()
        m = re.search(r"def _neg\(expr\):\s*\n\s*return PythonExpression\("
                      r"\[([^\]]+)\]\)", src)
        assert m, '找不到 _neg 的实现'
        assert '-abs(' in m.group(1), (
            '_neg 里必须是 -abs(...)：单纯写 -float(...) 时，'
            '调用方传了负数就会被翻成正数，vx_min 变正会让 MPPI 无法倒车退让')

    def test_angular_is_not_silently_capped(self):
        """角速度刻意不压 —— 这条测试防的是"顺手也把 wz 压了"。

        静默改角速度会让原地对齐段和 Spin 恢复行为跟着变，
        而那不是"限线速度"这个请求的一部分。
        """
        with open(self.NAV_LAUNCH, encoding='utf-8') as fh:
            src = fh.read()
        block = src[src.find('param_substitutions.update('):]
        block = block[:block.find('})') + 2]
        assert "'wz_max'" not in block, (
            'max_linear_speed 把 wz_max 也压了 —— 调用方只要求限线速度')


# =============================================================================
# max_linear_speed 的**转发**：白名单漏项会让整轮限速扫描变成假数据
#
# navigation.launch.py 有了 max_linear_speed 并不等于外层能用上它。
# IncludeLaunchDescription 的 launch_arguments 是白名单：没列进去的名字不会
# 传进子 launch，而子 launch 里 DeclareLaunchArgument 的 default_value 照常生效。
#
# 实际发生过的漏项（2026-08-27 发现）：nav2_full_bringup.launch.py 只转发了
# use_sim_time / controller_plugin / enable_arm_chassis_coupling / scan_topic，
# 于是 `nav2_full_bringup.launch.py ... max_linear_speed:=0.2`
#   · 不报错
#   · 不告警
#   · vx_max 仍然是 1.0
# 后果不是"限速没生效"这么轻——它会让「不同限速档位下的到位精度」这类扫描
# 产出若干档位数字**完全相同**的表，而每张表单独看都完全正常。
# 这类静默失败必须由测试钉住，不能靠读 launch 代码发现。
# =============================================================================


class TestSpeedCapForwarding:
    """外层 launch 必须把 max_linear_speed 显式转发给 navigation.launch.py。

    连带钉住 enable_posture_monitor —— 同一个漏项、但后果更重：
    实机上那个监控会**永久**把 /cmd_vel 归零且无复位路径，而在补上转发之前
    根本没有办法从这个入口关掉它（runbook 要求实机必须关）。
    """

    NAV_LAUNCH = os.path.join(
        _SRC, 'astribot_s1_navigation', 'launch', 'navigation.launch.py')

    # 必须逐一转发的参数。加新参数时把名字加进来即可。
    FORWARDED = ('max_linear_speed', 'enable_posture_monitor')

    @staticmethod
    def _src(path):
        with open(path, encoding='utf-8') as fh:
            return fh.read()

    @staticmethod
    def _default_of(src, name='max_linear_speed'):
        m = re.search(r"'%s'\s*,\s*default_value='([0-9.]+)'" % name, src)
        return float(m.group(1)) if m else None

    @classmethod
    def _declares(cls, src, name):
        """该 launch 是否声明了这个参数（默认值可以是表达式，不限于字面量）。"""
        return re.search(r"DeclareLaunchArgument\(\s*\n?\s*'%s'" % name,
                         src) is not None

    def _navigation_include_block(self):
        """截出 nav2_full_bringup 里 include navigation.launch.py 的那段。

        必须**只在这一段里**找，不能全文 grep：全文里 DeclareLaunchArgument
        本身就含有参数名字样，全文 grep 会在漏转发时照样通过 ——
        这正是这个缺陷能藏住的原因，测试不能重复同一个错误。
        """
        src = self._src(NAV_BRINGUP_LAUNCH)
        anchor = src.find("'navigation.launch.py'")
        assert anchor > 0, "nav2_full_bringup.launch.py 里找不到 navigation.launch.py 的 include"
        start = src.find('launch_arguments={', anchor)
        assert start > 0, 'navigation include 后面找不到 launch_arguments'
        end = src.find('}.items()', start)
        assert end > start, 'launch_arguments 块没有闭合'
        return src[start:end]

    @pytest.mark.parametrize('name', FORWARDED)
    def test_declared_in_outer_launch(self, name):
        assert self._declares(self._src(NAV_BRINGUP_LAUNCH), name), (
            'nav2_full_bringup.launch.py 里没有 %s 的 DeclareLaunchArgument —— '
            '不声明就没法从命令行给这一层传值' % name)

    @pytest.mark.parametrize('name', FORWARDED)
    def test_forwarded_to_navigation_launch(self, name):
        """**本类的核心断言。**"""
        block = self._navigation_include_block()
        assert ("'%s'" % name) in block, (
            'nav2_full_bringup.launch.py 的 navigation include 的 launch_arguments '
            '里没有 %s。launch_arguments 是白名单，漏项不报错也不告警，'
            '子 launch 会安静地用自己的默认值 —— '
            '限速扫描会产出若干档位数字完全相同的表；'
            '而漏掉 enable_posture_monitor 时实机的 /cmd_vel 会被永久归零。'
            % name)

    @pytest.mark.parametrize('name', FORWARDED)
    def test_forwarded_value_is_the_launch_configuration(self, name):
        """转发的必须是 LaunchConfiguration，不能是写死的字面量。

        写成 'max_linear_speed': '1.0' 同样能通过上一条测试，
        但命令行传值依旧无效 —— 而且更难发现，因为白名单里"有这一项"。
        """
        block = self._navigation_include_block()
        m = re.search(r"'%s'\s*:\s*([^,\n]+)" % name, block)
        assert m, '上一条测试已覆盖缺失情形'
        value = m.group(1).strip()
        assert 'LaunchConfiguration' in value, (
            "%s 转发的是 %r，不是 LaunchConfiguration('%s')。"
            '写死字面量时白名单里"有这一项"，但命令行传值仍然无效，比漏项更难发现。'
            % (name, value, name))

    def test_outer_default_matches_inner_default(self):
        """max_linear_speed 的两层默认值必须一致。

        外层默认值一旦与内层不同，它会**无条件覆盖**内层：
        外层 1.0 / 内层 0.5 时，只读 navigation.launch.py 的人会以为限到了 0.5。
        这与 TestSpeedCapOverride.test_launch_default_matches_yaml 是同一个隐患
        的第二段（yaml → 内层 launch → 外层 launch，三段都得对齐）。
        """
        outer = self._default_of(self._src(NAV_BRINGUP_LAUNCH))
        inner = self._default_of(self._src(self.NAV_LAUNCH))
        if outer is None or inner is None:
            pytest.skip('默认值缺失，由其它测试报错')
        assert abs(outer - inner) < 1e-9, (
            'nav2_full_bringup.launch.py 的 max_linear_speed 默认值是 %s，'
            'navigation.launch.py 是 %s。外层无条件覆盖内层，不一致时'
            '只读内层 launch 的人会看到一个从未生效的限速值。' % (outer, inner))

    def test_posture_monitor_default_follows_env(self):
        """姿态监控的默认值必须**跟着 env 走**，实机侧为 false。

        这一项刻意不照抄内层的 true：
        实机 /odom 是 3-DOF 轮式里程计，z/roll/pitch 恒等于 0（实测 509 帧
        min=max=0.0000），而判据是 |z-normal_height|>max_height_deviation，
        normal_height=0.134 是**仿真**值 -> |0-0.134|=0.134 > 0.06 ->
        一上电就判异常姿态 -> **永久**把 /cmd_vel 归零，且没有复位路径。

        所以外层默认值必须是一个含 env 的表达式，且 real 侧取 false。
        """
        src = self._src(NAV_BRINGUP_LAUNCH)
        m = re.search(
            r"DeclareLaunchArgument\(\s*\n\s*'enable_posture_monitor'\s*,\s*\n"
            r"\s*default_value=(.{0,400}?)\n\s*description=", src, re.S)
        assert m, "找不到 enable_posture_monitor 的 default_value"
        default = m.group(1)
        assert 'PythonExpression' in default, (
            'enable_posture_monitor 的默认值是写死的字面量 %r。'
            '写死 true 时实机会被永久归零 /cmd_vel；写死 false 时仿真里'
            '倾倒检测被静默关掉。必须跟着 env 走。' % default.strip())
        assert "'real'" in default, (
            '默认值表达式里没有出现 real —— 它没有按环境分支')
        real_branch = default.split("'real'")[0]
        assert "'false'" in real_branch, (
            "实机分支必须取 false。当前表达式: %s" % default.strip())



# =============================================================================
# 窄通道缩足迹：跨两个包的配置自洽性
#
# 这一节防的是**同一个几何量被抄在两个包里然后漂开**。
# 足迹字符串现在出现在三处：
#   nav2_params_mppi.yaml  的两个 ThreePhaseController 实例(narrow_footprint_*)
#   exploration_coordinator_params.yaml 的 footprint_watchdog.default_footprint
# 看门狗若"复原"成另一个形状，比不复原更糟 —— 它会一边报"已复原"一边把
# 机器人建成错的尺寸，而两边日志都正常。
# =============================================================================


def _square_cfg():
    """mppi 的两个 ThreePhaseController 实例的缩足迹配置。"""
    cp = _controller_params(os.path.join(NAV_CFG_DIR, 'nav2_params_mppi.yaml'))
    return _three_phase_instances(cp)


def _poly_inscribed(fp_str):
    """多边形内切半径。与 nav2 的 calculateMinAndMaxDistances 同一定义：
    取所有**顶点距离**与**边距离**里的最小值。
    实测核对：八边形 0.388039、正方形 a=0.31 -> 0.310000（与 nav2 逐位一致）。
    """
    fp = yaml.safe_load(fp_str)
    assert len(fp) >= 3, f'足迹少于 3 点: {fp_str}'

    def edge_dist(a, b):
        ax, ay = a
        bx, by = b
        ex, ey = bx - ax, by - ay
        l2 = ex * ex + ey * ey
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, -(ax * ex + ay * ey) / l2))
        return math.hypot(ax + t * ex, ay + t * ey)

    best = min(math.hypot(x, y) for x, y in fp)
    for i in range(len(fp)):
        best = min(best, edge_dist(fp[i], fp[(i + 1) % len(fp)]))
    return best


class TestNarrowSquareFootprint:
    """缩足迹的三处配置必须自洽，且守卫必须真的能拦住不安全的值。"""

    def test_watchdog_default_matches_controller_default(self):
        """看门狗复原用的足迹必须与控制器的默认足迹**几何一致**。

        比几何而不是比字符串：空格/小数写法不同不该算失败，
        但形状不同必须算失败。
        """
        coord = _coordinator_params()
        wd = coord.get('footprint_watchdog')
        assert wd, 'exploration_coordinator_params.yaml 缺 footprint_watchdog 段'
        wd_fp = yaml.safe_load(wd['default_footprint'])
        for name, inst in _square_cfg().items():
            ctrl_fp = yaml.safe_load(inst['narrow_footprint_default'])
            assert len(wd_fp) == len(ctrl_fp), (
                f'{name}: 看门狗默认足迹 {len(wd_fp)} 点 != 控制器 {len(ctrl_fp)} 点 —— '
                f'看门狗会"复原"成另一个形状，比不复原更糟')
            for (wx, wy), (cx, cy) in zip(wd_fp, ctrl_fp):
                assert math.isclose(wx, cx, abs_tol=1e-9) and \
                       math.isclose(wy, cy, abs_tol=1e-9), (
                    f'{name}: 看门狗与控制器的默认足迹顶点不一致')

    def test_narrow_footprint_is_actually_smaller(self):
        """小足迹的内切半径必须真的更小，否则切换毫无收益还白付风险。"""
        for name, inst in _square_cfg().items():
            big = _poly_inscribed(inst['narrow_footprint_default'])
            small = _poly_inscribed(inst['narrow_footprint_narrow'])
            assert small < big, (
                f'{name}: 窄通道足迹内切 {small:.4f} 不小于默认 {big:.4f}')

    def test_narrow_footprint_not_smaller_than_chassis(self):
        """🔴 小足迹不得小于底盘物理包络。

        数据源 astribot_s1_torso_wheel.xacro：躯干碰撞圆柱 radius 0.30、
        轮球 radius 0.08 且轮心 (±0.21635, ±0.21635)。
        轴向真实伸出 = max(0.30, 0.21635+0.08) = 0.30。
        把机器人建模成比这更小，在仿真里的表现是"通过率提高"——非常危险。
        """
        for name, inst in _square_cfg().items():
            small = _poly_inscribed(inst['narrow_footprint_narrow'])
            guard = float(inst['chassis_min_envelope_radius'])
            assert guard >= 0.30 - 1e-9, (
                f'{name}: chassis_min_envelope_radius={guard} 低于 URDF 躯干半径 0.30')
            assert small >= guard, (
                f'{name}: 窄通道足迹内切 {small:.4f} < 底盘包络下限 {guard:.4f}')

    def test_lease_timeout_exceeds_lease_period(self):
        """租约超时必须大于续租周期，否则看门狗会在正常工作时误判复原。"""
        coord = _coordinator_params()
        timeout = float(coord['footprint_watchdog']['lease_timeout_sec'])
        for name, inst in _square_cfg().items():
            period = float(inst['narrow_square_lease_period'])
            assert timeout > period, (
                f'{name}: 看门狗租约超时 {timeout}s 不大于续租周期 {period}s —— '
                f'会在控制器正常续租时误判进程已死')

    def test_square_and_watchdog_enabled_together(self):
        """缩足迹与看门狗必须同开同关。

        只开缩足迹 = 没有防锁存兜底；只开看门狗 = 无害但说明配置在漂。
        """
        coord = _coordinator_params()
        wd_on = bool(coord['footprint_watchdog']['enabled'])
        for name, inst in _square_cfg().items():
            sq_on = bool(inst['narrow_square_enabled'])
            assert sq_on == wd_on, (
                f'{name}: narrow_square_enabled={sq_on} 而 '
                f'footprint_watchdog.enabled={wd_on} —— 两者必须同开同关，'
                f'只开前者等于没有防锁存兜底')

    def test_both_instances_agree(self):
        """两个控制器实例(导航用/探索用)的缩足迹配置必须一致。

        漂开的话，A/B 只切一个实例，测出来的差异归因不到任何一处。
        """
        insts = _square_cfg()
        assert len(insts) >= 2, f'预期至少 2 个 ThreePhaseController 实例，实际 {len(insts)}'
        keys = ['narrow_square_enabled', 'narrow_footprint_default',
                'narrow_footprint_narrow', 'chassis_min_envelope_radius',
                'narrow_square_hard_timeout', 'narrow_square_lease_period',
                'narrow_square_verify_timeout']
        ref_name, ref = next(iter(insts.items()))
        for name, inst in insts.items():
            for k in keys:
                assert inst[k] == ref[k], (
                    f'{name}.{k}={inst[k]} != {ref_name}.{k}={ref[k]}')

    def test_exit_window_not_shorter_than_entry(self):
        """🔴 切出窗口必须 >= 切入窗口，否则**保证**切换振荡。

        算术：切入看 preview 内有麻烦、切出看 exit_preview 内没麻烦。
        若 exit < entry，则机器人处在 [exit, entry] 之间时两个判据同时成立
        ⇒ 切入、复原、切入…（第一轮实测 123 次切换的同类病因）。
        控制器侧有启动守卫会拒绝启动，这里在配置层再拦一道。
        """
        for name, inst in _square_cfg().items():
            entry = inst['narrow_prealign_preview']
            exit_w = inst['narrow_square_exit_preview']
            assert exit_w >= entry, (
                f'{name}: narrow_square_exit_preview={exit_w} < '
                f'narrow_prealign_preview={entry} —— 保证振荡')

    def test_hysteresis_band_exceeds_costmap_resolution(self):
        """迟滞带必须大于代价地图分辨率，否则它表达不出来。

        这条不是洁癖：八边形各向异性 0.032m < 分辨率 0.05m，正是纯预对齐
        在这张图上测不出任何效果的原因。迟滞带若也小于一格，就是同一个错。
        """
        res = _costmap_params(
            os.path.join(NAV_CFG_DIR, 'nav2_params_mppi.yaml'), 'local_costmap')['resolution']
        for name, inst in _square_cfg().items():
            band = inst['narrow_square_exit_preview'] - inst['narrow_prealign_preview']
            assert band > res, (
                f'{name}: 迟滞带 {band:.3f}m <= 栅格 {res}m —— '
                f'代价地图表达不出这个差异，等于没有迟滞')

    def test_min_dwell_below_hard_timeout(self):
        """最短驻留必须显著小于硬超时。

        反了的话「最短驻留」会吃掉整条正常复原路径，让每次都走硬超时那条
        异常分支 —— 表面上"复原了"，实际每次都是异常收尾。
        """
        for name, inst in _square_cfg().items():
            dwell = inst['narrow_square_min_dwell']
            hard = inst['narrow_square_hard_timeout']
            assert 0.0 <= dwell < hard, (
                f'{name}: narrow_square_min_dwell={dwell} 必须在 [0, {hard}) 内')

    def test_switch_ceiling_arithmetic_is_stated(self):
        """单次切换周期下限必须 > 实测的中位驻留 0.15s，且给出切换次数上限。

        0.15s = narrow_clear_ticks(3) / controller_frequency(20Hz)，是第一轮
        A/B 里 121/123 次复原的实际触发时刻。新配置的周期下限
        = min_dwell + cooldown，必须比它高一个量级，否则改了等于没改。
        """
        freq = _controller_params(
            os.path.join(NAV_CFG_DIR, 'nav2_params_mppi.yaml'))['controller_frequency']
        for name, inst in _square_cfg().items():
            old_dwell = inst['narrow_clear_ticks'] / freq
            new_floor = inst['narrow_square_min_dwell'] + inst['narrow_square_cooldown']
            assert new_floor > 10.0 * old_dwell, (
                f'{name}: 新的单次切换周期下限 {new_floor}s 不足旧中位驻留 '
                f'{old_dwell:.3f}s 的 10 倍 —— 改动量级不够')

    def test_lease_period_below_verify_timeout(self):
        """续租周期必须 < 回读验证超时，否则"等确认"期间可能一次都没续租。"""
        for name, inst in _square_cfg().items():
            assert inst['narrow_square_lease_period'] < inst['narrow_square_verify_timeout'], (
                f'{name}: narrow_square_lease_period 必须 < narrow_square_verify_timeout')

    def test_watchdog_lease_timeout_allows_consecutive_reads(self):
        """🔴 看门狗租约超时必须容得下"凑齐 N 次连续回读"所需的时间。

        算术：N 次连续回读**至少**需要 N 个回读周期。租约超时若比这还短，
        看门狗就可能在读数根本还凑不齐时动手 —— 那正是旧配置
        (2.0s 超时 vs 1.0s 回读周期 vs 需要 3 次) 造成 2.1s 误判的病因。
        留 1 个周期的抖动余量。
        """
        wd = _coordinator_params()['footprint_watchdog']
        period = wd['readback_period_sec']
        n = wd['consecutive_reads']
        assert period > 0 and n >= 1
        need = period * (n + 1)
        assert wd['lease_timeout_sec'] >= need, (
            f"lease_timeout_sec={wd['lease_timeout_sec']} < "
            f'readback_period_sec*(consecutive_reads+1)={need} —— '
            f'读数还凑不齐就可能动手')

    def test_watchdog_stale_threshold_allows_normal_jitter(self):
        """陈旧阈值必须容得下至少 2 个回读周期。

        太紧的话正常抖动就会让看门狗永远处于"读数太老、不敢动手"的瞎眼状态，
        而它瞎眼时**不会报"故障"**、只会安静地不动手 —— 等于没有兜底却看不出来。
        """
        wd = _coordinator_params()['footprint_watchdog']
        assert wd['readback_stale_sec'] >= 2.0 * wd['readback_period_sec'], (
            f"readback_stale_sec={wd['readback_stale_sec']} < "
            f"2*readback_period_sec={2.0 * wd['readback_period_sec']}")

    def test_watchdog_timeout_dwarfs_controller_lease_period(self):
        """看门狗租约超时必须远大于控制器续租周期（>= 4 倍）。

        这两个数在不同的包、不同的 yaml 里，最容易各改一个而漂开。
        余量不足时一次调度抖动就会让看门狗误判"设它的进程死了"。
        """
        wd_timeout = _coordinator_params()['footprint_watchdog']['lease_timeout_sec']
        for name, inst in _square_cfg().items():
            lease = inst['narrow_square_lease_period']
            assert wd_timeout >= 4.0 * lease, (
                f'{name}: 看门狗 lease_timeout_sec={wd_timeout} < '
                f'4*narrow_square_lease_period={4.0 * lease}')

    def test_watchdog_readback_period_matches_global_costmap(self):
        """看门狗的 readback_period_sec 必须等于 global costmap 的 publish_frequency。

        它只用于启动守卫的算术 —— 但算术的输入写错，守卫算出来的下限就是错的，
        而守卫会照样"通过"。这是本项目反复吃过的那类错：判据本身错了。
        """
        pf = _costmap_params(
            os.path.join(NAV_CFG_DIR, 'nav2_params_mppi.yaml'),
            'global_costmap')['publish_frequency']
        wd_period = _coordinator_params()['footprint_watchdog']['readback_period_sec']
        assert abs(wd_period - 1.0 / pf) < 1e-9, (
            f'footprint_watchdog.readback_period_sec={wd_period} != '
            f'1/global_costmap.publish_frequency={1.0 / pf}')

    def test_square_requires_prealign(self):
        """🔴 缩足迹依赖预对齐：开了前者却关了后者 = 静默无效。

        唯一的切入通路是预对齐的前视判定（顺序不可颠倒：先对齐后换足迹），
        所以 narrow_prealign_enabled=false 时缩足迹**永远进不去**，
        表现是切换 0 次、零告警 —— A/B 会得出"这机制没用"，真相是它一次没跑。
        控制器侧有启动守卫直接拒绝启动，这里在配置层再拦一道。
        """
        for name, inst in _square_cfg().items():
            if inst['narrow_square_enabled']:
                assert inst['narrow_prealign_enabled'], (
                    f'{name}: narrow_square_enabled=true 但 '
                    f'narrow_prealign_enabled=false —— 缩足迹会静默永不生效')


class TestCostThresholdSemantics:
    """🔴 两种代价查询、两个阈值，绝不混用。

    依据是 nav2 自己的 InflationLayer::computeCost：
        distance == 0                      -> 254 (LETHAL，障碍本体)
        distance*res <= inscribed_radius   -> 253 (INSCRIBED_INFLATED)
    253 的含义是"离障碍不超过内切半径"= **底盘中心放这里必然碰撞**，
    而内切半径就是底盘半宽 —— 膨胀层已经把底盘尺寸算进去了。

    · 足迹**多边形**查询(footprintCostAtPose) -> 254
      多边形已经表达了底盘尺寸，再要求外轮廓躲开 253 带 = 算两遍
    · **中心格**查询(getCost) -> 253
      那里膨胀带正好代表底盘尺寸

    上一版把多边形阈值写成 253，后果是窄于 4*侧向半宽(本机 ~1.58m)的通道里
    足迹代价恒为 253、零梯度，实测 909 次"连小足迹也过不去"全是假的。
    """

    def test_footprint_threshold_is_lethal_not_inscribed(self):
        for name, inst in _square_cfg().items():
            v = inst['narrow_footprint_lethal_threshold']
            assert v > 253.0, (
                f'{name}: narrow_footprint_lethal_threshold={v} <= 253 —— '
                f'足迹多边形用 253 会重复计底盘半宽；正确值 254')

    def test_center_threshold_stays_inscribed(self):
        for name, inst in _square_cfg().items():
            v = inst['narrow_center_lethal_threshold']
            assert 0.0 < v <= 253.0, (
                f'{name}: narrow_center_lethal_threshold={v} 必须在 (0,253] —— '
                f'中心格判据里膨胀带就代表底盘尺寸，设 254 会把放不进去的位置判成可站')

    def test_two_thresholds_must_differ(self):
        """两个阈值方向相反，"统一成一个数"会破坏其中一半。"""
        for name, inst in _square_cfg().items():
            fp = inst['narrow_footprint_lethal_threshold']
            ce = inst['narrow_center_lethal_threshold']
            assert fp > ce, f'{name}: 足迹阈值({fp}) 必须 > 中心格阈值({ce})'

    def test_square_actually_fits_the_narrowest_corridor(self):
        """缩足迹必须真的能过本环境最窄的通道，否则这个机制没有意义。

        场景事实：本仿真环境没有低于 0.65m 的通道。
        判据分两层，都要过：
          · 规划器的**中心格**判据：内切半径(含 padding) < 通道半宽
          · 足迹**多边形**判据：侧向半宽 < 通道半宽
        """
        import math
        NARROWEST = 0.65
        half = NARROWEST / 2.0
        for name, inst in _square_cfg().items():
            poly = yaml.safe_load(inst['narrow_footprint_narrow'])
            pad = _costmap_params(
                os.path.join(NAV_CFG_DIR, 'nav2_params_mppi.yaml'),
                'global_costmap').get('footprint_padding', 0.01)
            # 侧向半宽取所有朝向里最小的那个（= 对齐到最有利朝向）
            lateral = min(
                max(abs(-x * math.sin(t) + y * math.cos(t)) for x, y in poly)
                for t in [math.radians(d) for d in range(0, 91)])
            inscribed = _poly_inscribed(inst['narrow_footprint_narrow']) + pad
            assert lateral + pad < half, (
                f'{name}: 小足迹侧向半宽(含 padding) {lateral + pad:.4f} '
                f'>= 通道半宽 {half:.4f} —— 缩了也过不去，机制无意义')
            assert inscribed < half, (
                f'{name}: 小足迹内切半径(含 padding) {inscribed:.4f} >= 通道半宽 '
                f'{half:.4f} —— 规划器仍会报 Starting point in lethal space')

    def test_octagon_genuinely_cannot_fit_narrowest_corridor(self):
        """反面：默认八边形必须**真的**过不去，否则根本不需要缩足迹。

        这条是防"两个足迹其实都能过/都不能过"——那样 A/B 测不出任何东西。
        """
        NARROWEST = 0.65
        half = NARROWEST / 2.0
        for name, inst in _square_cfg().items():
            pad = _costmap_params(
                os.path.join(NAV_CFG_DIR, 'nav2_params_mppi.yaml'),
                'global_costmap').get('footprint_padding', 0.01)
            inscribed = _poly_inscribed(inst['narrow_footprint_default']) + pad
            assert inscribed > half, (
                f'{name}: 默认足迹内切半径 {inscribed:.4f} < 通道半宽 {half:.4f} —— '
                f'它本来就过得去，那缩足迹解决的是什么问题？')
