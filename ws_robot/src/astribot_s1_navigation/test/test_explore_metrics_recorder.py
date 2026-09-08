# Copyright 2026 Astribot.
#
# 录制器的无 ROS 依赖部分：状态串解析、协调器计数器提取，
# 以及"新文件到底会不会被装进 install"这件事。
#
# 后者不是形式主义：本包的 setup.py 没有 scripts/ 的通配，
# 而 ros2 launch / ros2 run 只看 install。少一行 data_files 的症状是
# "文件明明在 src 里，命令却说找不到" —— 同类坑本包吃过（.rviz 放到
# config/ 下时永不安装）。
import os
import re

import pytest

from astribot_s1_navigation.explore_metrics.state_parse import (
    coordinator_counters, parse_state_line)

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.abspath(os.path.join(_HERE, '..'))
SETUP_PY = os.path.join(_PKG, 'setup.py')

# 协调器真实发出的一整行（照 publishState() 的格式串拼的）
REAL_LINE = (
    'state=NAVIGATING goal_in_flight=1 goal=(1.23,-4.56) candidate=2/7 '
    'dispatched=5 succeeded=3 rejected=11 nav_fail=0/5 sample_fail=0/8 '
    'validate_fail=1/6 auto_resume=0/3 bootstrap=1/3 bootstrap_result=ok '
    'path_pts=36 replan_policy=on_invalid'
)


class TestParseStateLine:

    def test_parses_the_real_line(self):
        f = parse_state_line(REAL_LINE)
        assert f['state'] == 'NAVIGATING'
        assert f['dispatched'] == '5'
        assert f['replan_policy'] == 'on_invalid'

    def test_parenthesised_value_does_not_swallow_next_field(self):
        """**核心断言**：goal=(x,y) 里的括号不能把后面的字段吞掉。

        用贪婪正则匹配时 goal 的值会一路吃到行尾，dispatched 等字段
        全部解析不到 —— 而"字段没解析到"表现为计数器恒为 None，
        看起来像协调器没发那些字段。
        """
        f = parse_state_line(REAL_LINE)
        assert f['goal'] == '(1.23,-4.56)'
        assert f['candidate'] == '2/7'

    def test_unknown_field_does_not_raise(self):
        """协调器加了新字段不该让整次录制挂掉。"""
        f = parse_state_line(REAL_LINE + ' brand_new_field=42')
        assert f['brand_new_field'] == '42'
        assert f['state'] == 'NAVIGATING'

    def test_token_without_equals_is_ignored(self):
        f = parse_state_line('state=IDLE 这是一句中文说明 dispatched=1')
        assert f['state'] == 'IDLE'
        assert f['dispatched'] == '1'

    def test_empty_and_garbage(self):
        assert parse_state_line('') == {}
        assert parse_state_line('=1') == {}
        assert parse_state_line(None) == {}


class TestCoordinatorCounters:

    def test_extracts_plain_counters(self):
        c = coordinator_counters(parse_state_line(REAL_LINE))
        assert c['coord_dispatched'] == 5
        assert c['coord_succeeded'] == 3
        assert c['coord_rejected'] == 11

    def test_takes_numerator_of_current_over_limit(self):
        """nav_fail=0/5 里只有分子是计数，分母是上限。"""
        c = coordinator_counters(parse_state_line(REAL_LINE))
        assert c['coord_nav_fail'] == 0
        assert c['coord_validate_fail'] == 1

    def test_missing_field_is_none_not_zero(self):
        """**核心断言**：取不到必须是 None。

        0 会被当成"确实是 0 次"，而对账时"没这个字段"和"这个字段是 0"
        的含义完全不同 —— 前者说明串的格式变了，后者说明真的没失败过。
        """
        c = coordinator_counters({})
        assert c['coord_dispatched'] is None
        assert c['coord_nav_fail'] is None

    def test_non_numeric_is_none(self):
        c = coordinator_counters({'dispatched': 'n/a'})
        assert c['coord_dispatched'] is None

    def test_carries_replan_policy_through(self):
        """周期重规划下"没有一条路径被跟踪到位"，这一列要能看出来。"""
        c = coordinator_counters(parse_state_line(REAL_LINE))
        assert c['coord_replan_policy'] == 'on_invalid'


class TestInstalledArtifacts:
    """新文件必须真的被装进 install。"""

    @staticmethod
    def _setup_src():
        with open(SETUP_PY, encoding='utf-8') as fh:
            return fh.read()

    def test_recorder_has_a_console_script(self):
        src = self._setup_src()
        assert 'explore_metrics_recorder_node' in src, (
            'setup.py 的 console_scripts 里没有录制节点 —— '
            '`ros2 run astribot_s1_navigation explore_metrics_recorder_node` '
            '会报找不到')

    @pytest.mark.parametrize('script', [
        'scripts/run_speed_sweep.sh',
        'scripts/aggregate_speed_sweep.py',
    ])
    def test_scripts_exist_on_disk(self, script):
        assert os.path.isfile(os.path.join(_PKG, script)), script

    @pytest.mark.parametrize('script', [
        'scripts/run_speed_sweep.sh',
        'scripts/aggregate_speed_sweep.py',
    ])
    def test_scripts_are_listed_in_data_files(self, script):
        """**核心断言**：本包的 setup.py 没有 scripts/ 通配。

        不显式列出来时，文件躺在 src 里但 install 目录下永远没有它，
        而报错是"命令找不到"——很容易被当成环境问题。
        """
        assert script in self._setup_src(), (
            '%s 没有写进 setup.py 的 data_files —— 它永远不会被安装。'
            '本包只 glob launch/config/rviz/behavior_trees，没有 scripts/ 通配'
            % script)

    def test_scripts_are_executable(self):
        for s in ('scripts/run_speed_sweep.sh', 'scripts/aggregate_speed_sweep.py'):
            path = os.path.join(_PKG, s)
            assert os.access(path, os.X_OK), '%s 没有可执行位' % s

    def test_explore_metrics_is_a_package(self):
        """find_packages 只会收带 __init__.py 的目录。"""
        init = os.path.join(_PKG, 'astribot_s1_navigation', 'explore_metrics',
                            '__init__.py')
        assert os.path.isfile(init), (
            'explore_metrics 缺 __init__.py -> find_packages 收不到它 -> '
            '安装后 import 失败，而源码树里一切看起来正常')


class TestSweepScriptSafety:
    """扫描驱动的安全约束是硬要求，用测试钉住。"""

    @staticmethod
    def _src():
        with open(os.path.join(_PKG, 'scripts', 'run_speed_sweep.sh'),
                  encoding='utf-8') as fh:
            return fh.read()

    def test_observe_only_defaults_true(self):
        """**核心断言**：默认不使能写通路。"""
        src = self._src()
        m = re.search(r'OBSERVE_ONLY="\$\{OBSERVE_ONLY:-([a-z]+)\}"', src)
        assert m and m.group(1) == 'true', (
            'OBSERVE_ONLY 默认不是 true —— 脚本会在没有授权的情况下让机器人动')

    def test_enable_write_defaults_false(self):
        src = self._src()
        m = re.search(r'ENABLE_WRITE="\$\{ENABLE_WRITE:-([a-z]+)\}"', src)
        assert m and m.group(1) == 'false'

    def test_asks_for_explicit_confirmation(self):
        """每个档位使能前必须停下来要求确认，技术前置条件不构成授权。"""
        src = self._src()
        assert 'confirm_enable' in src
        assert "= 'YES'" in src, '确认没有要求一个明确的肯定输入'
        assert '不构成授权' in src, '确认提示里没有写明技术前置条件不等于授权'

    def test_readiness_is_lifecycle_not_process_count(self):
        """**核心断言**：就绪判据必须查生命周期，不能数进程。

        实测过 9 个进程全活而 lifecycle_manager 报 Aborting bringup。
        """
        src = self._src()
        assert 'get_state' in src, '没有查生命周期状态'
        assert 'lifecycle_msgs/srv/GetState' in src

    @staticmethod
    def _code():
        """剥掉注释行后的脚本正文。

        为什么必须剥：本文件里"禁止出现某个字符串"的断言会命中**说明为什么
        禁止它的那句注释**（实测过，第一次跑就在自己写的
        「曾硬编码 `FollowPath.vx_max`」这行上失败）。
        """
        with open(os.path.join(_PKG, 'scripts', 'run_speed_sweep.sh'),
                  encoding='utf-8') as fh:
            return '\n'.join(ln for ln in fh.read().splitlines()
                             if not ln.lstrip().startswith('#'))

    def test_verifies_measured_cap_before_recording(self):
        """限速必须在线核对，且**不能硬编码键名**。

        原断言是 `assert 'FollowPath.vx_max' in src`，它把 bug 锁死了：
        2026-09-07 起 FollowPath 是三段式控制器，MPPI 的限速键下移到
        `<实例>.inner.vx_max`，`FollowPath.vx_max` 这个键根本不存在。
        测试却在要求脚本必须查那个不存在的键 —— 判据恒失败而测试恒通过，
        正是 [[tests-can-lock-in-the-bug-they-should-catch]] 那一类。
        现在改成枚举 *.vx_max，所以断言方向反过来：禁止出现硬编码键名。
        """
        src = self._src()
        assert 'verify_cap' in src
        assert 'FollowPath.vx_max' not in self._code(), \
            '又硬编码了 FollowPath.vx_max —— 这个键在三段式控制器下不存在'
        assert 'param list /controller_server' in src, '没有枚举控制器参数'
        assert '.vx_max$' in src, '没有按 *.vx_max 过滤枚举结果'
        # 0 个键时"全部一致"是空真，必须有守卫。
        assert '一个 *.vx_max 参数都没枚举到' in src, '缺少空枚举守卫'

    def test_uses_full_ros2_cli_not_vendor_subset(self):
        """厂商的 /opt/astribot_ros/middle_ware/bin 在 PATH 里排前面，
        但它的 ros2 是子集，缺 lifecycle/param 子命令。"""
        src = self._src()
        assert '/opt/ros/humble/bin/ros2' in src

    def test_guards_set_u_around_ros_setup(self):
        """`set -u` 下 source setup.bash 会静默退出，一个字都不打印。"""
        src = self._src()
        assert 'set +u' in src and 'set -u' in src

    def test_cleanup_covers_semaphore_shm(self):
        """清理模式漏了 sem.fastrtps_* 时实测还剩几十个，伪装成"栈没起来"。"""
        src = self._src()
        assert 'sem.fastrtps_' in src
        assert 'daemon stop' in src, '陈旧 daemon 会让话题数从 80 掉到 2'

    def test_pkill_pattern_goes_through_a_variable(self):
        """`pkill -f` 的模式若逐字出现在脚本自己的命令行里会杀掉自己的 shell。

        这里检查模式是通过变量传入的，而不是把长串直接写在 pkill 后面。
        """
        src = self._src()
        assert 'pkill -f "$pats"' in src, (
            'pkill 的模式没有走变量 —— 直接写在命令行里时，模式会匹配到'
            '脚本自身的进程，循环会从中间静默断掉')

    def test_forwards_posture_monitor_off_on_hardware(self):
        """实机必须显式关掉姿态监控，它会永久把 /cmd_vel 归零且无复位路径。"""
        src = self._src()
        assert 'enable_posture_monitor:=false' in src


class TestRecorderSpeedParamNamespace:
    """录制器在线查限速的**命名空间**必须可配，且失败不许静默。

    这一组测的是 2026-09-07 实测到的缺陷：录制器每 5s 向 controller_server
    查 `FollowPath.vx_max`，而三段式控制器把内层 MPPI 的键放在
    `<实例>.inner` 下 —— 键不存在，rclcpp 的 ParameterService 捕获
    ParameterNotDeclaredException 后返回**空的 values 列表**，录制器旧代码
    用 `except Exception: pass` 吃掉，实测限速恒为 None，
    报表里「限速与请求一致」整列不可用（13 轮 0 轮有值）。
    """

    @staticmethod
    def _src():
        with open(os.path.join(_PKG, 'astribot_s1_navigation',
                               'explore_metrics_recorder_node.py'),
                  encoding='utf-8') as fh:
            return fh.read()

    def test_speed_namespace_is_a_declared_parameter(self):
        src = self._src()
        assert "d('controller_speed_ns'" in src, \
            '限速参数的命名空间没有做成可配参数'

    def test_does_not_hardcode_the_query_name(self):
        """查参名必须由命名空间拼出来，不能再逐字写死。"""
        src = self._src()
        assert "'FollowPath.vx_max'" not in src, \
            '又把 FollowPath.vx_max 写成查参名了 —— 这个键不存在'
        assert "'%s.vx_max' % _ns" in src, '查参名没有从命名空间参数拼出来'
        assert 'self.speed_param_names' in src

    def test_no_silent_failure_paths(self):
        """禁止静默失败（仓库铁律）：查参的失败路径都要显式告警。

        断言必须**限定在 `_param_result` 这个函数体内**。第一版写成
        "全文不许出现 `except Exception: ... # noqa: BLE001`"，结果命中了
        TF 查询那处 —— 那一处并不静默，它 `self.tf_fail += 1` 且在自检里
        逐条打印成功/失败数。全文级的禁止字符串断言会这样误报。
        """
        src = self._src()
        body = self._func_body(src, '_param_result')
        assert body, '找不到 _param_result —— 查参的结果分发路径被改名或删了'
        assert '_warn_caps' in body, \
            '_param_result 里没有任何告警通路（旧版是 except Exception: pass）'
        assert not re.search(r'except[^\n]*:\s*\n\s*pass', body), \
            '_param_result 还在裸吞异常'
        # 返回项数不等于请求项数 == 至少一个名字不存在，必须单独报。
        assert 'len(res.values) != len(names)' in body, \
            '没有校验返回项数 —— 服务端对未声明的名字返回**空的 values 列表**，'\
            '不校验就等于静默接受"什么都没查到"'

    @staticmethod
    def _func_body(src, name):
        """取出一个方法的函数体（到下一个同缩进的 def 为止）。"""
        m = re.search(r'\n(    )def %s\(' % re.escape(name), src)
        if not m:
            return ''
        rest = src[m.end():]
        nxt = re.search(r'\n    (?:@|def )', rest)
        return rest[:nxt.start()] if nxt else rest

    def test_warns_once_not_every_cycle(self):
        """查参是 5s 一次；不去重会 72 条/小时把真告警刷掉。"""
        src = self._src()
        assert '_caps_warned' in src

    def test_selfcheck_reports_the_column_is_unusable(self):
        """自检里要说清后果是"整列不可用"，不是"少一个数"。"""
        src = self._src()
        assert '整列不可用' in src
        assert 'controller_speed_ns:=' in src, '自检没给出修复动作'
