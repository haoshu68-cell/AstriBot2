# Copyright 2026 Astribot.
#
# 逐轮采样落盘的**跨轮污染**回归测试。
#
# 这个 bug 在 run11 上毁掉了 18 轮里的 9 轮，而表面上一切正常：
# 每个通道的拍率都是对的（pose 21.8Hz、odom 52Hz），rounds.csv 每一列都有值，
# 没有任何告警。唯一的破口是 pose_coverage_ratio 偏低，以及
# distance_travelled_m 恒 0.000。
#
# 机制：_close_round(settle=True) 不立刻落盘，挂到 pending_settle 上等
# settle_window 秒；而 _build_raw 读的是 self.pose / self.cmd_* 这些**活缓冲**。
# 若这几秒里下一个目标到了，_open_round 会 clear() 掉缓冲并重新填，
# 于是落盘时写进"旧轮"文件的是**新轮**开头那几百毫秒的样本。
#
# 所以这里的断言必须按**时序**来，不能只调一次函数看返回值 ——
# 单独调 _build_raw 时缓冲还没被清，那条路径永远是绿的。
import os

import pytest

rclpy = pytest.importorskip(
    'rclpy',
    reason='录制器节点模块 import rclpy；本用例只用它的类不起节点')

from astribot_s1_navigation.explore_metrics_recorder_node import (  # noqa: E402
    ExploreMetricsRecorder, Series)


class _FakeLogger:
    def __init__(self):
        self.warns = []
        self.errors = []

    def info(self, m):
        pass

    def warn(self, m):
        self.warns.append(m)

    def error(self, m):
        self.errors.append(m)


class _Goal:
    """够 _open_round 用的最小 PoseStamped 替身。"""

    class _H:
        frame_id = 'map'

    class _P:
        class _Pos:
            x = 1.0
            y = 2.0
        class _Q:
            x = 0.0
            y = 0.0
            z = 0.0
            w = 1.0
        position = _Pos()
        orientation = _Q()

    header = _H()
    pose = _P()


def _make_recorder():
    """不起 ROS 节点，直接造一个只带本用例所需字段的录制器。

    绕过 __init__ 是有意的：__init__ 会建 10 个订阅、TF listener 和
    参数客户端，那些跟本用例要验的时序无关，拉进来只会让用例变脆。
    """
    r = object.__new__(ExploreMetricsRecorder)
    for name in ('pose', 'cmd_raw', 'cmd_final', 'odom', 'settle'):
        setattr(r, name, Series())
    r.pose_yaw = []
    r.settle_yaw = []
    r.scan_t = []
    r.scan_frames = []
    r.plans = []
    r.plan_history = []
    r.round = None
    r.round_index = 0
    r.pending_settle = None
    r.flushed = []
    r.fp_changed_in_round = False
    r.fp_inscribed = 0.4
    r.fp_circumscribed = 0.43
    r.goal_tol = 0.25
    r.map_frame = 'map'
    r.settle_window = 3.0
    r.measured = {'vx_max': 1.0, 'vx_min': -1.0, 'smoother_max': 1.0}
    r.speed_cap_requested = -1.0
    r.collisions_manual = 0
    r.low_obstacle_truth = 0
    r._logger = _FakeLogger()
    r.get_logger = lambda: r._logger
    # 时间由用例驱动，不用挂钟 —— 挂钟会让"3 秒内"这个条件不可复现
    r._t = 0.0
    r._now = lambda: r._t
    r._lookup_pose = lambda: None          # 关窗时不再补一帧，样本集保持干净
    r._clock_source = lambda: 'system'
    r._use_sim_time = lambda: False
    r._round_plan_reqs = lambda _r: []
    r._flush = lambda raw_round: r.flushed.append(r._build_raw(raw_round))
    return r


def _fill(r, t0, n, step=0.05):
    """给轮内缓冲灌 n 个样本，时间从 t0 起。"""
    for i in range(n):
        t = t0 + i * step
        r._t = t
        r.pose.add(t, float(i), 0.0)
        r.pose_yaw.append(0.0)
        r.odom.add(t, 0.5, 0.0)


class TestRoundSampleIsolation:

    def test_deferred_flush_keeps_its_own_samples(self):
        """**核心断言**：settle 窗内来了新目标，旧轮落盘的仍是旧轮的样本。

        这一条就是 run11 那次数据损坏。断言写成"样本时间必须落在
        [goal_stamp, end_stamp] 里"，而不是只比样本个数 —— 个数在
        坏实现下也可能凑巧对上，时间落在窗口之外才是无法伪装的。
        """
        r = _make_recorder()
        r._open_round(_Goal())
        _fill(r, t0=1.0, n=20)              # 轮 0：t = 1.00 ~ 1.95
        r._t = 2.0
        r._close_round('ARRIVED', settle=True)

        # 驻留窗还没走完，新目标到达 —— 这一步在坏实现里会清空缓冲
        r._t = 2.5
        r._open_round(_Goal())
        _fill(r, t0=2.5, n=10)              # 轮 1：t = 2.50 ~ 2.95

        assert len(r.flushed) == 1, '旧轮应当在新轮开窗时就已落盘'
        raw = r.flushed[0]
        assert raw['index'] == 0
        ts = raw['pose_t']
        assert ts, '轮 0 的样本不能是空的'
        assert min(ts) >= raw['goal_stamp'], (
            '轮 0 的样本早于它自己的开窗时刻：%.2f < %.2f' % (min(ts), raw['goal_stamp']))
        assert max(ts) <= raw['end_stamp'], (
            '轮 0 的样本落在窗口关闭之后（run11 实测 107%%~146%%）：'
            '%.2f > %.2f' % (max(ts), raw['end_stamp']))
        assert len(ts) == 20

    def test_second_round_still_gets_its_own_samples(self):
        """修复不能反过来把新轮的样本弄丢。"""
        r = _make_recorder()
        r._open_round(_Goal())
        _fill(r, t0=1.0, n=20)
        r._t = 2.0
        r._close_round('ARRIVED', settle=True)
        r._t = 2.5
        r._open_round(_Goal())
        _fill(r, t0=2.5, n=10)
        r._t = 3.6
        r._close_round('ARRIVED', settle=False)

        assert len(r.flushed) == 2
        raw1 = r.flushed[1]
        assert raw1['index'] == 1
        assert len(raw1['pose_t']) == 10
        assert min(raw1['pose_t']) >= 2.5

    def test_interrupted_settle_window_is_flagged_not_silent(self):
        """驻留窗被打断要显式标记 —— 静置漂移会因此偏小，不能让人当完整值读。"""
        r = _make_recorder()
        r._open_round(_Goal())
        _fill(r, t0=1.0, n=20)
        r._t = 2.0
        r._close_round('ARRIVED', settle=True)
        # 驻留窗里采到 2 帧就被打断
        for t in (2.1, 2.2):
            r._t = t
            r.settle.add(t, 0.0, 0.0)
            r.settle_yaw.append(0.0)
        r._t = 2.5
        r._open_round(_Goal())

        raw = r.flushed[0]
        assert raw['settle_truncated'] is True
        assert len(raw['settle_t']) == 2, '被打断前采到的那几帧要保留'
        assert any('驻留窗被新目标打断' in m for m in r._logger.warns), \
            '截断必须有告警，不能静默'

    def test_undisturbed_round_is_not_flagged(self):
        """没被打断的轮不能被误标成截断（否则这个标记就没有信息量了）。"""
        r = _make_recorder()
        r._open_round(_Goal())
        _fill(r, t0=1.0, n=20)
        r._t = 2.0
        r._close_round('ARRIVED', settle=True)
        for i in range(60):                 # 完整 3s 驻留窗
            t = 2.0 + i * 0.05
            r._t = t
            r.settle.add(t, 0.0, 0.0)
            r.settle_yaw.append(0.0)
        r._t = 5.1
        r._tick()                           # 驻留窗到点，正常落盘

        assert len(r.flushed) == 1
        assert r.flushed[0]['settle_truncated'] is False
        assert len(r.flushed[0]['settle_t']) == 60


class TestSnapshotIsMandatory:

    def test_missing_snapshot_reports_error(self):
        """没有快照时必须显式报错，不能静默产出一行看起来正常的假数据。"""
        r = _make_recorder()
        raw = r._build_raw({'index': 7, 'goal': (0.0, 0.0, 0.0),
                            'goal_stamp': 0.0, 'end_stamp': 1.0,
                            'outcome': 'ARRIVED'})
        # 顺序要紧：先判键不在，再取值。反过来写 raw['pose_t'] 会先求值抛 KeyError
        assert 'pose_t' not in raw or raw['pose_t'] == []
        assert any('没有采样快照' in m for m in r._logger.errors)
