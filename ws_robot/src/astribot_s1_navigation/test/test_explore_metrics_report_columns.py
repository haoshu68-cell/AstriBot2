# Copyright 2026 Astribot.
#
# 锁住"报告与判据之间的口径"这一层。这里的每条测试都对应一个**已经发生过**的
# 静默失效，不是假想的：
#
#   1. summarize() 拿到 CSV 字符串 -> 'False' 是非空字符串 -> success_rate 恒 1.0
#   2. 报告问的列名不存在（path_length_m / cross_track_p95_m）-> 恒报 n=0
#      "没测到"，而数据一直都在
#   3. 位姿覆盖率不足的轮次照样出"到位误差"（实测 1 个样本出 0.083m）
#   4. 轮数对账拿"开机累计"比"本会话计数" -> 结构上恒不一致，红旗恒亮
#
# 四个症状的共同点：**一个都不报错**，表面看起来完全正常。
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from astribot_s1_navigation.explore_metrics import round_metrics as rm   # noqa: E402


# --------------------------------------------------------------- CSV 类型还原
class TestCoerceCsvRow:
    """CSV 全是字符串。还原不对，summarize() 会静默给出错的数。"""

    def test_false_string_becomes_real_false(self):
        # 这是那个恒 1.0 的成功率的根：'False' 布尔求值为真
        assert bool('False') is True                     # 先把前提摆出来
        out = rm.coerce_csv_row({'success': 'False'})
        assert out['success'] is False

    def test_true_string_becomes_real_true(self):
        assert rm.coerce_csv_row({'success': 'True'})['success'] is True

    def test_numbers_become_floats(self):
        out = rm.coerce_csv_row({'arrival_error_xy_m': '0.1404'})
        assert out['arrival_error_xy_m'] == pytest.approx(0.1404)

    def test_empty_string_becomes_none_not_zero(self):
        # "没测到"必须是 None。填 0 会让"0 次事件"与"没采到样"混成一个数，
        # 而这两者在结论上是相反的
        assert rm.coerce_csv_row({'brake_latency_s_proxy': ''})[
            'brake_latency_s_proxy'] is None

    def test_text_columns_survive(self):
        assert rm.coerce_csv_row({'outcome': 'ARRIVED'})['outcome'] == 'ARRIVED'

    def test_nan_becomes_none(self):
        assert rm.coerce_csv_row({'x': 'nan'})['x'] is None
        assert rm.coerce_csv_row({'x': 'inf'})['x'] is None

    def test_non_string_values_pass_through(self):
        # compute_round 直接产出的行本来就是带类型的，过一遍不能被改坏
        out = rm.coerce_csv_row({'a': 1.5, 'b': True, 'c': None})
        assert out == {'a': 1.5, 'b': True, 'c': None}


class TestSummarizeRejectsRawCsv:
    """宁可报错，也不发一个看着正常的 success_rate。"""

    def test_string_rows_raise_instead_of_lying(self):
        rows = [{'success': 'True', 'counter_mismatch': 'False'},
                {'success': 'False', 'counter_mismatch': 'False'}]
        with pytest.raises(ValueError, match='coerce_csv_row'):
            rm.summarize(rows)

    def test_coerced_rows_give_the_true_success_rate(self):
        rows = [rm.coerce_csv_row(r) for r in
                [{'success': 'True', 'counter_mismatch': 'False',
                  'arrival_error_xy_m': '0.10'},
                 {'success': 'False', 'counter_mismatch': 'False',
                  'arrival_error_xy_m': '0.20'},
                 {'success': 'True', 'counter_mismatch': 'False',
                  'arrival_error_xy_m': '0.30'},
                 {'success': 'True', 'counter_mismatch': 'False',
                  'arrival_error_xy_m': '0.40'}]]
        s = rm.summarize(rows)
        assert s['succeeded'] == 3
        assert s['success_rate'] == pytest.approx(0.75)   # 不是 1.0

    def test_numeric_columns_actually_participate(self):
        # 字符串行的第二个症状：isinstance(v, (int,float)) 全部落选，
        # 122 列一列都进不了统计，②节只剩三行 —— 这个是看得见的，
        # 但和恒 1.0 是同一个根因，一起锁住
        rows = [rm.coerce_csv_row(r) for r in
                [{'success': 'True', 'counter_mismatch': 'False',
                  'arrival_error_xy_m': '0.10'},
                 {'success': 'True', 'counter_mismatch': 'False',
                  'arrival_error_xy_m': '0.30'}]]
        s = rm.summarize(rows)
        assert s['arrival_error_xy_m__n'] == 2
        assert s['arrival_error_xy_m__mean'] == pytest.approx(0.20)

    def test_empty_rows_still_fine(self):
        assert rm.summarize([]) == {'rounds': 0}


# ------------------------------------------------------------------ 位姿覆盖率
def _cov(pose_t, duration_s):
    row = {'duration_s': duration_s}
    rm._pose_coverage({'pose_t': pose_t}, row)
    return row


class TestPoseCoverage:
    """位姿覆盖率：所有由位姿导出的列都靠它定生死。"""

    def test_single_sample_round_is_flagged(self):
        # 实测轮 2：1 个样本、时长 7.12s，却出了 0.083m 的"到位误差"。
        # 单样本算不出到位，它只是一张快照
        row = _cov([3715.0], 7.12)
        assert row['pose_coverage_suspicious'] is True
        assert row['pose_span_s'] is None
        assert row['pose_coverage_ratio'] is None

    def test_sparse_round_is_flagged(self):
        # 实测轮 1：10 个样本跨 0.45s，而时长 7.18s -> 覆盖率 6.3%
        pt = [100.0 + 0.05 * i for i in range(10)]
        row = _cov(pt, 7.18)
        assert row['pose_coverage_ratio'] == pytest.approx(0.45 / 7.18, rel=1e-3)
        assert row['pose_coverage_suspicious'] is True

    def test_full_round_is_not_flagged(self):
        # 实测轮 0：330 个样本 / 16.435s 跨度 / 16.44s 时长
        pt = [100.0 + 16.435 * i / 329 for i in range(330)]
        row = _cov(pt, 16.44)
        assert row['pose_coverage_ratio'] == pytest.approx(0.9997, abs=1e-3)
        assert row['pose_coverage_suspicious'] is False
        assert row['pose_rate_measured_hz'] == pytest.approx(20.08, abs=0.1)

    def test_measured_rate_matches_configured_20hz(self):
        # 实测频率 20.08 / 22.2 / 20.14 Hz vs 配置 pose_rate_hz=20.0。
        # 这一条是关键判据：频率对得上 => 轮 1/2 的缺样**不是掉帧**，
        # 而是样本落在轮次窗口之外（轮次开启时刻晚于它记录的 goal_stamp）
        pt = [0.05 * i for i in range(201)]
        row = _cov(pt, 10.0)
        assert row['pose_rate_measured_hz'] == pytest.approx(20.1, abs=0.2)

    def test_boundary_at_half_coverage(self):
        pt = [0.0, 5.0]
        assert _cov(pt, 10.0)['pose_coverage_suspicious'] is False   # 恰好 0.5
        assert _cov(pt, 10.01)['pose_coverage_suspicious'] is True

    def test_zero_duration_does_not_divide_by_zero(self):
        row = _cov([1.0, 2.0], 0.0)
        assert row['pose_coverage_ratio'] is None

    def test_empty_pose_is_flagged(self):
        assert _cov([], 5.0)['pose_coverage_suspicious'] is True


# ------------------------------------------------------- 报告要问的列必须存在
class TestReportAsksForColumnsThatExist:
    """报告问不存在的列名，症状与"传感器没数据"完全一样：n=0。

    ROS 那边参数名写错是静默忽略，CSV 这边列名写错是静默 n=0 —— 同一类坑。
    这条测试把 KEY_METRICS 里的每个列名都对着 compute_round 的真实输出核一遍。
    """

    @staticmethod
    def _report_key_metrics():
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.abspath(__file__)))))), 'tools'))
        import report_explore_metrics as rep
        return rep.KEY_METRICS

    @staticmethod
    def _real_columns():
        """跑一轮最小 raw，拿到 compute_round 真实产出的列名。"""
        raw = {
            'index': 0, 'outcome': 'ARRIVED',
            'goal': (1.0, 0.0, 0.0), 'goal_stamp': 0.0, 'end_stamp': 5.0,
            'pose_t': [0.0, 1.0, 2.0], 'pose_x': [0.0, 0.5, 1.0],
            'pose_y': [0.0, 0.0, 0.0], 'pose_yaw': [0.0, 0.0, 0.0],
            'plans': [{'stamp': 0.0, 'points': [(0.0, 0.0), (1.0, 0.0)]}],
            'scan_t': [0.0, 1.0],
            'scan_frames': [], 'plan_requests': [],
            'footprint_inscribed': 0.3992, 'footprint_circumscribed': 0.4342,
        }
        return set(rm.compute_round(raw))

    def test_every_key_metric_column_exists(self):
        cols = self._real_columns()
        missing = [k for k, _lab, _c in self._report_key_metrics()
                   if k not in cols]
        assert not missing, (
            '报告要的这些列 compute_round 从来不产出：%s —— '
            '症状是恒 n=0"没测到"，而不是报错' % missing)

    def test_every_companion_column_exists(self):
        cols = self._real_columns()
        missing = [c for _k, _lab, c in self._report_key_metrics()
                   if c is not None and c not in cols]
        assert not missing, '伴随计数列不存在：%s' % missing

    def test_the_two_names_that_never_existed_are_gone(self):
        # 这两个名字曾经写在报告里，而 compute_round 从不产出它们
        keys = {k for k, _lab, _c in self._report_key_metrics()}
        assert 'path_length_m' not in keys, '真名是 traveled_m'
        cols = self._real_columns()
        assert 'cross_track_p95_m' in cols, 'p95 这一列现在必须真的存在'

    def test_cross_track_p95_is_not_an_alias_of_max(self):
        # 关键性质：p95 必须**抗单样本极值**，而 max 不抗。
        # 造一条"20 个小偏差 + 1 个大尖峰"的轨迹：
        #   max 被尖峰主导 = 1.0，p95 应该仍然贴近那串小偏差。
        #
        # ⚠ 不要断言 mean <= p95：这条在一般分布下成立，但在
        # "尖峰+一串零"下**不成立**（实测 mean=0.0476 > p95≈0），
        # 我第一版就是这么写错的。p95 与 max 的关系才是这一列要保证的。
        raw = {
            'index': 0, 'outcome': 'ARRIVED',
            'goal': (3.0, 0.0, 0.0), 'goal_stamp': 0.0, 'end_stamp': 3.0,
            'pose_t': [float(i) * 0.1 for i in range(21)],
            'pose_x': [float(i) * 0.15 for i in range(21)],
            'pose_y': [0.02] * 20 + [1.0],
            'pose_yaw': [0.0] * 21,
            'plans': [{'stamp': 0.0, 'points': [(0.0, 0.0), (3.0, 0.0)]}],
            'scan_t': [], 'scan_frames': [], 'plan_requests': [],
            'footprint_inscribed': 0.3992, 'footprint_circumscribed': 0.4342,
        }
        row = rm.compute_round(raw)
        assert row['cross_track_max_m'] == pytest.approx(1.0, abs=1e-6)
        assert row['cross_track_p95_m'] is not None
        # 尖峰没有主导 p95：它还在小偏差那一档
        assert row['cross_track_p95_m'] < 0.5, (
            'p95 被单个极值主导了，那它就只是 max 的别名')
        assert row['cross_track_p95_m'] >= 0.02

    def test_cross_track_p95_tracks_a_spread_distribution(self):
        # 偏差单调铺开时，p95 必须落在 mean 与 max 之间 —— 这才是它作为
        # "能比档位的那个数"的意义
        n = 21
        raw = {
            'index': 0, 'outcome': 'ARRIVED',
            'goal': (3.0, 0.0, 0.0), 'goal_stamp': 0.0, 'end_stamp': 3.0,
            'pose_t': [float(i) * 0.1 for i in range(n)],
            'pose_x': [float(i) * 0.15 for i in range(n)],
            'pose_y': [0.01 * i for i in range(n)],
            'pose_yaw': [0.0] * n,
            'plans': [{'stamp': 0.0, 'points': [(0.0, 0.0), (3.0, 0.0)]}],
            'scan_t': [], 'scan_frames': [], 'plan_requests': [],
            'footprint_inscribed': 0.3992, 'footprint_circumscribed': 0.4342,
        }
        row = rm.compute_round(raw)
        assert row['cross_track_mean_m'] <= row['cross_track_p95_m']
        assert row['cross_track_p95_m'] <= row['cross_track_max_m']


# ------------------------------------------------------------ 代理量定义要跟上
class TestProxyDefinitionsMatchTheCode:
    """定义随数据落盘。定义与代码漂开时，没有任何报错。"""

    def test_narrow_definition_no_longer_adds_the_robot(self):
        d = rm.PROXY_DEFINITIONS['narrow_success_rate_proxy']
        assert 'left_min+right_min)' in d
        assert '+2*外接半径' not in d.split('⚠')[0], (
            '定义里还写着被否证的旧公式（多加一遍机器人直径）')

    def test_narrow_definition_warns_about_the_old_formula(self):
        # 只删掉不够：得留下"为什么不能加"，否则下一个人会再加回去
        d = rm.PROXY_DEFINITIONS['narrow_success_rate_proxy']
        assert '墙到墙' in d and '0.868' in d

    def test_intrusion_definition_names_its_geometry(self):
        d = rm.PROXY_DEFINITIONS['geometric_intrusion_episodes_proxy']
        assert '内切' in d
        # 最近回波的方位角没记录 => 落在内切与外接之间时不可判定，
        # 这一点必须写在定义里，否则 0 会被读成"没有侵入八边形"
        assert '方位角' in d and '不可判定' in d


class TestNarrowFormulaIsWallToWall:
    """定义改了，实现也必须是这个口径 —— 两者只对上一个不算。"""

    def test_two_walls_at_0_8_is_below_1_62(self):
        from astribot_s1_navigation.explore_metrics import scan_metrics
        # 左右各 0.80m -> 净宽 1.60 < 1.62 -> 窄。
        # 旧口径 +2*外接半径(0.868) 会变成 2.468 -> 判成不窄，四个 narrow_* 列全空
        n = 360
        st = scan_metrics.frame_stats(
            ranges=[0.80] * n,
            angle_min=-math.pi, angle_increment=2.0 * math.pi / n,
            range_min=0.05, range_max=10.0, circumscribed=0.4342)
        assert st['free_width'] == pytest.approx(1.60, abs=1e-6)
        assert st['free_width'] < rm.DEFAULTS['narrow_width_m']

    def test_free_width_does_not_depend_on_the_footprint(self):
        # 净宽是墙到墙的几何量，与机器人多大无关。传不传 circumscribed 都一样 ——
        # 旧实现把它当成公式的一部分，这条能直接抓住回退
        from astribot_s1_navigation.explore_metrics import scan_metrics
        n = 360
        kw = dict(ranges=[0.80] * n, angle_min=-math.pi,
                  angle_increment=2.0 * math.pi / n,
                  range_min=0.05, range_max=10.0)
        assert (scan_metrics.frame_stats(**kw)['free_width']
                == pytest.approx(
                    scan_metrics.frame_stats(circumscribed=0.4342, **kw)[
                        'free_width']))
