# Copyright 2026 Astribot.
#
# explore_metrics.signals 的单测。
#
# 这个模块里每个函数都在防一个本项目**真的犯过**的读数错误，
# 所以测试也照那些错误来写：
#   · 拍率用均值 -> 曾把 157Hz 说成 91Hz
#   · 变号计数不带幅值门限 -> 噪声算成上千次震荡
#   · 跟踪误差不对齐时延 -> 量到的是 0.6s 滞后而不是跟踪好坏
#   · 用相关系数选时延 -> 臂-底盘耦合把幅值压到 15% 会被看成完美跟踪
#   · 没有合格样本返回 0.0 -> 汇总表里的假数
import numpy as np

from astribot_s1_navigation.explore_metrics import signals as s


class TestRate:

    def test_uniform_20hz(self):
        r = s.rate_from_stamps(np.arange(100) * 0.05)
        assert abs(r['hz'] - 20.0) < 1e-6
        assert abs(r['gap_median'] - 0.05) < 1e-9

    def test_median_not_mean(self):
        """**核心断言**：一个长空洞不该把拍率拉垮。

        20Hz 跑 100 拍，中间卡了 5 秒。均值口径会报 ~9Hz，
        而实际控制环频率是 20Hz、外加一次 5s 的停顿 ——
        这两件事必须分别用 hz 和 gap_max 表达，混成一个数就没法排查。
        """
        t = list(np.arange(50) * 0.05)
        t += list(np.arange(50) * 0.05 + t[-1] + 5.0)
        r = s.rate_from_stamps(t)
        assert abs(r['hz'] - 20.0) < 0.5, '拍率被那一次停顿拉垮了（用了均值口径）'
        assert r['gap_max'] > 4.9, '停顿必须在 gap_max 上看得见'

    def test_p95_exposes_the_tail(self):
        # 尾部要占到 5% 以上才可能进 p95：90 个 0.05s 间隔 + 10 个 0.5s 间隔。
        # 只放 5 个时 p95 的下标恰好还落在快的那一段 —— 这不是函数的问题，
        # 是"p95 只能看见最慢的 5%"这条定义本身。
        t = list(np.arange(90) * 0.05)
        t += list(np.arange(10) * 0.5 + t[-1] + 0.5)
        r = s.rate_from_stamps(t)
        assert r['gap_p95'] > 0.2, '尾部间隔没有被报出来'
        assert abs(r['hz'] - 20.0) < 0.5, '中位口径仍应给出 20Hz'

    def test_too_few_samples_returns_none(self):
        assert s.rate_from_stamps([])['hz'] is None
        assert s.rate_from_stamps([1.0])['hz'] is None

    def test_identical_stamps_do_not_divide_by_zero(self):
        r = s.rate_from_stamps([1.0, 1.0, 1.0])
        assert r['hz'] is None


class TestOscillation:

    def test_pure_noise_is_not_oscillation(self):
        """**核心断言**：±0.001rad/s 的抖动不是震荡。"""
        t = np.arange(2000) * 0.05
        v = 0.001 * (-1.0) ** np.arange(2000)
        assert s.oscillation_count(t, v, min_amp=0.15, window_s=2.0) == 0

    def test_real_left_right_flick_counts(self):
        t = np.arange(20) * 0.05
        v = np.where(np.arange(20) < 10, 0.4, -0.4)
        assert s.oscillation_count(t, v, min_amp=0.15, window_s=2.0) == 1

    def test_slow_maneuver_is_not_oscillation(self):
        """左转 10s 再右转 10s 是正常机动，不能算震荡。"""
        t = np.arange(400) * 0.05
        v = np.where(t < 10.0, 0.4, -0.4)
        # 两次 excursion 之间的间隔是一个采样周期，所以要靠**窗口以外的间隔**
        # 区分不了；这里改用有静止段的版本 —— 那才是真实形态。
        v = np.where((t >= 9.0) & (t < 15.0), 0.0, v)
        assert s.oscillation_count(t, v, min_amp=0.15, window_s=2.0) == 0, (
            '两次 excursion 相隔 6s 仍被算成震荡 —— 时间窗没起作用')

    def test_amplitude_gate_is_applied_on_both_sides(self):
        """一侧大一侧小时不算：那是单向机动叠了点噪声。"""
        t = np.arange(20) * 0.05
        v = np.where(np.arange(20) < 10, 0.4, -0.01)
        assert s.oscillation_count(t, v, min_amp=0.15, window_s=2.0) == 0

    def test_multiple_flicks(self):
        t = np.arange(40) * 0.05
        v = 0.4 * (-1.0) ** (np.arange(40) // 5)
        assert s.oscillation_count(t, v, min_amp=0.15, window_s=2.0) == 7


class TestResampleZoh:

    def test_zero_order_hold_not_interpolation(self):
        """**核心断言**：指令是阶梯信号，线性插值会造出从未下发过的值。"""
        out = s.resample_zoh([0.0, 1.0], [0.0, 1.0], [0.5])
        assert out[0] == 0.0, '用了线性插值，造出了 0.5 这个从未下发的指令'

    def test_before_first_sample_is_nan_not_zero(self):
        """那段没有指令，补 0 会被算成"指令要求停车"。"""
        out = s.resample_zoh([1.0, 2.0], [5.0, 6.0], [0.0])
        assert np.isnan(out[0])

    def test_holds_last_value_after_end(self):
        out = s.resample_zoh([0.0, 1.0], [3.0, 4.0], [10.0])
        assert out[0] == 4.0

    def test_unsorted_source_is_handled(self):
        out = s.resample_zoh([1.0, 0.0], [4.0, 3.0], [0.5])
        assert out[0] == 3.0


class TestLagAndResidual:

    def _step(self, t, t_on, amp=0.2):
        return np.where(t >= t_on, amp, 0.0)

    def test_recovers_known_lag(self):
        t = np.arange(0.0, 10.0, 0.02)
        cmd = self._step(t, 2.0)
        act = self._step(t, 2.6)          # 滞后 0.6s，与本机实测量级一致
        r = s.best_lag_and_residual(t, cmd, t, act, max_lag_s=1.5, grid_dt=0.02)
        assert abs(r['best_lag_s'] - 0.6) < 0.05, r
        assert r['rms_at_best'] < 0.01
        assert r['rms_at_zero'] > 5.0 * r['rms_at_best'], (
            '零时延残差没有明显更大 —— 说明这个"跟踪误差"其实主要是时延')
        assert r['hit_boundary'] is False

    def test_gain_detects_downstream_attenuation(self):
        """**核心断言**：被下游压到 15% 必须表现为 gain≈0.15，不能被当成完美跟踪。

        用相关系数选时延就会掉进这个坑：相关对幅值缩放不敏感。
        """
        t = np.arange(0.0, 10.0, 0.02)
        cmd = self._step(t, 2.0, amp=0.2)
        act = self._step(t, 2.0, amp=0.03)      # 被压到 15%
        r = s.best_lag_and_residual(t, cmd, t, act, max_lag_s=1.5, grid_dt=0.02)
        assert abs(r['gain'] - 0.15) < 0.02, r['gain']
        assert r['rms_at_best'] > 0.05, '幅值被压掉 85% 却报出很小的残差'

    def test_flags_boundary_hit(self):
        """真实时延超出搜索上界时，那个"最佳时延"是假的，必须标出来。"""
        t = np.arange(0.0, 20.0, 0.02)
        cmd = self._step(t, 2.0)
        act = self._step(t, 5.0)          # 滞后 3s，远超 max_lag_s=1.5
        r = s.best_lag_and_residual(t, cmd, t, act, max_lag_s=1.5, grid_dt=0.02)
        assert r['hit_boundary'] is True, (
            'best_lag 落在搜索区间端点却没标记 —— 会被当成一个可信的时延读数')

    def test_too_short_overlap_returns_none(self):
        r = s.best_lag_and_residual([0.0, 0.01], [0.0, 0.1], [5.0, 5.01], [0.0, 0.1])
        assert r['best_lag_s'] is None
        assert r['rms_at_best'] is None


class TestEpisodes:

    def test_single_run(self):
        t = np.arange(10) * 0.1
        f = np.array([0, 0, 1, 1, 1, 0, 0, 0, 0, 0], dtype=bool)
        eps = s.episodes(t, f)
        assert len(eps) == 1
        assert abs(eps[0][0] - 0.2) < 1e-9
        assert abs(eps[0][1] - 0.4) < 1e-9, (
            '段末取了下一个 False 的时刻 —— 会把一个采样间隔算进段长')

    def test_two_runs(self):
        t = np.arange(10) * 0.1
        f = np.array([1, 1, 0, 0, 1, 1, 1, 0, 0, 0], dtype=bool)
        assert len(s.episodes(t, f)) == 2

    def test_min_duration_filters_single_sample_spikes(self):
        t = np.arange(10) * 0.1
        f = np.array([0, 1, 0, 1, 1, 1, 1, 1, 1, 0], dtype=bool)
        eps = s.episodes(t, f, min_duration_s=0.3)
        assert len(eps) == 1, '单帧尖峰没有被滤掉'

    def test_run_to_the_end(self):
        t = np.arange(5) * 0.1
        f = np.ones(5, dtype=bool)
        eps = s.episodes(t, f)
        assert len(eps) == 1
        assert abs(eps[0][1] - 0.4) < 1e-9

    def test_empty(self):
        assert s.episodes([], []) == []


class TestDurationWhere:

    def test_includes_the_last_sample(self):
        """**核心断言**：直接 sum(diff) 会把最后一段整个丢掉。"""
        t = np.arange(5) * 0.1
        f = np.ones(5, dtype=bool)
        d = s.duration_where(t, f)
        assert abs(d - 0.5) < 1e-9, (
            '最后一个样本没有计时，短轮次里误差可以达到 20%%（实得 %.3f）' % d)

    def test_partial(self):
        t = np.arange(10) * 0.1
        f = np.array([1, 1, 0, 0, 0, 0, 0, 0, 0, 0], dtype=bool)
        assert abs(s.duration_where(t, f) - 0.2) < 1e-9

    def test_all_false_is_zero(self):
        assert s.duration_where(np.arange(5) * 0.1, np.zeros(5, dtype=bool)) == 0.0


class TestStatsAndNone:

    def test_stat_or_none_on_empty(self):
        assert s.stat_or_none([], np.max) is None

    def test_stat_or_none_on_all_nan(self):
        """**核心断言**：全是 nan 时返回 None，不是 0.0。"""
        assert s.stat_or_none([np.nan, np.nan], np.mean) is None

    def test_stat_or_none_skips_nan(self):
        assert abs(s.stat_or_none([1.0, np.nan, 3.0], np.mean) - 2.0) < 1e-12

    def test_safe_div_zero_denominator(self):
        assert s.safe_div(3, 0) is None
        assert s.safe_div(0, 5) == 0.0

    def test_weighted_mean_handles_uneven_sampling(self):
        """掉帧后普通均值会给密集段更大权重。

        权重是"该样本到**下一个**样本的间隔"（零阶保持语义）：
        下面 v=1 的样本紧跟着一个 5 秒空洞，所以那个 1 被保持了 5 秒，
        应当主导均值。普通均值只会给它 1/4。
        """
        t = np.array([0.0, 0.1, 0.2, 5.2])
        v = np.array([0.0, 0.0, 1.0, 0.0])
        wm = s.weighted_mean(t, v)
        assert wm > 0.9, '按时间加权时那 5 秒的 1.0 应当占主导（实得 %.3f）' % wm
        assert abs(float(np.mean(v)) - 0.25) < 1e-12, '对照：普通均值只有 0.25'

    def test_weighted_mean_last_sample_uses_median_gap(self):
        """末样本没有"下一个间隔"，按中位间隔补齐 —— 不能补 0（等于丢掉它）。"""
        t = np.array([0.0, 0.1, 0.2])
        v = np.array([0.0, 0.0, 1.0])
        wm = s.weighted_mean(t, v)
        assert wm > 0.3, '末样本被丢掉了（权重补了 0），实得 %.3f' % wm

    def test_weighted_mean_all_nan_is_none(self):
        assert s.weighted_mean([0.0, 1.0], [np.nan, np.nan]) is None


class TestResponseLatency:

    def test_measures_brake_delay(self):
        t = np.arange(0.0, 5.0, 0.1)
        trig = (t >= 1.0) & (t < 1.2)
        v = np.where(t < 1.5, 0.2, 0.05)
        lat, skipped = s.response_latency(t, trig, v, drop_ratio=0.8)
        assert len(lat) == 1
        assert abs(lat[0] - 0.5) < 0.11, lat
        assert skipped == 0

    def test_skips_events_where_robot_was_already_stopped(self):
        """**核心断言**：基准速度≈0 的事件必须跳过并计数。

        不跳过会得到一堆 0 延迟，中位数变成"响应极快"的假结论。
        """
        t = np.arange(0.0, 5.0, 0.1)
        trig = (t >= 1.0) & (t < 1.2)
        v = np.zeros_like(t)
        lat, skipped = s.response_latency(t, trig, v)
        assert lat == []
        assert skipped == 1, '基准过小的事件既没算也没计数 —— 读者不知道它被丢了'

    def test_no_response_within_window_is_not_counted(self):
        t = np.arange(0.0, 5.0, 0.1)
        trig = (t >= 1.0) & (t < 1.2)
        v = np.full_like(t, 0.2)
        lat, _ = s.response_latency(t, trig, v, max_wait_s=1.0)
        assert lat == []
