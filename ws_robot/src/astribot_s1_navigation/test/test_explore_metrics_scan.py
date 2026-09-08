# Copyright 2026 Astribot.
#
# explore_metrics.scan_metrics 的单测。
#
# 钉住四条最容易静默出错的：
#   · range 里的 0 是**无效标记**，当成 0m 障碍会让"最近障碍距离"恒为 0
#   · 左右扇区不能互换（互换后居中度符号反了，而绝对值统计看不出来）
#   · 开阔地不能算居中度，且合格样本占比必须一起报
#   · 窄段数为 0 时穿越成功率必须是 None，不是 1.0
import math

import numpy as np

from astribot_s1_navigation.explore_metrics import scan_metrics as sm

# 四束扫描，角度序 [-180°, -90°, 0°, +90°] = 后/右/前/左。
# 这四个角度刚好各落在一个扇区里（后向不属于任何扇区），
# 因此能单独验证每个扇区的取值，不会互相污染。


def _f4(back, right, front, left, **kw):
    """角度序 [-180, -90, 0, +90] 对应 后/右/前/左。"""
    ranges = np.array([back, right, front, left], dtype=float)
    kw.setdefault('range_min', 0.05)
    kw.setdefault('range_max', 20.0)
    return sm.frame_stats(ranges, -math.pi, math.pi / 2.0, **kw)


class TestValidMask:

    def test_zero_is_invalid_not_zero_distance(self):
        """**核心断言**：0 是无效标记。

        当成 0m 障碍的后果：最近障碍距离恒为 0、净空恒为负、
        "几何侵入"每一帧都触发 —— 整张安全相关的列全废。
        """
        m = sm.valid_mask([0.0, 1.0], 0.05, 20.0)
        assert m.tolist() == [False, True]

    def test_inf_and_nan_are_invalid(self):
        m = sm.valid_mask([np.inf, np.nan, 1.0], 0.05, 20.0)
        assert m.tolist() == [False, False, True]

    def test_out_of_band_is_invalid(self):
        m = sm.valid_mask([0.01, 25.0, 1.0], 0.05, 20.0)
        assert m.tolist() == [False, False, True]


class TestFrameStats:

    def test_sector_assignment_is_not_swapped(self):
        """**核心断言**：左右扇区不得互换。

        互换后 center_bias 整体反号。而汇总只报 |bias| 的均值/最大值，
        符号错了完全看不出来 —— 这类错误只能靠这条测试拦。
        """
        f = _f4(back=5.0, right=0.8, front=3.0, left=1.5)
        assert abs(f['left_min'] - 1.5) < 1e-12, '左扇区取到了右边的光束'
        assert abs(f['right_min'] - 0.8) < 1e-12, '右扇区取到了左边的光束'
        assert abs(f['front_min'] - 3.0) < 1e-12

    def test_center_bias_sign_means_more_room_on_the_left(self):
        f = _f4(back=5.0, right=0.5, front=3.0, left=1.5)
        assert f['center_bias'] > 0, 'bias 为正应当表示左侧更宽敞'

    def test_centered_corridor_has_zero_bias(self):
        f = _f4(back=5.0, right=1.0, front=3.0, left=1.0)
        assert abs(f['center_bias']) < 1e-12

    def test_open_space_is_not_in_corridor(self):
        """**核心断言**：开阔地不能算居中度。

        两侧都是 8m 时 (8-8)/(8+8)=0，会被读成"居中完美"。
        """
        f = _f4(back=9.0, right=8.0, front=9.0, left=8.0)
        assert f['in_corridor'] is False
        assert f['center_bias'] is None, '开阔地给出了 center_bias 数值'

    def test_min_range_ignores_invalid(self):
        f = _f4(back=0.0, right=np.inf, front=1.2, left=np.nan)
        assert abs(f['min_range'] - 1.2) < 1e-12

    def test_all_invalid_returns_none_not_zero(self):
        f = _f4(back=0.0, right=np.inf, front=np.nan, left=0.0)
        assert f['min_range'] is None
        assert f['valid_ratio'] == 0.0

    def test_valid_ratio(self):
        f = _f4(back=1.0, right=1.0, front=np.inf, left=np.nan)
        assert abs(f['valid_ratio'] - 0.5) < 1e-12

    def test_self_residual_counts_beams_inside_envelope(self):
        """自滤残留：物理上不可能有回波落在自己的外接包络内。"""
        f = _f4(back=0.30, right=1.0, front=1.0, left=1.0, circumscribed=0.42)
        assert f['self_residual'] == 1

    def test_self_residual_is_none_without_footprint(self):
        """拿不到足迹时必须是 None，不能报 0（那会被读成"自滤干净"）。"""
        f = _f4(back=0.30, right=1.0, front=1.0, left=1.0)
        assert f['self_residual'] is None

    def test_free_width_is_wall_to_wall_and_does_not_add_the_robot(self):
        """净宽 = 两侧回波之和，**不能**再加机器人直径。

        left_min/right_min 从传感器原点（≈底盘中心）量到墙，相加就已经是
        墙到墙全宽，机器人占的那部分含在里面了。旧实现多加 2*外接半径，
        等于把底盘算两遍：实测 run1 的 3270 帧里，正确口径有 1382 帧
        （42.3%）净宽 <1.62m，而旧口径 **0 帧** —— 四个 narrow 列全是死的。
        """
        f = _f4(back=5.0, right=0.5, front=3.0, left=0.5, circumscribed=0.42)
        assert abs(f['free_width'] - 1.0) < 1e-12, (
            '0.5+0.5=1.0；多加一个直径 0.84 就成了 1.84，'
            '在 1.62m 的窄通道阈值上恰好从"窄"翻成"不窄"')

    def test_free_width_does_not_need_a_footprint(self):
        """净宽只由两侧回波决定 —— 拿不到足迹时也该有值。

        旧实现把它挂在 circumscribed 非空上，于是没有足迹的跑次连净宽
        都没有，而这个量其实和足迹无关。
        """
        f = _f4(back=5.0, right=0.6, front=3.0, left=0.7)
        assert abs(f['free_width'] - 1.3) < 1e-12

    def test_a_real_narrow_corridor_is_below_the_threshold(self):
        """端到端：1.5m 的真实通道必须判成窄（阈值 1.62）。"""
        f = _f4(back=5.0, right=0.75, front=3.0, left=0.75, circumscribed=0.42)
        assert f['free_width'] < sm.DEFAULT_NARROW_WIDTH_M, (
            '1.5m 通道判不成窄，窄通道策略就没有任何指标在看它')

    def test_empty_scan(self):
        f = sm.frame_stats(np.zeros(0), 0.0, 0.1, 0.05, 20.0)
        assert f['n_beams'] == 0
        assert f['min_range'] is None


class TestClearance:

    def test_clearance_can_be_negative(self):
        """**核心断言**：不得夹到 0。

        "净空 0.00"既可能是刚好贴上，也可能是已经嵌进去 0.3m ——
        这两件事该做的处置完全不同。
        """
        c = sm.clearance(0.12, 0.42)
        assert c < 0
        assert abs(c - (-0.30)) < 1e-12

    def test_missing_input_is_none(self):
        assert sm.clearance(None, 0.42) is None
        assert sm.clearance(1.0, None) is None


class TestAggregate:

    def _corridor_frames(self, n, left, right, circ=0.42):
        return [_f4(back=5.0, right=right, front=3.0, left=left, circumscribed=circ)
                for _ in range(n)]

    def test_empty_gives_all_none(self):
        a = sm.aggregate([], [])
        assert a['scan_frames'] == 0
        assert a['nearest_obstacle_m'] is None
        assert a['center_bias_abs_mean'] is None

    def test_corridor_sample_ratio_is_reported(self):
        """**核心断言**：合格样本占比必须一起报。

        没进过通道时 center_bias_abs_mean 是 None；但如果只有 2/100 帧
        处在通道里，那个均值在统计上没有意义 —— 读者要靠这一列判断该不该信。
        """
        frames = self._corridor_frames(2, 0.5, 1.0)
        frames += [_f4(back=9.0, right=8.0, front=9.0, left=8.0, circumscribed=0.42)
                   for _ in range(98)]
        a = sm.aggregate(frames, np.arange(100) * 0.1, circumscribed=0.42)
        assert a['corridor_samples'] == 2
        assert abs(a['corridor_sample_ratio'] - 0.02) < 1e-12

    def test_nearest_obstacle_and_clearance_are_consistent(self):
        frames = self._corridor_frames(10, 0.5, 0.5)
        a = sm.aggregate(frames, np.arange(10) * 0.1, circumscribed=0.42)
        assert abs(a['nearest_obstacle_m'] - 0.5) < 1e-12
        assert abs(a['min_clearance_m'] - 0.08) < 1e-12

    def test_self_filter_residual_ratio(self):
        clean = self._corridor_frames(9, 1.0, 1.0)
        dirty = [_f4(back=0.1, right=1.0, front=1.0, left=1.0, circumscribed=0.42)]
        a = sm.aggregate(clean + dirty, np.arange(10) * 0.1, circumscribed=0.42)
        assert a['self_filter_residual_frames'] == 1
        assert abs(a['self_filter_residual_ratio_proxy'] - 1.0 / 40.0) < 1e-12

    def test_scan_rate_is_reported(self):
        frames = self._corridor_frames(50, 1.0, 1.0)
        a = sm.aggregate(frames, np.arange(50) * 0.1, circumscribed=0.42)
        assert abs(a['scan_hz'] - 10.0) < 1e-6


class TestNarrowTraversals:

    def _frames(self, n, half_gap, circ=0.42):
        return [_f4(back=5.0, right=half_gap, front=3.0, left=half_gap,
                    circumscribed=circ) for _ in range(n)]

    def test_no_episode_returns_none_not_one(self):
        """**核心断言**：没进过窄通道 != 窄通道 100% 通过。"""
        t = np.arange(50) * 0.1
        wide = self._frames(50, 2.0)
        r = sm.narrow_traversals(t, wide, np.linspace(0, 5, 50))
        assert r['narrow_episodes'] == 0
        assert r['narrow_success_rate_proxy'] is None, (
            '段数为 0 时返回 1.0 会在汇总表里变成"窄通道全通过"的假结论')

    def test_successful_traversal(self):
        t = np.arange(50) * 0.1
        # 2*0.35 + 0.84 = 1.54 < 1.62 -> 窄
        narrow = self._frames(50, 0.35)
        prog = np.linspace(0.0, 2.0, 50)
        r = sm.narrow_traversals(t, narrow, prog)
        assert r['narrow_episodes'] == 1
        assert r['narrow_success'] == 1
        assert r['narrow_success_rate_proxy'] == 1.0

    def test_no_progress_is_a_failure(self):
        t = np.arange(50) * 0.1
        narrow = self._frames(50, 0.35)
        prog = np.zeros(50)             # 卡在窄处没推进
        r = sm.narrow_traversals(t, narrow, prog)
        assert r['narrow_episodes'] == 1
        assert r['narrow_success'] == 0

    def test_round_failure_forces_all_episodes_to_fail(self):
        """整轮失败时，窄段一律判否 —— 不能一边导航失败一边报"窄通道通过"。"""
        t = np.arange(50) * 0.1
        narrow = self._frames(50, 0.35)
        r = sm.narrow_traversals(t, narrow, np.linspace(0, 2, 50), round_failed=True)
        assert r['narrow_episodes'] == 1
        assert r['narrow_success'] == 0

    def test_short_episode_is_filtered(self):
        t = np.arange(50) * 0.1
        frames = self._frames(2, 0.35) + self._frames(48, 2.0)
        r = sm.narrow_traversals(t, frames, np.linspace(0, 2, 50),
                                 min_episode_s=0.5)
        assert r['narrow_episodes'] == 0, '0.1s 的单帧噪声被算成了一次穿越'

    def test_width_threshold_boundary(self):
        """1.62m 是实测的 MPPI 足迹代价饱和宽度，判据必须真的用上它。

        净宽 = 两侧之和，不加机器人直径（见
        test_free_width_is_wall_to_wall_and_does_not_add_the_robot）。
        """
        t = np.arange(20) * 0.1
        just_wide = self._frames(20, 0.82)      # 0.82*2 = 1.64 > 1.62
        assert sm.narrow_traversals(t, just_wide, np.zeros(20))['narrow_episodes'] == 0
        just_narrow = self._frames(20, 0.80)    # 0.80*2 = 1.60 < 1.62
        assert sm.narrow_traversals(t, just_narrow, np.zeros(20))['narrow_episodes'] == 1


class TestLowObstacleDetection:

    def test_without_truth_returns_none_and_says_why(self):
        """**核心断言**：没有人工真值就必须是 None，且要说明原因。

        绝不能用"回波数/光束数"之类的东西冒充检出率 ——
        那量的是有没有回波，不是有没有检出**指定的**障碍。
        """
        r = sm.low_obstacle_detection([], [(1.0, 1.0)])
        assert r['low_obstacle_detect_rate'] is None
        assert '真值' in r['low_obstacle_note']

    def test_hit_within_tolerance(self):
        r = sm.low_obstacle_detection([(1.0, 1.0, 0.2)], [(1.1, 1.0)], tol_m=0.25)
        assert r['low_obstacle_detect_rate'] == 1.0

    def test_miss_outside_tolerance(self):
        r = sm.low_obstacle_detection([(1.0, 1.0, 0.2)], [(2.0, 2.0)], tol_m=0.25)
        assert r['low_obstacle_detect_rate'] == 0.0

    def test_partial(self):
        truth = [(1.0, 1.0, 0.2), (5.0, 5.0, 0.2)]
        r = sm.low_obstacle_detection(truth, [(1.0, 1.0)], tol_m=0.25)
        assert abs(r['low_obstacle_detect_rate'] - 0.5) < 1e-12

    def test_note_mentions_the_z_slice(self):
        """z 切片 [0.05,1.63] 是构造性限制，必须随数据一起说明。"""
        r = sm.low_obstacle_detection([(1.0, 1.0, 0.02)], [])
        assert '0.05' in r['low_obstacle_note']
