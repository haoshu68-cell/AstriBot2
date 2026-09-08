# Copyright 2026 Astribot.
#
# explore_metrics.geometry 的单测。
#
# 重点不是"覆盖每个函数"，而是钉住**会静默出错**的那几条：
#   · 拿不到数据必须返回 None，不能返回 0.0（假均值）
#   · 内切半径必须含"边距离"，只取顶点会高估
#   · 超调必须区分径向与沿轨（全向底盘会横向甩出去）
#   · 里程的噪声门限必须真的生效，且被丢弃的步数要能报出来
#   · 规模：9 万样本 × 800 顶点的横向偏差不能爆内存/卡死
import math
import time

import numpy as np
import pytest

from astribot_s1_navigation.explore_metrics import geometry as g


class TestWrapAndYaw:

    def test_wrap_scalar_returns_float(self):
        """返回 float 而不是 0 维 ndarray —— 后者写进 CSV 会变成 'array(0.1)'。"""
        v = g.wrap_angle_scalar(3.0 * math.pi)
        assert isinstance(v, float)
        # 边界约定：取模实现下 ±pi 一律落到 -pi。这条测试就是把这个约定钉住，
        # 免得有人"顺手修成 +pi"而让别处依赖符号的代码静默改变行为。
        assert abs(v + math.pi) < 1e-9

    def test_wrap_folds_multiples_of_two_pi(self):
        for k in (-3, -1, 0, 1, 3):
            assert abs(g.wrap_angle_scalar(0.3 + 2.0 * math.pi * k) - 0.3) < 1e-9

    def test_wrap_range_is_documented_half_open(self):
        """全区间扫一遍，确认落在 [-pi, pi)。"""
        vals = g.wrap_angle(np.linspace(-20.0, 20.0, 4001))
        assert np.all(vals >= -math.pi - 1e-12)
        assert np.all(vals < math.pi + 1e-12)

    def test_yaw_from_quat_identity(self):
        assert abs(g.yaw_from_quat(0, 0, 0, 1)) < 1e-12

    def test_yaw_from_quat_half_pi(self):
        s = math.sin(math.pi / 4.0)
        c = math.cos(math.pi / 4.0)
        assert abs(g.yaw_from_quat(0, 0, s, c) - math.pi / 2.0) < 1e-9


class TestLengths:

    def test_polyline_length_exact(self):
        assert abs(g.polyline_length([(0, 0), (3, 4)]) - 5.0) < 1e-12

    def test_polyline_length_single_point_is_zero(self):
        assert g.polyline_length([(1, 1)]) == 0.0

    def test_polyline_has_no_noise_gate(self):
        """规划路径的顶点是确定值，加门限只会低估参考长度。"""
        pts = [(0.0, 0.0), (0.0005, 0.0), (0.001, 0.0)]
        assert abs(g.polyline_length(pts) - 0.001) < 1e-12

    def test_traveled_gate_drops_jitter(self):
        """静置抖动不该累计成里程。"""
        pts = [(0.0, 0.0)] + [(0.0005 * (-1) ** i, 0.0) for i in range(100)]
        dist, dropped = g.traveled_length(pts, min_step=0.002)
        assert dist == 0.0, '抖动被算成了里程'
        assert dropped == 100, '被丢弃的步数必须报出来，否则无法判断该不该信这个比值'

    def test_traveled_gate_keeps_real_motion(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        dist, dropped = g.traveled_length(pts, min_step=0.002)
        assert abs(dist - 2.0) < 1e-12
        assert dropped == 0

    def test_traveled_reports_dropped_count_separately(self):
        """真实位移 + 抖动混合时，两个返回值都要对。"""
        pts = [(0.0, 0.0), (1.0, 0.0), (1.0005, 0.0), (2.0, 0.0)]
        dist, dropped = g.traveled_length(pts, min_step=0.002)
        assert dropped == 1
        assert abs(dist - (1.0 + 0.9995)) < 1e-9


class TestCrossTrack:

    def test_point_on_line_is_zero(self):
        d, idx, t = g.cross_track_distances([(0.5, 0.0)], [(0, 0), (1, 0)])
        assert abs(d[0]) < 1e-12
        assert idx[0] == 0
        assert abs(t[0] - 0.5) < 1e-12

    def test_perpendicular_offset(self):
        d, _, _ = g.cross_track_distances([(0.5, 0.3)], [(0, 0), (1, 0)])
        assert abs(d[0] - 0.3) < 1e-12

    def test_beyond_endpoint_clamps(self):
        """越过端点时距离是到端点的距离，不是到无限延长线的距离。"""
        d, _, t = g.cross_track_distances([(2.0, 0.0)], [(0, 0), (1, 0)])
        assert abs(d[0] - 1.0) < 1e-12
        assert t[0] == 1.0

    def test_empty_polyline_is_nan_not_zero(self):
        """**核心断言**：没有路径 != 偏差为 0。"""
        d, idx, _ = g.cross_track_distances([(1.0, 1.0)], [])
        assert np.isnan(d[0]), '没有路径时返回 0 会在汇总表里变成一个漂亮的假均值'
        assert idx[0] == -1

    def test_single_vertex_polyline_is_point_distance(self):
        d, _, _ = g.cross_track_distances([(3.0, 4.0)], [(0, 0)])
        assert abs(d[0] - 5.0) < 1e-12

    def test_duplicate_vertices_do_not_produce_nan(self):
        """重复顶点会让线段长度为 0 —— 不能变成 0/0。"""
        d, _, _ = g.cross_track_distances([(0.0, 1.0)], [(0, 0), (0, 0), (1, 0)])
        assert np.all(np.isfinite(d))
        assert abs(d[0] - 1.0) < 1e-12

    def test_picks_nearest_segment_not_last(self):
        """必须取最近段。取最后一段会让 L 形路径的偏差凭空变大。"""
        poly = [(0, 0), (10, 0), (10, 10)]
        d, idx, _ = g.cross_track_distances([(5.0, 0.1)], poly)
        assert idx[0] == 0
        assert abs(d[0] - 0.1) < 1e-12

    def test_chunking_matches_unchunked(self):
        """分块计算必须与不分块逐位一致（否则规模一大结果就变）。"""
        rng = np.random.default_rng(7)
        q = rng.uniform(-5, 5, size=(g._CHUNK * 2 + 13, 2))
        poly = np.column_stack([np.linspace(-5, 5, 40), np.sin(np.linspace(0, 6, 40))])
        d, _, _ = g.cross_track_distances(q, poly)
        ref = np.empty(len(q))
        for i in range(len(q)):
            ref[i] = g.cross_track_distances(q[i:i + 1], poly)[0][0]
        assert np.allclose(d, ref, atol=1e-12)


class TestSmoothness:

    def test_straight_line_is_perfectly_smooth(self):
        s = g.smoothness([(0, 0), (1, 0), (2, 0), (3, 0)])
        assert abs(s['turn_per_m']) < 1e-12
        assert abs(s['max_turn_rad']) < 1e-12

    def test_right_angle_turn(self):
        s = g.smoothness([(0, 0), (1, 0), (1, 1)])
        assert abs(s['max_turn_rad'] - math.pi / 2.0) < 1e-9
        assert abs(s['turn_per_m'] - (math.pi / 2.0) / 2.0) < 1e-9

    def test_zero_length_returns_none_not_inf(self):
        """**核心断言**：没走路就没有"每米转角"。"""
        s = g.smoothness([(1, 1), (1, 1), (1, 1)])
        assert s['turn_per_m'] is None, '返回 inf/0 会被汇总当成有效样本'

    def test_decimation_suppresses_sampling_noise(self):
        """50Hz 采样的抖动不抽稀时会主导平滑度读数。"""
        rng = np.random.default_rng(3)
        n = 2000
        track = np.column_stack([
            np.linspace(0, 10, n) + rng.normal(0, 3e-4, n),
            rng.normal(0, 3e-4, n)])
        noisy = g.smoothness(track, min_step=0.0)['turn_per_m']
        clean = g.smoothness(track, min_step=0.05)['turn_per_m']
        assert noisy > 10.0 * clean, (
            '抽稀没起作用：不抽稀量到的是传感器噪声而不是路径平滑度 '
            '(noisy=%.3f clean=%.3f)' % (noisy, clean))


class TestDecimate:

    def test_keeps_first_and_last(self):
        """末点必须保留 —— 到位位置正好在轨迹尾巴上。"""
        pts = [(0, 0), (1, 0), (1.0001, 0)]
        out = g.decimate_by_step(pts, 0.5)
        assert np.allclose(out[0], (0, 0))
        assert np.allclose(out[-1], (1.0001, 0)), '末点被剪掉了，到位位置会读错'

    def test_drops_below_step(self):
        pts = [(0, 0), (0.001, 0), (0.002, 0), (1.0, 0)]
        out = g.decimate_by_step(pts, 0.5)
        assert len(out) == 2


class TestOvershoot:

    def _straight(self, end_x, n=200):
        return np.column_stack([np.linspace(0, end_x, n), np.zeros(n)])

    def test_never_entered_returns_none(self):
        """**核心断言**：没到过容差圈就谈不上超调。"""
        ov = g.overshoot(self._straight(1.0), (5.0, 0.0), 0.18)
        assert ov['radial_m'] is None, '没到过目标却报出超调数值'
        assert ov['along_track_m'] is None

    def test_stops_inside_tolerance_has_small_overshoot(self):
        ov = g.overshoot(self._straight(2.0), (2.0, 0.0), 0.18)
        assert ov['radial_m'] <= 0.18 + 1e-9

    def test_overshoot_beyond_goal(self):
        ov = g.overshoot(self._straight(2.5), (2.0, 0.0), 0.18)
        assert abs(ov['radial_m'] - 0.5) < 1e-6
        assert abs(ov['along_track_m'] - 0.5) < 1e-6

    def test_lateral_fling_separates_radial_from_along_track(self):
        """全向底盘横向甩出去：radial 大而 along_track ≈ 0。

        这两个量不分开报，就分不清"刹不住"和"被横向甩出去"，
        而这两件事该改的东西完全不同。
        """
        n = 100
        track = np.column_stack([np.linspace(0, 2.0, n), np.zeros(n)])
        # 到目标后横向甩 0.5m
        fling = np.column_stack([np.full(30, 2.0), np.linspace(0, 0.5, 30)])
        ov = g.overshoot(np.vstack([track, fling]), (2.0, 0.0), 0.18)
        assert ov['radial_m'] > 0.45
        assert ov['along_track_m'] < 0.05, (
            'along_track 把横向位移也算进去了 —— 那就无法区分横甩与刹不住')

    def test_along_track_is_never_negative(self):
        """没冲过头时沿轨超调是 0，不是负数（负数会把均值拉低成"负超调"）。"""
        n = 100
        track = np.column_stack([np.linspace(0, 1.9, n), np.zeros(n)])
        ov = g.overshoot(track, (2.0, 0.0), 0.18)
        assert ov['along_track_m'] is None or ov['along_track_m'] >= 0.0


class TestSettleDrift:

    def test_single_sample_returns_none(self):
        assert g.settle_drift([(0, 0)], [0.0]) == (None, None)

    def test_drift_relative_to_window_start(self):
        pts = [(0, 0), (0.01, 0), (0.02, 0)]
        d, _ = g.settle_drift(pts, [0.0, 0.0, 0.0])
        assert abs(d - 0.02) < 1e-12

    def test_monotonic_drift_differs_from_oscillation(self):
        """相对窗首而不是取极差：单向漂移与来回抖动必须给出不同的数。

        取极差时下面两组会得到同一个值，而"没抓住位置"和"控制在抖"
        是两个不同的问题。
        """
        mono, _ = g.settle_drift([(0, 0), (0.05, 0), (0.10, 0)], [0, 0, 0])
        osc, _ = g.settle_drift([(0.05, 0), (0.10, 0), (0.05, 0)], [0, 0, 0])
        assert abs(mono - 0.10) < 1e-12
        assert abs(osc - 0.05) < 1e-12
        assert mono != osc

    def test_yaw_drift_wraps(self):
        _, yaw = g.settle_drift([(0, 0), (0, 0)], [math.pi - 0.01, -math.pi + 0.01])
        assert yaw < 0.05, 'yaw 漂移没折角，跨 ±pi 时会报出 ~2pi 的假漂移'


class TestPolygonRadii:

    def test_square_inscribed_is_edge_distance_not_vertex(self):
        """**核心断言**：内切半径必须含边距离。

        只取顶点距离时正方形会给 0.4243 而不是 0.30 —— 高估内切半径的后果是
        净空算出正数、而实际已压在 nav2 判"起点致命"的格上。
        """
        a = 0.30
        ins, circ = g.polygon_radii([(a, a), (-a, a), (-a, -a), (a, -a)])
        assert abs(ins - a) < 1e-12, '内切半径只取了顶点距离'
        assert abs(circ - a * math.sqrt(2.0)) < 1e-12

    def test_octagon_matches_nav2_measured_value(self):
        """本机八边形足迹，与 nav2 calculateMinAndMaxDistances 实测值对齐。

        注意外接半径是 0.420021 而不是 0.42：顶点 (0.297,0.297) 的模是
        0.4200214，这个八边形并不是正八边形。把它当 0.42 会让"净空"
        差 2e-5 —— 数值上无所谓，但写测试时按 0.42 卡死会让人以为算错了。
        """
        fp = [(0.42, 0.0), (0.297, 0.297), (0.0, 0.42), (-0.297, 0.297),
              (-0.42, 0.0), (-0.297, -0.297), (0.0, -0.42), (0.297, -0.297)]
        ins, circ = g.polygon_radii(fp)
        assert abs(ins - 0.388039) < 1e-5, ins
        assert abs(circ - 0.4200214) < 1e-6, circ

    def test_too_few_vertices_returns_none(self):
        """**核心断言**：拿不到足迹必须说拿不到，不能退回一个默认半径。"""
        assert g.polygon_radii([]) == (None, None)
        assert g.polygon_radii([(1, 0), (0, 1)]) == (None, None)


class TestScale:
    """规模测试。小样本全绿而真实规模卡死，本项目已发生过一次。"""

    def test_cross_track_at_real_scale(self):
        """30 分钟 @50Hz = 9 万位姿样本，路径 800 顶点。

        写成完整 M×N 矩阵是 7.2e7 个 float64 = 576MB，会换页到卡死。
        这条测试就是拦那个写法的。
        """
        n_pose = 90_000
        n_path = 800
        rng = np.random.default_rng(11)
        s = np.linspace(0, 60, n_pose)
        track = np.column_stack([s, 0.1 * np.sin(s) + rng.normal(0, 0.01, n_pose)])
        poly = np.column_stack([np.linspace(0, 60, n_path),
                                0.1 * np.sin(np.linspace(0, 60, n_path))])
        t0 = time.time()
        d, _, _ = g.cross_track_distances(track, poly)
        elapsed = time.time() - t0
        assert d.shape == (n_pose,)
        assert np.all(np.isfinite(d))
        assert elapsed < 20.0, '真实规模下横向偏差耗时 %.1fs' % elapsed

    def test_traveled_length_at_real_scale(self):
        """真实采样密度：50Hz × 30 分钟，0.2m/s → 单步 0.004m（高于 2mm 门限）。

        刻意不用"更密的采样"来做规模测试：单步一旦低于 min_step 门限，
        全部位移都会被正确地丢弃、结果是 0 —— 那是函数按设计工作，
        而按 0 去断言等于把噪声门限测成了 bug。
        """
        n = 90_000
        step = 0.004
        track = np.column_stack([np.arange(n) * step, np.zeros(n)])
        t0 = time.time()
        dist, dropped = g.traveled_length(track)
        assert dropped == 0, '真实步长被误判成抖动'
        assert abs(dist - step * (n - 1)) < 1e-3
        assert time.time() - t0 < 5.0

    def test_gate_drops_everything_when_sampling_is_denser_than_gate(self):
        """采样密到单步 < 门限时，里程会是 0 —— 这是设计行为，必须能看出来。

        所以 dropped 计数是**必须**一起报的：只看里程会以为机器人没动，
        看了 dropped 才知道是"采样太密 / 机器人太慢"，门限该调。
        """
        n = 5000
        track = np.column_stack([np.linspace(0, 1.0, n), np.zeros(n)])
        dist, dropped = g.traveled_length(track, min_step=0.002)
        assert dist == 0.0
        assert dropped == n - 1


@pytest.mark.parametrize('bad', [[], [(1.0, 2.0)]])
def test_all_entry_points_survive_degenerate_input(bad):
    """空/单点输入不得抛异常 —— 一轮采集不全不该让整次扫描的数据全丢。"""
    assert g.polyline_length(bad) >= 0.0
    g.traveled_length(bad)
    g.smoothness(bad)
    g.settle_drift(bad, [])
    g.overshoot(bad, (0.0, 0.0), 0.18)
    g.polygon_radii(bad)


# ==================== published_footprint 是全局系，不是车体系 ====================
# 实测 /global_costmap/published_footprint 的 frame_id=map，顶点是绝对坐标
# （机器人在 (2.2,-9.5) 时顶点就在那附近）。直接算内切半径量到的是
# "地图原点到多边形的距离"，实测随行驶单调增长 6.12 -> 8.81m，
# 而底盘内切只有 0.42m。下游 min_clearance = min_range - circumscribed
# 因此恒为大负数、free_width 凭空多加 2*9m、self_residual 把几乎所有点算成自碰。
OCTAGON = [(0.42, 0.0), (0.297, 0.297), (0.0, 0.42), (-0.297, 0.297),
           (-0.42, 0.0), (-0.297, -0.297), (0.0, -0.42), (0.297, -0.297)]
SQUARE = [(0.31, 0.31), (-0.31, 0.31), (-0.31, -0.31), (0.31, -0.31)]


def _translate(pts, dx, dy):
    return [(x + dx, y + dy) for x, y in pts]


def _rotate(pts, a):
    c, s = math.cos(a), math.sin(a)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


@pytest.mark.parametrize('fp', [OCTAGON, SQUARE])
def test_symmetry_center_finds_origin_for_body_frame_footprint(fp):
    """车体系足迹的对称中心必须≈原点 —— 否则"中心非零即全局系"这条判据失效。"""
    c = g.symmetry_center(fp)
    assert c is not None
    assert math.hypot(c[0], c[1]) < 1e-9


@pytest.mark.parametrize('fp', [OCTAGON, SQUARE])
@pytest.mark.parametrize('shift', [(2.2, -9.5), (-13.0, 41.0)])
@pytest.mark.parametrize('yaw', [0.0, 0.7, 2.9])
def test_radii_survive_global_frame_pose(fp, shift, yaw):
    """核心回归：先旋转再平移（= published_footprint 的真实形态），
    recenter 后的两个半径必须与车体系原值逐位一致。

    半径绕原点旋转不变，所以平移回对称中心就够，不需要 TF、不需要 yaw。
    """
    ins0, circ0 = g.polygon_radii(fp)
    moved = _translate(_rotate(fp, yaw), *shift)
    pts, center = g.recenter_polygon(moved)
    assert pts is not None
    assert math.hypot(center[0] - shift[0], center[1] - shift[1]) < 1e-9
    ins1, circ1 = g.polygon_radii(pts)
    assert abs(ins1 - ins0) < 1e-9
    assert abs(circ1 - circ0) < 1e-9


@pytest.mark.parametrize('shift', [(2.2, -9.5), (8.0, 8.0)])
def test_unrecentered_radii_are_the_measured_bug(shift):
    """反向断言：不 recenter 就会复现实测的错值（≈到原点的距离）。

    这条存在的意义是防止有人把 recenter 去掉后测试仍然全绿。
    """
    moved = _translate(OCTAGON, *shift)
    ins_bad, _ = g.polygon_radii(moved)
    d = math.hypot(*shift)
    assert ins_bad > d - 0.5          # 量的是到原点的距离，不是底盘尺寸
    assert ins_bad > 5.0              # 而真值是 0.4157


def test_non_symmetric_polygon_is_refused_not_guessed():
    """不中心对称就必须返回 None：静默用质心会给出偏移的半径而每行看着都正常。"""
    tri = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    assert g.symmetry_center(tri) is None
    pts, c = g.recenter_polygon(tri)
    assert pts is None and c is None


def test_padded_footprint_stays_symmetric():
    """nav2 的 footprint_padding 向外等量膨胀，仍中心对称 —— 判据不能被 padding 打破。"""
    pad = 0.01
    padded = [(x + math.copysign(pad, x) if x else x,
               y + math.copysign(pad, y) if y else y) for x, y in OCTAGON]
    assert g.symmetry_center(padded) is not None


class TestHeadingTangent:
    """「机头 − 路径切向」误差。这是路线 A（掐掉 vy）的**唯一**验收判据：
    蟹行角 atan2(vy,vx) 在 vy≡0 之后恒等于 0，量它等于量自己的输入。
    """

    def test_perfectly_aligned_on_straight_path(self):
        poly = [(x * 0.05, 0.0) for x in range(40)]      # 沿 +x 的直线
        track = [(0.5, 0.0), (1.0, 0.0)]
        err, dist = g.heading_tangent_errors(track, [0.0, 0.0], poly)
        assert np.allclose(err, 0.0, atol=1e-9)
        assert np.allclose(dist, 0.0, atol=1e-9)

    def test_ninety_degree_heading_error_is_reported(self):
        """蟹行：机头指 +y、路径沿 +x —— 误差必须是 pi/2，不是 0。"""
        poly = [(x * 0.05, 0.0) for x in range(40)]
        err, _ = g.heading_tangent_errors([(1.0, 0.0)], [math.pi / 2], poly)
        assert err[0] == pytest.approx(math.pi / 2, abs=1e-9)

    def test_sign_is_preserved_and_wrapped(self):
        poly = [(x * 0.05, 0.0) for x in range(40)]
        err, _ = g.heading_tangent_errors(
            [(1.0, 0.0), (1.0, 0.0)], [-0.3, 0.3], poly)
        assert err[0] == pytest.approx(-0.3, abs=1e-9)
        assert err[1] == pytest.approx(+0.3, abs=1e-9)

    def test_error_near_pi_does_not_come_back_as_near_2pi(self):
        """机头几乎反着 —— 必须落在 [-pi,pi)，否则 P90 之类的分位会爆掉。"""
        poly = [(x * 0.05, 0.0) for x in range(40)]
        err, _ = g.heading_tangent_errors([(1.0, 0.0)], [math.pi - 0.01], poly)
        assert abs(err[0]) == pytest.approx(math.pi - 0.01, abs=1e-9)

    def test_tangent_follows_a_curved_path(self):
        """1m 半径圆弧上，机头切向对齐 ⇒ 误差应当很小（受抽稀基线限制）。"""
        ang = np.linspace(0.0, math.pi / 2, 200)
        poly = np.stack([np.cos(ang), np.sin(ang)], axis=1)
        # 取弧上几点，机头 = 该点真实切向（逆时针 ⇒ 切向 = ang + pi/2）
        pick = [50, 100, 150]
        track = poly[pick]
        yaws = ang[pick] + math.pi / 2
        err, dist = g.heading_tangent_errors(track, yaws, poly)
        # 0.20m 抽稀基线在 R=1m 上对应的弦-切偏差是 step/(2R)=0.10rad 量级，
        # 这是基线长度的**固有**代价，不是实现错误 —— 所以判据取 0.12。
        assert np.all(np.abs(err) < 0.12), err

    def test_single_segment_tangent_would_be_quantized(self):
        """抽稀是必要的：8 连通栅格路径的单段方向量化到 45° 整数倍。

        不抽稀（step=0）时同一条路径会报出接近 45° 的假误差，
        抽稀到 0.20m 后降到很小 —— 这条测试就是在钉住"为什么要抽稀"。
        """
        # 沿 +x 前进但逐格锯齿（栅格 A* 的典型形状），整体走向仍是 +x
        pts = []
        for i in range(40):
            pts.append((i * 0.05, 0.0 if i % 2 == 0 else 0.05))
        raw, _ = g.heading_tangent_errors(
            [(1.0, 0.025)], [0.0], pts, tangent_step_m=0.0)
        dec, _ = g.heading_tangent_errors(
            [(1.0, 0.025)], [0.0], pts, tangent_step_m=0.20)
        assert abs(raw[0]) > math.radians(30), raw
        assert abs(dec[0]) < abs(raw[0]) / 2, (raw, dec)

    def test_degenerate_path_gives_nan_not_zero(self):
        for bad in ([], [(0.0, 0.0)]):
            err, dist = g.heading_tangent_errors([(1.0, 1.0)], [0.0], bad)
            assert np.isnan(err).all()
            assert np.isnan(dist).all()

    def test_empty_track_is_empty_not_nan(self):
        poly = [(x * 0.05, 0.0) for x in range(40)]
        err, dist = g.heading_tangent_errors([], [], poly)
        assert err.size == 0 and dist.size == 0

    def test_length_mismatch_raises(self):
        poly = [(x * 0.05, 0.0) for x in range(40)]
        with pytest.raises(ValueError):
            g.heading_tangent_errors([(1.0, 0.0), (2.0, 0.0)], [0.0], poly)

    def test_dist_is_returned_so_far_from_path_samples_can_be_dropped(self):
        poly = [(x * 0.05, 0.0) for x in range(40)]
        _, dist = g.heading_tangent_errors([(1.0, 3.0)], [0.0], poly)
        assert dist[0] == pytest.approx(3.0, abs=1e-9)
