# Copyright 2026 Astribot.
#
# explore_metrics.round_metrics 的单测。
#
# 这个文件是**唯一的判据出口**，所以测试要覆盖到"换个写法也能跑但结论不同"
# 的那几处：
#   · 横向偏差必须按"生效路径"分段，不能整轮都比最后一条路径
#   · 重规划次数不能把逐字相同的重复发布算进去
#   · 耦合衰减这一列缺了，整个限速扫描是混淆的
#   · 到位位置必须来自 map 系 TF，不是 /odom
#   · 缺数据的轮次不能抛异常，但缺什么要能看出来
import math

import numpy as np
import pytest

from astribot_s1_navigation.explore_metrics import round_metrics as rm
from astribot_s1_navigation.explore_metrics import scan_metrics as sm


def _scan_frames(n, half_gap=2.0, circ=0.42):
    out = []
    for _ in range(n):
        ranges = np.array([5.0, half_gap, 3.0, half_gap], dtype=float)
        out.append(sm.frame_stats(ranges, -math.pi, math.pi / 2.0,
                                  0.05, 20.0, circumscribed=circ))
    return out


def make_raw(**over):
    """一条"直线走 2m 到位"的合成轮次。所有测试都在它上面改一处。

    scan_frames 按**最终生效的** footprint_circumscribed 生成，而不是写死 0.42。
    录制节点也是用同一个足迹值算 frame_stats 的；夹具写死会让
    "拿不到足迹"这一路的测试失去意义（帧里仍然带着自滤残留结果）。
    """
    n = 200
    t = np.arange(n) * 0.05
    x = np.linspace(0.0, 2.0, n)
    y = np.zeros(n)
    yaw = np.zeros(n)
    circ = over.get('footprint_circumscribed', 0.42)
    raw = dict(
        index=0, goal=(2.0, 0.0, 0.0), goal_stamp=float(t[0]),
        end_stamp=float(t[-1]), outcome='ARRIVED',
        pose_t=t, pose_x=x, pose_y=y, pose_yaw=yaw,
        settle_t=t[-20:], settle_x=np.full(20, 2.0),
        settle_y=np.zeros(20), settle_yaw=np.zeros(20),
        plans=[{'stamp': float(t[0]) - 0.1, 'points': [(0.0, 0.0), (2.0, 0.0)]}],
        cmd_raw_t=t, cmd_raw_vx=np.full(n, 0.2),
        cmd_raw_vy=np.zeros(n), cmd_raw_wz=np.zeros(n),
        cmd_final_t=t, cmd_final_vx=np.full(n, 0.2),
        cmd_final_vy=np.zeros(n), cmd_final_wz=np.zeros(n),
        odom_t=t, odom_vx=np.full(n, 0.2), odom_vy=np.zeros(n),
        scan_t=t, scan_frames=_scan_frames(n, circ=circ),
        footprint_inscribed=0.388, footprint_circumscribed=0.42,
        goal_tolerance=0.18,
        vx_max_measured=0.2, speed_cap_requested=0.2,
        plan_requests=[{'accept': float(t[0]) - 0.4, 'end': float(t[0]) - 0.1,
                        'status': 'SUCCEEDED'}],
    )
    raw.update(over)
    return raw


class TestArrival:

    def test_position_error_is_map_frame_hypot(self):
        raw = make_raw()
        raw['pose_x'] = np.append(raw['pose_x'][:-1], 2.09)
        raw['pose_y'] = np.append(raw['pose_y'][:-1], 0.03)
        row = rm.compute_round(raw)
        assert abs(row['arrival_error_xy_m'] - math.hypot(0.09, 0.03)) < 1e-9

    def test_error_uses_both_axes(self):
        """**核心断言**：只用 x 会漏掉横向误差。

        本项目曾用 hypot(x,y) 与目标 x 比、又用 /odom 的 y 与 map 系目标的 y 比，
        两次都得出假误差。这条测试固定住"两轴都算、且都在 map 系"。
        """
        raw = make_raw(goal=(2.0, 0.0, 0.0))
        raw['pose_y'] = np.append(raw['pose_y'][:-1], 0.50)
        row = rm.compute_round(raw)
        assert row['arrival_error_xy_m'] > 0.49, '横向误差没被算进去'

    def test_yaw_error_wraps(self):
        raw = make_raw(goal=(2.0, 0.0, math.pi - 0.01))
        raw['pose_yaw'] = np.append(raw['pose_yaw'][:-1], -math.pi + 0.01)
        row = rm.compute_round(raw)
        assert abs(row['arrival_error_yaw_rad']) < 0.05, (
            'yaw 误差没折角，跨 ±pi 时会报出 ~2pi 的假误差')

    def test_no_pose_samples_gives_none(self):
        raw = make_raw(pose_t=[], pose_x=[], pose_y=[], pose_yaw=[])
        row = rm.compute_round(raw)
        assert row['arrival_error_xy_m'] is None
        assert row['pose_samples'] == 0, '缺数据必须能从列里看出来'

    def test_one_error_column_not_two(self):
        """「到位精度差距」与「位置到位误差」是同一个量，只能有一列。

        出两列的后果是它们哪天算法不一致，而表里两个数都在，没人知道信哪个。
        """
        row = rm.compute_round(make_raw())
        dup = [k for k in row
               if k not in ('arrival_error_xy_m',) and 'arrival' in k and 'xy' in k]
        assert dup == [], '到位位置误差出现了重复列：%s' % dup


class TestSettleDrift:

    def test_stationary_settle_has_zero_drift(self):
        row = rm.compute_round(make_raw())
        assert row['settle_drift_xy_m'] == 0.0
        assert row['settle_samples'] == 20

    def test_drifting_settle_is_caught(self):
        raw = make_raw()
        raw['settle_x'] = np.linspace(2.0, 2.05, 20)
        row = rm.compute_round(raw)
        assert abs(row['settle_drift_xy_m'] - 0.05) < 1e-9

    def test_missing_settle_window_is_none(self):
        raw = make_raw(settle_x=[], settle_y=[], settle_yaw=[])
        row = rm.compute_round(raw)
        assert row['settle_drift_xy_m'] is None
        assert row['settle_samples'] == 0


class TestPathMetrics:

    def test_length_ratio_on_a_straight_run(self):
        row = rm.compute_round(make_raw())
        assert abs(row['path_len_ratio_vs_plan'] - 1.0) < 1e-3
        assert abs(row['path_len_ratio_vs_straight'] - 1.0) < 1e-3

    def test_detour_shows_up_as_ratio_above_one(self):
        raw = make_raw()
        n = len(raw['pose_t'])
        raw['pose_y'] = 0.5 * np.sin(np.linspace(0, math.pi, n))
        row = rm.compute_round(raw)
        assert row['path_len_ratio_vs_straight'] > 1.1

    def test_ungated_length_is_also_reported(self):
        """**核心断言**：两个口径都要有。

        位姿 20Hz、限速 0.1m/s 时单步 5mm，掉到 50Hz 就是 2mm ——
        与门限同量级，门限会把真实位移当抖动丢掉，traveled_m 变 0
        而看不出原因。限速扫描恰恰要往低速档走。
        """
        row = rm.compute_round(make_raw())
        assert row['traveled_m_ungated'] is not None
        assert row['traveled_gate_dropped_ratio'] is not None

    def test_gate_suspicious_flag_fires_on_dense_slow_sampling(self):
        raw = make_raw()
        n = 2000
        raw['pose_t'] = np.arange(n) * 0.02
        raw['pose_x'] = np.linspace(0.0, 1.0, n)     # 单步 0.5mm < 2mm 门限
        raw['pose_y'] = np.zeros(n)
        raw['pose_yaw'] = np.zeros(n)
        row = rm.compute_round(raw)
        assert row['traveled_gate_suspicious'] is True, (
            '采样密度与门限不匹配时没有告警 —— traveled_m 会是 0 而看不出原因')
        assert row['traveled_m_ungated'] > 0.9, '未门限口径仍应给出真实里程'

    def test_replan_ignores_identical_republish(self):
        """**核心断言**：逐字相同的重复发布不算重规划。

        nav2 在某些配置下会周期性重发同一条路径。计进去会得出
        "每秒重规划一次"，而那是发布行为不是重规划。
        """
        pts = [(0.0, 0.0), (2.0, 0.0)]
        raw = make_raw(plans=[{'stamp': i * 1.0, 'points': list(pts)}
                              for i in range(5)])
        row = rm.compute_round(raw)
        assert row['plans_published'] == 5
        assert row['replan_count'] == 0, '同一条路径被重发 5 次算成了 4 次重规划'

    def test_replan_counts_real_changes(self):
        raw = make_raw(plans=[
            {'stamp': 0.0, 'points': [(0.0, 0.0), (2.0, 0.0)]},
            {'stamp': 1.0, 'points': [(0.0, 0.0), (1.0, 0.5), (2.0, 0.0)]},
        ])
        row = rm.compute_round(raw)
        assert row['replan_count'] == 1

    def test_planning_failure_rate(self):
        raw = make_raw(plan_requests=[
            {'accept': 0.0, 'end': 0.2, 'status': 'SUCCEEDED'},
            {'accept': 1.0, 'end': 1.3, 'status': 'ABORTED'},
        ])
        row = rm.compute_round(raw)
        assert abs(row['planning_failure_rate'] - 0.5) < 1e-12
        assert row['planning_aborted'] == 1

    def test_planning_time_falls_back_and_says_so(self):
        """拿不到 action 状态时用退化口径，但必须在列里标明来源。"""
        raw = make_raw(plan_requests=[])
        raw['plans'] = [{'stamp': raw['goal_stamp'] + 0.7,
                         'points': [(0.0, 0.0), (2.0, 0.0)]}]
        row = rm.compute_round(raw)
        assert abs(row['planning_time_median_s'] - 0.7) < 1e-9
        assert row['planning_time_source'].startswith('fallback'), (
            '换了口径却没标出来 —— 两种口径的数会被混在一张表里比较')

    def test_no_plan_requests_and_no_plans_is_none(self):
        raw = make_raw(plan_requests=[], plans=[])
        row = rm.compute_round(raw)
        assert row['planning_time_median_s'] is None
        assert row['path_len_ratio_vs_plan'] is None

    def test_stale_plan_from_previous_round_never_yields_negative_time(self):
        """本轮缓冲里的首条 /plan 可能是上一轮的陈旧路径。

        nav2 周期性重规划，缓冲的第一条完全可能早于 goal_stamp。此前的实现
        直接 plans[0].stamp - goal_stamp，实测三轮全为负（-0.496 / -2.507 /
        -0.297 s）。时长为负是**不可能**的，必须取目标之后的那一条。
        """
        raw = make_raw(plan_requests=[])
        raw['plans'] = [
            {'stamp': raw['goal_stamp'] - 2.5,          # 上一轮的陈旧路径
             'points': [(0.0, 0.0), (1.0, 0.0)]},
            {'stamp': raw['goal_stamp'] + 0.4,          # 本轮真正的首条
             'points': [(0.0, 0.0), (2.0, 0.0)]},
        ]
        row = rm.compute_round(raw)
        assert abs(row['planning_time_median_s'] - 0.4) < 1e-9, (
            '取到的不是 goal_stamp 之后的第一条 plan')
        assert row['planning_time_source'] == 'fallback:goal_to_first_plan'

    def test_only_stale_plans_reports_none_not_a_negative_number(self):
        """一条目标之后的 plan 都没有时，报 None，绝不报负数。

        缺一格容易发现，错一格会被当成真实读数拿去比较。
        """
        raw = make_raw(plan_requests=[])
        raw['plans'] = [{'stamp': raw['goal_stamp'] - 1.0,
                         'points': [(0.0, 0.0), (1.0, 0.0)]}]
        row = rm.compute_round(raw)
        assert row['planning_time_median_s'] is None
        assert row['planning_time_source'] == 'fallback:no_plan_after_goal'

    def test_planning_time_is_never_negative_across_stamp_orderings(self):
        """遍历陈旧/同时/之后三种时序，断言这一列恒非负或为 None。"""
        for offset in (-3.0, -0.001, 0.0, 0.001, 2.0):
            raw = make_raw(plan_requests=[])
            raw['plans'] = [{'stamp': raw['goal_stamp'] + offset,
                             'points': [(0.0, 0.0), (2.0, 0.0)]}]
            v = rm.compute_round(raw)['planning_time_median_s']
            assert v is None or v >= 0.0, (
                f'offset={offset} 得到 {v} —— 规划时长不可能为负')

    def test_real_requests_take_priority_and_label_the_source(self):
        """有 action 状态就用它，且来源列必须说清楚。

        这条路径此前是**死代码** —— 没有任何代码往 plan_requests 里写，
        整列都在走退化口径，而 'fallback:' 前缀让它看起来像偶发退化。
        """
        raw = make_raw(plan_requests=[
            {'accept': 10.0, 'end': 10.3, 'status': 'SUCCEEDED'},
            {'accept': 11.0, 'end': 11.9, 'status': 'SUCCEEDED'},
        ])
        row = rm.compute_round(raw)
        assert row['plan_requests'] == 2
        assert row['planning_time_median_s'] == pytest.approx(0.6)
        assert row['planning_time_max_s'] == pytest.approx(0.9)
        assert row['planning_time_source'] == \
            'compute_path_to_pose/_action/status'

    def test_request_with_end_before_accept_is_dropped(self):
        """两个时间来源不一致造出的负时长，必须被滤掉而不是落盘。"""
        raw = make_raw(plan_requests=[
            {'accept': 50.0, 'end': 10.0, 'status': 'SUCCEEDED'},
        ])
        row = rm.compute_round(raw)
        assert row['plan_requests'] == 1, '条目本身要照实计数'
        assert row['planning_time_median_s'] is None
        assert row['planning_time_source'] == \
            'compute_path_to_pose/_action/status'

    def test_unfinished_request_is_counted_but_has_no_duration(self):
        raw = make_raw(plan_requests=[
            {'accept': 10.0, 'end': None, 'status': None},
        ])
        row = rm.compute_round(raw)
        assert row['plan_requests'] == 1
        assert row['planning_time_median_s'] is None

    def test_canceled_is_not_counted_as_a_planning_failure(self):
        raw = make_raw(plan_requests=[
            {'accept': 10.0, 'end': 10.2, 'status': 'CANCELED'},
            {'accept': 11.0, 'end': 11.2, 'status': 'ABORTED'},
            {'accept': 12.0, 'end': 12.2, 'status': 'SUCCEEDED'},
        ])
        row = rm.compute_round(raw)
        assert row['planning_aborted'] == 1, 'CANCELED 被算进失败了'
        assert row['planning_failure_rate'] == pytest.approx(1.0 / 3.0)


class TestCrossTrackSegmentation:

    def test_uses_the_active_plan_not_the_last_one(self):
        """**核心断言**：横向偏差必须按生效路径分段。

        构造：前半程沿 y=0 走、生效路径就是 y=0；后半程换了一条 y=1 的路径，
        机器人也跟着走到 y=1。
        · 正确（按生效路径）：两段偏差都 ≈0
        · 错误（整轮都比最后一条 y=1 的路径）：前半程会报出 ≈1m 的假偏差
        """
        n = 200
        t = np.arange(n) * 0.05
        x = np.linspace(0.0, 4.0, n)
        y = np.where(t < 5.0, 0.0, 1.0)
        raw = make_raw(
            pose_t=t, pose_x=x, pose_y=y, pose_yaw=np.zeros(n),
            goal=(4.0, 1.0, 0.0),
            scan_t=t, scan_frames=_scan_frames(n),
            cmd_raw_t=t, cmd_raw_vx=np.full(n, 0.2), cmd_raw_vy=np.zeros(n),
            cmd_raw_wz=np.zeros(n),
            cmd_final_t=t, cmd_final_vx=np.full(n, 0.2),
            cmd_final_vy=np.zeros(n), cmd_final_wz=np.zeros(n),
            odom_t=t, odom_vx=np.full(n, 0.2), odom_vy=np.zeros(n),
            plans=[
                {'stamp': -0.1, 'points': [(0.0, 0.0), (4.0, 0.0)]},
                {'stamp': 5.0, 'points': [(0.0, 1.0), (4.0, 1.0)]},
            ])
        row = rm.compute_round(raw)
        assert row['cross_track_max_m'] < 0.05, (
            '横向偏差 %.3f —— 整轮都在比最后一条路径，前半程被算出了假偏差'
            % row['cross_track_max_m'])

    def test_samples_before_the_first_plan_are_not_counted(self):
        """第一条路径之前的样本没有可比对象，必须是 nan 而不是 0。"""
        n = 100
        t = np.arange(n) * 0.05
        raw = make_raw(
            pose_t=t, pose_x=np.linspace(0, 2, n), pose_y=np.zeros(n),
            pose_yaw=np.zeros(n), scan_t=t, scan_frames=_scan_frames(n),
            cmd_raw_t=t, cmd_raw_vx=np.full(n, 0.2), cmd_raw_vy=np.zeros(n),
            cmd_raw_wz=np.zeros(n), cmd_final_t=t,
            cmd_final_vx=np.full(n, 0.2), cmd_final_vy=np.zeros(n),
            cmd_final_wz=np.zeros(n), odom_t=t, odom_vx=np.full(n, 0.2),
            odom_vy=np.zeros(n),
            plans=[{'stamp': 2.5, 'points': [(0.0, 0.0), (2.0, 0.0)]}])
        row = rm.compute_round(raw)
        assert row['cross_track_samples'] < n, (
            '第一条路径之前的样本也被计入了 —— 那段根本没有路径可比')

    def test_no_plan_gives_none_not_zero(self):
        raw = make_raw(plans=[])
        row = rm.compute_round(raw)
        assert row['cross_track_max_m'] is None
        assert row['cross_track_samples'] == 0


class TestOmniHeading:

    def test_pure_strafe_has_large_heading_error_but_small_motion_error(self):
        """**核心断言**：全向底盘横移时，车头误差大不代表跟踪差。

        只报 heading_error 会把一次完全正确的横移判成"跟踪很糟"。
        """
        n = 100
        t = np.arange(n) * 0.05
        # 沿 +y 横移，车头始终朝 +x
        raw = make_raw(
            pose_t=t, pose_x=np.zeros(n), pose_y=np.linspace(0, 2, n),
            pose_yaw=np.zeros(n), goal=(0.0, 2.0, 0.0),
            scan_t=t, scan_frames=_scan_frames(n),
            cmd_raw_t=t, cmd_raw_vx=np.zeros(n),
            cmd_raw_vy=np.full(n, 0.2), cmd_raw_wz=np.zeros(n),
            cmd_final_t=t, cmd_final_vx=np.zeros(n),
            cmd_final_vy=np.full(n, 0.2), cmd_final_wz=np.zeros(n),
            odom_t=t, odom_vx=np.zeros(n), odom_vy=np.full(n, 0.2),
            plans=[{'stamp': -0.1, 'points': [(0.0, 0.0), (0.0, 2.0)]}])
        row = rm.compute_round(raw)
        assert row['heading_error_mean_rad'] > 1.5, '车头误差应当接近 pi/2'
        assert row['motion_dir_error_mean_rad'] < 0.05, (
            '运动方向误差被算大了 —— 那会把一次正确的横移判成跟踪失败')
        assert 'motion_dir_error' in row['heading_note']


class TestCouplingAttenuation:

    def test_attenuation_is_measured(self):
        """**核心断言**：这一列缺了，整个限速扫描是混淆的。

        臂-底盘耦合实测能把底盘压到 15%：两个不同的限速档位完全可能
        产生同一个实际速度，而表面看不出来。
        """
        raw = make_raw()
        raw['cmd_final_vx'] = np.full(len(raw['cmd_final_t']), 0.2 * 0.15)
        row = rm.compute_round(raw)
        assert abs(row['coupling_atten_p50'] - 0.15) < 0.01
        assert row['coupling_atten_samples'] > 0

    def test_no_attenuation_is_one(self):
        row = rm.compute_round(make_raw())
        assert abs(row['coupling_atten_p50'] - 1.0) < 1e-6

    def test_idle_samples_do_not_inflate_the_ratio(self):
        """控制器没要求速度的样本不参与 —— 否则 0/0 会把中位数拉到 1.0。"""
        raw = make_raw()
        n = len(raw['cmd_raw_t'])
        raw['cmd_raw_vx'] = np.where(np.arange(n) < n // 2, 0.0, 0.2)
        raw['cmd_final_vx'] = np.where(np.arange(n) < n // 2, 0.0, 0.05)
        row = rm.compute_round(raw)
        assert abs(row['coupling_atten_p50'] - 0.25) < 0.01, row['coupling_atten_p50']
        assert row['coupling_atten_samples'] <= n // 2 + 1


class TestGateAndOscillation:

    def test_pure_rotation_is_counted_as_gate_closed(self):
        raw = make_raw()
        n = len(raw['cmd_raw_t'])
        raw['cmd_raw_vx'] = np.where(np.arange(n) < 40, 0.0, 0.2)
        raw['cmd_raw_wz'] = np.where(np.arange(n) < 40, 0.3, 0.0)
        row = rm.compute_round(raw)
        assert abs(row['gate_rotate_only_s'] - 2.0) < 0.06, row['gate_rotate_only_s']
        assert row['gate_episodes'] == 1

    def test_moving_while_turning_is_not_gate_closed(self):
        raw = make_raw()
        raw['cmd_raw_wz'] = np.full(len(raw['cmd_raw_t']), 0.3)
        row = rm.compute_round(raw)
        assert row['gate_rotate_only_s'] == 0.0, (
            '边走边转被算成了"闸门关" —— 闸门的语义是只转不走')

    def test_oscillation_counted_on_command_not_on_pose(self):
        raw = make_raw()
        n = len(raw['cmd_raw_t'])
        raw['cmd_raw_wz'] = 0.4 * (-1.0) ** (np.arange(n) // 5)
        row = rm.compute_round(raw)
        assert row['osc_count_angular'] > 10


class TestZeroProgress:

    def _stuck(self):
        """卡死 15 秒：一直下发速度，但位置不动。"""
        n = 750                                   # 15s @50Hz
        t = np.arange(n) * 0.02
        raw = make_raw(
            pose_t=t, pose_x=np.zeros(n), pose_y=np.zeros(n),
            pose_yaw=np.zeros(n), outcome='PAUSED',
            scan_t=t, scan_frames=_scan_frames(n),
            cmd_raw_t=t, cmd_raw_vx=np.full(n, 0.2), cmd_raw_vy=np.zeros(n),
            cmd_raw_wz=np.zeros(n),
            cmd_final_t=t, cmd_final_vx=np.full(n, 0.2),
            cmd_final_vy=np.zeros(n), cmd_final_wz=np.zeros(n),
            odom_t=t, odom_vx=np.zeros(n), odom_vy=np.zeros(n))
        return raw

    def test_stuck_with_command_is_an_event(self):
        row = rm.compute_round(self._stuck())
        assert row['zero_progress_events_proxy'] >= 1

    def test_episodes_are_collapsed(self):
        """**核心断言**：一次卡死不能记成几百次。

        50Hz 采样下，逐样本计数会把一次 15s 的卡死记成数百次事件。
        """
        row = rm.compute_round(self._stuck())
        assert row['zero_progress_events_proxy'] <= 3, (
            '一次卡死被记成 %d 次事件 —— 逐样本计数没有压成段'
            % row['zero_progress_events_proxy'])

    def test_idle_without_command_is_not_an_event(self):
        """没下发速度就不是"无进展" —— 那是正常的等待。"""
        raw = self._stuck()
        raw['cmd_raw_vx'] = np.zeros(len(raw['cmd_raw_t']))
        row = rm.compute_round(raw)
        assert row['zero_progress_events_proxy'] == 0

    def test_downstream_dead_is_separated(self):
        """控制器发了但链路没执行 —— 不该算规划器的账。"""
        raw = self._stuck()
        raw['cmd_final_vx'] = np.zeros(len(raw['cmd_final_t']))
        row = rm.compute_round(raw)
        assert row['zero_progress_events_proxy'] >= 1
        assert row['downstream_dead_ratio'] == 1.0, (
            '下游没执行的情形没有被单列出来，会被读成规划器无进展')

    def test_naming_is_not_false_positive(self):
        """列名不得叫"误判次数"：nav2 在 follow_path 模式下不发布 trip 原因。"""
        row = rm.compute_round(make_raw())
        assert 'zero_progress_events_proxy' in row
        assert not any('false_positive' in k or 'misjudge' in k for k in row)


class TestSafetyProxies:

    def test_geometric_intrusion_is_not_named_collision(self):
        """**核心断言**：本机没有碰撞传感器，不能把几何侵入叫成碰撞。"""
        row = rm.compute_round(make_raw())
        assert 'geometric_intrusion_episodes_proxy' in row
        assert row['collisions_manual'] is None, '真实碰撞只能人工填'
        assert not any(k == 'collisions' or k == 'collision_count' for k in row)

    def test_intrusion_fires_when_obstacle_inside_inscribed(self):
        raw = make_raw()
        n = len(raw['scan_t'])
        frames = _scan_frames(n)
        for i in range(50, 80):
            frames[i] = sm.frame_stats(
                np.array([5.0, 2.0, 0.20, 2.0]), -math.pi, math.pi / 2.0,
                0.05, 20.0, circumscribed=0.42)
        raw['scan_frames'] = frames
        row = rm.compute_round(raw)
        assert row['geometric_intrusion_episodes_proxy'] == 1
        assert row['geometric_intrusion_s'] > 1.0

    def test_missing_footprint_gives_none_not_zero(self):
        raw = make_raw(footprint_inscribed=None, footprint_circumscribed=None)
        row = rm.compute_round(raw)
        assert row['geometric_intrusion_episodes_proxy'] is None
        assert row['min_clearance_m'] is None
        assert row['self_filter_residual_ratio_proxy'] is None

    def test_footprint_change_is_flagged(self):
        """足迹被临时缩放过（本项目做过 36% 的临时足迹）时净空前后不是同一口径。"""
        row = rm.compute_round(make_raw(footprint_changed=True))
        assert row['footprint_changed_in_round'] is True


class TestProxyDefinitionsShipWithData:

    def test_every_proxy_column_has_a_definition(self):
        """**核心断言**：代理量的定义必须跟着数据走。

        定义只写在代码注释里，半年后没人能复核那张表。
        """
        row = rm.compute_round(make_raw())
        proxies = sorted(k for k in row if k.endswith('_proxy'))
        assert proxies, '一个 _proxy 列都没有，命名约定失效了'
        missing = [k for k in proxies if k not in rm.PROXY_DEFINITIONS]
        assert missing == [], '这些代理量没有写定义：%s' % missing

    def test_definition_of_zero_progress_disclaims_false_positive(self):
        d = rm.PROXY_DEFINITIONS['zero_progress_events_proxy']
        assert '误判' in d and '无法判定' in d

    def test_definition_of_intrusion_disclaims_collision(self):
        d = rm.PROXY_DEFINITIONS['geometric_intrusion_episodes_proxy']
        assert '不是碰撞次数' in d


class TestSummarize:

    def test_success_rate(self):
        ok = rm.compute_round(make_raw())
        bad = rm.compute_round(make_raw(outcome='PAUSED'))
        s = rm.summarize([ok, ok, bad])
        assert s['rounds'] == 3
        assert abs(s['success_rate'] - 2.0 / 3.0) < 1e-12

    def test_none_values_are_excluded_and_n_is_reported(self):
        """**核心断言**：None 不能当 0 参与平均，且 n 必须报出来。

        n 与轮数差很多时，那个均值不能直接拿来比档位。
        """
        ok = rm.compute_round(make_raw())
        blind = rm.compute_round(make_raw(plans=[]))     # 无横向偏差数据
        s = rm.summarize([ok, blind])
        assert s['cross_track_max_m__n'] == 1, (
            'None 被当成 0 参与了平均，均值会被系统性拉低')
        assert s['rounds'] == 2

    def test_empty_input(self):
        assert rm.summarize([])['rounds'] == 0

    def test_sd_needs_two_samples(self):
        ok = rm.compute_round(make_raw())
        s = rm.summarize([ok])
        assert s['arrival_error_xy_m__sd'] is None


class TestDegenerateInput:

    @pytest.mark.parametrize('drop', [
        'pose_t', 'plans', 'cmd_raw_t', 'cmd_final_t', 'odom_t',
        'scan_t', 'scan_frames', 'plan_requests', 'settle_x',
    ])
    def test_missing_key_does_not_raise(self, drop):
        """一轮采集不全不该让整次扫描的数据全丢。"""
        raw = make_raw()
        raw.pop(drop, None)
        row = rm.compute_round(raw)
        assert isinstance(row, dict)
        assert row['round_index'] == 0

    def test_numpy_arrays_do_not_trip_truthiness(self):
        """`raw.get(k) or []` 会对长度>1 的 ndarray 抛 ValueError。

        这个错误只在数组长度 > 1 时出现 —— 长度 0/1 的小样本测试完全测不出来，
        所以必须专门用长数组测一次。
        """
        row = rm.compute_round(make_raw())
        assert row['pose_samples'] == 200

    def test_all_empty_round(self):
        raw = dict(index=7, goal=(1.0, 2.0, 0.0), goal_stamp=0.0,
                   end_stamp=1.0, outcome='PAUSED',
                   pose_t=[], pose_x=[], pose_y=[], pose_yaw=[])
        row = rm.compute_round(raw)
        assert row['round_index'] == 7
        assert row['success'] is False
        assert row['arrival_error_xy_m'] is None


class TestRoundScale:

    def test_a_ten_minute_round_completes(self):
        """10 分钟 @50Hz + 800 点路径 + 10Hz 激光，端到端一轮必须算得动。"""
        import time
        n = 30_000
        t = np.arange(n) * 0.02
        x = np.linspace(0, 60, n)
        raw = make_raw(
            pose_t=t, pose_x=x, pose_y=0.1 * np.sin(x), pose_yaw=np.zeros(n),
            goal=(60.0, 0.0, 0.0),
            settle_x=np.full(50, 60.0), settle_y=np.zeros(50),
            settle_yaw=np.zeros(50),
            plans=[{'stamp': -0.1, 'points': np.column_stack([
                np.linspace(0, 60, 800), 0.1 * np.sin(np.linspace(0, 60, 800))])}],
            cmd_raw_t=t, cmd_raw_vx=np.full(n, 0.2), cmd_raw_vy=np.zeros(n),
            cmd_raw_wz=np.zeros(n),
            cmd_final_t=t, cmd_final_vx=np.full(n, 0.2),
            cmd_final_vy=np.zeros(n), cmd_final_wz=np.zeros(n),
            odom_t=t, odom_vx=np.full(n, 0.2), odom_vy=np.zeros(n),
            scan_t=np.arange(6000) * 0.1, scan_frames=_scan_frames(6000))
        t0 = time.time()
        row = rm.compute_round(raw)
        elapsed = time.time() - t0
        assert row['pose_samples'] == n
        assert elapsed < 60.0, '一轮 10 分钟的数据算了 %.1fs' % elapsed
