# Copyright 2026 Astribot.
#
# 一轮探索导航的原始时序 → 一行指标。**纯函数，无 ROS 依赖**。
#
# 这里是**唯一的判据出口**：录制节点只负责订阅与落盘，任何"算出一个数"的逻辑
# 都在这个文件里，因此都能被单测覆盖。上次姿态监控的顺序变异之所以能在
# 20 条测试下活下来，就是因为判据留在了回调里。
#
# ======================== 输出里的命名约定（很重要）========================
#   xxx                 直接可测量
#   xxx_proxy           **代理量**：定义写在 PROXY_DEFINITIONS 里，随数据一起落盘
#   None / 空           没有合格样本。**绝不用 0 代替** ——
#                       "0 次震荡"和"没测到震荡"在汇总表里必须区分得开
# ========================================================================
import math

import numpy as np

from . import geometry, scan_metrics, signals

# ------------------------------------------------------------------ 阈值默认值
#
# 每一项都写清来源。没有来源的阈值等于把结论建在猜上。
DEFAULTS = {
    # 位移噪声门限：TF 位姿抖动，低于此值的单步不计入里程
    'min_step_m': 0.002,
    # 实测轨迹算平滑度前的抽稀步长。不抽稀时算出的是传感器噪声
    'track_decimate_m': 0.05,
    # 超调的"进入方向"取首次进入容差前这么长里程的净位移
    'approach_span_m': 0.5,
    # 闸门关（只转不走）判据
    'gate_v_eps': 0.02,          # m/s
    'gate_w_eps': 0.05,          # rad/s
    # 震荡：变号两侧幅值都要超过它，且两次 excursion 间隔不超过窗口
    'osc_w_min': 0.15,           # rad/s
    'osc_v_min': 0.05,           # m/s
    'osc_window_s': 2.0,
    # 速度跟踪误差的时延搜索上界。本机链路实测滞后约 0.6s，1.5 给足余量
    'max_lag_s': 1.5,
    'lag_grid_dt': 0.02,
    # 零进展事件：复算 nav2 PoseProgressChecker 的判据。
    # 默认值取自 nav2_params_mppi.yaml 的 progress_checker
    'progress_radius_m': 0.5,
    'progress_window_s': 10.0,
    # 动态障碍：前向净空在 dyn_window_s 内跌落超过 dyn_drop_m 视为障碍出现
    'dyn_drop_m': 0.30,
    'dyn_window_s': 0.3,
    'dyn_brake_ratio': 0.8,
    # 窄通道
    'narrow_width_m': scan_metrics.DEFAULT_NARROW_WIDTH_M,
    'narrow_min_episode_s': 0.5,
    'narrow_min_progress_m': 0.3,
    'corridor_max_range_m': scan_metrics.DEFAULT_CORRIDOR_MAX_RANGE,
}

# 随数据一起落盘。代理量的定义不跟着数据走，半年后没人能复核这张表。
PROXY_DEFINITIONS = {
    'narrow_success_rate_proxy':
        '窄段=连续>=narrow_min_episode_s 满足 (left_min+right_min)<narrow_width_m；'
        '成功=段末沿路径进展-段初>=narrow_min_progress_m 且该轮未失败。'
        '真正的"该不该穿过去"没有真值，故为代理量。段数 0 时为 None，不是 1.0。'
        '⚠ left_min+right_min 是**墙到墙净宽**，两者都从传感器原点量起，'
        '机器人自己占的那部分已含在内 —— 绝不能再加 2*外接半径。'
        '旧口径多加了一遍(本机 +0.868m)，实测使 3270 帧里 <1.62m 的从 1382 帧'
        '(42.3%)变成 0 帧，四个 narrow_* 列全部恒空，且不报任何错。',
    'zero_progress_events_proxy':
        '按 nav2 PoseProgressChecker 的判据从实测数据复算：'
        '滑窗 progress_window_s 内下发速度非零而 map 系位移 < progress_radius_m。'
        '**不叫"误判次数"**：follow_path 模式没有 BT，nav2 不发布 trip 原因，'
        '真伪无法判定。同时给 downstream_dead_ratio（窗内最终 /cmd_vel 也≈0 的占比），'
        '那部分是链路没执行，不该算规划器的账。',
    'geometric_intrusion_episodes_proxy':
        'min(有效 range) < 足迹**内切**半径 的连续段数。'
        '**不是碰撞次数** —— 本机没有碰撞传感器，真实碰撞由 --collisions 人工填。'
        '⚠ 用内切半径是保守口径：最近回波的**方位角没有记录**，因此无法判断它落在'
        '八边形的哪条边上。内切与外接相差 35mm（0.3992 / 0.4342），回波落在两者'
        '之间时是否真的侵入八边形**由现存数据不可判定**。同理 min_clearance_m'
        '（按外接半径算）为负也**不构成侵入证据** —— 两列几何口径不同，不是算术矛盾。',
    'self_filter_residual_ratio_proxy':
        'range < 外接半径 的光束占比。自滤生效时物理上应恒为 0；'
        '非 0 即机器人把自身结构当障碍（历史故障：夹爪不在自滤链里）。',
    'brake_latency_s_proxy':
        '前向 ±30° 最近回波在 dyn_window_s 内跌落 > dyn_drop_m 视为障碍出现，'
        '延迟 = 到 |cmd_vel| 降至事发时的 dyn_brake_ratio 倍所用时间。'
        '量的是**刹车响应**，不是识别延迟；基准速度已≈0 的事件跳过并计入 skipped。',
}

OUTCOME_SUCCESS = 'ARRIVED'


def _f(x):
    """写 CSV 用：None 保持 None，numpy 标量转成 float。"""
    if x is None:
        return None
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    v = float(x)
    return None if not math.isfinite(v) else v


def _seq(raw, key):
    """取一条时序，缺失或 None 时给空列表。

    **不能写 `raw.get(k) or []`**：raw 里的时序既可能是 list 也可能是
    ndarray，而 ndarray 参与 `or` 会走 __bool__ 抛
    "truth value of an array with more than one element is ambiguous"。
    这个错误只在数组长度 > 1 时出现 —— 也就是说小样本测试（长度 0/1）
    完全测不出来，恰好是最容易漏的一类。
    """
    v = raw.get(key)
    if v is None:
        return []
    return v


def cumulative_progress(px, py, min_step_m):
    """逐样本累计里程（用于"沿路径进展"）。返回与输入等长的数组。"""
    pts = geometry.as_xy(np.column_stack([px, py])) if len(px) else geometry.as_xy([])
    if len(pts) == 0:
        return np.zeros(0)
    if len(pts) == 1:
        return np.zeros(1)
    steps = np.hypot(*np.diff(pts, axis=0).T)
    steps[steps < float(min_step_m)] = 0.0
    return np.concatenate(([0.0], np.cumsum(steps)))


def active_plan_index(plan_stamps, sample_stamps):
    """每个样本时刻生效的路径下标。早于第一条路径的样本给 -1。

    生效区间是 [plans[i].stamp, plans[i+1].stamp)：控制器拿到新路径的那一刻
    旧路径就作废。用"最近的一条"而不是"最后一条"很关键 —— 拿最后一条去算
    整轮的横向偏差，等于用机器人还没见过的路径去评判它之前的行为。
    """
    ps = np.asarray(plan_stamps, dtype=float)
    ts = np.asarray(sample_stamps, dtype=float)
    if ps.size == 0 or ts.size == 0:
        return np.full(ts.shape, -1, dtype=int)
    return np.searchsorted(ps, ts, side='right') - 1


def cross_track_series(pose_t, pose_x, pose_y, plans):
    """逐样本横向偏差与最近段切向。无生效路径的样本给 nan。

    按"生效路径"分段计算，而不是把所有位姿都拿去比最后一条路径。
    """
    n = len(pose_t)
    dist = np.full(n, np.nan)
    tang = np.full(n, np.nan)
    if n == 0 or not plans:
        return dist, tang
    idx = active_plan_index([p['stamp'] for p in plans], pose_t)
    px = np.asarray(pose_x, dtype=float)
    py = np.asarray(pose_y, dtype=float)
    for k in range(len(plans)):
        sel = np.flatnonzero(idx == k)
        if sel.size == 0:
            continue
        poly = plans[k]['points']
        q = np.column_stack([px[sel], py[sel]])
        d, seg, _t = geometry.cross_track_distances(q, poly)
        dist[sel] = d
        tang[sel] = geometry.tangent_at(poly, seg)
    return dist, tang


def zero_progress_events(pose_t, pose_x, pose_y, cmd_t, cmd_speed,
                         final_t, final_speed, radius_m, window_s,
                         v_eps=0.02):
    """复算 PoseProgressChecker 的"无进展"判据。见 PROXY_DEFINITIONS。

    对每个位姿样本 i：取窗口 [t_i - window_s, t_i]，
      若窗内**有下发速度**（cmd 峰值 > v_eps）而位移 < radius_m → 计一次事件。
    连续满足的样本压成一段，一段算一次事件（否则 50Hz 采样下一次卡死会记 500 次）。

    再看该窗内**最终** /cmd_vel 的峰值：也 ≈0 的话，说明控制器发了但链路
    没执行（臂-底盘耦合/写门/联锁），这类计入 downstream_dead。
    """
    t = np.asarray(pose_t, dtype=float)
    if t.size < 2:
        return {'zero_progress_events_proxy': 0, 'downstream_dead_ratio': None}
    x = np.asarray(pose_x, dtype=float)
    y = np.asarray(pose_y, dtype=float)
    ct = np.asarray(cmd_t, dtype=float)
    cs = np.abs(np.asarray(cmd_speed, dtype=float))
    ft = np.asarray(final_t, dtype=float)
    fs = np.abs(np.asarray(final_speed, dtype=float))

    lo_idx = np.searchsorted(t, t - float(window_s), side='left')
    flag = np.zeros(t.size, dtype=bool)
    dead = np.zeros(t.size, dtype=bool)
    for i in range(t.size):
        j = lo_idx[i]
        if t[i] - t[j] < float(window_s) * 0.9:
            continue                                  # 窗口还没攒满
        disp = float(np.max(np.hypot(x[j:i + 1] - x[j], y[j:i + 1] - y[j])))
        if disp >= float(radius_m):
            continue
        cm = ct >= t[j]
        cm &= ct <= t[i]
        if not np.any(cm) or float(np.max(cs[cm])) <= v_eps:
            continue                                  # 没下发速度，不是"无进展"
        flag[i] = True
        fm = (ft >= t[j]) & (ft <= t[i])
        dead[i] = (not np.any(fm)) or float(np.max(fs[fm])) <= v_eps

    eps = signals.episodes(t, flag, 0.0)
    if not eps:
        return {'zero_progress_events_proxy': 0, 'downstream_dead_ratio': None}
    dead_eps = 0
    for t0, t1 in eps:
        m = (t >= t0) & (t <= t1)
        if np.any(dead[m]):
            dead_eps += 1
    return {
        'zero_progress_events_proxy': len(eps),
        'downstream_dead_ratio': signals.safe_div(dead_eps, len(eps)),
    }


def _arrival(raw, row):
    """到位位姿与误差。全部在 map 系。

    位姿取**轮次终止时刻最后一个 TF 样本**。绝不用 /odom ——
    /odom 是轮式里程计，它的 (x,y) 不在 map 系。本项目曾把 /odom 的 y
    与 map 系目标的 y 相减，造出 0.43m 的假误差，再去排查自己造的矛盾。
    """
    gx, gy, gyaw = raw['goal']
    row['goal_x'] = _f(gx)
    row['goal_y'] = _f(gy)
    row['goal_yaw'] = _f(gyaw)
    px = _seq(raw, 'pose_x')
    if not len(px):
        row['actual_x'] = row['actual_y'] = row['actual_yaw'] = None
        row['arrival_error_xy_m'] = row['arrival_error_yaw_rad'] = None
        return
    ax, ay = float(px[-1]), float(_seq(raw, 'pose_y')[-1])
    ayaw = float(_seq(raw, 'pose_yaw')[-1])
    row['actual_x'] = _f(ax)
    row['actual_y'] = _f(ay)
    row['actual_yaw'] = _f(ayaw)
    # 「到位精度差距」与「位置到位误差」是同一个量，只出一列，避免两列不一致
    row['arrival_error_xy_m'] = _f(math.hypot(ax - gx, ay - gy))
    row['arrival_error_yaw_rad'] = _f(geometry.wrap_angle_scalar(ayaw - gyaw))


def _pose_coverage(raw, row):
    """位姿采集覆盖率：所有由位姿导出的列都靠它定生死。

    为什么必须有这一列（本项目实测踩到）：轮 1/2 的 duration_s 是 7.18s / 7.12s，
    而落进本轮的位姿样本只有 **10 个和 1 个**，traveled_m 因此是 0.000 ——
    然而 arrival_error_xy_m 照样出数（0.053 / 0.083 m），看上去是漂亮的到位精度。
    单样本算不出"到位"，它只是一张快照。

    这不是 TF 失败：同期 tf_lookup_fail 全程只有 4 次、tf_lookup_ok 每轮 200+。
    样本是在 `self.round is None` 的窗口外被丢掉的，即**轮次开启时刻晚于它自己
    记录的 goal_stamp**，于是 duration_s 覆盖的时间里大半根本没在录。

    口径只用本轮自带的数据（位姿时间跨度 / 轮次时长），不引外部标称频率 ——
    标称频率会被参数写错静默改掉，而覆盖率是自证的。
    """
    pt = np.asarray(_seq(raw, 'pose_t'), dtype=float)
    dur = row.get('duration_s')
    row['pose_span_s'] = _f(float(pt[-1] - pt[0])) if pt.size >= 2 else None
    row['pose_rate_measured_hz'] = None
    row['pose_coverage_ratio'] = None
    if pt.size >= 2 and row['pose_span_s'] and row['pose_span_s'] > 0.0:
        row['pose_rate_measured_hz'] = _f(pt.size / row['pose_span_s'])
    if dur and dur > 0.0 and row['pose_span_s'] is not None:
        row['pose_coverage_ratio'] = signals.safe_div(row['pose_span_s'], dur)
    # 覆盖率 < 0.5 或样本 < 2 ⇒ 到位误差/里程/横偏这一行都不是测量结果。
    # 阈值 0.5 的来源：低于一半就意味着轮次时长里多数时间没有位姿，
    # 此时 traveled_m 必然偏小而 arrival_error 退化成快照，两者都会被误读成好成绩。
    row['pose_coverage_suspicious'] = bool(
        pt.size < 2
        or (row['pose_coverage_ratio'] is not None
            and row['pose_coverage_ratio'] < 0.5))


def _path_metrics(raw, cfg, row):
    """路径相关：长度比、平滑度、重规划、规划耗时与失败率。"""
    plans = _seq(raw, 'plans')
    px, py = _seq(raw, 'pose_x'), _seq(raw, 'pose_y')
    track = np.column_stack([px, py]) if len(px) else np.zeros((0, 2))

    traveled, dropped = geometry.traveled_length(track, cfg['min_step_m'])
    row['traveled_m'] = _f(traveled)
    row['traveled_steps_below_gate'] = int(dropped)
    row['min_step_gate_m'] = _f(cfg['min_step_m'])
    # 两个口径都给，谁也不必信那个门限。
    #
    # 为什么必须这样：门限 2mm、位姿 20Hz、限速档位 0.1m/s 时单步只有 5mm，
    # 只差 2.5 倍；掉到 50Hz 就是 2mm，与门限**同量级**，此时门限会把
    # 真实位移当抖动丢掉，"路径长度比"直接变成 0 而看不出原因。
    # 限速扫描恰恰要往低速档走，所以这个风险是真实的，不是理论上的。
    row['traveled_m_ungated'] = _f(geometry.polyline_length(track))
    n_steps = max(len(track) - 1, 0)
    row['traveled_gate_dropped_ratio'] = signals.safe_div(dropped, n_steps)
    # 丢弃过半 → 采样密度与门限不匹配，traveled_m 不可用，看 ungated
    row['traveled_gate_suspicious'] = bool(
        n_steps > 0 and dropped > 0.5 * n_steps)

    ref_len = geometry.polyline_length(plans[0]['points']) if plans else 0.0
    row['first_plan_len_m'] = _f(ref_len) if ref_len > 0 else None
    row['path_len_ratio_vs_plan'] = signals.safe_div(traveled, ref_len) \
        if ref_len > 0 else None

    gx, gy, _ = raw['goal']
    if len(_seq(raw, 'pose_x')):
        straight = math.hypot(gx - float(_seq(raw, 'pose_x')[0]),
                              gy - float(_seq(raw, 'pose_y')[0]))
        row['straight_line_m'] = _f(straight)
        row['path_len_ratio_vs_straight'] = signals.safe_div(traveled, straight)
    else:
        row['straight_line_m'] = row['path_len_ratio_vs_straight'] = None

    # 平滑度两组：规划路径不抽稀（顶点是确定值），实测轨迹必须抽稀
    last_plan = plans[-1]['points'] if plans else []
    sp = geometry.smoothness(last_plan, min_step=0.0)
    row['plan_turn_per_m'] = _f(sp['turn_per_m'])
    row['plan_max_turn_rad'] = _f(sp['max_turn_rad'])
    st = geometry.smoothness(track, min_step=cfg['track_decimate_m'])
    row['track_turn_per_m'] = _f(st['turn_per_m'])
    row['track_max_turn_rad'] = _f(st['max_turn_rad'])
    row['track_decimate_m'] = _f(cfg['track_decimate_m'])

    # 重规划次数：路径内容变化才算。逐字相同的重复发布不算 ——
    # nav2 在某些配置下会周期性重发同一条路径，计进去会得出"每秒重规划一次"
    changed = 0
    for i in range(1, len(plans)):
        a, b = plans[i - 1]['points'], plans[i]['points']
        if len(a) != len(b) or not np.allclose(
                geometry.as_xy(a), geometry.as_xy(b), atol=1e-9):
            changed += 1
    row['plans_published'] = len(plans)
    row['replan_count'] = changed

    reqs = _seq(raw, 'plan_requests')
    durations = [r['end'] - r['accept'] for r in reqs
                 if r.get('end') is not None and r.get('accept') is not None
                 and r['end'] >= r['accept']]
    aborted = sum(1 for r in reqs if r.get('status') == 'ABORTED')
    row['plan_requests'] = len(reqs)
    row['planning_time_median_s'] = _f(signals.median_or_none(durations))
    row['planning_time_max_s'] = _f(signals.stat_or_none(durations, np.max))
    row['planning_failure_rate'] = signals.safe_div(aborted, len(reqs))
    row['planning_aborted'] = aborted
    if not reqs and plans and len(_seq(raw, 'pose_t')):
        # 退化口径：拿不到 action 状态时用「下发 → 首条 /plan」。
        #
        # 🔴 只能取 goal_stamp **之后**的那条 /plan。nav2 周期性重规划，
        #    本轮缓冲里的第一条 /plan 完全可能是**上一轮**留下的陈旧路径，
        #    直接相减就是负数 —— 时长不可能为负。
        #    实测（首次真机图外的仿真跑，三轮全中）：
        #      planning_time_median_s = -0.496 / -2.507 / -0.297 s
        #    即首条 plan 比目标下发早 0.3~2.5s。这一列此前把陈旧读数
        #    当成了当前值，而 210 条单测全绿 —— 因为测试只喂了目标之后
        #    的那一条 plan，陈旧分支一次都没被走到。
        after = [p for p in plans if p['stamp'] >= raw['goal_stamp']]
        if after:
            row['planning_time_median_s'] = _f(
                after[0]['stamp'] - raw['goal_stamp'])
            row['planning_time_source'] = 'fallback:goal_to_first_plan'
        else:
            # 宁可报"没有"，也不报一个负数。缺这一格比错一格容易发现。
            row['planning_time_median_s'] = None
            row['planning_time_source'] = 'fallback:no_plan_after_goal'
    elif reqs:
        row['planning_time_source'] = 'compute_path_to_pose/_action/status'
    else:
        # reqs 为空时来源绝不是 action 状态，别贴错标签。
        row['planning_time_source'] = 'none'


def _tracking_metrics(raw, cfg, row):
    """跟踪类：横向偏差、航向/运动方向跟踪、速度跟踪、频率、震荡、闸门。"""
    pose_t = np.asarray(_seq(raw, 'pose_t'), dtype=float)
    dist, tang = cross_track_series(
        _seq(raw, 'pose_t'), _seq(raw, 'pose_x'), _seq(raw, 'pose_y'), _seq(raw, 'plans'))
    row['cross_track_max_m'] = _f(signals.stat_or_none(dist, np.max))
    row['cross_track_mean_m'] = _f(signals.weighted_mean(pose_t, dist))
    # p95 才是能比档位的那个数：max 是单样本极值，一次 TF 抖动就能主导整轮。
    # 之前报告脚本要 cross_track_p95_m 而这一列从不存在 —— ROS 那边参数写错
    # 会静默忽略，CSV 这边列名写错则表现为"n=0 没测到"，两者一样不报错。
    row['cross_track_p95_m'] = _f(
        signals.stat_or_none(dist, lambda a: np.percentile(a, 95.0)))
    row['cross_track_samples'] = int(np.count_nonzero(np.isfinite(dist)))

    yaws = np.asarray(_seq(raw, 'pose_yaw'), dtype=float)
    if yaws.size and tang.size:
        he = np.abs(geometry.wrap_angle(yaws - tang))
        row['heading_error_max_rad'] = _f(signals.stat_or_none(he, np.max))
        row['heading_error_mean_rad'] = _f(signals.weighted_mean(pose_t, he))
    else:
        row['heading_error_max_rad'] = row['heading_error_mean_rad'] = None

    # 全向底盘可横移：车头误差大**不等于**跟踪差。真正该看的是运动方向。
    ve = _motion_direction_error(raw, tang, cfg)
    row['motion_dir_error_max_rad'] = _f(signals.stat_or_none(ve, np.max))
    row['motion_dir_error_mean_rad'] = _f(signals.weighted_mean(pose_t, ve))
    row['heading_note'] = ('全向底盘：heading_error 大不代表跟踪差，'
                           '请看 motion_dir_error')

    raw_t = _seq(raw, 'cmd_raw_t')
    fin_t = _seq(raw, 'cmd_final_t')
    rate = signals.rate_from_stamps(raw_t)
    row['local_plan_hz'] = _f(rate['hz'])
    row['local_plan_gap_p95_s'] = _f(rate['gap_p95'])
    row['local_plan_gap_max_s'] = _f(rate['gap_max'])
    row['cmd_raw_msgs'] = rate['n']
    row['cmd_final_msgs'] = len(fin_t)

    raw_speed = signals.hypot_series(
        _seq(raw, 'cmd_raw_vx'), _seq(raw, 'cmd_raw_vy'))
    fin_speed = signals.hypot_series(
        _seq(raw, 'cmd_final_vx'), _seq(raw, 'cmd_final_vy'))
    odom_speed = signals.hypot_series(
        _seq(raw, 'odom_vx'), _seq(raw, 'odom_vy'))

    lag = signals.best_lag_and_residual(
        fin_t, fin_speed, _seq(raw, 'odom_t'), odom_speed,
        max_lag_s=cfg['max_lag_s'], grid_dt=cfg['lag_grid_dt'])
    row['vel_track_rms_at_best_lag'] = _f(lag['rms_at_best'])
    row['vel_track_rms_at_zero_lag'] = _f(lag['rms_at_zero'])
    row['vel_track_best_lag_s'] = _f(lag['best_lag_s'])
    row['vel_track_gain'] = _f(lag['gain'])
    row['vel_track_lag_hit_boundary'] = bool(lag['hit_boundary'])
    row['vel_track_note'] = ('位置积分式链路实测滞后约 0.6s，'
                             '零时延残差主要成分是时延，不是跟踪误差；'
                             'hit_boundary=True 时 best_lag 不可信')

    # 臂-底盘耦合衰减：不记这一列，整个限速扫描是混淆的（耦合实测可压到 15%）
    att = _coupling_attenuation(raw_t, raw_speed, fin_t, fin_speed, cfg)
    row.update(att)

    row['osc_count_angular'] = signals.oscillation_count(
        raw_t, _seq(raw, 'cmd_raw_wz'), cfg['osc_w_min'], cfg['osc_window_s'])
    row['osc_count_linear'] = signals.oscillation_count(
        raw_t, _seq(raw, 'cmd_raw_vx'), cfg['osc_v_min'], cfg['osc_window_s'])

    gate = _gate_metrics(raw_t, raw_speed, _seq(raw, 'cmd_raw_wz'), cfg)
    row.update(gate)


def _motion_direction_error(raw, tang, cfg):
    """实际运动方向与路径切向的夹角。位移过小的样本给 nan。

    位移门限用 min_step_m：静止时 atan2 出来的是噪声方向，
    不设门限会在停车段贡献一堆接近均匀分布的"跟踪误差"。
    """
    x = np.asarray(_seq(raw, 'pose_x'), dtype=float)
    y = np.asarray(_seq(raw, 'pose_y'), dtype=float)
    if x.size < 2 or tang.size != x.size:
        return np.zeros(0)
    dx = np.diff(x, prepend=x[0])
    dy = np.diff(y, prepend=y[0])
    step = np.hypot(dx, dy)
    out = np.full(x.size, np.nan)
    ok = (step >= float(cfg['min_step_m'])) & np.isfinite(tang)
    out[ok] = np.abs(geometry.wrap_angle(np.arctan2(dy[ok], dx[ok]) - tang[ok]))
    return out


def _coupling_attenuation(raw_t, raw_speed, fin_t, fin_speed, cfg):
    """控制器输出 → 底盘实收 的衰减比。

    为什么必须有这一列：链路是
      /cmd_vel_nav_body_raw → /cmd_vel_nav_body → /cmd_vel_pre_arm_coupling → /cmd_vel
    臂-底盘耦合实测能把底盘压到 **15%**（folded_reference 是全零奇异构型）。
    两个不同的限速档位完全可能产生同一个实际速度 ——
    不记衰减比，"不同限速下的到位精度"这张表就是混淆的，而表面看不出来。

    只在控制器确实要求了速度（raw > gate_v_eps）的样本上取比值，
    否则 0/0 的样本会把中位数拉到 1.0。
    """
    if len(raw_t) == 0 or len(fin_t) == 0:
        return {'coupling_atten_p50': None, 'coupling_atten_p05': None,
                'coupling_atten_samples': 0}
    aligned = signals.resample_zoh(fin_t, fin_speed, raw_t)
    rs = np.asarray(raw_speed, dtype=float)
    ok = np.isfinite(aligned) & (rs > float(cfg['gate_v_eps']))
    if not np.any(ok):
        return {'coupling_atten_p50': None, 'coupling_atten_p05': None,
                'coupling_atten_samples': 0}
    ratio = aligned[ok] / rs[ok]
    return {
        'coupling_atten_p50': _f(np.median(ratio)),
        'coupling_atten_p05': _f(np.percentile(ratio, 5.0)),
        'coupling_atten_samples': int(np.count_nonzero(ok)),
    }


def _gate_metrics(raw_t, raw_speed, wz, cfg):
    """闸门关（只转不走）时长。

    判据：下发线速度被压到 ~0（< gate_v_eps）而角速度仍在输出
    （> gate_w_eps）。这正是三段式 ALIGN 相位的特征 —— 那一段 vx/vy
    严格为 0、只出 wz。

    只能从**下发速度**观察：控制器内部的相位不发布任何话题，
    所以这个指标是相位的**代理**而不是相位本身。真正的原地对齐
    与"卡住了只能干转"在这个指标上不可区分，读数要配合位移一起看。
    """
    t = np.asarray(raw_t, dtype=float)
    v = np.asarray(raw_speed, dtype=float)
    w = np.abs(np.asarray(wz, dtype=float))
    n = min(t.size, v.size, w.size)
    if n == 0:
        return {'gate_rotate_only_s': None, 'gate_episodes': 0,
                'gate_longest_s': None}
    t, v, w = t[:n], v[:n], w[:n]
    flag = (v < float(cfg['gate_v_eps'])) & (w > float(cfg['gate_w_eps']))
    eps = signals.episodes(t, flag, 0.0)
    return {
        'gate_rotate_only_s': _f(signals.duration_where(t, flag)),
        'gate_episodes': len(eps),
        'gate_longest_s': _f(max((b - a for a, b in eps), default=None)
                             if eps else None),
    }


def _scan_and_safety(raw, cfg, row):
    """激光类：净空、居中度、窄段、自滤残留、几何侵入、刹车响应延迟。"""
    frames = _seq(raw, 'scan_frames')
    scan_t = _seq(raw, 'scan_t')
    inscribed = raw.get('footprint_inscribed')
    circumscribed = raw.get('footprint_circumscribed')

    row['footprint_inscribed_m'] = _f(inscribed)
    row['footprint_circumscribed_m'] = _f(circumscribed)
    # 足迹可能被临时缩放过（本项目做过 36% 的临时足迹）。轮内变化必须标出来：
    # 变了之后净空这一列前后不是同一个口径。
    row['footprint_changed_in_round'] = bool(raw.get('footprint_changed', False))

    row.update(scan_metrics.aggregate(
        frames, scan_t, circumscribed=circumscribed,
        narrow_width_m=cfg['narrow_width_m']))

    # 沿路径进展插值到激光时刻，供窄段成功判据用。
    # np.interp 的 xp 为空时会抛 ValueError("array of sample points is empty")，
    # 所以必须先判 prog.size —— 一轮位姿采集不全不该让整轮数据全丢。
    prog = cumulative_progress(_seq(raw, 'pose_x'), _seq(raw, 'pose_y'),
                               cfg['min_step_m'])
    pose_t = np.asarray(_seq(raw, 'pose_t'), dtype=float)
    if len(scan_t) and prog.size and pose_t.size:
        m = min(prog.size, pose_t.size)
        prog_at_scan = np.interp(
            np.asarray(scan_t, dtype=float), pose_t[:m], prog[:m])
    else:
        prog_at_scan = np.zeros(len(scan_t))
    row.update(scan_metrics.narrow_traversals(
        scan_t, frames, prog_at_scan,
        narrow_width_m=cfg['narrow_width_m'],
        min_episode_s=cfg['narrow_min_episode_s'],
        min_progress_m=cfg['narrow_min_progress_m'],
        round_failed=(row['outcome'] != OUTCOME_SUCCESS)))

    # 几何侵入（**不是碰撞**）：最近回波落进内切半径
    if inscribed is not None and frames:
        intr = np.asarray(
            [(f['min_range'] is not None and f['min_range'] < float(inscribed))
             for f in frames], dtype=bool)
        eps = signals.episodes(scan_t, intr, 0.0)
        row['geometric_intrusion_episodes_proxy'] = len(eps)
        row['geometric_intrusion_s'] = _f(signals.duration_where(scan_t, intr))
    else:
        row['geometric_intrusion_episodes_proxy'] = None
        row['geometric_intrusion_s'] = None
    # 真实碰撞只能人工填：本机没有碰撞传感器，急停按钮在操作者手里
    row['collisions_manual'] = raw.get('collisions_manual')

    row.update(_brake_latency(raw, cfg, frames, scan_t))
    row.update(scan_metrics.low_obstacle_detection(
        _seq(raw, 'low_obstacle_truth'),
        _seq(raw, 'low_obstacle_detections')))


def _brake_latency(raw, cfg, frames, scan_t):
    """动态障碍刹车响应延迟。见 PROXY_DEFINITIONS。"""
    if not frames or len(scan_t) < 3:
        return {'brake_latency_s_proxy': None, 'brake_events': 0,
                'brake_events_skipped': 0}
    t = np.asarray(scan_t, dtype=float)
    front = np.asarray([np.nan if f['front_min'] is None else f['front_min']
                        for f in frames], dtype=float)
    # 前向净空在 dyn_window_s 内跌落超过 dyn_drop_m → 障碍出现
    trig = np.zeros(t.size, dtype=bool)
    lo = np.searchsorted(t, t - float(cfg['dyn_window_s']), side='left')
    for i in range(t.size):
        j = lo[i]
        if j >= i or not np.isfinite(front[i]):
            continue
        prev = front[j:i]
        prev = prev[np.isfinite(prev)]
        if prev.size and (float(np.max(prev)) - front[i]) > float(cfg['dyn_drop_m']):
            trig[i] = True
    speed = signals.hypot_series(
        _seq(raw, 'cmd_final_vx'), _seq(raw, 'cmd_final_vy'))
    speed_at_scan = signals.resample_zoh(
        _seq(raw, 'cmd_final_t'), speed, t)
    lat, skipped = signals.response_latency(
        t, trig, speed_at_scan, drop_ratio=cfg['dyn_brake_ratio'])
    return {
        'brake_latency_s_proxy': _f(signals.median_or_none(lat)),
        'brake_events': len(lat),
        'brake_events_skipped': int(skipped),
    }


def compute_round(raw, cfg=None):
    """一轮的原始时序 → 一行指标。这是**唯一**的判据出口。

    raw 的必需键见 recorder 节点的 _flush_round()。缺键一律按"没有数据"处理，
    不抛异常 —— 一轮采集不全不该让整次扫描的数据全丢，
    但缺什么必须在输出里看得出来（各 *_samples / *_msgs 列）。
    """
    c = dict(DEFAULTS)
    if cfg:
        c.update(cfg)

    row = {
        'round_index': int(raw.get('index', -1)),
        'outcome': raw.get('outcome', 'UNKNOWN'),
        'success': bool(raw.get('outcome') == OUTCOME_SUCCESS),
        'start_stamp': _f(raw.get('goal_stamp')),
        'end_stamp': _f(raw.get('end_stamp')),
        'duration_s': _f((raw.get('end_stamp') or 0.0) - (raw.get('goal_stamp') or 0.0)),
        # 限速：**实测**值，来自在线查参，不是回填命令行（见 recorder 的自检）
        'vx_max_measured': _f(raw.get('vx_max_measured')),
        'vx_min_measured': _f(raw.get('vx_min_measured')),
        'smoother_max_velocity_measured': _f(
            raw.get('smoother_max_velocity_measured')),
        'speed_cap_requested': _f(raw.get('speed_cap_requested')),
        'goal_tolerance_m': _f(raw.get('goal_tolerance')),
        'pose_samples': int(len(_seq(raw, 'pose_x'))),
        'clock_source': raw.get('clock_source'),
        'use_sim_time': raw.get('use_sim_time'),
    }

    _arrival(raw, row)
    _pose_coverage(raw, row)

    drift_xy, drift_yaw = geometry.settle_drift(
        np.column_stack([_seq(raw, 'settle_x'), _seq(raw, 'settle_y')])
        if len(_seq(raw, 'settle_x')) else [],
        _seq(raw, 'settle_yaw'))
    row['settle_drift_xy_m'] = _f(drift_xy)
    row['settle_drift_yaw_rad'] = _f(drift_yaw)
    row['settle_samples'] = int(len(_seq(raw, 'settle_x')))

    track = np.column_stack([_seq(raw, 'pose_x'), _seq(raw, 'pose_y')]) \
        if len(_seq(raw, 'pose_x')) else np.zeros((0, 2))
    ov = geometry.overshoot(track, (raw['goal'][0], raw['goal'][1]),
                            raw.get('goal_tolerance') or 0.18,
                            approach_span_m=c['approach_span_m'])
    row['overshoot_radial_m'] = _f(ov['radial_m'])
    row['overshoot_along_track_m'] = _f(ov['along_track_m'])

    _path_metrics(raw, c, row)
    _tracking_metrics(raw, c, row)
    _scan_and_safety(raw, c, row)

    row.update(zero_progress_events(
        _seq(raw, 'pose_t'), _seq(raw, 'pose_x'), _seq(raw, 'pose_y'),
        _seq(raw, 'cmd_raw_t'),
        signals.hypot_series(_seq(raw, 'cmd_raw_vx'),
                             _seq(raw, 'cmd_raw_vy')),
        _seq(raw, 'cmd_final_t'),
        signals.hypot_series(_seq(raw, 'cmd_final_vx'),
                             _seq(raw, 'cmd_final_vy')),
        radius_m=c['progress_radius_m'], window_s=c['progress_window_s']))
    return row


_CSV_TRUE = ('true', 'True', 'TRUE')
_CSV_FALSE = ('false', 'False', 'FALSE')


def coerce_csv_row(row):
    """把 csv.DictReader 读出来的**全字符串**行还原成 compute_round 的类型。

    没有这一步，summarize() 会静默给出错的数：
      · `success` 从 CSV 读出来是 `'False'` —— 非空字符串，**布尔求值为真**，
        于是 `sum(1 for r in rows if r['success'])` 恒等于轮数，
        **success_rate 恒为 1.0**，一次失败也看不出来。
      · summarize() 用 `isinstance(v, (int, float))` 挑数值列，
        字符串全部落选 ⇒ 122 列一列都不参与统计，②节只剩三行。
    两个症状一个不报错。前者比后者危险得多：空表看得见，假的 1.0 看不见。

    空字符串 -> None（"没测到"），绝不填 0 —— 见文件头的命名约定。
    """
    out = {}
    for k, v in row.items():
        if not isinstance(v, str):
            out[k] = v
            continue
        s = v.strip()
        if s == '':
            out[k] = None
        elif s in _CSV_TRUE:
            out[k] = True
        elif s in _CSV_FALSE:
            out[k] = False
        else:
            try:
                f = float(s)
            except ValueError:
                out[k] = v                       # 真正的字符串列（outcome 等）
            else:
                out[k] = None if not math.isfinite(f) else f
    return out


def summarize(rows):
    """跨轮汇总。到位成功率在这里算。

    **不做**的事：把 None 当 0 参与平均。每一项都报参与统计的样本数 n，
    n 与轮数差很多时那个均值不能直接用来比档位。

    从 CSV 读来的行必须先过 coerce_csv_row()，否则结果是**静默错的**（见那里）。
    这里显式拦一道 —— 宁可让调用方报错，也不发一个看着正常的 success_rate。
    """
    out = {'rounds': len(rows)}
    if not rows:
        return out
    bad = [k for k in ('success', 'counter_mismatch')
           if isinstance(rows[0].get(k), str)]
    if bad:
        raise ValueError(
            'summarize() 收到的是字符串类型的行（%s 是 str）。'
            '这通常是直接把 csv.DictReader 的结果传进来了 —— '
            "请先过 coerce_csv_row()：'False' 作为非空字符串会被判为真，"
            'success_rate 会恒为 1.0 且不报任何错。' % ', '.join(bad))
    succ = sum(1 for r in rows if r.get('success'))
    out['succeeded'] = succ
    out['success_rate'] = signals.safe_div(succ, len(rows))

    numeric = [k for k, v in rows[0].items()
               if isinstance(v, (int, float)) and not isinstance(v, bool)]
    for k in numeric:
        vals = [r[k] for r in rows
                if isinstance(r.get(k), (int, float))
                and not isinstance(r.get(k), bool)
                and r.get(k) is not None and math.isfinite(float(r[k]))]
        out['%s__n' % k] = len(vals)
        out['%s__mean' % k] = _f(np.mean(vals)) if vals else None
        out['%s__sd' % k] = _f(np.std(vals, ddof=1)) if len(vals) > 1 else None
        out['%s__max' % k] = _f(np.max(vals)) if vals else None
    return out
