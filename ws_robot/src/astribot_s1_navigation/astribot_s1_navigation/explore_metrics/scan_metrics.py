# Copyright 2026 Astribot.
import math

import numpy as np

FRONT_HALF_ANGLE = math.radians(30.0)
SIDE_LO = math.radians(30.0)
SIDE_HI = math.radians(150.0)

DEFAULT_CORRIDOR_MAX_RANGE = 2.0

DEFAULT_NARROW_WIDTH_M = 1.62


def valid_mask(ranges, range_min, range_max):
    """有效回波掩码。见文件头 ②。

    0 被当作**无效**而不是 0m 障碍。若哪天上游改成真的会报 0m，
    这里会低估近距障碍 —— 但反过来（把无效当 0m）会让最近障碍距离恒为 0，
    那是完全不可用的。两害取轻，并且把这个选择写在这里。
    """
    r = np.asarray(ranges, dtype=float)
    return np.isfinite(r) & (r > 0.0) & (r >= float(range_min)) & (r <= float(range_max))


def beam_angles(n, angle_min, angle_increment):
    return float(angle_min) + np.arange(int(n), dtype=float) * float(angle_increment)


def frame_stats(ranges, angle_min, angle_increment, range_min, range_max,
                circumscribed=None, corridor_max_range=DEFAULT_CORRIDOR_MAX_RANGE):
    """单帧激光的派生量。

    返回 dict（拿不到的量一律 None，不填 0）：
      n_beams        总光束数
      valid_ratio    有效光束占比
      min_range      最近有效回波 (m)
      front_min      前向 ±30° 最近回波
      left_min       左 [30,150]° 最近回波
      right_min      右 [-150,-30]° 最近回波
      in_corridor    两侧都有 < corridor_max_range 的回波
      center_bias    (left-right)/(left+right)，仅 in_corridor 时给值
      free_width     left_min + right_min（墙到墙净宽；不加机器人直径，见下）
      self_residual  range < 外接半径 的光束数（见文件头 ④）
    """
    r = np.asarray(ranges, dtype=float)
    n = r.size
    out = {'n_beams': int(n), 'valid_ratio': None, 'min_range': None,
           'front_min': None, 'left_min': None, 'right_min': None,
           'in_corridor': False, 'center_bias': None, 'free_width': None,
           'self_residual': None}
    if n == 0:
        return out
    ok = valid_mask(r, range_min, range_max)
    out['valid_ratio'] = float(np.count_nonzero(ok)) / float(n)
    if not np.any(ok):
        return out
    ang = beam_angles(n, angle_min, angle_increment)
    rv, av = r[ok], ang[ok]
    out['min_range'] = float(np.min(rv))

    def sector_min(mask):
        return float(np.min(rv[mask])) if np.any(mask) else None

    out['front_min'] = sector_min(np.abs(av) <= FRONT_HALF_ANGLE)
    out['left_min'] = sector_min((av >= SIDE_LO) & (av <= SIDE_HI))
    out['right_min'] = sector_min((av <= -SIDE_LO) & (av >= -SIDE_HI))

    if circumscribed is not None:
        out['self_residual'] = int(np.count_nonzero(rv < float(circumscribed)))

    lm, rm = out['left_min'], out['right_min']
    if lm is not None and rm is not None:
        if lm <= corridor_max_range and rm <= corridor_max_range:
            out['in_corridor'] = True
            denom = lm + rm
            out['center_bias'] = float((lm - rm) / denom) if denom > 1e-6 else None
        out['free_width'] = float(lm + rm)
    return out


def clearance(min_range, circumscribed):
    """通行净空 = 最近回波 − 外接半径。

    可以是负数，负数就是"障碍已经落在外接包络里"。**不夹到 0** ——
    夹掉之后"净空 0.00"既可能是刚好贴上也可能是已经嵌进去 0.3m，
    而这两件事该做的处置完全不同。
    """
    if min_range is None or circumscribed is None:
        return None
    return float(min_range) - float(circumscribed)


def narrow_flags(free_widths, narrow_width_m=DEFAULT_NARROW_WIDTH_M):
    """逐帧的"处在窄通道里"标记。free_width 为 None 的帧记 False。"""
    return np.asarray(
        [(w is not None and float(w) < float(narrow_width_m)) for w in free_widths],
        dtype=bool)


def aggregate(frames, stamps, circumscribed=None,
              narrow_width_m=DEFAULT_NARROW_WIDTH_M):
    """把一轮的逐帧 frame_stats 汇总。

    返回 dict。所有"没有合格样本"的量给 None。
    corridor_sample_ratio 必须一起看：它很小时 center_bias 的统计没有意义
    （见文件头 ③）。
    """
    from . import signals

    if not frames:
        return {
            'scan_frames': 0, 'scan_hz': None, 'scan_gap_p95': None,
            'valid_ratio_mean': None, 'valid_ratio_min': None,
            'nearest_obstacle_m': None, 'min_clearance_m': None,
            'front_min_m': None,
            'center_bias_abs_mean': None, 'center_bias_abs_max': None,
            'corridor_sample_ratio': None, 'corridor_samples': 0,
            'self_filter_residual_ratio_proxy': None,
            'self_filter_residual_frames': 0,
            'self_filter_scored_frames': 0,
            'narrow_sample_ratio': None,
        }
    rate = signals.rate_from_stamps(stamps)
    vr = [f['valid_ratio'] for f in frames]
    mins = [f['min_range'] for f in frames]
    fronts = [f['front_min'] for f in frames]
    bias = [abs(f['center_bias']) for f in frames if f['center_bias'] is not None]
    corridor = sum(1 for f in frames if f['in_corridor'])

    scored = [f for f in frames if f['self_residual'] is not None]
    residual_beams = sum(f['self_residual'] for f in scored)
    scored_beams = sum(f['n_beams'] for f in scored)
    residual_frames = sum(1 for f in scored if f['self_residual'] > 0)

    nearest = signals.stat_or_none(
        [m for m in mins if m is not None], np.min)
    narrow = narrow_flags([f['free_width'] for f in frames], narrow_width_m)

    return {
        'scan_frames': len(frames),
        'scan_hz': rate['hz'],
        'scan_gap_p95': rate['gap_p95'],
        'valid_ratio_mean': signals.stat_or_none(
            [v for v in vr if v is not None], np.mean),
        'valid_ratio_min': signals.stat_or_none(
            [v for v in vr if v is not None], np.min),
        'nearest_obstacle_m': nearest,
        'min_clearance_m': clearance(nearest, circumscribed),
        'front_min_m': signals.stat_or_none(
            [f for f in fronts if f is not None], np.min),
        'center_bias_abs_mean': signals.stat_or_none(bias, np.mean),
        'center_bias_abs_max': signals.stat_or_none(bias, np.max),
        'corridor_sample_ratio': signals.safe_div(corridor, len(frames)),
        'corridor_samples': corridor,
        'self_filter_residual_ratio_proxy': signals.safe_div(
            residual_beams, scored_beams) if scored else None,
        'self_filter_residual_frames': residual_frames,
        'self_filter_scored_frames': len(scored),
        'narrow_sample_ratio': signals.safe_div(
            int(np.count_nonzero(narrow)), len(frames)),
    }


def narrow_traversals(stamps, frames, progress_m, narrow_width_m=DEFAULT_NARROW_WIDTH_M,
                      min_episode_s=0.5, min_progress_m=0.3, round_failed=False):
    """窄通道穿越（**代理量**，定义写在这里并原样写入输出）。

    定义：
      窄段 = 连续 >= min_episode_s 满足 (left_min + right_min + 2*外接半径)
             < narrow_width_m 的时间段
      成功 = 段末沿路径进展 − 段初沿路径进展 >= min_progress_m 且该轮未失败
      成功率 = 成功段数 / 窄段数

    为什么是代理量：真正的"穿越成功"需要知道那条通道通向哪里，
    以及机器人是否本来就该穿过去。这里只能用"在窄段里确实往前推进了"
    近似，并且靠 round_failed 把"轮次整体失败"的段一律判否。
    段数为 0 时返回 None 而不是 1.0 —— 没进过窄通道不等于窄通道 100% 通过。
    """
    from . import signals

    flags = narrow_flags([f['free_width'] for f in frames], narrow_width_m)
    eps = signals.episodes(stamps, flags, min_episode_s)
    if not eps:
        return {'narrow_episodes': 0, 'narrow_success_rate_proxy': None,
                'narrow_success': 0}
    t = np.asarray(stamps, dtype=float)
    prog = np.asarray(progress_m, dtype=float)
    ok = 0
    for t0, t1 in eps:
        if round_failed:
            continue
        i0 = int(np.searchsorted(t, t0, side='left'))
        i1 = int(np.searchsorted(t, t1, side='right')) - 1
        if i0 >= prog.size or i1 < 0 or i1 <= i0:
            continue
        p0, p1 = prog[min(i0, prog.size - 1)], prog[min(i1, prog.size - 1)]
        if np.isfinite(p0) and np.isfinite(p1) and (p1 - p0) >= float(min_progress_m):
            ok += 1
    return {
        'narrow_episodes': len(eps),
        'narrow_success': ok,
        'narrow_success_rate_proxy': signals.safe_div(ok, len(eps)),
    }


def low_obstacle_detection(truth, detections, tol_m=0.25):
    """低矮障碍检出率。**需要人工真值**，没有真值就返回 None。

    truth      [(x, y, height_m), ...]  map 系，人工标注
    detections [(x, y), ...]            map 系，由 /scan 回波反投影得到
    命中 = 存在一个 detection 落在 truth 点的 tol_m 内

    没有 truth 时返回 {'rate': None, 'reason': ...} —— 绝不用
    "回波数 / 光束数" 之类的东西冒充检出率：那量的是有没有回波，
    不是有没有检出**指定的**障碍。
    另外要记住 /scan 是 [0.05,1.63] 的 z 切片，低于 0.05m 的障碍
    按构造不可见，此时 rate=0 反映的是感知配置而不是算法（见文件头 ①）。
    """
    if not truth:
        return {'low_obstacle_detect_rate': None, 'low_obstacle_truth_n': 0,
                'low_obstacle_note': '未提供人工真值(--low-obstacle-truth)，本项不可计算'}
    det = np.asarray(detections, dtype=float).reshape(-1, 2) if len(detections) else \
        np.zeros((0, 2))
    hit = 0
    for tx, ty, _h in truth:
        if det.size and np.any(np.hypot(det[:, 0] - tx, det[:, 1] - ty) <= tol_m):
            hit += 1
    return {
        'low_obstacle_detect_rate': float(hit) / float(len(truth)),
        'low_obstacle_truth_n': len(truth),
        'low_obstacle_note': 'z 切片 [0.05,1.63]：低于 0.05m 的障碍按构造不可见',
    }
