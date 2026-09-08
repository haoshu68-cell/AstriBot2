# Copyright 2026 Astribot.
#
# 探索评测的时序信号计算。**纯函数，无 ROS 依赖**。
#
# ======================= 这里每个函数都在防一个具体的读数错误 =======================
#
# ① 拍率不能用均值
#   本项目把 157Hz 说成 91Hz 过一次，根因是拿"只在超阈时上报的样本"求均值。
#   同类错误还有"脉冲平均成速率"。所以 rate_from_stamps() 报的是
#   **中位间隔的倒数**，并且强制同时给出 p95 间隔 —— 尾部才是会让控制失稳的那部分，
#   而中位数看起来永远很好。
#
# ② 变号计数不能只看符号
#   ω 在 ±0.001rad/s 上下抖动会产生上千次"变号"。必须要求变号两侧的幅值
#   都真的超过阈值，且两次excursion 间隔不超过一个窗口 —— 否则"先左转 10 秒
#   再右转 10 秒"这种正常机动会被算成震荡。
#
# ③ 速度跟踪误差必须先对齐时延
#   本机链路是 位置积分式：cmd_vel 积成位置目标，SDK 伺服过去，实测滞后约 0.6s。
#   直接把 cmd 与 odom 逐点相减，算出来的"跟踪误差"主要成分是时延，
#   与跟踪好坏无关。所以 best_lag_and_residual() 先互相关求最佳时延，
#   同时报**零时延残差**和**最佳时延残差**，并在最佳时延命中搜索边界时置
#   hit_boundary=True —— 命中边界说明真实时延可能更大，那个"最佳"是假的。
#
# ④ 没有合格样本必须返回 None
#   返回 0.0 会在汇总表里变成一个漂亮的假数。冻结读数被当成当前值，
#   本项目一天内犯过三次。
# ================================================================================
import math

import numpy as np


def _arr(x):
    a = np.asarray(x, dtype=float)
    return a if a.ndim else a.reshape(1)


def rate_from_stamps(stamps):
    """从到达时刻序列算拍率。见文件头 ①。

    返回 dict：
      n           样本数
      hz          1 / 中位间隔（样本 < 2 时 None）
      gap_median  中位间隔 (s)
      gap_p95     p95 间隔 (s)   —— 尾部
      gap_max     最大间隔 (s)
      span_s      首末时间跨度
    """
    t = np.sort(_arr(stamps))
    if t.size < 2:
        return {'n': int(t.size), 'hz': None, 'gap_median': None,
                'gap_p95': None, 'gap_max': None,
                'span_s': 0.0 if t.size == 0 else 0.0}
    gaps = np.diff(t)
    gaps = gaps[gaps > 0.0]
    if gaps.size == 0:
        return {'n': int(t.size), 'hz': None, 'gap_median': None,
                'gap_p95': None, 'gap_max': None, 'span_s': float(t[-1] - t[0])}
    med = float(np.median(gaps))
    return {
        'n': int(t.size),
        'hz': (1.0 / med) if med > 0.0 else None,
        'gap_median': med,
        'gap_p95': float(np.percentile(gaps, 95.0)),
        'gap_max': float(np.max(gaps)),
        'span_s': float(t[-1] - t[0]),
    }


def oscillation_count(stamps, values, min_amp, window_s):
    """震荡次数：带幅值门限与时间窗的变号计数。见文件头 ②。

    判据：找出所有 |v| > min_amp 的样本，按符号压缩成"excursion 序列"
    （连续同号算一次），相邻两次 excursion 符号相反且起止间隔 <= window_s
    时记一次震荡。

    这样"左转 10s 后右转 10s"（window_s=2 时）不计入，
    而"0.3s 内左右各甩一下"计入 —— 后者才是控制在抖。
    """
    t = _arr(stamps)
    v = _arr(values)
    n = min(t.size, v.size)
    if n < 2:
        return 0
    t, v = t[:n], v[:n]
    big = np.abs(v) > float(min_amp)
    if not np.any(big):
        return 0
    sign = np.sign(v[big])
    ts = t[big]
    # 压缩成 excursion：记下每段同号区间的起止时刻
    bounds = np.flatnonzero(np.diff(sign) != 0)
    starts = np.concatenate(([0], bounds + 1))
    ends = np.concatenate((bounds, [sign.size - 1]))
    exc = [(float(sign[s]), float(ts[s]), float(ts[e]))
           for s, e in zip(starts, ends)]
    count = 0
    for i in range(1, len(exc)):
        prev_sign, _, prev_end = exc[i - 1]
        cur_sign, cur_start, _ = exc[i]
        if cur_sign != prev_sign and (cur_start - prev_end) <= float(window_s):
            count += 1
    return count


def resample_zoh(t_src, v_src, t_grid):
    """零阶保持重采样：t_grid 每点取最近的**不晚于**它的源样本。

    用零阶保持而不是线性插值：指令是阶梯信号，线性插值会造出从未下发过的
    中间值，再拿它去算"跟踪误差"就是在跟自己造的数比。

    t_grid 中早于第一个源样本的位置给 nan（**不是 0**：那段没有指令，
    补 0 会被算成"指令要求停车"）。
    """
    ts = _arr(t_src)
    vs = _arr(v_src)
    tg = _arr(t_grid)
    out = np.full(tg.shape, np.nan)
    if ts.size == 0 or vs.size == 0:
        return out
    order = np.argsort(ts)
    ts, vs = ts[order], vs[order]
    idx = np.searchsorted(ts, tg, side='right') - 1
    ok = idx >= 0
    out[ok] = vs[idx[ok]]
    return out


def best_lag_and_residual(t_cmd, v_cmd, t_act, v_act,
                          max_lag_s=1.5, grid_dt=0.02):
    """速度跟踪误差：先对齐时延再算残差。见文件头 ③。

    做法：把指令与实测都零阶保持到统一 grid_dt 网格，在 [0, max_lag_s]
    内平移指令，取**残差 RMS 最小**的时延（不是取相关系数最大：
    相关最大对幅值缩放不敏感，而本链路的臂-底盘耦合正是在缩幅值，
    用相关会把"被压到 15%"看成完美跟踪）。

    返回 dict：
      best_lag_s      残差最小的时延
      rms_at_best     该时延下的残差 RMS (m/s)
      rms_at_zero     零时延残差 RMS，用于对照"多少是时延贡献的"
      gain            最佳时延下 实测/指令 的最小二乘增益
                      —— 明显 < 1 说明被下游限速压过，不是跟踪差
      hit_boundary    best_lag 落在搜索区间端点（真实时延可能更大，
                      此时 best_lag 不可信）
      n               参与计算的网格点数
    """
    tc, vc = _arr(t_cmd), _arr(v_cmd)
    ta, va = _arr(t_act), _arr(v_act)
    if tc.size < 2 or ta.size < 2:
        return {'best_lag_s': None, 'rms_at_best': None, 'rms_at_zero': None,
                'gain': None, 'hit_boundary': False, 'n': 0}
    t0 = max(float(np.min(tc)), float(np.min(ta)))
    t1 = min(float(np.max(tc)), float(np.max(ta)))
    if t1 - t0 < max(3.0 * grid_dt, 0.1):
        return {'best_lag_s': None, 'rms_at_best': None, 'rms_at_zero': None,
                'gain': None, 'hit_boundary': False, 'n': 0}
    grid = np.arange(t0, t1, float(grid_dt))
    act = resample_zoh(ta, va, grid)

    lags = np.arange(0.0, float(max_lag_s) + 1e-9, float(grid_dt))
    best = None
    rms_zero = None
    for lag in lags:
        cmd = resample_zoh(tc, vc, grid - lag)
        ok = np.isfinite(cmd) & np.isfinite(act)
        if np.count_nonzero(ok) < 5:
            continue
        rms = float(np.sqrt(np.mean((act[ok] - cmd[ok]) ** 2)))
        if lag == 0.0:
            rms_zero = rms
        if best is None or rms < best[1]:
            denom = float(np.sum(cmd[ok] ** 2))
            gain = float(np.sum(cmd[ok] * act[ok]) / denom) if denom > 0 else None
            best = (float(lag), rms, gain, int(np.count_nonzero(ok)))
    if best is None:
        return {'best_lag_s': None, 'rms_at_best': None, 'rms_at_zero': None,
                'gain': None, 'hit_boundary': False, 'n': 0}
    lag, rms, gain, n = best
    return {
        'best_lag_s': lag,
        'rms_at_best': rms,
        'rms_at_zero': rms_zero,
        'gain': gain,
        'hit_boundary': bool(lag >= float(max_lag_s) - 1e-9 or lag <= 1e-9),
        'n': n,
    }


def episodes(stamps, flag, min_duration_s=0.0):
    """把布尔时序压成"段落"列表 [(t_start, t_end), ...]。

    段末时刻取**该段最后一个 True 样本**的时刻，不取下一个 False 的时刻：
    后者会把一个采样间隔算进段长，在 0.5s 的门限上足以改变段数。

    min_duration_s 过滤掉太短的段（单帧噪声）。注意 min_duration_s > 0 时
    "只有一个样本"的段长度是 0，必然被滤掉 —— 这是有意的。
    """
    t = _arr(stamps)
    f = np.asarray(flag, dtype=bool)
    n = min(t.size, f.size)
    if n == 0:
        return []
    t, f = t[:n], f[:n]
    out = []
    i = 0
    while i < n:
        if not f[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and f[j + 1]:
            j += 1
        if (t[j] - t[i]) >= float(min_duration_s):
            out.append((float(t[i]), float(t[j])))
        i = j + 1
    return out


def duration_where(stamps, flag):
    """布尔时序为 True 的总时长。

    每个 True 样本按"到下一个样本的间隔"计时，最后一个样本按中位间隔补齐 ——
    直接用 sum(diff) 会把最后一段整个丢掉，在只有一段的短轮次里误差很大。
    """
    t = _arr(stamps)
    f = np.asarray(flag, dtype=bool)
    n = min(t.size, f.size)
    if n == 0:
        return 0.0
    t, f = t[:n], f[:n]
    if n == 1:
        return 0.0
    gaps = np.diff(t)
    med = float(np.median(gaps)) if gaps.size else 0.0
    gaps = np.concatenate((gaps, [med]))
    return float(np.sum(gaps[f]))


def stat_or_none(values, reducer):
    """对可能全是 nan / 空的序列做统计，无有效样本时返回 None。见文件头 ④。"""
    a = _arr(values)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return None
    return float(reducer(a))


def weighted_mean(stamps, values):
    """按时间间隔加权的均值。样本率不均匀时才正确。

    等间隔采样下与普通均值相同；但一旦有掉帧（本机 TF 查询会失败），
    普通均值会给密集段更大权重。无有效样本返回 None。
    """
    t = _arr(stamps)
    v = _arr(values)
    n = min(t.size, v.size)
    if n < 2:
        return stat_or_none(v, np.mean)
    t, v = t[:n], v[:n]
    gaps = np.diff(t)
    med = float(np.median(gaps)) if gaps.size else 0.0
    w = np.concatenate((gaps, [med]))
    ok = np.isfinite(v) & (w > 0)
    if not np.any(ok):
        return None
    return float(np.sum(v[ok] * w[ok]) / np.sum(w[ok]))


def response_latency(stamps, trigger_flag, values, drop_ratio=0.8,
                     max_wait_s=3.0):
    """触发后"被控量开始下降"的延迟。用于动态障碍刹车响应延迟。

    对每个 trigger_flag 由 False 变 True 的时刻 t0：
      基准 = t0 时刻的 |values|
      找 t > t0 内第一个 |values| <= drop_ratio * 基准 的时刻 t1
      延迟 = t1 - t0
    基准本身已经 ~0（机器人本来就没动）时跳过该事件 ——
    不跳过会得到一堆 0 延迟，把中位数拉成"响应极快"的假结论。

    返回 (延迟列表, 因基准过小被跳过的事件数)。两个都要报：
    跳过数远大于延迟数时，这个指标本轮没有意义。
    """
    t = _arr(stamps)
    f = np.asarray(trigger_flag, dtype=bool)
    v = np.abs(_arr(values))
    n = min(t.size, f.size, v.size)
    if n < 2:
        return [], 0
    t, f, v = t[:n], f[:n], v[:n]
    rising = np.flatnonzero(f[1:] & ~f[:-1]) + 1
    lat, skipped = [], 0
    for i in rising:
        base = v[i]
        if not np.isfinite(base) or base < 0.02:
            skipped += 1
            continue
        limit = t[i] + float(max_wait_s)
        j = i + 1
        hit = None
        while j < n and t[j] <= limit:
            if np.isfinite(v[j]) and v[j] <= float(drop_ratio) * base:
                hit = float(t[j] - t[i])
                break
            j += 1
        if hit is not None:
            lat.append(hit)
    return lat, skipped


def median_or_none(values):
    return stat_or_none(values, np.median)


def safe_div(num, den):
    """比率：分母为 0 时返回 None 而不是 0.0。见文件头 ④。"""
    if den in (0, 0.0) or den is None or num is None:
        return None
    return float(num) / float(den)


def hypot_series(xs, ys):
    return np.hypot(_arr(xs), _arr(ys))


def deg(x):
    return None if x is None else math.degrees(float(x))
