# Copyright 2026 Astribot.
import math

import numpy as np

_CHUNK = 2048

DEFAULT_MIN_STEP_M = 0.002

DEFAULT_TANGENT_STEP_M = 0.20


def wrap_angle(a):
    """把角度折到 **[-pi, pi)**。

    边界约定写清楚：取模实现下 ±pi 一律落到 **-pi**（(pi+pi) mod 2pi = 0 -> -pi）。
    物理上 +pi 与 -pi 是同一个朝向，但报出来的符号会差一个 —— 汇总时对
    yaw 误差取绝对值即可，不要拿符号做判断。

    不用 math.atan2(sin, cos)：那对 numpy 数组不通用，而本模块两种输入都要吃。

    nan 输入（"该样本没有可比对象"）原样传出去，并且**不打 RuntimeWarning**：
    取模碰到 nan 会刷 "invalid value encountered in remainder"，
    一轮几万个样本能刷出满屏告警，把真的告警埋掉。
    """
    with np.errstate(invalid='ignore'):
        return (np.asarray(a, dtype=float) + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angle_scalar(a):
    """标量版，返回 float 而不是 0 维数组（写 CSV 时 0 维数组会变成 'array(0.1)'）。

    边界与 wrap_angle 一致：±pi -> -pi。
    """
    return float((float(a) + math.pi) % (2.0 * math.pi) - math.pi)


def yaw_from_quat(x, y, z, w):
    """四元数 → yaw。只取绕 z 的分量。"""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def as_xy(points):
    """把 (N,2) 或 (N,3) 的序列规整成 float64 的 (N,2)。

    空输入返回 (0,2) 而不是 (0,) —— 后者会让下游的 pts[:, 0] 抛
    IndexError 而不是安静地返回空结果，排查时要多绕一圈。
    """
    arr = np.asarray(points, dtype=float)
    if arr.size == 0:
        return np.zeros((0, 2), dtype=float)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    return np.ascontiguousarray(arr[:, :2])


def polyline_length(points):
    """折线总长。顶点数 < 2 时返回 0.0。

    这是"参考路径长度"，用于路径长度比的分母。**不加**位移门限：
    规划器给出的顶点是确定值，没有噪声，加门限只会低估。
    """
    pts = as_xy(points)
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(pts, axis=0).T)))


def traveled_length(points, min_step=DEFAULT_MIN_STEP_M):
    """实测轨迹里程。低于 min_step 的单步位移不累计（见文件头 ②）。

    返回 (里程, 被门限丢弃的步数)。第二个值必须一起报：
    丢弃比例畸高说明机器人基本没动，此时"路径长度比"本身就没有意义，
    不给出这个数就没法判断该不该信那个比值。
    """
    pts = as_xy(points)
    if len(pts) < 2:
        return 0.0, 0
    steps = np.hypot(*np.diff(pts, axis=0).T)
    keep = steps >= float(min_step)
    return float(np.sum(steps[keep])), int(np.count_nonzero(~keep))


def cross_track_distances(query, polyline):
    """每个查询点到折线的最短距离（点到线段，含端点）。

    返回 (dist, seg_index, t)：
      dist       (M,)  最短距离
      seg_index  (M,)  最近线段下标
      t          (M,)  该线段上的归一化投影位置，已裁到 [0,1]

    折线只有 1 个顶点时退化为点距；0 个顶点时返回全 nan（**不是 0**：
    "没有路径"和"偏差为 0"是完全不同的两件事，返回 0 会让汇总里出现
    一个看起来很漂亮的假均值）。
    """
    q = as_xy(query)
    poly = as_xy(polyline)
    m = len(q)
    if m == 0:
        return (np.zeros(0), np.zeros(0, dtype=int), np.zeros(0))
    if len(poly) == 0:
        nan = np.full(m, np.nan)
        return nan, np.full(m, -1, dtype=int), nan.copy()
    if len(poly) == 1:
        d = np.hypot(q[:, 0] - poly[0, 0], q[:, 1] - poly[0, 1])
        return d, np.zeros(m, dtype=int), np.zeros(m)

    a = poly[:-1]                       # (N,2) 线段起点
    seg = poly[1:] - a                  # (N,2) 线段向量
    seg_len2 = np.einsum('ij,ij->i', seg, seg)      # (N,)
    safe_len2 = np.where(seg_len2 > 0.0, seg_len2, 1.0)

    dist = np.empty(m)
    idx = np.empty(m, dtype=int)
    tt = np.empty(m)
    for lo in range(0, m, _CHUNK):
        hi = min(lo + _CHUNK, m)
        rel = q[lo:hi, None, :] - a[None, :, :]
        t = np.einsum('ijk,jk->ij', rel, seg) / safe_len2[None, :]
        np.clip(t, 0.0, 1.0, out=t)
        proj = rel - t[:, :, None] * seg[None, :, :]
        d = np.sqrt(np.einsum('ijk,ijk->ij', proj, proj))
        best = np.argmin(d, axis=1)
        rows = np.arange(hi - lo)
        dist[lo:hi] = d[rows, best]
        idx[lo:hi] = best
        tt[lo:hi] = t[rows, best]
    return dist, idx, tt


def polyline_headings(polyline):
    """每段线段的朝向 (N-1,)。顶点数 < 2 时返回空数组。"""
    poly = as_xy(polyline)
    if len(poly) < 2:
        return np.zeros(0)
    d = np.diff(poly, axis=0)
    return np.arctan2(d[:, 1], d[:, 0])


def tangent_at(polyline, seg_index):
    """给定最近线段下标，取该段切向。下标为 -1（无路径）时给 nan。"""
    head = polyline_headings(polyline)
    idx = np.asarray(seg_index, dtype=int)
    out = np.full(idx.shape, np.nan)
    if head.size == 0:
        return out
    ok = (idx >= 0) & (idx < head.size)
    out[ok] = head[idx[ok]]
    return out


def heading_tangent_errors(track, yaws, polyline,
                          tangent_step_m=DEFAULT_TANGENT_STEP_M):
    """每个位姿点的「机头朝向 − 路径局部切向」有符号误差 (rad)。

    这是"机头是否沿路径方向"的**直接**度量。不要用蟹行角
    ``atan2(vy, vx)`` 代替它：掐掉 vy 之后蟹行角按定义恒等于 0，
    量它等于量自己的输入，任何配置都会"通过"。

    切向**不取单段**。全局规划器点距约等于代价地图分辨率 (0.05m)，
    单段方向在 8 连通栅格上量化到 45° 的整数倍，直接拿来当基准会把
    量化噪声算成朝向误差。所以先按 tangent_step_m 抽稀折线，
    抽稀后的段长就是切向的基线长度（默认 0.20m = 4 个栅格）。

    返回 (err, dist)：
      err   (M,) 有符号误差，已折到 [-pi, pi)
      dist  (M,) 该点到抽稀后折线的最短距离 —— 偏离路径很远时那一点的
                 "切向"没有物理意义，汇总时要用它筛掉

    折线抽稀后不足 2 个顶点时 err/dist 全 nan（**不是 0**：
    "没有路径"与"完全对齐"是两件事）。
    """
    q = as_xy(track)
    y = np.asarray(yaws, dtype=float).reshape(-1)
    if len(q) != len(y):
        raise ValueError(
            'track 与 yaws 长度不一致: %d vs %d' % (len(q), len(y)))
    poly = decimate_by_step(polyline, tangent_step_m)
    if len(q) == 0:
        return np.zeros(0), np.zeros(0)
    if len(poly) < 2:
        nan = np.full(len(q), np.nan)
        return nan, nan.copy()
    dist, idx, _ = cross_track_distances(q, poly)
    tan = tangent_at(poly, idx)
    return wrap_angle(y - tan), dist


def smoothness(polyline, min_step=0.0):
    """折线平滑度。

    返回 dict：
      turn_per_m   单位弧长平均转角 (rad/m)  —— 越小越平滑
      max_turn_rad 单顶点最大转角 (rad)      —— 抓"打折"的尖点
      length_m     参与计算的弧长
      vertices     参与计算的顶点数

    min_step > 0 时先按最小步长抽稀（用于**实测轨迹**：不抽稀的话
    50Hz 采样的抖动会让每一步都是一个微小转角，turn_per_m 被噪声主导，
    算出来的"平滑度"其实是在量传感器噪声）。规划路径传 0。

    长度为 0（原地）时 turn_per_m 返回 None 而不是 inf/0：
    没走路就没有"每米转角"这回事，返回数会被汇总当成有效样本。
    """
    poly = as_xy(polyline)
    if min_step > 0.0 and len(poly) > 2:
        poly = decimate_by_step(poly, min_step)
    head = polyline_headings(poly)
    length = polyline_length(poly)
    if head.size < 2 or length <= 0.0:
        return {'turn_per_m': None, 'max_turn_rad': None,
                'length_m': length, 'vertices': int(len(poly))}
    turns = np.abs(wrap_angle(np.diff(head)))
    return {
        'turn_per_m': float(np.sum(turns) / length),
        'max_turn_rad': float(np.max(turns)),
        'length_m': length,
        'vertices': int(len(poly)),
    }


def decimate_by_step(points, min_step):
    """按最小步长抽稀：只保留与上一个保留点距离 >= min_step 的点。

    首点恒保留；末点在与前一保留点距离不足时**也保留**，
    否则轨迹尾巴会被剪掉，而"到位位置"正好在尾巴上。
    """
    pts = as_xy(points)
    if len(pts) < 2 or min_step <= 0.0:
        return pts
    keep = [0]
    last = pts[0]
    for i in range(1, len(pts)):
        if math.hypot(pts[i, 0] - last[0], pts[i, 1] - last[1]) >= min_step:
            keep.append(i)
            last = pts[i]
    if keep[-1] != len(pts) - 1:
        keep.append(len(pts) - 1)
    return pts[np.asarray(keep, dtype=int)]


def first_entry_index(track, goal_xy, tolerance):
    """轨迹首次进入目标容差圈的下标；从未进入返回 None。"""
    pts = as_xy(track)
    if len(pts) == 0:
        return None
    d = np.hypot(pts[:, 0] - goal_xy[0], pts[:, 1] - goal_xy[1])
    hit = np.flatnonzero(d <= float(tolerance))
    return int(hit[0]) if hit.size else None


def overshoot(track, goal_xy, tolerance, approach_span_m=0.5):
    """超调量。两个量都给，因为它们答的不是同一个问题。

    radial_m
      首次进入容差圈之后，到目标的最大距离。答"冲过去多远又回来"。
      从未进入容差圈 → None（不是 0：没到过就谈不上超调）。

    along_track_m
      沿"最后进入方向"超出目标的最大投影。答"沿着来的方向冲过头多少"。
      方向取首次进入前最后 approach_span_m 里程的位移方向 ——
      不用瞬时速度方向，那个在低速段噪声很大。
      算不出方向（进入前几乎没位移）→ None。

    两者的区别在全向底盘上很实在：机器人可以斜着冲过去，
    radial 大而 along_track 小意味着它是**横向**甩出去的，不是刹不住。
    """
    pts = as_xy(track)
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    k = first_entry_index(pts, (gx, gy), tolerance)
    if k is None:
        return {'radial_m': None, 'along_track_m': None, 'entry_index': None}

    after = pts[k:]
    radial = float(np.max(np.hypot(after[:, 0] - gx, after[:, 1] - gy)))

    along = None
    if k >= 1:
        seg = pts[:k + 1]
        steps = np.hypot(*np.diff(seg, axis=0).T)
        cum = np.cumsum(steps[::-1])[::-1]        # cum[i] = 从 i 走到末端的里程
        start = 0
        idxs = np.flatnonzero(cum <= float(approach_span_m))
        if idxs.size:
            start = int(idxs[0])
        vec = seg[-1] - seg[start]
        norm = math.hypot(vec[0], vec[1])
        if norm > 1e-6:
            u = vec / norm
            proj = (after - np.array([gx, gy])) @ u
            along = float(max(0.0, float(np.max(proj))))
    return {'radial_m': radial, 'along_track_m': along, 'entry_index': int(k)}


def settle_drift(track, yaws):
    """静置漂移量：窗内位姿相对**窗首**的最大位移与最大转角。

    相对窗首而不是取极差：极差会把"缓慢单向漂移"和"来回抖动"算成同一个数，
    而这两者的含义完全不同（前者是没抓住位置，后者是控制抖）。
    相对窗首能直接回答"松手之后跑了多远"。

    样本 < 2 → (None, None)：一个样本量不出漂移。
    """
    pts = as_xy(track)
    if len(pts) < 2:
        return None, None
    d = float(np.max(np.hypot(pts[:, 0] - pts[0, 0], pts[:, 1] - pts[0, 1])))
    yaw_max = None
    ys = np.asarray(yaws, dtype=float)
    if ys.size >= 2:
        yaw_max = float(np.max(np.abs(wrap_angle(ys - ys[0]))))
    return d, yaw_max


def symmetry_center(vertices, tol=1e-3):
    """中心对称多边形的对称中心；**不对称就返回 None**。

    为什么需要它：nav2 的 published_footprint 是**全局系绝对坐标**
    （frame_id=map，顶点直接落在机器人当前位置附近），不是车体系。
    拿它直接算内切/外接半径，量到的是"地图原点到多边形的距离" ——
    实测随机器人开远单调增长 6.12 -> 8.81m，而底盘内切只有 0.42m。
    下游因此全错：min_clearance = min_range - 9 恒为大负数、
    free_width 凭空多加 18m、self_residual 把几乎所有激光点算成自碰。

    为什么平移回中心就够、不需要 TF 也不需要 yaw：
    内切/外接半径都是**绕原点旋转不变**的量，只对平移敏感。
    所以只要把对称中心搬回原点，剩下的那个未知偏航角不影响任何一个半径。

    为什么用"对称中心"而不是"质心"：两者对中心对称多边形相同，但质心对
    任意多边形都算得出一个数 —— 那会在足迹一旦不再中心对称时静默给出
    偏移的半径。这里要的是"不满足前提就说不满足"，故显式校验对称性：
    每个顶点 v 都必须有 2c - v 也是顶点。本机两个足迹（八边形、正方形）
    都满足，nav2 的 footprint_padding 向外等量膨胀也保持中心对称。
    """
    pts = as_xy(vertices)
    if len(pts) < 3:
        return None
    c = pts.mean(axis=0)
    mirrored = 2.0 * c - pts
    for m in mirrored:
        if float(np.min(np.hypot(pts[:, 0] - m[0], pts[:, 1] - m[1]))) > tol:
            return None
    return float(c[0]), float(c[1])


def recenter_polygon(vertices, tol=1e-3):
    """把中心对称多边形平移到以对称中心为原点；不对称返回 None。

    返回 (顶点列表, (cx, cy))。调用方拿 (cx, cy) 可以判断原始多边形离
    原点多远 —— 车体系足迹应当≈0，明显非零就说明拿到的是全局系坐标。
    """
    c = symmetry_center(vertices, tol=tol)
    if c is None:
        return None, None
    pts = as_xy(vertices)
    return [(float(p[0] - c[0]), float(p[1] - c[1])) for p in pts], c


def polygon_radii(vertices):
    """多边形的内切/外接半径（相对原点）。

    内切半径用 nav2 的 calculateMinAndMaxDistances 同一定义：
    **顶点距离与边距离一起取最小**。只取顶点距离会高估内切半径，
    而 nav2 判"起点致命"用的正是内切半径 —— 高估会让净空算出正数
    却实际已经压在致命格上。

    顶点 < 3 时（含空）返回 (None, None)：拿不到足迹就必须显式说拿不到，
    不能退回一个"默认半径"，那会让净空这一列变成猜的。
    """
    pts = as_xy(vertices)
    if len(pts) < 3:
        return None, None
    circ = float(np.max(np.hypot(pts[:, 0], pts[:, 1])))
    best = circ
    n = len(pts)
    for i in range(n):
        a = pts[i]
        b = pts[(i + 1) % n]
        seg = b - a
        len2 = float(seg @ seg)
        if len2 <= 0.0:
            d = float(math.hypot(a[0], a[1]))
        else:
            t = min(1.0, max(0.0, float((-a) @ seg) / len2))
            foot = a + t * seg
            d = float(math.hypot(foot[0], foot[1]))
        best = min(best, d)
    return best, circ
