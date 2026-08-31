#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2026, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------

"""
File: 215-chassis_track_mppi.py
Brief: MPPI 局部路径跟踪律的**离线验证**（只算不动）。与 214（RPP）成对使用。

================================================================================
这个脚本不会让机器人动
================================================================================
只读底盘当前位姿，其余全在内部理想运动学积分器里跑完。
**全程一次 `set_joints_position` 都不发，也没有执行开关。**
什么时候可以真动由人决定，到时再单独加。

依赖：Python 标准库 + numpy。机器人上实测 numpy 1.24.0 已装，**不新增依赖**。

================================================================================
先说清楚这个 MPPI 与 nav2 那个的关系
================================================================================
本文件是**按同一套原理、同一组参数**重写的 MPPI，不是 nav2 MPPI 的移植：

  相同：采样 batch_size 条控制序列 -> 用运动模型滚动 time_steps 步 ->
        critics 打分 -> softmax 加权求新控制序列 -> 取第一步下发 ->
        序列左移一格作为下次的初值（warm start）。
  相同：motion_model 是 **Omni**（vx, vy, wz 三自由度独立），这是本机 X 构型全向底盘
        的正确模型；差速模型会丢掉横移能力。
  不同：critics 只实现了四个主要项（见 CRITIC_* 常量），nav2 那套有十来个，
        权重也不可能一致。**所以本脚本的绝对代价值与 nav2 不可比**，
        可比的是**行为**：会不会斜向平移、要不要先转向、能不能刹到零。

参数沿用 nav2_params_mppi.yaml::

    controller_frequency  20.0        model_dt      0.05      time_steps  56
    batch_size            2000        vx_max        1.0       vy_max      1.0
    wz_max                2.0         ax_max        2.5       ay_max      2.5
    az_max                3.2         motion_model  "Omni"

================================================================================
实测：aarch64 上跑得动满规模
================================================================================
机器人上向量化微基准（2000×56）实测 12.79 ms/次 -> 78 Hz 上限，
20 Hz（50 ms 预算）有约 4 倍余量。所以这里**按原始 batch_size=2000、time_steps=56**
写，不缩规模。但真 MPPI 带 critics 比微基准重，**所以脚本自己测求解耗时并报出来**，
不假定它一定够快 —— 看 `--report-timing` 的输出。

================================================================================
一个从配置里读出来的问题：箱式加速度约束在最弱方向上要得最多
================================================================================
`ax_max: 2.5` 与 `ay_max: 2.5` 是**各轴独立**的箱式约束，合起来允许沿 45° 的加速度
幅值达到 sqrt(2.5^2+2.5^2)=3.54 m/s^2。而这台 X 构型底盘沿 ±45° 走时**只有两个轮
在出力**（另两个转速恰为 0），可用驱动力只有轴向的 1/sqrt(2)=0.707 倍
（0°/90°: 530.3 N，45°: 375.0 N，取 wheel_effort_limit 15 N·m / wheel_radius 0.08）。
也就是需求/能力约 2 倍超发。

本脚本用 `--accel-constraint` 让这件事可切换：
  box（默认，复刻 nav2）  各轴独立限幅，斜向可达 3.54
  disk                   按合成模长限幅到 2.5，斜向不再超发
两种都跑一遍就能看出差别；真实影响需要在真机上测打滑门槛才能定论。

================================================================================
用法
================================================================================
    source /opt/ros/humble/setup.bash
    cd ~/Downloads/astribot_sdk_aarch64 && source ./env.sh
    python3 examples/215-chassis_track_mppi.py                      # 斜向 30cm
    python3 examples/215-chassis_track_mppi.py --report-timing       # 看求解耗时
    python3 examples/215-chassis_track_mppi.py --both-accel          # box vs disk
    python3 examples/215-chassis_track_mppi.py --no-robot --seed 0   # 可复现
"""

import argparse
import math
import sys
import time

try:
    import numpy as np
except ImportError:
    sys.stderr.write('需要 numpy（机器人上实测已装 1.24.0）。'
                     '若报错，先确认 source 了 env.sh。\n')
    raise

# ---- 逐字沿用 nav2_params_mppi.yaml ----
CONTROLLER_FREQUENCY_HZ = 20.0
MODEL_DT = 0.05
TIME_STEPS = 56
BATCH_SIZE = 2000
VX_MAX, VX_MIN = 1.0, -1.0
VY_MAX = 1.0
WZ_MAX = 2.0
AX_MAX, AY_MAX, AZ_MAX = 2.5, 2.5, 3.2
# softmax 温度。nav2 的值是 0.3，但那是按**它自己那十几个 critic 的代价尺度**定的。
# 本脚本只实现四个 critic、权重也不同，代价差是 O(100) 量级，直接用 0.3 会让
# exp(-300/0.3)=exp(-1000) 在数值上塌成 one-hot ——
# MPPI 退化成"从 batch 个随机样本里挑最好的一个"，那是噪声控制器，实测表现为原地打转。
# 所以这里用**相对温度**：beta = TEMPERATURE_REL * std(cost)，与代价尺度无关。
# ESS（有效样本数）会被打印出来，塌缩时一眼可见。
TEMPERATURE_REL = 0.5
GAMMA = 0.015              # 控制代价权重（nav2 的 gamma）
# 采样噪声标准差。nav2 的 vx_std/vy_std/wz_std，取其常用默认值。
STD_VX, STD_VY, STD_WZ = 0.2, 0.2, 0.4
# ---- 视野随剩余距离收缩（本脚本的参数，见 MPPI.effective_horizon）----
MIN_TIME_STEPS = 8            # 视野下限，太短会看不到减速需求
HORIZON_MARGIN_STEPS = 6      # 余量，保证终点落在视野内
HORIZON_V_REF = 0.3           # 估算所需步数用的参考速度 (m/s)

# ---- critics 权重。只实现四项主要的，见文件头说明 ----
CRITIC_PATH_ALIGN = 10.0    # 贴合路径方向
CRITIC_PATH_DIST = 30.0     # 离路径的横向距离
CRITIC_GOAL = 5.0           # 接近终点
CRITIC_CONTROL = 1.0        # 控制量本身的代价（省力 + 抑制抖动）

PATH_STEP = 0.02
IDX_X, IDX_Y, IDX_THETA = 0, 1, 2

# ---- 底盘几何，用于算 45° 各向异性（omni_effort_drive_node.py:109,123-125,156）----
WHEEL_COEFF_VX = (-0.7071, 0.7071, -0.7071, 0.7071)
WHEEL_COEFF_VY = (-0.7071, -0.7071, 0.7071, 0.7071)
WHEEL_RADIUS = 0.08
WHEEL_EFFORT_LIMIT_NM = 15.0


def wrap_angle(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def straight_path(x0, y0, x1, y1, step=PATH_STEP):
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-12:
        return np.array([[x0, y0]])
    n = max(1, int(math.ceil(d / step)))
    t = np.linspace(0.0, 1.0, n + 1)
    return np.stack([x0 + (x1 - x0) * t, y0 + (y1 - y0) * t], axis=1)


def max_force_along(heading_rad):
    """沿某方向的最大合力 (N)。用于量化 45° 各向异性。"""
    f = WHEEL_EFFORT_LIMIT_NM / WHEEL_RADIUS
    ux, uy = math.cos(heading_rad), math.sin(heading_rad)
    return sum(abs(WHEEL_COEFF_VX[i] * ux + WHEEL_COEFF_VY[i] * uy)
               for i in range(4)) * f


def cross_track_error(xy, path):
    """点到折线的最短距离（numpy 版，逐段求点到线段距离）。"""
    a = path[:-1]
    b = path[1:]
    ab = b - a
    seg2 = (ab ** 2).sum(axis=1)
    seg2 = np.where(seg2 < 1e-18, 1.0, seg2)
    t = (((xy - a) * ab).sum(axis=1) / seg2).clip(0.0, 1.0)
    proj = a + t[:, None] * ab
    return float(np.min(np.hypot(*(xy - proj).T)))


class MPPI:
    """Omni 运动模型的 MPPI。控制序列在**本体系**里表达 (vx, vy, wz)。"""

    def __init__(self, batch=BATCH_SIZE, horizon=TIME_STEPS, dt=MODEL_DT,
                 accel_constraint='box', rng=None, shrink_horizon=True,
                 v_ref=0.3):
        self.batch = batch
        self.horizon = horizon
        self.dt = dt
        self.accel_constraint = accel_constraint
        self.rng = rng if rng is not None else np.random.default_rng()
        self.shrink_horizon = shrink_horizon
        self.v_ref = v_ref
        # warm start：控制序列跨周期保留，这是 MPPI 收敛的关键。
        # nav2 也这么做（优化器不是每周期从零开始）。
        self.u = np.zeros((horizon, 3))
        self.last_solve_sec = 0.0
        self.last_horizon = horizon
        self.last_ess = float(batch)

    def effective_horizon(self, remaining):
        """按剩余距离裁剪滚动步数。

        !!! 这是本脚本必须显式修的第二个结构性问题 !!!
        nav2 的 time_steps=56、model_dt=0.05 -> 视野 2.80 s；按 vx_max=1.0
        能滚出 2.8 m，而 30 cm 级任务全长只有 0.30~0.42 m ——
        **视野是任务全长的 6.6 倍**。于是滚动轨迹绝大部分落在终点之外，
        那里 path_dist / goal 两个 critic 都在往回拽，加权最优解退化成
        "几乎不动"（实测：600 步后距离从 0.4243 只变到 0.4285）。

        这与 214 里 RPP 的"前视距离比路径还长"是**同一类问题的两种形态**：
        预测/前瞻的尺度必须与任务尺度匹配，照抄长距离导航的参数会失效。

        修法：T_eff = clamp(ceil(remaining / (v_ref*dt)) + 余量, T_MIN, horizon)
        """
        if not self.shrink_horizon:
            return self.horizon
        need = int(math.ceil(remaining / max(1e-6, self.v_ref * self.dt)))
        return max(MIN_TIME_STEPS, min(self.horizon, need + HORIZON_MARGIN_STEPS))

    def _apply_accel_limits(self, u):
        """把控制序列的逐步变化限制在加速度约束内。

        box：各轴独立（复刻 nav2 的 ax_max/ay_max/az_max）
        disk：xy 按合成模长限幅（斜向不再超发，见文件头）
        """
        out = u.copy()
        prev = np.zeros(3)
        for k in range(out.shape[0]):
            d = out[k] - prev
            if self.accel_constraint == 'box':
                lim = np.array([AX_MAX, AY_MAX, AZ_MAX]) * self.dt
                d = np.clip(d, -lim, lim)
            else:
                n = math.hypot(d[0], d[1])
                cap = AX_MAX * self.dt
                if n > cap:
                    d[0] *= cap / n
                    d[1] *= cap / n
                d[2] = float(np.clip(d[2], -AZ_MAX * self.dt, AZ_MAX * self.dt))
            out[k] = prev + d
            prev = out[k]
        return out

    def _rollout(self, u_samples, state):
        """滚动 batch 条控制序列。返回每步的世界系位姿 (B, T, 3)。

        Omni 模型：本体系 (vx, vy) 按当前 yaw 旋到世界系再积分。
        """
        b, t, _ = u_samples.shape
        x = np.full(b, state[0])
        y = np.full(b, state[1])
        yaw = np.full(b, state[2])
        traj = np.empty((b, t, 3))
        for k in range(t):
            vx = u_samples[:, k, 0]
            vy = u_samples[:, k, 1]
            wz = u_samples[:, k, 2]
            c, s = np.cos(yaw), np.sin(yaw)
            x = x + (c * vx - s * vy) * self.dt
            y = y + (s * vx + c * vy) * self.dt
            yaw = yaw + wz * self.dt
            traj[:, k, 0] = x
            traj[:, k, 1] = y
            traj[:, k, 2] = yaw
        return traj

    def _cost(self, traj, u_samples, path, goal):
        """critics 打分。只实现四项主要的（见文件头）。"""
        b, t, _ = traj.shape
        pts = traj[:, :, :2].reshape(-1, 2)

        # path_dist：每个滚动点到路径的最短距离。向量化成 (N, seg) 的距离矩阵。
        a = path[:-1]
        seg = path[1:] - a
        seg2 = (seg ** 2).sum(axis=1)
        seg2 = np.where(seg2 < 1e-18, 1.0, seg2)
        rel = pts[:, None, :] - a[None, :, :]
        tt = ((rel * seg[None, :, :]).sum(axis=2) / seg2[None, :]).clip(0.0, 1.0)
        proj = a[None, :, :] + tt[:, :, None] * seg[None, :, :]
        d = np.sqrt(((pts[:, None, :] - proj) ** 2).sum(axis=2))
        path_dist = d.min(axis=1).reshape(b, t)

        # goal：终点距离，只算最后一步（其余步由 path_dist 约束）
        goal_dist = np.hypot(traj[:, -1, 0] - goal[0], traj[:, -1, 1] - goal[1])

        # path_align：速度方向与"指向路径最近点之后一点"的方向是否一致。
        # 用相邻滚动点的位移方向近似速度方向。
        dxy = np.diff(traj[:, :, :2], axis=1)
        step_len = np.hypot(dxy[:, :, 0], dxy[:, :, 1]) + 1e-9
        # 目标方向：从当前点指向终点（短路径上这与路径切向基本一致）
        gx = goal[0] - traj[:, :-1, 0]
        gy = goal[1] - traj[:, :-1, 1]
        gnorm = np.hypot(gx, gy) + 1e-9
        cosang = (dxy[:, :, 0] * gx + dxy[:, :, 1] * gy) / (step_len * gnorm)
        align = 1.0 - cosang            # 0 最好

        # control：控制量本身的代价，抑制无谓的大速度与自转
        ctrl = (u_samples ** 2).sum(axis=(1, 2))

        return (CRITIC_PATH_DIST * path_dist.sum(axis=1)
                + CRITIC_PATH_ALIGN * align.sum(axis=1)
                + CRITIC_GOAL * goal_dist * t
                + CRITIC_CONTROL * GAMMA * ctrl)

    def solve(self, state, path, goal):
        """一个优化周期。返回本体系 (vx, vy, wz)。"""
        t0 = time.perf_counter()
        remaining = math.hypot(goal[0] - state[0], goal[1] - state[1])
        t_eff = self.effective_horizon(remaining)
        self.last_horizon = t_eff
        u_base = self.u[:t_eff]
        noise = self.rng.normal(
            0.0, [STD_VX, STD_VY, STD_WZ], size=(self.batch, t_eff, 3))
        u_samples = u_base[None, :, :] + noise
        # 速度上限
        u_samples[:, :, 0] = u_samples[:, :, 0].clip(VX_MIN, VX_MAX)
        u_samples[:, :, 1] = u_samples[:, :, 1].clip(-VY_MAX, VY_MAX)
        u_samples[:, :, 2] = u_samples[:, :, 2].clip(-WZ_MAX, WZ_MAX)

        traj = self._rollout(u_samples, state)
        cost = self._cost(traj, u_samples, path, goal)

        # softmax 加权。温度取 cost 标准差的固定比例，避免尺度不匹配导致塌缩。
        c = cost - cost.min()
        beta = TEMPERATURE_REL * float(cost.std())
        if not np.isfinite(beta) or beta <= 1e-12:
            beta = 1e-12
        w = np.exp(-c / beta)
        wsum = w.sum()
        if not np.isfinite(wsum) or wsum <= 0.0:
            # 全部权重退化：保持上周期序列，不要把 NaN 传下去
            self.last_ess = 1.0
            self.last_solve_sec = time.perf_counter() - t0
            return tuple(self.u[0])
        w /= wsum
        # ESS = 1/sum(w^2)：接近 1 说明塌成了单样本（退化成随机挑一个），
        # 接近 batch 说明完全没有选择性。健康区间大致是 batch 的 1%~30%。
        self.last_ess = float(1.0 / (w ** 2).sum())
        u_new = (w[:, None, None] * u_samples).sum(axis=0)
        u_new = self._apply_accel_limits(u_new)
        self.u[:t_eff] = u_new

        out = tuple(self.u[0])
        # warm start：左移一格，末尾补最后一个值
        self.u = np.roll(self.u, -1, axis=0)
        self.u[-1] = self.u[-2]
        self.last_solve_sec = time.perf_counter() - t0
        return out


def simulate(start, dx_body, dy_body, accel_constraint='box', stop_tol=0.03,
             batch=BATCH_SIZE, horizon=TIME_STEPS, seed=None, max_steps=600,
             verbose_every=0, shrink_horizon=True, v_ref=HORIZON_V_REF):
    """理想运动学闭环。**不含动力学**，测的是控制律本身。"""
    dt_ctrl = 1.0 / CONTROLLER_FREQUENCY_HZ
    x, y, yaw = start
    c, s = math.cos(yaw), math.sin(yaw)
    gx = x + c * dx_body - s * dy_body
    gy = y + s * dx_body + c * dy_body
    path = straight_path(x, y, gx, gy)
    total = math.hypot(gx - x, gy - y)
    goal = (gx, gy)

    rng = np.random.default_rng(seed)
    mppi = MPPI(batch, horizon, MODEL_DT, accel_constraint, rng,
                shrink_horizon=shrink_horizon, v_ref=v_ref)
    horizons = []
    ess = []

    xte_max = 0.0
    overshoot = 0.0
    rotate_first = 0.0
    moved = False
    peak_vy = 0.0
    peak_wz = 0.0
    solve_times = []
    speed_at_arrival = None

    for step in range(max_steps):
        d = math.hypot(gx - x, gy - y)
        if d <= stop_tol:
            return {'converged': True, 'steps': step, 'final_dist': d,
                    'xte_max': xte_max, 'overshoot': overshoot, 'final_yaw': yaw,
                    'rotate_first': rotate_first, 'peak_vy': peak_vy,
                    'peak_wz': peak_wz, 'solve_times': solve_times,
                    'speed_at_arrival': speed_at_arrival, 'total': total,
                    'goal': goal, 'end_pose': (x, y, yaw),
                    'horizons': horizons, 'ess': ess}

        vx, vy, wz = mppi.solve((x, y, yaw), path, goal)
        solve_times.append(mppi.last_solve_sec)
        horizons.append(mppi.last_horizon)
        ess.append(mppi.last_ess)
        speed_at_arrival = math.hypot(vx, vy)
        peak_vy = max(peak_vy, abs(vy))
        peak_wz = max(peak_wz, abs(wz))

        cy, sy = math.cos(yaw), math.sin(yaw)
        x += (cy * vx - sy * vy) * dt_ctrl
        y += (sy * vx + cy * vy) * dt_ctrl
        yaw = wrap_angle(yaw + wz * dt_ctrl)

        xte_max = max(xte_max, cross_track_error(np.array([x, y]), path))
        overshoot = max(overshoot, math.hypot(x - start[0], y - start[1]) - total)
        if not moved:
            rotate_first = max(rotate_first, abs(wrap_angle(yaw - start[2])))
            if math.hypot(x - start[0], y - start[1]) > 0.02:
                moved = True

        if verbose_every and step % verbose_every == 0:
            print(f'    step {step:3d} d={d:.4f} v=({vx:+.3f},{vy:+.3f}) '
                  f'wz={wz:+.3f} T={mppi.last_horizon:2d} '
                  f'solve={mppi.last_solve_sec * 1000:.1f} ms')

    return {'converged': False, 'steps': max_steps,
            'final_dist': math.hypot(gx - x, gy - y), 'xte_max': xte_max,
            'overshoot': overshoot, 'final_yaw': yaw,
            'rotate_first': rotate_first, 'peak_vy': peak_vy, 'peak_wz': peak_wz,
            'solve_times': solve_times, 'speed_at_arrival': speed_at_arrival,
            'total': total, 'goal': goal, 'end_pose': (x, y, yaw),
            'horizons': horizons, 'ess': ess}


def read_robot_pose():
    """只读当前底盘位姿。**不发任何指令。**"""
    try:
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
    except Exception as exc:                                      # noqa: BLE001
        print(f'  SDK import 失败（{type(exc).__name__}），改用原点起算。'
              f'先 source /opt/ros/humble/setup.bash 再 source env.sh')
        return None
    try:
        bot = Astribot()
        q = bot.get_current_joints_position([bot.chassis_name])[0]
        if len(q) < 3:
            print(f'  底盘自由度 {len(q)} < 3（ROBOT_TYPE 未设为 S1？），改用原点起算')
            return None
        return (q[0], q[1], q[2])
    except Exception as exc:                                      # noqa: BLE001
        print(f'  读位姿失败（{type(exc).__name__}: {exc}），改用原点起算')
        return None


def report(tag, r, stop_tol, show_timing):
    print(f'\n--- {tag} ---')
    if not r['converged']:
        print(f'  !! 未收敛（{r["steps"]} 步用尽），终点还差 {r["final_dist"]:.4f} m')
    print(f'  收敛步数        {r["steps"]}  '
          f'({r["steps"] / CONTROLLER_FREQUENCY_HZ:.2f} s @ {CONTROLLER_FREQUENCY_HZ:g} Hz)')
    print(f'  终点误差        {r["final_dist"]:.4f} m   (到位判据 {stop_tol:g})')
    print(f'  最大横向误差    {r["xte_max"]:.4f} m')
    print(f'  过冲            {r["overshoot"]:+.4f} m')
    print(f'  起步原地转角    {r["rotate_first"]:.4f} rad '
          f'({math.degrees(r["rotate_first"]):.1f}°)   <- Omni 应接近 0')
    print(f'  |vy| 峰值       {r["peak_vy"]:.4f} m/s   <- 非零才说明真在用全向能力')
    print(f'  |wz| 峰值       {r["peak_wz"]:.4f} rad/s')
    print(f'  终态航向变化    {math.degrees(wrap_angle(r["final_yaw"])):+.1f}°')
    if r['speed_at_arrival'] is not None:
        print(f'  到位那刻的速度  {r["speed_at_arrival"]:.4f} m/s'
              f'   <- MPPI 无接近速度下限，可以收到 0')
    if r.get('ess'):
        e = r['ess']
        em = sum(e) / len(e)
        print(f'  softmax ESS     均值 {em:.1f} / batch  '
              f'(最小 {min(e):.1f})   <- 接近 1 = 塌成单样本，退化成随机挑一个')
    if r.get('horizons'):
        hs = r['horizons']
        print(f'  实际滚动步数    首步 {hs[0]} / 中位 {sorted(hs)[len(hs)//2]} / '
              f'末步 {hs[-1]}   (标称 {TIME_STEPS})')
    if show_timing and r['solve_times']:
        ts = sorted(r['solve_times'])
        mean = sum(ts) / len(ts)
        p95 = ts[min(len(ts) - 1, int(0.95 * (len(ts) - 1)))]
        budget = 1.0 / CONTROLLER_FREQUENCY_HZ
        print(f'  求解耗时        均值 {mean * 1000:.2f} ms  p95 {p95 * 1000:.2f} ms  '
              f'最差 {ts[-1] * 1000:.2f} ms')
        print(f'                  20 Hz 预算 {budget * 1000:.0f} ms -> '
              f'占用 {p95 / budget * 100:.1f}% (p95)'
              + ('  超预算!' if p95 > budget else '  够用'))


def main():
    ap = argparse.ArgumentParser(
        description='MPPI 跟踪律离线验证（只算不动，不发任何指令）')
    ap.add_argument('--dx', type=float, default=0.30, help='本体系 x 位移 (m)')
    ap.add_argument('--dy', type=float, default=0.30, help='本体系 y 位移 (m)')
    ap.add_argument('--stop-tol', type=float, default=0.03, help='到位判据 (m)')
    ap.add_argument('--batch', type=int, default=BATCH_SIZE)
    ap.add_argument('--horizon', type=int, default=TIME_STEPS)
    ap.add_argument('--accel-constraint', choices=('box', 'disk'), default='box',
                    help='box=复刻 nav2 的各轴独立限幅  disk=按合成模长限幅')
    ap.add_argument('--both-accel', action='store_true', help='box 与 disk 都跑')
    ap.add_argument('--report-timing', action='store_true', help='报求解耗时')
    ap.add_argument('--seed', type=int, default=None, help='固定随机种子以复现')
    ap.add_argument('--no-robot', action='store_true', help='不连机器人，从原点起算')
    ap.add_argument('--no-shrink-horizon', action='store_true',
                    help='用标称视野 56 步（展示"视野比任务长"的退化行为）')
    ap.add_argument('--both-horizon', action='store_true',
                    help='收缩视野 / 不收缩 两种都跑，直接对比')
    ap.add_argument('--trace-every', type=int, default=0)
    args = ap.parse_args()

    print('=' * 78)
    print('MPPI 跟踪律离线验证 —— 只算不动，全程不发 set_joints_position')
    print('=' * 78)

    start = None if args.no_robot else read_robot_pose()
    if start is None:
        start = (0.0, 0.0, 0.0)
        print(f'起始位姿：原点 {start}（未连机器人或读取失败）')
    else:
        print(f'起始位姿：机器人实测 x={start[0]:+.4f} y={start[1]:+.4f} '
              f'theta={start[2]:+.4f}')

    leg = math.hypot(args.dx, args.dy)
    heading = math.atan2(args.dy, args.dx)
    print(f'\n任务：本体系位移 ({args.dx:+.3f}, {args.dy:+.3f}) m，'
          f'合成 {leg:.4f} m，方向 {math.degrees(heading):+.1f}°')
    print(f'MPPI：batch={args.batch} horizon={args.horizon} model_dt={MODEL_DT:g} '
          f'(视野 {args.horizon * MODEL_DT:.2f} s)  motion_model=Omni')
    print(f'加速度约束：{args.accel_constraint}'
          + ('（两种都跑）' if args.both_accel else ''))

    f0 = max_force_along(0.0)
    fh = max_force_along(heading)
    box_mag = math.hypot(AX_MAX, AY_MAX) if abs(
        abs(math.degrees(heading)) - 45.0) < 1e-6 else AX_MAX
    print(f'\n沿该方向的可用驱动力 {fh:.1f} N，轴向 {f0:.1f} N '
          f'-> 各向异性 {fh / f0:.3f}')
    if abs(abs(math.degrees(heading)) - 45.0) < 1e-6:
        print(f'  45° 上 box 约束允许幅值 {box_mag:.2f} m/s^2，'
              f'而能力只有轴向的 {fh / f0:.3f} 倍 -> 需求/能力 ≈ '
              f'{box_mag / (AX_MAX * fh / f0):.2f}')
        print('  这是配置层面的超发，真实影响要靠真机打滑门槛测才能定论。')

    accels = (('box', 'box 约束（复刻 nav2）'), ('disk', 'disk 约束（合成模长限幅）')) \
        if args.both_accel else ((args.accel_constraint, f'{args.accel_constraint} 约束'),)
    horizons = ((True, '收缩视野'), (False, f'标称视野 {TIME_STEPS} 步（不收缩）')) \
        if args.both_horizon else ((not args.no_shrink_horizon,
                                    '收缩视野' if not args.no_shrink_horizon
                                    else f'标称视野 {TIME_STEPS} 步（不收缩）'),)
    for sh, htag in horizons:
        for ac, atag in accels:
            r = simulate(start, args.dx, args.dy, ac, args.stop_tol,
                         args.batch, args.horizon, args.seed,
                         verbose_every=args.trace_every, shrink_horizon=sh)
            report(f'{htag} · {atag}', r, args.stop_tol, args.report_timing)

    print('\n' + '=' * 78)
    print('这是理想运动学结果：无质量、无摩擦、无打滑。')
    print('critics 只实现了四项，绝对代价值与 nav2 不可比；可比的是行为。')
    print('本脚本没有执行路径 —— 要让底盘真动，由人决定后再单独加。')
    print('=' * 78)
    return 0


if __name__ == '__main__':
    sys.exit(main())
