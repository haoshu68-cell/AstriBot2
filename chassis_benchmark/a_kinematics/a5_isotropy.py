#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A5 · 45° 各向同性。**A 组最该先做的一项。**

为什么排在 A 组第一
================
全向底盘的运动学矩阵写错（历史四大根因之一）几乎只在斜向运动上露出来：
沿 ±x/±y 走时四个轮的角色对称，系数就算错了也常常自相抵消；
而 X 构型下沿 ±45° 走时**只有两个轮在出力**（另两个转速恰为 0），
矩阵一旦错，这里立刻表现为方向偏、距离短、或两者都有。

矩阵给出的三个可检验数（bench/kinematics.py 里有单测钉住）：

* 沿 45° 走 1 m/s：轮 LF、RR 转速恰为 0，另两个 12.50 rad/s
* 单轮转速需求比 0° 高 sqrt(2) ≈ 41%
* 可用驱动力只有 0° 的 1/sqrt(2)：530.3 N -> 375.0 N

**本测试刻意用低速（默认 0.3 m/s）** ：目的是测运动学，不是测动力学。
若低速下 45° 就明显更差，说明力/速度余量在远离边界处就已经不够，
那是 PID/摩擦补偿的问题，不是"斜向本来就弱"。
动力学边界由 b3_slip_threshold.py 单独测。

判据
====
* 各方向到位距离**极差 <= 3%**
* 垂直于指令方向的偏移 <= 0.03 m（1 m 行程上等于方向偏差 <= 1.7°）
* 每个方向内部的 σ <= 0.01 m

用法::

    # 仿真：只起 warehouse_sim.launch.py，不要起 nav2
    python3 a_kinematics/a5_isotropy.py --env sim --n 10
    # 只测两个方向、走 0.5 m：
    python3 a_kinematics/a5_isotropy.py --env sim --n 5 --headings 0,45 --distance 0.5
"""

import math
import os
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bench import kinematics as kin                          # noqa: E402
from bench import stats as st                                # noqa: E402
from bench.driver import SimCmdVelDriver, brake_in_time_profile   # noqa: E402
from bench.session import (Session, base_arg_parser, results_dir,   # noqa: E402
                           run_main)
from bench.truth import SimOdomTruth, displacement           # noqa: E402

TEST_ID = 'a5_isotropy'

DEFAULT_HEADINGS_DEG = (0.0, 22.5, 45.0, 67.5, 90.0)
DEFAULT_DISTANCE_M = 1.0
DEFAULT_CRUISE_MPS = 0.3        # 刻意远离能力边界
DEFAULT_ACCEL = 1.0             # 也刻意低于 nav2 的 2.5，把动力学影响压小

RANGE_RATIO_THRESHOLD = 0.03    # 各方向距离极差 <= 3%
LATERAL_THRESHOLD_M = 0.03      # 垂直偏移 <= 0.03 m
SIGMA_THRESHOLD_M = 0.01        # 单方向重复性 σ <= 0.01 m

SETTLE_SEC = 2.0                # 每趟结束后的静止等待，让惯性尾巴走完


def _run_one_sim(node, driver, truth, heading_rad, distance, cruise, accel):
    """跑一趟并返回实测量。返回 (沿指令方向位移, 垂直偏移, 偏航变化, 指令层位移)。"""
    ux, uy = math.cos(heading_rad), math.sin(heading_rad)
    dt = driver.dt

    # 起点位姿。等一小会儿确保拿到的是静止后的读数
    _spin(node, 0.5)
    p0 = truth.pose()

    traveled = 0.0
    v = 0.0
    t0 = time.monotonic()
    while True:
        remaining = distance - traveled
        v = brake_in_time_profile(remaining, cruise, accel, v, dt)
        traveled += v * dt
        driver.set_velocity(vx=v * ux, vy=v * uy, wz=0.0)
        _spin(node, dt)
        if abs(remaining) < 1e-4 and abs(v) < 1e-4:
            break
        if time.monotonic() - t0 > 120.0:
            raise TimeoutError(f'单趟超时，已走指令量 {traveled:.4f}/{distance:.4f}')
    driver.halt()
    _spin(node, SETTLE_SEC)

    p1 = truth.pose()
    # displacement 给的是"在 p0 车体系里"的 (前向, 侧向, 转角)。
    # 本测试要求机器人不旋转，所以 p0 车体系 == 起始世界朝向；
    # 再把它投到指令方向上，得到"沿指令方向"和"垂直指令方向"两个分量。
    fwd, lat, dyaw = displacement(p0, p1)
    along = fwd * ux + lat * uy
    perp = -fwd * uy + lat * ux
    return along, perp, dyaw, traveled


def _spin(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=min(0.005, max(0.0005, seconds / 4)))


def main(argv=None):
    p = base_arg_parser(TEST_ID, 'A5 45° 各向同性（A 组第一项）')
    p.add_argument('--headings', default=','.join(f'{h:g}' for h in DEFAULT_HEADINGS_DEG),
                   help='逗号分隔的方向角（度）')
    p.add_argument('--distance', type=float, default=DEFAULT_DISTANCE_M, help='每趟距离 (m)')
    p.add_argument('--cruise', type=float, default=DEFAULT_CRUISE_MPS, help='巡航速度 (m/s)')
    p.add_argument('--accel', type=float, default=DEFAULT_ACCEL, help='加速度 (m/s^2)')
    args = p.parse_args(argv)

    headings_deg = [float(x) for x in args.headings.split(',') if x.strip()]
    if not headings_deg:
        print('至少要给一个方向')
        return 2

    if args.env == 'real':
        print('真机版尚未实现：真机必须用 ManualTruth 逐次录入卷尺读数，\n'
              '而且需要每趟把机器人推回同一个地面贴标。等 sim 跑通、判据确认之后再加。\n'
              '（不做成"真机也自动跑"是有意的：真机上没有独立真值源，\n'
              '  自动跑只能读 SDK 的航迹推算，那正是本方案禁止的自我验证。）')
        return 2

    # 预告理论预测，跑完可以直接对照
    print('矩阵给出的预测（bench/kinematics.py，有单测）：')
    for h in headings_deg:
        r = math.radians(h)
        print(f'  {h:6.1f}° : 出力轮数 {kin.driving_wheel_count(r)}'
              f' · 单轮转速需求 {kin.max_wheel_speed_for_unit_translation(r):6.2f} rad/s per m/s'
              f' · 可用力 {kin.max_force_along(r):6.1f} N'
              f' ({kin.force_anisotropy_ratio(r):.3f}× of 0°)')
    print()

    with Session(TEST_ID, args.env, force=args.force, need_open_loop=True,
                 extra_context={
                     'headings_deg': headings_deg,
                     'distance_m': args.distance,
                     'cruise_mps': args.cruise,
                     'accel_mps2': args.accel,
                     'n_per_heading': args.n,
                     'settle_sec': SETTLE_SEC,
                 }) as s:

        truth = SimOdomTruth(s.node)
        truth.wait_ready()
        s.context['truth_source'] = truth.name
        s.context['truth_is_ground_truth'] = truth.is_ground_truth

        driver = SimCmdVelDriver(s.node, rate_hz=50.0)

        result = st.Result(TEST_ID, args.env,
                           description=f'各向同性：{len(headings_deg)} 个方向 × '
                                       f'{args.n} 次 × {args.distance:g} m',
                           context=s.context)
        result.note('低速低加速度是有意的：本项测运动学，动力学边界由 b3 单独测。')
        result.note(f'真值源：{truth.name}。仿真的 /odom 基于模型真实位姿、'
                    f'与底盘运动学解耦，所以打滑时它不会跟指令一起错。')

        # 每个方向一个"距离误差比"指标 + 一个"垂直偏移"指标
        along_metrics = {}
        perp_metrics = {}
        for h in headings_deg:
            along_metrics[h] = result.metric(
                f'{h:g}° 距离误差比', '', threshold=RANGE_RATIO_THRESHOLD, min_n=args.n,
                threshold_basis='与各方向极差同口径 3%；1 m 行程上等于 0.03 m')
            perp_metrics[h] = result.metric(
                f'{h:g}° 垂直偏移绝对值', 'm', threshold=LATERAL_THRESHOLD_M, min_n=args.n,
                threshold_basis='1 m 行程上 0.03 m 等于方向偏差 1.7°')

        raw_by_heading = {h: [] for h in headings_deg}
        aborted = None
        try:
            with driver:
                for h in headings_deg:
                    r = math.radians(h)
                    for i in range(args.n):
                        print(f'[{h:g}° {i + 1}/{args.n}] 走 {args.distance:g} m ...',
                              end='', flush=True)
                        along, perp, dyaw, cmd_travel = _run_one_sim(
                            s.node, driver, truth, r,
                            args.distance, args.cruise, args.accel)
                        raw_by_heading[h].append(along)
                        err_ratio = abs(along - args.distance) / args.distance
                        along_metrics[h].add(err_ratio, trial=i, along_m=along,
                                             commanded_m=args.distance,
                                             cmd_layer_travel_m=cmd_travel)
                        perp_metrics[h].add(abs(perp), trial=i, perp_m=perp,
                                            dyaw_rad=dyaw)
                        print(f' 实测 {along:+.4f} m (误差 {err_ratio * 100:5.2f}%)'
                              f' 垂直 {perp:+.4f} m 偏航 {dyaw:+.4f} rad')
        except (KeyboardInterrupt, TimeoutError) as exc:
            aborted = repr(exc)
            result.note(f'⚠️ 提前中止：{exc}。已采集的样本仍然写盘，'
                        f'但样本数不足的指标会判 INCONCLUSIVE 而不是 PASS。')

        # 跨方向的极差：这是本项的核心结论
        means = {h: (sum(v) / len(v)) for h, v in raw_by_heading.items() if v}
        if len(means) >= 2:
            lo, hi = min(means.values()), max(means.values())
            range_ratio = (hi - lo) / args.distance
            rng = result.metric('各方向距离极差比', '',
                                threshold=RANGE_RATIO_THRESHOLD, min_n=1,
                                threshold_basis='运动学矩阵错的主要表现；'
                                                '3% 对应 1 m 行程 0.03 m')
            rng.add(range_ratio,
                    per_heading_mean={f'{h:g}': m for h, m in means.items()},
                    min_m=lo, max_m=hi)
            result.note('各方向实测均值：'
                        + '，'.join(f'{h:g}°={m:.4f} m' for h, m in sorted(means.items())))

            # 45° 与 0° 的比较：与矩阵预测对照
            if 0.0 in means and 45.0 in means:
                ratio_45_0 = means[45.0] / means[0.0] if means[0.0] else float('nan')
                result.note(
                    f'45°/0° 实测距离比 = {ratio_45_0:.4f}。'
                    f'矩阵预测的**力**比是 {kin.force_anisotropy_ratio(math.radians(45)):.4f}，'
                    f'但在 {args.cruise:g} m/s 这种低速下力余量充足，'
                    f'距离比应该接近 1.000。若显著小于 1，说明力/摩擦补偿在远离'
                    f'边界处就已经不够（去查 pid_kp 与 friction_viscous_nm_s），'
                    f'而不是"斜向本来就弱"。')

        # 单方向重复性 σ：用 stats 的 σ，但单独立一个指标做判据
        for h in headings_deg:
            v = raw_by_heading[h]
            if len(v) >= 2:
                import statistics
                sm = result.metric(f'{h:g}° 重复性 σ', 'm',
                                   threshold=SIGMA_THRESHOLD_M, min_n=1,
                                   threshold_basis='均值误差可标定，σ 标定不掉；'
                                                   '对接/穿窄门受制于 σ')
                sm.add(statistics.stdev(v), n=len(v))

        if aborted:
            result.context['aborted'] = aborted

        print()
        print(result.to_markdown())
        paths = result.save(results_dir(args.results_dir))
        print(f'结果已写入：\n  {paths[0]}\n  {paths[1]}')
        return 0 if result.verdict == st.PASS else 1


if __name__ == '__main__':
    sys.exit(run_main(main))
