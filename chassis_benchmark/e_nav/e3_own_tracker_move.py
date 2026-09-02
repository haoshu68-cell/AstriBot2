#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E3 · 自己实现的路径跟踪律，走 30 cm。**不调用 Nav2 的任何接口。**

与 e2_nav2_short_move.py 的关系
==============================
e2 派发 ``/navigate_to_pose``，整条链是 Nav2 的（BT + 全局规划 + controller_server +
goal_checker + velocity_smoother）。本脚本一个都不用，自己闭环：

    直线路径生成  ->  bench/tracking.py 的控制律  ->  限幅/斜率限制  ->  底盘

这样做的两个实际好处：

1. **摆脱到位容差地板。** Nav2 侧生效的 ``xy_goal_tolerance`` 是 0.18，而 30 cm 任务里
   它吃掉 60%（单轴）/42%（斜向）的行程；而 0.10 已被实测否决不能调。
   自己闭环时到位判据由本脚本定，可以一路测到底盘的物理极限在哪。
2. **控制律的正确性已经离线证明。** ``tests/test_tracking.py`` 里有 31 个用例，
   含理想运动学下的闭环仿真（收敛、不过冲、横向误差、终态航向）。
   所以在机器人上出问题时可以直接排除"律写错了"——仿真里分不清这两者。

两套律，同一条路径、同一组参数
============================
* ``--law omni``（默认）全向律：前视点方向直接分解成 (vx, vy)，航向单独 P 律保持，
  速度用"刹得住"曲线收到 0（**没有** RPP 的 0.05 m/s 接近速度下限）。
* ``--law rpp`` 差速律：忠实复刻 Regulated Pure Pursuit，前视点 -> 曲率 ->
  曲率/接近双重限速，**输出没有 vy**，角差超 0.785 rad 时先原地转向。

离线闭环仿真给出的对照（理想执行，20 Hz）：

    律     目标          步数   终点误差   最大横向误差   终态 yaw
    omni  (0.3, 0.3)      17    0.0159     0.0000         0.0°
    rpp   (0.3, 0.3)      89    0.0494     0.0472        49.1°

真机/仿真上应该复现"终态 yaw"这个判别量：omni ≈ 0°，rpp ≈ 45°。

参数全部沿用仓库现值
==================
``bench/tracking.py`` 顶部逐字抄了 ``nav2_params_rpp.yaml`` 的 FollowPath 段与
``controller_frequency: 20.0``；限幅/斜率限制复用
``astribot_trajectory_bridge/chassis_integrator.py``（与 examples/213 同一套），
不另写一份。

用法
====
    # 仿真：只起 warehouse_sim.launch.py，**不要**起 Nav2（会抢 /cmd_vel）
    python3 e_nav/e3_own_tracker_move.py --env sim --n 5
    python3 e_nav/e3_own_tracker_move.py --env sim --n 5 --law rpp
    python3 e_nav/e3_own_tracker_move.py --env sim --n 3 --mode sequential
    python3 e_nav/e3_own_tracker_move.py --env sim --n 5 --stop-tol 0.02
"""

import math
import os
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_REPO_ROOT = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO_ROOT, 'ws_robot', 'src',
                                'astribot_trajectory_bridge'))

from bench import stats as st                                      # noqa: E402
from bench import tracking as tr                                   # noqa: E402
from bench.driver import SimCmdVelDriver                           # noqa: E402
from bench.session import (Session, base_arg_parser, results_dir,   # noqa: E402
                           run_main)
from bench.truth import SimOdomTruth, displacement, unwrap_delta   # noqa: E402

from astribot_trajectory_bridge import chassis_integrator as ci     # noqa: E402

TEST_ID = 'e3_own_tracker_move'

CONTROL_HZ = tr.CONTROLLER_FREQUENCY_HZ     # 20.0，与 controller_frequency 对齐
DEFAULT_STOP_TOL = 0.03                     # 本脚本自己的到位判据（不是 nav2 的 0.18）
DEFAULT_CRUISE = 0.30                       # 30 cm 任务用不到 desired_linear_vel 0.5
SETTLE_SEC = 2.0
RUN_TIMEOUT_SEC = 40.0
# 起步原地转角判别阈值，与 e2 保持同一口径
ROTATE_FIRST_RAD = 0.39
MOVE_ONSET_M = 0.02


def run_one(node, driver, truth, law_name, dx, dy, cruise, stop_tol, label):
    """跑一趟：生成直线路径 -> 闭环跟踪 -> 收尾。返回度量字典。"""
    p0 = truth.pose()
    # 路径在**世界系(odom)**里生成：起点是当前实测位姿，终点是本体系偏移旋进世界系。
    # 「x/y 各走 30 cm」是本体系语义，所以偏移必须按当前 yaw 旋转。
    c, s = math.cos(p0.yaw), math.sin(p0.yaw)
    gx = p0.x + c * dx - s * dy
    gy = p0.y + s * dx + c * dy
    path = tr.straight_path(p0.x, p0.y, gx, gy, step=0.02)
    total = math.hypot(gx - p0.x, gy - p0.y)

    law = tr.omni_step if law_name == 'omni' else tr.rpp_step
    dt = 1.0 / CONTROL_HZ
    idx = 0
    prev_vel = (0.0, 0.0, 0.0)
    xte_max = 0.0
    rotate_first = 0.0
    moved_onset = False
    peak_vy = 0.0
    peak_wz = 0.0
    path_len = 0.0
    last = p0
    rotating_steps = 0
    n_steps = 0
    t0 = time.monotonic()
    timed_out = False

    while rclpy.ok():
        p = truth.pose()
        d_goal = math.hypot(gx - p.x, gy - p.y)
        if d_goal <= stop_tol:
            break
        if time.monotonic() - t0 > RUN_TIMEOUT_SEC:
            timed_out = True
            break

        if law_name == 'omni':
            twist, info = law((p.x, p.y, p.yaw), path, dt, from_index=idx,
                              reference_yaw=p0.yaw, desired_linear_vel=cruise)
        else:
            twist, info = law((p.x, p.y, p.yaw), path, dt, from_index=idx,
                              desired_linear_vel=cruise)
        idx = info['path_index']
        if info.get('rotating_in_place'):
            rotating_steps += 1

        # ---- 复用仓库的限幅 + 斜率限制（与 examples/213 同一套）----
        vel = ci.clamp_velocity(twist, tr.MAX_VEL_XY, tr.MAX_VEL_THETA)
        vel = ci.slew_limit_velocity(vel, prev_vel,
                                     tr.MAX_ACCEL_XY, tr.MAX_ACCEL_THETA, dt)
        prev_vel = vel
        driver.set_velocity(vx=vel[0], vy=vel[1], wz=vel[2])

        peak_vy = max(peak_vy, abs(vel[1]))
        peak_wz = max(peak_wz, abs(vel[2]))
        xte_max = max(xte_max, tr.cross_track_error((p.x, p.y), path))
        if p.stamp != last.stamp:
            path_len += math.hypot(p.x - last.x, p.y - last.y)
            last = p
        net = math.hypot(p.x - p0.x, p.y - p0.y)
        if not moved_onset:
            rotate_first = max(rotate_first, abs(unwrap_delta(p.yaw, p0.yaw)))
            if net > MOVE_ONSET_M:
                moved_onset = True
        n_steps += 1

        end = time.monotonic() + dt
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.002)

    driver.halt()
    settle_end = time.monotonic() + SETTLE_SEC
    while time.monotonic() < settle_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.005)

    p_end = truth.pose()
    fwd, lat, dyaw = displacement(p0, p_end)
    # 停下之后又滑了多少：从"判定到位那一刻"到"静止"的额外位移
    return {
        'label': label, 'law': law_name, 'timed_out': timed_out,
        'goal_error_m': math.hypot(p_end.x - gx, p_end.y - gy),
        'net_displacement_m': math.hypot(p_end.x - p0.x, p_end.y - p0.y),
        'commanded_total_m': total,
        'path_length_m': path_len,
        'xte_max_m': xte_max,
        'body_fwd_m': fwd, 'body_lat_m': lat, 'dyaw_rad': dyaw,
        'rotate_first_rad': rotate_first,
        'rotating_steps': rotating_steps,
        'peak_abs_vy': peak_vy, 'peak_abs_wz': peak_wz,
        'control_steps': n_steps,
        'duration_sec': time.monotonic() - t0,
    }


def main(argv=None):
    p = base_arg_parser(TEST_ID, 'E3 自己实现的跟踪律走 30 cm（不用 Nav2）')
    p.add_argument('--law', choices=('omni', 'rpp'), default='omni',
                   help='omni=全向律(默认)  rpp=差速律(复刻 RegulatedPurePursuit)')
    p.add_argument('--dx', type=float, default=0.30, help='本体系 x 位移 (m)')
    p.add_argument('--dy', type=float, default=0.30, help='本体系 y 位移 (m)')
    p.add_argument('--mode', choices=('diagonal', 'sequential'), default='diagonal')
    p.add_argument('--cruise', type=float, default=DEFAULT_CRUISE,
                   help='巡航速度上限 (m/s)')
    p.add_argument('--stop-tol', type=float, default=DEFAULT_STOP_TOL,
                   help='本脚本自己的到位判据 (m)。Nav2 侧那个 0.18 在这里不适用')
    args = p.parse_args(argv)

    if args.env != 'sim':
        print('真机版未实现：真机没有独立真值源，闭环只能吃 SDK 的航迹推算，\n'
              '那样控制与评价共用同一个可能错的量。真机应人工量终点误差，\n'
              '并把闭环反馈这件事单独讨论。')
        return 2

    legs = ([('xy 斜向', args.dx, args.dy)] if args.mode == 'diagonal'
            else [('x', args.dx, 0.0), ('y', 0.0, args.dy)])
    leg_dist = math.hypot(args.dx, args.dy) if args.mode == 'diagonal' else max(
        abs(args.dx), abs(args.dy))

    print('=' * 78)
    print(f'不用 Nav2。控制律 = {args.law}，控制频率 {CONTROL_HZ:g} Hz '
          f'(= controller_frequency)')
    print(f'到位判据 {args.stop_tol:g} m —— 这是本脚本自己的判据；'
          f'Nav2 侧生效的是 0.18 m')
    print(f'  0.18 会吃掉本任务 {min(1.0, 0.18 / leg_dist) * 100:.0f}% 的行程，'
          f'而本判据只吃掉 {min(1.0, args.stop_tol / leg_dist) * 100:.0f}%')
    print(f'前视距离：标称 {tr.LOOKAHEAD_DIST:g} m 比全程 {leg_dist:.4f} m 还长 -> '
          f'已改为随剩余距离收缩（{tr.LOOKAHEAD_SHRINK_K:g}×剩余，下限 '
          f'{tr.LOOKAHEAD_FLOOR:g} m）')
    if args.law == 'rpp':
        print(f'rpp 律：角差 > {tr.ROTATE_TO_HEADING_MIN_ANGLE:g} rad 时先原地转向；'
              f'接近速度下限 {tr.MIN_APPROACH_LINEAR_VELOCITY:g} m/s（刹不到零）')
    else:
        print('omni 律：不先转向、无接近速度下限（用"刹得住"曲线收到 0）')
    print('=' * 78)

    with Session(TEST_ID, args.env, force=args.force,
                 need_open_loop=True,        # 必须确认 Nav2 没在抢 /cmd_vel
                 extra_context={
                     'law': args.law, 'control_hz': CONTROL_HZ,
                     'dx_body_m': args.dx, 'dy_body_m': args.dy, 'mode': args.mode,
                     'cruise_mps': args.cruise, 'stop_tol_m': args.stop_tol,
                     'nav2_xy_goal_tolerance_for_reference': 0.18,
                     'lookahead_nominal': tr.LOOKAHEAD_DIST,
                     'lookahead_shrink_k': tr.LOOKAHEAD_SHRINK_K,
                     'lookahead_floor': tr.LOOKAHEAD_FLOOR,
                     'reused_limiter': 'chassis_integrator.clamp_velocity + '
                                       'slew_limit_velocity',
                 }) as s:

        truth = SimOdomTruth(s.node)
        truth.wait_ready()
        s.context['truth_source'] = truth.name
        s.context['truth_note'] = ('仿真里 /odom 基于模型真实位姿，既是控制反馈也是真值。'
                                   '真机上这两者必须分开。')

        driver = SimCmdVelDriver(s.node, rate_hz=50.0)

        result = st.Result(TEST_ID, args.env,
                           description=f'{args.law} 律 · {args.mode} '
                                       f'({args.dx:+.2f}, {args.dy:+.2f}) m × {args.n}',
                           context=s.context)
        result.note('控制律的正确性已在 tests/test_tracking.py 离线证明'
                    '（含理想运动学闭环：收敛/不过冲/横向误差/终态航向）。'
                    '所以这里若出问题，可以排除"律写错了"。')
        result.note('限幅与斜率限制复用 chassis_integrator，与 examples/213 同一套，'
                    '不另写一份——两处漂移谁都不报错。')

        err_m = result.metric('终点误差', 'm', threshold=args.stop_tol * 2.0,
                              min_n=args.n,
                              threshold_basis=f'到位判据 {args.stop_tol:g} m 的 2 倍余量；'
                                              f'超出说明停不住（滑行超预期）')
        xte_m = result.metric('最大横向误差', 'm', threshold=0.05, min_n=args.n,
                              threshold_basis='30 cm 直线任务上横向误差应远小于'
                                              'inflation 余量 0.23 m，取 0.05 做紧判据')
        rot_m = result.metric('起步原地转角', 'rad', min_n=args.n,
                              threshold_basis='只记录。omni 应≈0，rpp 应≈0.785')
        vy_m = result.metric('|vy| 峰值', 'm/s', min_n=args.n,
                             threshold_basis='只记录。rpp 结构上恒 0')
        dyaw_m = result.metric('终态偏航变化绝对值', 'rad', min_n=args.n,
                               threshold_basis='只记录。这是两套律最干净的判别量')
        to_m = result.metric('超时次数', '次', threshold=0, min_n=1,
                             threshold_basis='跟踪律应在 40s 内完成 30 cm')

        raws = []
        timeouts = 0
        try:
            with driver:
                for i in range(args.n):
                    for label, dx, dy in legs:
                        print(f'\n[{i + 1}/{args.n} {label}] ...')
                        r = run_one(s.node, driver, truth, args.law, dx, dy,
                                    args.cruise, args.stop_tol, f'{label}#{i}')
                        raws.append(r)
                        if r['timed_out']:
                            timeouts += 1
                        print(f"  终点误差 {r['goal_error_m']:.4f} m"
                              f" · 净位移 {r['net_displacement_m']:.4f}"
                              f" / 目标 {r['commanded_total_m']:.4f} m")
                        print(f"  最大横向误差 {r['xte_max_m']:.4f} m"
                              f" · 起步转角 {math.degrees(r['rotate_first_rad']):.1f}°"
                              f" · 终态 dyaw {math.degrees(r['dyaw_rad']):+.1f}°")
                        print(f"  |vy|峰值 {r['peak_abs_vy']:.4f}"
                              f" · |wz|峰值 {r['peak_abs_wz']:.4f}"
                              f" · 控制步 {r['control_steps']}"
                              f" · {r['duration_sec']:.1f}s"
                              + ('  !! 超时' if r['timed_out'] else ''))
                        err_m.add(r['goal_error_m'], **{k: r[k] for k in
                                                        ('label', 'net_displacement_m',
                                                         'timed_out')})
                        xte_m.add(r['xte_max_m'], label=r['label'])
                        rot_m.add(r['rotate_first_rad'], label=r['label'])
                        vy_m.add(r['peak_abs_vy'], label=r['label'])
                        dyaw_m.add(abs(r['dyaw_rad']), label=r['label'])
        except (KeyboardInterrupt, TimeoutError) as exc:
            result.note(f'⚠️ 提前中止：{exc}。已采集样本仍写盘。')

        to_m.add(timeouts, total=len(raws))

        if rot_m.values and vy_m.values:
            mr = sum(rot_m.values) / len(rot_m.values)
            mv = sum(vy_m.values) / len(vy_m.values)
            expect = ('omni：起步转角≈0、|vy| 明显非零' if args.law == 'omni'
                      else 'rpp：起步转角≈0.785 rad、|vy| 恒 0')
            match = ((mr <= ROTATE_FIRST_RAD and mv > 0.01) if args.law == 'omni'
                     else (mr > ROTATE_FIRST_RAD and mv < 1e-9))
            result.note(f'行为对照（期望 {expect}）：实测起步转角均值 {mr:.4f} rad、'
                        f'|vy| 峰值均值 {mv:.4f} m/s -> '
                        f'{"符合" if match else "**不符合**，先看原始样本"}')
            print(f'\n行为对照：期望 {expect}；实测转角 {mr:.4f} rad / '
                  f'|vy| {mv:.4f} m/s -> {"符合" if match else "不符合"}')

        result.context['raw_runs'] = raws
        print()
        print(result.to_markdown())
        paths = result.save(results_dir(args.results_dir))
        print(f'结果已写入：\n  {paths[0]}\n  {paths[1]}')
        return 0 if result.verdict == st.PASS else 1


if __name__ == '__main__':
    sys.exit(run_main(main))
