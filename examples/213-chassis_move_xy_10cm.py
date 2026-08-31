#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2026, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------

"""
File: 213-chassis_move_xy_10cm.py
Brief: 底盘在 x/y 方向各移动 10 cm。复用仓库自己的 chassis_integrator 做速度->位置积分。

================================================================================
先讲清楚为什么**不能**用 Nav2 做这件事
================================================================================
Nav2 那条路径跟踪链（SmacPlanner2D + RPP/MPPI）在 10 cm 这个尺度上**结构性地不可用**，
不是"精度不够"而是"根本不会动"：

    nav2_params_rpp.yaml:35  goal_checker_plugins: ["general_goal_checker"]
    nav2_params_rpp.yaml:71  xy_goal_tolerance: 0.25

`SimpleGoalChecker` 在每个控制周期检查「当前位姿到目标的距离 < xy_goal_tolerance」。
目标只有 0.10 m 远，而容差是 0.25 m —— **第一次检查就判定已到达**，
controller_server 直接返回成功，机器人一步都不走。
（就算切到那个更严的 precise 档 0.18 m，0.10 < 0.18 仍然立刻成功。）

所以 10 cm 级的移动只能走**底盘指令层**，也就是本文件的做法：
仓库里 `astribot_trajectory_bridge/chassis_integrator.py` 那套速度->位置积分，
它本来就是给 Nav2 的 `/cmd_vel` 落到 SDK 位置通道用的，纯函数、可离线单测。
本例直接 import 它，不重写一份，避免两处漂移。

================================================================================
坐标系规则（搞错就是往反方向走）
================================================================================
1. **底盘是 3 个"关节"** `[x, y, theta]`，走 `set_joints_position` 的**位置**通道。
   `RobotJointController.msg` 只有 header/mode/name/command，线上没有速度字段。

2. **theta 是绝对朝向**：203 用 `get_desired_joints_position()[0][2]` 直接构造旋转矩阵
   `R(theta)`（body->world），说明这个分量是一个绝对角度。

3. **x/y 的增量按本体系累加**：203 先把世界系速度用 `rot_mat.T` 转成本体系
   (`203:61`)，**然后才** 累加进 pos_cmd (`203:63-65`)。也就是说 pos_cmd 的 x/y
   增量口径是**本体系**，不是世界系。`chassis_integrator.to_local_velocity()`
   把这两种口径显式做成了 `FRAME_BODY` / `FRAME_WORLD` 两个值。

4. 因此本例**全程锁定 theta 不变**（只发平移、wz 恒为 0）。theta 不变时
   本体系 == 启动时刻的那个系，「x 正方向 10 cm」就没有歧义了。
   若移动过程中还要转向，body/world 两种口径会分叉，必须显式选 `input_frame`。

   > 想实测确认第 3 条：先在 theta=0 时走 +x 10 cm，再原地转 90°（用 212），
   > 然后再走一次 +x 10 cm。若第二次是朝原来的 +y 世界方向走，则 x/y 增量确实是
   > 本体系口径；若仍朝原来的 +x 走，那就是世界系口径，本文件的 FRAME 选择要改。

5. **不要用 SDK 的 `world` 系表达目标**：它在会话启动时以当前底盘位姿重置，
   无法表达场景里的全局坐标。本例只用底盘自己的 3 个关节量。

6. TF 侧的根 frame 叫 `astribot_torso_base`（不是 `base_link`）。本例不查 TF，
   列在这里是因为拿这个例子去接 TF 时最容易踩。

================================================================================
为什么速度定得这么低
================================================================================
10 cm 和底盘的**停车距离同量级**，这是本例最大的实际约束：

* 理论最短停车距离 v^2/(2a)：0.5 m/s @ 2.5 m/s^2 = 0.05 m —— 已经是目标的一半
* Gazebo 实测滑行 0.069 ~ 0.100 m（1.4~2x 理论），且 4 s 仍未完全停住
* 真机上 `control_way='filter'` 还会在此之后再收敛约等量一段

所以默认 `V_MAX=0.08 m/s`、`ACCEL=0.25 m/s^2`，制动距离 v^2/(2a) = 0.0128 m，
只占行程的 13%。**把速度调高就会过冲**，这不是保守，是算出来的。

================================================================================
用法
================================================================================
    python3 examples/213-chassis_move_xy_10cm.py                 # 斜向一次走完(默认)
    python3 examples/213-chassis_move_xy_10cm.py --mode sequential  # 先 x 再 y
    python3 examples/213-chassis_move_xy_10cm.py --dx 0.1 --dy 0.0  # 只走 x
"""

import argparse
import math
import os
import sys
import threading
import time

# 复用仓库自己的积分算法。examples/ 不在那个 ROS 包的 python path 上，
# 所以这里按仓库相对位置显式加进去——刻意不复制一份函数进来，
# 两处漂移的后果是"测试按旧口径算、桥接按新口径跑"，谁都不报错。
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO_ROOT, 'ws_robot', 'src',
                                'astribot_trajectory_bridge'))
from astribot_trajectory_bridge import chassis_integrator as ci   # noqa: E402

from astribot_sdk.core.astribot_api.astribot_client import Astribot   # noqa: E402
import astribot_ros_middleware as ast_astribot_middleware              # noqa: E402

# ---------------- 可调参数 ----------------
FREQ = 250.0          # 控制频率 (Hz)，与 202/203 一致
V_MAX = 0.08          # 巡航速度 (m/s)。见文件头"为什么速度定得这么低"
ACCEL = 0.25          # 加减速度 (m/s^2)
HOLD_SEC = 3.0        # 到位后继续重发最终指令的时长，等 filter 收敛
PRINT_HZ = 5.0
POS_EPS = 1e-4        # 到位判据 (m)
# chassis_integrator 的限幅参数。与 nav2 velocity_smoother 同口径但取得更保守，
# 因为本例的行程只有 10 cm。
MAX_VEL_XY = 0.30
MAX_VEL_THETA = 0.50
MAX_ACCEL_XY = 0.50
MAX_ACCEL_THETA = 1.0
# ------------------------------------------


def brake_capped_speed(remaining, v_max, accel, v_now, dt):
    """由剩余距离反推本周期速度，**离散安全版**。

    天真写法 v = sqrt(2*a*s) 要求的减速度恰好等于 a（dv/dt = (a/v)*v = a），
    一步都不能落后；按当前剩余量算上限再限幅等于永远晚一个周期，会过冲。
    对开环积分位置的底盘来说，过冲意味着指令位置要往回收，正是漂移累积的来源。

    改成要求「本周期走完之后仍然刹得住」：
        v^2/(2a) + v*dt <= |s|   ->   v <= -a*dt + sqrt((a*dt)^2 + 2*a*|s|)
    """
    adt = accel * dt
    cap = min(v_max, max(0.0, -adt + math.sqrt(adt * adt + 2.0 * accel * abs(remaining))))
    target = math.copysign(cap, remaining) if remaining != 0.0 else 0.0
    return v_now + max(-adt, min(adt, target - v_now))


def move_leg(astribot, chassis, pos_cmd, dx, dy, label):
    """走一段直线位移 (dx, dy)（本体系，m）。返回更新后的 pos_cmd 与实测结果。

    theta 全程不动：wz 恒为 0，所以本体系 == 启动时刻的系。
    """
    dist = math.hypot(dx, dy)
    if dist < POS_EPS:
        print(f'[{label}] 位移 {dist:.4f} m 太小，跳过')
        return pos_cmd, None

    ux, uy = dx / dist, dy / dist
    dt = 1.0 / FREQ
    print_every = max(1, int(FREQ / PRINT_HZ))

    start_actual = list(astribot.get_current_joints_position([chassis])[0])
    start_cmd = list(pos_cmd)
    print(f'\n[{label}] 目标位移 ({dx:+.4f}, {dy:+.4f}) m，'
          f'合成 {dist:.4f} m，方向 {math.degrees(math.atan2(dy, dx)):+.1f}°')
    print(f'[{label}] 起点实测 x={start_actual[ci.IDX_X]:+.4f} '
          f'y={start_actual[ci.IDX_Y]:+.4f} theta={start_actual[ci.IDX_THETA]:+.4f}')

    traveled = 0.0
    speed = 0.0
    prev_vel = (0.0, 0.0, 0.0)
    tick = 0
    t0 = time.monotonic()

    while ast_astribot_middleware.ok():
        remaining = dist - traveled
        speed = brake_capped_speed(remaining, V_MAX, ACCEL, speed, dt)
        # 沿指令方向分解成本体系 (vx, vy)；wz 恒 0，theta 不动
        desired = (speed * ux, speed * uy, 0.0)

        # ---- 走仓库自己的那套：口径换算 -> 限幅 -> 斜率限制 -> 积分 ----
        local_vel = ci.to_local_velocity(desired, ci.FRAME_BODY,
                                         pos_cmd[ci.IDX_THETA])
        local_vel = ci.clamp_velocity(local_vel, MAX_VEL_XY, MAX_VEL_THETA)
        local_vel = ci.slew_limit_velocity(local_vel, prev_vel,
                                           MAX_ACCEL_XY, MAX_ACCEL_THETA, dt)
        prev_vel = local_vel
        pos_cmd = ci.integrate_step(pos_cmd, local_vel, FREQ)
        traveled += math.hypot(local_vel[0] * dt, local_vel[1] * dt)

        astribot.set_joints_position([chassis], [list(pos_cmd)])

        if tick % print_every == 0:
            act = astribot.get_current_joints_position([chassis])[0]
            moved = math.hypot(act[ci.IDX_X] - start_actual[ci.IDX_X],
                               act[ci.IDX_Y] - start_actual[ci.IDX_Y])
            print(f'[{label}] 指令 {traveled:.4f} m / 实测 {moved:.4f} m '
                  f'| v={math.hypot(local_vel[0], local_vel[1]):.4f} m/s '
                  f'| dtheta={act[ci.IDX_THETA] - start_actual[ci.IDX_THETA]:+.4f} rad')
        tick += 1

        if remaining < POS_EPS and math.hypot(local_vel[0], local_vel[1]) < POS_EPS:
            break
        if time.monotonic() - t0 > 60.0:
            print(f'[{label}] !! 超时，已走指令量 {traveled:.4f}/{dist:.4f} m')
            break
        rate.sleep()

    # 到位后必须继续重发：control_way='filter' 下 SDK 还会自己再收敛一段，
    # 循环一停就没人抓着底盘了（实测不重发会漂）。
    print(f'[{label}] 指令到位，保持重发 {HOLD_SEC:.1f}s 等收敛')
    for _ in range(int(HOLD_SEC * FREQ)):
        astribot.set_joints_position([chassis], [list(pos_cmd)])
        rate.sleep()

    end_actual = list(astribot.get_current_joints_position([chassis])[0])
    ax = end_actual[ci.IDX_X] - start_actual[ci.IDX_X]
    ay = end_actual[ci.IDX_Y] - start_actual[ci.IDX_Y]
    along = ax * ux + ay * uy
    perp = -ax * uy + ay * ux
    cmd_dx = pos_cmd[ci.IDX_X] - start_cmd[ci.IDX_X]
    cmd_dy = pos_cmd[ci.IDX_Y] - start_cmd[ci.IDX_Y]
    res = {
        'label': label,
        'commanded': (cmd_dx, cmd_dy),
        'actual': (ax, ay),
        'along': along,
        'perp': perp,
        'dtheta': ci.wrap_angle(end_actual[ci.IDX_THETA] - start_actual[ci.IDX_THETA]),
        'duration_sec': time.monotonic() - t0,
    }
    print(f'[{label}] 完成：指令 ({cmd_dx:+.4f}, {cmd_dy:+.4f}) '
          f'实测 ({ax:+.4f}, {ay:+.4f}) '
          f'沿向 {along:+.4f} 垂向 {perp:+.4f} dtheta {res["dtheta"]:+.4f}')
    return pos_cmd, res


def main():
    ap = argparse.ArgumentParser(description='底盘 x/y 各移动 10 cm')
    ap.add_argument('--dx', type=float, default=0.10, help='x 方向位移 (m)')
    ap.add_argument('--dy', type=float, default=0.10, help='y 方向位移 (m)')
    ap.add_argument('--mode', choices=('diagonal', 'sequential'), default='diagonal',
                    help='diagonal=斜向一次走完(45°，X 构型下只有两个轮出力)；'
                         'sequential=先 x 再 y')
    args = ap.parse_args()

    global rate
    astribot = Astribot(freq=FREQ)
    rate = ast_astribot_middleware.Rate(FREQ)
    chassis = astribot.chassis_name

    def spin_loop():
        while ast_astribot_middleware.ok():
            ast_astribot_middleware.spin()

    threading.Thread(target=spin_loop, daemon=True).start()

    # 种子取 desired（与 202/203 一致）。actual 只用于事后比对，不做种子——
    # 拿 actual 做种子会把当前的跟踪误差当成新的目标基准。
    pos_cmd = list(astribot.get_desired_joints_position([chassis])[0])
    if len(pos_cmd) != ci.CHASSIS_DOF_S1:
        print(f'!! 底盘自由度是 {len(pos_cmd)}，期望 {ci.CHASSIS_DOF_S1}。'
              f'ROBOT_TYPE 没设成 S1 时 chassis_dof 是 2，本例不适用。')
        return 2

    print('=' * 72)
    print(f'底盘位移示例 · mode={args.mode} · dx={args.dx:+.3f} dy={args.dy:+.3f} m')
    print(f'V_MAX={V_MAX} m/s  ACCEL={ACCEL} m/s^2  '
          f'制动距离 v^2/(2a)={V_MAX ** 2 / (2 * ACCEL):.4f} m')
    print(f'种子(desired) x={pos_cmd[0]:+.4f} y={pos_cmd[1]:+.4f} '
          f'theta={pos_cmd[2]:+.4f}')
    print('theta 全程不动 -> 本体系 == 启动时刻的系，x/y 方向无歧义')
    print('=' * 72)

    legs = ([('xy 斜向', args.dx, args.dy)] if args.mode == 'diagonal'
            else [('x 方向', args.dx, 0.0), ('y 方向', 0.0, args.dy)])

    results = []
    try:
        for label, dx, dy in legs:
            pos_cmd, res = move_leg(astribot, chassis, pos_cmd, dx, dy, label)
            if res is not None:
                results.append(res)
    except KeyboardInterrupt:
        # 不能直接退出：停止重发会让还在动的底盘继续漂。保持当前指令位置一段。
        print('\n收到中断，保持当前指令位置 3s 后退出（不要再按 Ctrl-C）')
        for _ in range(int(3.0 * FREQ)):
            astribot.set_joints_position([chassis], [list(pos_cmd)])
            rate.sleep()

    print('\n' + '=' * 72)
    print('汇总（实测，不是指令回显）')
    print('=' * 72)
    for r in results:
        err = abs(r['along']) - math.hypot(*r['commanded'])
        print(f"{r['label']:<10s} 指令 {math.hypot(*r['commanded']):.4f} m  "
              f"实测沿向 {r['along']:+.4f} m  误差 {err * 1000:+.1f} mm  "
              f"垂向 {r['perp'] * 1000:+.1f} mm  "
              f"dtheta {math.degrees(r['dtheta']):+.2f}°  "
              f"{r['duration_sec']:.1f}s")
    if results:
        print('\n判读要点：')
        print('  · 垂向偏移应接近 0。持续偏一侧 -> 运动学系数或几何标定问题')
        print('  · dtheta 应接近 0。转了 -> 纯平移指令产生了偏航，交叉耦合')
        print('  · 误差为负(走不够) 且随速度增大 -> 打滑；与速度无关 -> 标度因子')
    return 0


if __name__ == '__main__':
    sys.exit(main())
