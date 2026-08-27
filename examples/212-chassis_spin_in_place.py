#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2026, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------

"""
File: 212-chassis_spin_in_place.py
Brief: 底盘原地转圈 demo。参考 202-chassis_joy_control_local.py，把手柄换成内置速度规划。

和 202 的关系
============
202 的循环本质是「速度 -> 位置增量 -> set_joints_position」：

    pos_cmd = astribot.get_desired_joints_position([chassis_name])[0]   # [x, y, theta] 种子
    pos_cmd[i] += vel_cmd[i] / freq
    astribot.set_joints_position([chassis_name], [pos_cmd])

底盘在这套接口里就是 3 个"关节" [x, y, theta]，走的是 set_joints_position 的
**位置**通道（RobotJointController.msg 只有 header/mode/name/command，线上没有速度
字段），速度是上层自己积分出来的。本文件只改两处：

  1. vel_cmd 不来自手柄，来自下面的梯形速度规划（可复现，不用插手柄）；
  2. 只累加 theta，x/y 恒等于**启动时的种子值** —— 这就是「原地」。
     注意这是有意的：底盘是位置指令开环积分，若把 x/y 也每周期重新读 desired，
     打滑造成的漂移会被当成新起点一路累积。锁死种子值等于持续要求「回到原点」。

三个必须遵守的约束
================
  * 指令必须**每周期持续重发**。发一次抓不住正在运动的关节。
  * spin 必须放在独立线程里，否则收不到状态更新，get_* 全是陈旧值。
  * control_way 默认 'filter'：指令停下后 SDK 还会自己再收敛一段。
    所以到位后要继续重发最终指令（HOLD_SEC），不能立刻退出循环；
    Ctrl-C 也必须先减速到 0 再退，不能直接 break。

本文件不自动调 move_to_home()：那会带动整个上半身。真机上转圈前请自行把手臂收好
——展开的手臂既危险，也会被 arm_chassis_speed_coupling 限速。
"""

import math
import threading

from astribot_sdk.core.astribot_api.astribot_client import Astribot
import astribot_ros_middleware as ast_astribot_middleware

# ---------------- 可调参数 ----------------
FREQ = 250.0          # 控制频率 (Hz)，与 202 一致
TURNS = 0.25          # 转多少圈。0.25 = 90 度，首次上真机先用这个值
DIRECTION = +1        # +1 逆时针（theta 增大），-1 顺时针
OMEGA_MAX = 0.5       # 巡航角速度 (rad/s)。0.5 时一圈约 13 s
RAMP_SEC = 1.5        # 从 0 加到 OMEGA_MAX 的时间 (s)，决定角加速度
HOLD_SEC = 3.0        # 到位后继续重发最终指令的时长 (s)，等 filter 收敛
PRINT_HZ = 2.0        # 打印节流
# -----------------------------------------

IDX_X, IDX_Y, IDX_THETA = 0, 1, 2


def main():
    astribot = Astribot(freq=FREQ)
    rate = ast_astribot_middleware.Rate(FREQ)
    chassis = astribot.chassis_name

    # spin 必须独立线程，否则 get_* 拿到的是陈旧值
    def spin_loop():
        while ast_astribot_middleware.ok():
            ast_astribot_middleware.spin()

    threading.Thread(target=spin_loop, daemon=True).start()

    # 种子：desired 而非 actual，与 202 一致
    seed = list(astribot.get_desired_joints_position([chassis])[0])
    x0, y0, theta0 = seed[IDX_X], seed[IDX_Y], seed[IDX_THETA]

    total = DIRECTION * TURNS * 2.0 * math.pi
    print(f"[spin] 起点 x={x0:.4f} y={y0:.4f} theta={theta0:.4f}, "
          f"目标累计 {total:+.4f} rad ({DIRECTION * TURNS:+.2f} 圈)")

    # 先看一眼 theta 有没有硬限位。底盘 theta 通常是连续累加量（202 也从不归一化），
    # 但多圈会一路增大，真有限位就得在这里发现，而不是跑到一半被拒。
    try:
        lower, upper = astribot.get_joints_position_limit([chassis])
        lo, up = lower[0][IDX_THETA], upper[0][IDX_THETA]
        if math.isfinite(lo) and math.isfinite(up) and not (lo < theta0 + total < up):
            print(f"[spin] !! theta 目标 {theta0 + total:.4f} 超出限位 [{lo:.4f}, {up:.4f}]，"
                  f"减少 TURNS 或改 DIRECTION 后重试")
            return
    except Exception as exc:                                  # noqa: BLE001
        print(f"[spin] 限位查询失败（不阻断，仅提示）: {exc}")

    dt = 1.0 / FREQ
    alpha = OMEGA_MAX / RAMP_SEC          # 角加速度上限 (rad/s^2)
    print_every = max(1, int(FREQ / PRINT_HZ))

    traveled = 0.0        # 已下发的累计转角
    omega = 0.0           # 当前下发角速度
    theta_cmd = theta0
    tick = 0
    hold_ticks = 0
    interrupted = False

    try:
        while ast_astribot_middleware.ok():
            remaining = total - traveled

            if hold_ticks == 0:
                # 「刹得住」速度规划：由剩余角度反推允许速度，天然覆盖
                # 加速/巡航/减速三段，不用写状态机，短距离自动退化成三角形。
                cap = min(OMEGA_MAX, math.sqrt(max(0.0, 2.0 * alpha * abs(remaining))))
                if interrupted:
                    cap = 0.0                                  # 中断后只减速，不再前进
                omega_target = math.copysign(cap, remaining) if remaining != 0.0 else 0.0
                # 角加速度限幅：位置指令的二阶不连续会被 filter 抹掉一部分，但别指望它
                omega += max(-alpha * dt, min(alpha * dt, omega_target - omega))
            else:
                omega = 0.0

            step = omega * dt
            theta_cmd += step
            traveled += step

            # !!! x/y 恒为种子值 —— 这就是「原地」
            astribot.set_joints_position([chassis], [[x0, y0, theta_cmd]])

            if tick % print_every == 0:
                actual = astribot.get_current_joints_position([chassis])[0]
                # 只比 dispatched 等于只验证了自己，必须看实测位置
                print(f"[spin] cmd_theta={theta_cmd:+.4f} act_theta={actual[IDX_THETA]:+.4f} "
                      f"err={theta_cmd - actual[IDX_THETA]:+.4f} | omega={omega:+.3f} "
                      f"| act_xy=({actual[IDX_X]:+.4f},{actual[IDX_Y]:+.4f}) "
                      f"drift={math.hypot(actual[IDX_X] - x0, actual[IDX_Y] - y0):.4f}")
            tick += 1

            # 到位判据：角度到 + 速度归零。之后进入 HOLD 继续重发，等 filter 收敛
            if hold_ticks == 0 and abs(remaining) < 1e-4 and abs(omega) < 1e-4:
                print(f"[spin] 指令到位，保持重发 {HOLD_SEC:.1f}s 等收敛")
                hold_ticks = 1
            if hold_ticks > 0:
                hold_ticks += 1
                if hold_ticks > int(HOLD_SEC * FREQ):
                    break

            rate.sleep()

    except KeyboardInterrupt:
        # 不能直接退出：停止重发会让正在转的底盘继续漂。先减速到 0 再走。
        print("\n[spin] 收到中断，减速停车（不要再按 Ctrl-C）")
        interrupted = True
        while abs(omega) > 1e-4:
            omega += max(-alpha * dt, min(alpha * dt, -omega))
            theta_cmd += omega * dt
            astribot.set_joints_position([chassis], [[x0, y0, theta_cmd]])
            rate.sleep()
        for _ in range(int(HOLD_SEC * FREQ)):
            astribot.set_joints_position([chassis], [[x0, y0, theta_cmd]])
            rate.sleep()

    actual = astribot.get_current_joints_position([chassis])[0]
    print(f"[spin] 结束 cmd_theta={theta_cmd:+.4f} act_theta={actual[IDX_THETA]:+.4f} "
          f"残差={theta_cmd - actual[IDX_THETA]:+.4f} rad | "
          f"xy 漂移={math.hypot(actual[IDX_X] - x0, actual[IDX_Y] - y0):.4f} m")


if __name__ == '__main__':
    main()
