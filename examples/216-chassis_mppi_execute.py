#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2026, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------

"""
File: 216-chassis_mppi_execute.py
Brief: MPPI 闭环驱动底盘**真实运动**。首次验证用，默认 5 cm 单轴。

!!! 这个脚本会让机器人动 !!!
================================================================================
它有两道闸，都必须过：

  闸 1 · 预检（自动，不可跳过）—— 运动控制面没起来就**拒绝运行**。
  闸 2 · 人工确认 —— 打印完整动作计划 + 当前手臂姿态，要求输入 `GO` 才继续。

任何一道不过，一条指令都不发。

================================================================================
为什么必须有闸 1：这套链路会以"看起来成功"的方式失败
================================================================================
实测（2026-08-28，10.249.22.137）机器人上运动控制面**没有起来**：
`/joint_space_states` 无发布、`/joint_space_command` 无发布、
`/astribot_safe_mode/state` 与 `/astribot/control_rights` 两个服务都不存在。

这种状态下三条链会合起来骗人：

  1. `get_robot_mode()` 在 safe_mode 服务缺失时 **`return "simulation"`**
     （astribot_interface.py:397）—— SDK 以为自己在跟仿真器说话，而不是报错。
  2. `wait_for_interface_alive(timeout_s=2)` 会返回 False，然后构造函数里
     `if self.is_alive:` 整块被跳过 —— **不抛异常**。
  3. `set_joints_position` 的守卫 `@check_control_rights_and_is_robot_alive`
     只查控制权与 `is_stopped()`，**不查 `is_alive()`**；失败时
     `return None` 只打日志（astribot_interface.py:191-202）。

结果：拿到控制权 -> 250 Hz 发满整趟指令 -> 一次异常都不抛 -> 机器人一动不动。
所以本脚本的预检**主动**查这几项，并且在运动过程中还有"实际位移看门狗"（见下）。

================================================================================
控制权的副作用（必须知道）
================================================================================
`acquire_control_rights` 在控制权服务**不存在**时走 except 分支，
直接 `have_control_rights = True` 并自己建那个服务（astribot_interface.py:293-301）
—— 也就是说构造 `Astribot()` 会**静默拿到控制权**，不弹提示。
反过来，若已有人持权，它会 `input()` 阻塞等你输 `yes`，而抢到手会
**立即停掉对方正在进行的运动**。

本脚本一律用 `high_control_rights=False`（默认），**绝不抢别人的控制权**；
拿不到就在闸 1 拒绝。

================================================================================
坐标系（已确认）
================================================================================
底盘是 3 个"关节" `[x, y, theta]`，走 `set_joints_position` 的**位置**通道。
* `theta` 是**绝对朝向**
* `x`/`y` 的增量按**本体系**累加  <- 已由使用者确认
本脚本**全程锁定 theta**（MPPI 的 wz 输出被强制归零），所以本体系恒等于启动时刻
那个系，"沿 +x 走"没有歧义。这同时也让上面那条口径即使记错也不影响本次结果。

================================================================================
三层运动保护
================================================================================
1. **实际位移看门狗**：前 WATCHDOG_GRACE_SEC 内若"指令位移已超过 X 而实际位移
   不足 X 的 WATCHDOG_MIN_RATIO"，判定指令没落地 -> 减速停车并报错退出。
   这一条专门打上面那个"静默不动"的失败模式。
2. **Leash（缰绳）**：|指令位置 − 实际位置| 超过 LEASH_M 立刻减速停车。
   底盘是**开环位置积分**：轮子打滑时指令会持续超前实际，误差单调累积，
   一旦恢复附着力机器人会以最大能力冲向那个跑飞的位置。Leash 是真的安全件。
3. **硬距离/速度上限**：距离 > MAX_DISTANCE_M 或速度 > MAX_CRUISE_MPS 直接拒绝
   （要更大必须显式改常量，不给命令行参数）。

Ctrl-C 走减速停车 + 保持重发，**不是直接退出**（停止重发会让还在动的底盘继续漂）。

================================================================================
用法
================================================================================
    source /opt/ros/humble/setup.bash
    cd ~/Downloads/astribot_sdk_aarch64 && source ./env.sh

    python3 examples/216-chassis_mppi_execute.py --check-only   # 只跑预检，绝不动
    python3 examples/216-chassis_mppi_execute.py                # 默认 +x 5 cm
    python3 examples/216-chassis_mppi_execute.py --distance 0.10
    python3 examples/216-chassis_mppi_execute.py --axis y

建议分级：**先 0.05，再 0.10，再 0.30**。每级都看完打印再决定下一级。
"""

import argparse
import importlib.util
import math
import os
import sys
import threading
import time

# ---------------- 硬上限（要更大必须改这里，不开放成命令行参数）----------------
MAX_DISTANCE_M = 0.30
MAX_CRUISE_MPS = 0.15

# ---------------- 默认动作（首次验证用的保守值）----------------
DEFAULT_DISTANCE_M = 0.05
DEFAULT_CRUISE_MPS = 0.06
DEFAULT_ACCEL = 0.25
STOP_TOL_M = 0.005         # 到位判据基准。再按行程缩放，见 effective_stop_tol()

# ---------------- 环路 ----------------
STREAM_HZ = 250.0          # 位置重发频率，与 examples/202 一致
# MPPI 更新频率与批量。**真机实测数字，不是估的**：
#   真机微基准(无 critics) 2000x56 -> 12.8 ms
#   x86 跑真 MPPI batch=400        -> p95  23 ms
#   真机跑真 MPPI batch=400        -> 均值 126 ms   <- 比 x86 慢 6.3 倍
# 126 ms 只够 8 Hz。所以默认降到 8 Hz + 小批量，并在启动时**实测**再自动降档。
MPPI_HZ = 8.0
# batch 上限从高往低标定。**上一轮我把它砍到 64 是砍过头了**：真机实测求解只要
# 6.3 ms（预算 75 ms），batch 完全开得起；而 batch=64 时 ESS 只有 9.3，
# 等于用 9 个有效样本在 44 步 x 3 自由度里找序列 —— MPPI 输出方向是噪声，
# 实测把底盘带偏到 -75.4°（横向走了 26 mm 而前向只有 7 mm）。
MPPI_BATCH = 1000
MPPI_BATCH_FLOOR = 64      # 降到这个还不够快就拒绝运行
MPPI_MIN_ESS_RATIO = 0.02  # ESS/batch 低于此值只告警，不中止（诊断用）
SOLVE_BUDGET_RATIO = 0.6   # 求解耗时不得超过 MPPI 周期的这个比例
HOLD_SEC = 3.0             # 到位后继续重发的时长，等 SDK filter 收敛

# ---------------- 保护 ----------------
LEASH_M = 0.06             # |指令 − 实际| 上限
# 看门狗的宽限期按**指令位移**而不是墙钟。用时间的话，0.06 m/s 巡航 1.5 s 能积出
# 0.09 m 指令位移 —— 超过 leash 上限 0.06，等于 leash 先触发、看门狗形同虚设。
# 干跑 F 实测：看门狗在指令 0.0597 m 才响，leash 峰值 0.0597，差 0.0003 就被抢先。
# 看门狗需要**两个条件同时**成立。只用时间会被 leash 抢先（0.06 m/s 跑 1.5 s 积出
# 0.09 m 指令位移 > leash 上限 0.06）；只用位移又会过早触发——实测 t=0.14s 就判
# "指令没落地"，而 0.14 s 内任何底盘都走不完 10 mm，SDK filter 本身滞后就有 0.1~0.3 s。
WATCHDOG_START_CMD_M = 0.010   # 条件一：指令位移超过它
WATCHDOG_MIN_ELAPSED_SEC = 0.6 # 条件二：已过最小响应时间
WATCHDOG_MIN_RATIO = 0.25      # 实际位移至少要有指令位移的这个比例
# 过冲保护：打滑期间指令跑飞、恢复附着力后实际会冲过目标，而那一刻 leash 反而是
# 收敛的。干跑 G 实测：过冲 27% 却判成功、leash 峰值只有 0.0154 没触发。
# 只用比例会在短行程上误杀：1.15x 对 5 cm 只有 7.5 mm 余量，比停车滑行本身还小
# （干跑 H 实测正常路径被误判）。所以取"比例余量"与"绝对余量"的**较大者**。
OVERSHOOT_RATIO = 1.15
OVERSHOOT_ABS_M = 0.025
# 方向守卫。**这是上一轮设计缺陷的补丁**：原先写的是"MPPI 只负责方向和名义速度，
# 能不能停得住由几何层独立保证"，但几何层只管幅值不管方向 —— MPPI 一旦给出错方向，
# leash 和过冲保护都不会响（它们都不看方向）。实测代价：横向 26 mm、前向 7 mm，
# 底盘忠实跟随了一个 -75.4° 的错误指令。
# 现在把指令方向强制投影进"偏离指向目标不超过 MAX_DIR_DEV"的锥内，保幅值不变。
MAX_DIR_DEV_RAD = math.radians(20.0)
# 设定值超龄归零。250 Hz 流式环会一直拿 MPPI 上次给的速度积分，MPPI 线程一慢或一死
# 就照旧速度冲下去（干跑 H/J 实测冲到目标的 150%）。3 个 MPPI 周期没更新就当 0。
MAX_SETPOINT_AGE_SEC = 3.0 / MPPI_HZ
# 起步自检：实测达成的重发频率偏离标称超过这个比例就中止。
# 这次实测跑到了约 6500 Hz（标称 250），26 倍——这个自检本该在 0.5 s 内抓到它。
STREAM_RATE_TOLERANCE = 0.20
STREAM_RATE_CHECK_SEC = 0.5
RUN_TIMEOUT_SEC = 30.0

IDX_X, IDX_Y, IDX_THETA = 0, 1, 2
CONFIRM_WORD = 'GO'


def load_mppi_module():
    """从 215 加载 MPPI 实现，**不复制一份**。

    215 的文件名以数字开头，不能直接 import，所以用 importlib 按路径加载。
    两处各写一份 MPPI 的后果是"离线验过的律"和"真机跑的律"悄悄分叉。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, '215-chassis_track_mppi.py')
    if not os.path.exists(path):
        raise FileNotFoundError(
            f'找不到 {path}。216 复用 215 的 MPPI 实现，两个文件必须在同一目录。')
    spec = importlib.util.spec_from_file_location('mppi215', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Preflight:
    """闸 1。任何一项不过都拒绝运行。"""

    def __init__(self):
        self.problems = []
        self.info = {}

    def fail(self, msg):
        self.problems.append(msg)

    def run(self, astribot):
        # 1. 控制权：必须真的握在手里。绝不用 high_control_rights 去抢。
        rights = astribot.get_control_rights_status()
        self.info['control_rights'] = rights
        if not rights:
            self.fail('没有控制权。本脚本不抢别人的控制权（抢会立即停掉对方的运动）。'
                      '先确认没有其它程序在控制机器人。')

        # 2. interface 必须 alive。这是"状态话题有没有数据"的等价判据。
        alive = bool(getattr(astribot, 'is_alive', False))
        self.info['is_alive'] = alive
        if not alive:
            self.fail('interface 不 alive —— /joint_space_states 没有数据，'
                      '运动控制面没起来。判据：'
                      '`ros2 topic hz /joint_space_states` 必须有稳定频率。')

        # 3. robot_mode 不能是 "simulation"。真机上出现它，含义是
        #    /astribot_safe_mode/state 服务缺失（见文件头），不是"真的在仿真"。
        try:
            mode = astribot.astribot_interface.get_robot_mode()
        except Exception as exc:                                  # noqa: BLE001
            mode = f'<查询失败 {type(exc).__name__}>'
        self.info['robot_mode'] = mode
        if mode == 'simulation':
            self.fail('robot_mode = "simulation"。在真机上这说明 '
                      '/astribot_safe_mode/state 服务不存在（SDK 在服务缺失时'
                      '直接 return "simulation"，不报错）。判据：'
                      '`ros2 service list | grep safe_mode` 必须出现。')

        # 4. 底盘自由度必须是 3，且状态读得到
        try:
            q = astribot.get_current_joints_position([astribot.chassis_name])[0]
            self.info['chassis_actual'] = list(q)
            if len(q) != 3:
                self.fail(f'底盘自由度 {len(q)}，期望 3。ROBOT_TYPE 没设成 S1 时是 2，'
                          f'本脚本不适用。')
        except Exception as exc:                                  # noqa: BLE001
            self.fail(f'读不到底盘实际位置（{type(exc).__name__}: {exc}）')

        # 5. desired 也要读得到（积分种子取它，与 202/203 一致）
        try:
            d = astribot.get_desired_joints_position([astribot.chassis_name])[0]
            self.info['chassis_desired'] = list(d)
        except Exception as exc:                                  # noqa: BLE001
            self.fail(f'读不到底盘 desired 位置（{type(exc).__name__}: {exc}）')

        # 6. 手臂姿态：读出来给人看。展开的手臂既危险也会被耦合限速。
        for name_attr in ('arm_left_name', 'arm_right_name'):
            nm = getattr(astribot, name_attr, None)
            if nm is None:
                continue
            try:
                self.info[nm] = [round(v, 4) for v in
                                 astribot.get_current_joints_position([nm])[0]]
            except Exception as exc:                              # noqa: BLE001
                self.info[nm] = f'<读取失败 {type(exc).__name__}>'

        return not self.problems


class Pacer:
    """显式单调时钟节拍器。**刻意不用 mw.Rate**。

    实测 `mw.Rate(250).sleep()` 没有真的限速：主环跑到约 6500 Hz（标称 250 的 26 倍），
    SDK 自己都在报 "The calling frequency of the set function does not match the
    robot setting frequency"。闭源中间件里为什么不生效我没查清，但节拍是本脚本
    时序正确性的地基，不能建在一个行为不确定的调用上 —— 所以自己算 deadline。

    落后超过一个周期时重新对齐，不累积欠债（否则一次卡顿会换来之后一串零延迟发送，
    等于把突发流量甩给机器人）。
    """

    def __init__(self, hz):
        if hz <= 0.0:
            raise ValueError('hz 必须 > 0')
        self.dt = 1.0 / hz
        self.hz = hz
        self._next = None
        self.ticks = 0
        self.late = 0

    def sleep(self):
        now = time.monotonic()
        if self._next is None:
            self._next = now + self.dt
        else:
            self._next += self.dt
        slack = self._next - now
        if slack > 0.0:
            time.sleep(slack)
        elif slack < -self.dt:
            self.late += 1
            self._next = time.monotonic() + self.dt      # 重新对齐，不补欠债
        self.ticks += 1


def effective_stop_tol(distance):
    """到位判据随行程缩放，不超过行程的 10%。

    固定容差在短行程上会吃掉大比例位移，让"误差"这个数主要反映容差而不是跟踪
    —— 与 nav2 的 xy_goal_tolerance 0.18 吃掉 30 cm 任务 60% 行程是同一个错。
    """
    return min(STOP_TOL_M, 0.10 * abs(distance))


def brake_speed_limit(remaining, v_max, accel, dt):
    """**纯钳位**：返回"本周期走完后仍刹得住"的速度上限。

        v^2/(2a) + v*dt <= |s|  ->  v <= -a*dt + sqrt((a*dt)^2 + 2*a*|s|)

    !!! 签名里刻意没有 v_now !!!
    上一版的 brake_capped_speed 同时做了钳位**和**斜率限制，返回的是"朝上限收敛一步"
    的值。我在两处把它当钳位用：传 speed=0.103 进去它返回 0.1020（每周期只压
    accel*dt=0.001，要 43 个周期才压到 0.06），于是"叠一层几何刹车上限"这句话
    实际上什么都没限 —— 真机实测合成速度 0.103 m/s，是 cruise 上限的 1.72 倍。
    去掉 v_now 参数，结构上保证它只能当钳位用。斜率限制由调用方单独做。
    """
    if accel <= 0.0 or dt <= 0.0:
        raise ValueError('accel/dt 必须 > 0')
    adt = accel * dt
    return min(v_max, max(0.0, -adt + math.sqrt(adt * adt + 2.0 * accel * abs(remaining))))


def direction_guard(vx, vy, gx, gy, max_dev_rad=MAX_DIR_DEV_RAD):
    """把速度方向投影进"偏离指向目标不超过 max_dev_rad"的锥内，**幅值不变**。

    返回 (vx, vy, 原始偏差rad)。见 MAX_DIR_DEV_RAD 处的说明。
    """
    sp = math.hypot(vx, vy)
    if sp < 1e-9:
        return 0.0, 0.0, 0.0
    gn = math.hypot(gx, gy)
    if gn < 1e-9:
        return vx, vy, 0.0
    want = math.atan2(gy, gx)
    dev = math.atan2(math.sin(math.atan2(vy, vx) - want),
                     math.cos(math.atan2(vy, vx) - want))
    if abs(dev) <= max_dev_rad:
        return vx, vy, dev
    a = want + math.copysign(max_dev_rad, dev)
    return sp * math.cos(a), sp * math.sin(a), dev


class MppiWorker(threading.Thread):
    """MPPI 在独立线程里以 MPPI_HZ 求解，只产出一个本体系速度设定值。

    为什么不放进 250 Hz 主环：一次求解约 23 ms（batch=400，真机实测），
    塞进主环会造成 6 个周期的重发空档。重发必须连续。
    """

    def __init__(self, mppi_mod, path, goal, cruise, accel, get_pose, stop_tol,
                 batch=MPPI_BATCH, v_ref=None):
        super().__init__(daemon=True)
        self.batch = batch
        self.mod = mppi_mod
        self.path = path
        self.goal = goal
        self.cruise = cruise
        self.accel = accel
        self.get_pose = get_pose
        self.stop_tol = stop_tol
        self.mppi = mppi_mod.MPPI(batch=batch, horizon=mppi_mod.TIME_STEPS,
                                  dt=mppi_mod.MODEL_DT, accel_constraint='box',
                                  shrink_horizon=True,
                                  v_ref=v_ref if v_ref else max(0.05, cruise))
        self._lock = threading.Lock()
        self._vel = (0.0, 0.0)          # 只有 (vx, vy)；wz 恒 0，theta 锁定
        self._vel_stamp = 0.0           # 设定值产生时刻，供超龄判断
        self._running = True
        self.solve_times = []
        self.ess = []
        self.iterations = 0
        self.max_dir_dev = 0.0
        self.dir_clamped = 0

    @property
    def velocity(self):
        """返回 (vx, vy, 设定值年龄秒)。年龄由调用方判断是否超龄。"""
        with self._lock:
            age = time.monotonic() - self._vel_stamp if self._vel_stamp else 1e9
            return self._vel[0], self._vel[1], age

    def stop(self):
        self._running = False

    def run(self):
        dt = 1.0 / MPPI_HZ
        while self._running:
            t0 = time.monotonic()
            x, y, yaw = self.get_pose()
            remaining = math.hypot(self.goal[0] - x, self.goal[1] - y)
            if remaining <= self.stop_tol:
                with self._lock:
                    self._vel = (0.0, 0.0)
                    self._vel_stamp = time.monotonic()
            else:
                vx, vy, _wz = self.mppi.solve((x, y, yaw), self.path, self.goal)
                # 方向守卫：先把方向投影进锥内（幅值不变），再钳幅值。
                # 顺序要紧：先钳幅值再改方向会让幅值又超出。
                vx, vy, dev = direction_guard(vx, vy, self.goal[0] - x,
                                              self.goal[1] - y)
                self.max_dir_dev = max(self.max_dir_dev, abs(dev))
                if abs(dev) > MAX_DIR_DEV_RAD:
                    self.dir_clamped += 1
                # 纯钳位（不是斜率限制）：本周期走完后仍要刹得住
                speed = math.hypot(vx, vy)
                cap = brake_speed_limit(remaining, self.cruise, self.accel, dt)
                if speed > 1e-9 and speed > cap:
                    vx, vy = vx * cap / speed, vy * cap / speed
                with self._lock:
                    self._vel = (vx, vy)
                    self._vel_stamp = time.monotonic()
                self.solve_times.append(self.mppi.last_solve_sec)
                self.ess.append(self.mppi.last_ess)
                self.iterations += 1
            slack = dt - (time.monotonic() - t0)
            if slack > 0:
                time.sleep(slack)


def horizon_v_ref(distance, mppi_mod=None):
    """给 effective_horizon 用的参考速度。**不能直接用 cruise。**

    effective_horizon 算的是 need = remaining / (v_ref * model_dt)。若 v_ref 取巡航
    速度 0.06，need = 0.30/(0.06*0.05) = 100 步 > TIME_STEPS 56 -> 被截到 56，
    **视野收缩完全失效、恒跑满 56 步**。这就是上次真机求解 126 ms 的根因：
    我以为在用收缩视野，其实一直是满视野。

    正确取法：让 need 落进 TIME_STEPS 之内，即 v_ref >= remaining/(T_max*model_dt)，
    这里取该下限的 1.5 倍留余量。v_ref 只决定"视野多长"，不决定下发速度
    （下发速度由 cruise 与 250 Hz 几何刹车层决定），所以放大它是安全的。
    """
    dt = 0.05 if mppi_mod is None else mppi_mod.MODEL_DT
    t_max = 56 if mppi_mod is None else mppi_mod.TIME_STEPS
    return max(0.05, 1.5 * abs(distance) / (t_max * dt))


def calibrate_mppi_batch(mppi_mod, path, goal, pose, cruise, v_ref):
    """实测求解耗时并自动选一个跑得动的 batch。

    为什么必须实测：我用 x86 上的 p95 23 ms 估真机，实际是 126 ms（慢 6.3 倍），
    直接导致上一次运行 MPPI 整趟只解了 2 次、设定值 92% 超龄、机器人几乎没动。
    估算在这里不可用，只能测。

    返回 (batch, 实测p95秒) 或 (None, p95) 表示降到底仍不达标。
    """
    budget = SOLVE_BUDGET_RATIO / MPPI_HZ
    batch = MPPI_BATCH
    while batch >= MPPI_BATCH_FLOOR:
        probe = mppi_mod.MPPI(batch=batch, horizon=mppi_mod.TIME_STEPS,
                              dt=mppi_mod.MODEL_DT, accel_constraint='box',
                              shrink_horizon=True, v_ref=v_ref)
        ts = []
        for _ in range(4):
            t0 = time.perf_counter()
            probe.solve(pose, path, goal)
            ts.append(time.perf_counter() - t0)
        ts.sort()
        p95 = ts[-1]                     # 只 4 次，取最差当 p95
        print(f'  batch={batch:5d} -> 最差 {p95 * 1000:6.1f} ms '
              f'(预算 {budget * 1000:.0f} ms @ {MPPI_HZ:g} Hz) '
              f'{"够用" if p95 <= budget else "超预算，降档"}')
        if p95 <= budget:
            return batch, p95
        batch //= 2
    return None, p95


def confirm(plan_lines):
    """闸 2。必须精确输入 CONFIRM_WORD。"""
    print('\n' + '!' * 78)
    print('!! 下面这个动作会让机器人真的移动 !!')
    print('!' * 78)
    for ln in plan_lines:
        print('  ' + ln)
    print('!' * 78)
    print(f'确认周围空旷、手臂已收好、随手可按急停。')
    print(f'输入 {CONFIRM_WORD} 开始，其它任何输入都取消：', end='', flush=True)
    try:
        ans = sys.stdin.readline()
    except (EOFError, KeyboardInterrupt):
        print('\n已取消。')
        return False
    if ans.strip() != CONFIRM_WORD:
        print('已取消（未输入 %s）。' % CONFIRM_WORD)
        return False
    return True


def main():
    ap = argparse.ArgumentParser(
        description='MPPI 闭环驱动底盘真实运动（两道闸：预检 + 人工确认）')
    ap.add_argument('--axis', choices=('x', 'y'), default='x',
                    help='沿本体系哪个轴走。首次验证只做单轴：theta 锁定时'
                         '两种坐标系口径重合，不受口径歧义影响')
    ap.add_argument('--distance', type=float, default=DEFAULT_DISTANCE_M,
                    help=f'位移 (m)，可为负。硬上限 {MAX_DISTANCE_M}')
    ap.add_argument('--cruise', type=float, default=DEFAULT_CRUISE_MPS,
                    help=f'巡航速度 (m/s)。硬上限 {MAX_CRUISE_MPS}')
    ap.add_argument('--accel', type=float, default=DEFAULT_ACCEL, help='加减速 (m/s^2)')
    ap.add_argument('--check-only', action='store_true',
                    help='只跑预检并打印状态，**绝不发任何指令**')
    args = ap.parse_args()

    if abs(args.distance) > MAX_DISTANCE_M:
        print(f'拒绝：距离 {args.distance} 超过硬上限 {MAX_DISTANCE_M} m。'
              f'要更大请改脚本里的 MAX_DISTANCE_M 常量。')
        return 2
    if args.cruise > MAX_CRUISE_MPS or args.cruise <= 0:
        print(f'拒绝：速度 {args.cruise} 超出 (0, {MAX_CRUISE_MPS}] m/s。')
        return 2

    mppi_mod = load_mppi_module()
    from astribot_sdk.core.astribot_api.astribot_client import Astribot
    import astribot_ros_middleware as mw

    print('=' * 78)
    print('闸 1 · 预检')
    print('=' * 78)
    # high_control_rights=False：绝不抢别人的控制权
    astribot = Astribot(freq=STREAM_HZ, high_control_rights=False)
    pacer = Pacer(STREAM_HZ)

    # spin 必须独立线程，否则 get_* 拿到的是陈旧值（examples/202 同样处理）
    threading.Thread(target=_spin_forever, args=(mw,), daemon=True).start()

    pf = Preflight()
    ok = pf.run(astribot)
    for k, v in pf.info.items():
        print(f'  {k:22s} = {v}')
    if not ok:
        print('\n预检未通过：')
        for i, p in enumerate(pf.problems, 1):
            print(f'  {i}. {p}')
        print('\n一条指令都没发。修好上面的问题再来。')
        return 2
    print('  预检通过。')

    if args.check_only:
        print('\n--check-only：到此为止，未发任何指令。')
        return 0

    # ---- 规划 ----
    seed = list(astribot.get_desired_joints_position([astribot.chassis_name])[0])
    act0 = list(astribot.get_current_joints_position([astribot.chassis_name])[0])
    dx = args.distance if args.axis == 'x' else 0.0
    dy = args.distance if args.axis == 'y' else 0.0
    # 路径与目标在"启动时刻的本体系"里表达。theta 全程锁定 -> 该系不随时间变化。
    goal = (act0[IDX_X] + dx, act0[IDX_Y] + dy)
    path = mppi_mod.straight_path(act0[IDX_X], act0[IDX_Y], goal[0], goal[1])
    theta_lock = seed[IDX_THETA]
    brake_dist = args.cruise ** 2 / (2.0 * args.accel)
    stop_tol = effective_stop_tol(args.distance)
    print('\n标定 MPPI 求解耗时（真机上必须实测，估算不可用）...')
    mppi_batch, solve_p95 = calibrate_mppi_batch(
        mppi_mod, path, goal, (act0[IDX_X], act0[IDX_Y], act0[IDX_THETA]),
        args.cruise, horizon_v_ref(args.distance, mppi_mod))
    if mppi_batch is None:
        print(f'\n拒绝运行：即使降到 batch={MPPI_BATCH_FLOOR} 求解仍需 '
              f'{solve_p95 * 1000:.0f} ms > 预算 {SOLVE_BUDGET_RATIO / MPPI_HZ * 1000:.0f} ms。'
              f'\n           MPPI 跟不上 {MPPI_HZ:g} Hz，设定值会长期超龄、底盘走不动'
              f'（上次实测 92% 超龄、只走了 1.7 mm）。'
              f'\n           一条指令都没发。')
        return 2
    overshoot_limit = abs(args.distance) + max(
        OVERSHOOT_ABS_M, (OVERSHOOT_RATIO - 1.0) * abs(args.distance))

    plan = [
        f'动作      沿本体系 {args.axis} 轴走 {args.distance:+.3f} m（单轴，theta 锁定）',
        f'巡航速度  {args.cruise:.3f} m/s   加减速 {args.accel:.2f} m/s^2',
        f'制动距离  {brake_dist:.4f} m（占行程 {brake_dist / abs(args.distance) * 100:.0f}%）',
        f'预计耗时  约 {abs(args.distance) / args.cruise + 2 * args.cruise / args.accel:.1f} s'
        f' + 到位保持 {HOLD_SEC:.0f} s',
        f'控制律    MPPI（batch={mppi_batch} @ {MPPI_HZ:g} Hz，实测最差 '
        f'{solve_p95 * 1000:.0f} ms），叠一层几何刹车上限',
        f'当前实际  x={act0[0]:+.4f} y={act0[1]:+.4f} theta={act0[2]:+.4f}',
        f'目标      x={goal[0]:+.4f} y={goal[1]:+.4f}（theta 保持 {theta_lock:+.4f}）',
        f'到位判据  {stop_tol:.4f} m（占行程 {stop_tol / abs(args.distance) * 100:.0f}%）',
        f'保护      leash {LEASH_M} m · 位移看门狗 · 过冲上限 '
        f'{overshoot_limit:.4f} m · 超时 {RUN_TIMEOUT_SEC:.0f}s',
        f'手臂      {pf.info.get(getattr(astribot, "arm_left_name", ""), "?")} /'
        f' {pf.info.get(getattr(astribot, "arm_right_name", ""), "?")}',
    ]
    print()
    print('=' * 78)
    print('闸 2 · 人工确认')
    print('=' * 78)
    if not confirm(plan):
        print('一条指令都没发。')
        return 1

    # ---- 执行 ----
    pos_cmd = [seed[IDX_X], seed[IDX_Y], theta_lock]
    state = {'pose': (act0[IDX_X], act0[IDX_Y], act0[IDX_THETA])}
    state_lock = threading.Lock()

    def get_pose():
        with state_lock:
            return state['pose']

    worker = MppiWorker(mppi_mod, path, goal, args.cruise, args.accel,
                        get_pose, stop_tol, batch=mppi_batch,
                        v_ref=horizon_v_ref(args.distance, mppi_mod))
    worker.start()

    dt = 1.0 / STREAM_HZ
    t0 = time.monotonic()
    abort = None
    tick = 0
    print_every = int(STREAM_HZ / 5.0)
    max_leash = 0.0
    stale_ticks = 0
    rate_checked = False
    measured_hz = 0.0
    main_dir_clamped = 0
    max_cmd_speed = 0.0

    try:
        while True:
            act = astribot.get_current_joints_position([astribot.chassis_name])[0]
            with state_lock:
                state['pose'] = (act[IDX_X], act[IDX_Y], act[IDX_THETA])

            elapsed = time.monotonic() - t0
            cmd_moved = math.hypot(pos_cmd[IDX_X] - seed[IDX_X],
                                   pos_cmd[IDX_Y] - seed[IDX_Y])
            # 沿目标轴的位移（不是总位移）。用总位移会把"走错方向"算成"走到了"。
            ux = 1.0 if args.axis == 'x' else 0.0
            uy = 0.0 if args.axis == 'x' else 1.0
            sgn = 1.0 if args.distance >= 0 else -1.0
            act_moved = ((act[IDX_X] - act0[IDX_X]) * ux
                         + (act[IDX_Y] - act0[IDX_Y]) * uy) * sgn
            total_moved = math.hypot(act[IDX_X] - act0[IDX_X],
                                     act[IDX_Y] - act0[IDX_Y])
            leash = math.hypot(pos_cmd[IDX_X] - act[IDX_X], pos_cmd[IDX_Y] - act[IDX_Y])
            max_leash = max(max_leash, leash)
            remaining = math.hypot(goal[0] - act[IDX_X], goal[1] - act[IDX_Y])

            # 起步频率自检：节拍错了后面全错，必须在 0.5 s 内抓到。
            if not rate_checked and elapsed >= STREAM_RATE_CHECK_SEC:
                rate_checked = True
                achieved = tick / elapsed if elapsed > 0 else 0.0
                measured_hz = achieved
                if abs(achieved - STREAM_HZ) / STREAM_HZ > STREAM_RATE_TOLERANCE:
                    abort = (f'重发频率自检失败：实测 {achieved:.0f} Hz，标称 '
                             f'{STREAM_HZ:g} Hz，偏离超过 '
                             f'{STREAM_RATE_TOLERANCE:.0%}。节拍不对时 MPPI 线程会被'
                             f'饿死、设定值长期超龄，底盘走不动。')
                    break
            if leash > LEASH_M:
                abort = (f'LEASH 触发：|指令−实际| = {leash:.4f} > {LEASH_M} m。'
                         f'底盘是开环位置积分，误差会单调累积，恢复附着力时会猛冲。')
                break
            if (cmd_moved > WATCHDOG_START_CMD_M
                    and elapsed > WATCHDOG_MIN_ELAPSED_SEC
                    and act_moved < WATCHDOG_MIN_RATIO * cmd_moved):
                # 诊断分流：上一轮它报"指令没落地"，而实测是落地了、走错了方向
                # （实际总位移 27.0 mm ≈ 指令 28.2 mm，只是方向 -75.4°）。
                total_act = total_moved
                if total_act > 0.5 * cmd_moved:
                    why = (f'指令**落地了但方向不对**：实际总位移 {total_act:.4f} m '
                           f'≈ 指令 {cmd_moved:.4f} m，但沿目标轴只有 {act_moved:.4f} m。'
                           f'查 MPPI 输出方向与 ESS。')
                else:
                    why = ('指令**没落地**：实际总位移也远小于指令 —— '
                           '查 is_alive / 控制权 / 运动面。')
                abort = (f'看门狗触发：指令已走 {cmd_moved:.4f} m 而沿轴只走 '
                         f'{act_moved:.4f} m（< {WATCHDOG_MIN_RATIO:.0%}）。' + why)
                break
            # 过冲用**总位移**：走反方向也要被拦住（沿轴投影为负时不会触发）
            if total_moved > overshoot_limit:
                abort = (f'过冲保护触发：实际已走 {act_moved:.4f} m > 上限 '
                         f'{overshoot_limit:.4f}（目标 {abs(args.distance):.4f}）。'
                         f'典型成因是打滑期间指令跑飞、恢复附着力后猛冲。')
                break
            if elapsed > RUN_TIMEOUT_SEC:
                abort = f'超时 {RUN_TIMEOUT_SEC:.0f}s 未到位。'
                break
            if remaining <= stop_tol:
                print(f'\n到位：实际剩余 {remaining:.4f} m <= {stop_tol:.4f}')
                break

            vx, vy, age = worker.velocity
            if age > MAX_SETPOINT_AGE_SEC:
                # MPPI 设定值超龄：当 0，不要拿旧速度继续积分
                vx = vy = 0.0
                stale_ticks += 1
            # !!! 关键安全层 !!! MPPI 只定方向与名义速度；能不能停得住由这里
            # 按**实测剩余距离**独立判定，不依赖 critics 的调参，也不依赖 MPPI 及时刷新。
            vx, vy, dev = direction_guard(vx, vy, goal[0] - act[IDX_X],
                                          goal[1] - act[IDX_Y])
            if abs(dev) > MAX_DIR_DEV_RAD:
                main_dir_clamped += 1
            speed = math.hypot(vx, vy)
            cap = brake_speed_limit(remaining, args.cruise, args.accel, dt)
            if speed > 1e-9 and speed > cap:
                vx, vy = vx * cap / speed, vy * cap / speed
            max_cmd_speed = max(max_cmd_speed, math.hypot(vx, vy))
            pos_cmd[IDX_X] += vx * dt
            pos_cmd[IDX_Y] += vy * dt
            pos_cmd[IDX_THETA] = theta_lock          # 锁定，绝不累加 wz
            astribot.set_joints_position([astribot.chassis_name], [list(pos_cmd)])

            if tick % print_every == 0:
                print(f'  t={elapsed:5.2f}s 指令 {cmd_moved:.4f} 实际 {act_moved:.4f} '
                      f'剩余 {remaining:.4f} leash {leash:.4f} '
                      f'v=({vx:+.3f},{vy:+.3f}) age={age * 1000:4.0f}ms dtheta='
                      f'{act[IDX_THETA] - act0[IDX_THETA]:+.4f}')
            tick += 1
            pacer.sleep()

    except KeyboardInterrupt:
        abort = '收到 Ctrl-C'

    # ---- 减速停车 + 保持 ----
    worker.stop()
    # 注意：这里不再推进 pos_cmd —— worker 已停，速度设定值不再更新。
    # 持续重发**当前**指令位置就是减速停车：SDK 的 filter 会把实际收敛到它。
    # 停止重发才是危险的（还在动的底盘会继续漂）。
    print('\n减速停车并保持重发（不要按 Ctrl-C）...')
    for _ in range(int((1.0 + HOLD_SEC) * STREAM_HZ)):
        astribot.set_joints_position([astribot.chassis_name], [list(pos_cmd)])
        pacer.sleep()

    end = astribot.get_current_joints_position([astribot.chassis_name])[0]
    ax = end[IDX_X] - act0[IDX_X]
    ay = end[IDX_Y] - act0[IDX_Y]
    along = ax if args.axis == 'x' else ay
    perp = ay if args.axis == 'x' else ax
    print('\n' + '=' * 78)
    print('结果（实测，不是指令回显）')
    print('=' * 78)
    print(f'  目标位移      {args.distance:+.4f} m')
    err = along - args.distance
    print(f'  实测沿向      {along:+.4f} m   偏差 {err * 1000:+.1f} mm')
    print(f'                到位判据允许 {stop_tol * 1000:.1f} mm；超出容差的 '
          f'{max(0.0, abs(err) - stop_tol) * 1000:.1f} mm 才是真误差')
    print(f'  实测垂向      {perp * 1000:+.1f} mm     <- 应接近 0')
    print(f'  偏航变化      {math.degrees(end[IDX_THETA] - act0[IDX_THETA]):+.2f}°'
          f'   <- theta 锁定，应接近 0')
    print(f'  指令累计      {math.hypot(pos_cmd[0] - seed[0], pos_cmd[1] - seed[1]):.4f} m')
    print(f'  leash 峰值    {max_leash:.4f} m（上限 {LEASH_M}）')
    print(f'  指令速度峰值  {max_cmd_speed:.4f} m/s（cruise 上限 {args.cruise:.3f}）'
          + ('   <- **超过上限，钳位失效**' if max_cmd_speed > args.cruise * 1.02
             else '   <- 未超上限，钳位有效'))
    print(f'  方向偏差峰值  {math.degrees(worker.max_dir_dev):.1f}°'
          f'（守卫阈值 {math.degrees(MAX_DIR_DEV_RAD):.0f}°）'
          f'，被钳 worker {worker.dir_clamped} 次 / 主环 {main_dir_clamped} 次')
    if worker.ess:
        er = (sum(worker.ess) / len(worker.ess)) / max(1, mppi_batch)
        print(f'  ESS/batch     {er:.1%}'
              + ('   <- 偏低，MPPI 方向可信度差' if er < MPPI_MIN_ESS_RATIO * 3 else ''))
    print(f'  重发频率      实测 {measured_hz:.0f} Hz（标称 {STREAM_HZ:g}）'
          f'，节拍迟到 {pacer.late} 次')
    print(f'  设定值超龄    {stale_ticks} / {tick} 个周期'
          f'（超龄即按 0 处理，>0 说明 MPPI 跟不上 {MPPI_HZ:g} Hz）')
    if worker.solve_times:
        st = sorted(worker.solve_times)
        print(f'  MPPI 迭代     {worker.iterations} 次，求解均值 '
              f'{sum(st) / len(st) * 1000:.1f} ms，p95 '
              f'{st[min(len(st) - 1, int(0.95 * (len(st) - 1)))] * 1000:.1f} ms')
        print(f'  softmax ESS   均值 {sum(worker.ess) / len(worker.ess):.1f} / {MPPI_BATCH}')
    if abort:
        print(f'\n  !! 异常结束：{abort}')
        return 1
    print('\n  正常结束。下一级再加距离前，先确认垂向与偏航都接近 0。')
    return 0


def _spin_forever(mw):
    """spin 线程。**必须节流**，否则它会把 GIL 占满。

    examples/202 里是 `while ok(): spin()` 无节流死循环。当 spin() 本身阻塞等消息时
    那样没问题，但一旦它快速返回，这个循环就变成纯 CPU 自旋，把同进程的 MPPI 线程
    和主环一起饿死 —— 实测代价：同一份 MPPI 输入，单独调用 4.4 ms，
    放进有 spin 线程的进程里变成 2296 ms（**520 倍**）。
    上一次真机跑出的"求解 126 ms、整趟只解 2 次、设定值 92% 超龄"就是这个原因。

    加一个 1 ms 让渡：对 250 Hz 的状态更新完全够用，同时把 GIL 交出去。
    """
    while mw.ok():
        mw.spin()
        time.sleep(0.001)


if __name__ == '__main__':
    sys.exit(main())
