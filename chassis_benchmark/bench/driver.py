#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘指令下发抽象。

两个实现对应两条完全不同的链路：

* ``SimCmdVelDriver`` —— 仿真。发 ``geometry_msgs/Twist`` 到 ``/cmd_vel``，
  **车体系**（``VelocityControl``/``MecanumDrive`` 已整体移除，现在是
  ``omni_effort_drive_node`` 的全向轮逆解直接吃 ``/cmd_vel``）。
  必须**持续发流**：该节点 ``cmd_vel_timeout_sec: 0.5``，发一条不会持续运动。

* ``SdkPositionDriver`` —— 真机。底盘走的是**位置**通道
  （``RobotJointController.msg`` 只有 header/mode/name/command，线上没有速度字段），
  速度由上层积分，做法与 ``examples/202-chassis_joy_control_local.py`` 一致。
  也必须持续重发：发一次抓不住正在运动的关节（实测漂 0.045~0.37 rad）。

两者都提供同一个 ``ramped_move()``：带"刹得住"速度规划的定量运动。
用同一套规划保证 sim/real 的数字可比 —— 否则加减速曲线不同，
停车距离和到位误差都没法横向对照。
"""

import math
import threading
import time

from geometry_msgs.msg import Twist


def brake_in_time_profile(remaining, v_max, accel, v_now, dt, freeze=False):
    """由剩余量反推本周期应该用的速度，**离散安全**。

    天真的写法是 ``v = sqrt(2*a*|s|)``。那条曲线要求的减速度恰好等于 a
    （``dv/dt = (a/v)*v = a``），一步都不能落后；而按**当前**剩余量算上限再限幅，
    等于永远晚一个周期，结果是会过冲。对开环积分位置的底盘来说，过冲意味着
    指令位置要往回收 —— 正是漂移累积的来源。这个缺陷是 tests 里
    ``test_profile_brakes_so_it_can_stop_in_the_remaining_distance`` 抓出来的。

    所以这里用离散安全的形式：要求"本周期走完之后仍然刹得住"，即

        v^2/(2a) + v*dt <= |s|   ->   v <= -a*dt + sqrt((a*dt)^2 + 2*a*|s|)

    再对 v 做加速度限幅。这样天然覆盖加速/巡航/减速三段，
    距离短时自动退化成三角形，不需要写状态机。

    freeze=True 时目标速度强制为 0（用于中断后只减速不前进）。
    返回新的速度（带符号，符号跟 remaining 一致）。
    """
    if accel <= 0.0:
        raise ValueError('accel 必须 > 0')
    if dt <= 0.0:
        raise ValueError('dt 必须 > 0')
    if freeze:
        cap = 0.0
    else:
        adt = accel * dt
        brake_cap = -adt + math.sqrt(adt * adt + 2.0 * accel * abs(remaining))
        cap = min(v_max, max(0.0, brake_cap))
    target = math.copysign(cap, remaining) if remaining != 0.0 else 0.0
    step = accel * dt
    return v_now + max(-step, min(step, target - v_now))


class _StreamingDriver:
    """共用的"后台线程持续重发"骨架。子类实现 _send_once()。"""

    def __init__(self, rate_hz):
        if rate_hz <= 0.0:
            raise ValueError('rate_hz 必须 > 0')
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._sent = 0
        self._late = 0          # 超过 1.5 个周期才发出的次数：调度抖动指纹

    # ---- 子类实现 ----
    def _send_once(self):
        raise NotImplementedError

    def _on_stop(self):
        pass

    # ---- 生命周期 ----
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        next_t = time.monotonic()
        while self._running:
            self._send_once()
            self._sent += 1
            next_t += self.dt
            slack = next_t - time.monotonic()
            if slack < -0.5 * self.dt:
                self._late += 1
                next_t = time.monotonic()      # 重新对齐，不要越欠越多
            elif slack > 0.0:
                time.sleep(slack)

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._on_stop()

    @property
    def stats(self):
        """下发统计。``late_ratio`` 是调度抖动的指纹——D 组找双峰成因要用它。"""
        return {'sent': self._sent, 'late': self._late,
                'late_ratio': (self._late / self._sent) if self._sent else 0.0,
                'rate_hz': self.rate_hz}

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False


class SimCmdVelDriver(_StreamingDriver):
    """仿真：持续往 /cmd_vel 发车体系 Twist。"""

    frame = 'body'
    interface = 'velocity'

    def __init__(self, node, topic='/cmd_vel', rate_hz=50.0):
        super().__init__(rate_hz)
        self._node = node
        self._topic = topic
        self._pub = node.create_publisher(Twist, topic, 10)
        self._cmd = (0.0, 0.0, 0.0)

    def _send_once(self):
        with self._lock:
            vx, vy, wz = self._cmd
        m = Twist()
        m.linear.x = vx
        m.linear.y = vy
        m.angular.z = wz
        self._pub.publish(m)

    def _on_stop(self):
        # 停止发流前显式发几帧零速，不要靠 cmd_vel_timeout 兜底
        for _ in range(5):
            self._pub.publish(Twist())
            time.sleep(0.02)

    def set_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        with self._lock:
            self._cmd = (float(vx), float(vy), float(wz))

    def halt(self):
        self.set_velocity(0.0, 0.0, 0.0)

    @property
    def commanded(self):
        with self._lock:
            return self._cmd


class SdkPositionDriver(_StreamingDriver):
    """真机：位置增量积分 + 持续重发，做法同 examples/202。

    x/y/theta 三个"关节"走 set_joints_position 的位置通道。
    速度由本类自己积分成位置增量，语义与仿真侧的 set_velocity 对齐。
    """

    frame = 'body'
    interface = 'position'

    def __init__(self, astribot, rate_hz=250.0, control_way='filter'):
        super().__init__(rate_hz)
        self._astribot = astribot
        self._chassis = astribot.chassis_name
        self._control_way = control_way
        seed = list(astribot.get_desired_joints_position([self._chassis])[0])
        self._pos = [seed[0], seed[1], seed[2]]
        self.seed = tuple(self._pos)
        self._cmd = (0.0, 0.0, 0.0)
        self._lock_xy = False          # 原地旋转时锁死 x/y

    def _send_once(self):
        with self._lock:
            vx, vy, wz = self._cmd
            self._pos[0] += vx * self.dt
            self._pos[1] += vy * self.dt
            self._pos[2] += wz * self.dt
            if self._lock_xy:
                self._pos[0] = self.seed[0]
                self._pos[1] = self.seed[1]
            out = list(self._pos)
        self._astribot.set_joints_position([self._chassis], [out],
                                          control_way=self._control_way)

    def _on_stop(self):
        # 停止运动后仍要继续重发最终位置：control_way='filter' 下 SDK 还会自己
        # 再收敛一段，循环一停就没人抓着底盘了（实测不重发漂 0.045~0.37 rad）。
        with self._lock:
            out = list(self._pos)
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            self._astribot.set_joints_position([self._chassis], [out],
                                               control_way=self._control_way)
            time.sleep(self.dt)

    def set_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        with self._lock:
            self._cmd = (float(vx), float(vy), float(wz))

    def halt(self):
        self.set_velocity(0.0, 0.0, 0.0)

    def lock_xy(self, enable=True):
        """原地旋转专用：把 x/y 钉在种子值上（见 examples/212 的说明）。"""
        with self._lock:
            self._lock_xy = bool(enable)

    @property
    def commanded_position(self):
        with self._lock:
            return tuple(self._pos)

    @property
    def commanded(self):
        with self._lock:
            return self._cmd


def ramped_move(driver, axis, amount, v_max, accel, spin_fn=None,
                settle_sec=1.0, timeout_sec=60.0):
    """沿单一自由度做定量运动，用 brake_in_time_profile 规划。

    axis: 'x' | 'y' | 'wz'
    amount: 位移 (m) 或转角 (rad)，带符号
    返回 dict：指令层走过的量、耗时、下发统计。

    **注意返回值是"指令层"的量，不是实测量。** 实测必须从真值源单独取——
    只比对自己发出去的指令等于什么都没验证。
    """
    if axis not in ('x', 'y', 'wz'):
        raise ValueError(f"axis 必须是 'x'/'y'/'wz'，收到 {axis}")
    dt = driver.dt
    traveled = 0.0
    v = 0.0
    t0 = time.monotonic()
    while True:
        remaining = amount - traveled
        v = brake_in_time_profile(remaining, v_max, accel, v, dt)
        traveled += v * dt
        kw = {'x': {'vx': v}, 'y': {'vy': v}, 'wz': {'wz': v}}[axis]
        driver.set_velocity(**kw)
        if spin_fn is not None:
            spin_fn(dt)
        else:
            time.sleep(dt)
        if abs(remaining) < 1e-4 and abs(v) < 1e-4:
            break
        if time.monotonic() - t0 > timeout_sec:
            raise TimeoutError(f'ramped_move 超时 {timeout_sec}s，'
                               f'已走 {traveled:.4f}/{amount:.4f}')
    driver.halt()
    # 保持零速一段，让惯性和 SDK 的收敛尾巴走完
    end = time.monotonic() + settle_sec
    while time.monotonic() < end:
        if spin_fn is not None:
            spin_fn(dt)
        else:
            time.sleep(dt)
    return {'commanded_travel': traveled,
            'duration_sec': time.monotonic() - t0,
            'driver_stats': driver.stats}


def make_driver(env, node=None, astribot=None, **kw):
    if env == 'sim':
        if node is None:
            raise ValueError('sim driver 需要一个 rclpy Node')
        return SimCmdVelDriver(node, **kw)
    if env == 'real':
        if astribot is None:
            raise ValueError('real driver 需要一个已连接的 Astribot 实例')
        return SdkPositionDriver(astribot, **kw)
    raise ValueError(f'未知环境 {env}')
