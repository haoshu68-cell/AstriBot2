#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""桥接与外部世界之间的端口（Port）抽象，以及供测试用的替身（Fake）。

为什么要有这一层
==============
桥接的控制逻辑（使能/leash/闭环校正/轨迹执行/错误码）是**分支最多、也最需要
被测试覆盖**的部分，但它依赖三样在离线环境里拿不到的东西：

1. 厂商 SDK 会话（``Astribot`` 实例）—— 需要真机侧的接口进程活着才能建立；
2. TF（位姿源）；
3. 时钟。

把这三样收敛成端口之后，控制逻辑可以在**没有 SDK、没有 rclpy、没有仿真**的条件下
把全部分支跑通。等库依赖解决，只需换上真实实现即可进 Gate 0。

替身放在运行包里而不是 test/ 目录的理由
====================================
多个测试文件、以及将来的集成测试脚手架都要用同一套替身。放在 test/ 下需要靠
conftest 或 sys.path 技巧共享，容易在 colcon test 与直接 pytest 两种环境下
行为不一致（本项目已经踩过一次同名测试文件互相冲突）。
本模块**不被任何生产节点 import**，只被测试与调试脚本 import。
"""

import math



class SessionPort:
    """厂商 SDK 会话端口。方法签名与 examples 里实际调用的形式**逐字对齐**。

    对齐依据：
      * ``get_desired_joints_position(names)`` —— 101:46 / 202:44 / 203:43
      * ``get_current_joints_position(names)`` —— 101:45 / 106:45
      * ``get_current_joints_velocity(names)`` —— 签名 astribot_client.py:217，
        examples/101:47。只读，chassis_odom_node 用它填 Odometry.twist
      * ``set_joints_position(names, position, control_way, use_wbc,
        add_default_torso)`` —— 105:53、签名 astribot_client.py:704
      * ``move_joints_waypoints(names, waypoints, time_list, use_wbc,
        add_default_torso)`` —— 206:45、签名 astribot_client.py:639
      * ``get_joints_position_limit(names)`` -> **(lower, upper)** ——
        astribot_client.py:141（注意 examples/100:49 的解包顺序是反的）
      * ``get_robot_mode()`` —— **挂在内层 astribot_interface 上**，
        astribot_client.py:52 是 ``self.astribot_interface.get_robot_mode()``；
        取值 'safe'/'professional'/'extremity'，**其它任何值都表示仿真**
        （54-62 是 if/elif/elif/else，else 置 __in_simulation=True）
      * ``open_effector(names, duration)`` / ``close_effector(names, duration)``
        —— 109:41,43；**阻塞**、尊重 duration。极性见下
      * ``set_effector_max_force(names, max_force)`` —— 109:40，
        **仿真下是空操作**（astribot_client.py:1139 `if __in_simulation: return`）
      * ``get_dof(names)`` —— 100:44
    """

    def get_desired_joints_position(self, names):
        raise NotImplementedError

    def get_current_joints_position(self, names):
        raise NotImplementedError

    def get_current_joints_velocity(self, names):
        raise NotImplementedError

    def set_joints_position(self, names, position, control_way='filter',
                            use_wbc=False, add_default_torso=True):
        raise NotImplementedError

    def move_joints_waypoints(self, names, waypoints, time_list,
                              use_wbc=False, add_default_torso=True):
        raise NotImplementedError

    def get_joints_position_limit(self, names=None):
        raise NotImplementedError

    def get_robot_mode(self):
        raise NotImplementedError

    def get_dof(self, names=None):
        raise NotImplementedError


    def open_effector(self, names=None, duration=1.0):
        raise NotImplementedError

    def close_effector(self, names=None, duration=1.0):
        raise NotImplementedError

    def set_effector_max_force(self, names, max_force):
        """!!! 仿真下是空操作 !!! 力限只能在真机上验收。"""
        raise NotImplementedError


class PosePort:
    """位姿源端口（真实实现查 TF ``map -> base``）。

    !!! 真实实现必须用 ``lookup_transform(target, src, Time())``、**不带 timeout** !!!
    项目笔记记过一个坑：带 timeout 的**动态** TF 查询在非专用线程里必然失败，
    而 **static** TF 却能成功 —— 于是 ``pose_source=ground_truth``（map->odom 是
    静态 TF）会"看着正常"，而 ``slam`` 模式却查不到，极难归因。
    """

    def lookup(self):
        """返回 ``(pose, stamp_sec)``，pose 为 ``[x, y, theta]``。

        查不到时返回 ``(None, None)`` —— **不抛异常**：位姿源短暂不可用是正常
        工况（外环会冻结校正并上报状态），不该让调用方用异常处理常规分支。
        """
        raise NotImplementedError


class ClockPort:
    """时钟端口。注入而不是直接调 time.time()，让测试能确定性地推进时间。"""

    def now(self):
        raise NotImplementedError



class FakeClock(ClockPort):
    """可手动推进的时钟。"""

    def __init__(self, t0=0.0):
        self._t = float(t0)

    def now(self):
        return self._t

    def advance(self, dt):
        self._t += float(dt)
        return self._t


class FakePose(PosePort):
    """可脚本化的位姿源。

    ``set_pose(None)`` 模拟"查不到"；``stamp_offset`` 让测试能造出"数据过龄"。
    """

    def __init__(self, clock, pose=None, stamp_offset=0.0):
        self._clock = clock
        self._pose = list(pose) if pose is not None else None
        self.stamp_offset = float(stamp_offset)
        self.lookup_calls = 0

    def set_pose(self, pose):
        self._pose = list(pose) if pose is not None else None

    def lookup(self):
        self.lookup_calls += 1
        if self._pose is None:
            return (None, None)
        return (list(self._pose), self._clock.now() - self.stamp_offset)


class SdkCallFailure(RuntimeError):
    """替身用来模拟 SDK 抛异常的类型。

    生产代码**不能**依赖这个具体类型 —— 厂商 SDK 抛什么异常我们不掌握，
    所以桥接必须捕获宽泛的 Exception 再转成错误码/状态位。
    """


class FakeSession(SessionPort):
    """厂商 SDK 会话的替身。

    模拟能力：
      * 按部件维护 desired / current 位置，两者可**独立设置** ——
        这是模拟打滑的关键（指令一直加，实际不跟）；
      * ``fail_on`` 让任意方法在第 N 次调用时抛异常，用来测"异常必须转成错误码"；
      * 记录所有 ``set_joints_position`` 的入参，用来断言
        ``add_default_torso=False`` 这类容易回归成 SDK 默认值的参数。
    """

    def __init__(self, desired=None, current=None, limits=None,
                 robot_mode='safe', dofs=None, follow_ratio=1.0,
                 in_simulation=True, velocity=None):
        self._desired = dict(desired or {})
        self._current = dict(current or {})
        self._velocity = dict(velocity or {})
        self._limits = dict(limits or {})
        self._robot_mode = robot_mode
        self._dofs = dict(dofs or {})
        self.follow_ratio = float(follow_ratio)
        self.set_position_calls = []
        self.waypoints_calls = []
        self.effector_calls = []
        self.effector_force_calls = []
        self.effector_max_force = None
        self.in_simulation = bool(in_simulation)
        self.fail_on = {}


    def fail_after(self, method, ok_calls=0):
        """让 ``method`` 在成功 ok_calls 次之后开始抛异常。"""
        self.fail_on[method] = int(ok_calls)

    def _maybe_fail(self, method):
        if method in self.fail_on:
            if self.fail_on[method] <= 0:
                raise SdkCallFailure('FakeSession：%s 被注入为失败' % method)
            self.fail_on[method] -= 1


    def set_desired(self, part, values):
        self._desired[part] = list(values)

    def set_current(self, part, values):
        self._current[part] = list(values)

    def nudge_current(self, part, dxy_dtheta):
        """让实际位置动一点，用来模拟"指令加了但实际只跟了一部分"（打滑）。"""
        cur = self._current.get(part, [0.0, 0.0, 0.0])
        self._current[part] = [cur[0] + dxy_dtheta[0],
                               cur[1] + dxy_dtheta[1],
                               cur[2] + dxy_dtheta[2]]


    def get_desired_joints_position(self, names):
        self._maybe_fail('get_desired_joints_position')
        return [list(self._desired.get(n, [0.0, 0.0, 0.0])) for n in names]

    def get_current_joints_position(self, names):
        self._maybe_fail('get_current_joints_position')
        return [list(self._current.get(n, [0.0, 0.0, 0.0])) for n in names]

    def get_current_joints_velocity(self, names):
        self._maybe_fail('get_current_joints_velocity')
        return [list(self._velocity.get(n, [0.0, 0.0, 0.0])) for n in names]

    def set_joints_position(self, names, position, control_way='filter',
                            use_wbc=False, add_default_torso=True):
        self._maybe_fail('set_joints_position')
        self.set_position_calls.append(
            (list(names), [list(p) for p in position], control_way,
             use_wbc, add_default_torso))
        for n, p in zip(names, position):
            self._desired[n] = list(p)
            cur = self._current.get(n, [0.0] * len(p))
            self._current[n] = [
                c + self.follow_ratio * (t - c) for c, t in zip(cur, p)]
        return True

    def move_joints_waypoints(self, names, waypoints, time_list,
                              use_wbc=False, add_default_torso=True):
        self._maybe_fail('move_joints_waypoints')
        self.waypoints_calls.append(
            (list(names), [[list(p) for p in wp] for wp in waypoints],
             list(time_list), use_wbc, add_default_torso))
        return True

    def get_joints_position_limit(self, names=None):
        self._maybe_fail('get_joints_position_limit')
        keys = names if names is not None else list(self._limits.keys())
        lower, upper = [], []
        for k in keys:
            lo, hi = self._limits.get(k, ([-math.pi] * 7, [math.pi] * 7))
            lower.append(list(lo))
            upper.append(list(hi))
        return (lower, upper)      # 顺序：(lower, upper)，见 client.py:141

    def get_robot_mode(self):
        self._maybe_fail('get_robot_mode')
        return self._robot_mode

    def get_dof(self, names=None):
        self._maybe_fail('get_dof')
        keys = names if names is not None else list(self._dofs.keys())
        return [self._dofs.get(k, 7) for k in keys]


    def open_effector(self, names=None, duration=1.0):
        self._maybe_fail('open_effector')
        ns = list(names) if names else []
        for n in ns:
            self._desired[n] = [0.0]
            self._current[n] = [0.0 * self.follow_ratio]
        self.effector_calls.append(('open', ns, float(duration)))
        return 'move to joint position success'

    def close_effector(self, names=None, duration=1.0):
        self._maybe_fail('close_effector')
        ns = list(names) if names else []
        for n in ns:
            self._desired[n] = [100.0]
            self._current[n] = [100.0 * self.follow_ratio]
        self.effector_calls.append(('close', ns, float(duration)))
        return 'move to joint position success'

    def set_effector_max_force(self, names, max_force):
        self._maybe_fail('set_effector_max_force')
        self.effector_force_calls.append((list(names), list(max_force)))
        if self.in_simulation:
            return None
        self.effector_max_force = list(max_force)
        return None
