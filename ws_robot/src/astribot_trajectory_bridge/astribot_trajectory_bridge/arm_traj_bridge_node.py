#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂轨迹桥接节点（Gate 3）。**薄适配层** —— 执行逻辑全在 ArmTrajExecutor。

每个规划组一个 FollowJointTrajectory Action Server，动作名与 MoveIt 的
``moveit_controllers.yaml`` 对齐，这样上层业务零改动。

方案 A（move_joints_waypoints）挂在**独立 Service** 上，不混进 Action —— 它是
阻塞调用、期间不可取消，混进 Action 会让上层"以为能取消实际取不了"。
"""

import threading
import time

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

from astribot_bridge_msgs.srv import DispatchWaypoints, SetGripper
from astribot_trajectory_bridge.arm_bridge_core import (
    ArmBridgeConfig,
    ArmTrajExecutor,
    EC_INVALID_GOAL,
    EC_SUCCESSFUL,
    PH_ABORTED,
    PH_CANCELED,
    PH_DONE,
    PH_HOLDING,
    PH_SETTLING,
    PH_STREAMING,
    WaypointDispatcher,
)
from astribot_trajectory_bridge.arm_traj_math import reshape_waypoints
from astribot_trajectory_bridge.callback_layout import ARM_GROUPS, make_groups
from astribot_trajectory_bridge.gripper_core import (
    GripperConfig,
    GripperController,
)
from astribot_trajectory_bridge.gripper_math import GripperConfigError
from astribot_trajectory_bridge.ros_ports import RosClock, StatusReporter
from astribot_trajectory_bridge.write_gate import (
    TARGET_REAL,
    TARGET_SIM,
    evaluate_write_gate,
    is_simulation_mode,
)


def _gripper_code(name):
    """把核心层的错误码名映射成 SetGripper 的常量。映射不上**显式抛错**。

    与 _srv_code 同构：核心层用字符串（为了能在没编译 msgs 的环境下单测），
    srv 用数字。两边名字不一致的后果只在**故障发生时**才暴露 ——
    也就是最不该出问题的时候，所以宁可抛也不给默认值。
    """
    if not hasattr(SetGripper.Response, name):
        raise KeyError(
            '错误码名 %r 在 SetGripper.srv 里没有对应常量。'
            '新增错误码必须同时改 srv 定义。' % (name,))
    return getattr(SetGripper.Response, name)


def _srv_code(name):
    """把核心层的错误码名映射成 DispatchWaypoints 的常量。映射不上显式抛错。"""
    if not hasattr(DispatchWaypoints.Response, name):
        raise KeyError(
            '错误码名 %r 在 DispatchWaypoints.srv 里没有对应常量。'
            '新增错误码必须同时改 srv 定义。' % (name,))
    return getattr(DispatchWaypoints.Response, name)


class ArmTrajBridgeNode(Node):

    CALLBACK_GROUPS = ARM_GROUPS

    def __init__(self, session, node_name='arm_traj_bridge'):
        super().__init__(node_name)
        self._declare_params()
        self._session = session
        self._clock_port = RosClock(self)
        self.status = StatusReporter(self, node_name=node_name)

        self._write_allowed = self._check_write_gate(session)

        self._executors = {}       # group -> ArmTrajExecutor
        self._dispatchers = {}     # group -> WaypointDispatcher
        self._action_servers = {}
        self._goal_lock = threading.Lock()

        groups = self.get_parameter('groups').value
        name_map = self._parse_name_map()
        cb_groups = make_groups(
            [g for g in ARM_GROUPS if g != 'exec'],
            MutuallyExclusiveCallbackGroup, 'arm_traj_bridge')
        exec_group = ReentrantCallbackGroup()
        srv_group = cb_groups['srv']

        for grp in groups:
            part = name_map.get(grp)
            if part is None:
                raise RuntimeError(
                    'group %r 在 part_name_map 里没有对应的 SDK 部件名' % grp)
            joint_names = self.get_parameter('joint_names.%s' % grp).value
            cfg = self._build_config(part, joint_names)
            ex = ArmTrajExecutor(cfg, session, self._clock_port)
            ok, detail = ex.load_limits()
            if not ok:
                self.get_logger().error(
                    '组 %s 限位加载失败，该组 Action 不启动：%s' % (grp, detail))
                self.status.publish_events(ex.drain_events())
                continue
            self.status.publish_events(ex.drain_events())
            self._executors[grp] = ex
            self._dispatchers[grp] = WaypointDispatcher(
                cfg, session,
                enabled=self.get_parameter('enable_waypoints_service').value)
            action_name = self.get_parameter('action_name.%s' % grp).value
            self._action_servers[grp] = ActionServer(
                self, FollowJointTrajectory, action_name,
                execute_callback=self._make_execute_cb(grp),
                goal_callback=self._make_goal_cb(grp),
                cancel_callback=self._cancel_cb,
                callback_group=exec_group)
            self.get_logger().info(
                '组 %s -> 部件 %s，Action=%s，%d 个关节' %
                (grp, part, action_name, len(joint_names)))

        self.create_service(DispatchWaypoints, '~/dispatch_waypoints',
                            self._srv_dispatch_waypoints,
                            callback_group=srv_group)

        grip_group = cb_groups['grip']
        self._gripper = self._build_gripper(session)
        if self._gripper is not None:
            self.create_service(SetGripper, '~/set_gripper',
                                self._srv_set_gripper,
                                callback_group=grip_group)
            self.get_logger().info(
                '夹爪服务已启动：~/set_gripper，夹爪=%s（对外用 opening_fraction，'
                '1.0=全张开；厂商命令空间是 0=张开/100=闭合，翻转在桥接内部）'
                % (self._gripper.cfg.gripper_names,))

        if not self._executors:
            self.get_logger().error(
                '没有任何组成功初始化，本节点不会接受任何轨迹。')


    def _declare_params(self):
        d = self.declare_parameter
        d('groups', ['arm_left', 'arm_right'])
        d('part_name_map_keys', ['arm_left', 'arm_right'])
        d('part_name_map_values', ['astribot_arm_left', 'astribot_arm_right'])
        for grp, part in (('arm_left', 'left'), ('arm_right', 'right')):
            d('joint_names.%s' % grp,
              ['astribot_arm_%s_joint_%d' % (part, i) for i in range(1, 8)])
            d('action_name.%s' % grp,
              '/astribot/%s_controller/follow_joint_trajectory' % grp)
        d('stream_freq', 250.0)
        d('control_way', 'direct')
        d('use_wbc', False)
        d('add_default_torso', False)
        d('interp', 'cubic')
        d('max_traj_duration_sec', 60.0)
        d('limit_margin_rad', 0.0)
        d('cross_check_urdf', True)
        d('limit_cross_check_tol_rad', 0.01)
        d('strict_limit_check', False)
        d('max_tracking_error_rad', 0.10)
        d('abort_on_tracking_error', True)
        d('settle_tolerance_rad', 0.02)
        d('settle_timeout_sec', 2.0)
        d('hold_still_epsilon_rad', 0.001)
        d('hold_still_ticks_required', 25)
        d('hold_timeout_sec', 2.0)
        d('enable_waypoints_service', False)
        d('gripper.enable_service', True)
        d('gripper.names', [])
        d('gripper.default_duration_sec', 1.0)
        d('gripper.default_max_force_n', 0.0)
        d('gripper.settle_extra_sec', 0.0)
        d('gripper.stream_freq', 250.0)
        d('gripper.mid_stream_tolerance', 0.5)
        d('gripper.mid_stream_timeout_sec', 3.0)
        d('declared_target', 'sim')
        d('allow_write_to_real', False)
        d('allow_unsafe_mode', False)

    def _parse_name_map(self):
        """ROS2 参数不支持 dict，用两个等长数组表达映射。长度不等即拒绝启动。"""
        keys = self.get_parameter('part_name_map_keys').value
        values = self.get_parameter('part_name_map_values').value
        if len(keys) != len(values):
            raise RuntimeError(
                'part_name_map_keys(%d) 与 part_name_map_values(%d) 长度不等'
                % (len(keys), len(values)))
        return dict(zip(keys, values))

    def _build_config(self, part, joint_names):
        g = lambda n: self.get_parameter(n).value      # noqa: E731
        return ArmBridgeConfig(
            part_name=part, joint_names=joint_names,
            stream_freq=g('stream_freq'), control_way=g('control_way'),
            use_wbc=g('use_wbc'), add_default_torso=g('add_default_torso'),
            interp=g('interp'),
            max_traj_duration_sec=g('max_traj_duration_sec'),
            limit_margin_rad=g('limit_margin_rad'),
            cross_check_urdf=g('cross_check_urdf'),
            limit_cross_check_tol_rad=g('limit_cross_check_tol_rad'),
            strict_limit_check=g('strict_limit_check'),
            max_tracking_error_rad=g('max_tracking_error_rad'),
            abort_on_tracking_error=g('abort_on_tracking_error'),
            settle_tolerance_rad=g('settle_tolerance_rad'),
            settle_timeout_sec=g('settle_timeout_sec'),
            hold_still_epsilon_rad=g('hold_still_epsilon_rad'),
            hold_still_ticks_required=g('hold_still_ticks_required'),
            hold_timeout_sec=g('hold_timeout_sec'))

    def _check_write_gate(self, session):
        target = self.get_parameter('declared_target').value
        try:
            mode = session.get_robot_mode()
        except Exception as exc:      # noqa: BLE001
            self.get_logger().error('[WriteGate] 读机器人模式失败：%s' % exc)
            self.status.publish('SDK_CALL_FAILED', str(exc))
            return False
        actual = TARGET_SIM if is_simulation_mode(mode) else TARGET_REAL
        d = evaluate_write_gate([actual], target,
                               self.get_parameter('allow_write_to_real').value,
                               mode,
                               self.get_parameter('allow_unsafe_mode').value)
        if not d.allowed:
            self.get_logger().error('[WriteGate] 拒绝开启写通路：%s' % d.reason)
            self.status.publish(d.status_code, d.reason)
        return d.allowed


    def _make_goal_cb(self, grp):
        def cb(goal_request):
            if not self._write_allowed:
                self.get_logger().error('写通路准入未通过，拒绝目标')
                return GoalResponse.REJECT
            if self._goal_lock.locked():
                self.get_logger().warn('已有轨迹在执行，拒绝新目标')
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT
        return cb

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _make_execute_cb(self, grp):
        def cb(goal_handle):
            ex = self._executors[grp]
            result = FollowJointTrajectory.Result()
            traj = goal_handle.request.trajectory
            times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                     for p in traj.points]
            positions = [list(p.positions) for p in traj.points]
            velocities = ([list(p.velocities) for p in traj.points]
                          if all(p.velocities for p in traj.points) else None)

            with self._goal_lock:
                ok, ec, detail = ex.start(list(traj.joint_names), times,
                                          positions, velocities)
                self.status.publish_events(ex.drain_events())
                if not ok:
                    goal_handle.abort()
                    result.error_code = ec
                    result.error_string = detail
                    return result

                period = 1.0 / ex.cfg.stream_freq
                rate = self.create_rate(ex.cfg.stream_freq)
                while ex.phase not in (PH_DONE, PH_CANCELED, PH_ABORTED):
                    if goal_handle.is_cancel_requested:
                        ex.request_cancel()
                    ex.step()
                    self.status.publish_events(ex.drain_events())
                    for fb in ex.drain_feedbacks():
                        msg = FollowJointTrajectory.Feedback()
                        msg.joint_names = list(traj.joint_names)
                        msg.desired.positions = [float(v) for v in fb.desired]
                        msg.actual.positions = [float(v) for v in fb.actual]
                        msg.error.positions = [
                            float(a - d) for a, d in zip(fb.actual, fb.desired)]
                        goal_handle.publish_feedback(msg)
                    rate.sleep()

                result.error_code = ex.error_code
                result.error_string = ex.detail
                if ex.phase == PH_DONE:
                    goal_handle.succeed()
                elif ex.phase == PH_CANCELED:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return result
        return cb


    def _build_gripper(self, session):
        """建夹爪控制器。失败返回 None 并**响亮上报**，不静默跳过。"""
        names = list(self.get_parameter('gripper.names').value or [])
        if not names:
            try:
                names = list(session.effector_names())
            except Exception as exc:      # noqa: BLE001
                self.get_logger().error(
                    '夹爪服务未启动：gripper.names 未配置且向 SDK 查询 '
                    'effector_names 失败：%s' % exc)
                self.status.publish('SDK_CALL_FAILED',
                                    '查询 effector_names 失败：%s' % exc)
                return None

        in_sim = True
        try:
            in_sim = is_simulation_mode(session.get_robot_mode())
        except Exception as exc:      # noqa: BLE001
            self.get_logger().warning(
                '读机器人模式失败，force_applied 一律按仿真处理（False）：%s' % exc)

        try:
            cfg = GripperConfig(
                gripper_names=names,
                enable_service=self.get_parameter('gripper.enable_service').value,
                default_duration_sec=self.get_parameter(
                    'gripper.default_duration_sec').value,
                default_max_force_n=self.get_parameter(
                    'gripper.default_max_force_n').value,
                settle_extra_sec=self.get_parameter(
                    'gripper.settle_extra_sec').value,
                stream_freq=self.get_parameter('gripper.stream_freq').value,
                mid_stream_tolerance=self.get_parameter(
                    'gripper.mid_stream_tolerance').value,
                mid_stream_timeout_sec=self.get_parameter(
                    'gripper.mid_stream_timeout_sec').value)
        except GripperConfigError as exc:
            self.get_logger().error('夹爪配置非法，服务未启动：%s' % exc)
            self.status.publish('CONFIG_FALLBACK', '夹爪配置非法：%s' % exc)
            return None

        return GripperController(cfg, session, sleep_fn=time.sleep,
                                 clock_fn=time.monotonic,
                                 in_simulation=in_sim)

    def _srv_set_gripper(self, request, response):
        """夹爪开合。**阻塞** duration 秒（SDK 的 open/close_effector 是阻塞的）。

        对外用 opening_fraction（1.0 = 全张开），厂商命令空间的极性翻转
        （0=张开、100=闭合）关在 gripper_core 内部。
        """
        if self._gripper is None:
            response.ok = False
            response.error_code = _gripper_code('DISABLED_BY_CONFIG')
            response.detail = '夹爪控制器未初始化（见启动日志与 status 话题）'
            return response

        r = self._gripper.execute(
            name=request.name,
            opening_fraction=request.opening_fraction,
            duration=request.duration,
            use_raw_cmd=request.use_raw_cmd,
            raw_cmd=request.raw_cmd,
            max_force=request.max_force,
            write_allowed=self._write_allowed)

        self.status.publish_events(self._gripper.drain_events())

        response.ok = r.ok
        response.error_code = _gripper_code(r.error_code)
        response.detail = r.detail
        response.dispatched_cmd = r.dispatched_cmd
        response.dispatched_rad = r.dispatched_rad
        response.actual_cmd = r.actual_cmd
        response.force_applied = r.force_applied
        if not r.ok:
            self.get_logger().warning('[夹爪] %s：%s' % (r.error_code, r.detail))
        return response

    def _srv_dispatch_waypoints(self, request, response):
        """方案 A：整段下发。**阻塞、期间不可取消**（见 srv 定义里的说明）。"""
        grp = None
        for g, ex in self._executors.items():
            if ex.cfg.part_name == request.part:
                grp = g
                break
        if grp is None:
            response.ok = False
            response.error_code = _srv_code('UNKNOWN_PART')
            response.detail = ('part=%r 不是本节点管理的部件。已初始化的部件：%s'
                               % (request.part,
                                  [e.cfg.part_name for e in self._executors.values()]))
            return response

        if not self._write_allowed:
            response.ok = False
            response.error_code = _srv_code('WRITE_GATE_DENIED')
            response.detail = '写通路准入未通过（见 status 话题）'
            return response

        ex = self._executors[grp]
        try:
            waypoints = reshape_waypoints(list(request.flat_waypoints), request.dof)
        except Exception as exc:      # noqa: BLE001
            response.ok = False
            response.error_code = _srv_code('SHAPE_MISMATCH')
            response.detail = str(exc)
            return response
        if len(waypoints) != len(request.time_list):
            response.ok = False
            response.error_code = _srv_code('SHAPE_MISMATCH')
            response.detail = ('路点数 %d 与 time_list 长度 %d 不符'
                               % (len(waypoints), len(request.time_list)))
            return response

        with self._goal_lock:
            d = self._dispatchers[grp]
            ok, code_name, detail, disp, dropped = d.dispatch(
                waypoints, list(request.time_list), ex._lower, ex._upper)
            self.status.publish_events(d.drain_events())
        response.ok = bool(ok)
        response.error_code = _srv_code(code_name)
        response.detail = detail
        response.dispatched_points = int(disp)
        response.dropped_points = int(dropped)
        return response
