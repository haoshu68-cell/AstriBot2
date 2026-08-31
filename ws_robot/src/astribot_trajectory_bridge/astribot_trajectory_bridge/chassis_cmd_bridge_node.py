#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘控制桥接节点（Gate 5）。**薄适配层** —— 控制逻辑全在 ChassisBridgeCore。

本节点只做四件事：
  1. 订阅 /cmd_vel 喂给核心；
  2. 按 freq / outer_rate 调核心的两个 tick；
  3. 把核心产出的 StatusEvent 转成话题；
  4. 提供 ~/enable / ~/disable / ~/reset_leash 三个服务。

为什么内外环用两个独立的 MutuallyExclusiveCallbackGroup
====================================================
内环 250Hz（4ms）与外环 10Hz 都要读 SDK。放同一个 group 会串行化：外环那次
TF 查询 + 两次 SDK 读会挤掉内环的节拍。分到两个 group、配
MultiThreadedExecutor，两者可并行。

但 GIL 仍在 —— 这是单进程多节点方案（S-1）的真实代价，不是能靠分组消除的。
所以内环额外监控实际周期，超阈上报 LOOP_OVERRUN，让抖动可见而不是靠感觉。
"""

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from astribot_trajectory_bridge.chassis_bridge_core import (
    ChassisBridgeConfig,
    ChassisBridgeCore,
    ST_ENABLED,
)
from astribot_trajectory_bridge.ros_ports import (
    RosClock,
    StatusReporter,
    TfPosePort,
)
from astribot_trajectory_bridge.write_gate import (
    TARGET_REAL,
    TARGET_SIM,
    evaluate_write_gate,
    is_simulation_mode,
    validate_pose_source_target_combo,
)


class ChassisCmdBridgeNode(Node):

    def __init__(self, session, node_name='chassis_cmd_bridge'):
        super().__init__(node_name)
        self._declare_params()
        cfg = self._build_config()

        # !!! 不能叫 self._clock !!! rclpy.Node 在 node.py:210 用 self._clock 存自己的
        # 时钟，create_timer/get_clock 都读它。取同名会把节点时钟**覆盖掉**，
        # 报错是 "'RosClock' object has no attribute 'handle'" —— 信息指向时钟对象，
        # 真实原因是属性遮蔽，极难归因。
        self._clock_port = RosClock(self)
        self._pose = TfPosePort(self, cfg.map_frame, cfg.base_frame)
        self.core = ChassisBridgeCore(cfg, session, self._pose, self._clock_port)
        self.status = StatusReporter(self, node_name=node_name)

        # ---- 写通路准入。D-1 统一 domain 下这是唯一阻止"仿真指令打到真机"的机制 ----
        self._write_allowed = self._check_write_gate(session, cfg)

        inner_group = MutuallyExclusiveCallbackGroup()
        outer_group = MutuallyExclusiveCallbackGroup()
        srv_group = MutuallyExclusiveCallbackGroup()
        # !!! cmd_vel 订阅必须独立成组，绝不能和内环定时器共用 !!!
        #
        # 2026-08-31 实机实测出来的：原来两者共用 inner_group，而
        # MutuallyExclusiveCallbackGroup 保证组内**串行**。内环每拍要做两次跨进程
        # SDK 往返（get_current_joints_position 判 leash + set_joints_position 下发），
        # Python+GIL 下实测一拍 ~10ms，而定时器按 freq=250 每 4ms 就排一个 ——
        # 定时器回调永久积压、把组占满，`_on_cmd_vel` **一次都拿不到执行机会**。
        #
        # 后果极其隐蔽，因为每一层看起来都正常：
        #   · /cmd_vel 实测 26.8Hz、2708 帧非零 —— 消息确实到了订阅端
        #   · `_last_twist` 恒为初始值 (0,0,0) -> 积分零速度 -> pos_cmd 不变
        #   · SDK 的 desired 与 actual 6 秒内一个数位都不变
        #   · leash **不会** trip（指令与实测都不动，偏差恒 0）
        #   · 看门狗也不报，因为 `_last_twist_time` 是 None，走的是"无输入置零"
        #     那条不发事件的分支
        #   · 唯一的可观测量是 LOOP_OVERRUN（实测累计 23195 次）
        # 表现就是"nav2 一切正常、路径也规划出来了、机器人一动不动"。
        cmd_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self._on_cmd_vel, 10, callback_group=cmd_group)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)

        self._inner_timer = self.create_timer(
            1.0 / cfg.freq, self._inner_tick, callback_group=inner_group)
        self._outer_timer = self.create_timer(
            1.0 / cfg.outer_rate, self._outer_tick, callback_group=outer_group)

        self.create_service(SetBool, '~/enable', self._srv_enable,
                            callback_group=srv_group)
        self.create_service(Trigger, '~/disable', self._srv_disable,
                            callback_group=srv_group)
        self.create_service(Trigger, '~/reset_leash', self._srv_reset_leash,
                            callback_group=srv_group)

        # 周期抖动监控
        self._last_inner_time = None
        self._overrun_count = 0
        self._loop_overrun_factor = self.get_parameter('loop_overrun_factor').value

        self.get_logger().info(
            '底盘桥接已启动：part=%s freq=%.1fHz 外环=%.1fHz 口径=%s '
            'leash=(%.3fm, %.3frad) 闭环=%s 位姿源=%s 写通路=%s。'
            '启动即停用，需调 ~/enable。'
            % (cfg.part_name, cfg.freq, cfg.outer_rate, cfg.input_frame,
               cfg.leash_xy_m, cfg.leash_theta_rad,
               cfg.enable_slam_correction, cfg.pose_source,
               '允许' if self._write_allowed else '被拒绝'))

    # ------------------------------------------------------------------ 参数

    def _declare_params(self):
        d = self.declare_parameter
        d('part_name', 'astribot_chassis')
        d('cmd_vel_topic', '/cmd_vel')
        d('odom_topic', '/astribot/chassis/odom_from_sdk')
        d('freq', 250.0)
        d('input_frame', 'body')
        d('theta_reference', 'at_enable')
        d('start_disabled', True)
        d('cmd_vel_timeout_sec', 0.3)
        d('max_vel_xy', 1.0)
        d('max_vel_theta', 2.0)
        d('max_accel_xy', 2.5)
        d('max_accel_theta', 3.2)
        d('leash_xy_m', 0.25)
        d('leash_theta_rad', 0.35)
        d('require_manual_reset', True)
        d('enable_slam_correction', True)
        d('pose_source', 'slam')
        d('map_frame', 'map')
        d('base_frame', 'astribot_torso_base')
        d('outer_rate', 10.0)
        d('slam_max_age_sec', 0.5)
        d('slam_jump_threshold_m', 0.30)
        d('kp_xy', 0.35)
        d('kp_theta', 0.40)
        d('max_corr_vel_xy', 0.10)
        d('max_corr_vel_theta', 0.20)
        d('require_slam_to_enable', False)
        d('slam_loss_grace_sec', 2.0)
        d('odom_drift_window_sec', 2.0)
        d('odom_drift_warn_m', 0.15)
        d('loop_overrun_factor', 1.5)
        # 写通路准入
        d('declared_target', 'sim')
        d('allow_write_to_real', False)
        d('allow_unsafe_mode', False)

    def _build_config(self):
        """构造配置。**非法参数直接抛异常，不带着危险配置启动。**"""
        g = lambda n: self.get_parameter(n).value      # noqa: E731
        return ChassisBridgeConfig(
            part_name=g('part_name'), freq=g('freq'),
            input_frame=g('input_frame'), theta_reference=g('theta_reference'),
            cmd_vel_timeout_sec=g('cmd_vel_timeout_sec'),
            max_vel_xy=g('max_vel_xy'), max_vel_theta=g('max_vel_theta'),
            max_accel_xy=g('max_accel_xy'), max_accel_theta=g('max_accel_theta'),
            leash_xy_m=g('leash_xy_m'), leash_theta_rad=g('leash_theta_rad'),
            require_manual_reset=g('require_manual_reset'),
            enable_slam_correction=g('enable_slam_correction'),
            pose_source=g('pose_source'),
            map_frame=g('map_frame'), base_frame=g('base_frame'),
            outer_rate=g('outer_rate'), slam_max_age_sec=g('slam_max_age_sec'),
            slam_jump_threshold_m=g('slam_jump_threshold_m'),
            kp_xy=g('kp_xy'), kp_theta=g('kp_theta'),
            max_corr_vel_xy=g('max_corr_vel_xy'),
            max_corr_vel_theta=g('max_corr_vel_theta'),
            require_slam_to_enable=g('require_slam_to_enable'),
            slam_loss_grace_sec=g('slam_loss_grace_sec'),
            odom_drift_window_sec=g('odom_drift_window_sec'),
            odom_drift_warn_m=g('odom_drift_warn_m'))

    # -------------------------------------------------------------- WriteGate

    def _check_write_gate(self, session, cfg):
        target = self.get_parameter('declared_target').value
        combo = validate_pose_source_target_combo(cfg.pose_source, target)
        if not combo.allowed:
            self.get_logger().error('[WriteGate] %s' % combo.reason)
            self.status.publish(combo.status_code, combo.reason)
            return False
        try:
            mode = session.get_robot_mode()
        except Exception as exc:      # noqa: BLE001
            self.get_logger().error('[WriteGate] 读机器人模式失败：%s' % exc)
            self.status.publish('SDK_CALL_FAILED', '读机器人模式失败：%s' % exc)
            return False
        backends = self._discover_backends(mode)
        d = evaluate_write_gate(backends, target,
                               self.get_parameter('allow_write_to_real').value,
                               mode,
                               self.get_parameter('allow_unsafe_mode').value)
        if not d.allowed:
            # 拒绝时**节点存活并持续上报**，不退出 —— 上层要能看到"为什么写不动"
            self.get_logger().error('[WriteGate] 拒绝开启写通路：%s' % d.reason)
            self.status.publish(d.status_code, d.reason)
        return d.allowed

    def _discover_backends(self, robot_mode):
        """按 SDK 自己报的模式判定当前连上的是哪个后端。

        依据（Gate 0 探针实测，2026-08-26，MuJoCo 后端）：
        ``get_robot_mode()`` 在仿真下返回 ``'simulation'``，而
        astribot_client.py:54-62 的判据是"**不在** safe/professional/extremity
        三者之中即为仿真"。所以这个返回值就是权威的后端身份标识。

        本函数**曾经**按 declared_target 单值返回（即"发现"的其实是声明本身，
        等于没有校验）。现在返回的是**实测身份**，所以：
          * 声明 sim 而连上真机 → 会在闸门②/④被挡下；
          * 声明 real 而连上仿真 → 同样被挡下。

        仍然做不到的事（如实标注）：SDK 会话只能连**一个**后端，
        所以本函数永远只返回一个元素。"图上同时存在 sim 与 real 两个后端"
        这种情况本函数**探测不到** —— 闸门①的 len>1 分支因此仍然是死路径，
        保留它是为了在将来拿到多后端枚举能力时不必改闸门。
        D-1 期间"绝不同时拉起 MuJoCo 与真机"依然要靠操作纪律。
        """
        return [TARGET_SIM if is_simulation_mode(robot_mode) else TARGET_REAL]

    # ---------------------------------------------------------------- 回调

    def _on_cmd_vel(self, msg):
        self.core.submit_twist(msg.linear.x, msg.linear.y, msg.angular.z)

    def _inner_tick(self):
        if not self._write_allowed:
            return
        now = self._clock_port.now()
        if self._last_inner_time is not None:
            period = now - self._last_inner_time
            target = 1.0 / self.core.cfg.freq
            if period > target * self._loop_overrun_factor:
                self._overrun_count += 1
                self.status.publish(
                    'LOOP_OVERRUN',
                    '内环周期 %.4fs 超过目标 %.4fs 的 %.1f 倍（累计 %d 次）'
                    % (period, target, self._loop_overrun_factor, self._overrun_count),
                    period, target)
        self._last_inner_time = now

        try:
            self.core.inner_tick()
        except Exception as exc:      # noqa: BLE001 —— 单拍异常不能让定时器死掉
            self.get_logger().error('内环异常：%s' % exc)
            self.status.publish('SDK_CALL_FAILED', '内环异常：%s' % exc)
        self.status.publish_events(self.core.drain_events())

    def _outer_tick(self):
        if not self._write_allowed:
            return
        try:
            self.core.outer_tick()
        except Exception as exc:      # noqa: BLE001
            self.get_logger().error('外环异常：%s' % exc)
            self.status.publish('SDK_CALL_FAILED', '外环异常：%s' % exc)
        self.status.publish_events(self.core.drain_events())
        self._publish_odom()

    def _publish_odom(self):
        """把 SDK 的底盘位姿发成 Odometry。

        !!! 这**不是**可直接当 Nav2 odom 用的量 !!!
        它是否有累积漂移尚未取证（Gate 0-f）。在取证之前只作为诊断话题，
        不接进 Nav2 的 odom 输入 —— 接错了会让定位默默变坏。
        """
        if self.core.pos_cmd is None:
            return
        try:
            actual = self.core.session.get_current_joints_position(
                [self.core.cfg.part_name])[0]
        except Exception:      # noqa: BLE001 —— 诊断话题发不出不算故障
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sdk_chassis'      # 刻意不写 odom：它不是 odom
        msg.child_frame_id = self.core.cfg.base_frame
        msg.pose.pose.position.x = float(actual[0])
        msg.pose.pose.position.y = float(actual[1])
        half = float(actual[2]) * 0.5
        msg.pose.pose.orientation.z = __import__('math').sin(half)
        msg.pose.pose.orientation.w = __import__('math').cos(half)
        self._odom_pub.publish(msg)

    # ---------------------------------------------------------------- 服务

    def _srv_enable(self, request, response):
        if not self._write_allowed:
            response.success = False
            response.message = '写通路准入未通过，拒绝使能（见 status 话题）'
            return response
        if not request.data:
            ok, detail = self.core.disable()
        else:
            ok, detail = self.core.enable()
        self.status.publish_events(self.core.drain_events())
        response.success = bool(ok)
        response.message = detail
        return response

    def _srv_disable(self, request, response):
        ok, detail = self.core.disable()
        self.status.publish_events(self.core.drain_events())
        response.success = bool(ok)
        response.message = detail
        return response

    def _srv_reset_leash(self, request, response):
        ok, detail = self.core.reset_leash()
        self.status.publish_events(self.core.drain_events())
        response.success = bool(ok)
        response.message = detail
        return response

    @property
    def enabled(self):
        return self.core.state == ST_ENABLED
