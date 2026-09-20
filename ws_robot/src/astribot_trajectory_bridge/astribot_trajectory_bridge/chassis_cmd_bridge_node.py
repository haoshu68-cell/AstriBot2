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
内环 250Hz（4ms）与外环 10Hz 分组调度，命令订阅也有独立的回调组。
核心状态更新和使能/停用操作用同一把锁串行，防止换帧积分与停车服务交错；
外环的 ROS 发布和日志在核心锁外执行。

但 GIL 仍在 —— 这是单进程多节点方案（S-1）的真实代价，不是能靠分组消除的。
所以内环额外监控实际周期，超阈上报 LOOP_OVERRUN，让抖动可见而不是靠感觉。
"""

import json
from time import perf_counter

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger

from astribot_trajectory_bridge.callback_layout import CHASSIS_GROUPS, make_groups
from astribot_trajectory_bridge.chassis_bridge_core import (
    ChassisBridgeConfig,
    ChassisBridgeCore,
    ST_ENABLED,
    ST_LEASH_TRIPPED,
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

    CALLBACK_GROUPS = CHASSIS_GROUPS

    def __init__(self, session, node_name='chassis_cmd_bridge'):
        super().__init__(node_name)
        self._declare_params()
        cfg = self._build_config()

        self._clock_port = RosClock(self)
        self._pose = TfPosePort(self, cfg.map_frame, cfg.base_frame)
        self.core = ChassisBridgeCore(cfg, session, self._pose, self._clock_port)
        self.status = StatusReporter(self, node_name=node_name)

        self._write_allowed = self._check_write_gate(session, cfg)

        groups = make_groups(CHASSIS_GROUPS, MutuallyExclusiveCallbackGroup,
                             'chassis_cmd_bridge')
        inner_group = groups['inner']
        outer_group = groups['outer']
        srv_group = groups['srv']
        cmd_group = groups['cmd']

        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self._on_cmd_vel, 10, callback_group=cmd_group)

        if cfg.require_fresh_scan:
            self.create_subscription(
                LaserScan, self.get_parameter('scan_topic').value,
                self._on_scan, qos_profile_sensor_data,
                callback_group=cmd_group)
        else:
            self.get_logger().warning(
                'require_fresh_scan=False：/scan 时效性联锁**已关闭**。'
                '感知失效时底盘不会自动停 —— 只应在无感知的台架测试里这样配')
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

        self._rate_report_period_ticks = max(1, int(round(cfg.outer_rate * 10.0)))
        self._rate_report_countdown = self._rate_report_period_ticks
        self._state_report_period_ticks = max(1, int(round(cfg.outer_rate)))
        self._state_report_countdown = self._state_report_period_ticks

        vel_period = float(self.get_parameter('vel_trace_period_sec').value)
        self._vel_report_period_ticks = max(
            1, int(round(cfg.outer_rate * vel_period)))
        self._vel_report_countdown = self._vel_report_period_ticks

        self._last_inner_time = None
        self._overrun_count = 0
        self._loop_overrun_factor = self.get_parameter('loop_overrun_factor').value

        self.get_logger().info(
            '底盘桥接已启动：part=%s freq=%.1fHz 外环=%.1fHz 口径=%s '
            'leash=(%.3fm, %.3frad) 导航反馈=上层 x/y/yaw积分=逐帧pose同步重置 '
            '有限前瞻=(xy %.3fs, yaw %.3fs; 上界 %.3fm/%.3frad) 位姿源=%s 写通路=%s。'
            '启动即停用，需调 ~/enable。'
            % (cfg.part_name, cfg.freq, cfg.outer_rate, cfg.input_frame,
               cfg.leash_xy_m, cfg.leash_theta_rad,
               cfg.pose_preview_xy_sec, cfg.pose_preview_theta_sec,
               min(cfg.pose_preview_max_xy_m, 0.95 * cfg.leash_xy_m),
               min(cfg.pose_preview_max_theta_rad, 0.95 * cfg.leash_theta_rad),
               cfg.pose_source,
               '允许' if self._write_allowed else '被拒绝'))


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
        d('max_tick_dt_sec', 0.04)
        d('pose_preview_xy_sec', 0.5)
        d('pose_preview_theta_sec', 0.5)
        d('pose_preview_max_xy_m', 0.20)
        d('pose_preview_max_theta_rad', 0.34)
        d('leash_xy_m', 0.25)
        d('leash_theta_rad', 0.35)
        d('require_manual_reset', True)
        d('enable_slam_correction', False)  # 兼容旧 false；true 在配置校验时拒绝
        d('pose_source', 'slam')
        d('map_frame', 'map')
        d('base_frame', 'astribot_torso_base')
        d('outer_rate', 10.0)
        d('slam_max_age_sec', 0.5)
        d('slam_jump_threshold_m', 0.30)
        d('require_slam_to_enable', False)
        d('slam_loss_grace_sec', 2.0)
        d('odom_drift_window_sec', 2.0)
        d('odom_drift_warn_m', 0.15)
        d('loop_overrun_factor', 1.5)
        d('vel_trace_period_sec', 1.0)
        d('scan_topic', '/scan')
        d('require_fresh_scan', True)
        d('scan_max_age_sec', 0.5)
        d('scan_loss_grace_sec', 2.0)
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
            max_tick_dt_sec=g('max_tick_dt_sec'),
            pose_preview_xy_sec=g('pose_preview_xy_sec'),
            pose_preview_theta_sec=g('pose_preview_theta_sec'),
            pose_preview_max_xy_m=g('pose_preview_max_xy_m'),
            pose_preview_max_theta_rad=g('pose_preview_max_theta_rad'),
            leash_xy_m=g('leash_xy_m'), leash_theta_rad=g('leash_theta_rad'),
            require_manual_reset=g('require_manual_reset'),
            enable_slam_correction=g('enable_slam_correction'),
            pose_source=g('pose_source'),
            map_frame=g('map_frame'), base_frame=g('base_frame'),
            outer_rate=g('outer_rate'), slam_max_age_sec=g('slam_max_age_sec'),
            slam_jump_threshold_m=g('slam_jump_threshold_m'),
            require_slam_to_enable=g('require_slam_to_enable'),
            slam_loss_grace_sec=g('slam_loss_grace_sec'),
            odom_drift_window_sec=g('odom_drift_window_sec'),
            odom_drift_warn_m=g('odom_drift_warn_m'),
            require_fresh_scan=g('require_fresh_scan'),
            scan_max_age_sec=g('scan_max_age_sec'),
            scan_loss_grace_sec=g('scan_loss_grace_sec'))


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


    def _on_cmd_vel(self, msg):
        self.core.submit_twist(msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_scan(self, msg):      # noqa: ARG002 —— 只关心"来了一帧"，不看内容
        self.core.submit_scan_seen()

    def _inner_tick(self):
        if not self._write_allowed:
            return
        now = perf_counter()
        enabled = self.core.state == ST_ENABLED
        if enabled and self._last_inner_time is not None:
            period = now - self._last_inner_time
            self.core.timing.add('callback_period', period)
            target = 1.0 / self.core.cfg.freq
            if period > target * self._loop_overrun_factor:
                self._overrun_count += 1
                self.core.timing.overrun(period)
        self._last_inner_time = now if enabled else None

        try:
            self.core.inner_tick()
        except Exception as exc:      # noqa: BLE001 —— 单拍异常不能让定时器死掉
            self.get_logger().error('内环异常：%s' % exc)
            self.status.publish('SDK_CALL_FAILED', '内环异常：%s' % exc)
        self.status.publish_events(self.core.drain_events())
        if enabled:
            self.core.timing.add('callback_work', perf_counter() - now)

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
        self._report_tick_rate()
        self._report_vel_trace()
        self._state_report_countdown -= 1
        if self._state_report_countdown <= 0:
            self._state_report_countdown = self._state_report_period_ticks
            count, peak = self.core.timing.consume_overruns()
            if count:
                self.status.publish(
                    'LOOP_OVERRUN',
                    '使能期间内环超期 %d 次，窗口最大 %.4fs（累计 %d 次）'
                    % (count, peak, self._overrun_count), peak, 1.0 / self.core.cfg.freq)
            self._report_control_state()

    def _report_control_state(self):
        """Refresh the existing status channel so late subscribers see a latched stop."""
        if self.core.state == 'ENABLED':
            self.status.publish('OK', 'enabled')
            return
        codes = {'DISABLED': 'NOT_ENABLED', 'LEASH_TRIPPED': 'LEASH_TRIPPED',
                 'STOPPED_NO_POSE': 'SLAM_LOST_STOPPED', 'STOPPED_STALE_SCAN': 'SCAN_LOST_STOPPED'}
        code = codes.get(self.core.state)
        if code:
            stopped = self.core.last_stop
            self.status.publish(code, stopped[1] if stopped else 'disabled',
                                stopped[2] if stopped else 0.0, stopped[3] if stopped else 0.0)

    def _report_tick_rate(self):
        """周期性打印内环**实测**拍率。

        为什么值得专门打一行日志：在这行之前，唯一能看到拍率的东西是
        LOOP_OVERRUN，而它只在周期超阈时上报（截尾样本）。据它的周期均值反推
        出的"内环 91Hz、速度只剩 36%"是错的 —— 时间账反解的下界是 ≥157Hz。
        这里出的是 count / 墙钟时长，不需要任何推断。

        挂在外环上而不是新开定时器：新开定时器就要新增回调组，回调组数量是
        执行线程数的下限（见 callback_layout），不值得为一行日志动那套。
        """
        self._rate_report_countdown -= 1
        if self._rate_report_countdown > 0:
            return
        self._rate_report_countdown = self._rate_report_period_ticks
        timing = self.core.timing.consume()
        if timing:
            self.get_logger().info('BRIDGE_TIMING ' + json.dumps(
                {'state': self.core.state, 'clock': 'monotonic_wall',
                 'target_hz': self.core.cfg.freq,
                 'window': 'since_previous_report_or_enable', 'timing': timing},
                separators=(',', ':')))
        st = self.core.tick_stats()
        gap = self.core.consume_tick_window_gap()
        if st.count == 0:
            return
        if not st.live:
            stop = self.core.last_stop
            if stop is None:
                why = ('当前状态 %s，本次启动以来未发生过停车事件'
                       % self.core.state)
            else:
                why = ('当前状态 %s；最近一次停车原因：%s'
                       '（metric_1=%.4f metric_2=%.4f）'
                       % (self.core.state, stop[1], stop[2], stop[3]))
                if self.core.state == ST_LEASH_TRIPPED:
                    why += ('。**这是闩锁态**：require_manual_reset=%s，'
                            '不调 ~/reset_leash 就永远不会再动'
                            % self.core.cfg.require_manual_reset)
            self.get_logger().info(
                '内环已停用；%s。上一段使能期间实测拍率 %.1fHz（%d 拍，'
                '钳位 %d 次）。**这是历史值，不是当前拍率。**'
                % (why, st.rate_hz, st.count, st.clamp_count))
            return
        self.get_logger().info(
            '内环实测拍率 %.1fHz（标称 %.1f，比例 %.2f）平均步长 %.4fs '
            '本段累计 %d 拍，钳位 %d 次(%.2f%%)。'
            '本窗 %d 拍最大间隔 %.4fs(=%.1fHz 瞬时，超 2× 标称的 %d 拍)，'
            '本段最大间隔 %.4fs。'
            '★ 拍率只反映调度 —— 积分已改用实测 dt，拍率低不再等于速度损失。'
            '★ 均值查不出瞬时停顿，所以**最大间隔**才是判"有没有断供"的量：'
            'EtherCAT 的「电机长时间没有收到指令」只看间隔，不看均值。'
            '★ 统计窗口在每次 enable 时重开，所以均值是**本段**的。'
            % (st.rate_hz, self.core.cfg.freq,
               st.rate_hz / self.core.cfg.freq if self.core.cfg.freq else 0.0,
               st.mean_dt, st.count, st.clamp_count, 100.0 * st.clamp_ratio,
               gap.ticks, gap.max_dt,
               (1.0 / gap.max_dt) if gap.max_dt > 0.0 else 0.0,
               gap.over_count, st.max_dt))

    def _report_vel_trace(self):
        """记录输入、转换后的积分速度及停车置零；路径长和净位移分开统计。"""
        if self._vel_report_period_ticks <= 0:
            return
        self._vel_report_countdown -= 1
        if self._vel_report_countdown > 0:
            return
        self._vel_report_countdown = self._vel_report_period_ticks
        tr = self.core.consume_vel_trace()
        if tr.ticks == 0:
            return

        cmd_speed = tr.cmd_path / tr.wall if tr.wall > 0.0 else 0.0
        act_speed = tr.act_net / tr.wall if tr.wall > 0.0 else 0.0
        straight = tr.cmd_net / tr.cmd_path if tr.cmd_path > 1e-9 else 1.0
        self.get_logger().info(
            '[桥接速度链] %d拍/%.2fs | 输入 %.3f -> 积分 %.3f m/s(峰值) | '
            '角速度 输入 %.3f -> 积分 %.3f rad/s(峰值) | '
            '联锁置零 %d 拍 | 指令路径 %.4fm(净 %.4fm 直度%.2f) '
            '实际净位移 %.4fm | '
            '指令均速 %.3f 实际均速 %.3f m/s | dθ 指令 %.4f 实际 %.4f rad | '
            '角速度积分 %.4f rad pose换帧 %d 本帧积分=(%.4fm, %.4fm, %.4frad) | '
            '目标领先峰值=(%.4fm, %.4frad)'
            % (tr.ticks, tr.wall,
               tr.in_peak, tr.local_peak, tr.in_wz_peak, tr.local_wz_peak,
               tr.zeroed_ticks,
               tr.cmd_path, tr.cmd_net, straight,
               tr.act_net, cmd_speed, act_speed,
               tr.dtheta_cmd, tr.dtheta_act,
               tr.dtheta_integrated, tr.pose_rebases,
               tr.frame_dx, tr.frame_dy, tr.frame_dtheta,
               tr.lead_xy_peak, tr.lead_theta_peak))

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
