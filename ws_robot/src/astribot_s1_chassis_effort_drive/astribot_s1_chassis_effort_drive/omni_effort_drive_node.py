#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：X 型布局全向轮底盘力矩闭环驱动节点——治本方案的核心新增节点，替代
"VelocityControl 本体直控 + MecanumDrive 轮子仅视觉转动"两套独立运动学并行架构。

    /cmd_vel(不变) → 【本节点：全向轮逆解 → 四轮目标转速 → 轮速PID+摩擦前馈】
        → effort_controller/commands(Float64MultiArray)
        → ros2_control effort command_interface → gz_ros2_control GazeboSimSystem
        → 轮子关节(DART物理) → 轮地各向异性摩擦接触力 → 车身真实动力学响应 → /odom

本节点只负责"目标转速怎么算、力矩怎么闭环"这一层，不直接碰任何 gz-sim 插件、
不绕开轮子关节——真正的力传递、轮地接触摩擦完全交给物理引擎，这正是本节点存在
的意义（对应 astribot_sdk 设计方案第1/2/3节：单一力矩闭环链路，消除两套独立
运动学并行）。

!!! 运动学逆解符号方案必须标定，不能凭空定案 !!!：kinematics_sign_{vx,vy,wz}
三个参数数组(每个长度4，顺序RF/LF/RR/LR)编码了 ω_i=(1/r)*(sign_vx*vx +
sign_vy*vy + sign_wz*l*wz) 里的正负号——默认值是按现有 fdir1 对角线摩擦分组
(RF/LR一组、LF/RR一组)倒推的初始假设，真实符号取决于每个轮关节各不相同的物理
安装朝向，必须经"单轮测试"+"原地旋转测试"实测校正，不能改代码，只需要改
config yaml 里这三个数组的正负号。

!!! 安全降级规则(异常场景处理规则的落地) !!!：
  - /cmd_vel 超过 cmd_vel_timeout_sec 未更新 → 目标转速视为0(不是保持上一次的值，
    避免上游节点/导航栈挂掉后底盘继续冲下去)；
  - /joint_states 超过 joint_state_timeout_sec 未更新 → 直接下发全零 effort，
    平滑降速停机(说是"平滑"，是因为力矩置零不是瞬间刹停，车身靠惯性/剩余摩擦
    自然减速，不是硬急停)，防止在没有真实反馈的情况下让力矩指令凭"死数据"持续
    累积、越纠越偏；
  - 目标轮速超过 wheel_velocity_limit → 自动钳位，WARN日志，不崩溃；
  - 运动学/控制参数读取失败(极端情况，比如参数服务器返回异常类型) →
    使用硬编码默认值兜底，不直接抛异常崩掉节点；
  - 单轮持续跟踪误差超过 tracking_error_error_rad_s 并维持
    tracking_error_error_duration_sec 以上 → ERROR日志，
    auto_slowdown_on_tracking_error=true 时额外对目标速度整体打折(可选)。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64


# 四轮顺序，跟仓库既有 fdir1 对角线摩擦分组(RF/LR一组、LF/RR一组)、
# astribot_s1_controllers.yaml 里 wheel_effort_controller 的 joints 列表顺序
# 必须完全一致——这个顺序本身就是一份"接口契约"，改了任何一处都要三处同步改。
WHEEL_ORDER = ('RF', 'LF', 'RR', 'LR')
JOINT_NAMES = tuple(f'wheel_{w}_Joint' for w in WHEEL_ORDER)


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _sign(x):
    return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)


class _WheelLoop:
    """单轮速度环状态(PID积分项+上一次误差)，纯计算，不直接接触任何ROS接口——
    对应"运动学解算模块与控制闭环解耦"的编码规范：这个类可以脱离ROS单独单测。"""

    __slots__ = ('integral', 'prev_error')

    def __init__(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, target, measured, kp, ki, kd, tau_c, tau_v, deadband, tau_max, dt):
        error = target - measured

        tau_ff = 0.0
        if abs(target) > deadband:
            tau_ff = tau_c * _sign(target) + tau_v * measured

        derivative = (error - self.prev_error) / dt if dt > 1e-9 else 0.0
        tau_pid = kp * error + ki * self.integral + kd * derivative
        tau_unclamped = tau_pid + tau_ff
        tau = _clamp(tau_unclamped, -tau_max, tau_max)

        # anti-windup：只有没有饱和时才继续累积积分项，否则饱和后积分会一直
        # 往一个方向涨，clamp解除瞬间输出剧烈过冲。
        if tau == tau_unclamped:
            self.integral += error * dt

        self.prev_error = error
        return tau, error


class OmniEffortDriveNode(Node):

    def __init__(self):
        super().__init__('omni_effort_drive_node')

        # ---- 话题 ----
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('effort_command_topic', '/wheel_effort_controller/commands')

        # ---- 全向轮几何参数(统一走参数服务器，不写死；随轮径扫描测试改这里即可) ----
        self.declare_parameter('wheel_radius', 0.08)

        # ---- 逆解系数矩阵：ω_i = global_sign*(1/r)*(c_vx*vx + c_vy*vy + c_wz*wz) ----
        # !!! 重要：这不是"猜的符号"，是从 astribot_s1_torso_wheel.xacro 里四个轮关节
        # 真实的 origin rpy/xyz 直接算出来的（见 README 里的推导脚本）!!!
        # 本机器人四个轮子的**轴线在torso_base系里指向45°对角线**(RF轴线[0.707,-0.707,0]、
        # LF[0.707,0.707,0]、RR[-0.707,-0.707,0]、LR[-0.707,0.707,0])，
        # 也就是说这是一台 **X型布局全向轮(omni)** 底盘，不是"轮轴沿±y、辊子45°"的
        # 标准麦克纳姆布局——两者的逆解公式完全不同，这是之前把系数配错的根本原因。
        # 正确模型是全向轮投影模型：轮子只在自己的滚动方向 d_i(垂直于轴线)上着力，
        #   ω_i * r = d_i · (v_body + wz × p_i)
        # 展开后每个轮子的三个系数就是：
        #   c_vx = d_i.x,  c_vy = d_i.y,  c_wz = (p_i × d_i)_z   ← 力臂已经含在这里，
        # 所以下面**不再乘 wheel_geometry_l**（旧代码乘了0.4327，比真实力臂0.30大41%）。
        self.declare_parameter('wheel_coeff_vx', [-0.7071, 0.7071, -0.7071, 0.7071])
        self.declare_parameter('wheel_coeff_vy', [-0.7071, -0.7071, 0.7071, 0.7071])
        self.declare_parameter('wheel_coeff_wz', [-0.3060, -0.3060, -0.3024, -0.3024])
        # 整体正负号：上面的推导只保证四个轮子**相对关系**正确，"正ω到底让车往前
        # 还是往后"取决于关节正转方向和地面反作用力的约定，是一个全局符号——
        # 实测发现车身运动方向和cmd_vel相反时，只翻这一个参数即可，不要去动上面
        # 12个系数（那12个是几何算出来的，翻了就破坏相对关系）。
        self.declare_parameter('kinematics_global_sign', 1.0)

        # ---- 轮速PID ----
        # !!! 这里的声明默认值必须和 config/omni_effort_drive_params.yaml 保持一致 !!!
        # 原来这三个默认值是重构前那套(kp=2.0/ki=0.5/kd=0.02)，也就是**已知会发散**的值：
        # 稳定硬条件是 kp < 关节阻尼 d(=1.0)，kp=2.0 的环路增益 2.0 > 1，
        # 数学上必然发散→撞上力矩限幅→在 ±τ_max 之间 bang-bang 振荡。
        # 实测踩过：launch 的 params_file 被别的 yaml 顶掉时(共享 LaunchConfiguration 泄漏)，
        # 本节点悄悄退回这套默认值，四轮全部钉在 ±15N·m、净旋转力矩 60N·m，
        # 车身在没有任何 cmd_vel 的情况下自转 3.3rad/s，还连带把 MPPI 求解和
        # Nav2 进度检查一起搞崩 —— 而日志里看不出参数没加载。
        # 所以默认值一律取"已验证可用"的那套：配置缺失时退化成**能站住**的行为，
        # 而不是退化成一台会自己转起来的车。
        self.declare_parameter('pid_kp', 0.4)
        self.declare_parameter('pid_ki', 0.1)
        self.declare_parameter('pid_kd', 0.0)

        # ---- 摩擦前馈(库伦+粘性) ----
        # 同上：必须与 yaml 一致，且与 URDF 轮关节 <dynamics damping/friction> 对齐。
        # 粘性项就是用来抵消关节阻尼 d=1.0 的，配 0.02 只能提供 0.023N·m(小 50 倍)，
        # 稳态维持全压给 PID 去积分，是发散的另一个助推因素。
        self.declare_parameter('friction_coulomb_nm', 0.1)
        self.declare_parameter('friction_viscous_nm_s', 1.0)
        self.declare_parameter('friction_deadband_rad_s', 0.05)

        # ---- 强制限幅(所有力矩输出都必须clamp，编码规范硬要求) ----
        self.declare_parameter('wheel_effort_limit_nm', 15.0)
        self.declare_parameter('wheel_velocity_limit_rad_s', 40.0)

        # ---- 安全降级阈值 ----
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('joint_state_timeout_sec', 0.3)
        self.declare_parameter('control_period_sec', 0.01)  # 100Hz，对齐astribot_chassis.yaml

        # ---- 跟踪误差告警/可选自动限速 ----
        self.declare_parameter('warn_effort_ratio', 0.9)
        self.declare_parameter('tracking_error_error_rad_s', 3.0)
        self.declare_parameter('tracking_error_error_duration_sec', 1.0)
        self.declare_parameter('auto_slowdown_on_tracking_error', False)
        self.declare_parameter('auto_slowdown_scale', 0.5)

        self.declare_parameter('log_throttle_sec', 2.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        effort_command_topic = self.get_parameter('effort_command_topic').value

        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_wz = 0.0
        self._last_cmd_vel_time = None

        self._wheel_velocity = {name: 0.0 for name in JOINT_NAMES}
        self._wheel_position = {name: 0.0 for name in JOINT_NAMES}
        self._last_joint_state_time = None

        self._loops = {name: _WheelLoop() for name in JOINT_NAMES}
        self._error_high_since = {name: None for name in JOINT_NAMES}
        self._last_log_time = None

        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 10)
        self.create_subscription(JointState, joint_states_topic, self._joint_state_callback, 10)

        self._effort_pub = self.create_publisher(Float64MultiArray, effort_command_topic, 10)
        self._setpoint_pubs = {
            name: self.create_publisher(Float64, f'/wheel_velocity_setpoint/{wheel}', 10)
            for wheel, name in zip(WHEEL_ORDER, JOINT_NAMES)
        }
        self._effort_debug_pubs = {
            name: self.create_publisher(Float64, f'/wheel_effort/{wheel}', 10)
            for wheel, name in zip(WHEEL_ORDER, JOINT_NAMES)
        }

        period = self._safe_param('control_period_sec', 0.01)
        # ---- 启动自检：拿到的参数是不是"本节点的"参数 ----
        # 实测踩过：launch 的共享 params_file 被别的 yaml 顶掉后，本节点吃到的是
        # 探索协调器的 yaml —— 那份文件用 `/**:` 通配，会被正常加载且不报任何错，
        # 里面的 control_period_sec: 0.5 直接把 100Hz 控制环变成 2Hz，
        # 同时本节点自己的 PID/摩擦参数一条都没加载。三个"看起来无关"的故障
        # (底盘只有 2cm/s、MPPI 求解失败、控制器保不住 20Hz) 全是这一个根因。
        # 这两条自检就是为了让"参数没加载"这件事在启动时立刻可见，而不是靠猜。
        if period > 0.05:
            self.get_logger().error(
                'control_period_sec=%.3fs (%.1fHz) 远慢于本节点的设计值 0.01s(100Hz)。'
                '轮速PID的稳定性推导是按 100Hz 做的，这么慢必然发散并撞上力矩限幅，'
                '车身会在没有 cmd_vel 的情况下自己转起来。'
                '最常见原因是 launch 把**别的节点的 yaml** 传给了本节点'
                '(共享 LaunchConfiguration params_file 泄漏)，'
                '用 `pgrep -af omni_effort_drive_node | grep -o "params-file [^ ]*"` 核对。'
                % (period, 1.0 / period if period > 0 else float('inf')))
        kp_check = self._safe_param('pid_kp', 0.4)
        tau_v_check = self._safe_param('friction_viscous_nm_s', 1.0)
        if kp_check >= tau_v_check:
            self.get_logger().error(
                '轮速PID稳定条件不满足：pid_kp=%.3f 必须 < 粘性前馈/关节阻尼 %.3f。'
                '当前取值会让环路增益 >= 1，数学上必然发散→力矩 bang-bang 振荡。'
                % (kp_check, tau_v_check))
        self._timer = self.create_timer(period, self._control_step)

        self.get_logger().info(
            '全向轮力矩闭环驱动已启动：订阅 %s(目标)+%s(轮真实反馈)，'
            '按%.0fHz解算四轮effort，发布到 %s。轮半径=%.3fm，'
            '力矩限幅=±%.2fN·m，逆解全局符号=%+.0f。逆解系数是从URDF轮子安装rpy'
            '几何推导出来的(X型全向轮布局)，不是猜的符号。' % (
                cmd_vel_topic, joint_states_topic, 1.0 / period, effort_command_topic,
                self._safe_param('wheel_radius', 0.08),
                self._safe_param('wheel_effort_limit_nm', 15.0),
                self._safe_param('kinematics_global_sign', 1.0)))

    # ------------------------------------------------------------------
    # 运动学解算参数读取失败：加载默认参数，禁止直接崩溃(编码规范硬要求)。
    # ------------------------------------------------------------------
    def _safe_param(self, name, default):
        try:
            value = self.get_parameter(name).value
            if value is None:
                return default
            return value
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                '读取参数 %s 失败，使用默认值 %s：%s' % (name, default, exc))
            return default

    def _cmd_vel_callback(self, msg: Twist):
        try:
            self._target_vx = float(msg.linear.x)
            self._target_vy = float(msg.linear.y)
            self._target_wz = float(msg.angular.z)
            self._last_cmd_vel_time = self.get_clock().now()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('解析/cmd_vel出错，忽略本帧，保持上一次目标：%s' % exc)

    def _joint_state_callback(self, msg: JointState):
        try:
            name_to_vel = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
            name_to_pos = dict(zip(msg.name, msg.position)) if len(msg.position) == len(msg.name) else {}
            updated = False
            for joint in JOINT_NAMES:
                if joint in name_to_vel:
                    self._wheel_velocity[joint] = name_to_vel[joint]
                    updated = True
                if joint in name_to_pos:
                    self._wheel_position[joint] = name_to_pos[joint]
            if updated:
                self._last_joint_state_time = self.get_clock().now()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('解析/joint_states出错，本帧反馈丢弃：%s' % exc)

    # ------------------------------------------------------------------
    # 全向轮逆解：kinematics模块与下面的PID闭环模块解耦，可独立单测。
    # ω_i = global_sign * (1/r) * (c_vx*vx + c_vy*vy + c_wz*wz)
    # 系数含义与推导见 __init__ 里 wheel_coeff_* 的注释（力臂已含在c_wz里，
    # 这里不再额外乘轮距参数）。
    # ------------------------------------------------------------------
    def _inverse_kinematics(self, vx, vy, wz):
        r = self._safe_param('wheel_radius', 0.08)
        c_vx = self._safe_param('wheel_coeff_vx', [-0.7071, 0.7071, -0.7071, 0.7071])
        c_vy = self._safe_param('wheel_coeff_vy', [-0.7071, -0.7071, 0.7071, 0.7071])
        c_wz = self._safe_param('wheel_coeff_wz', [-0.3060, -0.3060, -0.3024, -0.3024])
        gsign = self._safe_param('kinematics_global_sign', 1.0)

        if r <= 1e-6:
            self.get_logger().error('wheel_radius<=0，逆解不成立，本帧输出全零目标转速')
            return {name: 0.0 for name in JOINT_NAMES}

        targets = {}
        for i, joint in enumerate(JOINT_NAMES):
            try:
                targets[joint] = gsign * (1.0 / r) * (
                    c_vx[i] * vx + c_vy[i] * vy + c_wz[i] * wz)
            except (IndexError, TypeError) as exc:
                self.get_logger().error(
                    '逆解系数数组下标%d读取失败(%s)，该轮目标转速置0' % (i, exc))
                targets[joint] = 0.0
        return targets

    def _control_step(self):
        now = self.get_clock().now()
        dt = self._safe_param('control_period_sec', 0.01)

        # ---- cmd_vel 陈旧检测：超时视为目标0，不是保持上一次的值 ----
        vx, vy, wz = self._target_vx, self._target_vy, self._target_wz
        cmd_timeout = self._safe_param('cmd_vel_timeout_sec', 0.5)
        if (self._last_cmd_vel_time is None or
                (now - self._last_cmd_vel_time).nanoseconds / 1e9 > cmd_timeout):
            vx = vy = wz = 0.0

        # ---- joint_states 陈旧检测：超时直接全零effort，平滑降速停机 ----
        joint_timeout = self._safe_param('joint_state_timeout_sec', 0.3)
        feedback_stale = (
            self._last_joint_state_time is None or
            (now - self._last_joint_state_time).nanoseconds / 1e9 > joint_timeout)

        try:
            targets = self._inverse_kinematics(vx, vy, wz)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('逆解整体异常，本帧全部目标转速置0：%s' % exc)
            targets = {name: 0.0 for name in JOINT_NAMES}

        v_limit = self._safe_param('wheel_velocity_limit_rad_s', 40.0)
        # 这些兜底值必须和 declare_parameter 里的一致(都是已验证可用的那套)。
        # 写成重构前的 kp=2.0/tau_v=0.02 等于埋了第二份发散配置：
        # 一旦哪天 _safe_param 真的走进兜底分支，车会自己转起来。
        kp = self._safe_param('pid_kp', 0.4)
        ki = self._safe_param('pid_ki', 0.1)
        kd = self._safe_param('pid_kd', 0.0)
        tau_c = self._safe_param('friction_coulomb_nm', 0.1)
        tau_v = self._safe_param('friction_viscous_nm_s', 1.0)
        deadband = self._safe_param('friction_deadband_rad_s', 0.05)
        tau_max = self._safe_param('wheel_effort_limit_nm', 15.0)
        warn_ratio = self._safe_param('warn_effort_ratio', 0.9)

        efforts = []
        for joint in JOINT_NAMES:
            target = targets.get(joint, 0.0)
            clamped_target = _clamp(target, -v_limit, v_limit)
            if abs(clamped_target - target) > 1e-6:
                self.get_logger().warn(
                    '%s 目标转速%.2frad/s超出限位±%.2f，已钳位' %
                    (joint, target, v_limit), throttle_duration_sec=2.0)

            if feedback_stale:
                # 没有真实反馈时绝不继续闭环下发力矩——直接置零，让物理引擎/惯性
                # 自然减速，防止"用陈旧数据算出的力矩"持续累积、越纠越偏。
                efforts.append(0.0)
                self._loops[joint].reset()
                continue

            measured = self._wheel_velocity[joint]
            tau, error = self._loops[joint].update(
                clamped_target, measured, kp, ki, kd, tau_c, tau_v, deadband, tau_max, dt)
            efforts.append(tau)

            if abs(tau) > warn_ratio * tau_max:
                self.get_logger().warn(
                    '%s 力矩%.2fN·m接近限值±%.2f的%.0f%%' %
                    (joint, tau, tau_max, warn_ratio * 100), throttle_duration_sec=2.0)

            self._check_tracking_error(joint, error, now)

            self._setpoint_pubs[joint].publish(Float64(data=clamped_target))
            self._effort_debug_pubs[joint].publish(Float64(data=tau))

        if feedback_stale and self._last_joint_state_time is not None:
            self.get_logger().warn(
                '/joint_states 已 %.2fs 未更新，全部轮子力矩置0(平滑降速停机)' %
                ((now - self._last_joint_state_time).nanoseconds / 1e9),
                throttle_duration_sec=2.0)

        self._effort_pub.publish(Float64MultiArray(data=efforts))

    def _check_tracking_error(self, joint, error, now):
        error_thresh = self._safe_param('tracking_error_error_rad_s', 3.0)
        error_duration = self._safe_param('tracking_error_error_duration_sec', 1.0)

        if abs(error) <= error_thresh:
            self._error_high_since[joint] = None
            return

        if self._error_high_since[joint] is None:
            self._error_high_since[joint] = now
            return

        elapsed = (now - self._error_high_since[joint]).nanoseconds / 1e9
        if elapsed >= error_duration:
            self.get_logger().error(
                '%s 跟踪误差%.2frad/s持续超限%.1fs以上(阈值%.2frad/s)' %
                (joint, error, elapsed, error_thresh), throttle_duration_sec=2.0)
            if self._safe_param('auto_slowdown_on_tracking_error', False):
                scale = self._safe_param('auto_slowdown_scale', 0.5)
                self._target_vx *= scale
                self._target_vy *= scale
                self._target_wz *= scale
                self.get_logger().warn(
                    '触发可选自动限速：目标速度整体乘 %.2f' % scale)


def main(args=None):
    rclpy.init(args=args)
    node = OmniEffortDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
