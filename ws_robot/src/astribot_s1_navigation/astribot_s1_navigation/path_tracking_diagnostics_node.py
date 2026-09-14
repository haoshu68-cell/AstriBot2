# Copyright 2026 Astribot.
import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

IDLE_NO_GOAL = '空闲(无路径，非故障)'
PLANNER_NO_OUTPUT = '局部规划器无输出'
SMOOTHER_BLOCKED = '速度平滑器拦截'
BODY_TO_WORLD_BLOCKED = '车体->世界转换断开'
ARM_COUPLING_BLOCKED = '臂-底盘耦合限速'
CHASSIS_NOT_RESPONDING = '底盘不响应(有指令无轮速)'
SLIPPING_OR_BLOCKED = '打滑或被卡住(轮子转但不位移)'
CMD_BUT_NO_MOTION = '有指令但无位移(轮速不可测，底盘不响应与打滑无法区分)'
HEALTHY = '正常'

STUCK_VERDICTS = (
    PLANNER_NO_OUTPUT, SMOOTHER_BLOCKED, BODY_TO_WORLD_BLOCKED,
    ARM_COUPLING_BLOCKED, CHASSIS_NOT_RESPONDING, SLIPPING_OR_BLOCKED,
    CMD_BUT_NO_MOTION,
)


def classify(snap):
    """纯函数判定，无副作用，便于离线复核。

    顺序从链路上游到下游，第一个断点即结论 —— 下游没输出多半是上游没给，
    所以必须按顺序判，不能各自独立判。
    """
    eps = snap['vel_epsilon']
    raw_active = snap['raw_rate'] > 0.0 and snap['raw_peak'] > eps
    smoothed_active = snap['smoothed_rate'] > 0.0 and snap['smoothed_peak'] > eps
    pre_active = snap['pre_rate'] > 0.0 and snap['pre_peak'] > eps
    cmd_active = snap['cmd_rate'] > 0.0 and snap['cmd_peak'] > eps
    wheels_turning = snap['wheel_peak'] > snap['wheel_epsilon']
    moving = snap['robot_speed'] > snap['motion_epsilon']

    if not snap['has_path'] and not raw_active:
        return IDLE_NO_GOAL
    if not raw_active:
        return PLANNER_NO_OUTPUT
    if not smoothed_active:
        return SMOOTHER_BLOCKED
    if not pre_active:
        return BODY_TO_WORLD_BLOCKED
    if not cmd_active:
        return ARM_COUPLING_BLOCKED
    if not snap['wheel_known']:
        return HEALTHY if moving else CMD_BUT_NO_MOTION
    if not wheels_turning:
        return CHASSIS_NOT_RESPONDING
    if not moving:
        return SLIPPING_OR_BLOCKED
    return HEALTHY


class ChannelMonitor:
    """一路速度话题的滑窗统计。只存时间戳和幅值，不存整条消息 ——
    诊断器是在系统已经出问题时才开的，本身不能成为负担。"""

    def __init__(self, window_sec):
        self.window = window_sec
        self.stamps = deque()
        self.peak_lin = 0.0
        self.peak_ang = 0.0

    def record(self, now_sec, vx, vy, wz):
        self.stamps.append(now_sec)
        self.peak_lin = max(self.peak_lin, math.hypot(vx, vy))
        self.peak_ang = max(self.peak_ang, abs(wz))
        self._prune(now_sec)

    def _prune(self, now_sec):
        while self.stamps and (now_sec - self.stamps[0]) > self.window:
            self.stamps.popleft()

    def rate(self, now_sec):
        self._prune(now_sec)
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        if span <= 0.0:
            return 0.0
        return (len(self.stamps) - 1) / span

    def reset_peaks(self):
        self.peak_lin = 0.0
        self.peak_ang = 0.0


class PathTrackingDiagnostics(Node):
    def __init__(self):
        super().__init__('path_tracking_diagnostics_node')
        window = self.declare_parameter('window_sec', 3.0).value
        self.period = self.declare_parameter('report_period_sec', 1.0).value
        self.only_when_stuck = self.declare_parameter('only_when_stuck', False).value
        self.vel_eps = self.declare_parameter('vel_epsilon', 1e-3).value
        self.wheel_eps = self.declare_parameter('wheel_epsilon', 0.05).value
        self.motion_eps = self.declare_parameter('motion_epsilon', 0.02).value
        self.wheel_prefix = self.declare_parameter('wheel_joint_prefix', 'wheel_').value
        self.path_stale_sec = self.declare_parameter('path_stale_sec', 2.0).value

        raw_t = self.declare_parameter('raw_topic', '/cmd_vel_nav_body_raw').value
        sm_t = self.declare_parameter('smoothed_topic', '/cmd_vel_nav_body').value
        pre_t = self.declare_parameter('pre_coupling_topic', '/cmd_vel_pre_arm_coupling').value
        cmd_t = self.declare_parameter('cmd_topic', '/cmd_vel').value
        path_t = self.declare_parameter('path_topic', '/plan').value
        odom_t = self.declare_parameter('odom_topic', '/odom').value
        js_t = self.declare_parameter('joint_states_topic', '/joint_states').value

        self.raw = ChannelMonitor(window)
        self.smoothed = ChannelMonitor(window)
        self.pre = ChannelMonitor(window)
        self.cmd = ChannelMonitor(window)

        self.path_points = 0
        self.last_path_sec = -1e9
        self.robot_speed = 0.0
        self.wheel_peak = 0.0
        self.wheel_known = False

        self.create_subscription(Twist, raw_t, self._mk(self.raw), 10)
        self.create_subscription(Twist, sm_t, self._mk(self.smoothed), 10)
        self.create_subscription(Twist, pre_t, self._mk(self.pre), 10)
        self.create_subscription(Twist, cmd_t, self._mk(self.cmd), 10)
        self.create_subscription(Path, path_t, self._on_path, 1)
        self.create_subscription(Odometry, odom_t, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, js_t, self._on_js, 10)

        self.create_timer(self.period, self._report)
        self.get_logger().info(
            f'路径跟踪诊断已启动。四段速度链路: {raw_t} -> {sm_t} -> {pre_t} -> {cmd_t}；'
            f'另监视 {js_t}(轮速) 与 {odom_t}(实位移)。'
            f'周期 {self.period:.1f}s 滑窗 {window:.1f}s')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _mk(self, mon):
        def cb(msg):
            mon.record(self._now(), msg.linear.x, msg.linear.y, msg.angular.z)
        return cb

    def _on_path(self, msg):
        self.path_points = len(msg.poses)
        self.last_path_sec = self._now()

    def _on_odom(self, msg):
        self.robot_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def _on_js(self, msg):
        peak = 0.0
        n = min(len(msg.name), len(msg.velocity))
        for i in range(n):
            if msg.name[i].startswith(self.wheel_prefix):
                self.wheel_known = True
                peak = max(peak, abs(msg.velocity[i]))
        self.wheel_peak = peak

    def _report(self):
        t = self._now()
        snap = {
            'has_path': self.path_points > 0 and (t - self.last_path_sec) < self.path_stale_sec,
            'raw_rate': self.raw.rate(t), 'raw_peak': self.raw.peak_lin,
            'smoothed_rate': self.smoothed.rate(t), 'smoothed_peak': self.smoothed.peak_lin,
            'pre_rate': self.pre.rate(t), 'pre_peak': self.pre.peak_lin,
            'cmd_rate': self.cmd.rate(t), 'cmd_peak': self.cmd.peak_lin,
            'wheel_peak': self.wheel_peak, 'robot_speed': self.robot_speed,
            'wheel_known': self.wheel_known,
            'vel_epsilon': self.vel_eps, 'wheel_epsilon': self.wheel_eps,
            'motion_epsilon': self.motion_eps,
        }
        verdict = classify(snap)
        stuck = verdict in STUCK_VERDICTS
        if self.only_when_stuck and not stuck:
            self._reset()
            return

        wheel_txt = (f'{snap["wheel_peak"]:.3f}rad/s'
                     if snap['wheel_known'] else 'n/a(无 velocity 字段)')
        line = (
            f'[跟踪诊断] {verdict} | '
            f'路径{"在" if snap["has_path"] else "无"}({self.path_points}点) | '
            f'raw {snap["raw_rate"]:.1f}Hz/{snap["raw_peak"]:.3f} -> '
            f'smooth {snap["smoothed_rate"]:.1f}Hz/{snap["smoothed_peak"]:.3f} -> '
            f'preCpl {snap["pre_rate"]:.1f}Hz/{snap["pre_peak"]:.3f} -> '
            f'cmd {snap["cmd_rate"]:.1f}Hz/{snap["cmd_peak"]:.3f} | '
            f'轮速峰值 {wheel_txt} | '
            f'实速 {snap["robot_speed"]:.3f}m/s'
        )
        if stuck:
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)

        if snap['has_path'] and 0.0 < snap['raw_rate'] < 12.0:
            self.get_logger().warn(
                f'[跟踪诊断] 局部规划器输出仅 {snap["raw_rate"]:.1f}Hz'
                f'（controller_server 期望 20Hz），控制环可能吃不住 CPU，'
                f'跟踪质量会随之恶化')
        self._reset()

    def _reset(self):
        for m in (self.raw, self.smoothed, self.pre, self.cmd):
            m.reset_peaks()


def main():
    rclpy.init()
    node = PathTrackingDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
