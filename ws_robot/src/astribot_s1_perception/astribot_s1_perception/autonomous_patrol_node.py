#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件用途：自主巡游建图节点——不依赖 Nav2 全套（本环境虽然装了 nav2-bringup，
但要接入完整代价地图/规划器/行为树需要额外一大批参数配置，超出"让机器人动起来把
仓库探索一遍建好图"这个具体需求的范围，这里用一个轻量级的反应式漫游策略：

    每个控制周期读一次融合后的 360° /scan，找到"最开阔的方向"（该方向上激光测距最远），
    因为本机器人是麦克纳姆轮全向底盘，不需要像差速轮那样先转向再前进——
    直接把 linear.x/y 设成指向那个最开阔方向的分量即可，同时叠加一个缓慢的持续自转
    （扩大扫到的角度范围、帮助建图），前方太近有障碍物时降速/纯自转避让。

    这不是频率意义上的"最优路径探索"（不会像 Nav2 explore_lite 那样跟踪未知区域边界），
    但对"让机器人动起来、把仓储环境逛一遍、让 SLAM Toolbox 有更多视角的扫描数据"这个
    目标来说足够了，且不需要额外安装/调试 Nav2 costmap+planner+behavior tree 那一整套。
    如果之后想升级成真正的前沿探索(frontier exploration)，可以在这个节点的位置换成
    `nav2_bringup` + 一个 frontier exploration 包（apt 里没有 ros-humble-explore-lite，
    需要自己源码编译），当前实现里预留了同样的 /cmd_vel 输出接口，替换很容易。

    !!! 实测踩坑记录（安全阀，非常重要）!!!：底盘实际驱动用的是 gz-sim 的
    VelocityControl 系统插件（直接对模型本体设定线速度/角速度，绕开了小尺寸轮子摩擦力学
    在本机物理引擎下的数值失效问题，见 astribot_s1.gazebo.xacro 里的详细说明），
    但这种"直接设定速度"的方式有个副作用：它不太理会真实的接触碰撞力——一旦机器人
    在巡游过程中撞上仓储货架之类的障碍物（雷达只装在躯干高度，可能没探测到伸出去的
    机械臂在别的高度蹭到货架），插件仍然按指令强行输出速度，和碰撞求解器的反作用力
    互相顶牛，实测出现过底盘位姿 z 坐标不断爬升、姿态四元数出现异常横滚/俯仰分量
    （相当于"飞起来/翻滚"）且不会自己恢复的情况。因为这是物理引擎数值层面的异常，
    仅靠 ROS 节点没法在事后把机器人"拽回地面"，这里加了一个安全监控：持续核对
    /odom 的 z 高度和姿态四元数，一旦明显偏离正常站立状态，立即持续下发零速度
    （防止巡游逻辑继续添乱、让情况更糟），并且大声报错提示需要人工重新
    `ros2 launch ...` 把机器人重新生成一遍——这不是"自动恢复"，是"止损"。
    同时把默认的巡游速度调低、安全距离调大，降低触发这类碰撞的概率
    （工程上的缓解措施，不是把根因彻底修复——更彻底的修复需要让避障逻辑感知机械臂的
    展开范围，或者把驱动机制换成"轮子摩擦力学真实生效"的方案，见 gazebo.xacro 里的
    後续优化建议，属于本方案范围之外的进一步工作）。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class AutonomousPatrolNode(Node):

    def __init__(self):
        super().__init__('autonomous_patrol_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('control_period', 0.2)
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('yaw_rate', 0.12)
        # 前方安全距离：这个扇区里如果有障碍物比这个近，强制减速/纯自转避让，
        # 不管"最开阔方向"算出来是哪——安全优先。默认值比早期版本调大了，
        # 降低撞上货架的概率（见文件头部"安全阀"说明）。
        self.declare_parameter('front_safety_distance', 0.9)
        self.declare_parameter('front_arc_deg', 70.0)
        # 连最开阔方向都比这个近，判定为"被困住/角落"，原地自转找出口，不再前进。
        self.declare_parameter('cornered_distance', 1.1)
        # 全向最近距离紧急阈值：不分方向，只要整圈雷达里有任意一个方向探测到比这个还近
        # 的障碍物，一律只原地自转、禁止平移——弥补"安全扇区只看选定移动方向"这一种
        # 检查方式仍可能被雷达安装高度之外的障碍物(比如伸展的机械臂蹭到货架)骗过的风险。
        self.declare_parameter('critical_stop_distance', 0.4)
        # 限制候选"最开阔方向"只在这个距离内取，避免总是死盯着老远一个门缝的对角线方向，
        # 导致巡游路线过于单一、不利于覆盖周边区域。
        self.declare_parameter('max_target_range', 6.0)
        # 一阶低通滤波系数，避免因为单帧扫描噪声导致速度指令抖动、机器人抽搐式转向。
        self.declare_parameter('smoothing_alpha', 0.25)
        # ---- 安全监控阈值 ----
        # !!! 实测踩坑记录 !!!：早期版本 max_height_deviation=0.15 / max_tilt_rad=0.35(20°)
        # 在真实撞击测试中太松——机器人撞上货架支腿后卡在一个俯仰角约18°的倾斜姿态里，
        # 没有继续恶化，但也没有被判定为异常，属于"卡住不动却不报警"的坏状态，
        # RViz 里能清楚看到机器人模型是斜的、扫描点乱飞。收紧到更贴近正常行驶抖动范围的
        # 数值，让这类"卡住但暂时没有持续恶化"的情况也能被尽早止损，而不是只抓
        # "持续爬升到夸张数值"这种更极端的情况。
        self.declare_parameter('normal_height', 0.134)
        self.declare_parameter('max_height_deviation', 0.06)
        self.declare_parameter('max_tilt_rad', 0.12)  # 约7°，比早期版本大幅收紧

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_callback, qos)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        self.latest_scan = None
        self.vx_filt = 0.0
        self.vy_filt = 0.0
        self.wz_filt = 0.0
        self.safety_tripped = False
        self.current_yaw = 0.0  # 从 /odom 持续更新，见下面 world-frame 换算说明

        period = self.get_parameter('control_period').value
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            '自主巡游节点已启动：读取 %s，反应式漫游 + 缓慢自转扩大扫描覆盖范围，输出到 %s；'
            '同时监控 %s 做异常姿态/高度的安全止损。' %
            (self.get_parameter('scan_topic').value, self.get_parameter('cmd_vel_topic').value,
             self.get_parameter('odom_topic').value))

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # 持续记录当前偏航角，供 control_loop 把"车体系下的最开阔方向"换算成
        # world 系下的速度分量用（原因见 control_loop 里的详细注释）。
        self.current_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if self.safety_tripped:
            return
        z = msg.pose.pose.position.z
        # 四元数转 roll/pitch（只需要判断是否明显倾斜，不需要完整姿态解算）
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        normal_height = self.get_parameter('normal_height').value
        max_dev = self.get_parameter('max_height_deviation').value
        max_tilt = self.get_parameter('max_tilt_rad').value

        if abs(z - normal_height) > max_dev or abs(roll) > max_tilt or abs(pitch) > max_tilt:
            self.safety_tripped = True
            self.get_logger().error(
                '!!! 安全监控触发止损：检测到异常姿态(z=%.3f, roll=%.3f, pitch=%.3f)，'
                '大概率是巡游中撞上了障碍物导致仿真物理异常（不是自动能恢复的问题）。'
                '已停止自主巡游节点继续下发速度指令，请检查 Gazebo 画面，'
                '必要时重新执行 ros2 launch 把机器人重新生成一遍。' % (z, roll, pitch))

    def control_loop(self):
        if self.safety_tripped:
            # 止损状态下持续发零速度，防止别的节点/残留指令继续让情况恶化。
            self.cmd_pub.publish(Twist())
            return

        msg = self.latest_scan
        if msg is None:
            return

        ranges = np.array(msg.ranges, dtype=np.float64)
        n = ranges.shape[0]
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        if not np.any(valid):
            # 完全没有有效读数（比如刚起步、雷达还没出数据），原地小幅自转等待数据，
            # 不盲目前冲。
            self._publish_smoothed(0.0, 0.0, self.get_parameter('yaw_rate').value)
            return

        # !!! 实测踩坑记录（这里曾经是个真实 bug，不是纸面猜测）!!!：早期版本的"前方安全
        # 距离"检查写死判断激光角度 0(车体朝向的正前方)附近的扇区，但麦克纳姆轮是全向轮，
        # 实际选定的移动方向 best_angle 可能和车体朝向(角度0)完全不是一回事——比如
        # "最开阔方向"是车体侧面或接近后方，那时候检查"正前方"这个扇区完全检查错了
        # 地方，等于对真正要走的那个方向"盲开"。实测复现过一次：机器人径直撞上了货架
        # 支腿，之后卡在原地一直保持一个不大不小的倾斜姿态（俯仰角约18°，没到旧的
        # 20°止损阈值），既不再往前走也没被判定为异常，属于"卡住了但没报警"的坏状态。
        # 修复：把安全扇区跟着 best_angle 走（检查"即将真正移动的那个方向"上是不是安全），
        # 而不是固定检查车体正前方；另外新增一个不分方向的"全向最近距离"检查——
        # 只要激光扫到的任何一个方向出现极近距离的读数，不管在不在选定的移动方向扇区里，
        # 都直接判定为高风险，只保留自转、不允许平移，防止某个方向的近距离障碍物因为
        # 恰好不在选定扇区内而被完全忽略掉。
        max_target_range = self.get_parameter('max_target_range').value
        # 候选"最开阔方向"只在 max_target_range 以内找，超出这个距离的都截断到这个值，
        # 这样如果四周都很空旷（比如仓库中央大空地），argmax 会在多个同样"很远"的方向里
        # 随机分布（numpy argmax 取第一个最大值，配合下面的缓慢自转，巡游方向会随时间
        # 自然漂移，不会卡死在同一个方向）。
        capped_ranges = np.where(valid, np.minimum(ranges, max_target_range), 0.0)
        best_idx = int(np.argmax(capped_ranges))
        best_range = capped_ranges[best_idx]
        best_angle = angles[best_idx]

        front_arc = math.radians(self.get_parameter('front_arc_deg').value) / 2.0
        # 安全扇区跟着"即将移动的方向"(best_angle)走，不是车体固定朝向的0度角——
        # 角度差要做 wrap-to-[-pi,pi]，否则 -170°和+170°这种数值上差很远但物理上
        # 几乎同一个方向的情况会被误判成"角度差很大、不在扇区内"。
        angle_diff = np.mod(angles - best_angle + math.pi, 2 * math.pi) - math.pi
        travel_mask = valid & (np.abs(angle_diff) <= front_arc)
        travel_min = float(np.min(ranges[travel_mask])) if np.any(travel_mask) else float('inf')

        # 全向最近距离：不分方向，只要整圈里有任何一个方向探测到极近障碍物就算危险——
        # 弥补"安全扇区只看选定移动方向"仍然可能被小范围盲区骗过的情况(比如障碍物正好卡在
        # 扇区边缘之外，或者机械臂/其它部位在雷达安装高度之外碰到了东西)。
        any_min = float(np.min(ranges[valid]))

        safety_distance = self.get_parameter('front_safety_distance').value
        cornered_distance = self.get_parameter('cornered_distance').value
        max_speed = self.get_parameter('max_linear_speed').value
        yaw_rate = self.get_parameter('yaw_rate').value
        critical_distance = self.get_parameter('critical_stop_distance').value

        if any_min < critical_distance:
            # 全向最近距离已经进入"紧急"范围：不管选定方向是哪，一律只原地自转，
            # 不做任何平移——优先避免撞得更实、卡得更死。
            vx, vy, wz = 0.0, 0.0, yaw_rate
        elif best_range < cornered_distance:
            # 四周（至少是最开阔的那个方向）都很局促，判定被困住/在角落里，原地自转找出口，
            # 不前进——避免硬闯墙角。
            vx, vy, wz = 0.0, 0.0, yaw_rate
        else:
            speed = max_speed
            if travel_min < safety_distance:
                # 即将移动的方向上有障碍但别的方向还开阔：降速前进（把速度往"最开阔方向"
                # 上打折），让转向/绕障碍先生效，不是硬刹车（麦克纳姆轮本来就能斜着走，
                # 不需要完全停下来再转向）。
                speed *= max(0.0, travel_min / safety_distance)
            # !!! 实测踩坑记录（这里是导致"一直在自转、位置却几乎钉死不动"的真正原因，
            # 花了不少功夫才排查出来）!!!：best_angle 是从 /scan 算出来的角度，
            # 而 /scan 是 pointcloud_to_laserscan 投影到 astribot_torso_base 这个
            # **车体自身**坐标系下的结果——也就是说 best_angle 是"车体系下的方位角"，
            # 会随着车身自转而在数值上跟着改变（哪怕现实里那个最开阔的方向压根没变）。
            # 但用 `strings` 翻查 gz-sim 的 velocity-control-system 插件二进制发现，
            # 它内部实际写入的分量类型叫 `WorldLinearVelocityCmd`/`WorldAngularVelocityCmd`
            # ——插件是把 /cmd_vel 的 linear.x/y **当成 world 系下的速度分量**直接使用，
            # 不是常见移动机器人 cmd_vel 语义里那种"车体自身坐标系"。早期版本直接把
            # cos(best_angle)/sin(best_angle) 发布出去，等于每个控制周期都把"车体系角度"
            # 当成"world系角度"来用；而本节点在平移的同时还叠加了一个持续自转(wz)，
            # 车体自身朝向一直在变，于是 best_angle(车体系) 每个周期也跟着规律性漂移，
            # 换算出的"world系速度方向"就跟着车身转速一起连续扫过整个圆周——一整圈转下来，
            # 各个方向的推力互相抵消，车身当然测得到的是"一直在原地打转，几乎不挪窝"。
            # 修复：把 best_angle 先加上当前航向角 self.current_yaw（从 /odom 持续更新），
            # 换算成真正的 world 系角度，再拿去算 vx/vy，才符合 VelocityControl 插件的
            # 实际语义。（wz 本身是绕Z轴的角速度标量，车体系和world系数值相同，不受影响，
            # 不需要跟着改。）
            world_angle = best_angle + self.current_yaw
            vx = math.cos(world_angle) * speed
            vy = math.sin(world_angle) * speed
            # 叠加一个缓慢恒定自转：不影响全向平移方向的选择（麦克纳姆轮平移和自转独立），
            # 但能让机器人持续转动朝向、给双 Mid-360 雷达和头部相机更多角度的观测数据，
            # 对 SLAM 建图覆盖率有帮助。
            wz = yaw_rate * 0.5

        self._publish_smoothed(vx, vy, wz)

    def _publish_smoothed(self, vx, vy, wz):
        alpha = self.get_parameter('smoothing_alpha').value
        self.vx_filt = alpha * vx + (1 - alpha) * self.vx_filt
        self.vy_filt = alpha * vy + (1 - alpha) * self.vy_filt
        self.wz_filt = alpha * wz + (1 - alpha) * self.wz_filt

        cmd = Twist()
        cmd.linear.x = self.vx_filt
        cmd.linear.y = self.vy_filt
        cmd.angular.z = self.wz_filt
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发一个零速度，避免节点被杀掉之后机器人保持着最后一个非零速度指令继续动
        # （回顾 VelocityControl 的行为：它是"设定即保持"，不会因为没有新消息就自动归零）。
        try:
            stop_cmd = Twist()
            node.cmd_pub.publish(stop_cmd)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
