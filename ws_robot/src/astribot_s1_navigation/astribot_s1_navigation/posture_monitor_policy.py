#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""姿态止损监控的**纯判定逻辑**，不依赖 ROS，可离线测。

为什么单独抽出来
================
这套判定原先内联在 cmd_vel_body_to_world_node.odom_callback 里，于是它的两个
缺陷都只能在实机上被发现，而实机上又恰好因为第三个原因看不出来：

  1. 那个节点用 `create_subscription(Odometry, topic, cb, 10)` —— depth=10 的
     **默认 QoS 就是 RELIABLE**。实机 /odom 的发布者是 BEST_EFFORT，
     两者不兼容 -> 回调**一帧都没执行过**（实测 RELIABLE 0 帧 / BEST_EFFORT 704 帧
     @50Hz）。也就是说这个安全监控在实机上一直是死代码。
  2. normal_height=0.134 是**仿真**的值（Gazebo 里 world z=0 不是地面）。
     实机 /odom 是轮式里程计、只暴露 3-DOF，z/roll/pitch **恒等于 0**。
     于是 |0 - 0.134| = 0.134 > max_height_deviation(0.06) —— 判定为异常姿态。
  3. 缺陷 1 掩盖了缺陷 2。谁把 QoS "顺手修好"（一个看起来完全正确的修法），
     节点就会在收到**第一帧** /odom 时 safety_tripped，然后**永久**把 /cmd_vel
     归零；而它只打一行 ERROR，文案还是"请检查 Gazebo 画面" —— 在实机上
     这条信息把人指向完全错误的方向。而且 safety_tripped 没有复位路径。

所以这里做两件事：
  · 判定拆成纯函数，能被单测和变异测试覆盖
  · 增加"数据源退化"判定：轮式里程计不携带姿态时，**明确报告监控不可用**，
    而不是把"没有姿态信息"误判成"姿态异常"

!!! 上面第 2 条的前提已经变了（2026-09-03）!!!
"实机 /odom 是轮式里程计、只有 3-DOF"曾经是对的，现在不是了：/odom 改由
tf_to_odom_node 从 SLAM 的 map->astribot_torso_base 变换导出，携带真实 6-DOF。
实测 30s：z 峰峰值 0.0034m、roll 0.0042~0.0049、pitch 0.0050~0.0057 rad。
于是退化判定不再命中，监控**真的会做阈值判定** —— 而这暴露了原设计的第四个缺陷：

  4. normal_height 是一个**绝对 z 常数**，但 z 只在某个具体 odom 坐标系里才有
     意义。Gazebo 里 world z=0 不是地面(躯干基座静止在 0.134)；SLAM map 里
     z=0 是**开机那一刻的位姿**。同一个常数不可能同时对两个系成立，实测
     |−0.0011 − 0.1340| = 0.1351 > 0.06，第一帧就止损、/cmd_vel 永久归零
     （实测 30s 内 1668 帧全零）。
     把它手改成 0.0 只是把常数换成另一个同样和坐标系绑死的常数。

所以本模块现在做四件事：
  · 判定拆成纯函数，能被单测和变异测试覆盖
  · 数据源退化判定：源不携带姿态时**明确报告监控不可用**，而不是误判成姿态异常
  · **基准自标定**：基准高度从开机后头 min_samples 帧学出来（那段窗口本来就
    不做判定），而不是写死一个跟坐标系绑定的常数。见 calibrate_datum。
  · **止损不再是单帧即永久锁死**：要 K 连续帧超限才止损（挡住 SLAM 回环
    修正造成的单帧 z 跳变），且允许有上限的自动复位。见 PostureLatch。

仍然成立的边界：这个监控测的是 SLAM 位姿里的 z/roll/pitch，它测不出
"SLAM 自己算错了"。真正独立的倾倒检测要用 IMU（/astribot_whole_body/chassis_imu、
/livox/imu_front、/livox/imu_back）。本模块的职责是**不说谎**：
能测就测，不能测就明说不能测。
"""

#: 判定数据源是否退化时，至少要看这么多帧。
#: 取 20 而不是 1~2：单帧恰好为 0 是正常的（机器人本来可能水平且 z 就是 0），
#: 连续多帧**一点变化都没有**才说明这个源不携带信息。
#: 实机 /odom 是 50Hz，20 帧 = 0.4s，比任何真实姿态变化都短。
MIN_SAMPLES_FOR_DEGENERACY = 20

#: 判定"完全没有变化"的容差。轮式里程计填的是精确的 0.0，
#: 所以这里可以取得很小；给一点余量是为了容忍序列化往返的浮点噪声。
DEGENERACY_EPS = 1e-9


def posture_out_of_bounds(z, roll, pitch,
                          normal_height, max_height_deviation, max_tilt_rad):
    """姿态是否超出允许范围。返回 (是否超限, 原因字符串或 None)。

    刻意返回原因：原实现只打一行"检测到异常姿态(z=.. roll=.. pitch=..)"，
    三个量一起报，读日志的人得自己比对三个阈值才知道是哪一项触发的。
    """
    if abs(z - normal_height) > max_height_deviation:
        return True, ('高度偏差 |%.4f - %.4f| = %.4f 超过 %.4f'
                      % (z, normal_height, abs(z - normal_height),
                         max_height_deviation))
    if abs(roll) > max_tilt_rad:
        return True, ('横滚 |%.4f| 超过 %.4f' % (roll, max_tilt_rad))
    if abs(pitch) > max_tilt_rad:
        return True, ('俯仰 |%.4f| 超过 %.4f' % (pitch, max_tilt_rad))
    return False, None


def is_degenerate_attitude_source(samples, eps=DEGENERACY_EPS,
                                 min_samples=MIN_SAMPLES_FOR_DEGENERACY):
    """判断这个 (z, roll, pitch) 序列是否**不携带姿态信息**。

    samples: [(z, roll, pitch), ...]

    判据是"三个量各自在整个窗口内都没有任何变化"，而**不是**"三个量都等于 0"。
    理由：等于 0 是个具体取值，换一台机器人/换一个 odom 源就可能是别的常数；
    而"恒定不变"才是"这个源不携带信息"的本质。

    样本不足时返回 False —— 宁可暂时不下结论，也不要凭 2 帧就宣布数据源坏了。
    """
    if len(samples) < min_samples:
        return False
    for idx in range(3):
        col = [s[idx] for s in samples]
        if max(col) - min(col) > eps:
            return False
    return True


def describe_monitor_state(enabled, degenerate, tripped):
    """把三个布尔量翻译成一句人能读的状态，供启动日志与止损日志共用。

    存在的理由：原实现里"监控开着但收不到数据"与"监控开着且一切正常"
    在日志上**完全无法区分** —— 两种情况都是那一条启动 INFO，之后再无输出。
    """
    if not enabled:
        return ('姿态监控**已显式禁用** —— 实机 /odom 是轮式里程计(3-DOF)，'
                'z/roll/pitch 恒为 0，不携带姿态信息，这个监控在此数据源上'
                '结构性无效。要做真的倾倒检测需改用 IMU。')
    if degenerate:
        return ('姿态监控**已自动停用**：/odom 的 z/roll/pitch 在整个观察窗口内'
                '一点变化都没有，说明这个数据源不携带姿态信息。'
                '继续按阈值判定只会把"没有信息"误判成"姿态异常"并永久停车。')
    if tripped:
        return '姿态监控**已止损**：检测到异常姿态，持续下发零速度。'
    return '姿态监控**在线**：/odom 携带姿态信息，阈值判定生效。'


# ---------------------------------------------------------------- 统一决策入口

#: 继续转发，什么都不做。
ACT_PASS = 'pass'
#: 样本还不够判退化 —— **不做超限判定**，直接放行本帧。
#: 少了这个状态就会出现：第一帧就触发止损，退化检测永远等不到它需要的样本数。
ACT_COLLECTING = 'collecting'
#: 数据源不携带姿态信息 -> 停用监控（不是止损）。
ACT_DISABLE_DEGENERATE = 'disable_degenerate'
#: 真的检测到异常姿态 -> 止损。
ACT_TRIP = 'trip'
#: 监控被显式禁用。
ACT_DISABLED = 'disabled'


def evaluate_posture(enabled, samples, z, roll, pitch,
                     normal_height, max_height_deviation, max_tilt_rad,
                     min_samples=MIN_SAMPLES_FOR_DEGENERACY):
    """姿态监控的**完整决策**，含判定顺序。返回 (动作, 原因或 None)。

    `samples` 必须是**含本帧在内**的历史窗口。

    !!! 判定顺序是这个函数存在的全部理由 !!!
    实机 /odom 的 z=0 同时满足两件事：在仿真阈值下"超限"，且是一个不携带
    姿态信息的退化源。两个判定都会命中，所以顺序直接决定行为：
      · 先判超限 -> 永久停车，日志说"检测到异常姿态"（错误结论，且无复位路径）
      · 先判退化 -> 停用监控，说明数据源不携带姿态信息（正确结论）
    把顺序留在节点的 callback 里，就没有任何测试能钉住它 —— 实测过：
    节点里顺序写对了，但"把顺序调回去"这个变异在 20 条测试下**全部存活**。
    """
    if not enabled:
        return ACT_DISABLED, None
    if is_degenerate_attitude_source(samples, min_samples=min_samples):
        return ACT_DISABLE_DEGENERATE, None
    if len(samples) < min_samples:
        return ACT_COLLECTING, None
    tripped, why = posture_out_of_bounds(
        z, roll, pitch, normal_height, max_height_deviation, max_tilt_rad)
    return (ACT_TRIP, why) if tripped else (ACT_PASS, None)
