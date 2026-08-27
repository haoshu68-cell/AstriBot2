#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""写通路准入（WriteGate）与环境变量守卫。纯逻辑，不依赖 rclpy / SDK。

为什么 D-1 统一 domain 必须配一个 WriteGate
==========================================
本工程曾用 domain 隔离(D-2)保证"仿真 /cmd_vel 绝不能有到真机的路径"。
开发阶段选了 D-1（统一 domain）之后，**那条保护就没有了**：同一个 domain 里，
如果 MuJoCo 和真机后端同时在图上，一条 /cmd_vel 会同时到两边，而 SDK 的
"连当前图上有谁"机制此时**行为未定义**。

WriteGate 是 D-1 下唯一阻止"仿真指令打到真机"的机制。它是**流程性准入检查，
不是网络层隔离** —— 强度低于 D-2，如实记录：一旦有人手动改配置或图上意外多了
个后端，防线只有这一层。建议 G4（底盘写通路）通过后、进入真机联调前切回 D-2。

三条硬规矩
=========
1. **读通路不受 WriteGate 限制**，只读永远可用（方便 G1 先跑通）。
2. ``allow_write_to_real`` 默认 False 且**不写进默认 yaml**，必须命令行显式给，
   防止被存进配置后忘记。
3. 拒绝时节点**存活并持续上报状态**，不退出 —— 上层要能看到"为什么写不动"，
   而不是节点消失。
"""

import os

TARGET_SIM = 'sim'
TARGET_REAL = 'real'
VALID_TARGETS = (TARGET_SIM, TARGET_REAL)

ROBOT_MODE_SAFE = 'safe'

# 真机的三种模式。**实测依据** astribot_client.py:54-62 的 if/elif/elif/else：
# 只有这三个值会被识别为真机，**落进 else 的一律是仿真**（该分支置
# __in_simulation = True）。Gate 0 探针实测仿真返回的字符串就是 'simulation'。
#
# 为什么必须区分"仿真"与"真机非安全模式"：
# 闸门若只认 'safe'，仿真下会被判成"非安全模式"而拒绝写 —— 逼人去开
# allow_unsafe_mode，而那个开关同时也把**真机的 professional/extremity 放开**。
# 一个仿真期的便利开关会变成真机上的安全缺口，所以两件事必须分开判。
ROBOT_MODES_REAL = ('safe', 'professional', 'extremity')


def is_simulation_mode(robot_mode):
    """按 SDK 自己的判据回答"这是仿真吗"。

    不是查表匹配 'simulation' 这个字面量 —— 厂商的判据是"**不在**三个真机模式里"，
    照抄它的判据比照抄它当前的返回值更稳（将来仿真换个字符串也不会误判成真机，
    而误判成真机是**安全方向**的错，误判成仿真才是危险方向）。
    """
    return robot_mode not in ROBOT_MODES_REAL

# SDK 的 fd 级静音开关。见 astribot_sdk/core/astribot_api/astribot_interface.py:34-41
ASTRIBOT_LOG_ENV = 'ASTRIBOT_LOG'
ASTRIBOT_LOG_TRUTHY = ('1', 'true', 'on')

# 决定 chassis_dof 的环境变量。见 astribot_sdk/core/common/astribot_base.py:34-38
ROBOT_TYPE_ENV = 'ROBOT_TYPE'
VALID_ROBOT_TYPES = ('S0', 'S1')


class GateDecision:
    """准入判定结果。allowed=False 时 status_code 指出该上报哪个状态位。"""

    def __init__(self, allowed, reason='', status_code=None):
        self.allowed = allowed
        self.reason = reason
        self.status_code = status_code


# 与 astribot_bridge_msgs/BridgeStatus.msg 的枚举保持一致。
# 这里用字符串常量而不是 import 消息类型，是为了让本模块可以在**没有编译 msgs**
# 的环境下被单测（纯逻辑测试不该依赖 rosidl 产物）。调用方负责映射成数字。
ST_SDK_NOT_ALIVE = 'SDK_NOT_ALIVE'
ST_MULTIPLE_BACKENDS = 'MULTIPLE_BACKENDS'
ST_TARGET_MISMATCH = 'TARGET_MISMATCH'
ST_REAL_WRITE_NOT_AUTHORIZED = 'REAL_WRITE_NOT_AUTHORIZED'
ST_ROBOT_MODE_UNEXPECTED = 'ROBOT_MODE_UNEXPECTED'
ST_POSE_SOURCE_INVALID = 'POSE_SOURCE_INVALID'


def evaluate_write_gate(discovered_backends, declared_target,
                        allow_write_to_real, robot_mode,
                        allow_unsafe_mode=False):
    """写通路准入判定。

    Args:
        discovered_backends: 图上发现的后端标识列表，元素取 'sim' / 'real'。
        declared_target: launch 声明的 target。
        allow_write_to_real: 是否已显式授权写真机。
        robot_mode: SDK ``get_robot_mode()`` 的返回（'safe'/'professional'/'extremity'）。
        allow_unsafe_mode: 是否允许在非 safe 模式下写。

    Returns:
        GateDecision
    """
    if declared_target not in VALID_TARGETS:
        return GateDecision(
            False,
            'declared_target=%r 非法，只能是 %s' % (declared_target, list(VALID_TARGETS)),
            ST_TARGET_MISMATCH)

    # ① 后端唯一性
    backends = list(discovered_backends or [])
    if len(backends) == 0:
        return GateDecision(False, '图上没有发现任何后端', ST_SDK_NOT_ALIVE)
    if len(backends) > 1:
        return GateDecision(
            False,
            '图上同时存在后端 %s。D-1 统一 domain 下这会让指令同时打到两个后端，'
            '拒绝开启写通路。' % (backends,),
            ST_MULTIPLE_BACKENDS)

    actual = backends[0]
    if actual not in VALID_TARGETS:
        return GateDecision(
            False, '发现的后端标识 %r 无法识别' % (actual,), ST_TARGET_MISMATCH)

    # ② 声明与实际一致
    if actual != declared_target:
        return GateDecision(
            False,
            'launch 声明 target=%s，实际后端是 %s，拒绝启动写通路。'
            % (declared_target, actual),
            ST_TARGET_MISMATCH)

    # ③ 打真机需要二次显式授权
    if actual == TARGET_REAL and not allow_write_to_real:
        return GateDecision(
            False,
            '目标是真机但未显式授权，需命令行给 allow_write_to_real:=true。'
            '（该参数刻意不写进默认 yaml，避免被存进配置后忘记）',
            ST_REAL_WRITE_NOT_AUTHORIZED)

    # ④ 机器人模式
    #
    # 仿真后端不参与本条判定：仿真的 get_robot_mode() 必然不是 'safe'
    # （厂商就是用"不在三个真机模式里"来识别仿真的），若在这里拒绝，
    # 使用者只能去打开 allow_unsafe_mode —— 而那个开关会连带把**真机的
    # professional/extremity 一起放开**。仿真期的便利绝不能是真机的缺口。
    #
    # 注意这里已经过了②，actual == declared_target，所以用 actual 判断即可。
    if actual == TARGET_REAL and robot_mode != ROBOT_MODE_SAFE:
        if not allow_unsafe_mode:
            return GateDecision(
                False,
                '真机模式为 %r，仅 safe 模式允许写。'
                '（真机模式取值见 astribot_client.py:54-62）' % (robot_mode,),
                ST_ROBOT_MODE_UNEXPECTED)

    # 反向一致性：声明 sim 但 SDK 报的是真机模式，说明连上的其实是真机。
    # 这条比②更强 —— ②比的是"发现的后端"，这条比的是**SDK 自己报的模式**。
    if actual == TARGET_SIM and not is_simulation_mode(robot_mode):
        return GateDecision(
            False,
            '声明 target=sim，但 SDK 报的模式是 %r（属于真机三模式之一），'
            '实际连上的是真机。拒绝写。' % (robot_mode,),
            ST_TARGET_MISMATCH)

    return GateDecision(True, 'OK')


def validate_pose_source_target_combo(pose_source, declared_target):
    """位姿源与目标的组合校验。

    ``ground_truth`` 位姿源在真机上**不存在** —— 它依赖仿真侧的静态真值 TF。
    这个组合必须拒绝，否则闭环会拿一个不存在的基准去校正真机底盘。
    """
    if pose_source == 'ground_truth' and declared_target == TARGET_REAL:
        return GateDecision(
            False,
            'pose_source=ground_truth 在真机上不存在（它依赖仿真侧的静态真值 TF），'
            '真机必须用 pose_source:=slam。',
            ST_POSE_SOURCE_INVALID)
    return GateDecision(True, 'OK')


# ---------------------------------------------------------------------------
# 环境变量守卫
# ---------------------------------------------------------------------------

def is_astribot_log_enabled(env=None):
    """按 SDK 自己的判据检查 ASTRIBOT_LOG 是否已开启。

    判据必须与 SDK 源码一致（``astribot_interface.py:34``）::

        quiet = os.getenv("ASTRIBOT_LOG", "").lower() not in ("1", "true", "on")

    注意它是**白名单**：``ASTRIBOT_LOG=yes`` / ``=2`` / ``=TRUE `` 带空格都会
    被判为 quiet。所以不能简单地"非空即认为开了"。
    """
    env = os.environ if env is None else env
    return env.get(ASTRIBOT_LOG_ENV, '').lower() in ASTRIBOT_LOG_TRUTHY


def check_env_before_sdk_import(env=None, sdk_module_names=None,
                                loaded_modules=None):
    """在 import 厂商 SDK **之前**做的环境自检。返回 (ok, 问题列表)。

    !!! 为什么顺序是硬要求 !!!
    ``astribot_interface.py:34-41`` 的静音逻辑是**模块顶层代码**，import 时立即
    执行，且用的是 ``os.dup2`` 做**进程级 fd 重定向**（连 C 扩展的输出一起吞）。
    所以：

    * ``ASTRIBOT_LOG=1`` 必须在 import 之前进入 os.environ，**import 之后设置
      完全无效**；
    * 因为是 fd 级，桥接自己的 ERROR 日志也会被吞 —— 包括本该"响亮失败"的那些。

    本函数额外检查"SDK 是否已经被 import 了"，因为一旦已 import，即使现在补上
    环境变量也来不及了，必须在启动期就报出来而不是等到故障时发现没有日志。
    """
    env = os.environ if env is None else env
    problems = []

    if not is_astribot_log_enabled(env):
        problems.append(
            '%s 未开启（当前值 %r）。SDK import 时会用 os.dup2 把 fd 1/2 重定向到 '
            '/dev/null，导致本节点所有日志（含 ERROR）被静默吞掉。'
            '必须在 import SDK 前设置 %s=1（合法值：%s）。'
            % (ASTRIBOT_LOG_ENV, env.get(ASTRIBOT_LOG_ENV, ''),
               ASTRIBOT_LOG_ENV, list(ASTRIBOT_LOG_TRUTHY)))

    robot_type = env.get(ROBOT_TYPE_ENV, '')
    if robot_type not in VALID_ROBOT_TYPES:
        problems.append(
            '%s=%r 非法。astribot_client.py:40 会对非 S0/S1 直接 raise ValueError；'
            '且 astribot_base.py:34-38 用它决定 chassis_dof —— 未设成 S1 时底盘是 '
            '2 自由度而不是 3。必须显式设置 %s=S1。'
            % (ROBOT_TYPE_ENV, robot_type, ROBOT_TYPE_ENV))

    # SDK 是否已被 import（此时补环境变量已经来不及）
    if loaded_modules is not None:
        names = sdk_module_names or ('astribot_sdk', 'astribot_ros_middleware')
        already = [n for n in names if n in loaded_modules]
        if already:
            problems.append(
                'SDK 模块 %s 已经被 import，此时再设置 %s 已无效'
                '（fd 重定向在 import 时就已发生）。'
                % (already, ASTRIBOT_LOG_ENV))

    return (len(problems) == 0, problems)
