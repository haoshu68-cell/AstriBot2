#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""厂商 SDK 会话适配器：把 ``Astribot`` 实例包装成 SessionPort。

!!! 本模块**刻意不在顶层 import 厂商 SDK** !!!
=============================================
``astribot_sdk/core/astribot_api/astribot_interface.py:34-41`` 在**模块顶层**执行::

    quiet = os.getenv("ASTRIBOT_LOG", "").lower() not in ("1", "true", "on")
    if quiet:
        _fd1, _fd2 = os.dup(1), os.dup(2)
        null_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(null_fd, 1); os.dup2(null_fd, 2)

这是 ``os.dup2`` 的**进程级 fd 重定向**，连 C 扩展的输出一起吞。后果：

* ``ASTRIBOT_LOG=1`` 必须在 import 之前进入 ``os.environ``，import 之后设置**无效**；
* 一旦被吞，本节点自己的 ERROR 日志也发不出来 —— 包括本该"响亮失败"的那些。

所以顶层 import 会让"环境变量检查"永远来不及。真实 import 被推迟到
``open_session()`` 内部，并且在 import **之前**先跑一遍环境自检。
"""

import os
import sys

from astribot_trajectory_bridge.ports import SessionPort
from astribot_trajectory_bridge.write_gate import (
    ASTRIBOT_LOG_ENV,
    ROBOT_TYPE_ENV,
    check_env_before_sdk_import,
)


class SdkSessionError(RuntimeError):
    """会话建立失败。**不做静默降级** —— 会话建不起来时上层必须知道。"""


# import 厂商 SDK 必需的 pip 包。**不在这里自动安装** —— 装包是环境动作，
# 不该由节点在运行期偷偷做；只负责把缺哪个说清楚。
_REQUIRED_PIP_PKGS = ('filterpy', 'tabulate', 'h5py')


def missing_pip_packages(pkgs=_REQUIRED_PIP_PKGS):
    """返回缺失的 pip 包名列表。用 find_spec，不真正 import（import 有副作用）。"""
    import importlib.util
    missing = []
    for name in pkgs:
        try:
            if importlib.util.find_spec(name) is None:
                missing.append(name)
        except Exception:      # noqa: BLE001 —— find_spec 对畸形包会抛，一律算缺失
            missing.append(name)
    return missing


def prewarm_robotics_library_py():
    """在 import 厂商 SDK **之前**把 ``robotics_library_py`` 包钉进 ``sys.modules``。

    !!! 这不是可选的优化，是绕过厂商打包缺陷的必要步骤 !!!

    厂商把扩展模块 ``robotics_library_py.so`` 放在了**同名的包目录**
    ``astribot_sdk/core/common/robotics_library_py/`` 里（该目录自己有
    ``__init__.py``）。谁先上 ``sys.path``，``.so`` 就把包遮掉；而编译过的
    ``util.py:26`` 要的偏偏是包形式 ``import robotics_library_py.robotics_library_py``，
    于是报::

        ModuleNotFoundError: No module named 'robotics_library_py.robotics_library_py';
                             'robotics_library_py' is not a package

    实测：先在**干净** ``sys.modules`` 上单独 import 一次，解析到的是包
    （``__init__.py``、有 ``__path__``、子模块也进了 ``sys.modules``），
    之后厂商链路就能正常走完。所以这里主动预热一次。

    失败**不抛异常**：真正的报错留给后面那次真实 import 去报，那里的上下文
    （``_import_failure_hints``）更完整。这里抛会把"缺 pip 包"误报成"预热失败"。
    """
    try:
        import robotics_library_py      # noqa: F401
        return True
    except Exception:      # noqa: BLE001
        return False


def _import_failure_hints():
    """import 失败时给出**当前实测**的可能原因，按排查成本从低到高排。

    !!! 不要再往这里写 "pinocchio ABI 冲突" !!!
    那是本项目一度写错并扩散到多处的结论，已被 ldd 否证：厂商 .so 的 DT_NEEDED
    烧死的是 ``libpinocchio_default.so.3.7.0``，仓库 third_party 自带该版本且
    env.sh 已挂进 LD_LIBRARY_PATH；系统 apt 的 4.0.0 soname 不同，**永不参与解析**。
    两版按 soname 共存，不是冲突。
    """
    hints = []
    missing = missing_pip_packages()
    if missing:
        hints.append(
            '缺 pip 包 %s。注意必须用 --no-deps 安装，否则会顶掉本项目钉住的 '
            'numpy 1.21.5：python3 -m pip install --no-deps %s'
            % (', '.join(missing), ' '.join(missing)))
    common = os.path.join(_sdk_root_guess(), 'astribot_sdk', 'core', 'common')
    if common not in os.environ.get('PYTHONPATH', '').split(os.pathsep):
        hints.append(
            'PYTHONPATH 里没有 %s —— 编译过的 util.py 用裸名 '
            '`import robotics_library_py.robotics_library_py`，需要这一层在路径上。'
            % common)
    hints.append('确认已 source 过 SDK 根目录的 env.sh（它负责 LD_LIBRARY_PATH '
                 '与 third_pkg/software 的 setup.bash）。')
    return '可能原因：\n  - ' + '\n  - '.join(hints)


def _sdk_root_guess():
    """从 ASTRIBOT_SDK_ROOT 取 SDK 根（env.sh:67 会设）。取不到返回空串。"""
    return os.environ.get('ASTRIBOT_SDK_ROOT', '')


def ensure_env_for_sdk(logger=None, robot_type='S1'):
    """在 import SDK 之前把环境准备好，并自检。返回 (ok, problems)。

    这里主动 setdefault 而不是只检查：节点可能被 ``ros2 run`` 直接拉起（没有经过
    我们的 launch），那时环境变量不会被设上。主动补一次比让日志静默消失好。
    但**已经 import 过就补不回来了**，所以自检仍然会报出来。
    """
    os.environ.setdefault(ASTRIBOT_LOG_ENV, '1')
    os.environ.setdefault(ROBOT_TYPE_ENV, robot_type)
    ok, problems = check_env_before_sdk_import(
        os.environ, loaded_modules=set(sys.modules.keys()))
    if not ok and logger is not None:
        for p in problems:
            logger.error('[SDK 环境自检] %s' % p)
    return (ok, problems)


class AstribotSession(SessionPort):
    """真实 SDK 会话。方法签名与 examples 里实际调用的形式逐字对齐。

    对齐依据（全部来自 examples 源码，不来自文档）：
      * ``Astribot(freq=..., high_control_rights=False)`` —— 202:31 / 203:33，
        签名 astribot_client.py:38（``high_control_rights`` 默认就是 False）
      * ``get_desired_joints_position(names)`` —— 101:46 / 202:44 / 203:43
      * ``get_current_joints_position(names)`` —— 101:45 / 106:45
      * ``set_joints_position(names, position, control_way, use_wbc,
        add_default_torso)`` —— 105:53
      * ``move_joints_waypoints(names, waypoints, time_list, ...)`` —— 206:45
      * ``get_joints_position_limit(names)`` -> **(lower, upper)** ——
        astribot_client.py:141
    """

    def __init__(self, astribot):
        self._bot = astribot

    # -- 部件名（来自 astribot_base.py:5-15，不硬编码字符串） --

    @property
    def chassis_name(self):
        return self._bot.chassis_name

    @property
    def whole_body_names(self):
        return list(self._bot.whole_body_names)

    def part_name(self, attr):
        """按属性名取部件名，例如 part_name('arm_left_name')。"""
        return getattr(self._bot, attr)

    # -- SessionPort --

    def get_desired_joints_position(self, names):
        return self._bot.get_desired_joints_position(names)

    def get_current_joints_position(self, names):
        return self._bot.get_current_joints_position(names)

    def set_joints_position(self, names, position, control_way='filter',
                            use_wbc=False, add_default_torso=True):
        return self._bot.set_joints_position(
            names, position, control_way=control_way, use_wbc=use_wbc,
            add_default_torso=add_default_torso)

    def move_joints_waypoints(self, names, waypoints, time_list,
                              use_wbc=False, add_default_torso=True):
        return self._bot.move_joints_waypoints(
            names, waypoints, time_list, use_wbc=use_wbc,
            add_default_torso=add_default_torso)

    def get_joints_position_limit(self, names=None):
        return self._bot.get_joints_position_limit(names)

    def get_robot_mode(self):
        """机器人模式。

        !!! ``Astribot`` 上**没有** get_robot_mode !!!
        实测（Gate 0 探针，2026-08-26）：``dir(bot)`` 里与模式相关的只有
        ``get_control_rights_status``；``get_robot_mode`` 挂在**内层**的
        ``astribot_interface`` 上 —— astribot_client.py:52 是
        ``self.astribot_interface.get_robot_mode()``。
        原先这里写 ``self._bot.get_robot_mode()``，真机/仿真上会直接
        ``AttributeError``，而它的调用点是**写入闸门**与状态上报，
        也就是最不能出错的路径。这个缺陷 267 条离线测试全绿也发现不了 ——
        FakeSession 实现了这个方法，只有真会话才暴露。

        返回值取值（astribot_client.py:54-62 逐字）：
        ``'safe'`` / ``'professional'`` / ``'extremity'``，
        以及**任何其它值都表示仿真** —— 源码是 if/elif/elif/else，
        else 分支置 ``__in_simulation = True``。所以不能把"未知值"当异常。
        """
        return self._bot.astribot_interface.get_robot_mode()

    def get_control_rights_status(self):
        """是否已取得控制权（astribot_client.py:67-79，返回 bool）。"""
        return self._bot.get_control_rights_status()

    def get_dof(self, names=None):
        return self._bot.get_dof(names)

    # -- 夹爪 --
    #
    # !!! 极性与直觉相反：0 = 张开、100 = 闭合 !!!
    #   open_effector  -> 下发 0.0   （astribot_client.py:813 / 817）
    #   close_effector -> 下发 100.0 （astribot_client.py:836 / 840）
    # 换算 rad = 0.0093 * cmd（cmd=100 正好是关节上限 0.93）。
    # 一律走 gripper_math，不要在调用点手写系数或极性。
    #
    # 这两个方法**阻塞**：实测 duration=1.0 时调用阻塞满 1.0s 才返回
    # 'move to joint position success'（内部走 move_to_joint_position）。
    # 所以**不能在 250Hz 内环里调**，只能放在 service/动作回调这类允许阻塞的地方。

    def open_effector(self, names=None, duration=1.0):
        return self._bot.open_effector(names, duration=duration)

    def close_effector(self, names=None, duration=1.0):
        return self._bot.close_effector(names, duration=duration)

    def set_effector_max_force(self, names, max_force):
        """设夹持力上限。

        !!! 仿真下这是空操作 !!! astribot_client.py:1139 第一行就是
        ``if self.__in_simulation: return``（实测返回 None）。
        所以**仿真里任何夹持力行为都不能作为验收证据** —— 力限只能在真机上验。
        真机上力值范围随机器人模式变化：safe 40N / standard 60N / extreme 80N，
        下限 10N（109-effector_open_close.py:39 的注释）。
        """
        return self._bot.set_effector_max_force(names, max_force)

    def effector_names(self):
        """夹爪部件名（astribot_base.py 的 effector_names 属性）。"""
        return list(self._bot.effector_names)

    # -- 急停（999-stop_robot.py:30,34） --

    def stop_robot(self):
        return self._bot.stop_robot()

    def restart_robot(self):
        return self._bot.restart_robot()


def open_session(freq=250.0, node_name='astribot_bridge', logger=None,
                 robot_type='S1'):
    """建立 SDK 会话。返回 AstribotSession。失败抛 SdkSessionError。

    !!! 永不申请高控制权 !!!
    ``high_control_rights`` 固定传 False（SDK 默认也是 False，
    见 astribot_client.py:38）。这是只读/受限写方向的**物理边界**，
    不是可配置项 —— 所以不提供参数让调用方改。
    """
    ok, problems = ensure_env_for_sdk(logger, robot_type)
    if not ok:
        raise SdkSessionError(
            'SDK 环境自检未通过，拒绝建立会话：\n  - ' + '\n  - '.join(problems))

    prewarm_robotics_library_py()

    # 这一行必须在环境自检**之后**（见模块头部说明）
    try:
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
    except Exception as exc:      # noqa: BLE001
        raise SdkSessionError(
            'import 厂商 SDK 失败：%s\n'
            '%s' % (exc, _import_failure_hints()))

    try:
        bot = Astribot(freq=freq, high_control_rights=False, node_name=node_name)
    except Exception as exc:      # noqa: BLE001
        raise SdkSessionError('构造 Astribot 失败：%s' % exc)

    # astribot_client.py:49 —— 构造里已经 wait_for_interface_alive，
    # 但它的返回值存在 is_alive，这里再确认一次并给出明确失败。
    if not getattr(bot, 'is_alive', True):
        raise SdkSessionError('SDK 接口未就绪（wait_for_interface_alive 失败）')

    session = AstribotSession(bot)
    if logger is not None:
        try:
            logger.info('SDK 会话已建立：freq=%.1fHz 模式=%s 部件=%s'
                        % (freq, session.get_robot_mode(),
                           session.whole_body_names))
        except Exception:      # noqa: BLE001 —— 打日志失败不该影响会话
            logger.info('SDK 会话已建立：freq=%.1fHz' % freq)
    return session
