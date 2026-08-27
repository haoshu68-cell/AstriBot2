#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪命令空间与弧度的换算。**纯函数，不依赖 rclpy、不依赖厂商 SDK。**

为什么这件事值得单独一个模块并测到
==============================
夹爪的命令空间 **0 = 张开、100 = 闭合**，与"0-100 像开合百分比、100 应该是全开"
的直觉**相反**。而且 ``[0, 100]`` 不是弧度 —— 直接当弧度用会差两个数量级。

本项目已经在夹爪上错过一次：SRDF 夹爪组只含主动关节，从动关节全不动，
结果"张口静默算成一半、抓取角错一倍且零报错"。极性/倍率这类错误的共同特征是
**两边都不报错**，只能靠把它钉进纯函数 + 断言来防。

事实来源（全部逐字，不来自文档描述）
================================
* ``open_effector()``  下发 **0.0**   —— astribot_client.py:813 / 817
* ``close_effector()`` 下发 **100.0** —— astribot_client.py:836 / 840
* SDK 限位 ``get_joints_position_limit`` 返回 **[0, 100]** —— Gate 0-c 实测
* MuJoCo 驱动关节 ``range="0 0.93"``，执行器 ``gainprm=4.65``、
  ``biasprm="0 -500 -10"``、``biastype=affine``，稳态 ``4.65*ctrl = 500*len``
  => ``rad = 0.0093 * cmd``，``cmd=100 -> 0.93 rad`` 正好等于关节上限

实测标定（MuJoCo 后端，0→100→0 双向扫描，每点持续重发到稳）
=======================================================
* 命令空间线性度 ``k=0.999837 b=0.0148``，最大残差 **0.0005**
* 回差（滞环）**0.0000**
* 实测换算系数 **0.009298**（理论 0.009300，吻合 0.02%）
* 越界命令 −20 / 150 被**夹到边界**，不乱走

!!! 仿真里夹持力不可验收 !!!
``set_effector_max_force`` 在仿真下是空操作（astribot_client.py:1139
``if self.__in_simulation: return``，实测返回 None）。力限只能在真机上验。
"""

# 命令空间边界。来自 SDK 限位实测，不是猜的。
CMD_OPEN = 0.0
CMD_CLOSED = 100.0

# 命令 -> 弧度的换算系数。理论 4.65/500 = 0.0093，实测 0.009298。
# 这里用**理论值**：它与关节上限 0.93 严格自洽（100*0.0093 = 0.93），
# 而实测值带着 0.0148 的读数偏置，用它会让 cmd=100 算出 0.9298 而非 0.93。
RAD_PER_CMD = 0.0093

# 驱动关节的弧度上限（MuJoCo range="0 0.93"）。
RAD_CLOSED = CMD_CLOSED * RAD_PER_CMD      # 0.93


class GripperConfigError(ValueError):
    """夹爪换算配置非法。"""


def clamp_cmd(cmd):
    """把命令夹到 [0, 100]。

    SDK 侧实测本来就会夹（−20 -> 0.014，150 -> 99.998），这里**先夹一次**是为了
    让越界在我们这一层就有明确记录，而不是依赖下游的隐式行为 ——
    依赖下游意味着换个后端（真机）行为可能不同。
    """
    return max(CMD_OPEN, min(CMD_CLOSED, float(cmd)))


def is_cmd_in_range(cmd):
    """命令是否在 [0, 100] 内（不夹，只判断）。"""
    return CMD_OPEN <= float(cmd) <= CMD_CLOSED


def cmd_to_rad(cmd, clamp=True):
    """命令空间 -> 驱动关节弧度。

    ``cmd=0`` -> ``0.0`` rad（张开），``cmd=100`` -> ``0.93`` rad（闭合）。
    注意弧度随**闭合**增大 —— 它是闭合角，不是张开角。
    """
    c = clamp_cmd(cmd) if clamp else float(cmd)
    return c * RAD_PER_CMD


def rad_to_cmd(rad, clamp=True):
    """驱动关节弧度 -> 命令空间。是 :func:`cmd_to_rad` 的严格逆。"""
    c = float(rad) / RAD_PER_CMD
    return clamp_cmd(c) if clamp else c


def opening_fraction_to_cmd(fraction):
    """"张开程度" -> 命令。

    ``fraction`` 取 ``[0, 1]``，**1 = 全张开**、0 = 全闭合 —— 这是给上层用的
    符合直觉的量。命令空间是反的，所以这里做一次翻转::

        cmd = (1 - fraction) * 100

    提供这个函数的目的就是**让上层不必自己记住极性是反的**。
    """
    f = max(0.0, min(1.0, float(fraction)))
    return (1.0 - f) * CMD_CLOSED


def cmd_to_opening_fraction(cmd):
    """命令 -> "张开程度"（1 = 全张开）。是 :func:`opening_fraction_to_cmd` 的逆。"""
    return 1.0 - clamp_cmd(cmd) / CMD_CLOSED


def validate_grasp_cmd(cmd, name='gripper_cmd'):
    """校验一个要下发的夹爪命令。越界**显式报错**，不静默夹。

    夹与报错的分工：
      * ``clamp_cmd`` 用在"已经决定要发、只是保证不越界"的地方；
      * 本函数用在**参数入口**（yaml / service 请求），那里越界说明调用方
        的意图本身有问题，静默夹会让"我要求 150"变成"实际 100"而无人知晓。
    """
    try:
        v = float(cmd)
    except (TypeError, ValueError):
        raise GripperConfigError('%s=%r 不是数值' % (name, cmd))
    if v != v:      # NaN
        raise GripperConfigError('%s 是 NaN' % name)
    if not is_cmd_in_range(v):
        raise GripperConfigError(
            '%s=%r 越出命令范围 [%g, %g]。'
            '提醒：命令空间是 0=张开、100=闭合（与直觉相反），'
            '若想表达"张开程度"请用 opening_fraction_to_cmd()。'
            % (name, v, CMD_OPEN, CMD_CLOSED))
    return v


def describe_cmd(cmd):
    """把命令翻译成人能读的说明。用于状态上报与日志。

    极性反转最容易在**看日志**时被误读（看到 100 会以为是全开），
    所以日志里一律带上文字说明。
    """
    c = clamp_cmd(cmd)
    if c <= 1.0:
        state = '张开'
    elif c >= 99.0:
        state = '闭合'
    else:
        state = '半开(%.0f%%张开)' % (cmd_to_opening_fraction(c) * 100.0)
    return '%s（cmd=%.1f，%.4f rad）' % (state, c, cmd_to_rad(c))
