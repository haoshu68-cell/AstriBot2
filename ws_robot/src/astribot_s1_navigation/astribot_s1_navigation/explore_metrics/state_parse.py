# Copyright 2026 Astribot.
#
# /exploration/state 单行 key=value 串的解析。**纯函数，无 ROS 依赖**。
#
# 放在这里而不是留在录制节点里，是为了它能在**不 source ROS** 的情况下被单测。
# 节点文件 import rclpy/tf2_ros，测试它就得整套 ROS 环境；而模块级
# pytest.importorskip 会把整个 collection 吃掉 —— pytest 6.2.5 下
# `pytest test/` 只报 "1 skipped"，几百条测试静默消失。
# 干净的 "1 skipped" 比报错更危险，所以判据一律留在无依赖的模块里。
#
# 协调器发的串形如：
#   state=NAVIGATING goal_in_flight=1 goal=(1.23,4.56) candidate=2/7
#   dispatched=5 succeeded=3 rejected=11 nav_fail=0/5 sample_fail=0/8
#   validate_fail=1/6 auto_resume=0/3 bootstrap=1/3 bootstrap_result=ok
#   path_pts=36 replan_policy=on_invalid


def parse_state_line(line):
    """单行 key=value 串 -> dict。

    按空格切、再按**第一个** '=' 分。不用正则贪婪匹配：值里可能带括号
    （goal=(1.23,4.56)），贪婪匹配会把后面的字段吞进来。

    解析不了的 token 直接丢掉而不是抛异常 —— 协调器哪天加了个新字段，
    不该让整次录制挂掉。
    """
    out = {}
    for tok in str(line).split():
        if '=' not in tok:
            continue
        k, _, v = tok.partition('=')
        if k:
            out[k] = v
    return out


def _leading_int(raw):
    """'3/5' -> 3，'7' -> 7，其它 -> None。

    协调器的失败类计数器是 "当前/上限" 形式，只有分子是计数。
    取不到时返回 None 而**不是 0**：0 会被当成"确实是 0 次"，
    而两者在对账时的含义完全不同。
    """
    if raw is None:
        return None
    head = str(raw).split('/')[0].strip()
    try:
        return int(head)
    except (TypeError, ValueError):
        return None


def coordinator_counters(fields):
    """取协调器**自己**的计数器，用于与录制器的统计双路对账。

    为什么要对账：成功率这类数只有一路来源时，那一路出错就无从发现。
    协调器的 dispatched= 与录制器看到的目标数不一致，差值就是漏录/多录的
    轮数 —— 不查清楚，成功率不可信。
    """
    return {
        'coord_dispatched': _leading_int(fields.get('dispatched')),
        'coord_succeeded': _leading_int(fields.get('succeeded')),
        'coord_rejected': _leading_int(fields.get('rejected')),
        'coord_nav_fail': _leading_int(fields.get('nav_fail')),
        'coord_validate_fail': _leading_int(fields.get('validate_fail')),
        'coord_replan_policy': fields.get('replan_policy'),
        'coord_path_pts': _leading_int(fields.get('path_pts')),
    }
