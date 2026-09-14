# Copyright 2026 Astribot.


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
