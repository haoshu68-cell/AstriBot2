#!/usr/bin/env python3
"""把 explore_metrics_recorder_node 的产物汇成一份可读报告。

设计纪律（每一条都来自本项目踩过的坑）：

1. **口径只有一份。** 汇总一律走 round_metrics.summarize()，绝不在这里另算一遍。
   两处各算一份必然漂开，而漂开之后没有任何报错。
2. **代理量必须带着定义一起出现。** recorder 落盘的 run.json 里带
   PROXY_DEFINITIONS，报告原样搬过来 —— 否则半年后没人能复核
   "narrow_success_rate_proxy=0.83" 到底是什么意思。
3. **区分"0"与"没测到"。** 0 次事件和一次都没采到样，在结论上是相反的。
   前者是好消息，后者是这一列不可用。一律显式打印 n=样本数。
4. **自检不通过就把结论标废**，不是照样出数：TF 一次都没取到 -> 到位精度整列
   不可用；counter_mismatch -> 轮数对不上，全表可信度存疑。
5. **不做单轮结论。** 打印逐轮明细 + 样本量提醒，不给"平均值"当结论。
"""
import argparse
import csv
import glob
import json
import os
import sys

_REPO = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

# 找 astribot_s1_navigation 包。**src 优先**，理由与"src 同步了不等于 install
# 重建了"相反：本工作区是 --symlink-install，install 下只有一个
# astribot-s1-navigation.egg-link 指向 build/，site-packages 里**没有包目录**，
# 所以原来那一条 sys.path 永远命中不了 —— 症状就是 ②节恒空报
# "No module named 'astribot_s1_navigation'"，而脚本其余部分照样出数。
#
# 报告是离线复算，口径应当等于**当前源码**的口径；但用了哪一份必须打印出来，
# 否则读报告的人无从判断这张表是谁的判据算的。
_PKG_CANDIDATES = (
    os.path.join(_REPO, 'ws_robot', 'src', 'astribot_s1_navigation'),
    os.path.join(_REPO, 'ws_robot', 'build', 'astribot_s1_navigation'),
    os.path.join(_REPO, 'ws_robot', 'install', 'astribot_s1_navigation',
                 'lib', 'python3.10', 'site-packages'),
)
for _cand in _PKG_CANDIDATES:
    if os.path.isdir(os.path.join(_cand, 'astribot_s1_navigation')):
        sys.path.insert(0, _cand)



def _pick(out_dir, name):
    """找 name，找不到就找 *_name（recorder 的 run_label 会给文件名加前缀）。

    这里必须区分"文件不存在"和"表里没有已完成的轮次" —— 两者都会让 rows 为空，
    而报告对后者的说法是"这不等于跑得不好"。实测踩过：run_label=seg 时文件叫
    seg_rounds.csv，本脚本硬拼 rounds.csv 读不到，于是把一次 10 轮全成功的
    跑机报成"还没有任何已完成的轮次"。返回 (路径, 是否存在) 让调用方能分辨。
    """
    direct = os.path.join(out_dir, name)
    if os.path.exists(direct):
        return direct
    hits = sorted(glob.glob(os.path.join(out_dir, '*_' + name)))
    if len(hits) > 1:
        # 多个 run_label 混在一个目录里：挑一个就是悄悄丢数据
        raise SystemExit(
            '目录 %s 下有 %d 份 %s：%s\n'
            '这是多个 run_label 的产物混在一起了。请指定单个 run 的目录，'
            '或先分开 —— 自动挑一份会静默丢掉其余几份的数据。'
            % (out_dir, len(hits), name, ', '.join(os.path.basename(h) for h in hits)))
    return hits[0] if hits else None


def load(out_dir):
    rounds_path = _pick(out_dir, 'rounds.csv')
    run_path = _pick(out_dir, 'run.json')
    rows = []
    if rounds_path:
        with open(rounds_path, encoding='utf-8') as fh:
            rows = list(csv.DictReader(fh))
    run = {}
    if run_path:
        with open(run_path, encoding='utf-8') as fh:
            run = json.load(fh)
    return rows, run, rounds_path


def num(v):
    try:
        f = float(v)
        return None if f != f else f          # NaN -> None
    except (TypeError, ValueError):
        return None


def stat(rows, key):
    """返回 (n, 最小, 中位, 最大)。n 是**真正有值**的轮数，不是总轮数。"""
    vals = sorted(x for x in (num(r.get(key)) for r in rows) if x is not None)
    if not vals:
        return 0, None, None, None
    n = len(vals)
    med = vals[n // 2] if n % 2 else (vals[n // 2 - 1] + vals[n // 2]) / 2.0
    return n, vals[0], med, vals[-1]


def fmt(x, nd=4):
    return 'n/a' if x is None else f'{x:.{nd}f}'


def line(ch='-', n=78):
    print(ch * n)


# ②和④共用同一张列表：两处各写一份，早晚有一处漏掉新列而没人发现。
# (列名, 中文标签, 同行的样本数列)
# 第三项是**区分"0 次"与"没测到"**的依据（纪律 3）。没有伴随计数列的写 None。
KEY_METRICS = (
    ('arrival_error_xy_m', '到位位置误差(m)', 'pose_samples'),
    ('arrival_error_yaw_rad', '到位航向误差(rad)', 'pose_samples'),
    ('settle_drift_xy_m', '静置漂移(m)', 'settle_samples'),
    ('cross_track_p95_m', '横向偏差 p95(m)', 'cross_track_samples'),
    ('cross_track_max_m', '横向偏差 max(m)', 'cross_track_samples'),
    ('duration_s', '单轮时长(s)', None),
    # 路径长度的列名是 traveled_m，**不叫 path_length_m**。
    # 报告此前问的是 path_length_m —— 那一列从来不存在，于是恒报
    # "n=0 没测到"，而 traveled_m 一直有数。列名写错与真的没数据同样是 n=0。
    ('traveled_m', '实走里程(m)', 'pose_samples'),
    ('first_plan_len_m', '首条规划路径长(m)', 'plans_published'),
    ('path_len_ratio_vs_plan', '里程/规划路径', 'pose_samples'),
    ('narrow_success_rate_proxy', '窄段通过率(代理)', 'narrow_episodes'),
    # 下面四列全部依赖足迹外接半径。published_footprint 是全局系绝对坐标，
    # 半径必须先平移回对称中心再算 —— 不做这步实测会拿到 6~9m 的外接半径，
    # 于是 min_clearance = 最近距离 - 9 恒为大负数，整段"净空"结论作废
    ('min_clearance_m', '最小净空(m,按外接)', 'scan_frames'),
    ('nearest_obstacle_m', '最近障碍(m)', 'scan_frames'),
    ('center_bias_abs_max', '居中偏差max(m)', 'scan_frames'),
    ('narrow_sample_ratio', '窄段帧占比', 'scan_frames'),
    ('zero_progress_events_proxy', '零进展事件(代理)', 'pose_samples'),
    ('geometric_intrusion_episodes_proxy', '几何侵入段数(代理,按内切)', 'scan_frames'),
    ('self_filter_residual_ratio_proxy', '自滤残留占比(代理)', 'self_filter_scored_frames'),
    ('brake_latency_s_proxy', '刹车延迟(s,代理)', 'brake_events'),
    ('planning_time_median_s', '规划耗时中位(s)', 'plan_requests'),
    ('pose_coverage_ratio', '位姿覆盖率', 'pose_samples'),
)


def companion_total(rows, key):
    """伴随计数列的合计。用来区分"测了但是 0 次"和"根本没采到样"。"""
    if key is None:
        return None
    tot = 0
    for r in rows:
        v = num(r.get(key))
        if v is not None:
            tot += int(v)
    return tot


def write_md(path, buf):
    """把已经 emit 出来的内容写成 markdown。

    必须在**每一条**返回路径上都调用，包括"没有已完成轮次"和"找不到
    rounds.csv"这两条早退。原来只写在 main() 末尾，于是 0 轮时 --md
    被静默忽略：调用方（实机会话脚本）照样打印"报告：…/report.md"，
    而那个文件根本不存在 —— 一次典型的假成功。
    """
    if not path:
        return
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('# 自主探索 路径评价指标报告\n\n```\n')
        fh.write('\n'.join(buf))
        fh.write('\n```\n')
    print(f'\n[markdown 已写入 {path}]')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out_dir', nargs='?', default='/tmp/explore_metrics_run')
    ap.add_argument('--md', help='同时写一份 markdown 到该路径')
    args = ap.parse_args()

    rows, run, rounds_path = load(args.out_dir)
    buf = []

    def emit(s=''):
        print(s)
        buf.append(s)

    line('=')
    emit(f'自主探索 路径评价指标报告   来源目录: {args.out_dir}')
    line('=')

    if not rows:
        # 必须先分清"表根本不存在"和"表在但没有已完成的轮次" ——
        # 两者 rows 都是空，而结论完全不同
        if rounds_path is None:
            emit('🔴 **在 %s 下找不到 rounds.csv（也没有 *_rounds.csv）**。'
                 % args.out_dir)
            emit('这不是"跑得不好"，是**根本没读到数据**：目录写错，'
                 '或 recorder 从未落盘过一轮。不要把这一条当成性能结论。')
            write_md(args.md, buf)
            return
        emit('rounds.csv 里还没有任何**已完成**的轮次。（读到的是 %s）'
             % os.path.basename(rounds_path))
        emit('这不等于"跑得不好" —— recorder 是每轮结束才落盘一行。')
        if run:
            mc = run.get('message_counts', {})
            emit('\n各话题已收到的消息数（0 的那几路是链路问题，不是性能问题）:')
            for k in sorted(mc):
                flag = '  🔴 一帧都没收到' if not mc[k] else ''
                emit(f'    {k:<28} {mc[k]}{flag}')
        write_md(args.md, buf)
        return

    # ---------- ① 先看自检，不通过就把相关列标废 ----------
    emit('\n【① 数据可用性自检】不通过的列，后面的数一律不作为结论')
    line()
    tf_ok = sum(int(num(r.get('tf_lookup_ok')) or 0) for r in rows)
    tf_fail = sum(int(num(r.get('tf_lookup_fail')) or 0) for r in rows)
    mism = [r.get('round_index') for r in rows
            if str(r.get('counter_mismatch')).lower() == 'true']
    # None/空 表示**没能对账**，与"一致"不是一回事，必须分开报
    nocheck = [r.get('round_index') for r in rows
               if str(r.get('counter_mismatch')).strip().lower()
               in ('', 'none')]
    # 这份 CSV 是不是老版录制器写的？增量列整列缺失即是。
    # 不做这个判断，老表会在下面打印 "协调器派发 +None ... 差 >1"，
    # 那是把"这一列还不存在"说成了"对账失败"，两件事的处置完全不同。
    stale_counter = 'coord_dispatched_delta' not in (rows[0] or {})
    cap_ok = [str(r.get('speed_cap_matches_request')).lower() for r in rows]
    emit(f'  TF 取到/失败            {tf_ok} / {tf_fail}'
         + ('   🔴 一次都没取到 ⇒ 到位精度、静置漂移整列不可用'
            if tf_ok == 0 else ''))
    if stale_counter and mism:
        emit(f'  轮数对账                {len(mism)}/{len(rows)} 轮标记不一致，'
             '但这份 CSV 是**老版录制器**写的')
        emit('      ← 老判据拿协调器"开机累计"比本会话计数，**结构上恒不一致**、'
             '无信息量。')
        emit('        新判据只比增量（列 coord_dispatched_delta / '
             'recorder_goals_delta）。这份表里没有这两列，故本项**不作为结论**。')
    else:
        emit(f'  轮数对账不一致的轮次    {mism if mism else "无"}'
             + ('   🔴 两侧派发增量差 >1，全表可信度存疑' if mism else ''))
        for r in rows:
            if str(r.get('counter_mismatch')).lower() == 'true':
                emit(f'      轮 {r.get("round_index")}: 协调器派发 '
                     f'+{r.get("coord_dispatched_delta")}，'
                     f'本录制器 +{r.get("recorder_goals_delta")}')
    if nocheck:
        emit(f'  🔴 无法对账的轮次        {nocheck}'
             '   ← 读不到协调器 dispatched，这几轮的成功率没有第二路证据')
    emit(f'  限速与请求一致          {cap_ok.count("true")}/{len(cap_ok)} 轮')
    susp_n = sum(1 for r in rows
                 if str(r.get('pose_coverage_suspicious')).lower() == 'true')
    if susp_n:
        emit(f'  🔴 位姿覆盖不足的轮次    {susp_n}/{len(rows)} 轮'
             '   ← 这几轮的到位误差/里程/横偏都不是测量结果，详见③')
    elif 'pose_coverage_ratio' not in (rows[0] or {}):
        emit('  位姿覆盖率              这份 CSV 是老版录制器写的，没有这一列'
             ' ← **未经检查**，不等于合格')

    # ---------- ② 汇总走 summarize()，不另算 ----------
    emit('\n【② 汇总】口径来自 round_metrics.summarize()，本脚本不另算一份')
    line()
    try:
        from astribot_s1_navigation.explore_metrics import round_metrics
        # 必须先还原类型：CSV 出来全是字符串，'False' 作为非空字符串会被判为真，
        # success_rate 会**恒为 1.0** 且不报错。summarize() 现在也会显式拦这一道。
        typed = [round_metrics.coerce_csv_row(r) for r in rows]
        s = round_metrics.summarize(typed)
        # 打真实路径：symlink-install 下 build/ 里那份是指回 src 的软链，
        # 直接打 __file__ 会让读报告的人以为用的是构建产物而不是当前源码。
        _src = os.path.realpath(round_metrics.__file__)
        emit('  （判据来自 %s）' % os.path.relpath(_src, _REPO))
        emit(f'  {"轮数":<26} {s["rounds"]}')
        emit(f'  {"到位成功轮数":<24} {s.get("succeeded")}')
        emit(f'  {"到位成功率":<25} {fmt(s.get("success_rate"))}')
        ncols = len([k for k in s if k.endswith("__n")])
        emit(f'  （summarize 共产出 {ncols} 个数值列的 n/mean/sd/max；'
             f'下面只列关键列，全量在 run.json 与 rounds.csv）')
        emit('')
        emit(f'  {"指标":<26}{"n":>4}{"mean":>11}{"sd":>11}{"max":>11}')
        for key, label, _comp in KEY_METRICS:
            n = s.get('%s__n' % key)
            if n is None:
                continue                     # 该列不是数值列，或本表没有
            emit(f'  {label:<24}{n:>4}{fmt(s.get("%s__mean" % key), 3):>13}'
                 f'{fmt(s.get("%s__sd" % key), 3):>11}'
                 f'{fmt(s.get("%s__max" % key), 3):>11}')
    except Exception as exc:                                  # noqa: BLE001
        emit(f'  🔴 无法调用 summarize()：{exc}')
        emit('  ⇒ 不在这里手算替代 —— 两处各算一份必然漂开且不会报错。')

    # ---------- ③ 逐轮明细 ----------
    emit('\n【③ 逐轮明细】不给平均值当结论；样本量少时逐轮看离散度')
    line()
    cols = [('round_index', '轮'), ('outcome', '结果'),
            ('arrival_error_xy_m', '到位误差m'), ('arrival_error_yaw_rad', '航向误差'),
            ('settle_drift_xy_m', '静置漂移m'), ('traveled_m', '实走里程m'),
            ('duration_s', '时长s'), ('cross_track_p95_m', '横偏p95'),
            ('pose_samples', '位姿样本'), ('pose_coverage_ratio', '位姿覆盖')]
    have = [(k, t) for k, t in cols if any(r.get(k) not in (None, '') for r in rows)]
    emit('  ' + ' '.join(f'{t:>10}' for _, t in have))
    for r in rows:
        cells = []
        for k, _ in have:
            v = num(r.get(k))
            cells.append(f'{v:>10.4f}' if v is not None else f'{str(r.get(k) or "n/a"):>10}')
        flag = ''
        if str(r.get('pose_coverage_suspicious')).lower() == 'true':
            flag = '  🔴 位姿覆盖不足'
        emit('  ' + ' '.join(cells) + flag)

    susp = [r.get('round_index') for r in rows
            if str(r.get('pose_coverage_suspicious')).lower() == 'true']
    if susp:
        emit('')
        emit('  🔴 轮 %s 的位姿覆盖率 < 0.5：这几轮的**到位误差、实走里程、横向偏差'
             '都不是测量结果**。' % susp)
        emit('     单个位姿样本算不出"到位"，它只是一张快照；里程会因此偏小到 0.000。')
        emit('     不要拿这几行当跟踪质量的证据 —— 无论数字看起来多好。')

    # ---------- ④ 关键指标的 n/最小/中位/最大 ----------
    emit('\n【④ 关键指标分布】n = **真正有值**的轮数；'
         'n=0 时看"伴随样本"才知道是 0 次事件还是没测到')
    line()
    for key, label, comp in KEY_METRICS:
        n, lo, med, hi = stat(rows, key)
        mark = ''
        if not n:
            tot = companion_total(rows, comp)
            if key not in (rows[0] or {}):
                # 列名压根不在表头里。这与"采不到样"是两件事：
                # 前者是这份 CSV 由老版录制器写的（或列名写错），后者是数据问题。
                # 报错时混为一谈，就会去查传感器而真因在列名 —— 本项目已中过一次
                # （报告问 path_length_m / cross_track_p95_m，两列从来不存在）。
                mark = '   ← 这份 CSV 里**没有这一列**（老版录制器/列名不符），非"没测到"'
            elif comp is None:
                mark = '   ← 没测到，这一列不可用'
            elif tot == 0:
                # 纪律 3：0 次事件和没采到样在结论上相反。有伴随计数且为 0，
                # 说明采集通路是好的、事件确实没发生 —— 这是好消息，不是缺数据。
                mark = f'   ← {comp}=0 ⇒ **测了，0 次事件**（好消息，不是缺数据）'
            elif tot is None:
                mark = '   ← 没测到，这一列不可用'
            else:
                mark = f'   🔴 {comp}={tot} 有样本却算不出这一列 ⇒ 判据有问题'
        emit(f'  {label:<26} n={n:<3} min={fmt(lo)} med={fmt(med)} max={fmt(hi)}{mark}')

    # ---------- ⑤ 代理量定义原样搬出 ----------
    proxy = run.get('proxy_definitions') or run.get('PROXY_DEFINITIONS') or {}
    if not proxy:
        try:
            from astribot_s1_navigation.explore_metrics import round_metrics
            proxy = round_metrics.PROXY_DEFINITIONS
        except Exception:                                     # noqa: BLE001
            proxy = {}
    if proxy:
        emit('\n【⑤ 代理量定义】带定义才可复核。名字里有 proxy 的都**不是**真值')
        line()
        for k in sorted(proxy):
            emit(f'  · {k}')
            for seg in str(proxy[k]).split('。'):
                if seg.strip():
                    emit(f'      {seg.strip()}。')

    # ---------- ⑥ 环境/足迹/话题 ----------
    if run:
        emit('\n【⑥ 本次运行的环境事实】')
        line()
        fp = run.get('footprint', {})
        emit(f'  足迹顶点数 {len(fp.get("vertices") or [])}  '
             f'内切 {fmt(fp.get("inscribed_m"))}  外接 {fmt(fp.get("circumscribed_m"))}')
        emit(f'  请求限速 {run.get("speed_cap_requested")}  '
             f'实测限速 {run.get("speed_cap_measured")}  '
             f'一致={run.get("speed_cap_matches_request")}')
        mc = run.get('message_counts', {})
        zero = [k for k in mc if not mc[k]]
        emit(f'  话题消息数：{len(mc)} 路'
             + (f'，其中 🔴 一帧未收到: {zero}' if zero else '，全部有数据'))

    emit('\n' + '=' * 78)
    emit(f'轮次 {len(rows)} 个。样本量 <5 时不要下"策略有效/无效"的结论 ——')
    emit('本项目已两次在 n=4~5 时宣布结论，随后被自己的数据否证。')
    line('=')

    if args.md:
        write_md(args.md, buf)


if __name__ == '__main__':
    main()
