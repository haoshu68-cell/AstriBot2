#!/usr/bin/env python3
# Copyright 2026 Astribot.
#
# 跨限速档位汇总：把若干 rounds.csv / run.json 变成一张可比的表。
#
# ==================== 这个脚本最重要的职责是**拒绝出表** ====================
# 限速扫描最容易产出的不是错数，而是**看起来正常的相同数**：
#   · 外层 launch 漏转发 max_linear_speed -> 每档 vx_max 都是 1.0
#   · 臂-底盘耦合把底盘压到 15% -> 两个档位产生同一个实际速度
# 这两种情况下每一行都完全正常，只有把「实测限速」和「耦合衰减」并列出来
# 才能看出整张表是混淆的。所以：
#   ① 实测 vx_max 与请求档位不符的档位，标 UNUSABLE 并从对比中剔除
#   ② 两个档位的实测 vx_max 相同时，直接报"这两档不是两个档位"
#   ③ 耦合衰减在档位间差异显著时，报"比的可能不是限速"
#
# 另外：n 与轮数差很多的均值不能直接比 —— 每一项都带 n。
import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys

# 汇总表默认展示的列。顺序按"先答问题、再给旁证"排。
DEFAULT_COLUMNS = [
    'arrival_error_xy_m',
    'arrival_error_yaw_rad',
    'settle_drift_xy_m',
    'overshoot_radial_m',
    'overshoot_along_track_m',
    'path_len_ratio_vs_plan',
    'planning_time_median_s',
    'replan_count',
    'planning_failure_rate',
    'plan_turn_per_m',
    'track_turn_per_m',
    'cross_track_max_m',
    'cross_track_mean_m',
    'motion_dir_error_mean_rad',
    'vel_track_rms_at_best_lag',
    'vel_track_best_lag_s',
    'vel_track_gain',
    'local_plan_hz',
    'osc_count_angular',
    'gate_rotate_only_s',
    'min_clearance_m',
    'nearest_obstacle_m',
    'center_bias_abs_mean',
    'corridor_sample_ratio',
    'narrow_success_rate_proxy',
    'zero_progress_events_proxy',
    'downstream_dead_ratio',
    'geometric_intrusion_episodes_proxy',
    'scan_hz',
    'valid_ratio_mean',
    'self_filter_residual_ratio_proxy',
    'brake_latency_s_proxy',
    'coupling_atten_p50',
]

TRUTHY = ('true', 'True', '1')


def _num(v):
    """CSV 里的一格 -> float 或 None。空串/'None'/nan 一律 None。

    **不把空串当 0**：那会在均值里塞进一堆假的 0。
    """
    if v is None:
        return None
    s = str(v).strip()
    if s == '' or s.lower() in ('none', 'nan', 'n/a'):
        return None
    try:
        f = float(s)
    except ValueError:
        return None
    return f if math.isfinite(f) else None


def load_run(directory):
    """读一个档位目录。返回 dict 或 None（读不到就说读不到）。"""
    rounds = sorted(glob.glob(os.path.join(directory, '*rounds.csv')))
    runs = sorted(glob.glob(os.path.join(directory, '*run.json')))
    if not rounds:
        return None
    rows = []
    with open(rounds[0], encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            rows.append(r)
    meta = {}
    if runs:
        try:
            with open(runs[0], encoding='utf-8') as fh:
                meta = json.load(fh)
        except (OSError, ValueError):
            meta = {}
    return {'dir': directory, 'rows': rows, 'meta': meta,
            'rounds_csv': rounds[0]}


def measured_cap(run):
    """该档位的**实测** vx_max。取每轮的值并要求一致。

    逐轮取而不是只读 run.json：run.json 是最后一次写入的快照，
    如果限速在跑的中途被改过，快照会掩盖这件事。
    """
    vals = {_num(r.get('vx_max_measured')) for r in run['rows']}
    vals.discard(None)
    if not vals:
        return None, '实测限速缺失（查参没成功）'
    if len(vals) > 1:
        return None, '同一档位内实测限速有多个值 %s —— 跑的中途被改过' % sorted(vals)
    return vals.pop(), None


def requested_cap(run):
    vals = {_num(r.get('speed_cap_requested')) for r in run['rows']}
    vals.discard(None)
    if len(vals) == 1:
        return vals.pop()
    return _num((run['meta'] or {}).get('speed_cap_requested'))


def column_stats(rows, key):
    """一列的 (mean, sd, n)。None 不参与，n 一起返回。"""
    vals = [v for v in (_num(r.get(key)) for r in rows) if v is not None]
    if not vals:
        return None, None, 0
    mean = statistics.fmean(vals)
    sd = statistics.stdev(vals) if len(vals) > 1 else None
    return mean, sd, len(vals)


def success_rate(rows):
    if not rows:
        return None, 0
    ok = sum(1 for r in rows if str(r.get('success')) in TRUTHY)
    return float(ok) / float(len(rows)), len(rows)


def audit(runs):
    """出表前的三道拒绝检查。见文件头。"""
    problems = []
    for run in runs:
        cap, why = measured_cap(run)
        req = requested_cap(run)
        run['cap_measured'] = cap
        run['cap_requested'] = req
        run['usable'] = True
        if cap is None:
            run['usable'] = False
            problems.append('%s: %s -> UNUSABLE' % (run['label'], why))
            continue
        if req is not None and abs(cap - req) > 1e-6:
            run['usable'] = False
            problems.append(
                '%s: 请求档位 %.3f 但实测 vx_max=%.3f -> UNUSABLE。'
                '最常见原因是外层 launch 没有把 max_linear_speed 转发下去'
                % (run['label'], req, cap))

    # ② 两个档位的实测限速相同 = 它们不是两个档位
    seen = {}
    for run in runs:
        cap = run.get('cap_measured')
        if cap is None:
            continue
        seen.setdefault(round(cap, 6), []).append(run['label'])
    for cap, labels in sorted(seen.items()):
        if len(labels) > 1:
            problems.append(
                '实测 vx_max=%.3f 出现在多个档位 %s —— 这些不是不同的档位，'
                '把它们并列对比会得出"限速对精度没有影响"的假结论'
                % (cap, labels))

    # ③ 耦合衰减在档位间差异显著 -> 比的可能不是限速
    atten = {}
    for run in runs:
        m, _sd, n = column_stats(run['rows'], 'coupling_atten_p50')
        run['atten_p50'] = m
        if m is not None and n > 0:
            atten[run['label']] = m
    if len(atten) >= 2:
        lo, hi = min(atten.values()), max(atten.values())
        if lo > 0 and (hi - lo) > 0.15:
            problems.append(
                '耦合衰减在档位间从 %.2f 变到 %.2f（差 %.2f）—— 臂-底盘耦合'
                '实测能把底盘压到 15%%，此时不同"限速档位"可能产生同一个实际'
                '速度。比较之前先把双臂构型固定下来' % (lo, hi, hi - lo))
    return problems


def build_table(runs, columns):
    header = ['档位(实测vx_max)', '请求档位', '轮数', '到位成功率']
    for c in columns:
        header.append(c)
    lines = [header]
    for run in sorted(runs, key=lambda r: (r.get('cap_measured') is None,
                                           r.get('cap_measured') or 0.0)):
        cap = run.get('cap_measured')
        sr, n = success_rate(run['rows'])
        tag = '' if run.get('usable') else '  [UNUSABLE]'
        row = ['%s%s' % ('n/a' if cap is None else '%.3f' % cap, tag),
               'n/a' if run.get('cap_requested') is None
               else '%.3f' % run['cap_requested'],
               str(n),
               'n/a' if sr is None else '%.1f%%' % (100.0 * sr)]
        for c in columns:
            mean, sd, cn = column_stats(run['rows'], c)
            if mean is None:
                row.append('n/a (n=0)')
            elif sd is None:
                row.append('%.4g (n=%d)' % (mean, cn))
            else:
                row.append('%.4g ±%.3g (n=%d)' % (mean, sd, cn))
        lines.append(row)
    return lines


def render_markdown(lines, problems, runs):
    out = ['# MPPI 限速 × 探索到位精度', '']
    if problems:
        out.append('## ⚠️ 出表前的告警（先看这一段）')
        out.append('')
        for p in problems:
            out.append('- %s' % p)
        out.append('')
        out.append('标记 `[UNUSABLE]` 的档位数据不可用，不要拿它做对比。')
        out.append('')
    else:
        out.append('三道自检（实测限速与请求一致 / 各档位限速互不相同 / '
                   '耦合衰减在档位间稳定）全部通过。')
        out.append('')

    out.append('## 汇总（均值 ±标准差, n=参与统计的轮数）')
    out.append('')
    out.append('> `n` 与「轮数」差很多的那一格不能直接比 —— 那一项在该档位'
               '大部分轮次里没有有效样本。')
    out.append('')
    # 转置：指标做行、档位做列，横向可比
    caps = [lines[0][0]] + [r[0] for r in lines[1:]]
    out.append('| 指标 | ' + ' | '.join(r[0] for r in lines[1:]) + ' |')
    out.append('|---|' + '---|' * (len(lines) - 1))
    for ci in range(1, len(lines[0])):
        name = lines[0][ci]
        out.append('| %s | %s |' % (
            name, ' | '.join(r[ci] for r in lines[1:])))
    del caps

    out.append('')
    out.append('## 口径与边界')
    out.append('')
    metas = [r['meta'] for r in runs if r.get('meta')]
    caveats = []
    for m in metas:
        for c in (m.get('caveats') or []):
            if c not in caveats:
                caveats.append(c)
    for c in caveats:
        out.append('- %s' % c)
    defs = {}
    for m in metas:
        defs.update(m.get('proxy_definitions') or {})
    if defs:
        out.append('')
        out.append('### 代理量定义（`*_proxy`）')
        out.append('')
        for k in sorted(defs):
            out.append('- **%s** — %s' % (k, defs[k]))
    return '\n'.join(out) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='跨 MPPI 限速档位汇总探索到位精度')
    ap.add_argument('dirs', nargs='+',
                    help='各档位的录制输出目录（含 rounds.csv / run.json）')
    ap.add_argument('--out-prefix', default='sweep',
                    help='输出 <prefix>.csv 与 <prefix>.md')
    ap.add_argument('--columns', default='',
                    help='逗号分隔的列名，留空用默认集合；'
                         'all 表示 rounds.csv 里所有数值列')
    args = ap.parse_args(argv)

    runs = []
    for d in args.dirs:
        run = load_run(d)
        if run is None:
            print('跳过 %s：里面没有 rounds.csv' % d, file=sys.stderr)
            continue
        run['label'] = os.path.basename(os.path.normpath(d))
        runs.append(run)
    if not runs:
        print('一个档位都没读到 —— 不出表。'
              '（这比出一张空表好：空表会被当成"测过了、没差异"）',
              file=sys.stderr)
        return 2

    if args.columns == 'all':
        cols = []
        for run in runs:
            for r in run['rows']:
                for k in r:
                    if k not in cols and _num(r.get(k)) is not None:
                        cols.append(k)
    elif args.columns:
        cols = [c.strip() for c in args.columns.split(',') if c.strip()]
    else:
        cols = list(DEFAULT_COLUMNS)

    problems = audit(runs)
    table = build_table(runs, cols)

    csv_path = args.out_prefix + '.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as fh:
        w = csv.writer(fh)
        for line in table:
            w.writerow(line)
        if problems:
            w.writerow([])
            w.writerow(['告警'])
            for p in problems:
                w.writerow([p])

    md_path = args.out_prefix + '.md'
    with open(md_path, 'w', encoding='utf-8') as fh:
        fh.write(render_markdown(table, problems, runs))

    for p in problems:
        print('告警: %s' % p, file=sys.stderr)
    print('已写出 %s 与 %s（%d 个档位，其中 %d 个可用）'
          % (csv_path, md_path, len(runs),
             sum(1 for r in runs if r.get('usable'))))
    # 有 UNUSABLE 档位时退出码非 0：脚本串起来时不能静默当成成功
    return 1 if any(not r.get('usable') for r in runs) else 0


if __name__ == '__main__':
    sys.exit(main())
