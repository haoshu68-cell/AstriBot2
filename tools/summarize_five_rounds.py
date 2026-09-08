#!/usr/bin/env python3
"""把 run_five_round_exploration.sh 的多轮产物汇成一张对照表。

回答的是"这两条缺陷的复现率"，所以每一列都必须是可独立复核的实测量：
  · 目标数 / 到位数        <- rounds.csv 的 outcome
  · 导航失败              <- 协调器自己的状态迁移行（与 outcome 对账）
  · FOLLOW->DONE          <- 控制器日志，kDone 锁存的直接证据
  · 最长零速段            <- cmd_raw 样本，判"整段停车"而不是"抖动"
  · raw非零->final零      <- 判零速是源头发的还是下游归零的（必须接近 0）
"""
import csv
import glob
import gzip
import os
import re
import sys
from collections import Counter

ROOT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/explore_5rounds'
ANSI = re.compile(r'\x1b\[[0-9;]*m')


def nonzero(v):
    return abs(v[1]) > 0.01 or abs(v[2]) > 0.01 or abs(v[3]) > 0.01


def one_round(rd):
    out = {'轮': os.path.basename(rd).replace('round_', '')}

    rc = glob.glob(os.path.join(rd, 'metrics', '*rounds.csv'))
    if rc:
        rows = list(csv.DictReader(open(rc[0])))
        oc = Counter(r['outcome'] for r in rows)
        out['目标'] = len(rows)
        out['到位'] = oc.get('ARRIVED', 0)
        out['outcome'] = dict(oc)
        errs = [float(r['arrival_error_xy_m']) for r in rows
                if r['outcome'] != 'ARRIVED' and r['arrival_error_xy_m']]
        out['未到位误差'] = ('%.2f~%.2f' % (min(errs), max(errs))) if errs else '-'
    else:
        out['目标'] = out['到位'] = 0
        out['outcome'] = {}
        out['未到位误差'] = '-'

    logs = sorted(glob.glob(os.path.join(rd, 'stack_*.log')))
    if logs:
        txt = ANSI.sub('', open(logs[-1], errors='replace').read())
        out['导航失败'] = txt.count('导航失败，重新选点')
        out['疑似被困'] = txt.count('连续导航失败，疑似被困')
        out['F->DONE'] = txt.count('相位 FOLLOW -> DONE')
        out['无进展'] = txt.count('Failed to make progress')
        # 不要只数某一个键名。这条计数是为了发现"录制器在查一个不存在的
        # 参数"，而键名本身正是会变的那一项（2026-09-07 FollowPath 改成三段式
        # 控制器后，MPPI 的限速键下移到 <实例>.inner）。只数旧键名的后果是
        # 缺陷换个键名就永远计到 0 —— 判据看起来通过，实际是判据自己瞎了。
        out['取参失败'] = txt.count('Failed to get parameters:')
    else:
        for k in ('导航失败', '疑似被困', 'F->DONE', '无进展', '取参失败'):
            out[k] = 0

    worst = 0.0
    killed = pair = 0
    for fn in sorted(glob.glob(os.path.join(rd, 'metrics', 'samples', '*.csv.gz'))):
        with gzip.open(fn, 'rt') as f:
            rows = list(csv.DictReader(f))
        raw = [(float(r['t']), float(r['a']), float(r['b']), float(r['c']))
               for r in rows if r['channel'] == 'cmd_raw']
        fin = [(float(r['t']), float(r['a']), float(r['b']), float(r['c']))
               for r in rows if r['channel'] == 'cmd_final']
        if len(raw) < 2:
            continue
        dts = [raw[i + 1][0] - raw[i][0] for i in range(min(200, len(raw) - 1))]
        dt = sorted(dts)[len(dts) // 2] if dts else 0.05
        run = best = 0
        for x in raw:
            if not nonzero(x):
                run += 1
                best = max(best, run)
            else:
                run = 0
        worst = max(worst, best * dt)
        # 逐帧最近邻配对（0.15s 内），判下游是否把非零改成零
        rt = [x[0] for x in raw]
        import bisect
        for f_ in fin:
            i = bisect.bisect_left(rt, f_[0])
            cand = [raw[j] for j in (i - 1, i)
                    if 0 <= j < len(raw) and abs(raw[j][0] - f_[0]) < 0.15]
            if not cand:
                continue
            r_ = min(cand, key=lambda x: abs(x[0] - f_[0]))
            pair += 1
            if nonzero(r_) and not nonzero(f_):
                killed += 1
    out['最长零速s'] = '%.1f' % worst
    out['下游归零%'] = '%.1f' % (100.0 * killed / pair) if pair else '-'
    return out


rounds = sorted(glob.glob(os.path.join(ROOT, 'round_*')))
if not rounds:
    sys.exit('没有找到 round_* 目录：%s' % ROOT)

cols = ['轮', '目标', '到位', '导航失败', '疑似被困', 'F->DONE', '无进展',
        '最长零速s', '下游归零%', '未到位误差', '取参失败']
print('  '.join('%-9s' % c for c in cols))
tot = Counter()
allout = Counter()
for rd in rounds:
    r = one_round(rd)
    print('  '.join('%-9s' % str(r.get(c, '-')) for c in cols))
    for k in ('目标', '到位', '导航失败', '疑似被困', 'F->DONE', '无进展'):
        tot[k] += r[k]
    allout.update(r['outcome'])

print()
print('合计: 目标 %d, 到位 %d (%.0f%%), 导航失败 %d, 疑似被困 %d, F->DONE %d, 无进展 %d'
      % (tot['目标'], tot['到位'],
         100.0 * tot['到位'] / tot['目标'] if tot['目标'] else 0,
         tot['导航失败'], tot['疑似被困'], tot['F->DONE'], tot['无进展']))
print('outcome 分布:', dict(allout))
