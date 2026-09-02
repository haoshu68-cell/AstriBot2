#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样本汇总与判据比对。**纯函数，无 ROS 依赖**，可离线单测。

为什么单独一层
============
本仓库有过多次"结论正确、数据是假的"，其中一类是**汇总方式本身**造成的：

* 只报均值 —— 均值误差可以标定掉，σ 标定不掉。对接/穿窄门受制于 σ。
* 样本双峰却取平均 —— 手臂跟踪那个 0.16/0.42 双峰，取平均得到 0.29，
  这个数**任何一次实际运行里都不会出现**。所以本模块强制做双峰检测，
  检测到就把结论降级成 INCONCLUSIVE，不允许 PASS/FAIL。
* 丢掉原始样本 —— 想换判据就得重跑。所以 dump 一律带 raw。
"""

import json
import math
import os
import statistics
import time

PASS = 'PASS'
FAIL = 'FAIL'
INCONCLUSIVE = 'INCONCLUSIVE'   # 样本双峰/太少，不允许给结论

# 判据方向
LESS_IS_BETTER = 'le'    # 值 <= 阈值 为通过
MORE_IS_BETTER = 'ge'    # 值 >= 阈值 为通过


def percentile(values, q):
    """线性插值分位数。q 取 0~100。空序列抛错而不是返回 0。"""
    if not values:
        raise ValueError('percentile 收到空序列')
    if not 0.0 <= q <= 100.0:
        raise ValueError('q 必须在 0~100')
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    pos = (len(s) - 1) * q / 100.0
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(s) - 1)
    frac = pos - lo
    return s[lo] * (1.0 - frac) + s[hi] * frac


def looks_bimodal(values, min_n=8, gap_ratio=2.0):
    """粗筛双峰：排序后最大相邻间隙是否显著大于其余间隙的中位数。

    不是严格的统计检验，目的只是**拦住"双峰取平均"这个具体错误**。
    返回 (是否可疑, 诊断字典)。样本少于 min_n 时不判（返回 False）。

    gap_ratio=2.0 的含义：最大间隙 > 2 倍的间隙中位数就算可疑。
    这个阈值刻意宽松——宁可多报几次 INCONCLUSIVE 去人工看一眼，
    也不要把 0.16/0.42 那种分布悄悄压成一个均值。
    """
    if len(values) < min_n:
        return False, {'reason': f'样本 {len(values)} < {min_n}，不做双峰判定'}
    s = sorted(values)
    gaps = [s[i + 1] - s[i] for i in range(len(s) - 1)]
    positive = [g for g in gaps if g > 0.0]
    if not positive:
        return False, {'reason': '所有样本相同'}
    med = statistics.median(positive)
    max_gap = max(gaps)
    idx = gaps.index(max_gap)
    if med <= 0.0:
        return False, {'reason': '间隙中位数为 0'}
    suspicious = max_gap > gap_ratio * med
    return suspicious, {
        'max_gap': max_gap,
        'median_gap': med,
        'ratio': max_gap / med,
        'split_below': s[:idx + 1],
        'split_above': s[idx + 1:],
    }


class Metric:
    """一个指标的样本集合 + 判据。"""

    def __init__(self, name, unit, threshold=None, direction=LESS_IS_BETTER,
                 threshold_basis='', min_n=1):
        self.name = name
        self.unit = unit
        self.threshold = threshold
        self.direction = direction
        self.threshold_basis = threshold_basis   # 判据来自哪里，必须写
        self.min_n = min_n
        self.samples = []        # [(value, meta_dict), ...]

    def add(self, value, **meta):
        if value is None or (isinstance(value, float) and math.isnan(value)):
            raise ValueError(f'{self.name}: 拒绝加入 None/NaN 样本，'
                             f'缺样本要显式记录为失败而不是静默丢弃')
        self.samples.append((float(value), meta))

    @property
    def values(self):
        return [v for v, _ in self.samples]

    def summary(self):
        v = self.values
        n = len(v)
        out = {
            'name': self.name,
            'unit': self.unit,
            'n': n,
            'threshold': self.threshold,
            'direction': self.direction,
            'threshold_basis': self.threshold_basis,
        }
        if n == 0:
            out['verdict'] = INCONCLUSIVE
            out['note'] = '零样本'
            return out

        out['mean'] = statistics.fmean(v)
        out['sigma'] = statistics.stdev(v) if n >= 2 else 0.0
        out['p95'] = percentile(v, 95.0)
        out['min'] = min(v)
        out['max'] = max(v)
        out['worst'] = max(v) if self.direction == LESS_IS_BETTER else min(v)

        bimodal, diag = looks_bimodal(v)
        out['bimodal'] = bimodal
        out['bimodal_diag'] = diag

        if n < self.min_n:
            out['verdict'] = INCONCLUSIVE
            out['note'] = f'样本 {n} < 要求的 {self.min_n}'
        elif bimodal:
            out['verdict'] = INCONCLUSIVE
            out['note'] = ('样本疑似双峰，均值无意义。必须先找出区分两峰的因子，'
                           '不允许用均值给结论（见测试方案 §5.2）')
        elif self.threshold is None:
            out['verdict'] = INCONCLUSIVE
            out['note'] = '本项无预设判据，只记录数值'
        else:
            # 用 worst 而不是 mean 判定：单次越界就是越界
            ok = (out['worst'] <= self.threshold if self.direction == LESS_IS_BETTER
                  else out['worst'] >= self.threshold)
            out['verdict'] = PASS if ok else FAIL
        return out


class Result:
    """一次测试运行的全部产出。"""

    def __init__(self, test_id, env, description='', context=None):
        self.test_id = test_id
        self.env = env                       # 'sim' | 'real'
        self.description = description
        self.context = dict(context or {})   # 版本、参数、话题、真值源等
        self.metrics = []
        self.notes = []
        self.started_at = time.time()

    def metric(self, *args, **kwargs):
        m = Metric(*args, **kwargs)
        self.metrics.append(m)
        return m

    def note(self, text):
        self.notes.append(text)

    @property
    def verdict(self):
        """整体结论：任一 FAIL 即 FAIL；否则任一 INCONCLUSIVE 即 INCONCLUSIVE。"""
        vs = [m.summary()['verdict'] for m in self.metrics]
        if not vs:
            return INCONCLUSIVE
        if FAIL in vs:
            return FAIL
        if INCONCLUSIVE in vs:
            return INCONCLUSIVE
        return PASS

    def to_dict(self):
        return {
            'test_id': self.test_id,
            'env': self.env,
            'description': self.description,
            'context': self.context,
            'started_at': self.started_at,
            'finished_at': time.time(),
            'verdict': self.verdict,
            'notes': self.notes,
            'metrics': [
                dict(m.summary(), raw=[{'value': v, **meta} for v, meta in m.samples])
                for m in self.metrics
            ],
        }

    def to_markdown(self):
        d = self.to_dict()
        lines = [
            f"# {d['test_id']} · {d['verdict']}",
            '',
            f"环境 `{d['env']}` · {d['description']}",
            '',
        ]
        if d['context']:
            lines += ['## 运行上下文', '']
            for k, v in d['context'].items():
                lines.append(f'* `{k}` = `{v}`')
            lines.append('')
        lines += [
            '## 指标',
            '',
            '| 指标 | 单位 | N | 均值 | σ | p95 | 最差 | 判据 | 结论 |',
            '|---|---|---|---|---|---|---|---|---|',
        ]
        for m in d['metrics']:
            if m['n'] == 0:
                lines.append(f"| {m['name']} | {m['unit']} | 0 | — | — | — | — | — | {m['verdict']} |")
                continue
            thr = '—' if m['threshold'] is None else \
                f"{'≤' if m['direction'] == LESS_IS_BETTER else '≥'} {m['threshold']:g}"
            lines.append(
                f"| {m['name']} | {m['unit']} | {m['n']} | {m['mean']:.4f} | "
                f"{m['sigma']:.4f} | {m['p95']:.4f} | {m['worst']:.4f} | {thr} | {m['verdict']} |")
        lines.append('')
        for m in d['metrics']:
            if m.get('threshold_basis'):
                lines.append(f"* **{m['name']}** 判据依据：{m['threshold_basis']}")
            if m.get('note'):
                lines.append(f"* **{m['name']}** 说明：{m['note']}")
            if m.get('bimodal'):
                diag = m['bimodal_diag']
                lines.append(
                    f"* **{m['name']}** ⚠️ 双峰：低组 {diag['split_below']} / "
                    f"高组 {diag['split_above']}（最大间隙是中位间隙的 "
                    f"{diag['ratio']:.1f} 倍）")
        if d['notes']:
            lines += ['', '## 备注', '']
            lines += [f'* {n}' for n in d['notes']]
        lines.append('')
        return '\n'.join(lines)

    def save(self, results_dir):
        os.makedirs(results_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(self.started_at))
        base = os.path.join(results_dir, f'{self.test_id}_{stamp}')
        with open(base + '.json', 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=2)
        with open(base + '.md', 'w', encoding='utf-8') as f:
            f.write(self.to_markdown())
        return base + '.json', base + '.md'
