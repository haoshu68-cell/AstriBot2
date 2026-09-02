#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench/stats.py 的离线单测。重点是那三条"汇总方式本身会骗人"的规则。"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bench import stats as st    # noqa: E402


def test_percentile_endpoints_and_interpolation():
    v = [1.0, 2.0, 3.0, 4.0]
    assert math.isclose(st.percentile(v, 0.0), 1.0)
    assert math.isclose(st.percentile(v, 100.0), 4.0)
    assert math.isclose(st.percentile(v, 50.0), 2.5)
    assert math.isclose(st.percentile([7.0], 95.0), 7.0)


def test_percentile_rejects_empty_and_bad_q():
    with pytest.raises(ValueError):
        st.percentile([], 50.0)
    with pytest.raises(ValueError):
        st.percentile([1.0], 101.0)


def test_metric_rejects_none_and_nan():
    """缺样本必须显式失败，不能静默丢弃——丢掉的样本正是最该看的那些。"""
    m = st.Metric('x', 'm')
    with pytest.raises(ValueError):
        m.add(None)
    with pytest.raises(ValueError):
        m.add(float('nan'))


def test_verdict_uses_worst_not_mean():
    """单次越界就是越界。均值通过但有一次越界，必须 FAIL。"""
    m = st.Metric('err', 'm', threshold=0.02, direction=st.LESS_IS_BETTER, min_n=3)
    for v in (0.001, 0.002, 0.050):     # 均值 0.0177 < 0.02，但最差 0.05 越界
        m.add(v)
    s = m.summary()
    assert s['mean'] < 0.02
    assert s['verdict'] == st.FAIL


def test_more_is_better_direction():
    m = st.Metric('rate', 'Hz', threshold=95.0, direction=st.MORE_IS_BETTER, min_n=1)
    m.add(99.0)
    assert m.summary()['verdict'] == st.PASS
    m.add(80.0)
    assert m.summary()['verdict'] == st.FAIL


def test_too_few_samples_is_inconclusive_not_pass():
    m = st.Metric('err', 'm', threshold=1.0, min_n=10)
    m.add(0.1)
    s = m.summary()
    assert s['verdict'] == st.INCONCLUSIVE
    assert '样本' in s['note']


def test_no_threshold_is_inconclusive_not_pass():
    """只记录数值的项不能自称通过。"""
    m = st.Metric('bandwidth', 'Hz')
    m.add(1.2)
    assert m.summary()['verdict'] == st.INCONCLUSIVE


def test_bimodal_samples_are_inconclusive_not_averaged():
    """手臂跟踪那个 0.16/0.42 双峰：均值 0.29 在任何一次运行里都不会出现。"""
    m = st.Metric('arm_err', 'm', threshold=0.30, min_n=8)
    for v in (0.16, 0.161, 0.158, 0.163, 0.42, 0.418, 0.425, 0.421):
        m.add(v)
    s = m.summary()
    assert s['bimodal'] is True
    assert s['verdict'] == st.INCONCLUSIVE        # 而不是 PASS(均值 0.29 < 0.30)
    assert s['bimodal_diag']['split_below'] == [0.158, 0.16, 0.161, 0.163]
    assert s['bimodal_diag']['split_above'] == [0.418, 0.42, 0.421, 0.425]


def test_unimodal_samples_are_not_flagged():
    m = st.Metric('err', 'm', threshold=0.02, min_n=8)
    for v in (0.010, 0.011, 0.012, 0.011, 0.013, 0.010, 0.012, 0.011):
        m.add(v)
    s = m.summary()
    assert s['bimodal'] is False
    assert s['verdict'] == st.PASS


def test_small_n_skips_bimodal_check():
    ok, diag = st.looks_bimodal([0.1, 0.9])
    assert ok is False
    assert '不做双峰判定' in diag['reason']


def test_result_verdict_precedence():
    r = st.Result('t', 'sim')
    good = r.metric('a', 'm', threshold=1.0, min_n=1)
    good.add(0.5)
    assert r.verdict == st.PASS
    unk = r.metric('b', 'm')          # 无判据 -> INCONCLUSIVE
    unk.add(0.5)
    assert r.verdict == st.INCONCLUSIVE
    bad = r.metric('c', 'm', threshold=1.0, min_n=1)
    bad.add(9.0)
    assert r.verdict == st.FAIL       # FAIL 优先级最高


def test_empty_result_is_inconclusive():
    assert st.Result('t', 'sim').verdict == st.INCONCLUSIVE


def test_dump_keeps_raw_samples_and_meta():
    """原始样本必须留着，否则换判据就得重跑。"""
    r = st.Result('t', 'sim', context={'cmd_topic': '/cmd_vel'})
    m = r.metric('err', 'm', threshold=0.02, min_n=1, threshold_basis='方案 §2.1')
    m.add(0.01, trial=0, heading_deg=0.0)
    m.add(0.015, trial=1, heading_deg=45.0)
    d = r.to_dict()
    raw = d['metrics'][0]['raw']
    assert [x['value'] for x in raw] == [0.01, 0.015]
    assert raw[1]['heading_deg'] == 45.0
    assert d['context']['cmd_topic'] == '/cmd_vel'


def test_markdown_reports_basis_and_verdict():
    r = st.Result('t_demo', 'sim', description='冒烟')
    m = r.metric('err', 'm', threshold=0.02, min_n=1, threshold_basis='方案 §2.1')
    m.add(0.05)
    md = r.to_markdown()
    assert '# t_demo · FAIL' in md
    assert '方案 §2.1' in md
    assert '≤ 0.02' in md


def test_save_writes_both_files(tmp_path):
    r = st.Result('t_save', 'sim')
    m = r.metric('err', 'm', threshold=1.0, min_n=1)
    m.add(0.1)
    j, md = r.save(str(tmp_path))
    assert os.path.exists(j) and os.path.exists(md)
