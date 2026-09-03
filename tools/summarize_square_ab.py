#!/usr/bin/env python3
# 缩足迹迟滞 A/B 的结果汇总。
#
# 为什么单独写一个脚本而不是每次手算：
#   · 上一轮我把 on/off 的"到位数"直接相比，而两臂时长 249~805s 差 3 倍 ——
#     归一化之后优势消失甚至反转。任何按次数计的量必须除以时长。
#   · 主判据（起点致命）只有在小足迹**真的生效了足够时长**时才归因得上，
#     所以本脚本先打"机制是否生效"的三个数，再打效果量。顺序不可颠倒。
import csv
import statistics
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/square_hyst/results.csv'


def num(s, d=0.0):
    try:
        return float(s)
    except (TypeError, ValueError):
        return d


rows = [r for r in csv.DictReader(open(PATH, encoding='utf-8'))
        if r.get('verdict') != 'SKIP']
if not rows:
    print('没有有效轮次'); sys.exit(1)

# 🔴 臂不符 / gate 未过的轮次必须剔除，不能"顺手也算进去"。
bad = [r for r in rows if r['arm'] != r['arm_verified'] or r['gate_ok'] != '1']
for r in bad:
    print(f"  !! cycle {r['cycle']} 剔除：arm={r['arm']} 反证={r['arm_verified']} gate={r['gate_ok']}")
rows = [r for r in rows if r not in bad]
if not rows:
    print('剔除后没有可信轮次'); sys.exit(1)

print(f'\n可信轮次 {len(rows)} 个（on={sum(1 for r in rows if r["arm"]=="on")} '
      f'off={sum(1 for r in rows if r["arm"]=="off")}）')

# ---------- ① 机制是否真的生效（先看这个，不然效果量归因不了）----------
on = [r for r in rows if r['arm'] == 'on']
if on:
    print('\n===== ① 机制生效度（本轮第一等判据）=====')
    print(f'{"cycle":>5} {"时长s":>6} {"可用拍":>6} {"无余量":>6} {"切换":>4} {"上限":>5} '
          f'{"驻留中位":>8} {"复原总":>6} {"装得下":>6} {"超时":>4} {"压住":>4} '
          f'{"冷却挡":>6} {"退化":>4} {"看门狗":>6} {"瞎眼":>4} {"收尾":>4}')
    for r in on:
        print(f'{r["cycle"]:>5} {num(r["dur_s"]):>6.0f} {r.get("sq_applicable","-"):>6} '
              f'{r["prealign_blocked_fav"]:>6} {r["sq_switch"]:>4} '
              f'{r["sq_switch_ceiling"]:>5} {r["sq_dwell_med_s"]:>8} '
              f'{r.get("sq_revert_total","-"):>6} {r["sq_revert_clear"]:>6} '
              f'{r["sq_revert_timeout"]:>4} {r["sq_sup_dwell"]:>4} '
              f'{r["sq_sup_cooldown"]:>6} {r["sq_exit_degraded"]:>4} '
              f'{r["sq_watchdog"]:>6} {r["sq_wd_blind"]:>4} {r["fp_end_vertices"]:>4}')
    # 🔴 判据修对了没有的直接证据：可用拍 / 无余量 的比值
    for r in on:
        ap = num(r.get('sq_applicable', 0)); bl = num(r['prealign_blocked_fav'])
        if ap + bl > 0:
            print(f'    cycle {r["cycle"]}: 可用拍占比 {ap / (ap + bl) * 100:5.1f}% '
                  f'(可用 {ap:.0f} / 无余量 {bl:.0f}) '
                  f'—— 阈值 253 时代这个占比实测约 0%（909 无余量 vs ~21 可用）')

    # 三条可否证的预测，逐条判定
    print('\n  --- 预测判定 ---')
    meds = [num(r['sq_dwell_med_s'], -1) for r in on if num(r['sq_dwell_med_s'], -1) > 0]
    if meds:
        ok = all(m >= 2.0 for m in meds)
        print(f'  {"✅" if ok else "🔴"} 驻留中位 >= 最短驻留 2.0s: 实测 {meds} '
              f'（上一轮 0.15s）')
    else:
        print('  ⚠️  没有"装得下"复原样本 —— 要么在锁存，要么日志串漂了，要么根本没切入')
    over = [(r['cycle'], r['sq_switch'], r['sq_switch_ceiling']) for r in on
            if int(r['sq_switch']) > int(r['sq_switch_ceiling']) > 0]
    print(f'  {"🔴" if over else "✅"} 切换次数 <= 算术上限: '
          f'{"超上限 " + str(over) if over else "全部在上限内"}')
    latch = [r['cycle'] for r in on if r['fp_end_vertices'] not in ('8', '?')]
    print(f'  {"🔴" if latch else "✅"} 收尾足迹复原为 8 顶点: '
          f'{"cycle " + str(latch) + " 被锁存！" if latch else "无锁存"}')
    blind = [r['cycle'] for r in on if int(num(r['sq_wd_blind'])) > 0]
    print(f'  {"⚠️ " if blind else "✅"} 看门狗未瞎眼: '
          f'{"cycle " + str(blind) + " 出现瞎眼" if blind else "全程读数是活的"}')

# ---------- ② 效果量（**必须按时长归一化**）----------
print('\n===== ② 效果量（按时长归一化，单位 次/min）=====')
KEYS = [('lethal_start', '起点致命'), ('plan_abort', '规划abort'),
        ('arrive', '到位'), ('narrow_engage', '窄通道接管'),
        ('narrow_saturated', '足迹饱和'), ('escape', 'ESCAPE')]
for arm in ('on', 'off'):
    grp = [r for r in rows if r['arm'] == arm]
    if not grp:
        continue
    tot_min = sum(num(r['dur_s']) for r in grp) / 60.0
    print(f'\n  [{arm}] {len(grp)} 轮，合计 {tot_min:.1f} min '
          f'（各轮时长 {[int(num(r["dur_s"])) for r in grp]}）')
    for k, label in KEYS:
        tot = sum(num(r[k]) for r in grp)
        print(f'    {label:<10} 合计 {tot:>5.0f}  归一化 {tot / tot_min:>6.2f} 次/min')

# ---------- ③ 样本量提醒 ----------
n_on = sum(1 for r in rows if r['arm'] == 'on')
n_off = sum(1 for r in rows if r['arm'] == 'off')
print(f'\n===== ③ 样本量 =====')
if n_on < 3 or n_off < 3:
    print(f'  ⚠️  on={n_on} off={n_off} —— 任一臂 <3 轮时**不要下效果量结论**。')
    print('     上一轮我在 n=4/5 时宣布"零例外"，随后被自己的 cycle 5 否证。')
else:
    print(f'  on={n_on} off={n_off}，可以谈趋势；仍需看逐轮离散度而不只看均值。')
