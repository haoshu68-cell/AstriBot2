#!/usr/bin/env bash
# 反复启动整套仿真栈 N 轮，逐轮记录**实测数字**，最后给一张表和一个发生率。
#
# 用法（`--` 之后的参数**原样**转交 launch_sim_stack.sh）：
#   tools/repeat_launch_verify.sh --runs 5 -- --mode explore --max-linear-speed 0.6
#   tools/repeat_launch_verify.sh --runs 3 -- --mode localize --map maps/demo_warehouse
#   tools/repeat_launch_verify.sh --runs 5 --dry-run -- --mode explore
#
# ════════ 为什么要有这个东西 ════════
# 单轮跑通证明不了任何事。这套栈上已知有一类**偶发**启动故障：进程活着、日志正常，
# 但 participant 在 DDS 图里一次都不出现（robot_state_publisher 1200s 只烧 3.7s CPU、
# livox_fusion_node 只烧 10s，而健康的预处理是 28%）。它每次中招的节点还不一样。
# 对这种故障，"我跑了一次是好的"和"我跑了一次是坏的"信息量一样接近零 ——
# 要的是 **分子/分母**：N 轮里闸门拦住了几轮。
# 所以这个脚本的输出核心不是 ✅，而是一张逐轮的数字表 + 一个带分母的发生率。
#
# ════════ 判据怎么定的（这几条都是踩过的）════════
# 1) 逐轮结论一律取 launch_sim_stack.sh 的**退出码**，再用日志里的失败行交叉核对。
#    不用 grep '✅' 数成功：本仓库有过 8 笔提交全报 ✅ 而每笔都带 1 failed 的教训
#    —— 只找成功词的判据必然假阳性。所以这里**显式枚举失败词**并计数。
# 2) 每轮包 `timeout --signal=KILL`：一轮挂死不能把整个 sweep 拖住。
#    被强杀的轮次单独记成 TIMEOUT 类，不混进"失败"也不混进"通过"。
# 3) 退出码 2/3/4（入参错 / 机器上有别人的栈 / 清理没干净）**立即中止整个 sweep**。
#    这三种是环境问题，继续跑只会产出一堆无意义的行。
# 4) 轮与轮之间**必须**清理。launch_sim_stack.sh 在失败时刻意保留栈（为了留现场），
#    对单次排查是对的，对 sweep 是污染 —— 下一轮的读数会带上一轮的残留。
#    现场不会丢：每轮的日志目录是独立的，证据文件都在里面。
# 5) grep -c 的坑：没命中时它**已经打印 0** 而退出码是 1。写 `|| echo 0` 会打印两个 0，
#    只能 `|| true`。这个坑在 launch_sim_stack.sh 里也有一份注释。
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

RUNS=3
OUTDIR=""
RUN_TIMEOUT=2400        # 单轮硬上限。默认按 launch 的默认值算：
                        # 3 次尝试 ×2 道闸门 ×(200+60) + 验证(120+240) + 余量
STOP_ON_FAIL="false"
KEEP_ON_FAIL="false"
DRY_RUN="false"
PASS=()

usage() {
  sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

选项：
  --runs N            跑几轮，默认 3
  --out DIR           输出根目录，默认 /tmp/simstack/sweep_<时间戳>
  --run-timeout SEC   单轮硬上限，默认 2400（超了记 TIMEOUT，不记失败）
  --stop-on-fail      第一轮失败就停（默认跑满 N 轮才有分母，所以默认不停）
  --keep-on-fail      失败那轮**不清理**，留活体现场，并停止 sweep
  --dry-run           只打印每轮将要执行的命令与输出路径，不启动任何东西
  --                  之后的参数原样转交 launch_sim_stack.sh（至少要有 --mode）
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --runs)         RUNS="${2:-}"; shift 2 ;;
    --out)          OUTDIR="${2:-}"; shift 2 ;;
    --run-timeout)  RUN_TIMEOUT="${2:-}"; shift 2 ;;
    --stop-on-fail) STOP_ON_FAIL="true"; shift ;;
    --keep-on-fail) KEEP_ON_FAIL="true"; shift ;;
    --dry-run)      DRY_RUN="true"; shift ;;
    -h|--help)      usage; exit 0 ;;
    --)             shift; PASS=("$@"); break ;;
    *) echo "🔴 未知选项: $1（转交给 launch_sim_stack.sh 的参数要放在 -- 之后）" >&2
       echo; usage; exit 2 ;;
  esac
done

die() { echo "🔴 $*" >&2; exit 2; }

case "$RUNS" in ''|*[!0-9]*) die "--runs 要给正整数，收到 '$RUNS'" ;; esac
[ "$RUNS" -ge 1 ] || die "--runs 至少是 1"
[ "${#PASS[@]}" -gt 0 ] || die "-- 之后必须给转交参数（至少 --mode explore|localize）
   例: tools/repeat_launch_verify.sh --runs 5 -- --mode explore"

# 输出根目录必须在 /tmp/simstack/ 下面：launch_sim_stack.sh 的"这栈是谁起的"守卫
# 靠 launch 进程 stdout 指向哪个文件来判，只有 /tmp/simstack/** 才算"本脚本起的"。
# 放到别处，第二轮会把第一轮的残留当成**别人的栈**而以退出码 3 停下。
if [ -z "$OUTDIR" ]; then
  OUTDIR="/tmp/simstack/sweep_$(date +%Y%m%d_%H%M%S)"
fi
case "$OUTDIR" in
  /tmp/simstack/*) ;;
  *) die "--out 必须在 /tmp/simstack/ 下（见脚本里这一段的注释：守卫靠这个路径认主）" ;;
esac
mkdir -p "$OUTDIR" || die "建不出输出目录 $OUTDIR"
TSV="$OUTDIR/sweep.tsv"

echo "════════════════════════════════════════════════════════════"
echo " 多轮启动验证"
echo "   轮数      : $RUNS"
echo "   转交参数  : ${PASS[*]}"
echo "   单轮上限  : ${RUN_TIMEOUT}s"
echo "   输出      : $OUTDIR"
echo "════════════════════════════════════════════════════════════"

if [ "$DRY_RUN" = "true" ]; then
  echo
  for i in $(seq 1 "$RUNS"); do
    echo "  第 $i 轮: timeout --signal=KILL ${RUN_TIMEOUT}s bash tools/launch_sim_stack.sh ${PASS[*]} --log-dir $OUTDIR/run$i"
    echo "           驱动输出 -> $OUTDIR/run$i.driver.log"
  done
  echo "  轮间清理: bash tools/clean_sim_stack.sh -> $OUTDIR/runN.clean.log"
  echo "  汇总     : $TSV"
  echo
  echo "✅ --dry-run：只打印，未启动任何东西。"
  exit 0
fi

printf '轮次\t退出码\t结果\t重试次数\t闸门A秒\tclock帧\t闸门B秒\tTF戳前进s\tfusedHz\tscanHz\t失败层\t用时s\n' > "$TSV"

n_pass=0; n_fail_infra_chain=0; n_fail_infra_deadlock=0; n_fail_verify=0; n_timeout=0
aborted=""

# ---- 从一轮的驱动日志里抽数字。抽不到就留 "-"，绝不填 0：
#      0 和"没抽到"在汇总表里是两个完全不同的意思。
pluck() {   # $1=日志 $2=sed 表达式
  local v; v="$(sed -n "$2" "$1" 2>/dev/null | tail -1)"
  printf '%s' "${v:--}"
}

for i in $(seq 1 "$RUNS"); do
  RUNDIR="$OUTDIR/run$i"
  DRIVER="$OUTDIR/run$i.driver.log"
  echo
  echo "──────── 第 $i / $RUNS 轮 ────────"
  t_start=$(date +%s)
  timeout --signal=KILL "$RUN_TIMEOUT" \
    bash "$REPO/tools/launch_sim_stack.sh" "${PASS[@]}" --log-dir "$RUNDIR" \
    > "$DRIVER" 2>&1
  rc=$?
  t_used=$(( $(date +%s) - t_start ))

  # 逐轮抽数（都来自驱动日志的实测行）
  gA=$(pluck "$DRIVER" 's/.*仿真在步进：等待 \([0-9.]*\)s.*/\1/p')
  clk=$(pluck "$DRIVER" 's/.*收到 \([0-9]*\) 帧 \/clock.*/\1/p')
  gB=$(pluck "$DRIVER" 's/.*链就绪.*等待 \([0-9.]*\)s.*/\1/p')
  tfa=$(pluck "$DRIVER" 's/.*戳前进\([0-9.]*\)s.*/\1/p')
  fus=$(pluck "$DRIVER" 's/.*fused_points=\([0-9.]*\)Hz.*/\1/p')
  scn="$(awk '/RSP的TF/ {for (j = 1; j <= NF; j++) if ($j ~ /scan/ && $j ~ /=/) {
             split($j, a, "="); gsub(/Hz/, "", a[2]); v = a[2] } } END { print (v == "" ? "-" : v) }' \
         "$DRIVER" 2>/dev/null)"
  # 重试次数 = 日志里出现过几次"第 N 次启动"横幅（第一次不打横幅，所以这就是重试数）
  retry="$({ grep -acF '次启动 ----' "$DRIVER" || true; } 2>/dev/null)"
  # 失败层：验证脚本把每条失败都打成 "🔴 [Lx ...] ..."，这里只取方括号里的层名。
  # ⚠️ `^ *` 不能省：这两行在驱动日志里有 3 个前导空格（launch_sim_stack.sh 把
  # 验证脚本的输出整体缩进了）。第一版写成 `^🔴 \[` 时这一列**恒为空** ——
  # 拿真实日志核对才发现：同一份日志里 🔴 计数是 5，而抽到的失败层是 0 个。
  # 一个恒为空的列不会报错，只会让汇总表看起来"没有失败层"。
  layers="$(sed -n 's/^ *🔴 \[\([^]]*\)\].*/\1/p' "$DRIVER" 2>/dev/null | paste -sd, - )"
  [ -z "$layers" ] && layers='-'

  # ---- 归类。**先看退出码**，再用日志里的失败词交叉核对（不靠 ✅ 判成功）
  case "$rc" in
    0)
      # 交叉核对：退出码 0 但日志里仍有 🔴 就不算通过 —— 宁可报"对不上"
      bad="$({ grep -acF '🔴' "$DRIVER" || true; } 2>/dev/null)"
      if [ "$bad" -gt 0 ]; then
        verdict="退出码0但有${bad}条🔴(判据不一致)"; n_fail_verify=$((n_fail_verify+1))
      else
        verdict="全过"; n_pass=$((n_pass+1))
      fi ;;
    137)
      verdict="TIMEOUT(${RUN_TIMEOUT}s强杀)"; n_timeout=$((n_timeout+1)) ;;
    5)
      # ⚠️ 判据必须用 launch_sim_stack.sh **最终出口**那句话，不能用「日志里出现过
      # 感知/TF 链这几个字」。理由是实测出来的：闸门 B 通过时打的是
      # 「✅ 感知/TF 链就绪」—— 同样含这几个字。而重试是分次的，
      # 一轮里完全可能第 1 次死锁、第 2 次链故障、第 3 次又死锁，
      # 那种情况下"出现过"会把最终成因判反。最终出口只会打其中一句。
      if grep -qaF '没能让感知/TF 链出数据' "$DRIVER" 2>/dev/null; then
        verdict="闸门B拦住(感知/TF链)"; n_fail_infra_chain=$((n_fail_infra_chain+1))
      elif grep -qaF '全部卡在加载期死锁' "$DRIVER" 2>/dev/null; then
        verdict="闸门A拦住(加载期死锁)"; n_fail_infra_deadlock=$((n_fail_infra_deadlock+1))
      else
        # rc=5 却两句都没有 = 上游改了措辞而这里没跟上。报"认不出"而不是随便归一类。
        verdict="rc=5但认不出成因(措辞变了?)"; n_fail_verify=$((n_fail_verify+1))
      fi
      # 顺带记下这一轮里两类各出现过几次（重试的中间过程，用于看是不是混合）
      d_cnt="$({ grep -acF '次是加载期死锁' "$DRIVER" || true; } 2>/dev/null)"
      c_cnt="$({ grep -acF '次是感知/TF 链故障' "$DRIVER" || true; } 2>/dev/null)"
      echo "  （重试过程中：死锁 ${d_cnt} 次、链故障 ${c_cnt} 次）" ;;
    2|3|4)
      verdict="环境问题(rc=$rc)"; aborted="第 $i 轮以 rc=$rc 退出" ;;
    *)
      verdict="验证未过(rc=$rc)"; n_fail_verify=$((n_fail_verify+1)) ;;
  esac

  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$i" "$rc" "$verdict" "$retry" "$gA" "$clk" "$gB" "$tfa" "$fus" "$scn" "$layers" "$t_used" >> "$TSV"
  echo "  rc=$rc  $verdict   用时 ${t_used}s"
  echo "  闸门A ${gA}s/${clk}帧   闸门B ${gB}s（TF戳前进 ${tfa}s, fused ${fus}Hz, scan ${scn}Hz）   重试 ${retry} 次"
  [ "$layers" != '-' ] && echo "  失败层: $layers"

  if [ -n "$aborted" ]; then
    echo "  🔴 这是环境问题，继续跑只会产出无意义的行 —— 中止 sweep。日志 $DRIVER"
    break
  fi

  if [ "$rc" -ne 0 ] && [ "$KEEP_ON_FAIL" = "true" ]; then
    echo "  ⚠️ --keep-on-fail：这轮的栈**不清理**，活体留着查。sweep 就停在这里。"
    aborted="第 $i 轮失败且 --keep-on-fail"
    break
  fi

  # 轮间清理（见文件头判据 4）。最后一轮也清 —— sweep 的语义是"跑完就交还机器"。
  if ! bash "$REPO/tools/clean_sim_stack.sh" > "$OUTDIR/run$i.clean.log" 2>&1; then
    echo "  🔴 轮间清理判定未清干净，后面每一轮的读数都不可信 —— 中止 sweep。"
    sed 's/^/     /' "$OUTDIR/run$i.clean.log"
    aborted="第 $i 轮的轮间清理失败"
    break
  fi
  echo "  轮间清理: $({ grep -aE '^结果' "$OUTDIR/run$i.clean.log" || true; } 2>/dev/null)"

  if [ "$rc" -ne 0 ] && [ "$STOP_ON_FAIL" = "true" ]; then
    aborted="第 $i 轮失败且 --stop-on-fail"
    break
  fi
done

done_runs=$(( $(wc -l < "$TSV") - 1 ))

echo
echo "════════════════════════════════════════════════════════════"
echo " 逐轮实测（同样落在 $TSV，可直接贴报告）"
echo "════════════════════════════════════════════════════════════"
column -t -s $'\t' "$TSV" 2>/dev/null | sed 's/^/  /' || sed 's/^/  /' "$TSV"

echo
echo "════════════════════════════════════════════════════════════"
echo " 汇总（分母 = 实际跑完的轮数 $done_runs，不是 --runs $RUNS）"
echo "════════════════════════════════════════════════════════════"
printf '  %-28s %s / %s\n' '全过'                "$n_pass"                "$done_runs"
printf '  %-28s %s / %s\n' '闸门B拦住(感知/TF链)' "$n_fail_infra_chain"    "$done_runs"
printf '  %-28s %s / %s\n' '闸门A拦住(加载期死锁)' "$n_fail_infra_deadlock" "$done_runs"
printf '  %-28s %s / %s\n' '验证层未过'          "$n_fail_verify"         "$done_runs"
printf '  %-28s %s / %s\n' 'TIMEOUT(被强杀)'     "$n_timeout"             "$done_runs"
[ -n "$aborted" ] && echo "  ⚠️ sweep 提前中止: $aborted"

echo
echo "  ⚠️ 这两个数是 $done_runs 轮的**样本**，不是概率。偶发故障的发生率要靠轮数堆出来；"
echo "     $done_runs 轮里 0 次不等于不会发生（那类故障已实测到过连续三次各坏一个不同节点）。"
echo "  证据: 每轮 $OUTDIR/run<i>/ 下有 stack.log / verify.log / verify.tsv；"
echo "        闸门不过的轮次还有 chain_evidence_*.txt 或 deadlock_evidence_*.txt。"

# 退出码：只有"跑完了 N 轮且每轮都全过"才 0。提前中止一律非 0。
if [ -n "$aborted" ]; then exit 6; fi
if [ "$n_pass" -eq "$done_runs" ] && [ "$done_runs" -eq "$RUNS" ]; then
  echo
  echo "✅ $RUNS 轮全部全过。"
  exit 0
fi
echo
echo "🔴 $done_runs 轮里有 $(( done_runs - n_pass )) 轮没全过（明细见上表的「结果」与「失败层」两列）。"
exit 1
