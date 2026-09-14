---
name: ros2-stack-ops
description: 在本仓库启动/清理/探查 ROS2+Gazebo 栈时必须用的安全流程。凡是要 pkill/pgrep 进程、数进程个数、查话题发布者或消息数、判断"栈起来了没"，一律先读这个 skill。它拦的是四类会把结论整个带偏、且我已反复犯过的错误。
---

# ROS2 栈操作：四条必须照做的规程

这个 skill 存在的唯一理由：下面四类错误我**每一条都有 memory、每一条仍然又犯了**。
memory 是"读到了才想起"，规程是"动手前照着做"。所以这里给的是**可直接执行的脚本**，
不是提醒。

---

## 规程 1 · 绝不把进程名模式写进命令行(`pkill -f` 会杀掉自己)

**已犯 2 次，最近一次 exit 144。**

Bash 工具跑的是 `bash -c '<整段脚本>'`，所以**整段脚本的文本就是那个进程的 cmdline**。
`pkill -f "rviz2|nav2_|..."` 里的模式串同时存在于我自己 shell 的 cmdline 里 →
pkill 把发起清理的 shell 一起杀了。症状极具误导性：命令静默中断、
后面要写的日志文件不存在，看起来像"启动失败"。

`pgrep -f` 同理：数出来的个数里含我自己，`pgrep -fc rviz2` 在 rviz 根本没起时也返回 ≥1。

### 照做：模式存文件，按 PID 杀，排除自身进程链

一次性建好这两个脚本（`patterns.txt` 里一行一个模式）：

```bash
mkdir -p /tmp/rosops
cat > /tmp/rosops/patterns.txt <<'EOF'
ign gazebo
ruby /usr/bin/ign
parameter_bridge
ros_gz
nav2_
controller_server
planner_server
bt_navigator
behavior_server
smoother_server
waypoint_follower
velocity_smoother
lifecycle_manager
slam_toolbox
exploration_coordinator
frontier_explorer
pointcloud_slice
omni_effort_drive
arm_chassis_speed
cmd_vel_body_to_world
robot_state_publisher
rviz2
controller_manager/spawner
explore_metrics_recorder
EOF

cat > /tmp/rosops/_selfsafe.sh <<'EOF'
# 求出自己这条进程链，供调用方排除。被 source，不单独执行。
_anc() { local p=$$ out=""; while [ "$p" -gt 1 ]; do out="$out $p";
  p=$(ps -o ppid= -p "$p" 2>/dev/null|tr -d ' '); [ -z "$p" ] && break; done; echo "$out"; }

# 判断一个 PID 是不是"工具 shell"。Codex 每次 Bash 调用都是
# `bash -c 'source .../shell-snapshots/snapshot-bash-*.sh; <脚本>'`，
# 这类进程的 cmdline 里含有我写的模式字串，会被 pgrep -f 命中，
# 但它们**不是栈进程**：杀掉会打断我自己的后台监控，数进去则把
# "存活 0" 变成 "存活 3"，两种都会让结论错。
# 光排除自身进程链**不够** —— 并发的其它工具 shell（后台 launch、
# until 等待循环）不在我的祖先链上。这一条是实跑脚本才暴露出来的。
_is_tool_shell() { grep -qa 'shell-snapshots/snapshot-' /proc/"$1"/cmdline 2>/dev/null; }

_pids_for() {                      # $1=模式文件；输出去重、排除自身链与工具 shell
  local anc; anc=" $(_anc) "; local out=""
  while IFS= read -r pat; do [ -z "$pat" ] && continue
    for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
      case "$anc" in *" $pid "*) continue;; esac
      _is_tool_shell "$pid" && continue
      case " $out " in *" $pid "*) ;; *) out="$out $pid";; esac
    done
  done < "$1"; echo "$out"; }
EOF

cat > /tmp/rosops/count.sh <<'EOF'
#!/usr/bin/env bash
set -uo pipefail; source /tmp/rosops/_selfsafe.sh
v="$(_pids_for /tmp/rosops/patterns.txt)"; n=0
for p in $v; do n=$((n+1)); printf "  pid=%-8s %s\n" "$p" "$(ps -o args= -p "$p" 2>/dev/null|cut -c1-90)"; done
echo "  ---- 存活 $n ----"
EOF

cat > /tmp/rosops/cleanup.sh <<'EOF'
#!/usr/bin/env bash
set -uo pipefail; source /tmp/rosops/_selfsafe.sh
v="$(_pids_for /tmp/rosops/patterns.txt)"
[ -n "${v// /}" ] && kill $v 2>/dev/null; sleep 3
v="$(_pids_for /tmp/rosops/patterns.txt)"
[ -n "${v// /}" ] && kill -9 $v 2>/dev/null; sleep 1
# 三件都要做：清理漏 sem. 前缀时实测还剩几十个；陈旧 daemon 会让话题数 80 掉到 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
(/opt/ros/humble/bin/ros2 daemon stop >/dev/null 2>&1 || true)
echo "清理完成。残留 shm: $(ls /dev/shm/ 2>/dev/null | grep -c fastrtps)"
bash /tmp/rosops/count.sh
EOF
chmod +x /tmp/rosops/count.sh /tmp/rosops/cleanup.sh
```

用法：`bash /tmp/rosops/cleanup.sh` / `bash /tmp/rosops/count.sh`。

**禁止**：在任何 Bash 调用里直接写 `pkill -f "…"`、`pgrep -fc <名字>`、
或把进程名列在 `for f in ... ; do pgrep` 的循环里。要计数就调 `count.sh`。

清理还必须覆盖 `/opt/ros/humble` 下的二进制（`nav2_*`、`parameter_bridge`）——
只按工作空间路径匹配会漏掉它们，留下的僵尸会污染下一轮测量。

---

## 规程 2 · 查询侧 domain 必须先对齐，且要证明而不是假设

**已犯 2 次。** 症状：所有话题 `pub=0`、节点一个都看不见，看起来像"系统没起来"。

本仓库的 domain 是**被 launch 钉死的**，不是由我的 shell 决定：
`warehouse_sim.launch.py` 用 `SetEnvironmentVariable('ROS_DOMAIN_ID', '25')`
（有意为之，注释写了理由：避免与同机其它 ROS2 图撞名）。
我在 shell 里 `export ROS_DOMAIN_ID=57` **不会**改变它拉起的任何进程。

### 照做：从运行中的进程反读，而不是从自己的 shell 猜

```bash
cat > /tmp/rosops/align_env.sh <<'EOF'
#!/usr/bin/env bash
# 从一个**运行中的栈进程**反读 DDS 环境，生成查询侧要 source 的文件。
# 反读而不是假设：domain 由 launch 决定，我 export 什么都不影响它。
set -uo pipefail; source /tmp/rosops/_selfsafe.sh
PID=""
for cand in controller_server slam_toolbox robot_state_publisher; do
  for p in $(pgrep -f -- "$cand" 2>/dev/null); do
    case " $(_anc) " in *" $p "*) continue;; esac
    PID="$p"; break 2
  done
done
[ -z "$PID" ] && { echo "栈没在跑，无从反读 —— 先启动，或本来就该报'栈没起来'"; exit 2; }
echo "# 反读自 pid=$PID ($(ps -o comm= -p $PID))"
{ echo 'set +u'
  echo 'source /opt/ros/humble/setup.bash'
  echo 'source /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/install/setup.bash'
  echo 'set -u'
  tr '\0' '\n' < /proc/$PID/environ | grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|FASTRTPS_DEFAULT_PROFILES_FILE|IGN_IP|GZ_IP)=' | sed 's/^/export /'
  echo 'export DISPLAY=${DISPLAY:-:1}'
} > /tmp/rosops/query_env.sh
cat /tmp/rosops/query_env.sh | grep '^export' | sed 's/^/  /'
EOF
chmod +x /tmp/rosops/align_env.sh
```

流程固定为：**启动栈 → `bash /tmp/rosops/align_env.sh` → 之后每个查询 shell
`source /tmp/rosops/query_env.sh`**。

只有在这一步做完之后，"话题没有发布者"才可以被当成系统问题。

---

## 规程 3 · "栈起来了没"的判据顺序(每一层用对的工具)

**禁止用进程数当判据**（实测 9 个进程全活而 `lifecycle_manager` 报
`Aborting bringup`，整套 nav2 从未 activate）。
**也禁止用发布者数当判据**（实测 `pub=1` 但 `0Hz`：BEST_EFFORT 发布者 +
RELIABLE 订阅者，一帧都收不到、只有一条 WARNING）。

按层次逐级判，上一层不过就不要查下一层：

| 层 | 判据 | 命令 | 坏掉的样子 |
|---|---|---|---|
| 0 仿真物理 | `ign topic -l` 条数 / `/stats` 有 `iterations` | 见规程 4 | 个位数 / echo 超时 |
| 1 时钟 | `/clock` **实测帧数** > 0 且时间在前进 | 订阅数 8s | 0 帧 = 全栈冻结 |
| 2 TF | `map→astribot_torso_base` 能查到且龄期小 | `lookup_transform` | 根 frame 不是 base_link |
| 3 生命周期 | 每个节点 `get_state` == `active` | `ros2 service call .../get_state` | 进程活着但从未 activate |
| 4 数据 | 每条话题的**实测拍率** | 订阅计数 | pub>0 而 0Hz |

时钟这一层是**最省时的分水岭**：`use_sim_time=true` 且 `/clock` 不推进时，
所有节点时钟恒为 0，`RCLCPP_*_THROTTLE` 永远被节流掉 →
节点"活着但一条日志都不打"，而 `create_wall_timer` 不受影响仍在跑、
状态话题照发。这个组合极像 DDS/QoS 故障，会把人引到完全错误的方向。

---

## 规程 4 · Gazebo 不步进：先在 ign-transport 层判，别碰 ROS 侧

ign-transport 与 ROS domain 无关，所以不受规程 2 的坑干扰。**这是第一步，不是第三步。**

```bash
cat > /tmp/rosops/gz_check.sh <<'EOF'
#!/usr/bin/env bash
# Gazebo 到底在不在步进。三个数按顺序看，任一不对就不用查 ROS 侧。
set -uo pipefail
export IGN_IP=${IGN_IP:-127.0.0.1}
LOG="${1:-}"
if [ -n "$LOG" ] && [ -f "$LOG" ]; then
  echo "  [ign gazebo-1] 日志行数: $(grep -c 'ign gazebo-1' "$LOG")   (个位数=卡在插件加载)"
  echo "  最后一条 ign gazebo 行:"
  grep 'ign gazebo-1' "$LOG" | tail -1 | cut -c1-160 | sed 's/^/    /'
fi
echo "  ign topic -l: $(timeout 15 ign topic -l 2>/dev/null | wc -l) 个   (正常 ~21，个位数=没步进)"
echo "  server 自己的 CPU（不看 ruby、不看总量）:"
ps -eo pcpu,args --no-headers | grep -F 'gazebo server' | grep -v grep \
  | awk '{printf "    %s%%   (正常 ~196%%)\n",$1}'
echo "  /stats:"
timeout 15 ign topic -e -t /stats -n 1 2>&1 \
  | grep -E 'iterations|sim_time|real_time_factor' | head -4 | sed 's/^/    /' \
  || echo "    **超时/无消息 = 物理一步都没走**"
EOF
chmod +x /tmp/rosops/gz_check.sh
```

### 已知卡点：`gz_ros2_control` 取 `robot_description` 的加载期死锁

server 的**最后一行**每次都是同一行：

```
[gz_ros2_control]: connected to service!! robot_state_publisher asking for robot_description
```

插件在 `Configure()` 里同步等 `robot_description`，Gazebo 主循环被卡在插件里，
物理一步都不走。连带：`controller_manager` 服务 0 个 →
`joint_state_broadcaster` spawner 60s 后 FATAL → `/joint_states` 无发布者 →
`/clock` 无消息 → 全栈冻结。

**关于 `headless` 的因果，我写错过两次，现在只写实测：**

| 日期 | `headless` | `[ign gazebo-1]` 行数 | 结果 |
|---|---|---|---|
| 2026-08-20 | `true` | 4 | 卡死 |
| 2026-08-20 | `false` | 267 | 正常，RTF 1.0068 |
| 2026-08-27 | `false` | 6 | **卡死**（连续 2 次，同一行） |

所以"headless 是触发条件、GUI 能跑"这条**不成立**——`headless:=false` 一样会卡。
`headless` 与这个死锁**没有稳定的因果关系**，别再把它当开关用。
同样已证伪的还有"GUI 软渲染吃满 CPU 饿死 server"：本机 28 核，
138% 的 GUI 不可能饿死 server（这条算术当年就该做）。

**根因未定。所以：不要宣布已修好，要给出实测过的那三个数。**

---

## 规程 5 · 清理是破坏性的：先确认机器没有别人在用

**已犯 1 次，代价是一整条错结论。**

2026-08-27 实测：**同一台机器上有另一个会话在跑同一套仿真栈**，
共用 `ROS_DOMAIN_ID=25`、共用同一份 `ws_robot/install/`、进程模式完全重叠。
它先跑了自己的清理，把我 4 分钟前起的实验杀了；我读到"卡在 6 行"，
据此写下"第 2 次同样卡死 → 是必然不是竞态"——**那个读数是它被杀时的状态，
不是稳定状态，结论整个是错的。**

反向同样成立：我跑一次 `cleanup.sh` 就会杀掉它正在跑的实验。

### 照做：清理前先查"这台机器上的栈是谁起的"

```bash
cat > /tmp/rosops/whose_stack.sh <<'EOF'
#!/usr/bin/env bash
# 清理前必跑。判断当前活着的栈是不是我起的 —— 靠 launch 进程的 stdout 指向哪个日志。
set -uo pipefail; source /tmp/rosops/_selfsafe.sh
MINE="${1:-}"                     # 我这轮的日志目录，例如 /tmp/mysweep/run1
found=0
for p in $(pgrep -f -- 'ros2 launch' 2>/dev/null); do
  _is_tool_shell "$p" && continue
  log="$(readlink /proc/$p/fd/1 2>/dev/null || echo '?')"
  start="$(ps -o lstart= -p "$p" 2>/dev/null)"
  tag="**不是我的**"
  [ -n "$MINE" ] && case "$log" in "$MINE"*) tag="（我的）";; esac
  echo "  launch pid=$p  起于$start"
  echo "     日志 -> $log   $tag"
  found=1
done
[ "$found" -eq 0 ] && echo "  没有 ros2 launch 在跑"
echo "  其它会话的痕迹（/tmp 下最近 1 小时被别人建的实验目录）:"
find /tmp -maxdepth 1 -type d -newermt '-60 min' 2>/dev/null \
  | grep -vE '^/tmp$|/tmp/rosops|/tmp/Codex|/tmp/\.' | sed 's/^/     /'
EOF
chmod +x /tmp/rosops/whose_stack.sh
```

**动手前**：`bash /tmp/rosops/whose_stack.sh /tmp/<我的日志目录>`
出现"不是我的"就**停下来问用户**，不要清理。

**动手后同样重要**：读到任何异常状态（卡住、行数不涨、话题消失）时，
先看自己的日志有没有 `signal_handler(SIGINT/SIGTERM)` /
`Escalating to SIGKILL` / `process has died`。有 → 是**外部把它杀了**，
这个读数不能用来下任何结论。

### 隔离手段（要与人共用机器时）

| 手段 | 作用 |
|---|---|
| 自己的日志目录 `/tmp/<我的名字>/runN/` | 能靠 `readlink /proc/PID/fd/1` 区分是谁的进程 |
| 启动期间**不跑**任何清理脚本 | 清理与实验必须串行 |
| 结论前核对时间线 | 日志停止写入的时刻 vs 我做过的操作时刻 |

`ROS_DOMAIN_ID` **不能**拿来隔离：本仓库由
`warehouse_sim.launch.py` 钉死成 25（见规程 2），改它等于改仓库行为。

---

## 收尾自检(每次动完栈都跑一遍)

```bash
bash /tmp/rosops/count.sh          # 存活进程（不含自身）
bash /tmp/rosops/gz_check.sh <log> # 仿真是否步进
source /tmp/rosops/query_env.sh    # 对齐后的查询环境
```

**报结论时的硬要求**：
- 说"栈起来了"必须附生命周期状态或实测拍率，不能附进程数。
- 说"清理干净了"必须附 `count.sh` 的输出，不能附自己写的 `pgrep`。
- 说"修好了"必须附修之前/之后的同一个数；只能规避不能解释时，
  明确写"规避手段，根因未知"。
