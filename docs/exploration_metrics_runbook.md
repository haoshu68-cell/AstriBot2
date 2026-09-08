# 自主探索仿真 + 指标采集 操作流程

面向：在这台开发机上跑一次"自主探索建图"，录 bag，出路径评价指标报告。

一条命令跑完全程：

```bash
RUN_LABEL=run12 nohup bash tools/run_explore_with_bag.sh > /tmp/run12_launcher.log 2>&1 &
```

产物全部落在 `/tmp/explore_<RUN_LABEL>/`：

| 路径 | 内容 |
|---|---|
| `bag/` | 白名单话题 bag（一次 836s 的跑约 480MB） |
| `bag_actions/` | action 状态/反馈（**隐藏话题，必须单独一个 bag**） |
| `metrics/<label>_rounds.csv` | 逐轮 130 列指标 |
| `metrics/<label>_run.json` | 汇总（97 个数值列的 n/mean/sd/max） |
| `metrics/samples/<label>_round_NNN.csv.gz` | 逐轮原始样本，长格式 `channel,t,a,b,c` |
| `stack.log` | 整个 launch 的合并日志，排查第一站 |
| `chain_check.log` | 感知链逐跳帧数表，第一个 🔴 是断点 |
| `stack_deadlock_attemptN.log` | 若发生加载期死锁，那一次的现场 |

出报告：

```bash
python3 tools/report_explore_metrics.py /tmp/explore_run12/metrics \
        --md /tmp/explore_run12/report.md
```

---

## 流程的五步，以及每一步为什么必须在那个位置

顺序不是随手排的，每一条都是踩出来的。改顺序前先读完这一节。

### [1/5] 清栈

`tools/clean_sim_stack.sh`。判据是"清理后残留 0 个 + `/clock` 发布者数 0"，不是脚本退出码。

两个已知坑：

- 按工作区路径杀进程会**漏掉 `/opt/ros/humble` 下的二进制**（`parameter_bridge`、`nav2_*`），留下的僵尸会污染下一轮测量。
- 残留 `/dev/shm/fastrtps_*`（**含 `sem.fastrtps_*` 前缀**）会让**新起的**参与者报 `open_and_lock_file failed`，老进程不受影响 —— 于是症状是"只有我的探针连不上"，极易误判成栈坏了。脚本之后还兜底扫一遍 0 字节残留。

### [2/5] 起仿真栈 + **步进自检与自动重启**

```bash
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=mapping exploration:=true controller_plugin:=mppi \
  use_rviz:=true headless:=false
```

`use_rviz:=true` 是硬要求（每次验证都要同时起 Gazebo 和 rviz）。起完必须核对 `rviz2` 进程数 == 1 —— 参数漏传不报错。

紧接着跑 `tools/wait_sim_stepping.py`，只回答一个问题：**仿真在走吗**。判据是 `/clock` 的实收帧数（≥20 帧且仿真时间在前进），不是发布者数、不是进程存活。

为什么必须有这一步：Gazebo 有一种**间歇性加载期死锁**，实测特征三条同时成立——

1. `/clock` 发布者数 = 1（桥接活着），但 6 秒 **0 帧**；
2. `ign gazebo server` 只烧 ~2 ticks/s，而 GUI 烧 ~143 ticks/s（渲染在转，物理没动）；
3. `stack.log` 里 `gz_ros2_control` 停在 `connected to service!! robot_state_publisher asking for robot_description` 之后再无输出。

实测这时等 240 秒一帧都没有，而一小时前同一份代码是好的 —— 它是**启动竞态**。`headless:=false` 也会中招（旧记录里"headless:=true 是触发条件"不完整）。对启动竞态，正确处置是**清栈重启重试**（脚本最多 3 次），不是继续等，更不是把人卡在启动界面上。

死锁现场会先 `cp` 成 `stack_deadlock_attemptN.log` 再清栈，否则下一轮日志会把现场覆盖掉。

### [3/5] 立刻开录 —— 必须早于第一个目标派发

三个进程同时起：`bag` / `bag_actions` / `explore_metrics_recorder_node`。

**排在步进自检之后**：死锁下 `/clock` 一帧不发，录下来的 bag 是空的、录制器 `use_sim_time` 的时钟永远停在 0，纯属白录。

**排在感知链检查之前**：链条走通要 90~120s（等 SLAM 插入扫描出 `/map`），而探索的第一个目标就在出图后不久派发，录制器排在链检查后面必然赶不上第 0 轮。提前开录不会脏数据：没有 `/map` 就没有前沿、就不会派发目标，录制器那段时间只是空转（它是只读的，不发任何话题）。

三个具体坑：

- **`/clock` 必须录进 bag**。不录它，回放时所有 `use_sim_time` 的一侧全部冻在 0。
- **action 话题是隐藏话题**，显式点名也不够，`ros2 bag record` 会直接跳过，只打一条
  `[WARN] [ROSBAG2_TRANSPORT]: Hidden topics are not recorded. Enable them with --include-hidden-topics`。
  "派发次数 / 失败码 / 每目标耗时"全靠它们，所以单开一个带 `--include-hidden-topics` 的 bag。不把这个开关加到主 bag 上，是因为它会连 `/parameter_events` 一起吸进来白白放大体积。
- **录制器参数名是 `output_dir` / `run_label`**。写成 `out_dir` / `run_tag` 会被**静默忽略、零告警**，数据落到默认目录且标签是空串。
- 用白名单而不是 `-a`：`-a` 会把四路原始点云（每帧两万点）一起录进来，体积和回放开销都不划算。

### [4/5] 感知链检查 —— 只观察，不阻塞，不删数据

`tools/gate_chain.py` 逐跳数 11 个话题的实收帧数：

```
/clock -> /odom -> /livox/lidar_left|right -> /livox/left|right/cloud_filtered
       -> /livox/fused_points -> /livox/cloud_self_filtered
       -> /scan_from_cloud -> /scan -> /map        （外加 TF 里有没有 map frame）
```

判据只能是**实收帧数**。三个看似等价的判据都骗过人：

- **进程存活**：`livox_fusion_node` 曾经 DDS 端点全无却活得好好的（30s 只烧 1 个 CPU tick），链条从 `/livox/fused_points` 起就是 0 帧，栈空转 900 秒没有任何一处报错；
- **发布者数 / `ros2 topic list`**：只有订阅端点的话题也会出现在 list 里，`/livox/fused_points` 有 SUB 无 PUB 照样在列；
- **`ros2 topic hz --qos-reliability best_effort`**：对实测 47Hz 的 `/odom` 报 0。

这一步**曾经是阻塞式硬门禁**（不通就等满 240s 再把 bag 和 metrics 一起删掉），已改掉，原因两条都是实测：

- 它挡不住真正的启动故障 —— 加载期死锁已由第 2 步处理，走到这一步仿真必然在走，剩下的差异只是 SLAM 出图早晚；
- 它把已经录到的数据删了。`/map` 出得晚不代表这段录制没用。

所以现在只观察一次、如实打印逐跳表，不影响后续流程。若最终 0 次派发，就以这份表为排查起点。

### [5/5] 录制状态复核

不看进程存活（这条判据骗过人）：bag 看**字节在长**，action bag 看有没有那条 `Hidden topics` 警告，录制器看输出目录里文件出来了没有。

---

## 判"探索到底怎么结束的"

**必须看协调器日志，不能看"机器人不动了"**。三种结局在外部表现上一样：

| 结局 | 日志特征 |
|---|---|
| 真完成 | 协调器打完成/COMPLETED |
| 卡死（PAUSED） | 刷 `自动恢复已达上限 3 次，停止重试，等待人工调整`；往前找原因行，典型是 `原始前沿格 185, 有效前沿块 0, 候选 0 个` + `全部 N 个前沿块被否，原因分布: 被障碍物包围/不可达` |
| 假完成 | 冷启动全未知地图时 `mapReady` 只查消息层就判 COMPLETED |

```bash
grep -E "exploration_coordinator" /tmp/explore_run12/stack.log \
  | sed 's/\x1b\[[0-9;]*m//g' | grep -vE "自动恢复已达上限" | tail -20
```

---

## 读报告：哪些数能当结论，哪些不能

报告第 ① 节是**数据可用性自检**，不通过的列后面的数一律不作为结论。当前最重要的一条是 `位姿覆盖率`。

### 位姿覆盖率低的那些轮，跟踪类指标是假的

run11 实测 18 轮里 9 轮覆盖率 < 0.5。**不是采样坏了** —— 逐通道算下来每个通道拍率都正常（pose 21.8Hz、odom 52Hz、scan 12Hz），但整轮样本跨度只有 0.3~0.55 秒。

定位这类问题的两条命令（缺一条就会得出错结论）：

1. **逐通道**算样本数与跨度。绝不能跨通道混算 —— 混算过一次，得出"跨度 -0.03s""973 个样本挤在同一时刻"两个假结论（首行是 pose 的第一个样本，末行是 plan 的唯一一条）。
2. 把样本时间**换算成它在轮次窗口里的百分位**。run11 坏轮读数是 **107%~146%**：样本整个落在窗口**关闭之后**（轮 0 窗口 4.08~12.28s，样本却在 14.95~15.50s）。

结论：逐轮样本落盘不是关窗时的快照，写的是之后被下一轮重新填过的活缓冲区。受影响的列 = 一切从样本缓冲算出来的：`实走里程`（恒 0.000 就是特征）、`横向偏差`、`最小净空`、`窄段*`、`居中偏差`。

**不受影响**：`到位误差`（关窗时现查 TF，是独立测量）、`到位成功率`、`时长`。所以"成功率 0.889 / 到位误差 med 0.121m"对全部 18 轮成立。

注意这**不是**"录制器挂晚了"。历史上有过那个问题（订阅 `/exploration/current_goal` 收到**锁存的历史目标**，给早已开始的目标开窗，整体错位一格），但 run11 已排除：bag 里 `/exploration/current_goal` 正好 18 条 = 18 轮，没有重复开窗。

### 代理量不是真值

名字带 `proxy` 的都不是真值，报告第 ⑤ 节带定义。两个最容易被当真的：

- `geometric_intrusion_episodes_proxy` **不是碰撞次数**（本机没有碰撞传感器）。用内切半径是保守口径，因为最近回波的**方位角没有记录**，判不出它落在正方形哪条边上。
- `zero_progress_events_proxy` **不叫"误判次数"**：`follow_path` 模式没有 BT，nav2 不发布 trip 原因，真伪无法判定。

### 样本量

轮数 < 5 时不要下"策略有效/无效"的结论。本项目已两次在 n=4~5 时宣布结论、随后被自己的数据否证。

---

## 收尾

```bash
# 先停录制让 bag 封口（栈可以留着看 rviz）
for f in bag.pid bag_actions.pid recorder.pid; do
  kill -INT "$(cat /tmp/explore_run12/$f)"; done
sleep 8 && du -sh /tmp/explore_run12/bag     # metadata.yaml 出来了才算封口

bash tools/clean_sim_stack.sh                 # 再整体清栈
```

**每轮跑完都要查一次陈旧 bag 进程**。实测有两个 `ros2 bag record` 活了 108 分钟，一直往一个已经 `mv` 走的目录里写（文件句柄跟着 inode 走，数据静静流进归档目录）：

```bash
ps -eo pid,etimes,args --no-headers | awk '!/awk/ && /bag record/'
```

---

## 查进程状态时的两条铁律

1. **不要用 `pkill -f` / `pgrep -f` 配长模式**。命令行里含有那个模式时会匹配到自己：实测 `pgrep -c -f run_explore_with_bag` 报 2 而真实进程 0 个，一个 `kill` 循环杀掉了自己的 shell（退出码 144）。用
   `ps -eo pid,args --no-headers | awk '!/awk/ && /pattern/'`。
2. **CPU tick 增量是不经过 DDS 的独立量**，探针自己连不上时用它交叉验证：
   `awk '{print $14+$15}' /proc/<pid>/stat` 取两次差。实测判据：正常处理节点 ~180 ticks/s，空转 spin ~50 ticks/s，**完全没干活 1 tick/30s**。

## 查询 shell 的环境变量

`ROS_DOMAIN_ID=25` + `ROS_LOCALHOST_ONLY=1`（脚本里设的就是这两个）。查询 shell 不带，症状是"节点全都 Node not found"，看着像整个系统没起来。
