# 事件重规划与持续跑机

本次修改保留 ThreePhaseController 的控制公式、参数及终端精调要求：欧氏位置误差不超过 0.03 m，角度误差不超过 1.5°。新加入的控制器阶段话题只用于观测。

## 启动和运行

在仓库根目录执行：

```bash
bash tools/launch_sim_stack.sh --mode baseline --max-linear-speed 0.35 --log-dir /tmp/astribot_session
source /tmp/astribot_session/env.sh
python3 tools/run_waypoint_route.py --duration 7200 --output /tmp/astribot_endurance
```

第一条命令保持前台运行；第二、三条在另一个终端执行。默认显示 Gazebo 和 RViz。`--headless` 只关闭 Gazebo GUI，`--no-rviz` 关闭 RViz。`--dry-run` 打印启动命令，跑机程序的同名选项则调用规划器检查路线、不发运动目标。

启动器兼容 mapping、explore、localize 参数，并增加 baseline 模式（仓库地图、仿真真值定位）。`--map` 是 SLAM 序列化地图基础路径，`--map-yaml` 是栅格地图 YAML。`--tracker mppi|rpp`、`--scan-source slice_scan|laserscan`、`--max-linear-speed` 均保留。

启动分为 Gazebo 物理步进、时钟/TF/扫描/里程计就绪、Nav2 七个生命周期节点 active 三步。探测进程有独立的外部墙钟超时，不以进程存在或话题发布者数量判定成功。日志、启动参数、PID、实际采样数保存在 session.json 和独立日志文件中。

Nav2 默认单独使用 UDP 配置；Gazebo/感知仍使用调用环境。`--nav-transport default` 可恢复原有通信配置。扫描源会同时传给感知和导航，并读取两个代价地图的实际订阅参数进行核验。启动失败最多尝试两次导航激活（`--nav-attempts 1..3`），只重启本次启动的导航进程组；超时或退出时清理本次创建的进程。该策略用于规避曾观察到的 DDS 生命周期响应超时，**不是根因修复，也不保证每次启动成功**。不会自动杀掉其他会话或全局删除共享内存。

## 规划策略

行为树取消 RateController 定时规划，也取消旋转、倒退和清空地图后盲目重试的恢复链。正常情况下，一个目标对应一次规划。每 0.2 秒检查的是当前路径的碰撞风险，不是重新计算全局路径。

KeepSafePath 只在新目标或明确的碰撞索引出现时进入规划。数据不新鲜、TF 缺失或服务超时均与碰撞区分；持续 2 秒无法获得有效检查结果时终止导航。每个目标最多允许 5 次碰撞重规划，避免在不可达目标上无限循环。规划分支运行时，ReactiveSequence 停止旧 FollowPath。多目标模式独立维护剩余目标列表并持续移除已通过点，更新剩余点本身不会触发重规划。多段拼接处的急弯同样由前视降速覆盖。

全局路径检查使用机器人轮廓，沿线按不大于半个栅格的步长插值，旋转按不大于 0.05 rad 插值；轮廓边界检查致命/未知栅格，中心检查内切膨胀区。已经通过的路径段不参与检查。最近点选择适用于当前 Smac 非自交路径；复杂自交或回环路径需要另行验证。

候选路径按 0.1 m 空间间隔计算离散曲率和曲率变化率，默认上限为 3 m⁻¹、12 m⁻²。不超限时保留原路径；超限时进行端点固定、相对原路径位移不超过 0.2 m 的迭代平滑，再恢复与原路径一致的点数，保留 MPPI 按点数设置的前视参数的空间尺度，然后复查曲率和碰撞。最多 200 次迭代。若平滑结果不可用，但原路径通过轮廓碰撞检查，则保留原路径，ArrivalController 在前方 1 m 窗口内检测急弯并限制速度（不超过 0.15 m/s，同时按曲率和变化率进一步约束），同比例缩放平移和转动指令；正常窗口不改变指令。降速判别会先过滤不超过 3 cm 的栅格端点拼接噪声，过滤只作用于指标，不改写实际路径。原路径也不安全或地图数据不可用时显式返回 PATH_QUALITY_UNSAFE。不会放宽到点精度或忽略碰撞。

这是一种受位移约束的离散平滑，不是严格的 C² 连续轨迹生成器。思路参考覆盖作业对曲率连续性的要求：[Fields2Cover 路径规划文档](https://fields2cover.github.io/source/tutorials/path_planning.html)。事件触发策略与 [Nav2 默认周期重规划行为树](https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html) 有意不同。

## 跑机数据与判据

跟踪总时长不参与性能评价，`elapsed_s` 仅作运行记录。允许通过降低速度改善质量；对照试验优先比较同一路段的航向偏差、横向误差及加速度/角加速度/jerk，并单独检查直线、弯道、弯后和到点接近段。到点 3 cm/1.5°、碰撞检查和异常退出要求保持不变。运行期限与单目标超时用于管理实验和防止无限等待，不是速度评分；较慢方案应预留足够的验证期限。航向收敛与异常持续时间只用于定位问题，不以更快完成整段作为优化依据。

默认六点路线来自 `/home/yjh/Downloads/run_waypoint_route.py`，yaw 单位为弧度。程序首先检查各段可规划性和循环接缝，再串行发送 NavigateToPose 目标。`--short-route --cycles 1` 使用前进、侧移、斜行、原地调角、返回五个短程目标。`--route route.json` 可提供 `[[x,y,yaw], ...]`。

每个目标必须同时满足 action 成功、稳定后的实测位置/角度限差、平移和角速度均不超过 0.01 的停止条件。检查器参数必须不宽于 3 cm/1.5°。读取 map→astribot_torso_base 的新鲜 TF 和 odom；数据持续缺失时取消本次目标。程序仅支持 use_sim_time=true 的仿真，不自动切换至真机。到达运行期限或 Ctrl+C 时取消本次尚未结束的目标并等待结果，不继续发下一点。

- `samples.csv`：约 20 Hz 的循环/目标编号、目标距离、位置、航向、实际速度、控制指令、控制阶段、路径版本、横向偏差、参考方向偏差和沿路径进度。
- `results.jsonl`：每目标的到点误差，FOLLOW 阶段及直线段的横向偏差 RMS/P95/最大值，正面方向误差，低速旋转时长、超过 30° 朝向误差时长、回退距离、振荡计数、加速度、角加速度、jerk、实测轨迹曲率及变化率。
- `heading_convergence`：行进 FOLLOW 首次连续 0.5 s 保持在 5° 内的收敛时间，以及超过 15° 的最长连续时长；排除终点朝向区，数据断档或路径切换不跨段累计，未收敛记 null。
- `plans.jsonl`：带目标编号和版本的原始规划路径；`metadata.json` 和 `runner_snapshot.py` 保存参数与运行脚本快照。
- `status.json`：每 5 秒原子更新进度、最近结果、PID 和截止时间。状态 failed/stopped/completed/duration_complete 必须区别处理。

ALIGN_START、ALIGN_GOAL 和 REFINE 的必要旋转不计为 FOLLOW 异常旋转。额外记录去除 GoalAngleCritic 末端预对齐区域后的行进航向指标；大航向告警依据该行进指标，原始 FOLLOW 指标仍保留。纯原地调角没有 FOLLOW 样本时相关指标为 null，而非零误差。横向 P95 超过 10 cm、持续大航向偏差、FOLLOW 旋转/停滞和数据缺失都会写出 warnings；它们是过程评估信号，不能由 action 成功掩盖，也不能用到点精度替代。过程告警阈值是初始观察阈值，不代表全工况合格标准。

`path_tracking/replan_event` 与 `path_tracking/path_quality` 提供重规划原因和曲率处理结果。`/plan` 消息数可能包含手动规划请求，评估重规划触发次数应结合事件和规划日志，不能只数话题发布者或消息。

## 验证记录

持续运行结果和本次实际通过范围见同目录 `PATH_TRACKING_ENDURANCE_VALIDATION.md`。长时间运行尚未完成时只能报告阶段结果，不应宣称两小时耐久已通过。单轮随机 MPPI 结果不能证明所有场景性能单调改善。

起步对齐触发与退出判据的修复、航向强化候选对照实验见 [航向收敛验证](HEADING_CONVERGENCE_VALIDATION.md)。
