# 自主包退役后的仿真回归

日期：2026-09-17。此记录补充[包退役验证](AUTONOMY_PACKAGE_RETIREMENT_20260917.md)，采用删除 `astribot_s1_autonomy` 后的当前源码。没有修改控制算法、控制参数或机器人设备状态。

证据根目录：[/home/yjh/WorkSpace/astribot_validation/autonomy_sim_regression_20260917_204620](/home/yjh/WorkSpace/astribot_validation/autonomy_sim_regression_20260917_204620)。包含源码摘要、构建日志、实际参数、会话清单、路线原始样本、逐目标结果和专项观测脚本；临时脚本不安装到产品包。

## 1. 构建和运行条件

- 确认没有其他运行栈后构建依赖闭包，16 个包成功；CMake 提示部分通用命令行变量未被相应包使用，没有构建失败。
- 用主管启动 Gazebo 和 RViz；运行环境从实际进程反读：域 25、localhost、导航 UDP profile。
- 基线：warehouse_baseline 固定地图、ground_truth 定位、MPPI、x/y 上限各 0.35 m/s、策略 off、slice_scan、simulation_precision。
- 实际控制器为 ArrivalController，`arrival.source=nav2_pose`，refine_timeout=135 s；到位阈值 2 mm / 0.1°，两张 costmap 的扫描时效 0.3 s。
- Gazebo transport 有物理步进，检查时 iterations=54244、RTF=1；启动探针实收扫描、里程计和时钟，TF 龄期约 18 ms，7 个导航生命周期 active。GUI 窗口与 RViz OpenGL/地图加载日志单独保存。
- 当前 ament 索引没有旧包；扫描由新感知组件包提供。检查窗口内 `/cmd_vel` 只有预期耦合节点的发布源。

## 2. 路线结果

路线使用现有 `tools/run_waypoint_route.py`，每条路线一轮。脚本先规划预检；完成后再等待 3 s，并复查实际位姿和停稳速度。以下位置/航向精度来自仿真真值定位链，不代表真机精度。

| 指标 | 五点短路线 | 六点完整路线 |
|---|---:|---:|
| 目标通过 | 5/5 | 6/6 |
| 最大到位欧氏误差 | 1.798 mm | 1.719 mm |
| 最大到位角度误差 | 0.081° | 0.059° |
| FOLLOW 横向 RMS / P95 / max | 1.756 / 3.077 / 3.252 cm | 3.341 / 6.641 / 11.499 cm |
| 行进段航向 RMS / P95 / max | 2.285 / 5.583 / 6.151° | 2.695 / 4.157 / 21.399° |
| 直线行进段横向 RMS / P95 | 1.656 / 2.991 cm | 3.013 / 5.544 cm |
| 直线行进段航向 RMS / P95 | 2.285 / 5.583° | 1.475 / 3.036° |
| 实测线加速度 RMS / max | 0.0722 / 0.1741 m/s² | 0.0404 / 0.1933 m/s² |
| 实测线 jerk RMS / max | 0.1562 / 1.5333 m/s³ | 0.0756 / 1.3844 m/s³ |
| runner 判定的 FOLLOW 异常原地旋转 | 0 s | 0 s |
| 无效采样 / runner 告警 | 0 / 0 | 0 / 0 |
| 每目标路径版本数 | 均为 1 | 均为 1 |

行进段按 runner 的终端航向半径排除终点附近样本；本轮半径 0.5 m。直线另要求参考曲率绝对值小于 0.1。加速度/jerk 使用里程计源时间、去重并逐目标分段，不能用轮询时间求导的另一组值混算。完整原始值见 `route_summary.json`、两条路线的 `results.jsonl` 和 `samples.csv`。

结论是**该版本的两条路线功能与到位判据通过**。单轮无告警不是所有过程指标优秀的证明：完整路线最大横向误差约 11.5 cm，不能据此放行 85 cm 通道。没有删包前同版本的交替重复对照，也没有统计意义上的性能完全不退化结论。耗时只作为实验预算，未计入质量评分。

## 3. 迁移后的组合入口

在上述真实 Gazebo 数据/TF 环境中，分别启动新包的组合入口：组件模式和独立进程模式均正常启动、发布前沿状态/Marker，并以退出码 0 正常结束。组件容器实际列出 `/frontier_explorer_node`，组件模式同时加载迁移后的 RViz 配置。

本项设置 `enable_perception:=false`，复用基线中已运行的新感知节点；没有重复启动扫描节点。前沿输出改为 `/regression/frontier_suggestion`，检查到该节点没有 Twist 发布源。这证明实际资源和前沿可视化入口可用，不是自主探索成功证明。

固定地图下未发布候选目标，超过 10 s 后节点按地图更新时效进入 WAITING_MAP；这项检查不以“有 Marker”推导探索完成。组件/独立模式各自完整双节点装载的隔离验证见上一份退役报告。

## 4. 自主探索

采用独立会话切换到 `--mode explore`，不传 map-yaml，使用 SLAM 建图定位及 standard 到位档位。**自主探索功能本轮未通过**，不能与 11 个基线导航目标合并成全部通过。

1. 首次导航启动的 controller configure 服务响应超时：控制器 inactive，其余主要节点 unconfigured；同时物理时钟、扫描和 TF 正常。主管按既有策略在 120 s 探针失败后只重启本次导航子栈，第二次 7 个节点 active、TF 龄期约 6 ms。记录了恢复成功，但没有证明首次超时的根因已经修复。
2. 导航就绪后，前沿搜索报告原始前沿 972 格、可达前沿 664 格，每轮 8 个采样候选全部被拒绝。原因是“前沿内退范围内无完整足迹已知空闲的可达观测位姿”。4/4 连续采样失败后暂停；3/3 自动恢复耗尽后仍无合法候选。
3. `dispatched=0`、`succeeded=0`、`goal_in_flight=0`，`/exploration/complete=false`。这不是“环境探索完成”，也不是本轮发现跟踪器无法执行已提交路径；故障发生在目标派发之前。
4. 另外调用人工 pause 服务成功，收到 `manual_pause=1`；实收里程计为静止。未进入活动导航取消或恢复后成功场景，这两个测试未完成。观测窗口未收到 `/cmd_vel` 消息，不能把缺消息算成零速度指令证据。

保留 `exploration_stack/session.log`、`session.json`、`exploration_states.json`、`exploration_samples.json`、`exploration_inputs.json.gz`（实际 SLAM 图、costmap、TF 等）、`exploration_manual_pause.json`。`exploration_status.json` 是失败汇总；观测脚本被有意中止，默认 ROS 信号处理导致其清理调用失败，随后由独立客户端完成上述人工暂停。`exploration_parameters.yaml` 的 CLI 查询未成功，不作为运行参数证据。

后续应对本轮实际地图做候选生成/回退/完整足迹验证的离线回放，区分有限采样遗漏、回退搜索连通性、地图稀疏未知格和观测可见性条件。当前日志不足以认定只能通过降低安全冗余解决，也没有证据把该问题归因于删除兼容包。本次没有为了通过回归修改采样或碰撞参数。

## 5. 正常关停与复现

两次仿真均通过各自主管正常停止，关停后检查会话清单和进程残留。主要日志分别为证据根目录下的 `baseline_stack/session.log` 与 `exploration_stack/session.log`。`latest_sim` 是索引，不能用它推断仿真仍在运行。

## 6. 能力边界

本次回归不覆盖 RPP、P2–P5、动态障碍故障注入、完整窄通道矩阵、SLAM 完整环境覆盖、双臂携物、视觉/mark 精调或真机。架构审查提出的新整机任务/载荷/资源模块也没有因本次导航回归而被实现或放行。

操作入口见[仿真手册](manuals/SIMULATION_OPERATIONS.md)、[真机手册](manuals/HARDWARE_OPERATIONS.md)，整体改进建议见[整机架构审查](WHOLE_ROBOT_ARCHITECTURE_REVIEW_20260917.md)。
