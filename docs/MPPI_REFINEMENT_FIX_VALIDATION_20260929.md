# MPPI 终点精调修复与复测（2026-09-29）

## 原因与最小修复

上次 `REFINEMENT_NO_PROGRESS` 的直接原因是 MPPI 配置选择了 `arrival.source: nav2_pose`，实际传入 Arrival 的位姿时间戳始终为 0。`ArrivalSettling::observe` 只接受时间戳严格递增的观测，故零命令后的窗口只有首个样本，`stop_span_xy_s`、`stop_span_yaw_s` 始终为 0，`stopped` 始终为 false。`ArrivalCoast` 等待该停稳判据才退出制动等待，导致精调锁在 COAST；剩余约 2.15° 的朝向误差没有继续纠正，15 s 无进展期限最终触发中止。

上次日志在 sim 677.250～690.717 s 持续记录 `phase=COAST source=nav2_pose pose_s=0.000 xy_held=1 yaw_held=0 stop_xy=0 stop_yaw=0`，最终在约 sim 691 s 报错。这次失败没有分层碰撞拒绝错误，不能归因于新 critic。

仅将 MPPI YAML 的 `arrival.source` 改为已有的 `slam_pose`，使用项目规定的 `/slam/pose`。不改停稳判据、超时、2 mm / 0.1° 验收精度或底盘参数。独立分支 `codex/fixed-posture-whole-body`，未合并 `chassis-effort-drive`。

用上次记录的 99 个真实停稳样本输入现有 C++ ArrivalSettling / ArrivalCoast：

| 时间戳输入 | 最终窗口 | stopped | COAST |
|---|---:|---|---|
| 复现原 nav2_pose 的恒定 0 | 0 s | false | 保持 |
| 样本原始 SLAM 时间戳 | 0.633397 s | true | 退出 |

这是离线因果复现，不替代下述实际仿真运动。

## 修复后的实际运动

自有会话 `mppi_slam_refine_20260929`，domain 95，独占仿真租约。使用原有仓库世界和 Gazebo 分层地图，未添加障碍物；不是相机生成的分层地图。运行参数读回 `source=slam_pose`，底盘 `idle_position_hold=true`、`idle_position_kp=3.0`；controller_server PID 1491883 实际加载本分支 critic。真实空载观察、双臂 HoldResources 和五消费者确认均完成后执行。

| 阶段 | NavigateToPose | SLAM 终态位置误差 | SLAM 终态角度误差 | 停稳窗口 |
|---|---|---:|---:|---:|
| 向前 0.8 m | SUCCEEDED (4) | 1.329 mm | 0.001106° | 0.625 s，7 样本 |
| 返回起点并转向 90° | SUCCEEDED (4) | 0.983 mm | 0.062714° | 0.629 s，7 样本 |

结束后另一次 SLAM 停稳窗口通过，HoldResources 返回资源已释放。自有仿真关闭后 `remaining_owned_pids=[]`。

活动阶段 575 个实际 `/slam/pose` 样本及相邻扫掠，以同一分层几何核回放，碰撞数 0。14 个双臂关节反馈最大范围约 1.17e-12 rad，仅代表这次仿真固定姿态观测，不代表真机精度。

从关闭的 rosbag 中取原始地图、包络、位姿和时钟，真实插件回放仍得到空闲候选代价 0、指向现有障碍的诊断候选代价 infinity。该诊断候选没有发给机器人，也不是实际 MPPI 批次；不能据此宣称真实遇障改道通过。

## 证据

目录 `/home/yjh/WorkSpace/astribot_whole_body_nav/runs/mppi_slam_refine_20260929/`：

- `navigation/result.json`：两段成功结果、SLAM 终态误差和资源释放。
- `reproduce_settling.cpp`、`old_stopped_samples.txt`、`reproduce_settling.log`：原故障复现。
- `stack/session.log`：`ARRIVAL_REACHED source=slam_pose` 与递增的 `pose_s`；`session.json`：归属及关闭状态。
- `motion_analysis.json`、`replay_motion_result.log`：双臂反馈、575 次实际扫掠和诊断候选评分。
- `motion_bag/metadata.yaml`：关闭后核对，共 267015 条消息；包括 1814 条 SLAM、387 条分层地图、282 条 MPPI 可视化。
- `navigation_screen.mp4`：149 s 原始录屏；`mppi_return_90.mp4`：其中第 20～100 s 的原速摘录，包含前行、返回及转向。旧 RViz 配置中的 selected trajectory 项未收到独立话题，不能作为最优轨迹证据。

## 现有货架附近验证

新增每秒最多一条非零拒绝数量日志 `WHOLE_BODY_COLLISION rejected=... batch=... all_blocked=...`，只增加可观测性，未改变评分或控制逻辑。修改后 `whole_body_collision_critic` 测试 1/1 通过。

第一次独立会话 `mppi_shelf_20260929` 选择目标 (-1.5, 4.5)，全局规划器报目标占据，在开始运动前中止。保留该失败和代价地图读数（目标代价 99），不把它记为 MPPI 碰撞拦截。随后根据当前全局代价地图选择货架旁的 (0, 2.5)，该点代价 46；未改世界、地图、代价或碰撞参数。

正式会话 `mppi_shelf_edge_20260929` 使用 domain 97、独占租约及相同固定双臂输入。控制器、双臂保持和五消费者准入重新核实后，完成以下实际运动：

| 阶段 | 结果 | SLAM 终态位置误差 | SLAM 终态角度误差 |
|---|---|---:|---:|
| 货架旁目标 (0, 2.5) | SUCCEEDED (4) | 0.569 mm | 0.051198° |
| 返回起点并转向 90° | SUCCEEDED (4) | 1.302 mm | 0.005377° |

实际 controller_server 日志记录 10 个发生分层碰撞拒绝的批次，各批 2000 条候选，被拒绝数量依次为 2、29、15、10、10、218、216、205、2、1，均 `all_blocked=0`。这些是在线评分中的实际候选，合计记录 708 条被赋无限代价、权重为零的候选。由于日志限频，此合计仅覆盖记录到的批次，不是整段总拒绝数，也不是独立障碍物数量。

真实运行证明新 critic 在行走中筛除碰撞候选，同时保留可行候选完成目标。没有禁用 critic 的 A/B 对照，不能把具体绕行形状全部归因于新 critic；全局规划和其他 critic 同时在工作。

活动阶段 1006 个 SLAM 位姿及相邻扫掠在记录的同版本分层地图中回放，碰撞数 0。14 个双臂关节反馈最大范围约 1.13e-12 rad。结束后额外 SLAM 停稳窗口 0.614 s，通过；HoldResources 已释放，自有仿真和补充进程已关闭。几何回放使用被测系统同一碰撞核，不是独立接触/力传感器证明。

证据目录 `/home/yjh/WorkSpace/astribot_whole_body_nav/runs/mppi_shelf_edge_20260929/`：

- `navigation/result.json`、`parameter_readback.json`、`loaded_critic.txt`：成功结果、实际参数和加载库。
- `online_rejections.log`：10 条原始在线拒绝日志；完整上下文在 `stack/session.log`。
- `motion_analysis.json`、`replay_motion_result.log`：活动阶段反馈、分层扫掠检查及隔离候选回放。
- `motion_bag/metadata.yaml`：1472 条 SLAM、308 条分层地图、523 条 MPPI 可视化消息。录制正常关闭后才读取 SQLite。
- `navigation_screen.mp4`：完整原始录屏；`mppi_layered_collision_verified.mp4`：第 12～132 s 原速摘录，包含货架旁往返及 90° 转向。此轮 RViz 已显示实际 `/trajectories` MarkerArray。
- `stack/session.json`：最终 stopped，`remaining_owned_pids=[]`。

## 验收边界

本轮已闭合：原精调失败的原因与修复、前行/返回/90° 转向、现有货架附近实际 MPPI 分层候选拒绝、终态停稳与保持资源释放。

仍未闭合：底盘可过但仅双臂高度碰撞的专门运动场景、碰撞接触/力证据、动态障碍、相机到分层地图的在线链路、真机。当前证据是现有 Gazebo 几何分层地图下的仿真结果，不提升为整体验收通过；仍留在独立分支，未合并开发主线。
