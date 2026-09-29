# 独立工位对齐最小实现与验证

2026-09-28。已按确认方案施工，代码构建及隔离测试通过；完整搬运运动闭环尚未验收。本记录只涵盖工位受阻交接、独立 SLAM 对齐及其必需的坐标配准。

## 实际数据链路

```text
搬运固定工位目标 → navigate_to_workstation.xml
  → 正常规划、三段跟踪
  → OBSTRUCTION_DEADLINE 且 SLAM 距原目标 ≤ 0.50 m
  → 结束旧 FollowPath，确认动作终态和 SLAM 停稳
  → 最新分层地图 + 当前整机/载荷包络 + SLAM 实际位置
  → 共享控制数学预测整段对齐及制停扫掠
  → 原导航会话提交 WORKSTATION_ALIGN
  → controller_server 选择独立 WorkstationAlignmentController
  → /slam/pose 闭环生成机体 vx / vy / wz
  → 到位与有符号净变化停稳 → FINISH 确认 → 原搬运事务后续阶段
```

不满足距离、指定失败原因、空间可达或输入条件时，不提交工位对齐。工位实体参与碰撞；未知空间不当成空闲。执行期间使用更新后的分层信息检查短时运动与制停扫掠；取消和明确停车仍有效。一次原始目标只提交一次对齐，不增加近场绕行、目标替换或新任务服务。

## 实施落点与边界

- `astribot_s1_path_tracking/src/workstation_approach_bt.cpp`：规划层交接。旧动作终态确认后才启动新控制器；评估、提交、执行、完成沿用同一 session、reference_path 和目标。原策略上下文的 0.5 秒保活机制没有放宽。
- `workstation_alignment_core.cpp`：预测与执行共用速度、加速度、加加速度约束及机体速度积分。检查平移与转动的联合扫掠，使用现有分层碰撞核心。
- `workstation_alignment_controller.cpp`：直接实现 Nav2 Controller，独立于 Arrival/ThreePhase 状态机；控制、误差与停稳只用 `/slam/pose`。实际插件测试故意给错误的 Nav2 pose/twist，确认未被用作对齐反馈。
- `ResolveRoute.srv`、`MotionConstraint.msg` 和现有策略模块：明确 NORMAL / WORKSTATION_ALIGN / FINISH 及完成确认。仅退役原路径的失败锁存和二维扫掠约束；输入覆盖、相机状态、禁行区、包络、上肢限速及明确 HOLD 保留。没有底盘末级速度保护或新增速度发布者。
- `navigate_to_workstation.xml`：固定工位任务专用 BT。`fixed_station_navigation.cpp` 默认从已安装导航包选择该文件，普通导航默认树不变。`workstation_navigation_bt` 可指定部署路径；空路径不允许发起工位任务。
- `navigation.launch.py`：工位 BT 和独立控制器读取同一份解析后的运动/到位参数。MPPI/RPP 与 standard/simulation_precision 四种组合均核对 18 项显式参数一致；没有新增第二套调参文件。
- `voxelslam.cpp` 与 `initial_map_pose.hpp`：在估计器源头使用实测 `General.initial_chassis_pose=[x,y,z,qx,qy,qz,qw]`，同步统一状态、点云和底盘位姿的地图原点；重力初始化保留测得底盘原点与航向。该初值不能与 `previous_map` 同时使用。static_map 配合启动 SLAM 时要求提供明确初值；新建局部地图的空初值语义不变。

普通搜索、MPPI、三段跟踪算法和 `idle_position_hold=true`、`idle_position_kp=3.0` 没有改动。现有 Nav2 通用进度检查器仍保留自身的正常输入；没有把本插件的 SLAM 证据扩大为整个 Nav2 仅使用 SLAM。

## 验证结果

证据根目录：`runs/mainline_20260928/workstation_alignment/`。测试使用私有安装和隔离 ROS 域；未覆盖原仿真安装。

| 层级 | 结果 | 证据 |
|---|---|---|
| 构建 | 消息、路径跟踪、策略 C++、SLAM、搬运 native 构建通过；导航、感知及既有 Python 策略安装通过 | 各包 `*.build.log`、`build_*.json`、`navigation_final_install.log` |
| C++ 核心、实际控制器、实际 BT | 最终代码 3/3 CTest 通过；覆盖 0.49/0.50/0.51 m、分层障碍/未知、速度和制停约束、SLAM 反馈、交接和取消 | `workstation_cpp_tests_frozen.log` |
| 既有关键接口回归 | envelope BT、policy execution、velocity restriction 3/3 通过 | `workstation_cpp_tests_final.log`；其中新模块三项随后因 HOLD 修正重跑 |
| 策略适配与约束 | Python 实际适配层 7/7，实际 C++ ROS 约束节点 2/2 通过 | `route_focused_tests.log`、`native_constraint_workstation_tests.log` |
| 工位搬运接口 | fixed_station_navigation 19/19 测试通过，含同一租约/目标、SLAM 到位、取消、默认选择已安装工位树 | `fixed_station_navigation_tests.log` 和对应 gtest XML |
| SLAM 数学及启动 | 2/2 CTest 通过，分别包含坐标初始化数学与启动参数校验 | `slam_initial_tests.log` |
| 启动接线 | 四种配置组合成功，18 项参数同源，安装文件与源码哈希相同 | `wiring_readonly.json` |
| 当前仿真真实输入、静止配准 | 独立 SLAM 观察 20 秒：121 帧 pose、110 帧 cloud，TRACKING；最大 XY 0.158 mm、Z 0.180 mm、yaw 0.00223° | `shadow_slam/observation_summary.json` |
| 当前场景空间评估 | 实际 5 层地图/包络、配准后的 SLAM 起点至原工位目标：251 步预测及制停扫掠 CLEAR；单次全段计算 8.386 ms，快照构造 4.475 ms，制停预测 0.046 ms | `live_assessment/result.json`；非运动或实时性能验收 |
| 完整运动 | scene68 抓取、附着及运输姿态后进入 NAVIGATE；普通规划首段拒绝，未进入工位精调与 PLACE；未通过 | `scene68_outcome.json`、case 下 `full_transfer/result.json` |

静止配准初值来自当前 Gazebo 实际底盘位姿，而非 spawn 指令。运行图确认全部新 SLAM 输出位于 `/workstation_validation`，原 `/slam/pose` 发布者未变；完成后已按进程身份停止测试节点，见 `shadow_slam/cleanup.json`。静止误差不是动态定位精度或硬件标定结论。

## 已确认的实际限制

1. 当前分层地图源是每约 500 ms 遍历更新的 Gazebo BOX/MESH 碰撞几何。它能反映该仿真中的实体变化，但不等于真实 RGB-D/LiDAR 自由空间覆盖，也不证明任意动态障碍已验收。
2. 初轮测试发现分别限制 vx、vy 可能让合速度越界，已修正为内接速度方框，原断言保留。对平移上限 0.06 m/s，新模块单轴最大约 0.0424 m/s，斜向合速度不超过 0.06 m/s；代价是单轴接近更慢。普通跟踪配置未改。
3. 当前 front67 已完成失败收尾，实际包络报告 `ARM_HOLD_UNCONFIRMED`，导航和运输姿态均未放行。读取它评估几何不能代表拥有执行许可；不合成租约、不复活旧任务。

## scene68 整栈实测及未放行点

本轮确实启动了原场景的完整搬运事务并录制 203.3 秒、2033 帧视频。启动后发现私有安装漏带原先已验证的 `_native_tf_buffer` 扩展，策略节点退出；补齐该不变依赖并仅恢复策略节点后，原 Gazebo 会话通过 ready，没有重启世界。修复记录在 `native_tf_deployment_repair.json`；私有构建脚本已显式启用该扩展及 PIC，该扩展随后独立重编并通过原有 12/12 测试，仿真收尾后安装到候选前缀；scene68 本次运行使用的是先补齐的原验证扩展，两个哈希在修复记录中分别保存。

正式 SLAM 的实际初值来自 175 帧落稳测量，随后 72 对同源时间样本的最大 XY 差为 1.274 mm、航向差 0.00698°；这是启动静止配准证据。底盘参数实际服务读回仍为 true / 3.0，见 case 下 `bootstrap_slam_readonly.json`、`idle_hold_readback.json`。

抓取和运输姿态完成后，起点模块返回 START_READY，普通 GridBased 随即在约 1.057 m 外返回 `PATH_QUALITY_UNSAFE ... segment=0`。这是首条普通路径的检查拒绝，不是已进入 50 cm 后的障碍等待超时；新工位模块没有接管，也没有进入 PLACE。当前日志尚未将该拒绝细定位到具体障碍或栅格，不把碰撞/未知/边界的合并原因写成已确定的单一原因。

父任务最终为 `RESOURCE_RECOVERY_REQUIRED:NAVIGATION_NOT_SUCCEEDED`。业务资源释放未确认，停稳确认超时，资源处置为 UNRESOLVED；这两个后续失败不能记成成功。已按本任务身份回收全部仿真进程，包括单独恢复的策略节点；仅进程收尾确认完成，不等价于业务释放或实测停稳通过。bag 正常封存，视频 ffprobe 及整段解码通过。

## 下一条验收链

先处理本次普通规划首段拒绝及失败收尾未确认，再使用本轮私有候选安装启动新的完整搬运事务，取得真实上肢保持及载荷状态；SLAM 必须使用与分层地图一致的实测初始位姿。依次验证普通跟踪受阻、≤0.50 m 交接、独立工位对齐、原任务进入 PLACE，并记录位姿、模式、分层版本和视频。中途真实碰撞/覆盖不足时保留原失败原因，不改目标、不缩小包络、不降低碰撞门槛。

运行代码优先 C++；既有 Python 策略只做原服务契约的必要接口配合，Python 新增内容限启动、验证与分析脚本。未扩展到 VLA、硬件、全局分层搜索或额外故障矩阵。
