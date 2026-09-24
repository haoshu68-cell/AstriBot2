# M3 导航约束节点迁移交付

本轮开始：2026-09-24 14:30 +08；单问题一小时检查点：15:30 +08。

## 当前状态

15:14 冻结的上肢百分比限速最终修订已由主任务构建、安装并通过 5/5 隔离 ROS 协议（domain 36，12.461 s）。包含 frame_id 检查和 steady 剩余 lease 上限。下述 4/4 PASS 对应补漏前历史修订，其源码已归档于 `m3_navigation_constraint_before_arm_limit.tar.gz`。最终哈希与验证状态见 `m3_arm_speed_limit_source_manifest.json`。

主协调任务已完成隔离构建：节点与测试程序通过编译，4 个 ROS 协议场景全部通过（domain 36，localhost only，8.468 s），4 个本节点静态检查通过。首次测试编译因 Humble 图查询接口位置差异失败，定点修正后重编通过；原失败日志保留。本任务未自行启动 ROS、Gazebo 或构建进程。此交付不构成闭环仿真或真机验收。

## 变更

- `src/final_protection_node.cpp` 迁为 `src/navigation_constraint_node.cpp`，类为 `NavigationConstraintNode`，节点为 `navigation_constraint`，可执行文件为 `navigation_constraint_cpp`，CMake 安装组件为 `navigation_constraint`。
- 发布 `/navigation_policy/constraint` 与 `/navigation_policy/constraint_state`；区域功能启用时保留 `/navigation_zones/applied`，consumer 为 `navigation_constraint`。
- 删除 Twist 发布、停机发布零速度、CommandRestriction 和整个 envelope ACK 发布；固定包络本体仍用于几何检查。节点退出不会写命令。
- 只读启动参数 `command_topic` 默认 `/cmd_vel_nav_body_raw`，用于底盘坐标控制意图预测。`/odom` 继续独立提供实测余速。不能直接改为可能处于世界坐标的 `/cmd_vel`。
- 命令过期时只将其从预测输入中置零，不产生速度输出；诊断使用 `requested_command_fresh`、`requested_command_speed_m_s` 和 `measured_speed_m_s`。
- 保留扫掠/覆盖/区域算法、原阈值、相机准入、时钟和传感器时效检查、独立连续清空时间；`PROTECTION_CLEAR_CONFIRMATION` 改名为 `NAVIGATION_CLEAR_CONFIRMATION`。
- 保留 publication anchor、来源期限交集和同周期 HOLD；不把发布时刻重标当作重新授予完整上游期限。HOLD 只表示导航约束，不声称执行器已停。
- 生产 launch、规划/BT/route 消费方、五消费者 ACK 由主协调任务负责。

## 主任务验证入口

CMake 目标：`navigation_constraint_cpp`、`navigation_constraint_ros_test`。仅使用主任务分配的隔离构建/install 和 ROS 域；不覆盖历史候选安装。C++ 场景：相机故障恢复须新 proposal；proposal 空窗不重置独立清空证据；真实障碍重置清空；fixed_v2 节点没有 Twist/envelope ACK publisher。静态入口：`python3 ws_robot/src/astribot_s1_navigation_policy_native/test/test_navigation_constraint_wiring.py`。

前三个行为场景使用 legacy geometry；fixed_v2 场景只验证发布/订阅边界。测试只证明本节点接口和约束行为。整链如何执行约束、有限时间停车、取消终态和搬运闭环仍需主任务验收。

主任务原始证据：

- `runs/mainline_20260924/nav_boundary/logs/policy_build.log`（节点构建成功、测试首编失败）
- `runs/mainline_20260924/nav_boundary/logs/policy_build_humble_fix.log`（测试重编通过）
- `runs/mainline_20260924/nav_boundary/protocol/constraint/stdout.log`、`gtest.xml`（4/4）
- `runs/mainline_20260924/nav_boundary/protocol/constraint/result.json`（命令、隔离环境、进程身份、退出码 0、无遗留进程）
- `runs/mainline_20260924/nav_boundary/logs/wiring_tests.log`（联合静态 28 项通过；主任务确认含本节点 4 项，不另计重复运行）

新源码哈希已与冻结记录逐项复核；节点和测试二进制哈希随 manifest 保存。

## 归档与范围

旧节点、构建、原协议/期限检查、核心几何/期限文件及生产装配共 12 文件先归档并逐成员 SHA256 校验。记录见 `m3_final_protection_before_migration.json`；新源码哈希见 `m3_navigation_constraint_source_manifest.json`。

`final_protection_core`、历史 probe/differential 脚本保留用于既有数学实现和历史证据；没有重命名整库或重写历史对照。它们的保留不代表生产执行链仍使用末级速度门控。

暂停中的 `fixed_station_navigation` helper、旧 M3 探针和 FoundationPose 后端阻塞未在本轮修改或重新验收。

## 限定旧对比入口清理

2026-09-24，按主协调任务转达的用户要求，归档且删除 native/test 下 `test_final_protection_protocol.py` 与 `benchmark_final_protection.py` 两个旧 final_protection_cpp 运行对比入口。相关生产构建/启动及工具树精确检索仅命中这两个文件内部的导入和来源登记。

归档 `m3_retired_final_protection_runtime_checks.tar.gz` 含两入口及 reference bootstrap/protection_node 参考快照，逐成员 SHA256 验证后才删除入口。详细清单、SHA 与依赖审计见 `m3_retired_final_protection_runtime_checks.json`。该归档用于源码追溯，并非完整可独立运行的 ROS 环境。

共享 `python_reference/.../protection_node.py` 仍被 `policy/test/test_protection_clock.py` 使用，保留在不随生产包安装的验证参考目录；共用 core/math differential、历史证据和 tools/vision 的故障探针均未删除。4 项冻结新节点源码哈希与保留核心/回归文件哈希复核一致。本轮仅验证归档和文件状态，不另启动运行测试。

## 上肢限速消费补漏

只读参数 `require_arm_speed_limit` 默认 false；主任务 launch 按原上肢耦合开关显式启用。启用时订阅专用 `/navigation_policy/arm_speed_limit`（nav2_msgs/SpeedLimit），仅接受 frame_id 与 profile.base_frame 一致、percentage=true、有限 0..100、非未来且不回退的采样时刻；source ROS 与 steady 接收年龄各小于 500 ms。同 stamp 重播不刷新接收年龄，同 stamp 改百分比视为无效，非法消息后必须新合法采样恢复。

线/角上限先取包络和策略较小值，再乘百分比；本专用 topic 的 0% 明确表示 HOLD，不使用通用 SpeedLimit 的 no-limit 约定。缺失、过期、非法均导致 HOLD，并保留已有故障原因。arm 采样截止纳入发布期限交集，并用 steady 接收有效期的剩余量收紧发布 lease；节点仍无 Twist 或 envelope ACK 输出。

新加 1 个 ROS 协议场景，覆盖 25% 上限、无采样/停源/重播、非法数字/类型/时间、0%、恢复及障碍原因优先；总计 5 个场景，主任务最终修订构建/安装通过，5/5 隔离 ROS 协议通过（12.461 s）。原始证据见 `runs/mainline_20260924/nav_boundary/protocol/constraint_arm/{stdout.log,gtest.xml,result.json}`，构建/安装日志见 `logs/policy_arm_final_build.log`、`logs/policy_arm_final_install.log`（同一 nav_boundary 目录）。进程退出码 0、无遗留进程；4 项冻结源码哈希复核一致，构建/安装节点与测试二进制 SHA 随 manifest 保存。

该用例采用 ROS 与 steady 时间 1:1 前进，只证明消费节点协议。未另跑低 RTF 独立时效矩阵，实际生产者联合、整栈仿真和真机仍由主任务后续验收。未扩大故障矩阵。任务计时保持 14:30 到 15:30，本补漏已于检查点前交付。
