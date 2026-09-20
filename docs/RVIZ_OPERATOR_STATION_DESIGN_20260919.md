# RViz + 自定义 Panel 上位机落地方案

关联补充：[故障记录、关键参数回放与分阶段落地](RVIZ_REPLAY_AND_DELIVERY_PLAN_20260919.md)。回放与参数记录从 P0/P1 开始建设，各阶段交付以补充方案为准。

日期：2026-09-19。状态：设计建议；基于当前源码静态核对，未完成真机运行验收。本文不修改 SLAM 实现或注释，不启动机器人。

## 1. 架构与部署

上位机采用 Ubuntu 22.04 / ROS 2 Humble / Qt5 / RViz2。工作站运行 RViz 和自定义插件；Orin 运行任务、设备、安全与地图生命周期后端。WCS/MES 通过独立网关连接同一任务后端。首版单机器人、人工跨楼层、人工充电。

RViz 负责交互与可视化，不承担搬运状态机、故障恢复或安全停车。关闭面板、RViz 崩溃和网络中断不能丢失机器人端任务事实。上位机不直接发布 cmd_vel，不直接调用 SDK；生产模式不暴露绕过任务后端的机械臂执行入口。

控制请求经过任务后端、现有导航仲裁/机械臂执行适配器、既有桥接与设备安全门。状态反向汇总到 RViz。默认只读连接；取得机器人端控制租约后才可提交操作。同一机器人只允许一个交互控制者，WCS 调度权和人工接管也必须在后端仲裁。

## 2. 与现有仓库的映射

| 能力 | 已核对源码 | 接入方案及缺口 |
| --- | --- | --- |
| 地图、TF、点云显示 | perception、navigation、exploration 已有 .rviz 配置 | 复用内置 Map、TF、PointCloud2、RobotModel、Path、Image、MarkerArray Display；从实际配置读取话题、frame、QoS |
| 建图/定位 | perception/launch/voxel_slam.launch.py 支持 mapping/localization | 包装现有 Voxel 生命周期，不替换为 slam_toolbox/AMCL |
| 地图保存 | perception/slam_session.py 校验 PGM、位姿、关键帧与 manifest | 地图后端管理完整 Voxel 会话；不能只保存二维 YAML/PGM |
| 导航仲裁 | navigation_policy/task_arbiter_node.py | operator: /navigate_to_pose，route: /route/navigate_to_pose，exploration: /exploration/navigate_to_pose；对应 NavigateThroughPoses 也有入口。搬运走 route，人工接管走 operator；不直连 /navigation_executor/* |
| 导航状态 | /navigation/execution_status，NavigationExecutionStatus | 展示 task_id、source、sequence、state、reason；区分已受理、执行中、取消中和终态 |
| 探索 | exploration_coordinator_node | 对接 pause/resume Trigger；有独立探索状态和完成标志。搬运执行期间由任务后端协调探索，不由面板并行发指令 |
| 机械臂 | manipulation、MoveIt 配置、transport_msgs | 当前规划组为 arm_left / arm_right / dual_arm。PlanManipulation 消息定义不是完整执行能力证明 |
| 技能规划 | /transport/plan_skill | 当前实现 plans only，且 use_sim_time=false 时拒绝。首版仅规划预览；真机执行需另行适配、验收 |
| 夹爪 | bridge_msgs/SetGripper | 现有张开度语义 1=全开、0=全闭；force_applied 必须展示，服务成功不代表抓取成功 |
| 日志 | astribot_logging | 继续使用 spdlog 会话日志；补结构化事件流给面板，按任务 ID 关联 |
| 搬运队列、地图目录服务、电池汇总、视觉物体位姿 | 本次定向检查未确认完整可交付链路 | 作为待实现/验收项显示，不以按钮存在宣称能力完成 |

## 3. 界面组成

采用一个 OperatorPanel 容器和按页拆分的业务组件，统一共享连接状态、权限和接口客户端，避免各 Panel 重复建任务客户端。

| 页面 | 主要操作与显示 | 第一阶段 |
| --- | --- | --- |
| 总览 | 机器人身份、硬件/仿真标识、定位/地图、任务、连接龄期、控制权、电池有效性、报警 | 实现；未知数据标未知，不显示为正常 |
| 地图与工位 | 会话列表、预览、建图/存图/定位请求、地图版本、工位编辑、禁行区域 | 先列表/预览/工位；切图待事务后端 |
| 导航与探索 | 点选目标、目标确认、任务受理/执行/取消状态、探索进度 | 优先实现 |
| 搬运任务 | 来源/目标地图及工位、零件、载荷、优先级、队列、阶段反馈、暂停/恢复/取消 | 先模拟后端，再真实闭环 |
| 机械臂 | 组选择、状态、目标/轨迹预览、规划校验原因、运输姿态、夹爪反馈 | 默认仅规划；执行按权限及能力开放 |
| 视觉 | 原图、标记、物体位姿、置信度、坐标系、标定版本、观测年龄 | 展示接口先行；不将检测框当 6D 位姿 |
| 回放与故障定位 | 事件时间线、轨迹/图像回看、参数历史、故障窗口、证据导出 | P1 最小回放，P2 自动故障记录 |
| 参数与诊断 | 白名单参数、差异、回读结果、生命周期、TF/数据新鲜度、关联日志 | 只读先行，调参需后端审查 |

中央 RViz 展示地图、机器人、路径、工位和目标预览；左侧任务页，右侧设备/告警，底部任务事件。软件“请求停止”与硬件急停状态分别显示，不把软件按钮命名成安全级急停。

点选工位使用自定义 rviz_common::Tool：返回 PoseStamped 后先预览并显示 map_id/version、frame、朝向、操作者，确认才提交。禁止全局配置中保留可绕开业务权限的默认目标发送工具。初始位姿调整也须经过地图后端准入，不能默认发布 /initialpose 就能操作当前定位后端。

## 4. 后端模块和接口（以下名称为建议新增，不是现成入口）

新增 astribot_operator_panel（Qt/RViz 插件）、astribot_operator_backend（状态汇总/权限/设备适配）、astribot_task_manager（搬运编排/持久化）、astribot_map_manager（会话/工位/切换）、astribot_operator_msgs（上位机协议）。复用已有 transport_msgs 和 navigation_msgs，避免修改顶层厂家 astribot_msgs 或新建重复驱动。

| 接口 | 形式 | 契约 |
| --- | --- | --- |
| /transport/execute | ExecuteTransport Action | 提交搬运；反馈阶段、重试、持物状态；结果含失败阶段与原因 |
| /transport/pause、resume | Service | 请求必须含 task_id、expected_revision、command_id；返回受理不等于已停稳 |
| 取消搬运 | Action cancel | 显示 CANCELING，设备确认受控停止后才变为 CANCELED |
| /transport/query、/transport/events | Service / Topic | 重连恢复快照与增量事件，含单调序号和任务版本 |
| /map_manager/list、get_location | Service | 地图/工位及版本查询 |
| /map_manager/switch、save | Action | 长时操作可观察反馈；失败返回确定状态，不假设可安全取消每个阶段 |
| /operator/acquire_control、renew_control | Service | 机器人端租约、操作者身份、命令类别权限 |
| /operator/status、/operator/events | Topic | 设备状态、有效性、时间戳、任务关联及结构化日志 |

业务 task_id 用 string UUID 或 unique_identifier_msgs/UUID；独立于 ROS action goal UUID。robot_id 用 string，不能用 uint8 标识任务。时间使用 builtin_interfaces/Time；位姿使用 PoseStamped。任务包含 schema_version、source/target map_id/version/location_id、part_code、载荷类型、质量、抓取策略、幂等键。状态包含 task_id、revision、stage、cargo_state、reason_code、human_required 和观测有效性。

同一 command_id 重发不得重复产生动作。断线后先查询后端任务事实，不凭 UI 旧状态重发抓取。后端持久化任务、阶段、幂等记录与可恢复检查点；进程重启默认 RECOVERY_REQUIRED，重新确认实物状态后恢复。

## 5. 搬运与故障语义

主线：PRECHECK → NAV_TO_PICK → PERCEIVE_PICK → PLAN_PICK → EXECUTE_PICK → VERIFY_GRASP → TRANSPORT_POSTURE → NAV_TO_PLACE → PERCEIVE_PLACE → PLAN_PLACE → EXECUTE_PLACE → VERIFY_RELEASE → STOW → COMPLETE。

若当前位置与取货地图不同，取货前同样执行地图切换流程。抓取和放置都有独立物理验证；不能以规划成功、轨迹返回成功或夹爪电流单项代替抓取成功。货物加入 PlanningScene，携物运输包络交给现有 envelope/导航策略链，必须确认版本生效才能行走。

将“空载收臂位”和“带货运输姿态”分开；不能把全零 home 当通用安全位，也不能在低电、碰撞或急停后自动回 Home。重试仅对可重试故障开放，携物状态不确定时进入人工确认。

暂停：底盘受控停止、机械臂受控保持，维持夹持；后端确认后进入 PAUSED。恢复：重验定位、地图版本、场景、货物和剩余轨迹。取消：先受控终止，不默认放开夹爪。低电按 BMS SOC 有效性、任务能耗余量和分级阈值处理，电压阈值不等同于百分比。关键低电进入负载相关安全处置，禁止无条件回臂。

通信：建议 1 s 心跳、5 s 租约阈值作为待验收配置；安全停车由机器人端看门狗执行。停车完成时间不是固定 5 s，而是检测时间加执行链和制动时间，需按速度/载荷测试。重连后不自动继续危险动作。硬件急停具体停止类别、驱动失能、制动及夹持保持依据厂家电气设计验证，不能简单承诺“全系统断电”。

## 6. 多地图切换事务

每个 map_id 包含版本、楼层、Voxel 完整会话位置与校验和、导航栅格、世界坐标系、工位集、受限区和人工交接点。工位包含导航接近位、操作位/标记、允许定位误差、载荷限制。

流程：导航到交接位 → 停稳并保持运输姿态 → 等待人工转运及确认 → 阻止新运动任务 → 备份旧会话信息 → 装载目标会话 → 验证定位质量、TF 新鲜度、地图/代价地图版本与安全状态 → 提交新地图版本 → 重新规划。

禁止通过更换 frame 名称伪造坐标变换；禁止新图配旧工位/旧路径继续执行。失败停留明确错误态；只有机器人仍在旧地图物理位置且条件满足时才允许恢复旧会话。人工已跨楼层时，不能靠回滚旧地图恢复运动。轮式机器人不把楼梯口当可自主通行的楼层连接。

## 7. RViz 插件实现与网络

C++17、Qt5 Widgets、rviz_common::Panel、pluginlib，使用 .ui 与独立 ViewModel。ROS action/service 全部异步；专用 executor 线程通过 Qt queued signal 更新主线程控件，禁止 GUI 线程 spin_until_future_complete。销毁时取消本地回调、停止 executor 并 join，不能随面板销毁取消机器人全部任务。

RViz 配置只存布局、机器人选择、显示偏好，不存任务权威状态或凭据。连接/查询失败明确显示 stale/unknown。诊断界面不允许任意 ros2 param set；白名单参数按类型/范围/运行状态审核，跨节点参数更改不能宣称原子，需应用确认、回读和失败恢复。

控制事件采用可靠传输和有界队列，快照可使用 transient_local；图像/点云采用与发布端匹配的传感器 QoS、限帧、降采样/压缩。避免无线链路传全量高密度点云拖垮控制反馈。DDS 不天然保证跨机器零拷贝；首版使用受控局域网和明确发现配置，ROS_DOMAIN_ID 是隔离配置而非权限认证。

操作权限必须在后端执行；生产环境隔离直接驱动接口，按部署条件配置 DDS Security/网关鉴权。多机首版只做总览和单机选中控制；当前绝对话题名需要命名空间改造，未改造前可用独立 DDS domain/网关隔离，不能只改 UI 下拉框就承诺多机控制。

## 8. 对原始提案的修正

- 保留项目当前 Voxel-SLAM，不假设 AMCL 的初始化和 load_map 服务可直接复用。
- 不将 YOLO Pose 输出直接视为零件 6D 位姿；还需物体几何/关键点、深度或 PnP、相机/手眼标定、时间同步及误差验收。ArUco 使用对应标记检测与位姿估计。
- 人员检测与 Collision Monitor 是软件保护，不据此宣称安全认证 SSM；距离阈值依据制动时间、载荷、感知误差验证，不能把示例 0.5/0.8/1.5 m 当验收值。
- MoveIt 规划器、碰撞后端、夹爪力控是否可用由实际插件和 SDK 能力决定；不由 UI 选择框推断支持。
- 保留统一 spdlog，不在每个 Panel 新建各自日志系统。跨机器日志可各自落盘并通过 session_id/task_id 合并检索。

## 9. 分期与验收

| 阶段 | 交付 | 退出条件 |
| --- | --- | --- |
| P0 接口冻结 | 状态/能力清单、命名、权限、QoS、错误码、假后端 | UI 明确区分 unavailable/ready/stale；重复命令不重复执行 |
| P1 运维导航台 | RViz 布局、总览、目标预览、仲裁导航、取消、探索、日志 | 无直接速度出口；高优先级接管反映 PREEMPTED；取消确认前不显示已停止 |
| P2 地图与工位 | 会话保存、校验、工位版本、人工换层事务 | 文件损坏/定位失败时禁止运动；旧路径失效；进程重启可识别未完成事务 |
| P3 抓放技能台 | 规划预览、场景物体、执行适配、持物反馈 | 规划成功与实际完成分离；携物包络确认；故障不自动松爪或回 Home |
| P4 搬运闭环 | 队列、持久化、WCS 网关、完整恢复流程 | 固定工位单房间闭环后再验收动态工位/人工跨楼层 |
| P5 现场验收 | 载荷/定位误差/制动/断线/低电矩阵、多机隔离 | 指标由真机证据确认，不以仿真通过代替 |

建议初始可测目标（待按链路和硬件确认）：任务受理反馈 LAN p95 < 300 ms；GUI 控件响应 p95 < 100 ms；1 Hz 状态源连续 3 s 未更新标为过期；100 次相同幂等键提交仅产生一个业务任务；模拟后端连续运行 8 h 无无界队列增长。单独注入 RViz 退出、网络断开、后端重启、定位丢失、载荷不确定、地图文件损坏、重复 ACK 和旧状态乱序事件。

## 10. 参考

- ROS 2 RViz Panel 官方教程：https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-Custom-Panel/RViz-Custom-Panel.html （架构参考，具体 API 以本机 Humble 头文件为准）
- ROS 2 Actions 设计：https://design.ros2.org/articles/actions.html
- Nav2 Collision Monitor：https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/


## 实施约束（2026-09-19 用户确认）

新增功能以 ROS2 功能包为交付单元，优先 C++，减少自定义 Python 节点与采集脚本。首批 `astribot_operator_station` 提供 C++ Panel/采集/离线查询，XML 启动；导航与探索、诊断采集控制接口纳入最小版本。UI 防误操作开关不替代后端权限；生产鉴权、地图事务、抓放执行仍须按分期补齐。详见该包 README 和回放方案第 11 节。
