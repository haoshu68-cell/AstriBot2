# 七项架构修复与验证

2026-09-14；跟踪公式、MPPI 权重、ThreePhase 参数和到位判据沿用当前基线。外部验证工程位于 `/home/yjh/WorkSpace/astribot_validation/architecture_fixes_20260914`，临时验证脚本不安装到产品包。

## 修复顺序及边界

| 顺序 | 问题 | 已实现内容 | 专项证据 |
|---|---|---|---|
| 1 | 点云配置部分生效、提交前刷新 | 完整候选配置校验；ROS 提交后事件通知；帧边界同步切换；无效整组拒绝；拓扑参数明确要求重启 | `config_checks.json`，6 项运行检查；恢复参数后扫描结果一致 |
| 2 | 探索搜索阻塞暂停 | 单个异步搜索、不可变地图和历史快照、协作取消、请求代次检查、独立命令回调组；TF 查询不在状态锁内等待 | `search_checks.json`：900 万格、100 次暂停；`search_cancel.json`；10 张地图候选/代价/航向逐项一致 |
| 3 | 多来源争抢导航 | 单一仲裁入口；人工 100、路线 50、探索 10；同优先级新任务可替换；最多一个待接管任务；收到旧任务终态后才下发新任务 | `task_checks.json`：延迟取消、拒绝低优先级、结果转发、用户取消、最多一个执行器所有者 |
| 4 | 世界版本和失败语义 | 任务 UUID、路径版本、地图内容版本、定位跳变版本、包络版本、时钟代次；在途候选绑定完整版本；同任务世界更新不刷新规划次数预算；机器可消费的执行/策略状态 | `context_checks.json` 7 项；原有 P3 协调器 17 项回放；探索被抢占后暂停而不是继续抢目标 |
| 5 | 新传感器需要修改观察器 | 抽出视觉适配器；按工厂和来源配置装配；提供已分割 PointCloud2 障碍框适配器；保留来源/协方差/采集时间/速度可观测语义 | `adapter_checks.json`；`sensor_runtime.json`：第三路模拟深度经真实 ROS 输入、融合和健康发布 |
| 6 | 健康与覆盖未参与决策 | 健康注册表填充 WorldSnapshot；采集过期/乱序/重复/旧标定处理；CameraInfo 标定注册；深度能力和连续方向覆盖；最终防护独立检查原始扫描所需覆盖 | `health_checks.json` 9 项；`coverage_intervals.json`：一度盲区、相关/不相关方向；完整扫描准入等价 |
| 7 | 包络与双臂/载荷接口分离 | 统一 RobotEnvelope；停车时提交运输姿态/载荷/限制；两张 costmap 足迹确认；策略和独立防护使用同一几何和限制；租约过期禁止释放运动 | `envelope_checks.json` 8 项；完整仿真包络及双足迹握手验证见集成回执 |

这些修复保留独立防护，未将其删除为“重复逻辑”。传感器适配与领域契约可扩展，但当前探索 Node 的兼容字段和策略 Node 继承关系仍可进一步整理；这不等同于已经消除了整份架构审查中的所有中期改进项。

## 导航任务入口

- 人工/RViz 兼容入口：`/navigate_to_pose`、`/navigate_through_poses`。
- 路线任务：`/route/navigate_to_pose`、`/route/navigate_through_poses`。
- 探索任务：`/exploration/navigate_to_pose`、`/exploration/navigate_through_poses`。
- 仅供仲裁器调用的执行端：`/navigation_executor/navigate_to_pose`、`/navigation_executor/navigate_through_poses`。
- 状态：`/navigation/execution_status`（任务 UUID、来源、状态、原因）、`/navigation/policy_status`（世界版本、动作约束与原因）。

取消确认不代表旧动作已经结束。仲裁器保留执行所有权，直到收到终态；接管等待超过 10 s 时拒绝新任务，旧执行上下文仍保留，不能绕开屏障。导航正常执行时长不受此接管超时限制。探索取消与被人工抢占分开处理，失败预算仍服务于有界退出，不作为跟踪质量评分。

本机 Humble 实测中，直接重映射 Action 名称及其 `_action` 子服务没有迁移服务端。实现采用仅针对 `bt_navigator` 的命名空间规则；BT 客户端节点保留原命名空间，因此 ComputePath/FollowPath 仍连接原执行链。生命周期服务与 bond 保持既有入口。启动探针检查两个公开 Action 和两个后台 Action 的唯一所有者，避免双服务端同时响应。

## 观测扩展

`PolicyObserver` 的启动参数 `observation_sources` 是 JSON 字符串数组，例如：

```json
[{"adapter":"pointcloud_boxes","sensor_id":"front_depth","topic":"/perception/front_depth/obstacle_segments","required":false,"max_points":32768,"calibration_epoch":0,"position_variance_m2":0.0025,"coverage_body_yaw_half_angle":[[0.0,0.7]]}]
```

`pointcloud_boxes` 输入必须是已分割、去地面的障碍点云；它保守包围整帧点集，不替代上游分割，也不把空点云当成自由空间。`vision_json` 维持原 schema_version=1 输入。自定义适配器可配置 `python.module:Class`，实现 `normalize()`、`last_packet` 和 `message_type`；不需要修改风险或跟踪算法。输入字节、点数和观测数量均有界。

可选 `camera_info_topic` 与 `depth_unit_m` 接入 CameraInfo。标定内容变化递增该来源的 epoch；配置此接口的视觉输入必须匹配当前标定。未配置 CameraInfo 的既有度量框接口继续显式提供度量坐标和协方差，不把分类置信度当作几何精度。二维视觉仍保留不确定性，不能授予深度覆盖。

`coverage_body_yaw_half_angle` 以基座坐标系弧度表示；属于来源配置的已验证覆盖，不是自动推断的相机能力。扫描覆盖则来自逐束有效数据及采集时刻 TF。健康话题只用于交互与审计，独立防护直接使用自身收到的原始扫描。

## 机器人包络与机械臂协作

`/navigation/set_robot_envelope` 接受 `SetRobotEnvelope`，包含半长/半宽/高度、载荷、运输姿态 ID、速度/角速度/加速度/制动上界和 `transport_ready`。调用时底盘必须停稳且里程计新鲜。

1. 上层机械臂任务先提交可覆盖计划姿态与载荷的包络，`transport_ready=false`。
2. 协调器将包络应用到策略、防护及全局/局部 costmap；在等待期间保持禁止释放运动。
3. 机械臂模块完成姿态变换、确认载荷与运输姿态后，再提交 `transport_ready=true`。
4. 两张 costmap 的已发布足迹确认后才发布 `TRANSPORT_READY`。控制器本身不发机械臂动作。

默认仿真使用现有固定运输包络（半长/宽 0.31 m、高 1.63 m、空载）；这是仿真假设，不是对所有真实双臂姿态的测量结论。包络接口不自动计算复杂双臂扫掠体；真机接入方须从完整碰撞模型/TF 和载荷生成覆盖整个姿态转换的包络，不能只取 TCP。

心跳接收与风险计算分离，采用深度为 1 的最新消息缓存，在计算边界同时应用几何、限制和版本，避免工作队列积压旧心跳。计算中途不切换几何；没有新心跳或采集租约真实过期仍保持停车。`envelope_mailbox_checks.json` 验证边界一致性与过期处理。运行观察状态包含包络龄期、就绪原因和必需传感器健康，便于区分真正输入失效与调度延迟。

## 集成验证状态

专项检查通过后进行了完整 Gazebo/RViz 集成。首次集成暴露双 Action 服务端缺陷，该轮保留为失败证据；修正命名空间和启动端点检查后重跑。完整路线、包络交互、最终源码版本与阶段放行以外部 `result.json` 为准，不能仅凭构建成功或进程数判定通过。

启动探针还要求速度转换、机械臂限速与耦合节点存在。集成中曾出现安装元数据不可用导致两个节点退出、其余生命周期仍 active 的失败，已保留对应回执；该轮没有有效运动，不用于评价跟踪性能。

验收继续要求到点欧式误差 ≤ 0.03 m、角度误差 ≤ 1.5°，记录跟踪横向/航向、直线效果、加减速与 jerk、异常旋转、换路原因；完成时长不进入质量评分。P4/P5 不因架构修复而自动开放。

### 最终仿真回执

- `route_verified`：六点全部到达，最大欧式误差 **0.018096 m**，最大角度误差 **1.022735°**；空间质量检查通过。
- 专项证据见上表；现有 C++ 回归 **7/7** 通过。900 万格地图的 100 次暂停 P99 为 **0.597 ms**，协作取消完成为 **0.531 ms**。这两个值分别衡量命令响应和搜索退出。
- `envelope_runtime_verified.json`：真实 ROS 服务与两个 costmap 完成扩展/确认/恢复；策略观察到对应版本及禁止释放状态，过小包络被拒绝。
- ThreePhase、ArrivalController 与导航跟踪参数的基线摘要一致。最终源码摘要 `verified_source_manifest.json` 共 149 项，运行后核对无漂移。
- 历史全过程质量检查**未全部通过**：第一段正面航向 P95 为 **4.255°**，历史上界 **3.363°**；同场修复前 P3 对照为 **4.211°**。其余五段检查通过。没有调整该上界，也不将单轮接近的数值解释为长期无退化证明。
- 初期修复版第三段出现过一次短暂 `INPUT_UNAVAILABLE` 停车。诊断复跑与最终缓存版本均未复现，最终版本该段加速度/jerk 检查通过；原始停顿缺少完整包络诊断，不能认定后续未复现已经证明其唯一根因。

最终仿真保留在 `/tmp/astribot_architecture_verified`，Gazebo/RViz 和 P3 导航保持运行，路线任务已结束、包络已恢复默认值。完整汇总：[`result.json`](/home/yjh/WorkSpace/astribot_validation/architecture_fixes_20260914/result.json)，逐段修复前后数据：[`before_after_verified.json`](/home/yjh/WorkSpace/astribot_validation/architecture_fixes_20260914/before_after_verified.json)。真机与 P4/P5 均未放行。

## 真机验证方案

仿真 profile 不能用于真机放行。保留 `hardware.template.json` 的运输包络、载荷、制动、时延和传感器覆盖证据要求。真机验证顺序为：静态模型/载荷测量与 CameraInfo 标定 → 只读时间/TF/覆盖核对 → 低速空载停车及取消屏障 → 包络变化握手/进程掉线/租约过期 → 载荷制动与窄通道扫掠 → 原路线同指标回归。每项记录源码和配置摘要、真实反馈和失败原因，独立签认后填写硬件证据，不自动复用仿真验收。
