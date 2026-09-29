# 规划后重新观测：限定设计核对（2026-09-24）

结论：**方向可行，当前接口不能直接接通。** 18.2 s 的规划不能通过现有把原 capture+5 s 覆盖推理和规划全过程的客户端。最小候选是把新鲜感知用于生成不可执行的规划假设，完成有界规划后取得新传感器帧的新证据，由 native owner 验证该证据仍支持同一库存物体和原抓取，再决定是否执行。旧结果的 header、valid_until、原接收 steady deadline 全部保留，不能改写成新鲜；MTC 的 120 s 缓存也不能替代感知有效期。

这是当前源码的只读设计检查；没有修改产品、构建、启动 ROS 或运行实际接口。18.2 s 来自本轮任务提供的实测前提，本报告未重新测量。

## 当前具体边界

| 源码 | 现有行为与影响 |
|---|---|
| [pick_planning_contract.cpp:34](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/pick_planning_contract.cpp:34) | check_context 同时检查原 ROS valid_until 和原 steady deadline；期限距 capture 不超过 5 s。129 行将 MTC timeout 截为剩余原有效期。 |
| [pick_planning_client.cpp:113](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/pick_planning_client.cpp:113) | MTC 等待中以及结果返回后都检查原 Request。仅在当前 plan() 返回后加 capture 无效：它已经因原期限过期取消。 |
| [single_box_request_source.cpp:149](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/single_box_request_source.cpp:149)；[single_box_request.cpp:170](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/single_box_request.cpp:170) | 已能取得新同 stamp 的 RGB/cloud/CameraInfo、该 stamp 的 base←camera 与 station←camera TF；期限取新 capture+5 s 和 fixture 期限较早者，steady 期限仍来自原传感器接收时间。可复用，不能把旧帧再次调用当作新观测。 |
| [ManipulationToHold.action](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/action/ManipulationToHold.action)；[hold_executor.cpp:150](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:150) | fullAction 只有物体、目标和抓取参数，没有感知证据或已有 MTC 结果输入。549–569 行自己读 scene、提交 MTC。591 行 consume_plan 是私有实现，注释预留 M3 接入不是已有可调用接口。 |
| [mtc_planner.cpp:88](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_mtc/src/mtc_planner.cpp:88) | RevalidateManipulation 按 context/start_index 取原缓存，期限自原 cached_at 起 120 s；只将新 octomap 放入原各阶段场景，几何检查原轨迹，不检查新物体位姿、身份或抓取。不能拿 success 当作感知刷新。 |
| [hold_executor.cpp:515](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:515) | 非 occupancy 的固定 scene 改变直接拒绝。占据图变更才走现有复验并再次完整回读；最终提交仍检查原阶段起点、guard、资源与基座。此边界应保留。 |
| [payload_transition.cpp:79](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_mtc/src/payload_transition.cpp:79) | payload 复验仅面向物理 ATTACH/DETACH 后的 index=4 与 transaction，不能借用来更新抓取前移动物体。 |

## 现成字段能做什么

[Context/Request/PlannedPick](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/include/astribot_s1_manipulation_perception/pick_planning_client.hpp:21) 已记录库存 object_instance/identity_revision、相机与 processing/source/clock epoch、模型/标定/工位/几何/抓取注册版本、单次 capture、6D 结果、候选、mapping、Plan::Goal 和完整六段结果。它们足够计算并保留旧抓取的物体相对变换：

`T_object_tcp = inverse(T_camera_object_old) * T_camera_grasp_old * T_grasp_tcp_old`

新帧用其自身精确 capture TF 与新 CAD 6D 结果计算：

`T_base_tcp_predicted_new = T_base_camera_new * T_camera_object_new * T_object_tcp`

将此量与原规划 target，以及原 CAD 世界几何、接近方向、实际宽度和注册版本一起核对，才是在验证原抓取。没有必要为了“刷新”重新挑选排名第一的 GraspNet 候选；改变候选、宽度或 mapping 必须视为新规划。候选 ID 是 task 加本次输出序号，不是跨帧对应标识；服务把请求的 object_id 回填，也不是新的身份识别证明（[server:474](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/manipulation_perception_server.cpp:474)、[server:603](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/manipulation_perception_server.cpp:603)）。同一物体仍依赖 M1 的独占单箱工位和库存身份绑定，不能外推为一般实例跟踪。

缺少的关键契约是**跨观测的一致性判据**。当前没有已验证的位姿变化预算、跨帧抓取对应检查或对称等价变换规则。姿态结果虽有 RMSE/coverage/symmetry_equivalent，但 [server:618](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/manipulation_perception_server.cpp:618) 明确残差不是标定位姿协方差，协方差对角填 1e6 表示未知。不能把 guard 的 0.05 rad、起点检查的 0.025 rad、RMSE 或任意毫米 epsilon 当作目标位姿安全余量。最窄合同可先拒绝任何无法证明等价的变化；它能安全拒绝，却不能宣称已经解决真实测量噪声下正常抓取的可用性。

## 建议的最小数据流与新增合同

1. **原新鲜 proposal → 不可执行的候选。** 保留既有 ≤5 s 推理和准入；增加明确的规划候选结果类型/内部步骤，允许它在独立的有界 MTC 预算内计算。不要关闭原客户端的 check() 后仍返回原语义的“新鲜 PlannedPick”。规划期间仍检查取消、资源、身份与版本变化；原观测到期只说明它不能再授权执行，不能把过期包装成当前事实。
2. **同一个 native owner 持有一次完整规划。** 复用现有 planner 与私有 consume_plan 边界，增加受 owner 控制的内部接入，不增加接收任意外部轨迹的公开接口，也不把 M3 已规划结果再交给 fullAction 重做第二次长规划。记录不可变 context、阶段内容/索引、原规划接收时点、规划场景和抓取绑定；重新观测不刷新这些时点。MTC 缓存丢失或到期仍拒绝，不能由新 capture 延期。
3. **规划完成 → 真正更新的 capture → CAD 复观测。** 用新帧独立请求，原 deadline 不动，新 deadline 只来自新帧及其首次接收时点；必须能证明新 capture 晚于本次规划完成的观察屏障，拒绝重复帧。新证据独立带上库存身份、源/处理/时钟、标定/模型/几何/工位/抓取注册绑定，以及它验证的旧 context/stage。比较 invariants 和新鲜度时不能把新 scene revision 直接冒充旧 revision；占据图变化走既有专用复验，其他几何变化走新规划。
4. **同物体、同抓取一致性 → 完整 scene 回读 → 原路径复验 → 最终提交检查。** 新证据的 5 s 必须覆盖实际消费它的最终 gate；任何异步 scene/复验等待后再次检查其 ROS 与 steady 期限、owner lease/context/stage generation，不能用调用前的新鲜度放行调用后的过期状态。若剩余期限不足，取得另一份新 capture，不能沿用旧帧改期限。只允许原 world/目标/attached/ACM/地图元数据不变的窄路径；确有物体位姿或其他固定几何改变则丢弃候选、完整重规划，现有 occupancy 复验不适用。
5. **逐段定义感知消费点。** 本场 PREGRASP 自身约 7.6 s，单次规划后刷新不可能让同一 5 s 结果在后续 APPROACH/CLOSE 时仍新鲜。至少在后续需要目标事实的阶段提交前设置新观察/验真屏障，并利用现有阶段终态和实测 Hold 边界停等（[hold_executor.cpp:853](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:853)）。还必须明确：感知时效要求是“每次授权该段时”还是“该段执行全程”；若要求全程，则超 5 s 段需要持续的新观察证据和到期停止策略，单次后验刷新方案不够。当前代码没有该政策，不能默认延长到整段结束。ATTACH 后由既有物理确认/完整 scene/ledger/geometry/payload 复验接管，不把抓取前感知当成附着证据。

还有一个必须统一的现有接缝：[planning_goal:130](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_manipulation_perception/src/pick_planning_contract.cpp:130) 仅在局部规划 scene 内替换观测 CAD 位姿，M3 不写全局 scene；native 当前自己读全局场景。接入时 owner 必须明确批准并绑定同一权威目标几何快照与回读；不能将 M3 目标姿态和另一个全局对象几何拼起来。若两者不一致，现成场景比较应拒绝。把观测提交为权威世界对象是另一个需要明确所有者和 readback 的步骤，不是删掉 scene hash 检查即可解决。

最小下一步：先由 M1/M3共同明确三项窄合同——“候选规划与新观测证明的分离”、“同库存物体及固定 T_object_tcp 的一致性/对称性/允许误差依据”、“5 s 的逐段消费或全程要求”；然后才增加内部接入及对应的过期、重复 capture、目标变更拒绝测试。现有原 deadline、MTC cache expiry、scene 比较、资源/guard/取消屏障保持不变。当前只能批准这一实现方向，不能声称完整正常 PICK 或实际接口已验证。

## 阅读版本

| 文件 | SHA256 |
|---|---|
| pick_planning_client.cpp | `0867dd6ca9a2aed2ceb124f8c8d2e19248d95748b7d0c9e35df8062a74a71f16` |
| pick_planning_contract.cpp | `740a5341621aec4f8f7bf41e3f262883ac927c0be5e0934b0b7be41930e68a87` |
| single_box_request.cpp | `6867b118e31af84057347a380e3c337f8b7be29c43f9bb04c239d1d760548a80` |
| manipulation_perception_server.cpp | `033dcd995076c084b3d1ca07f08ae99861c4a2645d39084e7c88ae9b0a68fd51` |
| hold_executor.cpp | `51c3bed145d29a61da72cffa88e79c04bdf7122dfb91f41edcc88f5258cca675` |
| mtc_planner.cpp | `2f6efed361e6042cd8a4f7934c6a9021cc926aac4e624c855c715c8adaa111b8` |

## 更小的先行候选：减少收集完整解的数量

追加只读结论：**值得先做计时对比，但尚不能解释 18.2 s 的主要耗时，更不能保证小于 5 s。** 不修改感知 TTL，先将 MTC 完整解数量做成可配置候选、默认保留 12、对照场次显式设 1，比先实现跨观测授权链更小。本次仅评估，没有改产品、构建或运行此候选。

[mtc_planner.cpp:303](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_mtc/src/mtc_planner.cpp:303) 确实调用 `task.plan(12)`。本地依赖源码 [Task::plan:261](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/deps/mtc_source/core/src/task.cpp:261) 的循环条件为仍可计算且完整解数量尚未达到 max_solutions；每轮 compute 后才再次检查。因此，1 会在首次产生至少一个完整解后的循环边界停止，某轮批量产解时不保证恰好只有一个。12 也不保证实际收齐十二个：搜索耗尽、超时或取消都会提前结束。

后续选择不是直接取“第一个发现的解”。`task.solutions()` 是 ordered 容器（[task.h:147](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/deps/mtc_source/core/include/moveit/task_constructor/task.h:147)），解比较器按 cost 升序（[storage.h:328](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/deps/mtc_source/core/include/moveit/task_constructor/storage.h:328)）。planner 在 305–357 行按该顺序检查候选，取**所收集解中第一个通过外部完整校验的候选**并停止；不是验证所有解后再用新的质量函数评分，也不能保证全局最低 cost。

如果仅改变收集数量，以下链可以原样保留：带关节 margin 的 IK/Connect/各段路径约束、IK collision 检查、完整串行 PICK/PLACE 与退出/收臂段、所有输出轨迹的外部几何校验与时间化/2.5 缩放、输出 waypoint margin 后验检查、同一结果与缓存绑定，以及消费者的六段完整性检查。`max_solutions=1` 不是缩短为仅首段，也不应同时减少 IK 分支上限、改变 stage timeout、降低碰撞检查或跳过退出段。

收益与质量权衡：

- 若首个完整解很早出现，当前代码其后继续搜集候选占了大部分时间，设 1 有机会明显减少等待；现有 18.2 s 记录没有本次首个完整解的时间，不能把这一推测写成原因已确认。
- 只收集较少解会失去后续更低 cost 的解，可能选择更长路径、较长执行时间或不同 IK 分支。cost 低并不自动等于实机误差更小，需记录实际轨迹指标。
- MTC 完整解仍可能被本项目额外 validator 或 margin 后验拒绝。当前有 12 解时可以在同一 task 内检查下一候选；只收集 1 后，若无候选通过，外层会重建整 task 进入下一 attempt（最多 8 次且受原总 deadline 限制）。因此 1 也可能增加重试开销、降低成功率，不能只比较某一次成功的耗时。
- IK 最多 32 解、其单阶段 timeout=2 s，Connect 和多个运动段各有 3 s 预算（[planner:220](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_mtc/src/mtc_planner.cpp:220)、[228](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_mtc/src/mtc_planner.cpp:228)、[237](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_mtc/src/mtc_planner.cpp:237)）。这些不是整个任务的 3 s 上限。完整首解本身就可能超过期限；task 外层检查预算也不能把正在执行的一次 compute 当作零时间操作。

最小必要实测，分两层进行，不能互相替代：

1. **仅规划计时配对。** 对同一个冻结 full scene、实测起点、target/pre/exit、touch links、宽度、RobotModel、碰撞配置和时间缩放，顺序对比 12 与 1。先记录首对结果；若有收益，再用少量顺序重复区分随机波动。记录 task.init、首次完整解、task.plan 返回、外部验证/时间化结束、Action 结果接收的 steady 时间，及每次解数量、attempt 数、外部拒绝原因、已选 cost、路径长度/时长/峰速、完整阶段列表。已有 Task/Solution callback 可作为首解观测位置；本次未新增诊断实现。仅“plan(1) 比 plan(12) 快”不能证明跨传感器链 5 s 成立。
2. **原 M3 契约下端到端计时。** 使用真正的新 capture 和首次接收锚定的 ROS/steady 原期限，计入采集等待、模型推理、scene 获取、规划、后验/时间化和最终消费 gate。必须在两个原始期限内完成；只测 MTC 部分小于 5 s 不够。保留到期取消及终态屏障，记录失败与超时，不能通过补发时间戳、跳过 check_context 或增大 valid_until 得到“通过”。

若 1 仍不能满足端到端原有效期，停止把它当期限解决方案，回到前述“不可执行规划候选＋独立新观测证明”的显式合同。即使端到端初始规划已满足期限，它也**不能替代跨 PREGRASP 约 7.6 s 后的重新观测契约**，更不解决 M3/native 接缝、物体跨帧身份或后续 APPROACH/CLOSE 的证据有效性。

本次辅助阅读的本地 MTC 源码 SHA256：`core/src/task.cpp = 97397b803d5ecb81a896e3f972738313203329a272bec11b5020442ca55ea9f6`；`core/include/moveit/task_constructor/cost_queue.h = a37920eeea1e9029f3af79aef312f3d0550623fc553ce78d929ccb71fbeca01e`；`storage.h = ffe7cb29bf1b5c8c695f33071502299daf5043fd8bbdcc96512bee12bcc9935e`。这是本地依赖实现依据，不是本次 18.2 s 运行的分段计时证据。
