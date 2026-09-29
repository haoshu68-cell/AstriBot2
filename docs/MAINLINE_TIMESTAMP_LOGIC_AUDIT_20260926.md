# 主线运行时间戳与失败传播审计

审计日期：2026-09-26。范围：当前 front62 抓取—运输—放置主线，以及 front55/57/58/61 与当前问题直接相关的记录。按用户最新要求先审计，不继续修改运行逻辑、部署或发送运动目标。

## 1. 结论与证据边界

确实存在已核实的时间逻辑问题，不能把连续出现的 `STALE/EXPIRED/INVALID` 全部归因于传感器断流。

1. **源时间领先本地 `/clock` 与源数据过旧被混为同一失败。** front62 旧执行器只要 `source > local_ros_now` 就返回 `MTC_BASE_STATE_STALE`，没有区分源超前、源过旧和本地接收停滞。
2. **连续观测缓存与失败状态机耦合过紧。** `/odom` 按到达顺序覆盖缓存；包络订阅回调也会重新检查里程计并立即停止机械臂任务。某次局部时序不满足被锁存为整个操作失败。
3. **相邻模块的时间口径不同。** 实际执行保护器允许源时间超前 10ms，搬运执行器允许 0ms；两者都保留 300ms 源年龄限制。前者读控制器状态和 TF，后者读 `/odom`，不能宣称检查同一帧。
4. **旧帧积压与生产/消费期限失配是另外两类已证实问题。** front55 的队列积压已有旧插件复现；front58 的内部日志证实 ROS 尚未过期而 steady 期限已到。
5. **后续报错会掩盖首次失败位置。** front62 的 `RENEW_REJECTED:MTC_BASE_STATE_STALE` 是操作先进入 STOPPING 后续租被拒，不是续租首先超时。

front62 的**具体内部 OR 分支没有记录数值**，所以目前不能声称已证明它恰好因为 1ms 或 3ms 超前而失败。只能确认：代码存在这条错误分类路径；外部观察确有源时间领先本地时钟；失败附近没有观察到 300ms 里程计断流。局部 executor 回调延迟仍无法由外部观察排除。

## 2. 实际运行版本与未加载修改

现场：`fixed_transfer_20260926_front62`，ROS domain 40。主线结果已失败并完成资源释放、停稳收尾；本轮不发送新目标。

| 组件 | front62 实际产物 | 审计意义 |
|---|---|---|
| trajectory_executor | `be63b73a…423715` | 本次报错来自旧里程计接收/检查逻辑 |
| 对应冻结源码 | `00ef157b…82b8e` | `geometry_expiry_observation/evidence/hold_executor.after.cpp`；以下“旧行号”均来自此文件 |
| execution_guard | `961443c5…e97596` | 运行清单、文件 SHA 和编译指令共同确认允许源时间超前 10ms |
| geometry_state | `64dd61ea…94ffb` | 已加载取消 100ms wall 节流的生产端 |
| fixed_station_navigation helper | `609c32ae…265269` | v4：首次绑定后不再重复处理包络与 ACK |
| recovery planner/controller | `ad55aefe…9af78f` | v4：短段执行不再重复做包络、ACK、碰撞与策略许可检查 |

实际启动与加载核验是 52 项一致，不代表搬运/脱困闭环通过。详见 [运行核验](../runs/mainline_20260926/recovery_loop_1852/resume_1954/actual_runtime_front62.json)。

用户要求先审计前，工作区已写入单条未来 odom 暂存及分支诊断修改，私有编译于 20:10 完成（`7b1550e7…e29057`），**未安装、未加载、未执行行为测试**。它不是 front62 的运行逻辑，也不算修复验收完成。冻结旧源码 SHA 已与旧构建记录核对，避免拿工作区新代码解释旧进程。

## 3. 必须区分的时间概念

| 概念 | 原本想回答的问题 | 当前使用举例 | 不能推导什么 |
|---|---|---|---|
| 源时间 `header.stamp` | 这份观测对应哪个采样时刻？ | odom、关节、图像、点云 | 数值大于上一帧不等于没有传输延迟 |
| 节点 ROS 时间 | 当前仿真推进到哪里？ | `now()`，受 `/clock` 驱动 | 不同订阅回调不会因此获得统一到达顺序 |
| 本地 steady 接收时间 | 本节点多久没有接到/处理新证据？ | `steady_now - received` | 不能直接等同于传感器采样年龄 |
| `valid_until` | 这份派生证据被生产者承诺到何时？ | geometry、hold、envelope、policy lease | 每次转发都重新盖章不能证明原输入更新 |
| 来源/session/epoch/版本 | 是否还是同一运行周期、对象、几何和任务？ | 载荷修订、包络 epoch、资源 lease | 这不是“数据过期秒数”，不能混同 |
| Action/服务/任务等待预算 | 一项操作允许等待多久？ | 场景查询 3s、资源 lease 2s、脱困总时限 | 这些 TIMEOUT 不代表相机或雷达旧数据 |
| 连续停稳窗口 | 是否连续观察到足够时长的停稳？ | hold/停止收尾 0.5s、脱困停稳 0.6s | 单个最新样本不能证明连续停稳 |

多个模块把 ROS 剩余寿命再当作 steady 剩余预算。例如接收时 ROS 尚余 97ms，则本地 steady 也只允许继续用 97ms。这里不是直接相减两个时钟的绝对数值，而是**把一个时钟下的时长应用到另一个时钟**。低实时率仿真中 steady 可能先到期。若原意是独立现实时间上限，这种限制可以有明确目的；但它不等同于“ROS 源数据已经过期”，也需要与实际生产频率匹配。

## 4. 运行中的数据链与职责

```text
Gazebo /clock ──────────────────────> 各节点本地 ROS 时钟

关节状态 + 附件账本 ─> 整体几何 ─┬─> 搬运执行器 / ArmHold
                                 └─> 固定运输包络协调器
                                      └─> 包络 + 各消费者 ACK
                                           └─> 导航首次绑定
                                                └─> 起点检查/脱困
                                                     └─> 普通导航

/odom ─> 搬运执行器自己的底盘缓存 ─> manipulation_ready
控制器状态 + TF ─> execution_guard ─> 搬运执行器检查反馈

深度图 + CameraInfo ─> 配对/CPU或CUDA ─> 点云/处理健康 ─> 感知消费者
```

**当前分层地图的实际来源是 Gazebo collision 插件**，不应把示意图写成“RGB-D 已生成当前分层地图”。相机处理健康、普通 scan 的新鲜度和当前分层地图是不同来源链。本次 `MTC_BASE_STATE_STALE` 也不是由图像时间检查直接产生。

## 5. front62：本次错误原逻辑、事实和传播

旧运行源码第 135 行：收到 odom 就覆盖 `odom_`，同时记录本地 steady 接收时间；没有先按源时间选择更新的一帧。

旧第 835–836 行的判断等价于：

```cpp
if (!odom || source_stamp <= 0 ||
    ros_now < source_stamp ||
    ros_now - source_stamp > 300ms ||
    steady_now - received_steady > 300ms) {
  reason = "MTC_BASE_STATE_STALE";
  return false;
}
```

五种原因共用一个错误码。举例：上一帧 source=103.500s 已可用，新帧 source=103.520s 先到，而本节点 ROS 时钟暂为 103.519s。新帧覆盖旧帧后，上述代码立即失败；这个分支判断的是“领先本地时钟”，不代表“超过 300ms 没有更新”。此例用于解释条件，不宣称就是 executor 现场缓存数值。

旧第 136 行包络回调以及第 1024 行阶段推进都会调用这个检查。也就是说，触发判断的消息可以是包络心跳，失败理由却来自另一个缓存中的 odom。

| 现场时刻（ROS 秒） | 同次记录 |
|---|---|
| 97.865 附近 | 开始 PREGRASP；尚未附着物体 |
| 103.499 | executor 仍报告 `EXECUTING_MTC_STAGE:PREGRASP` |
| 103.519 | 观察器收到 source=103.520 的 odom，领先观察器本地时间 1ms |
| 103.540 | 观察器收到 source=103.540 的 odom |
| 103.542 | guard 的状态 source=103.542，base_stamp=103.540，报告 `EXECUTION_WITHIN_BOUNDS` |
| 103.550 | executor 首次报告 `MTC_BASE_STATE_STALE`；续租序号 223 被拒 |
| 103.553 | guard 仍报告健康；不证明 executor 收到的是同一帧 |
| 104.240–104.940 | 36 个采样覆盖 0.7s，停稳检查通过，观测位移/旋转/速度/命令为零 |

在观察器 ROS 103.02–103.70s 的局部窗口，35 帧 odom 的最大观察接收间隔为 **40.236253ms**，最大源时间领先为 **3ms**。这是该观察器该窗口的结果，不能代替 executor 内部接收时间。

实际失败传播：

```text
odom 缓存/时间检查失败
  -> stopping("MTC_BASE_STATE_STALE") 锁定首因
  -> ResourceAuthority 进入 STOPPING，取消子动作并做收尾
  -> 后续 renew() 因 STOPPING 拒绝，返回已锁定的首因
  -> 上层脚本显示 RENEW_REJECTED:MTC_BASE_STATE_STALE
```

最后一次成功续租的 ROS 截止是 105.390s，报错在 103.550s；资源账本也记录 STOPPING 原因为 `MTC_BASE_STATE_STALE`。不能把错误前缀解释为最初发生了资源续租超时。

最终：`success=false`、导航目标 UUID 为空、`RELEASE_CONFIRMED`、`cleanup_complete=true`。**本次不是后退途中失败，甚至没有进入导航。**

直接证据：[front62 时间审计摘要](../runs/mainline_20260926/recovery_loop_1852/resume_1954/front62_timestamp_audit.json)，包含原结果 SHA、关键 JSONL 行、续租与收尾。冻结原逻辑：[hold_executor.after.cpp](../runs/mainline_20260926/geometry_expiry_observation/evidence/hold_executor.after.cpp)。

## 6. 与刚才几个场景对照

| 场景 / 表面错误 | 已核实的原逻辑和证据 | 已做处理与剩余边界 |
|---|---|---|
| front55：`ENVELOPE_EXPIRED` | BT 每 tick 的 `spin_some` 消费旧队列，队列后仍有有效新心跳，却先锁存旧帧过期；旧插件在积压夹具复现 | 改为每 tick 有界 1ms `spin_all`，批次处理后再判断所选帧时效；6/6 夹具通过。1ms 用尽不保证全部排空 |
| front57：`NAVIGATION_ACK_INVALID` | helper 把合法未来 ACK 和格式非法混为永久失败；现场 359 条观察 ACK 结构合法，但没有 helper 内部时钟差值 | 未来正 ACK 暂不采用，明确负 ACK 仍撤权；旧库确定注入失败、新库 11 项夹具通过。实际失败具体超前量未证实 |
| front58：`GEOMETRY_INVALID_OR_EXPIRED` | 内部日志确认接收 ROS=227.173、有效至227.270208102；接收剩余97.2081ms，steady拒绝晚于deadline12.7725ms，但ROS才227.208；相邻观察更新约125–133ms | 删除生产端100ms wall节流，保留20ms ROS调度机会、单个在途计算、1ms完成轮询；新产物已加载。不靠扩大TTL |
| front61：`NAVIGATION_ENVELOPE_INVALID` | 已选后退0.10m，FollowPath已接受，helper仍在重复检查包络并取消；100条观测命令全零，未实际完成后退。结构合法，源领先helper时钟是有支持的假设，内部数值未记录 | v4首次绑定后删除helper重复包络/ACK检查，并移除短段外层/执行中的重复门控。已构建及夹具验证，front62提前失败，尚无v4实场后退闭环 |
| front62：`MTC_BASE_STATE_STALE` | 新帧覆盖缓存；零未来容差；五个分支同码；callback/tick可立即停止；观察器见1–3ms超前及持续odom | 候选暂存/诊断修改仅编译，未加载未验收。当前按用户要求先审计 |

front58 的调度修复已经改善观察到的更新间隔，但不据此宣称所有尾延迟已解决。front59 的资源标记缺失、front60 的错误产物/崩溃属于其他操作或部署问题，不能塞进“时间戳过期”作为同一根因。

相关原记录：[front55/57 实施与复现](START_DEPARTURE_IMPLEMENTATION_20260926.md)、[front58 精确期限](NAVIGATION_START_RECOVERY_PLAN_20260926.md)、[front61 分析](../runs/mainline_20260926/recovery_loop_1852/front61_failure_analysis.json)。

## 7. 搬运主线中其余时间依赖清单

此表为原实现与当前行为清单，**不表示每项都在此次报错，更不表示已经全部发现缺陷**。旧行号统一指冻结的 `hold_executor.after.cpp`。

| 数据/模块 | 原检查及主要错误 | 原处理位置与行为 |
|---|---|---|
| 实际关节 | 完整关节样本；源及本地剩余期限约300ms；未来帧丢弃；`PAYLOAD_JOINT_STATE_STALE` | 旧557、610：附件事务不推进；不完整/非法输入不能冒充有效关节状态 |
| 整体几何 | ROS `valid_until` 与本地steady deadline双检查；`GEOMETRY_UNCONFIRMED` / `GEOMETRY_INVALID_OR_EXPIRED` | 旧1077、1110：普通操作停止；附件对账阶段的几何要求与实际关节、账本事务分工不同 |
| 控制器声明 | 从查询请求发出计ROS/steady 500ms；`CONTROLLER_CLAIMS_EXPIRED` | 旧1126、1279：声明过期停止；查询延迟也消耗预算，不是从回复到达重算500ms |
| 场景查询 | 单次读取3s，重验证轮10s；`MTC_SCENE_TIMEOUT` / `MTC_REVALIDATION_TIMEOUT` | 旧1040起：停止并清理pending；服务超时，不是图像源年龄 |
| MTC计划 | 创建后steady 120s；`MTC_PLAN_EXPIRED` | `mtc_plan.cpp:25`：不继续执行该计划 |
| execution_guard输入 | 关节反馈和TF源年龄≤300ms、提前≤10ms，控制器反馈接收≤300ms | `execution_guard.cpp:58–91`：启动初期可等待；已正常观察后出现问题会锁存；TF查询异常也会归入 `MANIPULATION_BASE_STATE_STALE` |
| execution_guard输出 | executor要求反馈源与接收≤300ms；`EXECUTION_GUARD_UNHEALTHY` | 旧1048–1061：执行前等待，执行中失效停止 |
| 物理附件状态 | 源与接收剩余期限300ms；`PAYLOAD_STATE_STALE`；物理应用等待2s | `payload_client.cpp:108`：不把未确认动作重复提交，保持未决事务 |
| 账本/对象位姿 | raw/diagnostic匹配版本、序号及stamp；300ms寿命；库存/位姿等待3s、事务30s | 旧579、673起：先等待匹配证据，再按事务预算失败；stamp也承担关联作用 |
| ArmHold | 至少3样本且ROS/steady均覆盖500ms；输出寿命取多证据最小剩余值，最多300ms | `arm_hold.cpp:134`：几何/声明/资源过期分别取消确认，错误为 `HOLD_GEOMETRY_EXPIRED` 等 |
| 资源租约 | 2s；ROS或steady任一到期；时钟回退 | `resource_authority.cpp:44–67`：`RESOURCE_LEASE_EXPIRED` / `RESOURCE_CLOCK_RESET`；拒绝继续授权 |
| 停止收尾 | 子动作终态＋双时钟500ms停稳＋事务清理；10s未闭合 | 旧1355–1368：`RESOURCE_RECOVERY_REQUIRED:<首因>`，不能把未释放资源记为已释放 |

## 8. 感知、几何与导航时间依赖清单

| 链路 | 实际规则 | 原处理 / 源码入口 |
|---|---|---|
| 关节→几何 | 各关节源stamp，最早源+300ms截止；关节间偏差≤100ms；未来包待处理 | `JOINTS_INCOMPLETE` / `JOINTS_STALE`；[joint_snapshot.hpp](../ws_robot/src/astribot_s1_robot_geometry/include/astribot_s1_robot_geometry/joint_snapshot.hpp) |
| 账本→几何 | 账本`observed_at/published_at/valid_until`；接收后映射steady剩余期限；同一采样重复发布不延长 | `ATTACHMENT_EVIDENCE_EXPIRED`；几何取关节、账本最早期限；[consumer.cpp](../ws_robot/src/astribot_s1_payload_state/src/consumer.cpp)、[geometry_state_node.cpp](../ws_robot/src/astribot_s1_robot_geometry/src/geometry_state_node.cpp) |
| 几何/Hold/载荷→包络 | 包络header是协调器发布时间，另带源几何sequence；valid_until取三者最早；ACK 500ms | 失败锁存，`navigation_allowed=false`，期限置为now；[fixed_envelope_core.cpp](../ws_robot/src/astribot_s1_navigation_policy_native/src/fixed_envelope_core.cpp) |
| RGB-D原始健康→policy | 实际head/torso健康门槛250ms；帧组最早capture+250ms；最早steady接收年龄也≤250ms | 原始或投影健康失败会使普通policy不可用；不是此次MTC错误来源；[camera_health_node.cpp](../ws_robot/src/astribot_s1_perception_components/src/camera_health_node.cpp)、[projection_capability.hpp](../ws_robot/src/astribot_s1_navigation_policy_native/include/astribot_s1_navigation_policy_native/projection_capability.hpp) |
| RGB-D投影worker | 接收阶段按源时间、顺序筛选，处理后再检查generation、输出顺序、ROS/steady年龄；CUDA进程另有请求等待预算 | 丢旧/丢未来帧、失效输出不发布，`OUTPUT_STALE`或`WORKER_REQUEST_TIMEOUT`等；后者不是源帧年龄。`max_pair_age_sec`源码默认250ms，不据默认值宣称所有相机实际配置一致；[rgbd_pointcloud_node.cpp](../ws_robot/src/astribot_s1_perception_components/src/rgbd_pointcloud_node.cpp) |
| 点云→scan→policy | 本次`/map_scan`源，点云age300ms，TF查询等待150ms；policy仿真模式scan/odom按ROS源龄300ms | 无效则policy输出HOLD及原因；仿真`ControlTime::fresh`没有对应的scan steady300ms限制；[navigation_math.cpp](../ws_robot/src/astribot_s1_navigation_policy_native/src/navigation_math.cpp) |
| 当前高度地图 | Gazebo collision插件遍历环境；间隔≥500ms steady且simTime推进后发布 | 5层来自29个环境碰撞体。真实接线证据是front62 `stack/session.log:1436`，不应写成当前RGB-D融合产物 |
| 起点评估输入 | pose源年龄−50～500ms；包络ROS≤300ms、接收steady≤500ms且未到valid_until；高度图ROS/steady均≤1.5s | 证据不可用返回UNAVAILABLE/`ALIGNMENT_LAYER_UNAVAILABLE:*_STALE`，实体碰撞返回BLOCKED；[layered_collision_reader.hpp](../ws_robot/src/astribot_s1_path_tracking/include/astribot_s1_path_tracking/layered_collision_reader.hpp) |
| 恢复BT | 总预算120s steady；输入/服务每轮2s，输入不可用每100ms重试；许可等候2s | WAIT_INPUT/WAIT_PERMISSION，预算耗尽报警、halt子动作；RECOVER阶段不重复查许可；[navigation_start_bt.cpp](../ws_robot/src/astribot_s1_navigation_recovery/src/navigation_start_bt.cpp) |
| 已批准短段控制器 | pose源年龄−50～500ms；总120s、无进展15s，ROS或steady任一到期；停稳双窗口0.6s | 失败抛错；完成声明STOPPED再复检。没有包络/ACK/碰撞重复准入；[departure_controller.cpp](../ws_robot/src/astribot_s1_navigation_recovery/src/departure_controller.cpp) |
| 普通Envelope BT | ROS源龄300ms、steady接收龄500ms、ROS<valid_until | 失效停止普通导航分支；明确撤销立即锁存；[path_guard_bt.cpp](../ws_robot/src/astribot_s1_path_tracking/src/path_guard_bt.cpp) |
| helper首次交接 | Envelope期限=min(valid_until,source+300ms)，ACK期限=source+500ms；ROS期限与steady剩余期限同时有效 | 许可不齐等待，v4绑定后停止消费这两类许可消息；[fixed_station_navigation.cpp](../ws_robot/src/astribot_s1_transport_native/src/fixed_station_navigation.cpp) |
| helper导航与收尾 | odom300ms、非零命令静默300ms steady；准入15s、目标接受5s、结果180s、取消收尾10s等分别计时 | 取消自有目标并确认终止、零命令与实测停车；收尾不明为UNRESOLVED，不等同传感器过期 |
| 上层policy与普通控制器许可 | policy每20ms wall评估；仿真时钟停滞500ms steady；发布期限还按scan/odom/proposal/上肢等最早截止裁短；普通PolicyLease使用ROS和steady双年龄 | policy发布HOLD/限速/允许约束；普通消费者拒绝过期许可。[navigation_constraint_node.cpp](../ws_robot/src/astribot_s1_navigation_policy_native/src/navigation_constraint_node.cpp)、[policy_lease.hpp](../ws_robot/src/astribot_s1_path_tracking/include/astribot_s1_path_tracking/policy_lease.hpp) |

这里的policy是上层规划约束链，类名包含`ProtectionProfile`不意味着重新启用了用户已取消的底盘末级执行保护。

对未来正向数据，当前还有明显不一致：helper初次绑定时暂不采用；`EnvelopeEvidence`最多暂存8条同上下文候选；policy的`ProtectionProfile::accept`却把`age<0`与`age>300ms`都写为`V2 stale heartbeat`并清空许可；相机健康门控在同上下文情况下会忽略未来正向样本、保留旧证据原截止。上述源码行为已经确认，但未把它们全部认定为front62的现场首因。

服务在2s内返回仅证明没有超过等待预算。恢复BT当前消费的是服务状态；恢复Planner会重新取快照。不能因此额外宣称BT已验证响应中全部数据的消费时年龄，也不在此次审计中新增检查。

## 9. 处理逻辑应怎样收口

以下是审计提出的修正方向，**本轮未实施**。应先按用户“只接受最新时间戳”的要求确定语义，不继续在每个模块临时加容差。

1. **把连续观测选择与任务失败解耦。** 对同来源/同运行周期，按源时间保留最新样本、丢弃倒序样本；重复帧不冒充新的源进展。接收回调更新缓存和诊断，不因某次 `/clock` 与数据到达顺序直接调用整任务停止。
2. **明确“只取最新”的模式。** 如果主线取消源年龄阻断，则源时间仅用于排序和诊断；不能生产端允许、helper/BT/executor又各自按另一套年龄规则否决。也不能只把阈值放大后称为已经取消检查。当前尚未全链切换到该模式。
3. **区分观测与事务。** 同版本 ACK、明确负确认、资源身份、Action等待时限、连续停稳窗口分别处理；“取消帧年龄阻断”不自动等于删除这些不同职责。stamp用于TF采样对应和RGB-D配对的用途也需明确保留或替换，不能把它当作报错开关一起清空。
4. **收紧错误传播，而不是隐藏原始错误。** 保留第一失败模块、具体条件和原始数值；后续取消/续租拒绝记录为后果。不要让“尚未收到匹配帧”“源略领先时钟”“明确撤权”“确实断流”都报同一个 STALE。
5. **保持已批准的脱困分工。** 检查起点→受阻时按当前环境/包络选短段→执行已批准短段→实测停稳→重新检查→READY后普通规划。短段内不重新引入已删除的包络/ACK/碰撞重复门控。

最小核对案例仅覆盖本次已出现的机制：消息先于clock、旧帧排队、几何周期与期限、错误首因传播、同一front场景PREGRASP到正常导航交接。不会在主线完成前扩大故障矩阵。

本轮完成的是源码、实际产物与离线现场记录审计；没有完成新的动作回归，也没有把候选编译通过记成问题已解决。
