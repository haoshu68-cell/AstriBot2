# 下一阶段：完整策略运行时迁移与绑定退役

已完成首个纯核心阶段：类型契约、持续融合、传感器健康/标定、多边形连续扫掠、风险组合
均有原生库。整包 8/8 CTest、126 个差分测试和 1,783 个额外风险回放通过；边界失败、
sanitizer 和配对性能证据见 [policy_risk](evidence/pybind_removal_20260921/policy_risk/README.md)
及相邻 fusion/health 目录。它不代表两策略运行入口已迁移；当前仍有 79 个自有 Python
运行时文件。下面的 ROS 接口、完整状态机与去绑定门槛继续适用。

Stage 6B 已形成直接 C++ observer 候选，记录于 `1b51e14e`：独立源码/安装11/11 CTest，34组ROS场景两侧通过，1600帧配对性能和输出验证。**仍未达到入口删除门槛**：超大JSON整数及孤立surrogate字符串有已确认差异；先按 [JSON边界方案](evidence/pybind_removal_20260921/policy_observer/json_boundary_review/PLAN.md) 补齐，再推进下述P2～H2闭包。原Humble CameraInfo拒绝问题单独留证，不在迁移中静默改变准入。[候选证据](evidence/pybind_removal_20260921/policy_observer/README.md)。

Stage 6B 整数修复已记录为 `b7a2240b`：任意整数校准版本/尺寸、精确像素比较、原输出时刻的uint64检查和整数位数环境门槛均接入。独立13/13 CTest、安装82/82成对ROS、1600性能帧通过，见[证据](evidence/pybind_removal_20260921/policy_json_integer_runtime/README.md)。新增P2选择器动态profile与路径语义身份核心已差分/UBSan验证，但未接入完整controller。下一步保留surrogate/Stamp边界及P2～H2闭包门槛。

## 可完成的绑定删除边界

后续字符串修复已记录为 `f4feccdf`：14/14 CTest、108/108安装成对ROS、47+28适配器/字符串sanitizer及1600性能帧通过。孤立surrogate、NUL、原health序列化与TF转换失败顺序已补齐，[证据](evidence/pybind_removal_20260921/policy_json_strings/README.md)。当前首项是[Stamp域及失败时机](evidence/pybind_removal_20260921/policy_stamp_boundary_review/README.md)，然后继续P2～H2完整controller；上述旧段落描述各自历史检查点。

| 运行入口与闭包 | 实际源码范围 | 完成后的绑定状态 |
|---|---|---|
| policy_observer | 16 个 policy 文件 + polygon.py，17 文件 / 2,815 行 | 不能删除绑定；Python PolicyNode 继承并调用这些模块 |
| policy_controller 全 P2/P3/P4/P5/H2 | policy 32 文件 / 5,023 行；加 polygon 与两个 social 外部模块，共 35 文件 / 5,391 行 | 与 observer 一起迁移后，才能删除 navigation 数学绑定 |
| transport_task → RosBackend | 11 文件 / 2,383 行，含共享依赖 | 是删除 geometry 绑定的另一必要运行单元 |

行数包含注释与空行，导入闭包包含配置分支，不表示所有路径同时执行。不得把这些
文件重新标成验证脚本来降低剩余数量。

`_navigation_math_native` 的生产直接导入者有 11 个：behavior、candidate_variants、
continuous_sweep、execution_context、motion_geometry、path_evidence、planning_session、
protection、risk、sensor_health、swept_geometry。

`_geometry_native` 的生产导入者为 observer_node、fusion、polygon，以及 transport 的
source_inbox。SourceInbox 虽然只有 18 行，仍属于完整搬运事务后端；不能新增 RPC
薄包装或改回 Python 算法来宣称该绑定已退役。

## 实施顺序

优先在现有 `astribot_s1_navigation_policy_native` 内增加运行库和两个 native 入口，
直接链接其已有 navigation_math / policy_profile；这两库当前不是独立导出的消费包。

1. **观测和世界状态：**observer_node、contracts、ports、fusion、observation_adapters、
   sensor_health、execution_context、profile、stop_reference、robot_envelope、risk、
   world_geometry、swept_geometry、continuous_sweep、motion_geometry、protection，以及
   polygon。交付独立 C++ observer 和完整持久融合状态；此时共享 Python 模块仍不能删除。
2. **P2：**policy_node、behavior、path_evidence（新增 426 行对应范围）。保留原路径证据、
   等待/低速、约束租约和状态输出。
3. **P3：**route_coordinator、planning_session、candidate_safety、candidate_variants、
   obstruction_retry、start_maneuver、start_maneuver_adapter（968 行）。保留有界请求、
   迟到响应淘汰、起步许可和 BT 最终提交权。
4. **P4/P5：**corridor、corridor_adapter、corridor_detection、fixed_corridor（659 行）。
   保留人工/自动候选、非对称载荷准入，以及通道内禁止旋转。
5. **H2 与入口退役：**social_adapter、social_behavior（307 行），及外部 social 校验和
   坐标变换 helper。两入口完整验证后，删除旧 console/module 和 navigation 绑定定义、
   构建及依赖；不能把原生数学库存在视为运行时迁移完成。
6. **后续 geometry 收尾：**完成搬运事务与 ROS 后端，再移除剩余 geometry 消费者与绑定。

第 1–5 步是最小完整策略交付，而不是五个可单独声称“去绑定完成”的阶段。
外部 social 节点仍是独立迁移单元，不能因策略不再导入其 helper 就删掉节点源码。

## 复用与缺口

- `navigation_math.hpp` 已有 YieldPolicy、ExecutionContext、PlanningSessionState、
  ControlTime、CommandRestriction、矩形扫掠、路径插值和候选变体。仍缺完整 ROS 适配、
  融合持久状态、风险组合、候选协调、通道与 H2 状态机。
- geometry_kernels 已有投影/清空、凸包、膨胀、包含、box 距离和 float32 hash规则，
  必须直接复用，不能重新实现数值规则。
- 新增 policy_fusion 已实现关联索引、去重、provenance、seen、unassociated、传感器
  顺序、epoch 和清空状态，复用 fusion_snapshot 单轨迹计算。ROS adapter 尚需接入
  包络更新、capture TF 和邮箱，不能直接视为 observer 迁移完成。
- 矩形 motion_clearance_rect 和点障碍布尔 protection_swept_collision，不能代替
  多边形连续扫掠中的 box 距离下界及候选排序。新增 policy_sweep / policy_risk 已通过
  原多边形距离、逐层细分、预测 owner 及 measured-speed 差分；候选与 ROS 状态仍待接入。
- ProtectionProfile 的验证不涵盖 FixedEnvelopeProfile 的 configuration_key、
  confirms_applied 与独立 heartbeat ACK 状态。保留原配置消费和确认时机。

## 必须通过的行为门槛

- **观测：**scan/odom sensor QoS、map transient-local、observer 的 /plan 与 controller
  的 /path_tracking/active_path 区别；有界邮箱最多 5 scan，选择最新可变换的采集帧。
  覆盖乱序/future、TF 晚到、过期、旋转地图、清空证据、重复 provenance 和 epoch。
- **权限和时钟：**observer 不发运动指令；controller 只发 MotionConstraint，最终限制
  仍属于 FinalProtection。覆盖 ROS/steady 两种时间、path 双龄期、boot+epoch/sequence、
  lease、clear-hold、实测速率大于 cap、未知观测不得放行。
- **几何握手：**保留 legacy/fixed_v2 显式策略、源期限、同 epoch 不可变、session、hash/
  完整顶点、非法 V2 撤销、heartbeat 只确认已应用配置。覆盖 float32 边界、负零、微小
  外凸、偏置载荷、包络过期和计算期间的新心跳。
- **P3：**保留 plan_candidate 客户端和 resolve_route 服务；geometry_valid 不是执行
  许可。覆盖 LOCAL→GLOBAL→VALIDATE、预算、取消后迟到响应、地图/定位/包络换代、
  COMMIT 前重新检查及提交后保持 hold 直到 active path 确认。
- **P4/P5/H2：**覆盖起步换代、未知地图外区域、后退覆盖缺失、通道反向/居中/退出、
  H2 只允许 P2、仿真真值门槛、社会观测乱序/标定变化/超时。
- **安装与去绑定：**AST 无生产导入、两旧入口和已退役生产模块不安装、绑定源码/目标/
  依赖删除、干净 C++ ELF 无 Python/绑定映射。旧测试应改为原生 probe 加冻结 oracle，
  不能要求生产安装保留绑定。

`observation_adapters.adapter_class()` 目前允许任意 `module:Class` Python 插件；仓库
配置只发现两个内置适配器。原生迁移需实现这两个并设计明确的原生扩展接口，不能声称
任意外部 Python 插件已透明迁移，也不能通过嵌入 Python 保留运行回退。外部插件如存在，
须纳入实际源码迁移与验证范围。
