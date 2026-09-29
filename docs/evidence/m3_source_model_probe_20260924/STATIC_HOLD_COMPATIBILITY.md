# 12:52检查点：现有静止Hold协议兼容性（只读）

2026-09-24。原11:52→12:52已到，实际probe保持BLOCKED_PREREQUISITE，模型NOT_RUN；新场0 Action工装失败不是模型失败。此核对没有修改运行时代码、启动节点、构建或重测静止矩阵。

## 可以提供的真实来源

- `astribot_s1_transport_native/action/HoldResources`，正式Action `/transport/hold_resources`，Goal只有task_id/request_id。它从当前实测geometry关节生成各控制器的1秒保持目标，等待所有子Action成功后进入真实Hold，不要求PREGRASP规划先成功。
- 持有期间由所有者使用 `/transport/hold_executor/renew`，绑定lease_id、resource_epoch与递增sequence。必须保持父Action及真实续约，取消不等于资源释放。
- typed `/navigation/arm_hold` 是`ArmHoldStatus`，包含owner_id、hold_id、lease_s、hold_confirmed、attachment_revision与源stamp。lease_id/resource_epoch还要关联该父Action反馈及执行器状态，不能用单条空速样本生成Hold。
- `/navigation/geometry_state`提供完整性、权威附件确认、source_id/model_revision/attachment_revision/clock_epoch与有效期；初始clock_epoch=0可用。完整EMPTY仍需既有权威库存和独立PlanningScene读回。
- `/get_planning_scene`是MoveIt的独立只读来源；HoldResources本身不会生成M3 scene revision。M3按已批准方式重新登记当前完整Scene并持续比较，不能拿Hold ID代替scene版本。
- `/navigation/set_fixed_envelope`生成真实coordinator session、envelope epoch及hold绑定；`/navigation/envelope_applied`为六消费者同session/epoch/installed_geometry_hash的当前ACK。

源码依据：hold_executor.cpp的start/send_children、all_successful→hold_.begin和typed发布路径；ArmHoldStatus.msg、HoldResources.action、RenewHold.srv；fixed_envelope_core.cpp的propose/acknowledge/tick。既有静止实测记录见docs/M2_MASS_RUNTIME_20260924.md，不重复其矩阵，也不把历史场的Hold/ACK用于新场。

## 当前确定的不兼容

`fixed_envelope_core.cpp:242`在六ACK齐全时设置`navigation_allowed=true`、reason=READY_FIXED；固定包络按新geometry/typed Hold持续给出valid_until。`probe.cpp:238`却要求`!envelope->navigation_allowed`，同时要求该包络新鲜、与初始coordinator/hold/epoch/clock一致。因此“真实静止Hold+当前六ACK READY_FIXED”不能直接通过冻结probe。

先取得六ACK再通过`/navigation/set_robot_envelope`撤销不是现成适配：fixed_envelope_node.cpp的该服务调用core.revoke(TASK_HOLD)，core.tick随后因锁存fault将valid_until置当前now；probe又会因已到期拒绝。不能故意断掉一个consumer、伪造false包络、用历史六ACK或修改消息字段维持表面通过。

结论：现有静止Hold路径可提供真正的持有和版本来源，也可以与PREGRASP解耦；但冻结probe的导航禁止条件与完整六ACK就绪条件互斥。后续需总调度另行决定最小契约调整或独立静止授权来源，本次不授权一行删除门槛，也不增加新运行时功能。保留4eac3656源码/产物，待调度。
