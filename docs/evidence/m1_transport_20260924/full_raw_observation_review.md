# 完整执行器原始观测屏障审查

日期：2026-09-24。范围：完整 ManipulationToHold 的物理应用、scene、ledger/geometry、剩余路径复验和最终回读屏障。仅只读审查；本次只新增此文档，未修改产品、构建或运行 ROS。

**结论：确认一个 P1 源码可达路径。** native 已消费的新物理 revision 或 UNKNOWN，可能被尚未过期的旧 ledger/geometry 和冻结 observation 掩盖，导致后续 LIFT／RETREAT 放行。这里的“确认”是源码控制流及数据流确认，**不是动态复现通过**。域 231 的动态 RED 尚未取得结果，不得记为 reproduced、RED passed 或补丁已修复。M1 正在修改，以下 SHA 和行号对应已审版本，不代表修改后的版本。

## 已审版本与证据边界

| 文件 | SHA256 |
|---|---|
| `ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp` | `51c3bed145d29a61da72cffa88e79c04bdf7122dfb91f41edcc88f5258cca675` |
| `ws_robot/src/astribot_s1_transport_native/src/payload_scene.cpp` | `ab7b4fee1eab9d6fb17bee12119028ed19eb890f56bf77e2c2324a041ac0658a` |
| `ws_robot/src/astribot_s1_transport_native/src/payload_frames.cpp` | `2dc5f571e4cc2525d6919f015b0cc4bb5c036fe0886cc9381c812c59b693d4fc` |

对照 [development_source.patch](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/m1_transport_20260924/full_checkpoint/development_source.patch)，已审 hold_executor 另有 369/370/389 行的 TF 立即查询重载修正；payload_frames 和 payload_scene 与该补丁一致。

[pick_229.json](/home/yjh/WorkSpace/astribot_sdk_ros2/runs/m1_transport_20260924/full_six_stage/pick_229.json) 证明了合成控制器、合成 Ignition 物理端点及真实 C++ ledger 的正常流程；没有真实 Gazebo 或真实 MTC 规划验收。其 `payload_revalidate` 服务直接返回合成成功，不能视为真实剩余路径碰撞复验。对应 frozen runner 在 scene apply 后约 0.03 s 注入 revision 变化、延迟 ledger 约 0.15 s；该安排针对较早的 reconciliation 等待窗口，不能据此认定已覆盖“旧 r2 已确认后，复验／最终回读期间 native 才消费 r3”的窗口。

## P1：直接的新物理事实未进入最终屏障

已审 [hold_executor.cpp:226](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:226) 的 raw 订阅回调：

- environment/session/source 不匹配的消息不接纳；运行中的 source/clock epoch 变化会停止。
- 同 epoch 的较新 sequence 入 `observations_`，没有令旧证明因新 revision、UNKNOWN 或不完整 inventory 失效。
- ledger Consumer 独立订阅 `/payload/attachment_state`，不会因 raw 回调收到新事实而自动撤销旧缓存。

[payload_sample:276](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:276) 按 epoch、clock、revision、sequence、stamp 寻找 raw/diagnostic 配对并检查期限；它没有要求 raw 的 `full_inventory` 或状态为当前事务所需的完整结果。

[advance_payload:359](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:359) 取得配对后只立即检查 source/clock epoch。物理应用确认时，412 行把当时的 raw 固定为 `payload_observation_`。之后 426–460 行依次检查：

1. Consumer 仍有效，geometry 仍有效；
2. ledger revision 与冻结 `payload_observation_` 一致；
3. ledger 与 geometry 的 attachment_revision 一致；
4. 对象列表与冻结 observation 一致；
5. 剩余路径复验及完整 scene 回读成立。

这些步骤没有把 **native 此时已经消费的最新 raw** 与冻结 observation 比较。最终 READBACK 可以清除未决标记、确认事务，并在未停止时调用 `next_stage()`（455–460 行）。geometry 的新鲜度检查只针对 geometry 自身；Consumer 的 `current()` 也只检查它自己的 ledger 缓存，二者不能证明没有更新 raw。

源码允许的时序是：

1. 自有物理命令完成为 r2；scene、权威 ledger、geometry 的 r2 已确认，执行器冻结 `payload_observation_=r2`。
2. 剩余路径复验或最终完整 scene 回读尚未完成。
3. **native 的 raw 回调已消费**同 epoch 的 r3；ledger/geometry 因独立传播延迟仍为新鲜 r2，scene 回读仍为 r2。
4. 当前最终屏障继续比较 r2 与冻结 r2，因此可以确认事务并推进 LIFT／RETREAT。

这破坏了“已知更新物理事实必须先完成 reconciliation，才能发后续运动”的边界。它不依赖旧 ledger 已过期；恰恰是旧证据仍处于其合法期限时出现。

## 为什么仅比较 payload_sample 返回值仍不足

`payload_sample()` 从最新 raw 向旧 raw 搜索匹配的 diagnostic。若 native 已消费 r3 raw，但同 sequence 的 diagnostic 尚未到达，它会继续找到仍有效的旧 r2 配对。只给返回的 `sample.first->revision` 增加检查，仍可能看到 r2 并放行。

因此必须区分：

- **授权证据：** 需要新鲜、完整、配对并且与事务绑定一致。
- **已知失效信号：** native 已消费的新 raw 就能使旧证明失效，不应等待另一条 diagnostic 才承认已知变化。

同 revision 也存在类似风险：较新 sequence 的 UNKNOWN 或 `full_inventory=false` 已被 native 消费，而 r2 ledger/geometry 暂未失效。只比较 revision 相等，仍会漏掉“当前物理状态已不确定”。即使 UNKNOWN 已与 diagnostic 配对，已审后续代码也只检查 epoch，随后继续使用冻结的完整 r2 observation。

## 最小修复屏障

在事务已经建立“物理已应用”的冻结 observation 后，将最新已接纳 raw 纳入事务失效判断，至少覆盖新 revision、同 revision 的 UNKNOWN／不完整 inventory，以及与已确认事务冲突的状态。失效判断不得依赖 `payload_sample()` 恰好返回新配对，也不得被后续返回的旧配对清除。

在剩余路径复验和最终 readback 等异步等待之后、确认事务／提交后续轨迹之前，仍需检查这一屏障及原 lease/context/stage generation。已知更新或未知状态没有完成明确 reconciliation 时，保持事务未决、停止推进，沿现有取消／资源恢复路径处理；不能只清除 payload_phase 或未决标记以方便释放。

不要把这一修复扩大成“任何 ledger 尚未追上都报错”：预期自有物理过渡期间的 pending 状态仍按原有有界协议等待；冻结的目标 r2 与同 context 的较旧 ledger r1 暂时共存也是合法等待。应拒绝的是已经消费的新失效事实被旧证明掩盖，而不是正常传播延迟本身。

## 必要的针对性验证

1. 在 r2 已完成 scene/ledger/geometry 对齐后，阻塞复验响应或最终 scene 回读；使 native **确认已消费** r3 raw，并暂时延迟其 diagnostic 和下游 ledger/geometry。释放异步响应，断言没有 LIFT／RETREAT child submission，未决期间不报告 resources_released=true。此例检查旧配对回退。
2. 同一晚到窗口下，让 r3 raw 与 diagnostic 均已由 native 消费，而旧 ledger/geometry 仍新鲜；断言仍不能由旧 r2 完成最终确认。此例检查最终 gate 本身。
3. 将新 raw 改为同 revision、较新 sequence 的 UNKNOWN／不完整 inventory，重复该晚到窗口；不能因 revision 相等放行。
4. 保留已有正常 PICK／PLACE 和合法延迟账本回归：旧 ledger 仅落后、没有已消费的冲突 raw 时可以有界等待，并在真实对齐后推进。PLACE 不应新增 EMPTY 准备要求。

动态测试必须在 native 接收处理边界建立可核实的消费屏障。测试端“已 publish”、DDS 已匹配或等待固定毫秒数，都不等于 native 回调已经消费消息。若未能证明消费先于最终 gate，只能记为时序未建立，不能用偶然成功／失败关闭 P1。域 231 RED 当前仍待结果；本次没有运行这些测试。

## 其他已核边界

未发现第二个确定 P1。正常路径已有物理应用→scene 修改→完整回读→权威 ledger/geometry→MTC 剩余路径复验→独立完整回读的顺序；问题是最终屏障缺失最新 raw 的失效判断。未知子终态、未决 payload／scene 和未确认取消仍阻止资源释放。PLACE 允许其他物体继续附着，不要求整机 EMPTY。

真实 Gazebo 仍需绑定实际 URDF、model-root 变换、物理 parent 注册和真实 MTC 缓存／复验，并验证完整场景往返；合成通过不能替代这些前置与实际验收。文档完成后暂停，等待主任务指派最终补丁复核。

## 最小修复的只读复核

复核结论：**上述晚到 raw／旧配对回退／同 revision UNKNOWN 的已报路径，在以下源码版本中已闭合；动态验收仍待完成。** 本次核验四个文件哈希并阅读实现和测试源码，没有构建、运行 helper 测试或启动 ROS。M1 报告该版本已编译，不能将该报告或本次静态结论写成域 232／233 已通过。

| 修复文件 | SHA256 |
|---|---|
| `src/hold_executor.cpp` | `18c0e47a6c74089d20523a0858b20dfbb2789f6d72c40fcd063bbf67aa1bf7ff` |
| `src/payload_scene.cpp` | `a653fefb1a68320326489175490eabfa800033498f1a584f9d8dd50676e9ede9` |
| `include/astribot_s1_transport_native/payload_scene.hpp` | `a3b5d96b638eeb0a76c70167b71cbc643e2e40870296d05183f5430d4d93f4ad` |
| `test/payload_scene_test.cpp` | `5467d51bd27898b826ff1b579380a3eb19c42125c4dce84219206c9a5c4cf915` |

以上文件均位于 `ws_robot/src/astribot_s1_transport_native`。复核限制为所报 P1 的关闭路径，没有重做完整执行器审查。

修复关闭路径的依据：

- [check_payload_raw:194](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:194) 先检查锁存失败，再直接读取 `observations_.back().value`。它不再依赖 `payload_sample()` 找到新 diagnostic 配对。helper 抛出的失败被保存，后来再次出现旧的正常配对或正常 raw 不能清除此失败。
- [raw callback:232](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:232) 对已绑定事务中的新 raw 入队后立即调用该检查；epoch 变化、同 sequence 不同内容也锁存。正常更小 sequence 仍按旧消息处理。由此，native **已经消费**的新失败证据会阻止推进；没有主张它能拦截尚未被 native 消费的发布消息。
- [require_payload_observation_bound:15](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/payload_scene.cpp:15) 检查 source/clock、revision、完整 inventory、EMPTY/ATTACHED 状态及 canonical 物体几何。新 revision、同 revision UNKNOWN／full_inventory=false、状态或几何冲突均不能借旧 r2 ledger 放行。
- 首次物理应用确认后，在 [425 行](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:425) 冻结 observation 并立即检查最新 raw；之后在完整 scene 回包 357 行、每次 payload 推进 368 行、后续 advance_plan 620 行重检。锁存失败不能让最终回读覆盖成 READBACK 成功，也不能继续推进后续轨迹。
- 原合法 ledger 延迟语义未改：`payload_revision_ready()` 仍让同上下文较旧 ledger 等待，相等才可继续，更高或改变上下文则失败。自有物理过渡尚未建立冻结 observation 前，pending 流程仍保留。

副作用后的隔离边界：如果物理已应用但事务尚未 commit，绑定点发现失效时仍保持 PHYSICAL；scene 已提交或复验等待期间发现失效时保持其原非 NONE 阶段和未决 scene 标记。失败锁存使后续 `advance_payload()` 不能走到清除标记的提交点。释放仍要求 `payload_phase_==NONE`、scene 不未决、物理 client 不未决及既有终态/实测/guard 条件（[906 行](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/src/hold_executor.cpp:906)），所以这些未提交的副作用不能被报成已释放。锁存只在新任务 start 时清除；旧任务仍 owned 时，准入拒绝新任务，不能借重入自动清除该失败。本结论针对所报未提交事务窗口，不扩大为所有其他恢复路径已经验证。

[新增 helper 测试:68](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport_native/test/payload_scene_test.cpp:68) 的源码明确构造：旧 r2 ledger 判据仍为 true，而提供给同一 C++ helper 的最新 r3、同 revision UNKNOWN／不完整 inventory、几何变化均抛错；正常下一 sequence 不抛错，较旧 ledger 仍等待。它提供的是“新对象已进入检查函数”的确定输入语义，不依赖 DDS 发布时序；本次只阅读测试内容，没有读取或宣称其运行结果。该 helper 测试本身也不等同于真实回调锁存、取消和资源隔离的动态验收。

域 231 证据更新：按 [LATE_RAW_REVISION.md](/home/yjh/WorkSpace/astribot_sdk_ros2/runs/m1_transport_20260924/full_six_stage/LATE_RAW_REVISION.md)，旧 ELF 在测试端发布 r3 后，确认 payload 并额外提交六个 JTC goal，随后因 geometry 失效进入 quarantine、resources_released=false。旧 ELF 没有逐 raw 消费 trace，因此这是 **published-race reproduction**，不能改写为“已证明旧 native 处理 r3 后仍放行”。其进程 exit 0 表示缺陷复现断言成立，不是安全运行通过。

剩余关闭证据仅保留原定窄项：运行已编译的 helper 测试；域 232 的未配对晚到 revision、域 233 的同 revision UNKNOWN 各自出现可归属 native 的 PAYLOAD_RAW 拒绝，随后无 payload confirmation、无后续 JTC 提交且保持 quarantine；最终候选正常 PICK／PLACE及合法延迟账本回归仍成立。上述动态结果尚未获得，P1 可标为“源码修复已复核，动态验收待完成”，不能标为整项已通过。
