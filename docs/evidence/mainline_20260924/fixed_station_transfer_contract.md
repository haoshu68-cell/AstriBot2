# 固定工位单箱正式主线：恢复施工契约

2026-09-24 14:05 +08 用户批准按建议顺序继续。第一交付限定当前导航仓库、READY 起点、固定工位单箱、C++ 正式父任务及同场录屏。真实模型、零位准备、通道/多负载/复合故障矩阵留在后续，不作为本轮新增施工项。

## 调度与必要性

| 所有者 | 唯一修改范围 | 对首个闭环的必要性 |
|---|---|---|
| M1 | native `hold_executor.cpp` 与组合任务验证脚本 | 一次获取资源，保持同一租约完成 PICK/NAV/PLACE，正确成功/失败收尾 |
| root | ResourceAuthority 三文件、FixedStationTransfer.action、CMake/package、集成与留档 | 已有 HOLDING 无法续执行、正常完成仅支持取消，必须补明确转换；统一接口/构建单写 |
| M3 | fixed_station_navigation.hpp/.cpp/test 三个新文件 | 复用统一导航入口，真实六 ACK、一个子目标终态和到位停稳 |
| M2 | fixed_station_scene.hpp/.cpp/test 三个新文件、私有仿真工装 | 移动后 MoveIt 的 base-fixed world 必须重算工位坐标；唯一实际仿真操作者 |
| M5 | 既有录屏/采证方案的核对和本场结果分析 | 同一真实流程录屏、阶段与状态证据；不做视频平滑或性能扩展 |

每次派发、接口交接、首次实景前及遇到新失败时复核其是否阻断这条主线；开发期间约每 15 分钟复核在途范围。单问题一小时未解决保留检查点，不靠改名重置历史。首因修复沿原历史于 14:05 恢复；组合父任务及其必要 helper 为本轮明确批准的下一依赖，14:10 开始资源核心施工。时间记录不是测试或仿真通过证据。

## 受控流程

`/transport/fixed_station_transfer` 为新的单组合父 Action；旧独立 ManipulationToHold 保持语义，不允许通过 owned() 旁路提交第二个 Goal。一次 acquire，原 lease/resource epoch/续租序列贯穿全程，正常阶段转换不 cancel、不 release/acquire。

1. 准入要求真实 READY/明确库存/控制器所有权与真实底盘停稳；输入为固定登记单箱及两工位。PICK/PLACE 目标与 station 对象的 `gazebo_world` 明确表示本场物理世界，不能冒充 MoveIt 的 `world` TF。
2. 取得同 capture 的实际 world_from_base 和唯一已登记工位观察，核对 ID/name/BOX 尺寸和固定世界位置；只更新两工位 Scene，独立完整读回；其他对象、附件、ACM 等保持。首次 Apply 前持久化副作用，未决读回不得释放资源。
3. 完整 PICK，沿既有物理附着→Scene→真实账本→剩余路径复核→最终实测 Hold。保留源 epoch、账本和 raw 失效锁存。
4. 实际 Hold/geometry/ledger 与同一 lease 持续有效；由 ArmHold::request 生成固定包络请求。导航 limits 明确来自本场 profile，真实质量另与账本核对。服务 accepted 不是全部 ACK；同 session/epoch/hash 六方新鲜确认后才发送 `/navigate_to_pose`。
5. 原始导航 Goal 在 map。每个验收 odom 源 stamp 取当时 map←odom 变换计算实际 map 位姿，不能冻结起点 map←odom。独立停稳使用 odom/最终速度指令，导航 UUID 成功终态及实际到位/停稳都成立后才准备 PLACE。
6. 到站撤销导航权限，得到 ACK 与负向读回；同一租约 `continue_from_hold` 持久化 HOLDING→RESERVED，新操作 context/generation。按到站实际 base 刷新工位并独立读回，转换 PLACE 目标后规划全部放置/退臂段。
7. 完整 PLACE、物理脱离与稳定、权威全库存 EMPTY、独立完整 Scene、空载实测 Hold、全部子目标终态确认后，`ResourceAuthority::complete` 先记录成功待释放再交接。只有其返回 true 后父 Action 成功。资源释放失败保留隔离，正常完成不伪造取消。

## 必须保留的连续性

- 不直接重用 start() 清空上阶段事实。原 authority/journal/lease、motion_ever_sent、真实库存及源/时钟/账本 epoch 贯穿任务。
- HOLD 与 payload transaction 标识包含操作 context/代次，不能 PICK/PLACE 都使用相同 lease+index。
- 同租约下异步 MTC、Scene、guard 回调仍须核对操作 generation/context，迟到旧回调不得推进新操作。
- raw failure 持续锁存；只有已授权自有物理 DETACH 发送边界，检查旧绑定后切换预期原始修订，不能在到站/换阶段直接清空。
- 取消或故障只取消本任务的导航/控制器目标；未返回接受、未知终态或 Scene/载荷事务未决仍保持隔离。首因和 cleanup 诊断分别保存。
- 父任务总 steady 预算最多 540 秒；采集使用既有上限 600 秒，覆盖最终 EMPTY/释放后有界收尾。这是流程/采集预算，所有传感器、控制器和证据 TTL 不变。

## 验证停止点

先关闭已列必要协议，不重跑全仓矩阵。M1 首因修复指定五场协议已回报通过，原失败不覆盖，精确证据见 `../m1_transport_20260924/full_first_cause_verified/`。资源核心本次原 11 项加 6 项转换必要检查共 17 项通过；只证明离线契约。

helper 的必要几何、ACK/终态/停稳检查通过后进行组合协议，再由 M2 独占当前导航仓库实际完整单箱场。第一场若失败，只定位直接原因；不同时加入新物体、窄通道、真实模型或画质优化。实际同场完整成功前不宣称主线已验收。
