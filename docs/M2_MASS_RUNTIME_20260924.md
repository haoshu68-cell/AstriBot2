# M2 质量与固定包络实际接线验证（2026-09-24）

本轮从 09:53 +08 开始，10:19 完成有界收尾。**新质量节点的静止正常链已通过**：同一独立世界中完成 EMPTY、质量低报、六消费者确认、附件转换撤销与恢复。主线普通导航/兼容抓放的旧通过不重复验收；非 home 动作、有载导航和实际臂展通道仍分别依赖 M1、M4，不能由静止通过替代。

## 范围与实际安装

- 使用标准仓库六相机基线，固定包络 `fixed_v2`、P4、social off、空通道清单。
- 在独立安装前缀中覆盖 A4 `fixed_envelope_cpp`，SHA256 为 `7848ef4eef5bf46512e6296e53db9b4416df8f076f49335a3c128f756be68794`；启动文件只补实际 payload environment/session/source 参数。原基线安装未修改。
- 真实 Gazebo ECM 库存 → C++ 账本 → 完整 PlanningScene 独立读回 → 实际关节几何 → 正式 HoldResources → 固定包络 → 六消费者 ACK。
- HoldResources 内部发送当前实测关节位置的保持目标；本轮没有底盘导航目标或非 home 轨迹。附件采用已登记的 Gazebo kinematic fixture，不证明夹持力、接触或负载惯性。
- 完整查询环境、实际映射与证据根目录：[M2_20260924_0953](/home/yjh/WorkSpace/astribot_validation/M2_20260924_0953)。本轮没有给出独占性能时延结论：构建负载未在全部探针期间持续采样。

## 已取得的证据

| 场景 | 已观察结果 | 边界 |
|---|---|---|
| 显式 EMPTY、0 kg | 新节点身份参数正确；真实 Hold 后六方同版本 ACK 通过 | 不是 geometry-only EMPTY 兼容分支 |
| 时钟冻结与恢复 | 旧包络撤销，恢复后新请求/新 epoch 再取得六 ACK | 证明整链撤销；不单独证明质量 watchdog 的独立时延 |
| 0.5 kg 附件申报 0.25 kg | `PAYLOAD_MASS_UNDERREPORTED` | 来自实际源质量及完整场景，未伪造账本 |
| 0.5 kg 附件申报 0.5 / 0.625 kg | 均生成新 epoch 并取得六 ACK；保持释放确认 | 保守高报允许，低报不允许 |
| 保持中实际加入第二附件 | 原权限撤销，出现质量/几何版本失效原因 | 原测试恢复顺序错误，不能将该原场景整体标通过 |
| 修正顺序后的双附件→单附件→EMPTY | 同一新世界中 `0.5→1.25→0.75→0 kg` 全通过；每次旧权限撤销后保持锁存，新 Hold 本身不恢复旧 epoch | 每次真实场景协调、停稳释放、新 Hold、新 epoch；卸掉一个后仍正确判为 0.75 kg 有载 |

最终结果见 [corrected_protocol_result.json](evidence/mainline_m2_20260924/corrected_protocol_result.json)：8 次正向准入均获六方同版本确认，1 次低报正确拒绝。最终完整场景附件为空、0 kg，36 个停稳样本覆盖 0.7 s，速度/漂移/命令均为 0；Hold 返回 `RELEASE_CONFIRMED`，域 90 journal 为 `resource_handoff_committed / RELEASED`。会话已结束且所有自有进程启动身份均退出。

原失败小结见 [stationary_results.json](evidence/mainline_m2_20260924/stationary_results.json)。各场景独立列结果，不拼接为一个成功的完整搬运任务。

## 本轮最小修复

`tools/sim/verify_fixed_mass.py` 原来只接受整数 `CollisionObject.operation=0`；实际 Humble 字典序列化返回字符 `\0`，造成有载完整场景误拒绝。现兼容这两种 ADD 表达，仍拒绝 REMOVE 和错误字符串，几何/坐标/质量规则不变。新增实际格式回归先复现失败再通过，16 项验证工具测试通过；原始有载场景读回离线重放通过。

版本转换验证原先在物理附件变化后先等待 Hold 释放，再同步 PlanningScene。此时几何尚未确认，执行器无法证明关节停稳，按现有逻辑进入资源未决；该失败属于验证编排错误，不据此修改执行器。正确顺序为：

1. 观察旧包络立即撤销；保持底盘停稳。
2. 将实际物理变化协调到 PlanningScene，并独立读回全部附件。
3. 取得新鲜完整几何及停稳证据，再确认旧 Hold 终态和资源释放。
4. 用新 Hold、新请求和新 epoch 准入；新鲜几何或新 Hold 本身不得恢复旧 epoch。

修正后的探针按旧 session/epoch 跟踪全部撤销后样本，在新 Hold 建立后再次检查旧权限未恢复。它不发布虚假账本、几何、Hold 或消费者 ACK。

## 原失败及资源收尾

- 初次 ECM 插件动态加载造成短暂仿真时间停顿，夹具准备因此拒绝。仅在确认加载命令终态后，使用已有 `--resume-registry` 重新观测同一登记集合；不降低停稳/时效门槛，不重复加载源。
- `stationary02` 有载检查受上述操作码解析影响；该场失败保留。
- `stationary03` 版本变化已撤销原包络，但错误恢复顺序产生 `RESOURCE_RELEASE_UNCONFIRMED`。域 89 原 journal 保留，未删除、清空或换 HOME。随后 `stationary04` 正确拒绝复用这笔未决资源。
- 原场景的 supervisor、Gazebo/JTC、MoveIt 和 Hold 启动身份均已确认退出；世界销毁不等于旧资源事务恢复成功。
- 经总调度确认，`corrected_world90` 使用新的世界、domain 90、partition/source epoch；启动前确认新域无占用且无历史 journal。新结果只算新独立资源上的正确协议验证，域 89 的故障恢复仍单列。

恢复入口为证据目录中的 `overlay.bash`、`run_m2_world90.py` 和 `run_loaded_world90.py`。它们是本轮静止验证工装，不是自动部署入口；新场次须使用新输出目录、核实实际资源归属并传入对应 owner/fixtures/查询环境。原安装前缀、参数、测试脚本哈希及实际加载库均保留。

## M4 衔接

[臂展通道方案第 9 节](NARROW_PASSAGE_ARM_SPAN_PLAN_20260923.md) 已给出主线顺序和最小矩阵：实际非 home 臂展直通、全身过窄拒绝、完整退出后的 90° 转向、携物后重判、途中几何失效。复用完整多边形和转向扫掠，不能只按底盘宽度规划；M1 实际动作与保持是前置。
