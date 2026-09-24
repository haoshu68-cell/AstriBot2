# Execution response evidence, 2026-09-24

归档范围：world91、`transport_ready` 初始 profile 的真实仿真 PREGRASP 首段 → Hold → 六方包络 ACK → 父任务取消并释放。仅该首段通过；不是完整 PICK/PLACE，不是 zero→READY，不是硬件验证。未启动 ROS、Gazebo 或构建，未改写原证据和 journal。

原证据根目录：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924`。逐文件原路径、大小和 SHA-256 见 `copy_manifest.json`；归档自身完整性见 `SHA256SUMS`。原视频仅保留实际路径及哈希于 `external_artifacts.json`，未复制或再次解码。

- `observer_precheck_order_fix/`：scene01 的前后源码、精确 diff、失败时序和离线 RED/GREEN 原件。移序把阻塞 observer 预检放在最终 freshness/scene 检查之前，0.3 s 门槛未放宽。时序报告明确缺少故障瞬间的 ledger 接收时间，不能把替代时钟回归称为实机因果证明。
- `ack_contract_fix/`：scene02 的前后源码、精确 diff、实际安装消息 schema/generated Python 和离线 RED/GREEN 原件；删除不存在的 `EnvelopeApplyStatus.mode` 读取。其错误没有被当作安全释放：原始失败摘录保留 `STOP_UNPROVEN_HOLD_NOT_CANCELLED_OWNING_SIMULATION_TEARDOWN_REQUIRED`。
- `execution_response_scene03_world91/`：outer result、原执行 summary、完整 own-lease journal、源身份/运行库清单、实际使用 runner/verify/prepare/overlay/launch/config 冻结原件。`first_stage_evidence_extract.json` 是**派生摘录**：来自原 17,351,016-byte result 的明确 JSON 指针，所选数组完整、无截断，包含原始 ACK/envelope、odom/最终命令、父终态和反馈；不是完整 result 的字节副本。
- `verification.json` 是本次独立离线核验：取消返回 status=5、`TASK_CANCELED`、resources_released=true；完整 journal 最末 `resource_handoff_committed`、phase=0、RELEASED、side_effects=false。取消前已收到六种不同 consumer 的新鲜同 session/epoch/hash ACK；首次匹配 READY_FIXED envelope 为 ROS 71.505（源 stamp 71.504、有效至 71.68196518），epoch=82832289353709。424 是全部记录数，包含取消及释放后的 ACK，绝不把 424 条都作为有效持有期确认。
- 5 个独立停稳窗口由保存的 odom/最终命令样本重新计算通过，各 36 样本、0.7 s；包括取消后及收尾窗口。历史检查调用时刻未单独序列化，重算以末样本接收时刻为基准；原检查记录的 fresh=true 单独保留，不冒充再次测得历史回调延迟。25 条 PID/start-ticks 身份当前均不存活；supervisor 和两个栈启动 PID 也已不存在，session 为 stopped、remaining_owned_pids=[]。

`held_joints`/summary 中实测关节是 JointState，不是原始 ArmHoldStatus。冻结 verifier 确有 typed Hold 订阅和门控，但 raw typed 消息未单独落盘。本归档只能由绑定本 lease/hold_id 的 coordinator READY_FIXED、六方 ACK 和 native hold_confirmed journal 间接佐证，不能称独立 raw typed Hold 观测。

`dependencies_at_archive/` 是归档时读取的依赖副本，**不能宣称执行时已独立冻结**；执行时源/安装身份以 used_*、runtime manifest 和 preparation hash 对照为准。新目录含 COLCON_IGNORE，不参与包扫描。`archive_verify.py` 为本次派生核验过程；其中 RED/GREEN 读取已有证据，不声称本轮重跑 ROS 测试。

未关闭边界：world90 旧 quarantine 未恢复，其 journal 未改；READY 只是本场起始配置，zero→READY 尚未验证；observer 退出码 130，300 s 观测为 INCOMPLETE；视频连续性/感知覆盖交由 M5 单独审计。路径为独立重规划，不能当作相同路径的 A/B 对照。
