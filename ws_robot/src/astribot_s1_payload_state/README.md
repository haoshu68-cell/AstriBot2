# 版本化附件账本

本包提供 C++ 账本、消费者和只读 MoveIt 场景核对节点。`EMPTY` 是一条有来源、采集时间和版本的完整库存证据；启动时没有消息、空数组或空 PlanningScene 都不能单独确认空载。

## 数据流和所有权

```text
仿真物理附件清单 / 真机执行反馈适配器（待接入）
  → /payload/attachment_observation [astribot_payload_msgs/AttachmentObservation]
  → 有界接收缓存 → Ledger 校验、记录
  + /get_planning_scene 完整附件只读回查
  → /payload/attachment_state [astribot_payload_msgs/AttachmentState]
  → geometry_state 的 Consumer → 关节和附件共同生成几何版本
  → 保持授权、包络协调及各消费者确认
```

物理抓取/释放、MoveIt attach/detach 和场景写入仍属于原执行任务。本节点不改变这些状态，不发送机械臂、夹爪或导航指令。发布者身份字段是契约校验，不是 DDS 身份认证或资源所有权服务。

## 输入约定

`AttachmentObservation` 必须由唯一、显式配置的环境适配器产生。`environment` 为 `simulation` 或 `hardware`；`session_id`、`source_id`、`source_epoch`、`clock_epoch`、`sequence`、`revision`、`transaction_id` 和采集期限组成追踪身份。

- `full_inventory=true` 表示覆盖当前支持的所有附件挂载点。不能把一只夹爪的空载推成整机空载。
- `UNKNOWN / TRANSITION` 不可执行。`EMPTY` 必须清单为空，`ATTACHED` 必须有完整碰撞几何。
- 心跳增加 `sequence`；状态、几何、质量或碰撞许可变化须增加源 `revision`。源重启必须换 `source_epoch`，时钟回退必须换 `clock_epoch`。
- 账本独立生成 `ledger_epoch / ledger_revision / attachment_revision`。空载→有载→空载的最后一个版本与第一个不同，内容哈希不能代替执行版本。
- `objects` 是完整的 `AttachedCollisionObject` 清单，操作为 `ADD`，坐标在各自 `link_name` 下。一个物体只有一个主挂载 link；本版本支持双臂分别持有不同物体，不表示已支持双臂共同约束同一物体。
- 支持箱体、球体、圆柱及相对挂载点的偏置和旋转。mesh、plane、空几何和不完整尺寸显式拒绝。`weight=0` 仅用于兼容未知质量；不能据此放行有载动力学，固定包络仍要求有载质量大于零。

## 节点参数和时间边界

可执行文件 `payload_state` 必填 `session_id`、`source_id`、`journal_path`，参数只读。`environment` 默认 `simulation`；`allowed_attachment_links` 默认左右 TCP link。日志父目录必须存在且当前进程可独占该文件。

接收队列最多 32 条；清单最多 32 个物体，每物体最多 64 个 primitive 和 64 个 touch link。溢出使证据失效，之后必须取得故障发生后的新采集，场景轮询不能自行恢复。DDS 反序列化前的大小限制属于部署配置，不能把队列容量当成线缆消息大小保护。

源证据期限最多 500 ms，同时检查 ROS 时间和接收时的单调时间。回调到工作线程的排队时间计入原期限；重复采集、场景回查和发布心跳均不能续源期限。场景请求约每 100 ms，待响应上限 500 ms，状态发布周期约 50 ms。工作线程处理持久化与场景核对，接收回调只入队。

JSONL 记录语义变化、首次确认和故障，使用独占锁和同步落盘。坏路径、非完整末行、写入失败或容量达到 200 MiB 后不能继续确认。历史只供审计；重启始终从未确认状态开始。未变化的每次心跳不做磁盘写入，因此这不是持续磁盘健康证明。

## 几何消费者接入

`geometry_state` 默认 `attachment_source_mode=ledger`，并要求 `payload_environment / payload_session_id / payload_source_id` 与账本一致。缺少可信输入时发布不完整几何，不自动降级为空载。

`planning_scene_legacy` 仅供显式兼容回归，不能作为物理附件确认的等价方案。导航 launch 的 `enable_payload_ledger` 默认关闭；开启时必须配置上述身份和日志路径，并另外接入物理证据发布者。它不会自动产生空载消息。

消费者核对账本身份、规范化几何哈希、版本和双时间期限。异步几何完成时再次检查输入版本，并取原作业期限与当前证据期限的较早值，不能因计算结束或收到心跳而延长有效期。

## MoveIt Humble 回读边界

物理来源的质量继续进入完整摘要和版本，不能被Scene默认值覆盖。Humble附着体不保存`weight`，因此独立回读的零值仅视为该接口没有提供质量；负数、非有限值或非零冲突仍拒绝。独立Scene核对身份、挂载link、形状、尺寸及touch_links；仅位姿往返容许1e-12的数值误差，四元数q/-q等价。质量仍必须由物理来源确认；本接口不能单独提供第二份质量测量。

## 当前证据

2026-09-23：51项账本/消费者C++检查通过；Gazebo库存和几何16项检查通过。实际导航仓库四种载荷及21阶段库存、Scene、过期与恢复矩阵通过，见 `docs/PAYLOAD_KINEMATIC_ACCEPTANCE_20260923.md`。仿真适配器把可信实际mesh保守外包络成primitive，账本仍拒绝原始mesh输入。正式ArmHold所有者、六消费者握手、fixed_v2动态运动和真机尚未验收。
