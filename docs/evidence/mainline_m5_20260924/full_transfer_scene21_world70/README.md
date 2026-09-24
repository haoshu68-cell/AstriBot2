# scene21：物理附着已生效，场景应用被拒绝，业务资源隔离保留

本次只读核对已关闭的 domain70 场景，归档阶段、物理诊断和业务状态关键索引。未修改原件，未启动 ROS/GPU、观察器、录像或仿真；未分析 MoveIt 实现根因。原始目录：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene21_world70`。

身份：task=`fixed_transfer_968fc02996ed42c9bd50d2c950e3d21e`，context=`83ed539bd3fc4a08b01233c3255e4fc9`，Goal UUID=`e46e2529d0c849a29258bb7697312e37`；lease=`10651d51-16f1-46ab-a183-528e495279d6_1`。仿真实例为 `fixed_transfer_20260924_station21`，物理来源为 `gazebo_kinematic_v1`。

下列数组索引均从 0 开始，指向原始 `full_transfer/result.json`。精简原字段见 `stage_excerpt.json`，不复制整个大结果文件。

| 原始位置 | 已确认的事实 |
|---|---|
| `executor_journal[25/45/65]` | PREGRASP、GRASP_APPROACH、GRASP_CONFIRM 分别确认完成；没有 ATTACH_CONFIRM 阶段完成记录 |
| `executor_journal[67]` | `physical_submission`，阶段 ATTACH_CONFIRM；capture=95.721 s；transaction=`10651d51-16f1-46ab-a183-528e495279d6_1:payload:1:3` |
| `payload_records[3323]` | 首次 accepted=1、applied=1、attached=true；诊断接收 steady=20929.927093862，源时间=95.766 s |
| `executor_journal[68]` | 同一 transaction 的 `physical_applied_scene_submission`；command_id=1、source_revision=2、source_sequence=1418 |
| `feedback_records[872]` | 首次 PAYLOAD_SCENE_APPLY_REJECTED，阶段 ATTACH_CONFIRM；接收 steady=20930.184948363、回调所见 ROS=95.959 s |
| `events[6]` | 父终态接收 steady=20940.200437529、ROS=103.488 s；status=6、success=false、resources_released=false |
| `executor_journal[71]` | `quarantined`，side_effects=true；journal 无 RELEASED 或 resource_handoff_committed |
| `payload_records[3992/3993]` | 末条物理诊断仍 attached=true；末 ledger 为 confirmed=true、ATTACHMENT_CONFIRMED，含 transport_box_01 |

Journal 顶层 issued_at/valid_until 是租约字段，不作为这些事件的精确发生时间；capture、诊断源时间和观察回调的 steady 时间分开保留。物理诊断附着生效、场景应用接受、ATTACH_CONFIRM 完成、完整 PICK 完成和资源释放不是同一结论。末 ledger 确认也不能覆盖父终态失败；这里不推断相互之间的代码根因。仿真运动学附着不等于接触力抓持或真机抓取通过。

父任务终态原因是 `RESOURCE_RECOVERY_REQUIRED:PAYLOAD_SCENE_APPLY_REJECTED`，cleanup_reason=`GEOMETRY_UNCONFIRMED`。业务处置 **UNRESOLVED**，业务 cleanup_complete=false，清理错误为 `RESOURCE_RELEASE_UNCONFIRMED`。保存的仿真停稳检查通过，仅证明所采样停稳条件，不代表业务资源释放。

外层 `result.json` 报告 cleanup_complete=true；`stack/session.json` 记录 state=stopped、remaining_owned_pids=[]、log_capture_errors=[]。这些是本场所有者的进程收尾记录，不能把仍未释放的业务资源改记为释放成功；本次未操作隔离资源。

1072 条父反馈均为 PICK，导航执行、状态、Goal UUID、包络和 ACK 数组为空。没有 LIFT/NAV/PLACE 或完整主线成功证据，本场不标通过。

视频仅复用所有者的解码结果：479 解码帧 = 479 sidecar 行 = 479 元数据帧；M5 独立核对 JSON、索引连续和计数，未解码、抽取关键帧或做视觉验收。10 fps 播放长度为 47.9 s，录制循环墙钟约 94.205 s，不能混作任务时长。`WAITING_FOR_PAYLOAD_SCENE_READBACK` 在状态事件中存在，但没有被逐帧 sidecar 捕获为某帧的最新状态，不能仅凭 stage_frames 索引声称该字幕实际出现在画面中。

来源路径及 SHA-256 见 `source_manifest.json`；所有者收尾和录像元数据见 `video_metadata_review.json`；本次离线归档校验见 `verification.json`。本次交付为失败边界证据，不是修复或完整搬运验收。
