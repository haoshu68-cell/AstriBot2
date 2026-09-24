# scene23：附着后剩余路径重验碰撞拒绝

只读核对关闭后的 scene23/domain68，结果文件仅解析一次。原始目录为 `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene23_world68`。本次没有启动 ROS/GPU、观察器、仿真或录像，没有解码，不分析碰撞实现根因。已冻结的验证器未修改。

task=`fixed_transfer_ad3f3a09e197439b81079e3dad831100`，context=`0d00520145224c8586e57b9ec2d92153`，Goal UUID=`c40f5d453ab94120a9cb6bae930ce437`，lease=`5a6ddc56-b9a1-4ac8-91b1-0dd4a76a084a_1`。下列索引从 0 开始，均指向原始 `full_transfer/result.json`。

| 关键索引 | 观察到的进展或边界 |
|---|---|
| `executor_journal[25/45/65]` | PREGRASP、GRASP_APPROACH、GRASP_CONFIRM 确认完成 |
| `executor_journal[67/68]` | physical_submission 后记录 physical_applied_scene_submission：command_id=1、source_revision=2、source_sequence=1609 |
| `payload_records[3995]` | 首次 accepted=1、applied=1、attached=true；接收 steady=22637.413984180，源时间=106.781 s |
| `feedback_records[1074/1077]` | 先出现 GEOMETRY_UNCONFIRMED 过渡状态，随后进入 REVALIDATING_ACTUAL_PAYLOAD_REMAINING_PATH；不把前者另记为首个停止原因 |
| `feedback_records[1079]` | 首停止错误：steady=22638.044129361、ROS=107.258 s，PAYLOAD_REVALIDATION_REJECTED:MTC_PAYLOAD_REVALIDATION:TRANSPORT_POSTURE:EXTERNAL_COLLISION:self collision: astribot_gripper_left_Link_L11<->transport_box_01 |
| `events[6]` | 父终态 steady=22648.062667827、ROS=114.801 s；status=6、success=false、resources_released=false |
| `executor_journal[71]` | quarantined，无 RELEASED 或资源移交提交 |

此时仍处于 ATTACH_CONFIRM 内的剩余路径重验；TRANSPORT_POSTURE 是被拒绝的后续候选阶段，不能说该阶段已实际执行。没有 ATTACH_CONFIRM 事务完成或 LIFT/NAV/PLACE 执行证据。1279 条父反馈全部属于 PICK。

状态推进到实际负载路径重验，支持执行器已越过场景应用和初次读回的流程判断；结果没有单独保存原始 ApplyPlanningScene 成功响应或附着后的完整 PlanningScene。因此本次不把运行时流程证据扩大成 M5 独立完整场景读回验收。末物理诊断仍 attached=true、末 ledger 已 confirmed；这些也不代表完整 PICK 或业务释放。

父终态保留上述重验拒绝，前缀为 RESOURCE_RECOVERY_REQUIRED，cleanup_reason 为空。业务资源 UNRESOLVED、业务 cleanup_complete=false，清理错误 RESOURCE_RELEASE_UNCONFIRMED。外层结果 cleanup_complete=true，stack/session.json 为 stopped、remaining_owned_pids=[]、log_capture_errors=[]：进程已收尾与业务隔离保留分开记录。

## 实际运行版本

本场 `used_run_full_transfer.py:198–201` 从各进程 `/proc/<pid>/maps` 保存实际映射路径和文件哈希。M5 读取已保存清单并重新计算相关文件，以下记录均一致；关闭后没有把新的进程检查冒充当时运行证据。完整路径、PID、启动 ticks、boot ID 和其他库见 `runtime_video_review.json`。

| 运行对象 | SHA-256 |
|---|---|
| trajectory_executor，payload_receipt_order_0013 | `466699591b98eb3ed2a5a37a62a96fd90a30fa12e7aaff415f95c218fc197fcb` |
| libpayload_scene.so，同安装 | `7b1f554d91108b89bcc499d9e2241ee99d8ee437fda9cfe79d13ad0b8ad98717` |
| libpayload_client.so，同安装 | `e4f2fbee6ce87305abd857abc14e68b9306e9d8fed4f5cef5695df6424e4d29c` |
| mtc_planner，rigid_grasp_2302 | `cfe4b67ec5fe42def9c456b02eeb69ef50dab6adce1ba4f89cc48b3c07acd6e6` |
| libtransport_scene_signature.so | `b09745947a9af5daf3eb2d8dfa02910feddce8f1dd1ed39807f5de4184e262c2` |
| 本场冻结 verify_full_transfer.py | `2c0c04601d690bfe4c7810f197d508de0682594587da137953ce9bbc43d67eb2` |

新验证器已随场冻结，但本场未走到最终放置几何校验，不能用此失败场景宣称该末端判据通过。

录像仅复用所有者完整解码记录：614 解码帧 = 614 sidecar 行 = 614 元数据帧；M5 核对连续索引和数量，没有解码、关键帧或视觉验收。固定 10 fps 的播放长度 61.4 s，录制循环墙钟约 107.394 s，不能混为动作耗时。录像阶段字幕与原始 Action/journal 分开使用。

`stage_excerpt.json` 保存阶段原始索引、首因和原结果 SHA；`runtime_video_review.json` 保存进程收尾、录像元数据、运行版本及来源 SHA；`verification.json` 保存本次归档检查。整项任务仍失败，未完成主线搬运。
