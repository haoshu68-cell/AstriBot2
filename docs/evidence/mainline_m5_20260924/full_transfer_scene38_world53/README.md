# scene38：放宽条件已读回，ATTACH 前场景重验失败

原始场景：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene38_world53`，domain 53。本次仅离线复核已落盘结果、参数、journal 和视频记录；未运行 ROS/GPU/build、解码视频或修改冻结脚本。原 06:02:36 检查点不变。

实际读回确认：task_trajectory_executor 和 manipulation_execution_guard 的 `simulation_relaxed_base_motion` 都为 bool true；底盘 `idle_position_hold` 为 bool true、`idle_position_kp` 为 double 3.0。报告明确本轮使用 rotation=0.05 rad、angular=0.10 rad/s 的放宽条件，原条件为 0.02 rad、0.03 rad/s；非 NAV 线速 0.02 m/s、非零命令拒绝和最终停车判据保持原值。本场不能算原精度验收。参数读回成功也不单独证明保持控制效果或完整搬运成功。

三个已完成阶段的实际发送与 response 元数据完全一致，同一控制器/UUID 的 terminal 均为成功码4，随后 stage_confirmed 包含该 UUID：

| 阶段 | send → response → terminal → confirmed 的 journal 索引 | 轨迹点数 | UUID |
|---|---|---:|---|
| PREGRASP | 8 → 14 → 21 → 26 | 424 | 3db64d37d6e65582cc25c6f95d6b608d |
| GRASP_APPROACH | 29 → 35 → 41 → 47 | 5 | 6f81e83939496713ad53736425a91d7c |
| GRASP_CONFIRM | 52 → 58 → 64 → 68 | 10 | e37f8758706ebd47a046d707c8db4316 |

索引均从0开始，指向原始 `full_transfer/result.json` 的 executor_journal。全部73条事件属于同一 owner/lease/epoch。GRASP_CONFIRM 为夹爪控制阶段完成，不代表物体已附着或抬起。

此后准备进入 ATTACH_CONFIRM，剩余计划重验以 `MTC_SCENE_CHANGED_DURING_REVALIDATION` 拒绝：journal[69] stop_requested；首个对应父反馈为 feedback_records[792]，接收 ROS=87.993 s、steady=42406.431140670，stage_id 仍是 GRASP_CONFIRM。没有 physical_submission、payload_suffix_adopted 或 payload_transaction_confirmed，因此本场没有执行实际 ATTACH、LIFT、TRANSPORT、NAVIGATE 或 PLACE；returned→adopted→sent 后缀链未到达。

原首因保留为 `RENEW_REJECTED:MTC_SCENE_CHANGED_DURING_REVALIDATION`。父终态 status=5、success=false、resources_released=true；journal[72] resource_handoff_committed/RELEASED，cleanup_complete=true、无清理错误。停车记录36样本/0.7 s（ROS 88.64–89.34），平移/旋转漂移、速度和命令均为0。会话已 stopped，remaining_owned_pids=[]。

视频所有者完成原视频解码：445帧，与metadata和连续0–444 sidecar一致。只读复核确认非空帧status与原始status事件对应；阶段标注覆盖 PREGRASP 244–272、GRASP_APPROACH 278–284、GRASP_CONFIRM 291–305、准备ATTACH 306–320、失败321–444；324起authority为RELEASED。初始112帧status为空，不补造阶段。M5未进行像素检查；这些状态标注不代替journal的完成证据。10 fps下44.5秒播放不等于97.1445秒录制窗口的实时速度。

本场使用的验证器SHA256为 `1333136611ff06fbf31d285858c61833893c6997f11a8937ec936b9ac3081184`，参数读取器为 `6db3a2214f222c66f02eff73d9deac3b8d6cbc698321bb4b4f8a653ea655240c`，均匹配冻结版。`stage_excerpt.json` 保存原始journal与关键反馈，`verification.json` 保存参数、阶段关联、清理、视频元数据和原文件哈希。未扩展场景变化根因排查或测试矩阵。
