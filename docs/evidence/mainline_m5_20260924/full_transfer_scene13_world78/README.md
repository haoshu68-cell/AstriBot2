# scene13：夹爪阶段完成且附着生效，任务失败并保留未释放资源

本目录仅核对关闭场景的原始阶段、附着及失败边界。原场景为 `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene13_world78`。未修改原件，未启动 ROS/GPU、新录像或仿真，未解码或查看视频画面。

父任务 task=`fixed_transfer_dfeca583499e49559238c52c5633f08b`，context=`cf1b0542156b4ce49828e35f0ad35f1f`，Goal UUID=`d3b2860887484377957943989b124488`。1361 条匹配反馈均属 PICK；journal 明确确认 PREGRASP、GRASP_APPROACH、GRASP_CONFIRM，18 个控制器子终态成功。

在 ATTACH_CONFIRM 阶段，`payload_records[4230]` 首次报告 accepted=1、applied=1、attached=true；接收 steady=17834.718622614，源时间=106.658 s。首个 MTC_CONTEXT_CHANGED 反馈接收于 steady=17834.745742697、ROS=106.678 s，在上述附着观测后约 27.120083 ms。这里只确认记录顺序，不凭时间相邻推断代码根因。仿真使用 gazebo_kinematic_v1，附着诊断不是接触力抓持或真机抓取验收。

取消请求 steady=17835.238158316，匹配 UUID 的回应 return_code=0；父终态 steady=17844.759409314、ROS=114.030 s：status=6、success=false、resources_released=false，reason=RESOURCE_RECOVERY_REQUIRED:MTC_CONTEXT_CHANGED，cleanup_reason=GEOMETRY_UNCONFIRMED。业务资源处置 UNRESOLVED，业务 cleanup_complete=false，错误为 RESOURCE_RELEASE_UNCONFIRMED；journal 最后 quarantined，没有 RELEASED 提交。原 domain78 隔离状态由场景所有者管理，本次未操作。

末条 ledger 虽已 confirmed=true、ATTACHMENT_CONFIRMED 且含箱，仍不能替代父任务资源释放。外层 `result.json` 的 cleanup_complete=true 仅表示该层进程清理完成，不能覆盖业务未释放。保存的仿真停稳检查通过，同样不等于业务资源已回收。

导航执行、状态、Goal UUID、包络及 ACK 数组均为空；没有完整 PICK、NAV/PLACE 或完整搬运验收，不交付成功演示。

仅复用所有者的完整解码记录：597 解码帧 = 597 sidecar 行 = 597 元数据帧。M5 核对 sidecar 索引连续和元数据数量，没有重复完整解码或关键帧视觉检查。固定 10 fps 对应 59.7 s 播放，录制循环墙钟约 109.902 s；两者不是同一个计时口径。

原始事件摘录见 `stage_excerpt.json`；录像阶段元数据见 `video_metadata_review.json`；来源路径和 SHA-256 见 `source_manifest.json`。以上仅是已保存仿真证据的只读核对，不是当前代码修复验收。
