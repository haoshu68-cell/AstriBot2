# scene12：GRASP_CONFIRM 端点未达，取消后资源释放

本目录仅整理已关闭场景的原始阶段和失败边界，不修改原件，不启动 ROS/GPU，不解码或检查视频画面。原始场景为 `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene12_world79`。

父任务 task=`fixed_transfer_1d0eb66534e84cf78f7862e84d7b3b57`，context=`7d3d5c39e00d491480cde56830c03f8f`，Goal UUID=`273261f09b914d6982e4f9313f40a03f`。一次提交、966 条匹配反馈，全部处于 PICK。Journal 确认 PREGRASP、GRASP_APPROACH；GRASP_CONFIRM 已开始但未确认完成。18 个控制器子结果成功不能替代阶段端点确认。

首个 native `MTC_ENDPOINT_NOT_REACHED` 反馈位于 steady=17280.238082416、ROS=102.490 s，早于探针取消约 445.520 ms。同期夹爪 JTC 实际值 0.4275420808 rad，参考值 0.4663773148 rad，误差 0.0388352340 rad；这些字段支持端点未达，不能独立证明具体预紧参数是成因。随后续租拒绝，探针记录 `RENEW_REJECTED:MTC_ENDPOINT_NOT_REACHED`。

取消请求 steady=17280.683602117，匹配 UUID 的回应 return_code=0；父终态 steady=17280.943336534，status=5、success=false、resources_released=true，保留 native 失败原因。Journal 最终提交 RELEASED，side_effects=false；资源处置 RELEASE_CONFIRMED。清理停稳记录 ROS 103.12–103.82 s，共 36 个样本，测得速度及漂移为零，观察到零指令。以上为保存的仿真证据，不是真机停车验收。

972 条物理诊断均 accepted=0、applied=0、attached=false，未显示物理附着执行；导航 UUID、执行和 ACK 记录为空，没有 NAV/PLACE 验收。不能据此交付完整搬运演示。

仅复用场景所有者的完整解码记录：818 解码帧 = 818 sidecar 行 = 818 元数据帧。M5 独立核对 sidecar 索引连续和元数据数量，不重复解码，也未做视觉验收。固定 10 fps 对应 81.8 s 播放，录制循环墙钟约 104.715 s；播放长度不是实际动作耗时。已记录 GC 最大 62.170472 ms，仅作为原始字段保留，未进行性能归因。

原始事件摘录见 `stage_excerpt.json`；录像阶段与帧数核对见 `video_metadata_review.json`；来源路径和 SHA-256 见 `source_manifest.json`。业务结果为失败，资源释放通过；视频文件可解码不等于任务通过。
