# scene09：部分 PICK 执行后取消，资源释放成功

本场实际接受唯一正式父任务，task=`fixed_transfer_7ba565080db04532b2cb9beba7f4037e`，context=`70d8234c6317413480ebf2f21d1d25d6`，Goal UUID=`5bab2ddfb0044fdba5872261edeb3c9c`。进入 PICK 的 PREGRASP、GRASP_APPROACH、GRASP_CONFIRM；探针报告 `ODOM_STALE` 后取消。**整项任务未成功，未进入 NAVIGATE、PLACE，不作为完整演示。**

本次按轻量范围复用 M2 的完整解码结果：原视频 548 帧 = sidecar 548 行 = 元数据 548 帧。M5 独立核对索引、逐帧状态与 recorder 原始事件一致，只解码并查看以下五帧，不重复完整解码、转码或分析相机性能。原视频为 [full_transfer.mp4](/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene09_world100/full_transfer.mp4)，固定 10 fps 播放 54.8 秒，录制循环墙钟约 94.026 秒。

| 帧号（从 0 开始） | overview 源时间 / s | 接收 steady / s | 原始阶段及实际画面 |
|---|---:|---:|---|
| [315](frame_0315.png) | 66.8 | 3158.660627 | 规划后、伸臂前；箱体仍在台面 |
| [367](frame_0367.png) | 77.1 | 3170.370440 | PREGRASP，左臂已伸出，夹爪在箱体上方 |
| [396](frame_0396.png) | 81.1 | 3174.848780 | GRASP_APPROACH，夹爪接近箱体 |
| [423](frame_0423.png) | 85.6 | 3180.118234 | GRASP_CONFIRM，夹爪位于箱体旁并较前收拢；尚无物体附着或抬离证据 |
| [547](frame_0547.png) | 106.5 | 3206.742348 | TASK_CANCELED / phase=0，手臂停留在箱体附近，物体仍在原台面 |

上述关键位置在主画面中可见，未被右下小窗或顶端字幕遮挡；接触细节部分由夹爪遮挡。头部小窗被手臂大幅遮挡，不能独立提供完整抓取视图。抽查五帧不是全帧视觉验收，也不能用闭爪画面宣称附着、接触力抓持或 PICK 完成。

903 条父反馈全部为 PICK。执行阶段首次反馈 steady 分别为 PREGRASP=3159.024668262、GRASP_APPROACH=3172.724103041、GRASP_CONFIRM=3177.199980428；反馈观察时间不是精确物理端点。Journal 仅确认 PREGRASP 和 GRASP_APPROACH 两阶段完成；虽有 18 个成功的控制器子结果，GRASP_CONFIRM 未确认，不能拼成 PICK 完成。

取消请求 steady=3180.162622083，响应 3180.173191629，匹配本父 UUID、return_code=0。父终态 steady=3180.928736857：status=5、`TASK_CANCELED`、`resources_released=true`；journal 最终 `resource_handoff_committed / RELEASED / side_effects=false`。清理停稳记录 ROS 86.3–87.0 s、36 样本，速度/漂移为零且观察到零指令。最后视频帧接收比父终态晚约 25.814 秒，覆盖取消后的观察尾段；资源释放结论来自 Action/journal，而非字幕。

物理原始诊断未出现 attach 成功，源库存与 ledger 仍为空；没有导航 UUID、NAV/PLACE 阶段或最终放置证据。`ODOM_STALE` 在此只记为探针触发的取消原因，本轮不重新归因仿真、GPU 或展开性能检查。

完整原始事件与逐帧对应、来源哈希见 [review.json](review.json)，归档校验见 [verification.json](verification.json)。本次未启动 ROS/GPU、新录像或第二场；原始素材和已验录制器未修改。
