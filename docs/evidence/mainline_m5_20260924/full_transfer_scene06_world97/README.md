# scene06：父任务拒绝，录像与阶段证据复核

本场 `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene06_world97` 已结束并完成归属清理。准备场景通过，但唯一正式父任务被拒绝：`PARENT_REJECTED`、`NO_ACCEPTED_PARENT`。**未进入 PICK、NAVIGATE 或 PLACE，不是完整抓取搬运放置通过。**

## 视频与画面

原视频 [full_transfer.mp4](/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene06_world97/full_transfer.mp4) 已在文件关闭后独立用 CPU 完整解码一次：149 帧 = sidecar 149 行 = 元数据 149 帧，索引连续，且与 M2 的解码结果一致。分辨率 1280×720，10 fps，播放 14.9 秒；录制循环耗时 23.330412 秒，两者不可互换。未转码、补帧或重录。

| 实际帧（从 0 开始） | overview 源时间 | 与父任务提交的接收时间关系 | 可见内容 |
|---|---:|---|---|
| [0](frame_0000.png) | 29.601 s | 提交前 | 初始准备画面；尚未收到 recorder 状态或 head 小窗 |
| [144](frame_0144.png) | 48.000 s | 提交后约 18.140 ms | 橙色箱体仍在原台面；字幕显示最近收到的 WAITING_FOR_TASK，不能据此推断 acceptance 时刻 |
| [148](frame_0148.png) | 48.500 s | 提交后约 567.272 ms | 箱体仍可见，字幕为 STATION_WORLD_OBSERVATION_STALE，phase=0、hold_confirmed=false |

抽查的箱体位置不被右下 head 小窗或顶端字幕遮挡；head 小窗主要显示通道，不能替代主画面的箱体证据。前 24 帧没有 head 图像，之后小窗是缓存帧而非与 overview 同步，样本最大缓存接收龄期约 287.901 ms。三帧视觉核对不能外推为未执行的导航路径和放置动作均可见，也不构成相机性能验收。

## 正式阶段及物体证据

父任务仅提交一次，UUID `8c850fbd8190436ca2263ab4f8d5a3c6`，steady=1936.623297659 s、ROS=47.950 s；见 [phase_audit.json](phase_audit.json)，保留原结果中的 task/context/object 关联。没有保存独立 acceptance response 或拒绝时刻，拒绝结论来自客户端 `PARENT_REJECTED` 和 `NO_ACCEPTED_PARENT` 结果。

父 feedback、导航执行记录/Nav UUID、envelope/ACK、renew 均无样本。录像状态有 WAITING_FOR_TASK 424 条、STATION_WORLD_OBSERVATION_STALE 24 条，均为 IDLE/phase=0、空 lease/hold ID。首次 STALE 接收在提交后约 31.574 ms；这是同期状态证据，本复核不定位其内部成因。没有 attach 成功、携物导航、detach、最终放置或已接受任务的 cancel/release 验收。初始空库存不能冒充放置后的 EMPTY。

观察器覆盖 24.026999 秒，所有者中断退出 130，writer_dropped=0、queue_peak=14。既有分析器正确返回 1 并标记 `INCOMPLETE`，不重标为完整 600 秒 PASS；见 [observer_analysis.json](observer_analysis.json)。根结果确认清理完成、无剩余自有进程；清理成功不改变父任务拒绝结论。

本轮仅读取已结束素材、运行既有离线分析器、解码并抽取三帧；未启动 ROS、GPU、第二场、第二路录制或性能矩阵。原始素材与源码未修改。各输入和产物哈希见 [video_audit.json](video_audit.json) 及 [verification.json](verification.json)。
