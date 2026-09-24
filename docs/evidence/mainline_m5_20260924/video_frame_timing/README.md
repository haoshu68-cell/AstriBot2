# 录像逐帧时间证据：最小改动冻结

2026-09-24，用户恢复任务后完成冻结；未启动 ROS、Gazebo、GPU 或真实动作。修改仅为 `tools/sim/record_transport_demo.py` 和新增定向离线测试，保留修改前已有的 native_hold、标题、task_id 等功能。未提交。

## 已实现与验证

- 每个 overview 编码调用对应 `.frames.jsonl` 一行，保留原 header stamp/frame、回调入口 steady/wall、编码调用前后 steady。
- head 像素与源/接收时间一起缓存，复用时不会给旧图像刷新时间。
- 每次状态回调保留完整 raw_status、独立本地 receive_seq 与接收时间；逐帧引用实际绘制的最近状态。没有新图像的阶段变化不会补造帧。
- 显式订阅 `/clock`，保留最近接收的 clock 值及其接收时间。没有收到时为 null；零源 stamp 仍原样保留为 0，不补成当前时间。
- 固定 10 fps 仅为播放；没有插帧。OpenCV write 无成功帧数返回，summary 明确其计数只是完成的写入调用。

暂停前 4 项定向离线测试通过：native 多状态/旧 head 复用/源 stamp 重复与回退，legacy 与零源 stamp，回调错误保留既有证据，中断传播与 finally 保留既有证据。测试执行真实 OpenCV 编码和逐帧解码，替换 ROS 回调输入，没有真实 DDS。测试日志 `tests_green.txt`；恢复后源码未改，没有重复跑这四项测试。

恢复后完成 Python AST、diff 检查，以及精确补丁在临时目录 apply/check/reverse 的逐字节往返。独立代码审查单独记录在 `independent_review.json`。

## 精确版本

| 项目 | SHA256 |
|---|---|
| 修改前候选录像脚本 | `219904f2a6d3ba3804eddba2d647604651e50f8103abfb5a22819c6d7d78a79e` |
| 冻结后录像脚本 | `dc19af167a6c2e8583df95f43283470714d73901137f95f724532a9ee461e92f` |
| 新定向测试 | `fd4bbcfce185234d3237a27e1bb1b85e1b5b9bd9eb16d1bcb1c1ce120ebef857` |
| 本轮精确补丁 | `96d48003895840c546a1b4ac6a394647566143557ced8f85aa433e7eb4af0bb0` |

`record_transport_demo.before.py` 原样保留前场分析引用的候选代码，不能用当前不断演进的源码路径替代该历史版本。`record_transport_demo.after.py` 是冻结副本。`record_transport_demo.sidecar.patch` 只包含 before→after 与新测试；没有把修改前已有的他人改动纳入本轮。

## M2 接线与本场验收

现有命令行无需改变。M2 使用已冻结源码，留存实际脚本/hash、进程 PID/start_ticks/boot_id、会话、输出路径、退出码及日志。三个对应文件为 `first_stage.mp4`、`first_stage.frames.jsonl`、`first_stage.json`；动作/取消/资源收尾仍由原所有者负责。

结束后必须实际逐帧解码，并核对 `decoded_frames == sidecar_rows == summary.frames > 0`，frame_index 严格为零基连续编号；任何不一致为证据失败。实际 source/receive 时间可分别计算重复、回退、接收空档和旧 head 缓存时间。画面收到以前的源延迟不能仅凭这些回调时间归因到渲染、bridge 或 DDS。

录像脚本的 summary 没有独立终态成功字段，必须结合 runner 的原始退出码和日志；中断时留下文件不代表完整运动成功。真实 DDS、SIGTERM/强杀路径和现场负载本轮未验证；强杀可能丢失尚未写入 summary 的状态事件。完整 `/clock` 时序由 M5 observer 保留，视频侧只取最近观察值，不能单独证明期间没有 clock 回退。

本次静态审查允许进入下一次已授权的有界采集，**不等于现场录像、动作或 Hold 已通过**。前场缺逐帧时钟的旧影片仍只能支持已报告的前后/短动作证据，不能由本次代码改动补齐旧数据。
