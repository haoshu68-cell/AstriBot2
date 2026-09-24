# execution_response_scene02：实际首段动作、录像与相机观测

本报告基于 `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02` 的已结束场次。M5 只做离线 JSON/JSONL 分析和一次必要的原片解码，提取 6 张未经修改的关键帧；没有重新编码视频，没有启动 ROS/Gazebo/GPU。

**观察到实际首段运动并到达 `FIRST_STAGE_HOLD_CONFIRMED`，但本场整体未通过。** 之后工装读取 `EnvelopeApplyStatus.mode` 抛异常，取消、停稳、资源释放没有完成验收。进程层面的所有者 teardown 完成，不能替代业务层释放 ACK。首段 Hold 也不是完整 PICK、搬运或放置通过。

[原始视频（保留原片）](/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02/first_stage.mp4)

## 实际动作与视频覆盖

执行反馈从 ROS 70.704 s 到首次 Hold 79.029 s，共 8.325 秒仿真时间，对应 steady 82289.863465815–82298.966628639 s，即 **9.103162824 秒现实时间**。本场 6 个 child 控制目标有成功终态；它们不是 6 个包络消费者 ACK。详细原始事件、UUID 和 guard 审计见 `action_guard_audit.json`。

| 录像指标 | 本场结果 |
|---|---:|
| 独立实际解码 / sidecar / summary | 383 / 383 / 383，索引连续、状态引用一致 |
| 录制循环现实时间 / 固定 10 fps 播放时间 | 63.685 s / 38.3 s |
| 执行反馈窗口内接收图像 | 39 帧，frame 336–374 |
| 严格按图像源 stamp 位于执行窗口 | 38 帧，源 70.8–78.9 s |
| 动作窗 overview 接收间隔 P99 / 最大 | 1138.131 / 1233.580 ms |
| 动作窗 overview 源 stamp 最大间隔 | 1000 ms |
| 动作窗 overview 接收间隔超过 250 ms | 8 次 |
| 首次执行反馈至首个窗口内图像接收 | 42.859 ms |
| 最后窗口内图像接收至首次 Hold 反馈 | 89.518 ms |
| 动作窗可见 head 缓存龄期 P99 / 最大 | 24.876 / 25.081 ms |
| 动作窗编码调用最大耗时 | 4.809 ms |

39 个接收帧中有一帧源 stamp 为 70.700 s，比执行反馈的 70.704 s 早 4 ms，因此严格源窗口为 38 帧。这正是源采集时间与回调接收时间不能混用的例子。

最长间隔在 frame 368→369：源 stamp 76.4→77.4 s，接收 steady 82296.014097050→82297.247676790 s。39 帧在固定 10 fps 下只播放 3.9 秒，**不能称为这段 9.10 秒运动的实时连续完整录像**。原片确实采样到动作过程和 Hold 后姿态，仍有可量化的画面空档。overview 是额外录像相机，不能将其 1.23 秒空档当成四路原始深度的接收间隔。

动作窗内 head 每帧有缓存，未见相邻图像复用同一 head 接收；相对最近观察 `/clock` 的 head 源龄期为 40–54 ms。steady 缓存龄期与 ROS 源龄期定义不同，也都不是完整采集到显示延迟。全片开头有 head 复用及无状态阶段，不能拿全片统计代替实际动作窗口。

查看关键帧可见机械臂从初始姿态移动到目标附近的 Hold 姿态；不从图像断言物体已抓取或接触力已验收：

- [执行前 frame 335，源 70.6 s](frame_335.png)
- [执行中 frame 363，源 74.6 s](frame_363.png)
- [Hold 后 frame 382，源 80.3 s](frame_382.png)

动作窗口内左臂 JTC 有 382 个样本，实测最大单关节位置范围 1.186596635 rad，最大速度 0.320546728 rad/s，最大跟踪误差 0.032049613 rad。末样本 ROS 79.020 s 的最大跟踪误差 1.4078×10⁻⁶ rad、最大速度 1.5642×10⁻⁵ rad/s。该末样本支持末端趋于稳定，不能代替故障后持续停稳、取消完成和资源释放证明。

## 动作窗口内四路原始深度

下表仅使用执行反馈窗口内接收到的相邻样本；跨边界间隔与首末静默另存 `observer_summary.json`，没有用全场最大值冒充动作窗最大值。

| 深度流 | 样本 | 接收 P99 | 接收最大 | >250 ms | 源 stamp 最大间隔 |
|---|---:|---:|---:|---:|---:|
| head | 142 | 184.068 ms | 263.180 ms | 1 | 250 ms |
| torso | 138 | 178.330 ms | 197.845 ms | 0 | 150 ms |
| left wrist | 131 | 249.710 ms | 264.959 ms | 2 | 250 ms |
| right wrist | 126 | 239.795 ms | 369.341 ms | 1 | 300 ms |

四流已接收源 stamp 无重复/回退；源 stamp 空档仍不能独立区分相机未产出与途中未收到。头/腹 CameraHealth 在动作窗内各 182 条全 `OK`，ProjectionHealth 各 455 条全 `READY`，窗口内 epoch 未变；腕部两类 health/cloud 全场无数据，不能据 raw 数据推成处理链有效。

全场含启动阶段时，四深度最大接收间隔分别为 844.834、343.845、264.959、796.765 ms；排除前 2 秒后为 263.180、284.013、264.959、430.805 ms。启动期有 STALE 和多个健康 epoch，原样保留在完整统计中；收到 transient-local 历史缓存不能单独证明本场发生源重启。动作窗较短，不能用较好的动作窗数值隐藏这些全场样本。

完整 observer 运行 64.288336611 秒，退出 130，INCOMPLETE，写入丢弃 0、队列峰值 19。41 条配置，全窗 34 条有数据，热身后 33 条；`/tf_static` 启动缓存 8 条仍保留。除腕部 6 项外，`/cmd_vel_policy_input` 全场无数据。热身后 `/clock` 最大接收间隔 59.549 ms、无重复/回退，源推进/接收跨度的 observer RTF 为 0.944785。该受控短窗不等于 300 秒完整性能验收。

## Guard 与失败后的边界

动作窗内 guard 共有 456 条，原 reason 均为 `EXECUTION_WITHIN_BOUNDS`，最大关节误差 0.032049613 rad；其中 453 条 active/healthy 为 true，末尾 3 条均为 false。第一条 false 的 data.stamp=78.976 s、接收 steady=82298.913337903 s，比首个 Hold feedback 接收早约 53.291 ms。因此不能只看 reason 文本宣称全窗 guard 都处于活动健康状态，也不能只凭 inactive 字段把结束过渡说成新的执行故障。原始字段和执行结束时序见独立 action/guard 审计。

本场 probe 的 `acks=[]`，且 observer 清单没有订阅包络消费者 ACK 话题。没有证据支持“6 ACK 通过”。`EnvelopeApplyStatus` 缺少工装所读取的 mode 字段，异常又打断 cleanup。外部所有者日志出现后续 `RESOURCE_LEASE_EXPIRED` 隔离，未提供 release 记录；这是与首段成功并列的失败结果，不删除或改写。

接收间隔预算仍有超限，录像仍有长空档，且取消/释放尚未证明。本场新增的有效证据是：实际首段运动、首段 Hold 反馈、对应 JTC、可定位的图像采样及原始 guard 状态。M2 记录本场 OMPL 轨迹与前场不同，不能把误差改善量归结为同一路径仅改变 time scaling 的严格 A/B。后续修复和新场验收由控制/运行所有者负责，不通过重复编码或补帧掩盖缺口。

## 文件与口径

- `motion_video_analysis.json`：源输入哈希、两种动作窗口、逐帧覆盖、JTC、关键帧哈希；对应 `analyze_motion_video.py` 仅执行过一次解码。
- `observer_analysis.json` / `observer_summary.json`：全窗、热身后及动作窗各流时序、health/epoch、guard 原始统计。
- `action_guard_audit.json`：独立动作/child/ACK/cleanup 审计。
- `verification.json`：本目录结果与输入哈希核验，不能替代现场通过。
