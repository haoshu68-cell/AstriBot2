# execution_response_scene01：采集链离线核验

输入为 `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene01`。本次仅分析已结束场次的 JSON/JSONL，未启动 ROS/Gazebo/GPU，未再次解码或重新编码视频。视频实际解码数引用 M2 本场 `result.json` 留存结果；本分析独立核验 sidecar、summary、状态引用和时间统计。

**本场没有发出 Action，只支持采集链已接通的证据。** 工装在同步 observer 预检后报 `STALE_LEDGER_CAPTURE`；probe 的 feedback/envelope/ack 均为 0，没有 pending Hold UUID，native 的 865 条录像状态全为 `WAITING_FOR_TASK`。不能将本场称为实际首段运动、Hold 或连续动作录像通过。前置检查顺序由 M2 单独修订，本报告不改该场原结果。

## 逐帧录像记录

| 项目 | 实际结果 |
|---|---:|
| M2 已解码帧 / sidecar 行 / summary 帧 | 307 / 307 / 307 |
| frame_index 与状态引用 | 零基连续；全部实际状态引用与原事件一致 |
| overview 源 stamp 范围 | 23.059–64.200 s |
| 首末帧接收 steady 跨度 | 43.182322473 s |
| 录像循环经过时间 | 44.275690868 s |
| 固定 10 fps 播放时长 | 30.7 s |
| overview 接收间隔 P50 / P99 / 最大 | 107.546 / 528.309 / 608.252 ms |
| overview 源 stamp 最大间隔 | 600 ms |
| overview 接收间隔超过 250 ms | 37 次，仅描述录像观察间隔 |
| overview 源重复 / 回退 | 0 / 0 |
| 编码调用耗时 P99 / 最大 | 5.046 / 6.350 ms |

最大空档为 frame 69→70：源 stamp 26.6→27.2 s，steady 接收间隔 608.251914 ms；不与短 raw bag 进程窗口重叠。源 stamp 间隔也较大，不能只凭这一条把原因归给视频写盘、渲染或 DDS；缺少上游逐环节 trace。`/transport/overview` 是录像相机，不是四路 raw depth。

前 47 个编码帧没有 head 缓存，实际只跨首帧接收后的 0.876762498 秒；不能按 47/10 解释为现实等待 4.7 秒。之后 260 个可见 head 样本均有不同源 stamp，未发现相邻画面复用同一 head 接收缓存或源 stamp 回退。head 缓存从其接收至 overview 接收的 steady 龄期 P99 24.174 ms、最大 36.877 ms；相对 overview 最近 `/clock` 的源龄期为 36–54 ms。这两者定义不同，均不能冒充完整采集到显示时延。

overview 相对最近 clock 的源龄期为 −1 至 54 ms，其中 5 个负样本原样保留；最近 clock 本身的接收滞后最大 19.499 ms。独立消息到达顺序导致该差值不能当作精确同时采样。固定 fps 会压缩或拉长真实间隔，帧数映射一致只证明记录对应关系，不能证明实时连续显示。

recorder 返回 −2，日志明确 `KeyboardInterrupt`；这是 M2 受控 SIGINT 收尾，保留了 finally 的 summary 和逐帧数据，不等于正常跑满录制时限。原始 `stage_frames.frame` 是状态到达时的下一待编码索引；实际画面所用状态以 sidecar 为准。

## 41 话题 observer

采集 manifest 含 41 个订阅，本场 Action 前 receipt 保留进程归属、41 条实际订阅和发布端信息。observer 实际经过 44.631505915 秒，热身 2 秒，分析指标窗口约 42.6315 秒。退出 130、`completed=false/interrupted=true`，写入丢弃为 0，队列采样峰值 17。**完整 300 秒采样状态为 INCOMPLETE，不改为 PASS。**

热身后四路原始深度的现实接收间隔如下，统计来自本场原始 events，分位数为线性插值：

| 原始深度 | 样本数 | P99 | 最大 | 超过 250 ms |
|---|---:|---:|---:|---:|
| head | 737 | 150.033 ms | 210.043 ms | 0 |
| torso | 690 | 160.165 ms | 270.468 ms | 1 |
| left wrist | 668 | 200.548 ms | 250.833 ms | 1 |
| right wrist | 643 | 171.186 ms | 365.507 ms | 3 |

四路深度均无已接收源 stamp 重复或回退，但源 stamp 最大间隔分别为 200、200、250、350 ms，尚不能区分源未产出与途中未收到。当前右腕最大 365.507 ms 属于本场观测，不能与历史 484–582 ms 混成同一试验，也不能据此断言多仿真是唯一原因。

热身后头/腹 CameraHealth 与 ProjectionHealth 均为有效，没有 STALE；该结论不覆盖热身内已记录的异常。腕部两类 health 和 cloud 没有数据，不把 raw 可用升级为腕部处理链健康。完整 16 raw 各流统计、全窗与热身后样本数、启动健康状态和 guard 原字段见 `observer_analysis.json` 及 `observer_summary.json`。latched `/tf_static` 应按全窗样本解读，不能仅因热身后不重发就宣布全场未收到。

`/clock` 热身后最大接收间隔 22.129 ms，无观测到的重复或回退，源推进/steady 接收跨度比为 0.961616；该值是 observer 口径的 RTF，不是 Gazebo 执行器内部性能测量。录像 overview 与 raw depth 的长间隔未表现为整个 observer `/clock` 回调同等长度的停顿，仍不能单靠这一点给出上游唯一根因。

guard 原始字段现已确实落盘，未再只有笼统上层故障字符串。本场 guard 处于未激活状态，不能把 `active=false/healthy=false` 当成执行中的故障。后续实际动作场再关联 context、原始 reason 和控制事件。

## 短 raw bag 与验收边界

七条基线 head 话题均有消息，实际 bag 记录跨度 2.525252108 秒；3 秒 timeout 含启动/发现时间，整个进程含 flush 为 5.538057168 秒，返回 124。专用 cloud 和专用 ProjectionHealth 明确 `NOT_ACTIVATED`。本次未逐条解 CDR 做同刻 tuple 核验，不能将七条存在性宣称为完整 M3 输入或未来模型输入。

本场已证明逐帧源时间记录、状态引用、guard typed 消息和 observer 接线能在现场产出可分析数据；**仍未通过完整时长、全部接收间隔预算、实际运动或 Hold 验收**。下场必须保留相同口径，并只对实际执行窗口评价图像覆盖。

复现本场轻量 sidecar 统计：`python3 -B docs/evidence/mainline_m5_20260924/execution_response_scene01_capture/analyze_video_sidecar.py`。输入哈希与精确统计见 `video_sidecar_analysis.json`；完整 observer 分析的退出码非零来自 INCOMPLETE，不是把中断包装为成功。
