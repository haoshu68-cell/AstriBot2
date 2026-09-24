# scene03_world91：首段闭环后的录像与相机观测

本场输入为 `/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91`，是**新 world/domain 91**。scene02/domain90 的 ACK 工装失败、隔离与释放未证记录保持不变；本场成功不证明旧 domain90 已恢复。

M2 本场记录首段 PREGRASP→Hold、六消费者 ACK、正常取消、资源释放与停稳通过，parent terminal 明确 `resources_released=true`，`resource_disposition=RELEASE_CONFIRMED`。这是限定首段链结果，不是完整 PICK/PLACE、零位到 READY 或 M3 模型验收。Action/ACK/释放的完整独立归档由总调度负责，本报告重点核验录像 sidecar 与 41-topic observer，避免重复深挖。

[原始视频](/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91/first_stage.mp4) · [M2 场次结果](/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene03_world91/summary.json)

M5 本次只处理已有 JSON/JSONL；引用 M2 已执行的 515 帧解码核验，没有再次全片解码、提取新图、重编码、运行 ROS/Gazebo/GPU。

## 录像记录与实际覆盖

| 项目 | 结果 |
|---|---:|
| M2 实际解码 / sidecar / summary | 515 / 515 / 515 |
| M5 独立元数据检查 | 索引连续，逐帧状态引用完全对应原事件 |
| 录制循环现实时间 / 固定 10 fps 播放时间 | 77.973 s / 51.5 s |
| 首执行→首 Hold 反馈，ROS 时间 | 59.023→71.419 s，12.396 s |
| 相同反馈窗口，steady 时间 | 82893.838213904→82906.853869587 s，13.015655683 s |
| 动作窗接收的 overview 帧 | 66 帧，frame 283–348 |
| 按源 stamp 严格位于动作窗 | 65 帧，源 59.1–71.1 s |
| 动作窗接收间隔 P99 / 最大 | 644.503 / 863.093 ms |
| 动作窗源 stamp 最大间隔 | 800 ms |
| 动作窗接收间隔 >250 ms | 13 次 |
| 首执行反馈至首个图像接收 | 28.207 ms |
| 最后窗口内图像接收至首 Hold 反馈 | 289.938 ms |
| 取消反馈后仍有视频接收 | 23.407 s |

动作窗有一张在执行反馈后接收、但源 stamp=59.000 s 的图像，因此接收窗口与源窗口相差 1 帧。最长空档为 frame 335→336：源 68.1→68.9 s，现实接收 82903.424631906→82904.287725216 s。66 帧按 10 fps 播放仅为 6.6 秒，**不能称为 13.02 秒动作的实时连续完整录像**。本次没有通过插帧隐藏间隔。

动作窗全部 66 帧有 head 缓存，没有相邻帧复用同一 head 接收缓存；steady 缓存龄期 P99 23.494 ms、最大 23.737 ms。相对 overview 最近 `/clock` 的 head 源龄期为 37–54 ms。缓存龄期与源龄期不是同一时钟口径，也不代表完整采集到显示时延。动作窗编码调用最大耗时 5.056 ms；仅凭这些时间仍不能唯一归因上游渲染/桥接/DDS 空档。

全片 overview 最大接收间隔 985.661 ms，源 stamp 最大间隔 900 ms，源重复/回退均为 0。全片开头缺 head 的帧与动作窗分开统计；完整指标及每个长间隔端点在 `video_metadata_analysis.json`。

左臂动作窗有 581 个 JTC 样本，最大实测速度 0.322530528 rad/s、最大跟踪误差 0.032265251 rad，最后样本 ROS 71.400 s 的最大误差为 1.2227×10⁻⁶ rad。JTC 与反馈证明层和视觉逐帧覆盖层分别保留，单个末样本不是持续停稳验收；停稳结果使用 M2 独立窗口记录。

## 动作窗口内原始深度

以下只统计首执行反馈到首 Hold 反馈之间接收到的相邻深度样本，边界静默与跨边界间隔另存 JSON：

| 深度流 | 接收 P99 | 接收最大 | >250 ms | 源 stamp 最大间隔 |
|---|---:|---:|---:|---:|
| head | 158.623 ms | 162.486 ms | 0 | 150 ms |
| torso | 160.044 ms | 187.056 ms | 0 | 150 ms |
| left wrist | 213.102 ms | 281.427 ms | 1 | 250 ms |
| right wrist | 205.257 ms | 214.292 ms | 0 | 200 ms |

左腕仍有一次超过 250 ms 的接收间隔；不能因为控制链 PASS 就把图像时序预算也记为 PASS。头/腹两类 health 在动作窗内全部 `OK/READY`，各自 epoch 固定；腕部处理 health/cloud 未提供对应数据，raw 可见不等于处理链验收通过。

完整 observer 持续 78.127651064 秒，退出 130，`completed=false/interrupted=true`，因此完整 300 秒采样仍为 **INCOMPLETE**。热身后 76.127651064 秒内四深度最大接收间隔分别为 272.870、302.937、281.427、238.988 ms，超过 250 ms 的次数为 1、1、3、0，不能用动作窗较短的数值覆盖全场尾部和静止样本。全窗、热身后、动作窗三组结果见 `observer_summary.json/.md`。

41 条配置中，全窗 35 条有样本、热身后 34 条；`/tf_static` 的 10 条启动缓存已保留，热身后不重发不表示全窗丢失。只有左右腕各自的 CameraHealth、ProjectionHealth、处理 cloud 共 6 项全场缺失。writer 丢弃 0、采样队列峰值 18，不等于 DDS 内部队列没有积压。启动期出现的 STALE/历史 epoch 保留在全窗统计中，没有作为正常动作窗样本删除。

热身后 `/clock` 无观测到的重复/回退，源推进与 steady 接收跨度比为 0.962983；该值是 observer 口径，不是 Gazebo 内部执行器指标。原始传输中的遗漏与源本身未产出不能只凭已接收 stamp 间隔区分。

## 原始 guard 与控制证据边界

动作窗 guard 共 652 条，reason 均为 `EXECUTION_WITHIN_BOUNDS`；其中 647 条 active/healthy 为 true，末尾 5 条均为 false，最大关节误差 0.032265251 rad。首次 inactive 为 source 71.354 s、接收 steady 82906.788152627 s，比首 Hold feedback 早 65.717 ms。保留原字段，既不把 reason 当全窗 active 健康，也不把结束过渡直接当新故障。

M2 的 424 条 ACK 跨越 Hold、取消和释放后多个阶段，不能将总数直接当作同一有效持有窗口的完整 ACK 集。`summary.json` 名为 `typed_hold` 的对象实际载荷为 held JointState，不能冒充原始 ArmHoldStatus 消息；同一 envelope/epoch/context 与有效时限的最终 ACK/typed状态论证以总调度独立归档为准。本报告不另造或补填缺失消息。

新场控制链的通过与录像间隔、深度超限、未完成的完整 PICK/PLACE/M3 是并列结论。新旧场的轨迹独立重规划，也不是只改变 time scaling 的严格 A/B。

## 产物

- `video_metadata_analysis.json` / `analyze_video_metadata.py`：无解码的复现分析、源文件哈希、两种动作窗口及逐阶段时间。
- `observer_analysis.json` / `observer_summary.json/.md`：41 订阅各流全窗、热身后、动作窗时序和健康原字段。
- `verification.json`：输入和派生产物哈希核对；不将 interrupted observer 升格为完整采样通过。

此前两场报告保留在同级 `execution_response_scene01_capture` 与 `execution_response_scene02_motion`，没有回写旧场结果。
