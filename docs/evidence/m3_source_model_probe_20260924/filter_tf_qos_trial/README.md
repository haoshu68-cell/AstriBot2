# scene14 → scene15：附着物过滤确认与动态 TF QoS 试验留档

状态：**DEFERRED_BY_USER / 专项暂停**。用户要求“偶现问题暂过”；2026-09-24 23:17 +08 的原问题计时起点保留。本目录冻结交 rootgit，仅整理已有文件；本次没有修改运行代码、构建、启动 ROS/Gazebo/模型或追加试验。

结论：**scene15 的 413 条 GeometryState 全部为 `GEOMETRY_CURRENT`，仅证明这一轮未复现 scene14 的过滤确认故障。未证明 DDS 根因，未证明完整搬运主线成功。** `.5 s` 接收新鲜度及同版本确认条件均未放宽。scene15 最终因 `PAYLOAD_STATE_STALE` 取消，载荷问题由 M1 负责。

## 来源与文件

原始目录（完整事件流留在原处，不复制）：

- scene14：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene14_world77`
- scene15：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene15_world76`
- 试验配置：`/home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260924/filter_tf_qos_2332`

本目录仅保留本 README 和 [evidence.json](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/m3_source_model_probe_20260924/filter_tf_qos_trial/evidence.json)。JSON 的 `provenance` 保存原始路径、字节数、SHA256 与加载二进制记录；`actual_parameters` 保存服务读回、端点统计、配置差异及探针限制；`metrics` 保存统计窗口与计数；`excerpts` 保存关键行号及时间摘录。完整原始端点与事件数据通过索引回查。JSON 同时保存本 README 的 SHA256。

运行日志是各场景 `stack/session.log`；`m5_observer/events.jsonl`、结果 JSON 与 `highrate_tf_*.txt` 是数据证据，不冒充运行日志。未独立取得的 robot_id、物理标定版本、硬件状态不填造；本报告不涉及真机验收。

## scene14：已确认的现象

`stack/session.log:1364` 在 UTC 15:17:28.018 报告 map → aft_mapped 查询 70.400，过滤节点本地 TF 最新仍为 70.300；独立 M5 已在约 8.10 ms 前收到该边 70.585 的变换。

`:1396` 在 UTC 15:17:30.202 查询 72.400，本地 TF 最新仍为 70.300；独立 M5 已在约 39.76 ms 前收到 72.580 的变换。

M5 中同一边的源时间 69–74 s 有 107 条变换，最大观察者接收间隔 54.603 ms、最大源间隔 55 ms。源端 `stack/simulation/artifacts/highrate_tf_501021_18433264920968.txt` 的 69.9–73.4 s 有 75 条，最大源间隔 55 ms。

几何记录中 97 条 CURRENT、22 条 `ATTACHMENT_FILTER_UNCONFIRMED`；首次过滤未确认在 ROS 70.962，73.180 恢复 CURRENT。父任务以 `GEOMETRY_UNCONFIRMED` 取消，资源释放已确认。scene14 未订阅记录过滤 ACK；JSON 中 ACK 数为 0 表示**未采集**，不能解释成发布数为 0。

由此确认：过滤节点本地 TF 数据落后于独立观察者及源端记录。reliable 接收端重传或排队只是候选解释；没有节点内部 TF 插入时间或 DDS 统计，不能认定传输层根因。

## scene15：配置与窗口结果

私有配置只增加主 `/pointcloud_slice_scan_node` 的动态 `/tf` 订阅 `best_effort`。两场景 `perception_runtime_manifest.json` 记录的节点及两个共享库 SHA 完全一致。观察记录另外增加了 ACK、参数服务与端点快照；这不是证明因果关系的重复对照试验。

`executor_registration/result.json` 记录 `/pointcloud_slice_scan_node/get_parameters` 首次请求成功，读回如下：

| 参数 | 实际读回 |
|---|---|
| `use_sim_time` | true |
| `cloud_pose_frame` | aft_mapped |
| `max_cloud_age_sec` | 0.3 s |
| `tf_timeout_sec` | 0.15 s |
| `tf_total_budget_sec` | 0.3 s |
| `qos_overrides./tf.subscription.reliability` | best_effort |
| `qos_overrides./tf_static.subscription.reliability` | reliable |

本机 Humble 的 TransformListener 允许启动时覆盖动态 TF 的 reliability，rclcpp 声明并在订阅构造时应用该只读参数；对应头文件 SHA 见 JSON 的 provenance。**启动图中没有名为 `pointcloud_slice_scan_node` 的端点**，存在未知节点名，不能把匿名 best-effort GID 认定为该节点。参数服务证据与图身份证据分别保留。

额外图探针因 overlay 覆盖 domain，在 `rclpy.init` 前以 `DOMAIN_MISMATCH` 拒绝，未启动 ROS，不能作为实际端点证据。此限制来自 root 的现场说明与 `candidate.json`；本归档未找到独立原始探针日志。

正式窗口为父目标提交 steady 19244.152138951 至终态 19285.715827682，共 **41.563688731 s**。

| 指标 | 观察结果 |
|---|---|
| 全部几何记录 | 413 / 413 CURRENT，记录覆盖正式窗口前后 |
| 正式窗口过滤 ACK | 787 条；同一 attachment revision |
| ACK 最大观察者接收间隔 | 329.359876 ms |
| 正式窗口 map → aft_mapped | 823 条；最大观察者接收间隔 149.300312 ms |
| 取消前 2 s 至终态 ACK | 44 条；最大观察者接收间隔 100.591282 ms |
| 最后 ACK 至取消请求 | 6.042517 ms |
| 正式窗口已观察 /clock 回退 | 0 |

唯一匹配的过滤无效帧警告在 `stack/session.log:1134`、UTC 15:29:14.914，约早于正式提交 20.848 s。本轮没有记录到 scene14 的 TF 查询失败现象。

首个续租拒绝是 steady 19284.911290887 / ROS **95.363** 的 `PAYLOAD_STATE_STALE`；取消请求为 steady 19285.211384473 / ROS **95.599**。终态 ROS 95.981，`success=false`、`resources_released=true`、`RELEASE_CONFIRMED`、`cleanup_complete=true`。不能把取消时刻当成首个失败时刻，也不能把释放成功当成业务成功。

## 指标复核与边界

1. 从 `full_transfer/result.json.events` 取 parent_submission、cancel_requested、parent_terminal。该文件的 `wall` 字段实际上是主机 steady 秒；不能直接当 Unix 时间。
2. 从 M5 事件流选 `/navigation/attachment_filter_applied`，按 `receive_monotonic_ns` 落入闭区间筛选，再计算相邻接收时间差的最大值；对 `/tf` 只选 frame_id=map、child_frame_id=aft_mapped。源时间间隔另用变换 header.stamp 计算。
3. 413 条几何计数来自全部 `geometry_records`，其范围在 JSON 的 metrics 中单列；并非声称正式窗口恰好有 413 条。几何状态由该节点的 reason 提供。
4. String ACK 没有源时间戳，`source_ns=null`。这里是 **M5 观察者接收间隔**，不是几何消费者内部接收间隔。故只能结合实际 geometry 状态说本轮未复现，不能仅凭 329 ms 证明每个消费者满足 `.5 s` 条件。
5. 警告与正式窗口的近似对齐使用同主机 `started_wall - started_steady`；相关绝对时钟换算不作为精密延迟测量。日志到 M5 的邻近比较使用各自 Unix 接收时间。
6. scene15 观察器 `completed=false`、`interrupted=true`、`writer_dropped=0`，是有界停止且覆盖正式窗口；未完成原定 600 s 采集。没有长时稳定性、丢包因果、接触力学或真机结论。

验收范围是小归档的可追溯性：参数、计数、摘录与源文件校验一致；不是新运行验收。仅在该问题再次阻断主线时由 root 决定恢复，沿用原起点、原始目录与上述窗口，不降低原有新鲜度/同版本门槛；当前不安排新探针或矩阵。
