# scene34：PICK 后缀交接已实测，TRANSPORT 中途取消

原始目录：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/full_action_wiring/full_transfer_scene34_world57`，domain 57。

本次仅离线读取已关闭场景的结果、journal 和控制器状态，生成证据摘录；没有启动 ROS/GPU/build、修改产品或夹具、解码视频。冻结验证器仍为 `40e36820d4398e62d22c27f0426a9f3fea2db3f070e8cdf1955d259a0a393eb9`，与本场使用副本一致。

## 已确认的实际进展

PICK 初始三个阶段 PREGRASP、GRASP_APPROACH、GRASP_CONFIRM 已确认完成。随后 ATTACH 的实际负载场景提交、后缀采纳、事务确认依次完成。采纳事件明确 `transport_replanned=true`。两个后缀阶段均满足 returned_digest = adopted_digest = 最终 goal 的 sent_digest，点数与有序关节名也完全一致；response 保留同份不可变发送元数据。

以下索引为原始 `full_transfer/result.json` 的 `executor_journal` 零基索引。全部 118 条事件具有相同 task owner、lease ID 与 epoch。

| 阶段 | 关联事件 | 已核实结果 |
|---|---|---|
| ATTACH 事务与采纳 | 70 physical_submission → 71 physical_applied_scene_submission → 72 payload_suffix_adopted → 73 payload_transaction_confirmed | 同 context/transaction，command_id=1，采纳完整 index 4/5 后缀 |
| LIFT，index 4，generation 5 | 75 send → 81 response → 87 terminal → 93 stage_confirmed | 11 点、7 关节；同 UUID 成功码4，阶段确认包含该 UUID |
| TRANSPORT_POSTURE，index 5，generation 6 | 96 send → 102 response → 108 stop_requested → 110 terminal | 839 点、7 关节；同 UUID 被接受后取消，码5、success=false；没有阶段完成确认 |

LIFT UUID：`04c749360b62a0a26089c775cbe0d3f4`。

LIFT 三环节共同摘要：`96419082bc629ebb4329c5cdc53cd2b658de1fe202eb8ea89b4b3f01fe1237c0`。

TRANSPORT_POSTURE UUID：`89f0cc03a0ca057c1da216dcbf3c420d`。

TRANSPORT_POSTURE 三环节共同摘要：`da7e473d0de82dd9ad2c5911b6adcc8c0ee667bc9fe1bcf89f3da0553d1f538b`。

这证明实际负载重规划产物已进入执行器并作为最终控制器目标发送，且 LIFT 完成。TRANSPORT_POSTURE 已进入执行但中途停止；不能宣称 TRANSPORT 到位、完整 PICK、NAVIGATE、PLACE 或整场搬运通过。PLACE 未到达。

## 同期关节运动旁证

在父反馈进入 EXECUTING_MTC_STAGE:TRANSPORT_POSTURE 至首次 MTC_BASE_MOVED 的接收窗口内，共保存 319 条左臂控制器状态。源时间覆盖 ROS 103.39–110.02 s，接收 steady 40796.922267590–40805.782108741。七关节 actual 位置范围约为 `[1.00180, 0.40371, 0.98761, 0.12426, 0.81160, 0.05814, 0.40341]` rad，确认同期存在仿真关节运动。

这些状态消息没有 goal UUID，也不包含完整目标轨迹；它们不能独立绑定该 UUID、重建全轨迹或计算完成比例。原始字段保存在 `transport_jtc_excerpt.json`，摘要见 `verification.json`。控制器成功/取消结果与阶段确认才用于本次阶段完成判断。

## 失败与清理边界

首次 TRANSPORT_POSTURE 执行反馈为 `feedback_records[1084]`，接收 ROS=103.386 s、steady=40796.917283426。停止原因首次出现在 `[1261]`，ROS=110.021 s、steady=40805.784542112，reason=MTC_BASE_MOVED。此前 ATTACH 期间的 GEOMETRY_UNCONFIRMED 是恢复后继续进入重验的过渡状态，未替换本场最终停止首因。journal 的 issued_at 按原样保留，不解释为各事件独立采样时间。

验证器原错误仍是 `RENEW_REJECTED:MTC_BASE_MOVED`，成功路径的完整 PICK+PLACE 交接验收未进入。本归档是针对已产生事件的独立离线部分核验，不把整场结果改为通过。

父终态 status=5、success=false、resources_released=true；RELEASE_CONFIRMED，cleanup_complete=true、无 cleanup_errors。清理停车证据为 36 条样本/0.7 s，记录平移漂移约 7.53 µm、旋转约 18.28 µrad，命令最大值0。关闭后的会话 state=stopped、remaining_owned_pids=[]。这些只证明本轮清理结果，不消除执行中 MTC_BASE_MOVED 的失败。

未在本任务中分析底盘移动根因、改变保护阈值或复测。原始文件未修改；`stage_excerpt.json`、`transport_jtc_excerpt.json`、`verification.json` 与清单构成本次证据。只读同行复核得到相同的链路和完成边界结论。
