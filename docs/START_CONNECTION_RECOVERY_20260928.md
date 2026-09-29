# 工位候选路径起步连接失败恢复

用户于 2026-09-28 批准：实际起点通过后，候选路径起步连接失败，也应结束当前导航跟踪，进入脱困，到达安全区域后重新规划。

## 已实现的数据流

```text
EnsureNavigationStart 检查当前实际位姿
  → READY
  → ComputePathWithRecovery 请求 PlanCandidate.INITIAL（保留原工位目标）
      ├─ 有效路径 → 原 FollowPath
      ├─ 其他规划失败 → 明确失败并走原搬运收尾
      └─ START_CONNECTION_BLOCKED + 实际候选首朝向
          → 清空旧路径，结束旧 FollowPath，SLAM 新样本确认停稳
          → PlanStartRecovery：当前位姿、分层地图、包络、二维代价图
          → 选择安全短平移及安全出口朝向
          → FollowPath(Departure) → 实测停稳
          → AssessNavigationStart 重新检查实际位置
              ├─ READY → INITIAL 从新实际位置规划原目标
              ├─ RECOVERY_REQUIRED → 再取当前数据并选择短动作
              └─ BLOCKED / UNAVAILABLE → 明确失败
```

“结束导航”在这里指结束失败路径及其跟踪阶段；恢复过程中保持同一个上层导航目标和搬运事务，避免先上报搬运失败、释放资源，再启动后台运动。没有新增速度发布者。规划器报告有类型的首连接失败，不从日志文本推断，不把所有无解都归入脱困。

候选搜索仍沿用后、左、右、前及最短安全出口规则，每次执行至多 0.2 m；不写死后退 10 cm。仅在实际首连接失败时使用返回朝向检查出口旋转扫掠。普通起点检查仍只判断当前实际姿态。恢复最多 10 个短段 / 120 秒，时间从首次首连接失败开始，同一执行不因 halt 或重新规划重置。

普通 Smac 搜索、MPPI、ThreePhase 算法不改；工位 BT 改为调用新增 INITIAL 模式。`idle_position_hold=true`、`idle_position_kp=3.0` 不改。停稳使用 `/slam/pose` 的有符号净变化，无里程计回退，也没有新增消息年龄门槛。

## 验证分层

| 层级 | 结果与边界 |
|---|---|
| 构建 | 新消息、规划库、恢复库及 BT 插件已在私有安装目录构建成功 |
| 正式恢复 CTest | departure_path、navigation_start_assessment、start_connection_recovery 共 3 项通过；包括真实恢复插件服务 |
| 实际 BT 插件 / 隔离 ROS | 12 组用例通过。确认原目标保持、以实际 `.123 m` 而非计划 `.2 m` 终点重规划、旧动作终态及新 SLAM 停稳、取消、非指定失败不恢复。121 秒普通导航后首次恢复仍可进入 |
| scene68 封存地图真实服务链 | 业务响应符合预期：静态 READY → 明确首连接失败 → BACKWARD 0.100 m → 测试 TF 移动 → READY → 原目标路径有效。此处 TF 是测试替身，不是实际运动。程序退出发生系统 Smac 全局数据重复释放，因此整个夹具不记通过 |
| 场景准备脚本 | 原脚本仍用 odom 监测，现已改为 `/slam/pose` 的 map 位姿及净变化；23 项定向离线测试通过 |
| 完整仿真 | scene69 抓取前因继承 scene68 的 QUARANTINED 资源记录而停止，未发送动作，进程收尾完成。旧 domain40 仍为 UNRESOLVED，未核销。scene70 在无资源历史的独立 domain43 新世界完成准备并执行抓取，但在 PICK 阶段由既有 SLAM 位移监控取消，未发导航目标；恢复运动及完整放置仍未验证 |

证据根目录：`runs/mainline_20260928/workstation_alignment/`。

- `blocker_repair/recovery_candidate_manifest.json`：本次资产哈希及 scene69 启动参数。
- `blocker_repair/offline_chain/targeted_ctest.log`：正式恢复测试。
- `start_connection_recovery/compute_path_with_recovery_bt_result.json`：12 组实际 BT 插件测试及二进制身份。
- `blocker_repair/offline_chain/chain_cleanup_asan.log`：退出期重复释放证据；不得忽略退出码把夹具记通过。
- `full_transfer_runner/front_transfer_scene69_workstation_world40_20260928/`：未发动作的场景准备失败、旧隔离状态核查与收尾。
- `full_transfer_runner/front_transfer_scene70_workstation_world43_20260928/`：独立新世界同场景复验、录屏和结果目录。
- `blocker_repair/prepare_slam_fix.json`：场景准备 SLAM 唯一监测的定向验证。

工位 XML 中包络监督保留在正常跟踪与工位精调分支，已准入的 Departure 短段沿用原模块执行契约；不在短段执行中重新套入普通导航准入。

本问题原始排查开始时间为 05:21 UTC；达到一小时仍未解决，按项目规则保存证据并完成有界收尾，不重置计时。

## scene70 最终阻塞与收尾

`SLAM_POSE_CHANGED_OUTSIDE_NAVIGATION`：取消前记录的 SLAM 净平移约 0.02126 m，超过原有 0.020 m 限制；净偏航约 -0.03770 rad，在已放宽的 0.050 rad 内。这是 map 下 SLAM 估计变化，不能直接认定为真实底盘滑动。当前没有 `/cmd_vel` 命令观测，导航目标 UUID 为空。

父任务已接受取消，返回 `resources_released=true`；验证端确认 `RELEASE_CONFIRMED` 及实际停稳，进程清理完成。没有修改阈值或底盘保持配置。录屏 999 帧与 999 条时间旁车一致，逐帧解码通过；它只展示本次抓取和取消，不是完整搬运成功视频。

最小后续入口是回放同一 scene70 的 SLAM 与已封存仿真位姿，区分估计漂移和真实底盘微移，避免重复重启猜测。结果见 `blocker_repair/scene70_outcome.json`。新恢复代码的静态/隔离验收与本轮整栈未通过分别记录。
