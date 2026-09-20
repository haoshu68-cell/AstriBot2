# 探索与 SLAM 会话收尾

## 行为约定

| 触发 | 探索处理 | SLAM 处理 |
| --- | --- | --- |
| 当前地图有效，连续确认无前沿且没有未解决未知格 | COMPLETED，冻结新目标 | 停稳后结束当前会话，优化并保存 |
| `~/cancel` / RViz“取消探索并保存地图” | 取消自己的导航目标，等待 Action 终态，冻结新目标 | 保存已有成果，manifest 标记 `CANCELED_PARTIAL` |
| `~/pause` | 暂停，可恢复 | 不结束、不自动存图 |
| 无可达前沿、候选冷却、地图不足、未知区域不可观测 | 保留原有等待/暂停/失败预算行为 | 不把这些情况误判为完整建图 |
| operator 抢占探索导航 | 保留原有抢占处理 | 不结束 SLAM，避免影响接管任务 |
| 强制关进程、急停、关机 | 原有安全关停流程 | 不承诺自动保存；不要用强制关停代替正常任务取消 |

正常完成仍采用原有 `completion_confirmations` / `completion_stable_sec` 连续确认。完成后的地图变化不再恢复已进入收尾的会话。`finalize_on_completion=false` 可保留旧的完成后重新检查行为；显式取消始终表示结束当前探索并请求保存。

## C++ 功能包

`astribot_s1_exploration` 新增 `mapping_session_node`，算法和文件校验不调用 Python、shell 或外部采集脚本。探索协调器只负责冻结任务、取消自己的 Action 和提交收尾请求，不直接设置 SLAM 参数。

流程：

```
探索完成 / 取消
  → 等待自己的导航 Action 终态
  → mapping_session 接受请求
  → WAIT_STOP：持续新鲜里程计、至少 5 帧、默认 0.5 s 停稳
  → READ_CONFIG：确认 General.is_save_map=1、唯一 mapname、finish=false
  → WAIT_FINISH：再次确认停稳
  → FINALIZING：仅一次设置 /voxelslam finish=true
  → 等待匹配目录和地图名的最终关键帧事件
  → 检查 PGM/YAML、轨迹行、关键帧文件并计算 SHA256
  → 原子提交兼容旧加载器的 manifest.json
  → SAVED
```

里程计必须只有一个发布端；平移阈值 0.01 m/s、角速度阈值 0.02 rad/s。拒绝非有限速度、过期/倒序/重复时间戳；使用 steady clock 判断等待期限和接收间隔。独立运行时里程计时效默认 0.3 s，启动配置可覆盖。只观察速度，不发布任何速度命令。其他任务仍可使用导航仲裁，不应在结束建图期间下发新的运动任务；此功能不替代全系统控制租约。

请求受理、SLAM 接受 finish、文件保存成功是三个不同阶段。二维栅格异步导出未完成时继续等待，不把“收到 final”当作保存成功。文件校验在后台 C++ 工作线程运行。默认等待期限 120 s（正在进行的文件校验不强行中断）；失败保留状态和路径。已发送 finish 的超时重试只等待最终事件/验证文件，不重复发送 finish。

| 接口 | 类型 | 语义 |
| --- | --- | --- |
| `/exploration_coordinator_node/cancel` | Trigger | 正常取消探索并请求保存；受理不是已经停稳 |
| `/mapping_session/finalize_completed` | Trigger | 协调器完成收尾入口 |
| `/mapping_session/finalize_canceled` | Trigger | 协调器取消收尾入口，记录部分成果 |
| `/mapping_session/finalize` | Trigger | 人工集成入口，调用方必须先冻结并结束运动任务 |
| `/mapping_session/retry` | Trigger | 仅 FAILED 状态允许重试 |
| `/mapping_session/status` | transient-local String/JSON | 状态、详情、绝对目录、finish 是否发送、探索结局 |

进入探索收尾后，pause/resume 均拒绝；重复 cancel 不重复提交。保存完成后需启动新的 SLAM/探索会话才能开始下一次建图。若收尾节点在 finish 后崩溃，重启后的请求会拒绝已结束的 SLAM，需人工检查会话；尚未实现进程崩溃后的事务恢复。

## 启动接入

- `exploration_coordinator.launch.py` 同时启动协调器和 C++ 收尾节点，共享里程计配置；独立启动时 SLAM 必须预先使用 `save_map:=1 map_name:=<唯一名称>`。
- `nav2_full_bringup.launch.py` 在 `exploration=true, mode=mapping, slam_backend=voxel` 时默认 `save_map=1`，生成 `explore_<时间>_<随机标识>`；其他模式默认仍为 0。显式覆盖仍生效。
- 默认输出 `/tmp/astribot_slam_sessions/<map_name>/`；正式部署通过 `save_path` 指向持久化磁盘。状态中的 `directory` 是实际路径。
- 真机 `hardware_exploration.py` 启动 C++ 节点；自主新建图且部署配置未指定名称时生成唯一名称。正常结束等待 SAVED，不再收到 exploration/complete 就关掉 SLAM。存图 FAILED 时保留任务现场供重试或人工关停。
- 真机地图根目录沿用 `deployed_sensors.json` 的 `save_path`（当前配置 `/home/astribot/maps/voxel/`）。定位/已有地图续建仍需明确配置，不自动更改已存地图策略。
- RViz 新增取消并保存、失败重试按钮，取消前确认；显示收尾状态。诊断录制白名单包含 `/mapping_session/status`。

## 验证边界

在 `/tmp/astribot-mapping-build` 隔离构建，不覆盖共享安装目录。C++ 测试使用 localhost 域 229/230 和假 SLAM/里程计：验证暂停不存图、取消幂等、取消后不能恢复、移动/陈旧里程计不触发 finish、正确最终事件与完整文件后提交、禁用保存/无里程计失败、缺失文件超时及仅校验重试。测试不发送底盘速度或生产导航目标。

SLAM 源码及其注释未修改。真实 Voxel 优化、实际栅格导出耗时、仿真全链路及真机停稳仍需后续验收；这些离线测试不代表真机存图已验收。

验证结果：新增 6 个 C++ 用例通过，操作台原有 8 个用例通过；4 种启动模式的默认存图开关检查通过。C++ 生成的 manifest 已由现有 `slam_session.inspect_session` 离线读取，文件哈希及元数据一致。诊断参数白名单增加 10 项收尾/存图参数，便于回放核对当时配置。
