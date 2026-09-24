# M1 首段 MTC 执行器交付 — 2026-09-24

## 状态与边界

问题 M1-EXEC 本轮原始起点 09:51 +08，10:51 为一小时检查点；此前未完成执行器的历史排查时长未知，不计为零。当前首段执行器已实现、独立 Release 构建安装、6/6 CTest 和 7/7 隔离合成 ROS 协议通过。真实 Gazebo 非 home→Hold **NOT_RUN**，由总调度审查后交 M2 唯一操作者运行。本任务不启动 Gazebo、GPU 或真机。

`PlanToHold` 接收完整 PICK/PLACE 规划输入，内部请求完整 `PlanManipulation` 并绑定 context、场景、全部资源关节实际起点。只执行第一段 PREGRASP/PREPLACE：6 个 JTC 子动作全终态 → 新鲜实测双时间 500 ms 稳定 → 左臂目标误差 ≤0.02 rad → guard 解除确认 → 保持任务租约及 ArmHold。保持不是完整 PICK/PLACE 成功；父任务保持活动，取消后经终态及新的稳定窗口才返回资源释放。

原有 `hold_executor` 仍只保持当前姿态；两个执行器不能同时运行，也不能同时存在旧 Python 控制器客户端。新程序使用同一 canonical resource journal/lock，重启未决记录不能自动清除。QUARANTINED 同一进程仍可接受迟到真实终态与新实测稳定证据后释放；重启缺少旧动作上下文时保持 recovery required。

## 验证结果

日志根目录：`/home/yjh/WorkSpace/astribot_sdk_ros2/runs/m1_transport_20260924/`。

| 场景 | domain | 独立判据 | 结果 |
|---|---:|---|---|
| normal | 170 | 6 子动作、实际夹具关节改变、Hold；导航许可交接期间保持；取消后释放 | PASS |
| wrong_context | 171 | context 不符，不发送 JTC，不发布确认 Hold | PASS |
| start_changed | 172 | 规划后实际起点改变，不发送 JTC | PASS |
| scene_changed | 173 | ACM 改变，不发送 JTC | PASS |
| pending_cancel | 174 | MTC 接受应答迟到；取消 ACK 不释放；迟到接受后取消，不发送 JTC | PASS |
| unknown_result | 175 | 左臂 UNKNOWN 不是终态；不得确认 Hold/释放，10 s 后 recovery required | PASS |
| no_endpoint | 176 | JTC 成功但实测未到目标，禁止确认 Hold | PASS |

各场景原始事件、状态和返回结果在 `protocol/<mode>_<domain>.json`，节点日志为同名 `.log`。合成状态不证明实际运动。`ctest_4.log` 包含 MtcPlan、SceneBinding、资源授权、子动作、journal、ArmHold 六个测试目标，不是六个场景总数。

验证夹具：`ws_robot/src/astribot_s1_transport_native/test/verify_plan_to_hold.py`。只允许在新确认归属且空闲的 domain 运行；已使用的 170–176 不能通过删 journal/lock 复用。重现命令在相同 overlay 下为：

```bash
python3 ws_robot/src/astribot_s1_transport_native/test/verify_plan_to_hold.py \
  --mode normal --domain "$ROS_DOMAIN_ID" \
  --executable runs/m1_transport_20260924/install/lib/astribot_s1_transport_native/trajectory_executor \
  --output runs/m1_transport_20260924/protocol/NEW_OWNED_RUN.json
```

## 安装覆盖层与最小真实请求

工作目录 `/home/yjh/WorkSpace/astribot_sdk_ros2`。先加载 M2 所属会话已核实的环境（domain、partition、RMW、实际 geometry/ledger/navigation overlay），再加载：

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/astribot_navigation_msgs/share/astribot_navigation_msgs/local_setup.bash
source ws_robot/install/astribot_transport_msgs/share/astribot_transport_msgs/local_setup.bash
source runs/mainline_20260924/mtc_export/install/local_setup.bash
source runs/m1_transport_20260924/install/share/astribot_s1_transport_native/local_setup.bash
```

此安装来自直接 CMake，**没有顶层 `install/local_setup.bash`**。正式程序：

```bash
runs/m1_transport_20260924/install/lib/astribot_s1_transport_native/trajectory_executor \
  --ros-args -p use_sim_time:=true -p simulation_commissioning:=true
```

启动前要求：实际 MTC `/transport/plan_manipulation`、完整 `/get_planning_scene`、`/transport/execution_guard/set` 及状态可用；固定 V2 fresh HOLD、navigation_allowed=false；新鲜完整几何/附着状态及 6 JTC claims；MoveIt `allow_trajectory_execution=false`；旧导航目标已终态、实测底盘停稳、旧 Hold 任务资源已释放，旧保持执行器已由所有者退出。节点本身不掌握旧导航 Action UUID，不能替操作者证明旧导航已终态。

动作 `/transport/plan_to_hold`，类型 `astribot_s1_transport_native/action/PlanToHold`。输入字段：

```text
task_id, request_id, context_id: 本次唯一标识
operation: PICK 或 PLACE
object_id: 完整 PlanningScene 中对应的真实对象标识
pre_target, target: 当前场景已核实 frame/姿态的 PoseStamped
exit_targets: 完整 MTC 规划所需退出目标列表
touch_links, grasp_width_m: 同一物体/模型的已验证注册值
```

最小真实试验使用现有成功场景的**当前实测**规划输入；不可把旧快照的数值直接当作新会话目标，不可用 synthetic normal 的全零示例驱动仿真。即便只执行 PREGRASP，MTC 仍规划全部 PICK 段，因此完整后续规划所需对象和目标必须有效。

调用方必须复用 M2 验证器的 Action client + 续约流程：从 feedback/status 读取本任务 lease_id/resource_epoch，调用 `/transport/hold_executor/renew`（RenewHold），严格递增 sequence，建议每 0.2 s；2 s 租约到期停止。单独 `ros2 action send_goal` 不能续约，不是可用的完整验证客户端。

当反馈 `FIRST_STAGE_HOLD_CONFIRMED`、实际 `/navigation/arm_hold` 和几何一致时，M2 才可用现有 SetFixedEnvelope 流程生成新 epoch 并验证六 ACK；M1 不自动授予导航。结束时取消这个父 Action，等 `resources_released=true`；未释放必须保留 journal，不能删除文件后重试。

## 已知限制与待验收

- 只支持当前左臂 MTC 轨迹和固定 22 关节 / 6 JTC；无任意外部轨迹执行接口。
- 尚未运行多阶段抓放、真实非 home、导航携物、真实载荷附着/放置确认、长期稳定性或性能 A/B。
- 首段版本在 live 场景绑定改变时明确拒绝；旧业务链的“只变占据图时对同一剩余计划重验证”尚未接入。因此存在动态占据场景的可用性差异，不能报告全场景等价迁移完成。
- 尚未接入 M3；其 planning scene 是提案，执行器仍需实际提交、独立读回并重验。不得悄悄重新规划不同 context。
- graph 检查是合作式写入者隔离，有发现延迟；不等于 DDS 身份鉴权或真机执行端 epoch 栅栏。

## 改动与复现

修改本包 CMakeLists.txt、package.xml、src/hold_executor.cpp；新增 action/PlanToHold.action、mtc_plan.hpp/.cpp、scene_binding.hpp/.cpp 及两个 C++ 测试和 verify_plan_to_hold.py。原 HoldResources 入口与既有资源核心复用。场景占据签名链接总调度导出的 MTC 库，没有复制兄弟包实现。

构建日志 `configure_3.log`、`build_4.log`、`install_4.log`；构建目录 `runs/m1_transport_20260924/build`，Release、BUILD_TESTING=ON、并发最多 2。源和安装哈希见本目录 `handoff_manifest.json`。未写共享 install，未提交或切换 Git 分支。早期配置依赖缺失、scene_binding 语法缺失及错误 overlay 路径的失败日志均保留，不记为验证通过。
