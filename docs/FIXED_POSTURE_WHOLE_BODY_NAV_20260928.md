# 固定双臂整机预测碰撞实现记录

## 范围与基线

- 用户要求：固定双臂导航；独立分支，最小 C++ 实现，不设计动态臂轨迹、兼容层或第二条速度控制链。
- 工作目录：`/home/yjh/WorkSpace/astribot_whole_body_nav`。
- 实现分支：`codex/fixed-posture-whole-body`。
- 用户确认的最终合并目标：`chassis-effort-drive`；整体验收通过前不合并。
- 原始提交：`854a8206`。依赖快照提交 `2cc6c89f` 保存原工作目录中相关包的既有未提交修改；不是本次功能实现。原目录未切分支、未暂存、未提交。
- 本次文件归属：robot_geometry 的分层碰撞核；path_tracking 的 reader/critic/最终检查；navigation_recovery 的 Departure 执行检查及对应测试、导航参数。
- 独立构建和原始日志：`runs/whole_body_nav/`。不覆盖共享 install。

## 实施与验收门槛

1. 固定实测姿态及载荷的包络，复用既有 fixed_v2 / ArmHold 五消费者准入。
2. 固定高度切片沿 MPPI 底盘 rollout 变换，每段做扫掠检查。没有逐候选关节 FK。
3. 同一分层几何用于最终命令和 Departure 的每周期检查；候选惩罚不能替代最终输出检查。
4. 存档端点地图保留未知阻塞语义；接收这种来源不证明实时感知覆盖。
5. 编译、几何对照、真实插件/ROS loopback、整栈仿真、停止实测分别记录，不互相替代。
6. 仿真必须有独占资源和本任务会话归属；保持 `idle_position_hold=true` / `idle_position_kp=3.0`。

当前停止检查仍使用既有一秒恒速扫掠。它不是经测量的停止模型；停车扫掠和性能验收未关闭前，功能不能标为整体完成。

## 实施发现与未完成项

- 原 Arrival 已有最终速度相关检查；本次扩展高度覆盖，并检查最后实际返回的命令。
- 原 Departure 测试明确要求短段执行时不重查环境。本次按用户批准的逐周期检查需求改变此语义，测试必须验证执行中障碍/撤销拒绝。
- 原 EnvelopeEvidence 的时间参数未参与有效性判断。新增真实插件测试已复现过期包络仍可评分；motion reader 在唯一入口检查 ROS 截止时间与接收后的单调时间，修复后测试通过（critic_red.log / critic_green.log）。诊断读取不授予运动许可。
- 初次隔离构建因未选择 `astribot_payload_msgs` 失败；保留 `baseline_build.log`，补全显式构建依赖后重试。
- 本机另有 domain 40 的独占性能仿真。未启动第二套仿真、未停止其进程；整栈与独占性能验收待资源窗口。

## 验收状态

| 项目 | 状态 |
|---|---|
| 独立分支与依赖快照 | 已完成 |
| 固定几何 MPPI critic | 隔离构建、真实 pluginlib 加载与 ROS loopback 测试通过 |
| 最终输出与 Departure 高度检查 | 实际 safeCommand 路径及 Departure 插件测试通过；整栈未验收 |
| 高度区分、点间扫掠、未知空间、全无解、撤销测试 | 通过；含过期包络、测得速度、部分候选零权重、1000 组几何对照 |
| 性能 | 5 层、2000×56 候选，100 次插件评分：P50 5.970 ms / P99 8.792 ms / 最大 8.806 ms；仅空闲地图，非独占整栈验收 |
| 停止模型、断流失效、真实感知覆盖 | 未验收 |
| 整栈仿真 | 未执行 |
| 真机 | 未执行 |
| 合并开发主线 | 未执行，等待整体验收 |

## 最终离线回归与基线对照

完整路径控制测试初次结果为 15/20 通过。5 项失败：policy_recovery、corner_observation、corner_contract、corner_approach_plant、corner_replan。以依赖快照 2cc6c89f 独立重建库及这 5 个测试，5/5 同样失败；失败断言列表逐项相同。来源与加载库路径记录在 runs/whole_body_nav/baseline_failure_comparison.json 和 baseline_loaded_library.txt。这些是基线未关闭项，不计为本次新增回归，也不计为通过；未修改无关控制逻辑或旧测试来掩盖失败。

初次 reader 测试在当前包安装前误加载共享旧库而崩溃；核实 ldd 并使用本分支库后通过。这次执行无效，不作为产品失败或通过证据。

Humble 上游 critic_manager 在 fail_flag 后停止后续 critic；optimizer 在处理失败批次前仍计算控制更新。因此部分碰撞候选用无限代价得到零权重，全碰撞批次显式失败且数值保持有限。最终输出检查仍必需。

最终运行结果：路径控制 15/20，导航恢复 5/5，机器人几何 8/10。新增 critic、分层几何、reader 和 Departure 测试通过。几何包两项失败均为测试引用的 `astribot_s1_robot_geometry.model` 不存在；冻结基线也缺少该模块，CMake 和三个相关测试入口与基线逐字相同。本次未恢复已删除模块或改写无关测试，完整回归不能记为通过。

原始证据位于本隔离目录的 `runs/whole_body_nav/`：

- `optimized_build.log`、`final_critic_build.log`、`final_critic_install.log`：最终构建与安装。
- `final_path_tracking_tests.log`、`final_recovery_tests.log`、`final_geometry_tests.log`：三个包的完整结果。
- `final_critic_result.log`：插件用例与性能数据；不含完整 MPPI 其他 critics 和控制链耗时。
- `baseline_failure_comparison.json`、`baseline_loaded_library.txt`：五项路径控制失败的独立基线对照。
- `critic_red.log`、`critic_green.log`：过期包络回归用例的修复前后证据。

## 恢复验收入口

1. 先处理开发主线已有回归缺口；当前工作目录是冻结快照，不能覆盖主线后续修改。尤其 Arrival/ThreePhase 正在其他任务中演进，合并前必须逐项核对。
2. 获取空闲独占仿真窗口，使用本任务独立 session/domain/日志和隔离安装。最后探测 `/tmp/astribot_sim_performance.lock` 仍被其他任务占用；本任务未启动或停止仿真。
3. 固定双臂开展直行、侧移、原地旋转、底盘可过但双臂碰撞、高度错开、未知阻塞、执行中障碍变化、许可撤销与断流场景。覆盖 Arrival、MPPI 输出和 Departure；核实双臂保持、实际停车和接触结果。
4. 记录有障碍场景下 critic 与完整控制周期的 P99/最大耗时，以及停止距离/扫掠是否覆盖实际运动。一秒恒速扫掠不能据此视为停车保证。
5. 整体验收通过后，才将本分支合入用户指定的 `chassis-effort-drive`。目前仅保存实现检查点，未合并。
