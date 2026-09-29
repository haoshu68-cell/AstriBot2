# 窄通道专属通过模式（部分仿真通过，未合并）

分支：`codex/narrow-passage-exclusive`；基线：`2c5354c3fe2d4df46103684182c1ab852a1cf64e`。

## 当前结果

专属阶段已实现：停稳交接 → 入口朝向对齐 → 实际包络居中 → 直行通过 → 后缘加余量越过出口 → 停稳后恢复普通导航。期间不执行普通起步转向、入口折点或终点精调；安全 HOLD 保留原阶段。没有放宽原有几何余量、未知格或保持资源准入。

| 场景 | 当前结果 | 实测证据 |
| --- | --- | --- |
| 1.50 m，对称，10° / +15 cm 起步 | PASS，action=4 | 最终控制器版本 `release_sym150_v2`，目标位置误差 7.59 mm；通道内采样包络中心最大偏差 6.02 mm |
| 1.50 m，非对称，10° / +15 cm 起步 | PASS，action=4 | 最终控制器版本 `release_asym150`，目标位置误差 13.30 mm；通道内采样包络中心最大偏差 7.91 mm |
| 1.25 m，对称，10° / +15 cm 起步 | FAIL，action=6 | `final_sym125` 完成对齐、横移到约 -0.51 cm，之后通行扫掠被当前占用阻挡；未进入通道 |
| 1.25 m，非对称，10° / +15 cm 起步 | FAIL，action=6 | `final_asym125` 完成对齐、横移到约 -7.51 cm，之后同类几何阻挡；未进入通道 |
| 1.25 m，非对称，预先对正补录 | 横移阶段通过；完整通行 FAIL | `final_offset` 实际从 +15.04 cm 移到 -7.247 cm；不能替代上述四组 |

两组通过样本均未出现后缘加 0.155 m 余量尚未越过 x=0.6 m 出口便进入普通控制的样本。首次普通 `REFINE` 时后缘 x 分别为 0.8433 / 0.8547 m。位置及停稳监测均来自 `/slam/pose`，包络采用准入时实际 `reserved_footprint`；这不是外部真值或真机精度。通道内样本按完整包络与 x=[-0.6,0.6] 有交叠筛选，分别 122 / 191 个；采样最小侧向墙距分别 0.3761 / 0.3026 m。

原四组及补录导航 action 活动期间，全部记录的包络均为 `READY_FIXED`、保持均已确认；各次完成后保持资源已释放。`ARM_HOLD_UNCONFIRMED` 收尾状态没有被当作失败起因。

退出后的普通路径替换另经回归定位并修正：新路线必须清除旧到点精调/滑行状态；通道中的路径更新仍保留通道控制权。`replan_before.log` 记录修复前真实断言失败，`replan_after.log` 和 `passage_test_final.log` 记录修复后通过。两组 `release_*` 使用该最终运行库；`final_*` 1.25 m 失败样本在这个普通路径交接修正前采集，均未执行到被修正的 NORMAL 分支。

离线验证：26 个固定通道策略测试通过；8 个相关控制器测试通过，另新增/相关的 passage_sweep、filled_collision、corridor_turns 通过。覆盖通道中 HOLD/恢复、路径更新、阶段交接、生命周期结束和普通路径仍需原折点检查。整栈暂停恢复单独记录；离线通过不替代该项。

## 1.25 m 剩余阻挡与恢复入口

`clearance_evidence.json` 使用本次实际包络和实际首个阻挡框重算：对称通行扫掠 y=[-0.52364,+0.52364]，记录占用下边界 y=0.51，预留区重叠 13.64 mm；非对称居中通行扫掠 y=[-0.59612,+0.59612]，记录下侧占用上边界 y=-0.56，重叠 36.12 mm。两者 `prediction_offset_ns=0`，不是远期运动预测。

非对称名义居中侧向空间为 184.63 mm，既有通道余量 155 mm，还存在扫描 50 mm 单元及位置协方差膨胀。因此仅凭名义宽度可放入包络，不能宣称当前感知链允许通行。对称的内侧占用来源尚未证明；采集时刻 TF 的诊断修订未消除阻挡，已撤回，不能称其为根因或修复。

`diag_sym125` 的只读探针同时记录 `/map_scan` 与 `/scan_from_cloud`、capture/latest TF，在本次插入墙附近的点集中观察到厘米级差异，但未记录策略每次实际投影所使用的变换，尚不能闭合阻挡单元的产生因果。探针退出时重复 shutdown 报错，运行不记作新的通过证据；原始数据与错误均保留。下一项最小调查应将策略投影的同帧变换、生成单元、融合保留/清空串起来。不能直接删除占用或把墙补入已知地图后替代原场景验收。

整体四组尚未验收通过，真机 NOT_RUN，分支不合并。暂停恢复及录屏完整性见后续记录。

## 复现与原始记录

成功标准：原四组场景均从入口外 10° 偏航、+15 cm 横偏开始；1.25 / 1.50 m × 双臂对称 / 非对称。必须实测完成入口对齐、包络居中、直行通过，完整后缘加余量越过出口后恢复普通导航并完成原目标。横移补录不能替代四组。安全暂停保留通道阶段；取消可终止。停止证据仅用 `/slam/pose`。保留 `idle_position_hold=true`、`idle_position_kp=3.0`，不放宽碰撞、未知区或包络准入。

实现范围：

- Arrival 先发布经过基本校验的选定路径，由绑定同一路径的通道策略决定控制归属，普通折点检查延后到普通模式。
- 固定包络通道模式由现有 C++ Arrival 执行；阶段切换使用现有 SLAM 停稳证据。ALIGN / CENTER / TRANSIT 期间不调用普通起步、折点或终点精调。
- CENTER 以包络中心目标和现有 1 cm 容差为完成条件；TRANSIT 使用同一轴线与不对称偏移。退出后只交回尚未执行的路径。
- 当前任务要求完整通过；终点必须允许完整包络出通道。不通过延长用户目标来完成通道模式。
- C++ 几何核检查实际凸包的旋转/横移扫掠；连续角度采样之间的偏差由保守膨胀覆盖。通道内部障碍不短路入口扫掠检查。地图未知格和预测障碍仍阻止对应动作。
- 保留现有非固定包络模式及原速度出口；没有增加控制节点或速度发布者。

验证入口：`/home/yjh/WorkSpace/astribot_narrow_passage/runs/narrow_passage_20260929/`。`environment.bash` 指定原主线底座与本分支独立安装；仿真驱动为 `capture_fixed.py`、`run_fixed_demo.py`。每次运行保留实际加载库哈希、隔离域/partition、SLAM、选定路径、控制阶段、通道请求、地图、包络、动作终态与视频。独占仿真会话由自有 supervisor 清理。

初轮离线（历史记录，后续已增加到 26 个策略测试）：24 个策略测试通过；新增 passage_execution / passage_sweep 通过，原 filled_collision / corridor_turns / arrival_clock_domain / policy_recovery / corner_stop / corridor_refinement / corridor_route / corner_turn / corner_sweep 通过。扩大回归中的 corner_observation / corner_contract / corner_approach_plant / corner_replan 失败，未修改基线重新编译复跑得到完全相同 67 条失败；不记为通过，基线输出与修改版输出均保留。

首次仿真 sym150_v1：完成入口转向后停在 CORRIDOR_SETTLING，动作自行失败（6）；随后发出的按 UUID 取消得到已无活动目标。停稳后保持资源已释放。已定位阶段交接观察顺序问题，增加接收时间比控制周期早 20 ms 的离线测试并修正；修复后重跑中。该录像保留为失败证据。

首次仿真清理另暴露驱动发送 SIGINT 给 ros2 run 父进程而未及时交给保持执行器的问题，最终需升级信号，运行标 INVALID。后续驱动直接启动已解析安装路径的保持执行器，使自有进程句柄对应真实节点；不修改执行器业务代码。清理是否通过须由后续运行确认。

整栈四组与暂停恢复仍待验收；真机未测试。此分支未合并。

后续检查：sym150_v2 在启动参数服务读回阶段超时，未运动，全部自有进程正常退出；独立查询进程继续强制读回同一参数。sym150_v3 完成实际居中和整机出通道，出口交回普通路径时暴露最近采样点位于机器人身后，引发 `CORNER_UNSUPPORTED_REVERSAL: index=1`，动作失败（6），末位姿 x=1.1631 m、y=0.00359 m。已改为投影到剩余线段再交接，避免生成反向段或短折点，并添加密集采样出口回归。sym150_v3 的保持执行器正常退出，清理无升级信号。

第一组完整通过：sym150_v4，action=4，末端位置误差 0.0086803 m；122 个物理通道重叠的 SLAM 样本，包络中心最大偏差 0.0055103 m，采样最小侧向墙距 0.3765854 m；没有包络后缘加余量尚未出通道便交回普通控制的样本。仅为该仿真样本，不是硬件安全界限。

后续三组揭示：sym125_v1 已对齐居中，随后通道内部几何门禁拒绝；asym150_v1 / asym125_v1 已实际横移，但策略仍检查已由专属入口动作替代的原入口折点。运行时路径检查现裁掉已替代的入口前缀，保留通道内顺序和出口转弯约束；出口后缘尚未清空时仍保留通道准入检查。新增离线正反例通过。

诊断修订失败记录：asym150_v2 的输入缺失阶段因新增诊断对象重复嵌套，策略发布停滞，且策略进程最终被 supervisor 升级信号清理；该次为 INVALID，不能计作功能结论。sym125_v2 在运动前由本任务按已核实身份中止，停止后 supervisor=stopped、remaining_owned_pids=[]。诊断在每次检查前清空临时证据；100 次连续缺输入的序列化尺寸回归通过，累计 26 个策略测试通过。

1.50 m 非对称完整通过：asym150_v3，action=4，末端位置误差 0.0083837 m；200 个物理通道重叠 SLAM 样本，包络中心最大偏差 0.0071472 m，采样最小侧向墙距 0.3020475 m，无提前交回普通控制样本。目标底盘横偏为 -0.0724688 m，实际完成横移并通过。

1.25 m 阻挡已经区分为通行阶段的当前占用（prediction_offset_ns=0），入口对齐/横移检查均为 CLEAR。对称样本存在障碍 y=[0.51,0.64]；非对称样本存在 y=[-0.69,-0.56]。所用静态 /map 在现场新增两侧墙所在栅格仍为 free，墙面由扫描占用路径进入策略；保留 5 cm 扫描单元、采集协方差和全部原有余量，未以注释或仿真真值删除障碍。

TF 对照：已核对既有项目时间契约，局部试验优先采集时刻 TF 并保留缺失时 latest 语义，3 个离线试验通过；sym125_v4/asym125_v4 仍拒绝通行。现场只读 TF 探针未覆盖关键转向阶段，结束后混入后续同 domain 会话的数据，不能用于关键转向因果结论。该 TF 源码修改与新增测试已从最终分支撤回，试验日志保留。没有把尚未证实的感知修订纳入本次最小实现。


## 暂停恢复补测的无效记录与资源状态

- `transit_hold_resume`：在静态 EMPTY 场景临时创建障碍，违反背景清单；先出现 `PAYLOAD_MASS_EVIDENCE_UNAVAILABLE` / `GEOMETRY_VERSION_CHANGED`，导航执行被撤销。该次不是窄通道 HOLD/恢复验证，标 INVALID。导航末态 6；末段 SLAM 净位移最大 0.0608 mm、净转角最大 0.0000284 rad；保持动作的 `resources_released=false`，不能用进程退出当作业务资源释放。
- 测试域 32 的 `/home/yjh/.local/state/astribot/transport/domain_32.jsonl` 保留 `RESOURCE_RECOVERY_REQUIRED:TASK_CANCELED`（phase=5），未删除、截断或重置。副本、哈希和恢复边界保存在 `transit_hold_resume/unresolved_resource.json` 及同目录副本。该测试域不再用于新准入；恢复旧资源需要单独核对原动作和资源恢复契约。
- `transit_hold_resume_v2` 在运动前因上述未决资源拒绝保持请求；NOT_RUN。`transit_hold_resume_fresh33` 是独立新仿真，进一步读源码发现静态背景位置也受约束后，已在准入/注入前中止；没有请求保持资源，域 33 的账本当时为空，标 INVALID。
- `transit_hold_kinematic33` 改为复用仓库现有 `verify_kinematic_inventory.py --prepare-only`，但验证脚本仍等待无任务时并不持续发布的 `/cmd_vel` 零命令，准备失败、未运动。后续调用该脚本已有的事件驱动准备判据：策略 HOLD 且速度上限为零、保持执行器 IDLE、SLAM 连续停稳；没有制造零速度或放宽运动/资源约束。
- 上述已结束会话的 `session.json` 均为 `stopped`、`remaining_owned_pids=[]`。新增的测试只使用独立仿真资源，不宣称域 32 的旧资源恢复。

## 录屏与复现入口

- [1.50 m 对称通过录像](/home/yjh/WorkSpace/astribot_narrow_passage/runs/narrow_passage_20260929/release_sym150_v2/screen.mp4)
- [1.50 m 非对称通过录像](/home/yjh/WorkSpace/astribot_narrow_passage/runs/narrow_passage_20260929/release_asym150/screen.mp4)
- [1.25 m 对称失败录像](/home/yjh/WorkSpace/astribot_narrow_passage/runs/narrow_passage_20260929/final_sym125/screen.mp4)
- [1.25 m 非对称失败录像](/home/yjh/WorkSpace/astribot_narrow_passage/runs/narrow_passage_20260929/final_asym125/screen.mp4)
- [实际横移补录](/home/yjh/WorkSpace/astribot_narrow_passage/runs/narrow_passage_20260929/final_offset/screen.mp4)
- [场景结果、动作身份和实际加载库](/home/yjh/WorkSpace/astribot_narrow_passage/docs/evidence/narrow_passage_20260929/results.json)

原四组驱动：先加载 `runs/narrow_passage_20260929/environment.bash`，再调用同目录 `capture_fixed.py corridor <新名称> --width 1.25|1.5 --posture symmetric|asymmetric --domain <独立且资源已就绪的测试域>`。默认起始偏航 10°、横偏 +0.15 m、首路径点 x=-1.25 m。预对齐补录另加 `--yaw 0 --entry-waypoint-x -1.2`。不得在未恢复的域 32 重试；各既有场景已保存当时实际使用的驱动、覆盖层、进程身份与库哈希。


## 最终补测与结束状态

`transit_hold_kinematic33_v2` 使用现有 kinematic inventory 正式确认四个已登记、未附着的物体，并保持 MoveIt 场景为空。测试物体是 0.25 × 0.25 × 1.4 m 的已登记 box，位于远处；只通过既有 kinematic command 在通行阶段移到机器人前方。不是行人模型，不计为行人避障验收。

该场景准备检查通过，机器人进入 `CORRIDOR_TRANSIT`；移入事件发生于 ROS 71.939 s，SLAM x=-0.5421 m。随后先出现 `PAYLOAD_MASS_EVIDENCE_UNAVAILABLE` / `GEOMETRY_VERSION_CHANGED`，导航因 `ENVELOPE_EXPIRED` 撤销，action=6。保持动作返回 `HOLD_GEOMETRY_VERSION_CHANGED`，这次 `resources_released=true`；未进入预期 HOLD/恢复，完整恢复验收 FAIL。没有因此修改包络版本检查或把失败当作模式保持通过。后续必须先区分未附着环境物体运动与机器人附着几何版本转换，不能用通道模式忽略有效的包络撤销。

最终验收交付：26 个策略测试、8 个相关控制器测试、3 个相关几何测试均通过；两组 1.50 m 静态通道整栈仿真通过。1.25 m 通行、通行中真实障碍恢复、行人场景和真机不计通过。既有 4 组基线回归失败仍保留，未宣称全仓测试通过。

五段交付录像均经全帧解码检查，返回值 0 且无解码错误；1.50 m 两段已抽帧核对机器人、双臂姿态与通道画面。视频原件未剪掉失败或收尾，时间长度分别记录于 `results.json`。失败录像用于说明仍未通过，不能作为成功演示。

结束检查 `cleanup_audit.json`：本目录 28 个会话均 `stopped`，`remaining_owned_pids=[]`。域 33 最后资源记录为 IDLE / RELEASED；域 32 未决账本哈希与留存副本一致，保留隔离。主工作区仍为 `chassis-effort-drive`；本次源代码修改仅在独立 worktree / 分支，未合并。
