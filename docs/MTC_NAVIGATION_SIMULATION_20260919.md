# MTC 接入与统一导航仿真（2026-09-19）

## 当前结果

原生 MTC 已接入，导航仓库中的完整基线任务 `task20`（本证据目录内的编号）于本轮通过，墙钟耗时 **127.58 秒**。流程包含当前 RGB-D 定位、PICK、运输包络确认、两段导航、PLACE、释放确认、退臂、收臂及空载 HOLD，最终账本为 `SUCCEEDED / PLACED`，资源记录无未确认执行器。

| 本轮实测 | 结果 |
|---|---|
| 仿真世界与速度 | 导航 `small_warehouse.world`，目标实时因子 1.0 |
| 离台点到位误差 | 1.090 mm / 0.0576° |
| 放置工位到位误差 | 0.679 mm / 0.0478°，保留原 2 mm / 0.1° 导航标准 |
| 物体放置误差 | Gazebo 两次稳定观测 0.318 mm；MoveIt 场景 0.425 mm |
| 最终载荷状态 | PLACED，附着清空，载荷质量 0，底盘 HOLD |

上述数值是该次仿真真值下的结果，不是真机精度、统计成功率或夹持力验收。失败迭代、只读规划探针和最终成功记录分开保存。

![本轮实测底盘轨迹与最终物体位置](../runs/mtc_navigation_20260919/warehouse_transfer_trace.png)

证据：[结果摘要](../runs/mtc_navigation_20260919/task20/summary.json)、[完整事件](../runs/mtc_navigation_20260919/task20/events.jsonl)、[PICK 计划](../runs/mtc_navigation_20260919/task20/pick_mtc_plan.json)、[PLACE 计划](../runs/mtc_navigation_20260919/task20/place_mtc_plan.json)。请求快照同目录保存为 `*_mtc_request.json`。

## 范围

依据 `MTC_TRANSPORT_ARCHITECTURE_PROPOSAL_20260919.md`，原生 MTC 负责工位内 PICK/PLACE 完整序列规划，任务层保留实际执行、取消终态、物体事务和保载责任。导航继续走现有任务仲裁、包络握手与运动保护。机器人关机期间不连接硬件，不把虚拟相机安装值当作真机外参。

唯一仿真世界为导航的 `small_warehouse.world`，唯一高层启动入口为 `tools/launch_sim_stack.sh`。搬运支持节点只增加 RGB-D 观察器和 Gazebo 位姿服务，不创建第二个世界、地图、时钟或导航实例。可选择导航已有 mapping/localize/baseline 模式；当前固定工位验收采用 baseline 地图坐标。

## 实现

- 新包 `astribot_s1_transport_mtc`，`/transport/plan_manipulation` action 支持取消与阶段反馈。
- `FixedState → SerialContainer`：PICK 预抓取、接近、夹爪闭合、预测附着、抬升、运输姿态；PLACE 以 `GeneratePose + ComputeIK` 搜索最多 32 个预放置关节解，经 `Connect` 连接当前姿态，继续接近、张爪、预测解除附着、`Alternatives` 退让候选、收臂。
- 返回完整有序轨迹及每段预计起始 RobotState。仅规划，不调用 MTC execute，不直接驱动控制器。
- 取货前增加头部朝向与到位检查，再等待 RGB-D 三帧稳定观测。头部直接使用现有 head_controller 的 JTC action；不新增整机执行器。
- 完整序列经现有限位、碰撞、奇异和时间参数化校验后，任务层逐段校核 HOLD、底盘位置、起始关节、场景、标定版本、载荷归属及计划有效期。PICK/PLACE 在各自工位重新取快照，轨迹不跨导航缓存。
- MTC 仿真执行默认速度/加速度缩放均为 0.1；规划前检查 0.5 秒不同时间戳的底盘稳定样本（1 mm / 0.003 rad），执行前仍保留原有 2 cm / 0.02 rad 偏移拒绝条件。
- PLACE 的逆解搜索独立预算为 2 秒，最多 32 个候选；请求传入 45 秒规划预算，客户端设置 55 秒截止并在超时后等待取消终态。每次搜索最多收集 12 个完整方案，安全校验失败时最多重搜 8 次，每次依据剩余预算决定是否继续。MTC 专用 OMPL 采样间隔比例为 0.0004、RRTConnect range 为 0.2，最终独立轨迹校验继续保留。
- 实际 attach/detach 继续通过现有 MoveIt/Gazebo 确认；计划中的场景变更不代替物理确认。放置整段规划失败时不张爪。
- 31 项离线测试覆盖事务、RGB-D、几何以及计划缺段、乱序、过期、状态变化、重复消费等拒绝条件。

## 控制边界与阶段

```mermaid
flowchart LR
  T[任务事务与资源租约] --> H[头部朝向和 RGB-D 定位]
  H --> P[MTC 完整 PICK 规划]
  P --> E[逐段校验并使用原执行器]
  E --> A[实测附着与运输包络确认]
  A --> N[原导航仲裁器：离台、运输、进站]
  N --> S[停稳 HOLD、刷新场景]
  S --> Q[MTC 完整 PLACE 及退路规划]
  Q --> R[放置、释放确认、退臂、空载 HOLD]
```

MTC 只管理固定底盘的工位内动作。头部、双臂、夹爪、躯干及底盘仍由任务层持有协作式互斥租约。完整 PLACE 计划失败时，夹爪不会收到释放指令。局部规划仅为箱体与指垫、箱体与对应台面允许必要接触，离开接触阶段恢复原碰撞矩阵，不全局关闭碰撞。

当前抓取姿态由场景给定；MTC 联合规划接近、抓取、抬升和收臂，放置退路有三组候选。尚未接入任意物体的自动抓取候选生成器。

## 依赖

MTC Humble 源码固定在 `756634951326ae17ae099882f7110c6f1d0a98c0`，在 `ws_robot/deps` 中针对本机 MoveIt 2.5.9 编译；消息与辅助库使用官方 ROS 包的本地前缀。`tools/setup_mtc_humble.sh` 可复现准备过程。未升级系统 ROS/MoveIt，也不需要 sudo。

验证构建位于 `/tmp/codex_transport_build`、`/tmp/codex_transport_install`。这是同一仓库环境的构建覆盖层，不是另一仿真世界。通过 `ASTRIBOT_OVERLAY_SETUP` 使用同一导航入口加载覆盖层。

复用本机已验证构建时，所有终端先加载 `/opt/ros/humble/setup.bash`、`/tmp/codex_transport_install/setup.bash` 和 `tools/setup_mtc_humble.sh`；导航入口设置 `ASTRIBOT_OVERLAY_SETUP=/tmp/codex_transport_install/setup.bash`。正式使用前可按包 README 将相同源码构建进常规 `ws_robot/install`，不要把旧 install 当成已验证版本。

## 清理

已删除搬运独立入口 `transport_sim.launch.py`、空世界 `worlds/transport.sdf`、空地图 `transport_map.yaml/pgm`，并移除安装目录中的同名遗留文件及源/安装目录的旧入口字节码缓存。复查上述目录已无这些文件。第三方仓库模型是导航依赖，保留。旧视频、事件账本和文档作为历史证据保留，不能用于证明新 MTC/仓库验收。

## 验证状态

| 项目 | 结果与证据 |
|---|---|
| 构建与 ABI | MTC、接口、操作适配、感知覆盖层构建通过；MTC 动态链接到本机 MoveIt 2.5.9，无缺失库 |
| 离线契约 | 31 项测试通过；包括失败保载、释放不确定态、整段计划拒绝、顺序、重复消费、RGB-D 歧义及深度缺失 |
| 原生规划拒绝 | 不可达 PLACE 返回 ABORTED、空轨迹；附着物不变；仅规划探针 |
| 原生规划取消 | 返回 CANCELED 终态、空轨迹；附着物不变；仅规划探针 |
| 长路线 task10 | RGB-D、MTC PICK 与首段带载导航通过；第二段因 OBSERVATIONS_STALE 失败并保载停止，未执行 PLACE |
| 双工位 task11 | 抓取和大部分导航通过；终端预测扫掠与放置台栅格冲突，TEMPORARILY_BLOCKED 超时，保载失败，未执行 PLACE |
| 慢速调整工位 task12 | 放置点改为 (1.15, 0.72, 1.095)，预测碰撞通过；末端微调停滞后主动请求取消，确认 CANCELED、ATTACHED、无未确认执行器；未执行 PLACE |
| 实时 task13 | Release SLAM、1 倍步进，PICK 和导航通过，到位误差 1.576 mm / 0.0508°；原单分支 PLACE 接近规划失败，保载停止，无释放指令 |
| 多逆解 PLACE 探针 | 同一 task13 现场快照，改用 ComputeIK + Connect 后六段规划及验证通过；只规划，不执行 |
| task14 / task15 | task14 在首批 TF 时效准入失败，未创建工位；task15 在 PREGRASP 后触发 MTC_BASE_MOVED，未夹取；之后补充新鲜 TF 等待、停稳采样及较低机械臂速度 |
| task16 | PICK、导航通过（0.406 mm / 0.0363°）；PLACE 退让无完整解，保载停止；为 ComputeIK 增加独立 1 秒搜索预算后，同一快照六段规划通过 |
| task17 / task18 | 导航通过；分别被最终碰撞校验与退让完整性校验拒绝，均保载停止；随后加密 MTC 采样、增加候选预算并记录完整请求 |
| task19 | PICK 通过；直接进站路径在起步时被 NO_SAFE_REVERSE_EXIT 拒绝，保载停止；场景补充前向离台中间点 |
| 最终 task20 | 同一导航仓库会话 nav19、默认 1 倍步进；RGB-D → PICK → 离台 → 进站 → PLACE → 退臂 → 空载 HOLD 全流程通过 |

`task10` 第一段到位误差 0.000162 m、0.001067 rad，为仿真真值坐标下的实测误差，不代表真机精度。失败账本保持 ATTACHED，底层 action 已确认终止，随后仅销毁本任务拥有的仿真实例以重建独立测试夹具；没有将失败流程标记成功，也没有验证 ATTACHED 自动恢复。

长路线失败后的 45 秒只读采样未复现持续过期；odom / joint_states / scan 的最大龄期约 20 / 10 / 136 ms，三路各有唯一发布者。旧错误缺少逐路龄期，无法事后确定是哪一路瞬时触发。已增加逐路龄期诊断，并改用整数纳秒保持原有时效边界，不能据此宣称根因已经解决。

双工位基线取货中心 `(0.10, 0.70, 1.095)` m，先导航到离台点 `(0.45, 0, 0)`，再到 `(1.10, 0, 0)`，放置中心 `(1.15, 0.72, 1.095)` m。它用于验证一次抓取、移动、放置的衔接；不替代长路线压力或随机场景测试。task11 的阻挡来自放置台栅格 (1.175/1.225, 0.625)，当前姿态净空仍为正，但路径末端预测姿态有碰撞，未屏蔽该检查。

## 运行与时效限制

启动诊断保留在 `/tmp/codex_mtc_20260919`：旧安装缺失的 SLAM 包已补构建；旧 `/scan` 选项没有生产者，已回到标准 `/scan_from_cloud`。仿真 LiDAR DDS 与待处理队列保留最新一帧，硬件默认仍为 DDS 深度 1000、内部队列不设新增上限。

早期仅收敛队列时，0.15 与 0.10 倍步进仍触发时效保护，task12 使用 0.05 倍步进做功能诊断。后续环境已恢复默认 1 倍步进；没有改写传感器时间戳，也没有降低时效、TF、碰撞、到位或导航安全门槛。墙钟超时保留机械臂 240 秒、每段导航 1200 秒的调试余量。

检查发现 SLAM 的 `CMAKE_BUILD_TYPE` 缓存为空，原默认 Release 设置未覆盖空缓存，实际编译没有优化。已修正默认值设置（仍保留用户显式选择的 Debug 等配置），重建并确认 `-O3 -DNDEBUG`。使用相同 8 秒采样方法、1 倍步进，`/map_scan_filtered` 接收龄期从平均 686 ms / P95 762 ms（8 帧）降为平均 14.5 ms / P95 17.2 ms（77 帧）。这是当前机器与场景的短窗口实测，不代表长期耐久或真机时延。

task13 的低速微调随后自行收敛并通过原有 2 mm / 0.1° 判据，因此不能把 task12 的取消推断为确定的底盘控制根因。PLACE 的只读采样逆解/碰撞检查全部有效，但单个预放置分支不能连续完成下降；多逆解候选解决了该现场快照的完整规划问题，最终执行验收单独记录。

证据目录：`runs/mtc_navigation_20260919`。最终成功会话的实际统一日志为 `/tmp/codex_mtc_20260919/nav19/session.log`，并已复制至证据目录的 `nav19/session.log`；`navigation_session.json` 保存就绪测量（7 个导航节点 active，TF 龄期及实测 topic 帧数）。旧空场景的 task20 是另一证据目录的历史编号，不能混用于本轮结果。

nav10、nav14 在 gz_ros2_control 加载期停滞：仅 5 个 Ignition 话题、/stats 无迭代。已按进程归属停止并使用同一入口重试；nav11、nav15 正常就绪。该启动问题的根因未修复。

## 明确边界

箱体仍为运动学附着，未验证力、摩擦、滑落或动力学。RGB-D 为 Gazebo 虚拟通用相机；真机型号/标定/驱动待开机只读审计。MTC 集成不等于电池回充、任意状态自动恢复、双臂闭链或全身同时规划已经完成。MoveIt 当前仅管理任务工位及载荷几何，仓库固定障碍由导航地图/感知管理；不宣称完成整仓三维操作碰撞融合。
