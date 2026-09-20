# 非 home 搬运：隔离施工与分阶段验证（2026-09-19）

本轮按用户明确授权，新增同一导航仓库的隔离运行实例。复用 canonical warehouse、Nav2、MTC、任务仲裁与独立保护。没有建立另一种演示世界，没有操作真机，也没有停止其他任务的会话。

当前结论：完成隔离能力、公共几何边界补强、高度观测链路修复及一轮规划器对照。两次完整抓取—收臂—带载导航—放置成功；空载非 home 通道穿越成功；三种运动中断更停车成功。**P3/P5 仍有未放行项，不能把本报告理解为全场景验收完成。**

证据根目录：`runs/nonhome_isolated_20260919/`。图与机器可读指标为 `verification_evidence.png`、`measured_metrics.json`，生成脚本为 `analyze_evidence.py`。真实启动日志分别为 `stack01/session.log`、`stack02/session.log`，不是用 `latest_sim` 判断存活。

## P0：冻结基线与实例隔离

- `p0_source_hashes.json`：247 个相关源码/配置的 SHA256；`p0_git_head.txt`、`p0_git_status.txt`、`p0_worktree.patch` 保留开始时共享工作区状态。
- `p0_contract.json`：控制权、故障语义、对照方式和预算。几何/关节源证据 300 ms，保持租约 300 ms，消费者 ACK 500 ms，关节保持误差 0.003 rad，每侧净空预算 80 mm，导航限速 0.2 m/s。未放宽碰撞、到位或保持门限。
- `frozen_warehouse_transfer.json`、`frozen_arrival_motion.yaml`、`frozen_arrival_precision_sim.yaml`：本次对照配置。操作 240 s、导航 1200 s 已存在于 P0，未为本轮结果放宽；验证工具自身还使用更短的有界操作超时。
- MTC 控制静止操作；任务层管理资源、附着、保持与包络事务；导航经现有 arbiter；Nav2 规划与跟踪；策略层决策；独立 protection 持有最终速度输出。

新增 `tools/sim_isolation.py`，supervisor 暴露 `--instance`、`--ros-domain-id`，连通导航、感知和 Gazebo 的参数转发。默认实例仍保持原 Domain 25。显式隔离模式使用独立 ROS domain、Ignition/Gazebo partition、发现端口、锁、地图目录与日志索引，并检查已有直接启动的 ROS/Gazebo 进程冲突。

本次身份：Domain **74**，partition **astribot_nonhome_p0_p5_20260919**，Ignition 发现端口 **18148 / 18149**。`isolation_process_audit.json` 核对实际进程环境；不是只查看 shell 环境。

实测：ROS 测试标记在本实例收到 80 帧，在另两实例 Domain 213、67 均为 0，同时两者时钟均有真实收数。Ignition 标记在相同身份收到 60 帧，错 partition 和错发现端口均为 0。见 `ros_isolation_result.json`、`ign_isolation_result.json`。这是控制图/传输命名空间隔离证据，不是网络安全认证。

## P1：公共几何

保留 `astribot_s1_robot_geometry` 的 URDF/FK、物理凸包、含关节误差的预留凸包、高度切片、逐关节源时间、模型/附着/时钟版本接口。

本轮发现并修复 OBJ 解析欠包络：原解析只接受行首 `v `，缩进或 tab 顶点被忽略；现在按词元解析全部顶点，拒绝不完整坐标及不支持的齐次权重。`p1_mesh_before.log` 留有失败用例，后续测试覆盖旋转、负缩放 mesh、偏置箱体/球/圆柱及关节误差角点。

公共几何及投影契约共 24 项通过，包括缺失、NaN、过大关节误差拒绝。采样测试不能替代对任意模型的数学证明。URDF mesh 使用保守边界；**MoveIt 任意附着 mesh/plane 仍拒绝，尚未实现相应精确自过滤和运行验收**。没有把复杂 mesh 转成大 AABB 后直接过滤点云，以免擦除外部障碍。

## P2：同版本确认与失效停车

六类消费者：global_costmap、local_costmap、planner、controller、policy、protection。会话、epoch、geometry hash、源租期及实际安装多边形必须一致。

新增消费者逐一错误会话/epoch/hash、负 ACK、过期、未来时间及撤销用例；六消费者的负 ACK 均不能维持放行。固定包络及通道单测 22 项通过。

运动故障通过正常资源租约与导航 arbiter 发起，在实测速度约 0.04 m/s 后注入。记录独立保护的零速与停稳，再请求取消导航；不是用主动 cancel 制造停车结果。停稳判据：新鲜速度 <0.005 m/s、角速度 <0.01 rad/s、最终输出零值持续 0.6 s。

| 故障 | 包络撤销原因 | 首次零输出延迟 | 停稳延迟 | 最大后续位移 |
|---|---|---:|---:|---:|
| 停止本任务保持续期 | ARM_HOLD_EXPIRED | 260 ms | 737 ms | 25.84 mm |
| 暂停本实例几何生产者 | GEOMETRY_EXPIRED | 133 ms | 623 ms | 18.48 mm |
| 暂停本实例策略节点 | WAITING_FOR:policy | 249 ms | 841 ms | 27.67 mm |

三例均无零速后的再次非零输出。时间均为仿真源时间，位移来自仿真 odom；仅证明这些速度和空载姿态的测试，不外推到满速、载荷或真机制动距离。工具只接受匹配 domain、partition、instance 和执行角色的 PID，用 pidfd 暂停/恢复，finally 恢复进程并确认取消。

`moving_obstacle02` 在新高度链路、compact 姿态、实测 0.1007 m/s 时，于前方 0.75 m 创建实体阻挡墙。首次零输出 305 ms，停稳 1.045 s，故障后最大位移 61.57 mm，预留凸包外接圆到障碍的保守净空下界 150.05 mm，未降到 80 mm 预算以下。策略进入 HOLD/YIELD，控制器同时报告 FOLLOW_POLYGON_SWEEP_BLOCKED；最后才发 cancel，移除自有夹具。此例是突现障碍停车，不是行人连续横穿或动态绕行验收。前次 `moving_obstacle01` 在等待实测速度阶段失败，没有注入障碍，不计成功。

`moving_hold02` 在新感知链路、实测 0.1004 m/s 重复保载断更：零输出 278 ms，停稳 1.340 s，后续位移 68.39 mm，无再次非零输出。高速度几何断更的两次准备失败也保留：`moving_geometry02` 的目标路线被 PATH_QUALITY_UNSAFE 拒绝，`moving_geometry03` 未达到触发速度、末态 CLEAR_CONFIRMATION；两例均没有暂停几何节点，不得计为故障停车通过。复位后的连续任务准入稳定性仍需闭环，不靠放宽门限消除这些失败。

## P3：多场景决策与通道验证

`verify_nonhome_navigation.py` 现在除 200 个投影组合，还直接调用生产 `FixedCorridorPolicy` 检查 1,200 个组合：五种宽度、四种几何、五种航向、双向路线、通道内外及输入/几何故障。保持以下决策规则：

| 场景 | 决策与限制 |
|---|---|
| 已对齐、侧向预留和路径扫掠均满足 | 直接低速通过，不因非 home 标签强制停住 |
| 前向伸展、直行宽度可行 | 可以候选直行；转弯另验全身旋转扫掠 |
| 横向伸展超过可用宽度 | 通道外改姿态或重规划；不自动缩小包络 |
| 偏置载荷 | 计算左右独立净空，路径使用相应偏置；不能只按对称半宽准入 |
| 入口航向错误 | 在全身尚未入通道且完整旋转扫掠可行时停车对齐 |
| 入口侧向偏差 | 仅通道外、有界平移扫掠可行时居中；禁止边转边挤入 |
| 通道内部航向/预留越界 | HOLD 或明确失败；不原地旋转、不释放载荷 |
| 双向路线 | 用对应方向的通道及全身投影重验；这不等于已支持底盘倒车 |
| 缺失/过期状态、版本不一致 | 撤销许可并停止，不能使用名义 home 包络兜底 |
| 出口 | 全身和余量离开后才解除通道转向限制 |
| 内部精确停靠 | 本轮未通过，任务应优先设置出口外终点；误差超界时明确失败，不放宽精度 |

`through01`：1.30 m 实体墙通道、空载 compact 非 home、(1.4,0,π) 到 (−1.4,0,π)，正常 arbiter + NavigateThroughPoses + RequireCorridorRoute 完成。2,810 个实际 TF 轨迹样本确认经过入口/出口，不能用墙外绕行替代。终点位置误差 **1.047 mm**、航向 **0.02255°**。

中心位于墙体区间时，180 个 odom 样本最大中心偏差 **26.80 mm**；物理凸包到平行墙最小侧向净空 **283.90 mm**，含模型/关节预留凸包为 **255.37 mm**，未扣掉 80 mm 净空预算。该计算保守地视墙为无限平行直线；是仿真几何距离，不是接触传感器测量。FOLLOW 单独 59 样本，绝对横向误差 P95 **26.26 mm**；REFINE、失败导航及故障停车不并入这一指标。

负例：85 cm 不足宽度拒绝；1.10 m 内部反向终点由当前部署规划器拒绝；1.10 m 沿轴终点仅规划成功。

失败记录必须保留：`internal01` 在内部目标附近留下航向残差，控制器以 `GOAL_HEADING_UNREACHABLE: ROTATION_FORBIDDEN_IN_NARROW_PASSAGE` 终止；`entry02` 的逆向返回路线不满足当前通道路径约束；清除自有实体墙后恢复普通策略的 `entry03` 达到工具超时。`through01` 从其实际停车位（距准备点约 19 mm）重新验证路径后成功，不能抹掉前述失败。

## P4：规划器对照与高度覆盖

新增 `tools/sim/planner_benchmark/`，载入现场冻结的 costmap、start/goal、已确认 installed polygon 和版本。三个原生插件真实配置并规划；costmap 在 configure 前明确设置 polygon 模式，关闭 downsampling/unknown，Hybrid 用 72 bins、Reeds-Shepp、0.5 m 转弯半径；Lattice 使用安装包的 5 cm/0.5 m Omni primitives（16 个非均匀方向）。

所有结果再经同一 filled-polygon 连续扫掠检查：未知/254+ 为碰撞，平移与半径×转角确定采样数，采样间用扩张栅格保守覆盖。部署的 ExactGoalPlanner 路径也在同一冻结输入上复核。只有 `through01/*.polygon.json` 是最终 polygon 配置结果；早期 `*.benchmark.json` 演示了仅调用 setRobotFootprint 不会退出默认圆形模式，不能混入结论。

| 冻结场景 | 部署 Exact 2D | Hybrid | Omni Lattice |
|---|---|---|---|
| 0.85 m、内部沿轴目标 | 拒绝 | 无路径 | 无路径 |
| 1.10 m、内部沿轴目标 | 几何通过 | 几何通过 | 几何通过 |
| 1.10 m、终点要求反向 | 拒绝 | 几何通过，含倒车 | 几何通过，含倒车 |
| 1.30 m、完整穿越路线 | 几何及实际执行通过 | 几何通过 | 几何通过 |

本批原生 search 耗时约 0.2–7.6 ms，不含配置、生命周期、统一碰撞复核或控制器执行。样本数很小且机器有并行仿真，不据此宣传实时性能。**维持 Exact 2D + 统一多边形检查为当前执行基线**；优先继续评估符合全向底盘特性的 Omni Lattice，Hybrid 保留候选。两者仍有 execution gate，需验证带符号速度、路径 yaw、倒车感知覆盖、入口/出口规则及停车，不能只换 plugin 即放行。

高度实体测试发现此前截断：投影上界已扩到 2.2 m，SLAM `/map_scan_filtered` 的 ROI 仍约 1.534 m；高处点云在投影前已丢失。低矮稀疏障碍也存在漏标。修正只影响 fixed_v2 仿真：

1. 投影改取 `/map_scan`，保留 SLAM/建图原 `/map_scan_filtered` 接口。
2. 导航投影不再执行统计离群点删除，low/overhead 的 min_points=1，保守保留稀疏障碍。
3. 几何准入检查完整输入源、坐标系、连续启用的切片、稀疏点策略和高度上界；旧的“只检查 overhead.z_max”不能再通过。

`height01` 原链路漏标 0.1/1.9 m 目标；`height02` 新链路对距底盘 2 m 的 0.1/0.6/1.2/1.9 m 目标均产生 254 障碍。2.5 m 目标有原始回波、超出当前投影范围，不标为本体高度内障碍。前后是相同相对距离/尺寸的夹具，世界位置和底盘姿态不同，不是严格同位姿 A/B。尝试原子热切换旧输入被节点拒绝（不支持 input_cloud_topic 热更新），参数未改变，没有把该尝试算作已完成 A/B。

当前有限高度用例可由保守 2D 投影处理，尚不足以放行所有高处障碍场景，本轮未默认启用 VoxelLayer。靠近高障碍后目标可能离开垂直视场，扁平 LaserScan 的 clearing 也不能证明整个高度柱自由；下一步必须测试高障碍接近、遮挡及重现。如果需保持这些高度占据或在横梁下通过，应接未裁剪 PointCloud2 的逐体素 marking/clearing 与分层几何，不能仅凭同一水平角无回波清空整根高度柱。**VoxelLayer 无法补回上游裁掉或遮挡而未观测的点**。这些夹具结果不是整圈完整三维可视空间证明，真机相机外参/盲区仍待开机后验证。

## P5：任务结果、时序与放行边界

| 完整任务 | 感知链路 | 用时（墙钟） | 两次导航位置误差 | Gazebo 放置误差 |
|---|---|---:|---|---:|
| transport01 | 原高度链路 | 199.99 s | 1.431 / 1.170 mm | 0.537 mm |
| transport02 | 修复高度链路 | 253.25 s | 0.364 / 0.632 mm | 0.428 mm |

两次均为 SUCCEEDED / PLACED，45 条事务事件，底盘/机械臂通过真实 ROS control 和 MTC 执行，载荷使用逐物理步的运动学附着。不是接触抓取力或真机精度证明；随机机械臂路径、感知负荷与同机运行负载不同，不将耗时差全归因于此次修改。

同步证据分开记录：物理步原生 parent/payload 表面误差界最大分别 **0.598 / 5.040 mm**，低于任务 6 mm 门限。ROS/TF 与原生载荷同采样时间对照最大平移误差分别 **2.033 / 19.054 mm**；第二轮峰值出现在机械臂快速运动，伴随少量 TF 等待不足的不可比较样本，差异原因尚未闭环。不能用原生插件自检代替 ROS 几何时序验收。运动样本使用预期位置速度 >5 mm/s、相邻源时间 <50 ms 筛选，详见 measured_metrics.json。第二轮探针较任务早启动并先结束，未宣称覆盖全部尾部放置阶段。

自动检查：76 搬运/VLA/同步、24 公共几何/投影、22 固定包络/通道、6 隔离/launch、3 原生 CTest，共 **131 项通过**。另外 1,200 个策略组合与 200 个几何组合。曾遇到合并 pytest 目录中同名 test_geometry 导入冲突，按包分别运行后通过，未把失败收集算作检查通过。

共享源码在运行期间另有改动：`source_changes_since_p0.json` 中包含本轮未编辑的 path_tracking CMakeLists、policy_lease.hpp、arrival_controller.cpp 与 transport_support.launch.py。未覆盖、回退或混入正在运行的原生库。`validated_native_binaries.json` 保留实际加载库路径及 SHA256；原生 CTest 对应本轮使用的 continuation 构建，不能据此声称其他任务后来修改的原生源码已经验收。

仍未放行：通道内部毫米/0.1°精确停靠；任意 mesh 附着；带载通道、多姿态实体通道统计回归；候选规划器闭环倒车；ROS/TF 快速机械臂几何一致性峰值；真机全高度覆盖、保持、制动和接触抓取。上述项维持拒绝/实验边界，不降低碰撞或到位门限换取“成功率”。

## 复现入口与运行产物

隔离启动示例（先构建选择的 overlay，再加载 MTC 已有依赖）：

```bash
source /tmp/codex_nonhome_height_install/setup.bash
python3 tools/sim_stack_supervisor.py \
  --instance nonhome_p0_p5_20260919 --ros-domain-id 74 \
  --mode baseline --navigation-geometry-mode fixed_v2 --navigation-policy p5 \
  --max-linear-speed .2 --headless --no-rviz \
  --map-yaml maps/warehouse_baseline.yaml --log-dir <新的证据目录>
```

技能/任务终端须先 source 实例的 env.sh，再 source 本轮 overlay 与 `tools/setup_mtc_humble.sh`。本轮使用独立 `/tmp/codex_nonhome_isolated_install`、`/tmp/codex_nonhome_height_install` 构建前缀，没有覆盖其他会话已加载的库；常规 `ws_robot/install` 不等于本轮已更新。任务命令是 `ros2 run astribot_s1_transport transport_task`，Python backend 模块本身不提供 `python -m` 入口。

只读规划器复现：`cmake -S tools/sim/planner_benchmark -B <独立构建目录>` 后构建，传入冻结 planner_input JSON、输出 JSON 和本机安装的 Omni primitive 文件。工具不启动控制器、不发包络 ACK、不向仿真发运动命令。

未提交或推送共享仓库。临时夹具只按本任务生成的实体名删除；进程停止只依据会话根、PID/启动时间和 pidfd，不使用全局清理、SHM 清理或跨实例 daemon 操作。

收尾：stack01、stack02 及本任务技能/支持节点已按所有权停止，两个 cleanup_result 均无剩余拥有进程。`final_process_count.txt` 与 `final_owner_audit.json` 保留结束时审计，另一 Domain 67 会话仍在。结束审计未见 Domain 213，本任务没有向其发送停止指令；不把早期存活记录当作当前状态。隔离入口与构建产物保留，后续可复用同一世界启动新证据目录。
