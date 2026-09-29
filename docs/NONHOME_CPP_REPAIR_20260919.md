# 非 home 导航与搬运：C++ 迁移及验收修复

证据根目录：`runs/nonhome_cpp_repair_20260919/`。本页补充
`NONHOME_FOLLOWUP_20260919.md`，保留此前失败记录。仅在导航仓库的隔离实例
`nonhome_cpp_repair_20260919` / ROS Domain 74 / Ignition partition
`astribot_nonhome_cpp_repair_20260919` 中验证；未连接真机。

## 已实施

1. 新增仓库技能 `.agents/skills/robot-runtime-cpp/SKILL.md`，并从栈操作技能链接：
   运行时优先 C++，Python 主要用于启动、验证与分析，现有适配层逐步迁移。
2. `astribot_s1_robot_geometry` 新增 pybind11 C++ 内核：完整凸包与 AABB 距离、
   凸包生成、扫描覆盖整个占据格的批量清除检查。现有 ROS 适配层调用内核，
   计算期间释放 GIL。未知、NaN、遮挡、角度空隙与不完整视场均不能清除障碍；
   保留逐格采集时间、原有留存规则和完整填充多边形碰撞语义。
   后续补充扫描投影、静态地图过滤和占据格生成的 C++ 批处理：保留原有地图
   五乘五邻域、范围边界和格子身份，仍使用采集时刻 TF，不扩大传感器有效期。
3. 逐关节状态缓存迁到 C++。DDS 数据先于 `/clock` 到达时，有界排队后再应用；
   不修改采集时间，不延长旧采样有效期。仍逐关节检查完整性、300 ms 时效、
   100 ms 关节间时差；回退清空缓存并递增 epoch。畸形消息与队列溢出仍拒绝。
   搬运协议绑定使用同一 C++ 源时间选择规则，缓存最多 64 条 ROS 消息，
   保留任务原有 10 ms 未来容差与观测年龄上限。
4. C++ 到位控制按剩余平移距离协调航向收敛：采用有界的 `2*v/d` 增益，
   上限 3/s，保留制动补偿、速度上限、平移门禁、完整扫掠检查与停稳窗口。
   终端零指令锁存后仍禁止在通道内重新原地修正。
5. 固定姿态模式的直线候选生成采用 0.05 rad 接近航向窗口；旧模式仍为 1°。
   候选必须通过原有完整凸包扫掠、未知栅格和路径位移限制，实际通道准入仍单独检查。
   目标航向、2 mm / 0.1° 到位容差及碰撞净空预算没有改变。
6. 复用共享仓库已有的 C++ MTC 关节限位余量实现，独立构建并检查：默认 0.1 rad，
   约束 IK、连接与 Cartesian 阶段，并复核输出路点。该实现由另一项工作提供；
   本轮未重写它，也不将避开硬限位分支表述为底层物理限位动力学已修复。

Python 仍承担 ROS 协议适配、融合和部分策略逻辑。本轮是分步迁移，没有完成整条
导航／搬运运行时的 C++ 替换。

## 同输入对照与回归

- 技能格式检查通过。
- 最新几何／策略 78 项、搬运 86 项 Python 验证通过。
- C++ 导航 4 项 CTest；MTC 守卫和关节余量 2 项 CTest 通过。
- 比较覆盖全／部分视场、遮挡、无效射线、不同旋转、退化箱体、凸包内障碍、
  微米级外凸点、异步局部关节更新、时效精确边界、时钟回退和未来消息洪泛。
- `kernel_benchmark.json`：2,117 个格子的清除检查中位耗时 26.8 → 0.52 ms。
- `actual_polygon_benchmark.json`：使用录制的 90 顶点带载包络和相同合成障碍输入，
  2,117 组距离计算中位耗时 30.6 → 6.17 ms；凸包生成 6.95 → 0.113 ms。
  这是内核对照，不是完整传感器回放或端到端截止时间验收。
- `scan_projection_benchmark.json`：同一 723 射线合成输入的扫描投影与格子生成
  中位耗时 4.17 → 0.104 ms。80 组旋转地图／三维外参及无效射线差分测试一致。

## 隔离仿真记录

| 用例 | 结果与边界 |
|---|---|
| transport01 | 搬运姿态阶段因关节时间戳超前本地时钟 16 ms 被取消；ATTACHED 保留，执行取消已确认。引出源时间缓存修复。 |
| transport02 | 抓取、抬升、姿态确认及首段导航通过；首站 0.465 mm / 0.0177°。后续包络以 GEOMETRY_INPUT_CHANGED_DURING_COMPUTE 撤销，安全保载。该失败引出逐关节 C++ 缓存迁移，不能算完整搬运成功。 |
| stack03 | Gazebo 的 robot_description 读取响应超时，物理／数据未就绪。辅助节点曾因环境文件尚未生成而进入默认 Domain；运动验证脚本拒绝执行，随后按日志、PID 和启动时间清理自有节点。之后统一使用环境缺失即退出的脚本。 |
| internal04 | 1.30 m 实体通道内部终点通过；1.127 mm / 0.0633°，0.62 s 停稳窗口。0.85 m 及反向终点拒绝；停止保持续租后，期限后 17 ms 撤销，ARM_HOLD_EXPIRED。 |
| internal05 | 1.10 m、入口横向偏移 15 mm、航向偏移 −0.03 rad。通道内路线最大航向偏差 0.05423 rad，超过 0.05 rad，未进入通道，明确返回 CORRIDOR_BLOCKED: CORRIDOR_OFFSET_ROUTE_REQUIRED。占据预测也提示净空不足；未降低安全预算。该记录早于直线候选修复。 |
| transport06 | MTC 动态库路径缺失，准入返回 MTC_SERVER_UNAVAILABLE，未执行抓取。补齐 setup_mtc_humble.sh 并通过 ldd 核验后再开启新用例。 |
| transport07 | 抓取、抬升、姿态确认与首站通过，首站 0.379 mm / 0.0220°。第二段因 REQUIRED_COVERAGE_UNAVAILABLE 安全停止；ATTACHED 保留、stop_error 为空。完整搬运不通过。 |
| internal08 | 入口横向偏移 15 mm、航向偏移 −0.03 rad，1.30 m 通道执行通过；1.017 mm / 0.0478°、0.60 s 停稳。实际采用 aligned_direct_validated 候选；0.85 m 和反向终点继续拒绝。停止续租后期限后 7 ms 撤销，ARM_HOLD_EXPIRED。 |
| transport09 | 最新 C++ 扫描内核下完整搬运成功，墙钟 293.39 s。首站 0.662 mm / 0.0291°，第二站 1.434 mm / 0.00567°；均经过原有停稳窗。物体脱附为 PLACED、机械臂撤退收拢，最终 SUCCEEDED。 |

`internal04` 的里程计／预留凸包对已知墙体平面距离最小 239.18 mm，高于每侧
80 mm 预算；墙体区段内未观察到平移指令小于 5 mm/s 时的转向指令。
这属于命令及几何证据，不是 Gazebo 接触力或真机外部测量。

`transport02` 包络就绪期间 390 个观测周期中 363 个有效，扫描年龄 P95 328 ms，
仍存在超过 300 ms 的周期。同机编译期间墙钟与仿真时间比例变化明显，不能将
内核加速等同于全部负载下实时性通过；原始记录及 CPU／墙钟指标均保留。

`transport07` 的完整记录中，包络就绪的 1,592 个观测周期有 890 个有效，扫描
年龄 P95 486 ms。扫描角度覆盖仍为全周，主要失败证据是计算完成后的采集时效
过期，而不是相机外参或视场缺口。2,832 个占据格的合成快照剖析显示，Python
快照构造、逐条契约检查和属性读取仍有明显开销；扫描 C++ 迁移仅处理其中一段，
不能宣称融合与策略整体实时性已经修复。见 `runtime07_analysis.json`、`fusion_profile.txt`。

`internal08` 包络就绪的 395 个周期全部有效，扫描年龄最大 250 ms。对已知墙体
及预留凸包的 1,000 个姿态采样，最小几何距离 229.94 mm；墙体区段内低于平移
门槛的转向指令为 0。该用例仅证明这一负载与初始位姿下通过，不替代带载或多负载
实时性验收。分析脚本为 `tools/sim/analyze_nonhome_runtime.py`。

`transport09` 的执行守卫记录 3,295 条正常执行采样和 15 条初始等待证据采样，
无越界故障；正常采样最大关节跟踪误差 0.03632 rad（限值 0.05）、底盘平移
0.0170 mm（限值 20 mm）、旋转 0.0000543 rad（限值 0.02）。物体放置后的
Gazebo 位置误差 0.0331 mm；该值属于仿真内部真值，不代表外部物理测量精度。
记录中的 6,050 个同物理步载荷样本最大跟随平移误差 0.2181 mm；与 TF 目标的
最大平移差 0.00226 mm。未事后调整时间戳；`force_grasp_validated=false`。

完整流程成功不等于实时性全部通过：`transport09` 包络就绪的 936 个观测周期
中 820 个有效，扫描年龄 P95 370.5 ms、最大 1.352 s，超时期间仍暂停而未绕过
保护。不同运行的负载与路径阶段不一致，因此不能把 transport07 → transport09
直接作为单变量性能因果试验。C++ 批处理的等价性与耗时由同输入离线测试证明，
高负载下融合、快照与策略的剩余运行时仍需继续迁移及验收。

## 带入口偏差的规划器对照

使用 `internal08` 的四份实测代价图、安装后凸包及几何版本，通过现有 C++
`planner_benchmark` 对照。逐份校验结果中的几何哈希与输入一致；保留输入文件摘要。

| 场景 | 部署 Exact 2D | 原生 Smac 2D | Hybrid 72 bins | Omni Lattice |
|---|---|---|---|---|
| 0.85 m、对齐终点 | 拒绝 | 无合格路径 | 无合格路径 | 无合格路径 |
| 1.10 m、对齐终点 | 几何候选通过 | 几何候选通过 | 几何候选通过 | 几何候选通过 |
| 1.30 m、对齐终点 | 几何候选通过 | 几何候选通过 | 几何候选通过 | 几何候选通过 |
| 1.10 m、反向终点 | 拒绝 | 完整扫掠不通过 | 几何候选通过 | 几何候选通过 |

结果见 `planner_comparison08/summary.json`。所有候选使用同一保守填充凸包、未知格
拒绝和扫掠检查；没有通过降碰撞门槛提高成功率。原生候选约 25.5 mm 的格网终点
误差只用于离线搜索对照，不能替代 2 mm 到位验收。Hybrid／Lattice 的反向候选
仍需验证倒车许可、整机航向、退出通道及动态制动，因此没有切换默认规划器或启用
自动后备执行。

## 运行隔离与可复核性

- `stack01`–`stack09` 自有实例均已停止，逐次清理结果为 `remaining=[]`。
  `final_process_count.txt` 仍列出其他任务的进程；未清理它们或全局 DDS 资源。
- 运行节点的真实 Domain、Ignition partition、PID／启动时间及已映射原生库摘要
  保存在 `runtime_identityNN.json`；最新源码清单为 `source_manifest09.json`。
- 最终组合为 `/tmp/codex_nonhome_cpp_v5_install`、
  `/tmp/codex_nonhome_corridor_cpp_install`、`/tmp/codex_nonhome_margin_cpp_install`，
  并显式加载 `tools/setup_mtc_humble.sh`。各次栈的参数与生命周期见 `stackNN/session.json`。
  这些是本机验证覆盖层，不是已发布的全新机器安装包。
- 最新完整搬运日志为 `runs/nonhome_cpp_repair_20260919/stack09/session.log`；
  `latest_sim` 仅为日志索引，不表示栈仍在运行。

## 尚待放行

本轮已通过一组完整带载搬运；通道内部终点已覆盖对齐和带入口偏差两个初始位姿。
多负载截止时间、动态遮挡／高度覆盖、候选规划器执行对照与任务进程失联的独立保护
仍需分别验收。真机相机外参、保持及制动须在用户开机后单独验证。

下一步优先迁移已测到的融合与快照热点到 C++，保留源采样时间、障碍留存和同版
包络确认；通过同输入差分后，再跑带载通道、动态遮挡及负载矩阵。当前 P0–P5
尚不能整体放行；本页中的单次成功不覆盖历史失败记录。

## 融合快照热点迁移（Phase A，2026-09-20）

### 已实现

1. `astribot_s1_robot_geometry` 新增纯 C++ 快照推导内核
   `include/astribot_s1_robot_geometry/fusion_snapshot.hpp`：对每条航迹只计算
   派生量——`age`／`old`（预测速度归零）、平移 `min(age, track_memory)`、
   协方差膨胀 `min(age,3)²×age_variance`、stationary 检测、预测方差、region
   相关半径。内核不拥有时钟、权限或 lease；调用方保留 `ConservativeFusion`
   的状态（tracks、epoch、sequence、sensors）。
2. pybind11 绑定 `snapshot_tracks`（32 列行布局 + 变长 sample 数组），逐条对应
   `fusion.py::snapshot()` 的算术：capture 时间、时钟 epoch、几何／包络版本、
   协方差保留、障碍留存（`spatial_occupancy` 不移动且方差置 0）、未知拒绝、
   `occupancy_only` 由 `velocity_m_s` 决定（镜像参考所读的同一字段，而非借用
   `velocity_covariance` 的间接不变量）、region 过滤、旧航迹预测速度归零。
3. `fusion.py` 增加 `snapshot_native()` 薄适配：构造 32 列 numpy 行与变长 sample
   数组，调内核后用 `_new_frozen`（`object.__new__` + `object.__setattr__`）重建
   冻结数据类，跳过 ingest 期已完成、快照无需重复的 `require`/`label` 校验。
   epoch 变化仍走 `update((), now)`，与 `snapshot()` 一致；原生模块缺失时显式
   抛错，不静默降级回 Python。
4. 测试：C++ CTest `fusion_snapshot_test`（空输入、NaN 拒绝、未来、旧、新鲜、
   占据格、`has_velocity_m_s` 映射守卫、region）；pytest 差分
   `test_fusion_snapshot.py`（结构化 + 200 条随机 + 空 + epoch 清空 + 内核拒绝
   NaN／Inf／坏形状），逐字段比对两路径输出。

### 已验证

- 独立构建 `/tmp/fusion_cpp_build → /tmp/fusion_cpp_install` 成功。
- CTest 2/2 通过（`filled_collision`、`fusion_snapshot`）。
- pytest 差分 6/6 通过；既有内核回归 `test_native_kernels.py` 18/18 通过，
  新增 `snapshot_tracks` 不影响既有箱距／凸包／扫掠内核。
- 微基准（2,832 条合成航迹，同输入）：`snapshot()` 中位 33.7 ms →
  `snapshot_native()` 中位 29.7 ms（min 30.8 → 23.6 ms），约 23% 墙钟下降；
  两路径逐字段等价、航迹数一致（2,832）。

原始日志在 `runs/nonhome_cpp_repair_20260919/fusion_snapshot_phase_a/`
（`fusion_cpp_build.log`、`fusion_cpp_ctest.log`、`fusion_snapshot_pytest.log`）。
以上是同输入内核等价性对照，不是完整传感器回放或端到端截止时间验收。

### 未通过 / 未验收

- Phase B（感知覆盖鲁棒性）、Phase C（规划器／窄通道回归）、Phase D（隔离仿真
  端到端）本轮未在 live 隔离 Gazebo 实例上重跑，仍为 未验收。
- `snapshot_native()` 是增量新增的可选路径，`update()` 与现有调用方仍走
  `snapshot()`；把该路径接为默认返回需在 Phase D live 仿真回归后再做。
- 微基准里快照构造仍占主导：约 23% 增益来自去掉逐条契约校验；对象重建
  （MetricBox／Vec3／Covariance3／PredictionModel／TrackedObstacle）仍在 Python
  侧，没有被消除。不能据此宣称融合与策略整体实时性已修复；
  `transport07`／`transport09` 的扫描年龄超限记录与结论仍有效。
- 高负载下其余 Python 热点（预测行、风险几何、策略输入）尚未迁移。

## Phase B/D 追加验证（2026-09-20）

### 快照内核边界修复

补充修复了 C++ 快照内核的运行时边界：运行链路使用整数纳秒采集时间，避免
超过 2^53 后由 double 转换造成 lease 边界改变；无整数时间传输时，超过精确
double 范围的兼容输入会拒绝。补充有限值、正尺寸、协方差对称/半正定、布尔标志、
区域半径溢出和采样偏移检查，并在内核计算时释放 GIL。

差分测试 8 项、全几何/策略测试 84 项、C++ CTest 2 项通过。实测 2,832 条
合成航迹上，当前 `snapshot_native()` 包含 numpy 行打包和 Python 冻结对象重建，
中位约 21.2 ms，对照 Python 参考约 16.0 ms；因此没有接为默认运行路径。该内核
仍是已验证的迁移候选，待把对象重建和风险计算一并下沉后再重新评估。

### 动态障碍与独立保护

在隔离实例 `nonhome_next_20260920`（ROS Domain 76、独立 Ignition partition）中：

| 注入场景 | 结果 |
|---|---|
| 动态障碍 | `YIELD`，路径状态 `OCCUPIED`；首个零指令 0.305 s，稳定停止 0.880 s，最大故障后位移 31.2 mm，无反弹，保护净空下界 127.9 mm，PASS |
| geometry_state 进程暂停 | `GEOMETRY_EXPIRED`；首个零指令 0.248 s，稳定停止 0.841 s，无反弹，PASS |
| policy_controller 进程暂停 | `WAITING_FOR:policy`；首个零指令 0.226 s，稳定停止 0.877 s，无反弹，PASS |

原始证据分别位于 `moving_obstacle06/summary.json`、`geometry_fault07/summary.json`
和 `policy_fault08/summary.json`。这些是独立保护与控制停止证据，不是接触力或实机
制动距离验收。

### 高度覆盖结果

早期 `height11/height12` 保留了两个独立问题：验证器曾把世界坐标高度直接与
`astribot_torso_base` 切片上限比较，且 MPPI 栈曾出现 `controller_server` 心跳
丢失。前者会把 2.5 m 目标误判为不在投影范围，后者属于仿真运行稳定性问题，
不能混入感知覆盖结论。

本轮在隔离实例 `nonhome_height_rpp_20260920`（ROS Domain 78、独立 Ignition
partition、RPP 控制器）完成修复后的 `height06/summary.json`：

| 世界高度 | 基座切片 Z 包络（m） | 新鲜点云 | 代价图由 105→254 | 判定 |
|---:|---:|---:|---:|---|
| 0.10 | -0.2083…0.0017 | 是 | 是 | 通过 |
| 0.60 | 0.2914…0.5014 | 是 | 是 | 通过 |
| 1.20 | 0.8911…1.1010 | 是 | 是 | 通过 |
| 1.90 | 1.5907…1.8007 | 是 | 是 | 通过 |
| 2.50 | 2.1904…2.4003 | 是 | 是 | 通过 |

验证器现在同时记录世界坐标与基座坐标包络，按 TF 旋转后的完整盒体 Z 投影与
`[-0.03, 2.2] m` 切片范围比较；基线代价固定取夹具创建前的空闲样本，避免点云
清除延迟把新夹具误记为“创建前已占用”。完整点云输入为 `/map_scan`；
`/map_scan_filtered` 仍保留 Voxel-SLAM 历史 `nav_scan_z_max=1.534 m` 裁剪，
因此不能用后者替代固定版本的高度覆盖证据。

RPP 栈在 height06 后进行约 20 s 稳定性采样：生命周期节点全部 `active`、
`/clock` 持续有数据、日志无 `controller_server` 心跳失败。验证结束后按记录的
supervisor/skills PID 发送 SIGINT，`session.json` 为 `stopped` 且
`remaining_owned_pids=[]`；实例环境扫描为 0 个残留进程。height03（未继承
Ignition 分区）、height04（TF 尚未连通）和 height05（基线时序误判）作为工具
边界失败证据保留，不能覆盖 height06 的通过结果。

随后在全新隔离实例 `nonhome_mppi_stability_20260920`（ROS Domain 79）运行
MPPI 空闲稳定性对照约 185 s：七个导航生命周期节点持续 `active`，日志中
`Have not received a heartbeat` 与 `CRITICAL FAILURE: SERVER controller_server`
均为 0；实测 `/clock` 与 `/scan_from_cloud` 均有持续数据。该结果说明 height12
的心跳故障不是确定性复现，但尚未覆盖运动负载、长时间传感器压力或进程失联注入，
因此仍作为“待根因定位”的可靠性问题保留，而非宣称修复。

在同类隔离 MPPI 实例上发送 0.5 m 裸导航目标的运动入口探针被安全门拒绝，
`mppi_motion_summary.json` 记录原因 `ENVELOPE_V2_NOT_READY`、路径点数为 0、
`controller_server` 心跳失败为 0 且生命周期仍为 `active`。这确认未建立非 home
几何租约时不会进入控制器执行；该拒绝是预期安全行为，不能作为 MPPI 运动成功，
也不能把裸导航动作当作搬运动态回归。
该 MPPI 运动探针随后按 supervisor PID 清理，`session.json` 为 `stopped`、
`remaining_owned_pids=[]`，实例环境扫描无残留。

离线多场景矩阵 `offline_nonhome_navigation.json` 覆盖 200 个几何组合和 1,200
个生产 `FixedCorridorPolicy` 决策：126 个组合几何可通过、74 个拒绝；输入或
几何无效时均保持 HOLD，通道内没有 ALIGN/CENTER 原地转向。该矩阵是规则与故障
语义回归，仍不替代已建立包络后的 live MPPI/RPP 运动回归。

代码回归补跑结果：导航策略 31 项、几何 55 项、搬运 86 项 Python 测试均通过；
融合快照差分 8 项通过，`filled_collision` 与 `fusion_snapshot` 两项 CTest 通过。
合并收集多个测试目录时存在同名 `test_geometry.py` 的 pytest 模块冲突，已按包
分开执行，未把该收集器问题计入运行时失败。

### 当前剩余工作

- 将 snapshot 的对象构造、预测行和风险几何继续迁移到 C++，直到真实回放中 P95
  处理时间和采集时效回到门槛内，再接入默认路径。
- 高度投影与 1.9–2.5 m 覆盖在 RPP 隔离实例已通过；遮挡、稀疏点云和 VoxelLayer
  参数矩阵已在新的隔离域完成，但双 RGB-D 源持续新鲜度仍未达到放行门槛。
- 在同一几何版本与包络下补做端到端多负载抓取／搬运／放置；保护性 HOLD/拒绝继续
  算安全结果，不算任务成功。

### 当前阻塞与放行边界（2026-09-20 追加）

- MPPI 隔离栈在 `height12` 出现过 `controller_server` 心跳丢失并主动降级；
  RPP 对照栈在 `height06` 已稳定通过，但 MPPI 的复现、根因定位和恢复策略仍未
  完成，不能把 RPP 结果写成默认控制器的稳定性结论。
- C++ `snapshot_native()` 的边界与等价性已通过，但含 numpy 打包和 Python 冻结
  对象重建时仍慢于参考路径，不能接为默认运行时；预测行、风险几何和对象构造尚未
  全部下沉。
- 动态障碍、几何过期、策略暂停的保护停机，以及遮挡／稀疏／VoxelLayer 安全保持
  已通过；端到端多负载搬运在 READY_RIGHT skill planner 无解处停止，Hybrid/Omni
  候选的 live 执行回归仍未完成。历史完整搬运成功不替代这些验收。
- VLA 通用接口和 55 项接口回归已完成，但当前仍是 Python 适配层/示例策略；真实
  模型服务、完整 VLA 抓取搬运放置动态回归、动作块执行和真机相机外参均未放行。

## 传感器 profile 与 MPPI 运动复核（2026-09-20 追加）

### 已实现

1. 修复 `navigation.launch.py` 的参数重写范围。此前通用 `topic` 重写会递归改写
   头部／躯干点云话题，导致 `PointCloud2` 被错误接到 `LaserScan` 订阅，局部代价图
   不会配置。现在只重写两个 obstacle-layer 的 `scan.topic`，深度点云保持原消息类型
   和话题。
2. 为 `nav2_full_bringup.launch.py`、`perception_slam_bringup.launch.py` 和
   `sim_stack_supervisor.py` 增加一致的传感器开关：`use_lidar`、`use_camera`、
   `use_camera_postprocess`、`use_camera_pointcloud`、`enable_depth_obstacles`。默认
   保留激光、RGB-D 和三维障碍门槛；只有明确的 lidar-only 回归才关闭相机与深度层。
   `enable_depth_obstacles=false` 只改变代价图观测源，不放宽包络、碰撞或策略安全门。
3. 运行时仍遵守 C++ 优先：上述改动只负责启动与参数接线，几何和碰撞内核继续由
   `astribot_s1_robot_geometry` C++ 实现提供，Python 仅作为启动／验证适配层。

### 验证证据

- `astribot_s1_navigation`、`astribot_s1_perception` 定向构建通过；三份 launch 文件和
  `sim_stack_supervisor.py` 通过 `py_compile`，`git diff --check` 通过。
- 隔离实例 `nonhome_mppi_lidar_only_rtf05_20260920`（ROS Domain 86、RTF 0.5、P5、
  MPPI、固定 V2）完成包络握手和一次 1 m 运动：
  `runs/nonhome_mppi_lidar_only_rtf05_20260920/envelope_motion/summary.json` 中
  `envelope_ready=true`、`navigation_succeeded=true`、`hold_revoked=true`，固定几何
  哈希为 `44e83422d77fd8980202342934a6f670fcabcc4c6d7b5e09fc98100a287a3b2b`，最终平面
  误差 `0.001135 m`，严格 2 mm／0.1° 到位判据通过。日志包含 `ARRIVAL_REACHED`，导航
  生命周期在采样时均为 `active`。
- 同一实例的 `/clock` 约 495 Hz、`/scan_from_cloud` 约 5 Hz；
  `/navigation_policy/costmap_scan` 为 `LaserScan`，两个 costmap 订阅者均为
  `BEST_EFFORT`；关闭相机后头／躯干点云话题无发布者，证明 lidar-only profile 没有
  隐式接入失配的 RGB-D 话题。
- 早先默认实时因子 1.0 且同时存在多个自有栈时，运动探针分别出现
  `WAIT_TIMEOUT` 与 `SIM_CLOCK_STALLED`。清理自有旧实例后以 RTF 0.5 重跑通过；这两条
  记录保留为负载边界证据，不改写为控制器碰撞失败。一次 `REQUIRED_COVERAGE_UNAVAILABLE`
  也被策略保持为 HOLD，说明激光覆盖过期时不会以降低门槛换取运动成功。

### 当前剩余工作

- 上述运动成功是明确标注的 lidar-only profile，不包含 RGB-D/VoxelLayer 三维覆盖验收；
  默认相机 profile 仍需在低负载隔离实例补做新鲜点云、稀疏／遮挡和窄通道矩阵。
- VLA 真实模型服务、完整抓取／搬运／放置动态回归和真机相机外参按当前任务继续跳过；
  其接口静态回归不等于真实服务验收。

默认相机 profile 的只读检查在隔离实例 `nonhome_camera_profile_20260920`（ROS Domain
88、RTF 0.5）完成：头部和躯干 RGB-D self-filter 均发布 `sensor_msgs/PointCloud2`，
local costmap 的参数明确保留 `scan head_depth torso_depth`，头部点云订阅者的类型和
BEST_EFFORT QoS 与发布者一致，导航生命周期可进入 `active`。但该实例同时记录了
点云观测超过 0.30 s 的 warning，未达到三维感知新鲜度验收；实例已按 PID 清理，不能
把“话题接通”写成“点云覆盖通过”。

## RGB-D／遮挡／VoxelLayer 与多负载回归（2026-09-20 追加）

### 三维覆盖实现

在现有导航参数中补齐了 VoxelLayer 的可选 profile，不复制第二套导航栈：
`obstacle_layer_plugin:=nav2_costmap_2d::VoxelLayer` 时同一 local/global source
块切换为 VoxelLayer，并启用 `origin_z=0`、`z_resolution=0.10 m`、`z_voxels=32`、
`max_obstacle_height=2.0 m`、`mark_threshold=0` 和 `publish_voxel_map`；默认值仍为
`ObstacleLayer`。`sim_stack_supervisor.py`、`nav2_full_bringup.launch.py` 及
`navigation.launch.py` 已透传该选择，深度源仍严格使用 `PointCloud2`。新增的
`tools/sim/verify_3d_coverage_matrix.py` 只负责观测和证据，不把点云缺失解释成安全
通过。

### 隔离场景结果

| 场景 | 隔离实例与证据 | 结果 |
|---|---|---|
| VoxelLayer + RGB-D | `coverage_voxel_20260920`，Domain 91，RTF 0.5，七个 Nav2 生命周期均 `active`；`coverage_voxel/summary.json` 记录 `/local_costmap/voxel_grid` 74 条、代价图 12 条，参数 dump 明确为 `nav2_costmap_2d::VoxelLayer`、32 层 | `PASS`：体素链路已发布；头部点云仅有空样本、躯干只收到一帧非空，因此不把它写成完整双目新鲜度通过 |
| 遮挡 | 同一实例临时创建并在探针结束时删除的 Gazebo 静态遮挡体；`coverage_occluded/summary.json` | `PASS_SAFE_HOLD`：点云覆盖退化时保持 HOLD，VoxelGrid 仍有 55 条；没有用遮挡后的残留体素放行 |
| 稀疏／无 RGB-D | `coverage_sparse_20260920b`，Domain 93，`use_camera_pointcloud=false`，VoxelLayer 仍有 56 条输出；`coverage_sparse/summary.json` 头／躯干均为 0 点 | `PASS_SAFE_HOLD`：缺失三维输入没有被当作可通行 |
| 窄通道三维规则 | `runs/narrow_3d_matrix_20260920.json` | 200 个几何组合、1,200 个生产 `FixedCorridorPolicy` 决策；126 个可通过、74 个拒绝，通道内无 ALIGN/CENTER 原地转向，输入/包络无效时 HOLD |

一次同时启动第二个 Gazebo 实例的尝试在 `coverage_sparse_20260920` 被
`gz_ros2_control` 的 `robot_description` 加载期阻塞，监督器按超时退出且
`remaining_owned_pids=[]`；这属于同机并发调度边界，不能归因给 VoxelLayer。后续分开
实例重跑即得到上表的 sparse 结果。

### 多负载搬运动态回归

新增 `tools/sim/verify_multi_load_dynamic.py`，复用生产 `FixedEnvelope` 协调器回放
空载、0.2 kg 轻载、1.2 kg 偏置重载和释放后的空载四个连续几何版本。证据
`runs/multi_load_dynamic_20260920e/summary.json` 显示每一载荷均获得新 epoch 与几何
hash，包络半长从 0.3301 m → 0.4001 m → 0.5001 m 后恢复，附着载荷质量严格为正，
所有消费者 ACK 后才 `transport_ready=true`；缺 `controller` ACK 得到
`WAITING_FOR:controller`，未确认 hold 被拒绝为 `ARM_HOLD_UNCONFIRMED`。

同时启动真实 `transport_task`（无 VLA）时，搬运事务已通过 ADMISSION、固定包络与
场景同步，但在 `READY_RIGHT` 的 MoveIt skill planner 返回
`RETRIES_EXHAUSTED ... planner returned no solution`，因此没有把这次运行记作抓取／
搬运／放置成功。原始记录在 `runs/transport_multi_load_20260920/nominal_retry/`；
MTC 首次启动缺少 `librviz_marker_tools.so`，补齐隔离依赖路径后 MTC 进程正常启动，
剩余阻塞是 READY_RIGHT 规划无解。该项仍是完整动态搬运的明确未放行卡点，不影响上述
包络事务回归和安全 HOLD 结论。

### 本轮验证边界

- `astribot_s1_navigation` launch 回归 3/3、搬运事务／payload／geometry 回归 26/26，
  `verify_multi_load_dynamic.py` 通过；覆盖采集器、窄通道矩阵和各隔离 supervisor 均在
  结束后核对 `session.json` 为 `stopped` 且 `remaining_owned_pids=[]`。
- RGB-D 的真实双源持续新鲜度、遮挡后的可见体积证明、真实接触力以及 VLA 服务仍未放行。
  完整抓取／搬运／放置还需先解决 `READY_RIGHT` 规划无解，再按同一多负载矩阵重复端到端
  事务；本轮没有用降低碰撞或时效门槛来掩盖该失败。
