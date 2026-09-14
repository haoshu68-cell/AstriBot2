# 路径航向收敛优化

## 检查范围与原因

检查了 ExactGoalPlanner 的路径方向生成、ThreePhaseController 的起步/跟踪/终点切换和惯性补偿、ArrivalController 的急弯限速与终端精调、MPPI 的 PathAngle/PathAlign/CurvatureSpeedLimit 评分以及 velocity_smoother 的加减速约束。

发现起步对齐的触发与完成判据混用：进入 ALIGN_START 后，只要误差低于 `start_min_angle=0.2 rad` 就退出，因此实际约 11° 就开始平移；配置的 `align_tolerance=0.02 rad` 没有决定已开始对齐的完成时机。全向 MPPI 可以通过横移追踪位置，残余车头偏差还需后续收敛。

FOLLOW 的 PathAngleCritic 朝向同一个前方路径点，不等同于逐点切线约束；优化前 PathAlignCritic 只评估位置。已核对 [Nav2 Humble PathAngle 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/path_angle_critic.cpp) 和 [PathAlign 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/path_align_critic.cpp)，以及本机安装头文件。前方点方位和当前位置路径切线在弯道上不同，不能把所有偏差归因于角速度上限。

## 保留的修改

- 在 ThreePhaseController 中记录本次起步是否实际触发对齐。未触发时保留小误差直接 FOLLOW 的行为；已触发则必须达到原 `align_tolerance`，并在启用惯性补偿时达到原 `align_settled_wz`，才进入 FOLLOW。新目标重新初始化这一状态。
- 起步判据修复本身不修改原有转向增益、直线及角速度上限、惯性模型、MPPI 权重、全向能力、碰撞检查、终端 3 cm/1.5° 判据与事件重规划策略。
- 跑机指标增加 `heading_convergence.first_stable_5deg_s`：从首次行进 FOLLOW 到首次连续至少 0.5 s 小于等于 5° 的区间起点，未满足为 null；`longest_over_15deg_s` 记录行进大航向误差最长连续时间。排除终点朝向区，数据断档、阶段切换和路径版本切换切断连续累计。该指标是首次收敛，不能代替整段 P95 或后续弯道表现。
- 修复运行器 Ctrl+C 先销毁 ROS context、再取消 action 的顺序问题，保持 context 到取消请求及结果处理完成。

## 未采用的候选

在原栈运行时临时开启原生 `PathAlignCritic.use_path_orientations=true`，原权重 12 不变。前三点成功，第三段行进航向 P95 为 8.82°，原九轮该段中位值为 23.08°；但该段角加速度 P95 从基线中位 0.144 升为 0.245 rad/s²，jerk P95 从 0.649 升为 1.303 m/s³。第四段在碰撞重规划后出现 `PATH_GUARD: collision check unavailable for 2s` 并终止。未证明通信超时由此参数引起，但该实验不满足完整路线通过和综合效果不退化的要求，因此不写入默认配置。

记录：`/tmp/astribot_heading/orientations/`、原栈 `navigation_1.log`。候选仅为不完整单轮，不能据此宣称弯道整体优化完成。

## 验证

隔离编译与 83 项 C++ 检查通过，包括原有 81 项，以及实际控制器触发后不提前退出、新目标清除状态两项；5 项指标边界检查通过。临时验证程序放在 `/tmp/astribot_heading/regression/`，不新增仓库单元测试。构建与检查结果分别为 `/tmp/astribot_heading/build.log`、`regression_result.log`。

Gazebo/RViz 使用 0.5 m/s、仓库六点路线、真值定位。新库运行映射已确认来自 `/tmp/astribot_heading/install/astribot_s1_path_tracking/lib/libastribot_s1_path_tracking.so`。正式实验栈目录为 `/tmp/astribot_heading/start_fix_stack2/`；第一次隔离启动因调用时遗漏查询侧 domain 环境被主动停止，不计为控制效果失败。

修复前耐久以 55/55 个到点成功归档，提前终止，不能声称两小时通过。Ctrl+C 时旧运行器写出 cancel_error；Nav2 日志同时确认 Goal canceled 和控制器停止。新的信号处理会另外验证。

完整路线结果见下文。有限轮次和单地图仿真不证明所有工况性能均不退化。

## 普通弯道降速对照

先试过直接把共享 `a_lat_max` 从 0.35 改为 0.25，但这同时加强了候选轨迹 `speed*abs(wz)` 的惩罚，首段行进航向 P95 达到 61.98°、横向 P95 达到 72.30 cm。虽然两个已完成目标都到位，过程明显退化，已主动取消并撤回该方案。证据目录 `turn_speed/`，不计为降速通过结果。

第二个候选（已撤回）保持原 `a_lat_max=0.35`，单独增加 `path_a_lat_max=0.25 m/s²`，只用于前方路径曲率的制动包络和速度参考，`v_min_turn` 从 0.324 调为 0.25 m/s。未配置新参数时默认继承 `a_lat_max`，保持原评分；参数须有限、正且不大于 `a_lat_max`。既有 `soft_ratio=0.6`、前视时间、制动预算、评分权重、急弯处理以及终点附近退出规则不变。软速度参考为 `clamp(sqrt(0.6*path_a_lat_max/|κ|), v_min_turn, vx_max)`，在本次直线限速 0.5 m/s 下：

| 曲率 m⁻¹ | 原参考 m/s | 新参考 m/s |
|---:|---:|---:|
| 0 | 0.500 | 0.500 |
| 0.5 | 0.500 | 0.500 |
| 1.0 | 0.458 | 0.387 |
| 1.5 | 0.374 | 0.316 |
| 2.0 | 0.324 | 0.274 |
| 3.0 | 0.324 | 0.250 |

这是优化器中的软代价参考，不能当成实测速度或硬限速。角速度上限不变，通过提前降低平移速度增加跟随曲线的时间。参数在 critic 初始化时读取，修改后必须重新加载导航，单纯动态 set 参数不能证明已生效。

降速前基线为 `start_fix/` 两轮 12 点（含起步判据修复）；修正后目录为 `path_speed/`，由仓库正常启动脚本加载正式安装版本，保持同一地图、初始仿真和路线。额外离线统计“弯后 1 m”：参考曲率绝对值曾达到 0.35 m⁻¹，随后降至 0.1 m⁻¹ 以下，沿路径进度距最近弯道样本不超过 1 m 的行进样本；排除终点朝向区，并在数据断档或路径切换时重置。这个窗口是可重复的评价口径，不是控制器新增的切换条件。分析脚本与原始数据均保存在 `/tmp/astribot_heading/`。

### 起步修复单独验证结果

两轮共 12 点，12/12 到位合格，最大误差 2.283 cm / 1.316°；总目标执行时间 698.90 s。每个目标一次规划，没有 FOLLOW 异常旋转。大误差触发对齐后，退出残差由原记录约 11° 收紧到约 1°。原小误差直接进入 FOLLOW 的例外仍保留（本轮出现 8.25°、对齐耗时 0 s 的直接进入），不能把所有起步均描述成强制 1° 对齐。

这两轮尚有横向偏差告警；弯后 1 m 窗口航向 P95 41.59°、横向 P95 40.82 cm，作为后续降速的对照基线。Ctrl+C 实测结果为运行器 exit code 0、状态 stopped、action status 5（CANCELED），没有 cancel_error：`/tmp/astribot_heading/cancel_result.json`。


### 降速候选的最终判定

`path_speed/` 两轮 12/12 到位合格，最大 2.886 cm/1.445°，无 FOLLOW 异常旋转。但弯道横向误差 P95 从 21.43 cm 升至 26.99 cm，中位值从 3.71 cm 升至 7.08 cm，因此撤回 `path_a_lat_max` 和转弯速度下限改动，恢复原曲率 critic。否决依据是过程质量，运行时长不参与评价。

随后试过 0.35 m/s、开启路径方向评分并添加整段预测控制量差分代价。前三点均到位，但首段航向 P95 173.54°、横向 P95 210.5 cm，明显不可用。该代价项及配置、插件声明已全部删除。离线功能检查通过不能代替闭环效果验证。证据：`/tmp/astribot_heading/quality/`。

当前继续验证原生方向评分较早启用（offset 20→6）、平移正加速度限制 2.5→0.5 m/s² 的候选；减速度和角速度约束保持原值。尚未据此声明改善。Humble MPPI 本机约束结构与 [Humble optimizer 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp) 不读取 ax_max/az_max，因此本轮限制放在实际执行的 velocity_smoother。

评价按用户要求排除跟踪总时长，优先航向与横向误差、加减速过渡和终点接近段，保留 3 cm/1.5° 到位条件。低速试验增加运行期限，不通过放宽误差或取消异常检查换取通过。


## 本轮保留配置与完整复验

保留起步对齐退出修复，开启原生 PathAlignCritic 的方向评分，并将 offset_from_furthest 从 20 改为 6，使低速预测进度下仍可启用评分。velocity_smoother 的平移正加速度上限为 0.5 m/s²，制动与角速度约束保持原值。未增加新的 critic 或更改 MPPI 的控制公式。仿真启动器默认速度调整为本轮验证的 0.35 m/s，可用 --max-linear-speed 覆盖；直接 ROS launch 的速度默认值未改，本轮结果不外推到其 1.0 m/s 工况。

`native_quality/` 与起步修复基线 `start_fix/` 均为相同六点路线、两轮、真值定位；基线运行限速 0.5 m/s，候选为 0.35 m/s。这是整套调优方案对照，不把效果归因于某一个参数。跟踪总时长不参与选择。

候选 12/12 到位通过，最大位置误差 2.128 cm，角度误差 1.126°；无 FOLLOW 异常旋转，无无效采样。每目标一次规划，共 12 次，未触发定时重规划。

各路段两轮 P95 的中位值（前→后）：

| 段 | 航向 ° | 横向 cm | 平移 jerk m/s³ | 角加速度 rad/s² |
|---:|---:|---:|---:|---:|
| 1 | 12.50→2.57 | 12.59→2.83 | 0.841→0.627 | 0.076→0.082 |
| 2 | 3.18→1.84 | 3.08→4.03 | 1.212→0.904 | 0.066→0.083 |
| 3 | 24.39→3.74 | 20.96→6.33 | 0.910→0.544 | 0.134→0.161 |
| 4 | 29.22→4.45 | 29.93→5.13 | 0.867→0.497 | 0.135→0.183 |
| 5 | 21.69→6.89 | 32.25→10.77 | 0.785→0.486 | 0.154→0.141 |
| 6 | 22.19→16.85 | 10.57→5.61 | 0.828→0.676 | 0.141→0.210 |

按直线/弯道/弯后窗口合并采样的 P95（前→后）：

| 窗口 | 航向 ° | 横向 cm |
|---|---:|---:|
| 直线 | 28.02→4.12 | 11.49→4.56 |
| 弯道 | 29.02→12.74 | 21.43→9.02 |
| 弯后 1 m | 41.59→2.69 | 40.82→6.19 |

低速增加了某些区段的采样权重，因此另按每 10 cm 路径进度区间取误差中位值、每个区间只贡献一次进行检查；原始分布和空间统计同时保留，不能将某处停车样本多误当成更差的整条路线。空间分布文件为 `/tmp/astribot_heading/native_quality_spatial.json`。

限制：部分路段的角加速度高于基线；第一轮精调加速度 P95 的六段中位值从 0.0189 升到 0.0240 m/s²。故这是航向、横向及平移平顺性的改善，不能宣称每项指标均不退化。接近区分别统计，未以最终到点成功代替过程质量。第二、四段接近曲线见 `/tmp/astribot_heading/native_approach_comparison.png`。必要的终点大角度旋转仍存在。

最终构建与 83 项 C++ 回归通过：`build_native_quality.log`、`regression_final_result.log`；Python 语法检查与启动 dry-run 通过。Gazebo/RViz 启动实测七个生命周期节点 active，4.67 秒观测到 3893 帧时钟、195 帧里程计及 40 帧扫描，TF 龄期 0.011 秒。有效参数已由服务读回，保存于 `native_quality/effective_configuration.json`。这两轮不等于两小时耐久通过，也不覆盖 SLAM、视觉或 mark 定位噪声工况。
