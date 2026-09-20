# 路径跟踪与到位精度的评价、运行日志

## 当前评价口径

| 项目 | 定义及数据来源 |
|---|---|
| 横向误差 `cross_track_m` | 将机器人 base 位姿和当前规划路径放在同一坐标系内，求到路径有限线段的最近投影距离。以路径前进方向为准，左正右负。单位 m。 |
| 路径航向误差 `heading_error_deg` | `wrap(机器人 yaw − 投影所在路径段方向)`，范围 ±180°。使用路径段切线，不使用最终目标朝向。 |
| 到位位置误差 `xy_m` | `hypot(robot_x − goal_x, robot_y − goal_y)`，按欧式距离验收，不是分别判定 x/y。 |
| 到位角误差 `yaw_deg` | `abs(wrap(robot_yaw − goal_yaw))`，单位度；跨 ±180° 时取最短角差。 |
| RMS / P95 / max | 对选定样本的误差绝对值计算；RMS 为平方均值开根号，P95 取排序后第 `ceil(0.95*N)` 个样本。 |

横向误差的投影参数为 `u=clamp(dot(robot−A, B−A)/|B−A|², 0, 1)`，投影点 `Q=A+u(B−A)`，距离 `|robot−Q|`。落在端点时该距离可能包含纵向残差，不能视为无限直线的纯法向距离。重复点形成的零长线段跳过；没有有效线段时标记无效，不记录成零误差。

首次收到路径时从全路径选择最近投影；随后只比较上一投影弧长前 1 m、后 3 m 范围内起始的线段，减少路径交叉时跳到另一支。每次 `setPlan` 清空投影锚点，增加 `plan` 编号。诊断使用控制器实际收到的原路径，不改变路径或控制命令。

现有 `tools/run_waypoint_route.py` 最多约 20 Hz（墙钟每 0.05 秒）采样 TF `map→astribot_torso_base`、`/plan` 和 `/odom`，生成 `samples.csv` 与 `results.jsonl`。RMS/P95 的默认口径是有有效投影的 **FOLLOW 全阶段**；直线样本另要求参考曲率绝对值 `<0.1 m⁻¹`，曲率由投影段附近约 0.5 m 邻域方向差估计。加速度/jerk 使用去重后的 odom 源时间计算；位置样本没有按 TF 时间戳去重。

上一份报告的 2.58 cm / 5.35 cm 横向 P95 使用了额外条件：**FOLLOW 且距离最终目标大于 0.5 m**。它和 `results.jsonl` 的默认全 FOLLOW 统计不完全相同。比较前后版本必须统一阶段、排除半径、采样频率和路线条件，不能混用两种 P95。

## 运行日志

控制器通过现有 `RCLCPP_INFO` 接入统一 spdlog `session.log`（日志级别需为 info 或更详细），默认周期诊断为 **2 Hz（ROS 时间）**。时间判断发生在投影搜索和日志格式化之前，所以横向计算也降到 2 Hz；控制、碰撞检查、到位判定仍按原控制频率运行。时钟暂停时不反复输出，时间回跳后采样计时重新对齐。

| 标记 | 含义 |
|---|---|
| `TRACKING_METRICS` | 起始对齐或路径跟踪期间的横向误差、路径航向误差、目标误差、实际速度。`follow_sample=1` 才计入 FOLLOW 统计；`travel_sample=1` 同时排除了目标附近区间；`straight_sample=1` 表示 FOLLOW 中参考曲率 `<0.1`。需要行进直线口径时同时要求 `travel_sample=1` 和 `straight_sample=1`。 |
| `ARRIVAL_METRICS` | 精调期间的目标位置/角度误差、实际速度、两轴保持状态和稳定时长。`phase=REFINE` 为修正，`SETTLING` 为连续稳定确认，`DONE` 为本周期满足成功条件。此阶段不冒充路径横向误差样本。 |
| `ARRIVAL_REACHED` | 每次导航尝试首次稳定到位时立即输出，不受周期诊断降采样或关闭影响。包含误差、验收阈值、速度、坐标系、定位源和稳定时长。它表示控制器到位事件，不表示 action 返回后再次测量。 |

每条周期记录包含 `ros_s`、`pose_s`、`plan`、`phase`、`source`、`frame` 和 `sample_hz`。跟踪期使用导航位姿；精调期使用选定的 `nav2_pose/slam_pose/vision/mark` 位姿。`path_valid=0` 时投影相关数值为 `nan`，统计时应排除。日志未包含政策让行/接管提前返回阶段，这些状态仍由原阶段与策略日志记录，不能按普通 FOLLOW 样本计入。

当前精度仿真档位为 2 mm / 0.1°，其他配置应以日志中的 `xy_tol_m/yaw_tol_deg` 为准。到位仍要求 XY/yaw 均满足严格阈值、速度满足停止阈值、连续稳定窗口及源时间推进；这些控制判据没有因增加日志而改变。

验证脚本还会在导航 action 结束后等待 `--settle`（默认 2 秒，上次实测为 3 秒），再用 TF 和 odom 测量最终误差、检查 action 状态和停止速度，写入 `results.jsonl`。因此报告中的“停稳后精度”与 `ARRIVAL_REACHED` 时间点可能略有差别。真机日志是相对于选定定位源的误差，仍需独立真值评估真实物理精度。

## 降采样参数与查看命令

MPPI/RPP 的导航 YAML 均支持：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      metrics:
        sample_hz: 2.0
        terminal_exclusion_radius: 0.5
```

`sample_hz` 范围为 0–20；0 关闭周期诊断，成功/异常事件仍保留。实际采样频率受控制频率约束；`terminal_exclusion_radius` 只决定日志样本标志，实际排除半径取该值与捕获半径的较大者，不影响控制。参数在插件 configure 时读取，修改后重新启动导航生效。

```bash
# 查看当前仿真的跟踪、精调和稳定到位记录
tail -F "$HOME/.ros/log/astribot/latest_sim/session.log" \
  | rg --line-buffered 'TRACKING_METRICS|ARRIVAL_METRICS|ARRIVAL_REACHED'
```

周期日志是降采样后的在线观测，可能漏掉短时尖峰，不能将其 P95/max 当作原 20 Hz 验证数据的相同统计量。现有验证脚本采样和报告口径保持不变，以保留历史对照及加速度/jerk 的时序分辨率。


## 2026-09-15 验证

- 编译通过，独立验证目录的 95 项 C++ 回归全部通过，包括诊断开启/关闭时 60 组末端速度指令逐值完全一致。
- 600 组随机路径/位姿/弧长窗口与现有 Python 投影方法对照一致，距离最大浮点差异为 `2.22e-16 m`。
- 原图形会话因 OpenGL 上下文创建失败退出；改用 `LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa` 的无界面仿真，未修改产品渲染默认值。实测时钟前进、TF 有效、Nav2 七节点 active 后才发导航目标。
- 两个目标均成功：停车后误差分别约 0.860 mm / 0.05529°、0.553 mm / 0.01718°。检查到 41 条 TRACKING_METRICS、91 条 ARRIVAL_METRICS，周期记录最小间隔为 0.5 秒 ROS 时间；两个 ARRIVAL_REACHED 均即时写入，每个目标一次。
- 证据及真实日志摘录：`runs/tracking_metrics_20260915/`。本轮仅验证日志与计算口径，不用软件渲染条件下的短路线声称历史横向性能问题已解决。

实际到位日志的主要字段如下（完整时间/速度/路径编号见日志摘录）：

```text
ARRIVAL_REACHED source=nav2_pose xy=0.000860m yaw=0.055293deg settled=0.61s ... plan=1 frame=map xy_tol_m=0.002000 yaw_tol_deg=0.100000
ARRIVAL_REACHED source=nav2_pose xy=0.000553m yaw=0.017185deg settled=0.62s ... plan=2 frame=map xy_tol_m=0.002000 yaw_tol_deg=0.100000
```

## 指令平滑性与桥接耗时（2026-09-17）

`navigation_speed_report.py` 的各话题报告包含 `command_dynamics_10hz`。先按 ROS 接收时间
对指令做 100 ms 分箱时间平均，再计算相邻分箱的加速度及 jerk；跨目标和无有效指令覆盖的
间隔不连差分。数据使用报告中声明的 body/map 坐标口径。它用于相同路线下比较指令平滑性，
不是底盘实测加速度；接收抖动和分箱平均也使它不能用于证明瞬时硬限值。

桥接每约 10 秒在统一日志输出 `BRIDGE_TIMING`：回调周期、工作耗时、控制锁等待及 SDK
读写调用的均值、P50/P95/P99、最大值。时钟为单调墙钟，独立于控制积分的 ROS 时间。
每项最多保存 8192 个样本，溢出丢样数随报告输出。SDK 调用时段可能包含线程抢占，不能
把该时段直接解释为 SDK 内部阻塞或底盘机械延迟；各窗口分位数也不能当作全程分位数。
可用 `tools/robot/analyze_bridge_timing.py` 汇总报告。
