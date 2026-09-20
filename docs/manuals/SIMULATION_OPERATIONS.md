# Gazebo 与 RViz 仿真操作手册

2026-09-18：SLAM 接入已改为仿真/真机共用 Voxel-SLAM；接口、命令与逐项验收状态见[统一 SLAM 参考](../SLAM_INTEGRATION_REFERENCE_20260918.md)。旧中转链已删除；真机尚未实测。

适用范围、版本和能力边界见[手册首页](README.md)。所有命令在开发机执行，默认仓库根目录 `/home/yjh/WorkSpace/astribot_sdk_ros2`。首次编写只核对入口；2026-09-17 补充删包后的实际 Gazebo/RViz 回归，已执行项与结果见[本轮记录](../AUTONOMY_SIMULATION_REGRESSION_20260917.md)，未覆盖场景仍需独立验收。

## 1. 启动前准备

环境为 Ubuntu 22.04、ROS2 Humble、项目使用的 Gazebo/Ignition、Nav2 与 RViz。首次构建在未加载厂家 SDK 的干净终端执行：

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
bash ws_robot/src/astribot_s1_perception/scripts/prepare_livox_driver2.sh
bash tools/robot/build_slam_dependencies.sh
export CMAKE_PREFIX_PATH="$PWD/ws_robot/deps/gtsam:${CMAKE_PREFIX_PATH:-}"
cd ws_robot
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DGTSAM_DIR="$PWD/deps/gtsam/lib/cmake/GTSAM"
```

不要在其他会话使用同一 `ws_robot/install` 时覆盖其运行库。修改 C++、接口或 launch 后构建相应包及依赖；`ros2 launch` 读取安装空间，源码改动不一定已进入运行版本。

新的操作终端加载：

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
source /opt/ros/humble/setup.bash
source ws_robot/install/setup.bash
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
```

启动器和 Gazebo launch 固定域 25，单独修改当前 shell 的域不能建立第二套隔离仿真。真机同为域 25，但网络范围不同；不要把仿真改为对真机网络开放。

共享环境先按 [ros2-stack-ops](../../.agents/skills/ros2-stack-ops/SKILL.md) 确认旧栈归属。该流程中的 `/tmp/rosops/*.sh` 是按技能生成的临时工具，不保证每台机器已存在。存在其他人的栈时，由其主管正常停止或在已获授权后接管；不要用全局 `pkill` 清理。

## 2. 推荐基线：固定地图、真值定位

先预览，不启动节点：

```bash
bash tools/launch_sim_stack.sh --mode baseline --dry-run
```

实际启动 Gazebo、RViz 和导航，保留该前台终端：

```bash
bash tools/launch_sim_stack.sh --mode baseline
```

该入口显式选择：

| 项目 | 值 |
|---|---|
| 地图 | `maps/warehouse_baseline.yaml` |
| 定位 | `ground_truth` |
| 跟踪器 | MPPI |
| 最大线速度 | x/y 各 0.35 m/s；不是对角合速度保证 |
| 到位档位 | `simulation_precision`：2 mm / 0.1° |
| 策略 | `off` |
| 扫描 | `slice_scan`，Nav2 输入 `/scan_from_cloud` |
| GUI | Gazebo 与 RViz 开启 |
| 自动任务 | 不自动派发导航目标 |

需要固定本次日志位置时使用一个**尚不存在**的目录：

```bash
SIM_RUN="$HOME/.ros/log/astribot/manual_baseline_$(date +%Y%m%d_%H%M%S)"
bash tools/launch_sim_stack.sh --mode baseline --log-dir "$SIM_RUN"
```

启动器会创建目录，不要预先 `mkdir "$SIM_RUN"`。无显示环境可用 `--headless --no-rviz`；这只改变显示，不保证解决物理插件加载故障。

## 3. 模式与默认值的区别

| 入口 | 行为 |
|---|---|
| `--mode baseline` | 固定地图 + 真值定位 + 仿真精度档；用于控制效果对照 |
| `--mode explore`，不传 `--map-yaml` | 显式 SLAM 建图定位，并启动自主探索，会自动行走 |
| `--mode mapping` 或无参数 | 统一 Voxel-SLAM 建图和定位；保存需启动时传入新的 `--save-session` 目录 |
| `--mode localize --map ...` | 接收 Voxel 会话目录；重定位成功后才启动导航 |
| `--tracker rpp` | 更换内层控制器；需单独做效果验收 |
| `--scan-source laserscan` | Nav2 直接选择 `/scan`；与默认切片链分开记录 |

`--map` 是完整 Voxel 会话目录，不能仅指定 PGM/YAML。`--map-yaml` 仅供 `--mode baseline`。`--save-session` 指定新的会话目录；停稳并结束探索/导航后用 `ros2 run astribot_s1_perception slam_session save` 保存。mapping、explore 和 localize 均使用统一 Voxel 后端，无需修改共享地图来源 YAML。

直接 `navigation.launch.py` 的默认值是 RPP、1.0 m/s、standard，和上述一键基线不同。开发调试若绕过主管，必须显式指定控制器、速度、时钟、扫描、地图及精度档，并自行负责启动时序和所有权。

ROS launch 风格与兼容 CLI 风格二选一：

```bash
bash tools/launch_sim_stack.sh mode:=baseline navigation_policy:=off max_linear_speed:=0.35
# 等价的兼容 CLI 风格；不要与上一种参数语法混用
bash tools/launch_sim_stack.sh --mode baseline --navigation-policy off --max-linear-speed 0.35
```

上面两条是替代命令，不是让两套栈同时启动。

## 4. 按层确认就绪

主管自动先检查仿真数据，再启动导航并检查生命周期；默认最多尝试两次导航启动。日志里的进程数量、发布者数量或 RViz 已打开，均不能单独证明就绪。

1. **Gazebo 物理层**：transport 可见，`/stats` 的仿真时间和 iterations 推进。
2. **ROS 时钟**：实际收到持续推进的 `/clock`。
3. **TF**：`map → astribot_torso_base` 可查且龄期正常。
4. **生命周期**：7 个导航节点 active。
5. **数据与接线**：实收扫描和里程计；costmap 选择正确扫描，任务入口只有预期所有者。

基础物理观察：

```bash
timeout 15 ign topic -e -t /stats -n 1
```

开始 ROS 查询前，从本次主管记录的子进程反读 DDS 环境，再按仓库技能生成并加载 `query_env.sh`；不能仅靠当前 shell 猜运行域。环境对齐后：

```bash
python3 tools/sim_stack_probe.py --phase data --timeout 15 --scan /scan_from_cloud
python3 tools/sim_stack_probe.py --phase navigation --timeout 30 --scan /scan_from_cloud
```

策略开启时，导航探针还需要检查适配后的扫描：

```bash
python3 tools/sim_stack_probe.py --phase navigation --timeout 30 \
  --scan /scan_from_cloud --costmap-scan /navigation_policy/costmap_scan
```

选择 `laserscan` 时把原始 `--scan` 改为 `/scan`。探针是有界实际订阅，不发送导航目标。`ready=true` 只证明基础就绪，不证明路径跟踪质量或每个故障场景已通过。

## 5. 人工目标与路线验证

先用 RViz 的 Nav2 Goal 在已知空旷区域设置近距离目标；Fixed Frame 用 `map`。基线使用真值定位，不需反复发送 2D Pose Estimate 改变定位。

运行路线脚本前结束其他目标。脚本只支持 `use_sim_time=true`，预检发现活动导航任务会拒绝抢占。规划预检不发送运动目标，但会调用规划服务、生成候选路径。

```bash
ROUTE_ROOT="$HOME/.ros/log/astribot/route_$(date +%Y%m%d_%H%M%S)"
python3 tools/run_waypoint_route.py --short-route --cycles 1 \
  --dry-run --output "$ROUTE_ROOT/preflight"
```

预检通过后执行五目标短路线：

```bash
python3 tools/run_waypoint_route.py --short-route --cycles 1 \
  --duration 900 --timeout 180 --settle 3 --output "$ROUTE_ROOT/short"
```

短路线的 map 坐标为 `(1,0,0°) → (1,-1,0°) → (0,-2,-90°) → (0,-2,90°) → (0,0,0°)`。是否安全以当前地图预检与运行碰撞检查为准。脚本使用人工 `/navigate_to_pose` 入口，运行期间不要从 RViz 另发目标。

完整默认六点路线持续跑机：

```bash
python3 tools/run_waypoint_route.py --cycles 3 --duration 7200 \
  --timeout 300 --settle 3 --output "$ROUTE_ROOT/full"
```

`--cycles 0` 表示只受 duration 限制；duration、timeout 是墙钟实验预算，低实时率仿真需留足预算，它们不是控制质量排名指标。Ctrl+C 会尝试取消本工具持有的目标；随后确认停车。该脚本没有通用 STOP 文件接口，不要与真机精度工具混淆。

自定义路线使用 JSON 文件，元素为 `[x_m, y_m, yaw_rad]`，不是角度：

```json
[[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
```

将文件路径传给 `--route /绝对路径/route.json`；先带 `--dry-run` 预检，再执行。目标坐标绑定当前地图和定位原点，不能跨 SLAM 重启无条件复用。

结果目录包含 `status.json`、`metadata.json`、`preflight.json`、`samples.csv`、`plans.jsonl`、`results.jsonl` 及 runner 源码快照。`completed` 或 `duration_complete` 要结合成功目标数、失败原因和到位复测判断，不把达到时长上限当成全部目标通过。

## 6. 策略与窄通道验证

每阶段使用独立日志，先正常结束上一套。示例为 0.32 m/s 策略实验；对照 OFF 时也必须使用相同速度、地图、定位档和起点。

```bash
bash tools/launch_sim_stack.sh --mode baseline --navigation-policy p2 --max-linear-speed 0.32
# 结束 P2 后，另一次运行才使用 P3
bash tools/launch_sim_stack.sh --mode baseline --navigation-policy p3 --max-linear-speed 0.32
```

P4 需要人工通道文件，格式和地图坐标语义见[策略包](../../ws_robot/src/astribot_s1_navigation_policy/README.md)；P5 自动生成地图候选。它们是试验入口，不代表完整窄通道已验收。文件几何、Gazebo 实体和 Nav2 地图必须一致；只修改地图图片不能制造对应的物理通道。

```bash
# 将路径替换为本场景已核对的通道标注；先预览
bash tools/launch_sim_stack.sh --mode baseline --navigation-policy p4 \
  --corridor-file /绝对路径/corridors.json --max-linear-speed 0.32 --dry-run
```

回归矩阵按场景记录，不只测终点成功：

| 场景 | 重点判据 |
|---|---|
| 开阔直线、曲线、短路径、接近段 | 原跟踪质量，直线航向、横向误差、速度平滑 |
| 远处占据、近处横穿、暂时封堵 | 保留安全制动空间；合理慢行/让行，不无故提前停车 |
| 障碍持续阻塞、移走 | 局部绕行/重规划及恢复，陈旧占据与候选撤销 |
| 1.30/1.15/1.10/1.05/1.00/0.95 m | 原路径准入、阈值边界、入口/出口和滞回设计验证 |
| 0.90/0.85/0.84/0.80 m | 联合误差预算及拒绝边界；85 cm 不预设必过 |
| 偏心、带偏角、短斜入口、内部重启 | 完整机身与转向扫掠，不仅检查中心路径 |
| 反向退出、后方遮挡、终点带朝向要求 | 后向覆盖、停稳恢复及航向不可达契约 |
| 暂停/恢复、时钟回跳、扫描短抖动/断流 | 新鲜度、租约撤销、停机与恢复语义 |

故障注入仅在隔离仿真完成，保存注入时间线并保证解除。真机不复制 SIGSTOP、制造碰撞或破坏 TF 的实验步骤。

## 7. 评价与阶段放行

固定基线后一次只改变一个因素，每组至少进行多个有效重复，并交替运行基线/候选。建议按相同路径进度和场景分组，避免把起点不同的两轮混算。

- 到位：欧氏 XY、最短 yaw、Action 结果、停稳窗口、动作结束后的漂移；不放宽当前档位。
- 过程：横向及路径航向误差 RMS/P95/max、直线速度波动、加速度/jerk、曲率过渡、非预期旋转、停滞/误停、最小净空及重规划次数。
- 分段：完整 FOLLOW、距终点大于 0.5 m 的行进段、直线、弯道、接近和 REFINE 分开。
- 数据质量：源时间去重，路径版本和阶段切换分段，不跨缺帧求导；标注定位来源、RTF 和无效样本。
- 耗时只作预算/超时记录；不以更快抵消质量下降，也不以平均值改善掩盖某个场景退化。

当前扫描超时[历史对照](../SCAN_OBSERVATION_TIMEOUT_20260917.md)支持指定场景中的短抖动改善与断流检出，不能替代后续源码的完整窄通道、RPP 或真机回归。

2026-09-17 删包后 MPPI/off 真值基线实跑短路线 5/5、完整路线 6/6，最大到位误差 1.798 mm / 0.081°。完整路线 FOLLOW 横向 P95 为 6.641 cm、最大 11.499 cm；这只证明记录场景的功能与到位判据通过，不是 85 cm 通道或统计性能不退化的证据。原始目录和指标口径见[回归报告](../AUTONOMY_SIMULATION_REGRESSION_20260917.md)。

2026-09-17 的 SLAM 自主探索**未通过**：首次 Nav2 生命周期启动由主管重试后恢复，但前沿每轮 8 个候选全部未通过回退位姿/完整足迹检查，3 次自动恢复耗尽，未派发目标。`exploration/complete=false`，不能当作探索完成。该记录对应旧链路；2026-09-18 统一 Voxel 链路的建图、载图、短路线和探索验证见[最新 SLAM 记录](../SLAM_SIMULATION_VALIDATION_20260918.md)，不覆盖整仓库探索或暂停后的恢复验收。

### 7.1 包退役后的组合调试入口

正常自主探索用 `--mode explore`。单独看前沿建议和 Marker 时，组合入口已改为：

```bash
ros2 launch astribot_s1_exploration autonomy_bringup.launch.py \
  enable_perception:=false use_composition:=true use_rviz:=true \
  goal_topic:=/debug/frontier_suggestion
```

此示例假设本栈已有切片扫描节点，故关闭重复感知启动；若使用完全独立环境，按[迁移表](../AUTONOMY_PACKAGE_RETIREMENT_20260917.md)选择完整组合。`use_composition:=false` 是另一种进程组织方式，不能与上一命令同时启动同名节点。组合入口没有探索任务协调器，不应把它作为自动导航入口；独立建议话题不得另外接一个目标转发器抢占正在执行的任务。

前沿节点要求地图持续有效；固定地图长时间不更新时会按其 map_timeout_sec 进入 WAITING_MAP。本轮固定地图测试仅验证组件、Marker、资源路径和 RViz，自动选点执行需用独立 SLAM 探索会话确认。

自主探索暂停/恢复服务为：

```bash
ros2 service call /exploration_coordinator_node/pause std_srvs/srv/Trigger '{}'
# 确认取消终态、实际底盘停稳及没有新目标后，再按任务需要恢复
ros2 service call /exploration_coordinator_node/resume std_srvs/srv/Trigger '{}'
```

暂停服务返回成功不等于执行器已停稳；核对 `/exploration/state` 中 `manual_pause=1`、`goal_in_flight=0`，并观察新鲜 `/odom` 和 `/cmd_vel`。恢复会重新选目标并自动行走。

## 8. 日志、参数与正常关停

启动终端打印的绝对 `session.log` 是本次主日志。默认索引为 `~/.ros/log/astribot/latest_sim`；设置 `ROS_HOME` 或日志根目录时以实际输出为准。

```bash
SIM_SESSION="$(readlink -f "$HOME/.ros/log/astribot/latest_sim")"
cat "$SIM_SESSION/session.json"
rg 'PATH_TRACKING/|ARRIVAL_REACHED|TRACKING_METRICS|IMMEDIATE_RISK|INPUT_UNAVAILABLE' \
  "$SIM_SESSION/session.log"
```

`latest_sim` 只指向最近记录，不证明仍在运行。参数查询在对齐 DDS 环境后执行：

```bash
ros2 param get /controller_server use_sim_time
ros2 param get /controller_server precise_goal_checker.xy_goal_tolerance
ros2 param get /controller_server FollowPath.arrival.refine_timeout
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.expected_update_rate
ros2 param get /global_costmap/global_costmap static_layer.map_topic
```

正常停止顺序：停止路线工具并取消其目标→确认机器人停稳→在主管前台 Ctrl+C→检查本会话 `session.json` 与子进程清理结果。后台启动时只能在核对主管 PID、命令、启动身份与本会话归属后给该主管 SIGINT。不要杀整个域，也不要把临时报告中的旧 PID 当作当前目标。

## 9. 常见故障

| 现象 | 优先检查与处理 |
|---|---|
| 已有栈/锁冲突 | 确认主管及日志归属，正常关闭旧会话；不直接删锁文件绕过 |
| 所有话题都不可见 | 从运行进程反读域、localhost、RMW/DDS 环境，再检查实际订阅 |
| Gazebo 有界面但时钟不走 | 先查 transport `/stats`、暂停状态与插件加载；不先调整 ROS 参数 |
| 节点存在但 Nav2 不可用 | 看七个 lifecycle 状态和主管 readiness 报告 |
| 发布者存在但没有扫描 | 检查 QoS、自滤所需连杆 TF、源时间与扫描实际拍率 |
| 原点反复 HOLD、恢复确认不完成 | 分别查时钟停滞、ROS/墙钟年龄、租约、扫描与策略状态 |
| 门口停止 | 区分原路径风险、转向扫掠、未知/越界、过期输入与通道接管；保存拒绝障碍/净空，不先减膨胀 |
| 到点失败 | 查看实际精度档、位姿源、停稳漂移、`PATH_TRACKING` 原因；不只看 RViz 中心点 |
| 源码改了效果没变 | 核对安装路径、启动日志参数和库哈希；先停止旧实例再加载新版本 |

详细操作约束见仓库 [ros2-stack-ops](../../.agents/skills/ros2-stack-ops/SKILL.md)，日志字段见[统一日志](../LOGGING.md)。
