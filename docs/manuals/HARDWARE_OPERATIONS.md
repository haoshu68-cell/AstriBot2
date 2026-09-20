# 真机部署与操作手册

2026-09-18：SLAM 接入已改为仿真/真机共用 Voxel-SLAM；接口、命令与逐项验收状态见[统一 SLAM 参考](../SLAM_INTEGRATION_REFERENCE_20260918.md)。旧中转链已删除；真机尚未实测。

SLAM、导航栅格与消息分别由 `astribot_s1_slam`、`astribot_s1_mapping`、`astribot_slam_msgs` 管理。部署快照包含项目 Eigen 和 `ws_robot/third_party/gtsam` 源码，构建脚本在目标架构重新编译数值依赖；不复用开发机库或外部 SLAM 工作空间。接口仍按领域分包，见[架构评审](../SLAM_ARCHITECTURE_REVIEW_20260918.md)。

版本范围见[手册首页](README.md)。以下地址、厂家目录来自当前部署脚本的默认值；本次没有连接机器人确认现场状态。换机器人时先核对这些路径和 DDS 配置。

## 1. 运行位置与命令作用

| 项目 | 当前默认位置 |
|---|---|
| 开发机源码 | `/home/yjh/WorkSpace/astribot_sdk_ros2` |
| SSH 目标 | `astribot@10.249.22.137` |
| 机器人版本根目录 | `/home/astribot/astribot_projects/releases/<版本名>` |
| 当前版本入口 | `/home/astribot/astribot_projects/current` |
| ARM64 厂家 SDK | `/home/astribot/Downloads/astribot_sdk_aarch64` |
| SLAM 与 Livox 驱动 | 当前 release 的 `ws_robot/install` |
| DDS 配置 | `/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml` |

`current` 只选择**下一次启动**使用的版本，不会切换已运行进程。机器人使用 ARM64 原生库，不能把开发机 x86 的 build/install/.so 复制到机器人。

| 命令 | 是否会改运行状态 |
|---|---|
| `run_deployed.sh --help` | 只显示帮助 |
| `precision/explore/sensors --dry-run` | 只打印计划，不创建 SDK 会话或 ROS 节点；可在开发机预览 |
| `run_deployed.sh check` | 订阅、TF 与图检查，不使能、不发速度 |
| `run_deployed.sh sensors` | 先停止识别到的旧导航/感知，再启动雷达、SLAM 和辅助链；不启动新的 SDK 桥接 |
| `run_deployed.sh precision` | 重启感知、导航和桥接，通过检查后使能，等待人工目标 |
| `run_deployed.sh explore` | 上述启动完成后自动选目标并行走 |
| 无参数、`start`、`restart` | 与 precision 相同，会启动并使能 |
| `run_deployed.sh rviz` | 启动界面；在界面发送目标会触发运动 |
| `run_deployed.sh navigation` / `bridge` | 专用调试入口，会启动相应组件；正常操作不与 precision/explore 叠加 |
| `stop_robot_tasks.sh` / `run_deployed.sh stop` | 取消任务、停车/停用、检查反馈并关闭选定任务及感知 |

## 2. 新版本部署与切换

在开发机先查看帮助，然后使用新的唯一版本名部署：

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
bash tools/robot/deploy_project.sh --help
ROBOT=astribot@10.249.22.137 bash tools/robot/deploy_project.sh "manual_$(date +%Y%m%d_%H%M%S)"
```

第二条部署命令会 SSH、同步源码、校验 SHA-256 并在机器人独立版本目录构建，不切换 `current`、不启动运动。包含工作区未提交源码；不上传开发机 `build/install/log/runs`，厂家原生依赖通过机器人已有目录链接复用。Voxel 与驱动随应用构建，厂家 SDK 和设备网络/外参设置须单独登记。

版本目录中保留 `deployment/source_manifest.json` 和 `deployment/build.log`。构建失败时先查它们，不覆盖当前正在运行的安装目录。

需要在机器人手动重建某个**非运行版本**时，用干净终端直接执行该版本的 `tools/robot/build_robot.sh`。不要先 source 厂家运行环境；脚本会拒绝混入厂家 ROS 头文件的构建环境。`--clean` 会删除该版本的构建产物，日常增量构建不要随意添加。

在机器人终端选择新版本，下面的路径必须替换为部署成功的实际目录：

```bash
NEW_RELEASE=/home/astribot/astribot_projects/releases/实际版本名
test -f "$NEW_RELEASE/deployment/source_manifest.json"
test -f "$NEW_RELEASE/ws_robot/install/setup.bash"
```

核对构建成功、任务归属和现场停车条件后，用**旧版本**入口结束旧任务，再切换链接：

```bash
OLD_RELEASE="$(readlink -f /home/astribot/astribot_projects/current)"
bash "$OLD_RELEASE/tools/robot/stop_robot_tasks.sh" --dry-run
bash "$OLD_RELEASE/tools/robot/stop_robot_tasks.sh"
```

确认停止报告后才执行：

```bash
ln -sfn "$NEW_RELEASE" /home/astribot/astribot_projects/current
```

保留 `OLD_RELEASE` 的绝对路径用于回滚。切换本身不启动机器人；若没有旧版本入口，按部署现场管理流程核对旧任务，不猜测目录执行清理。

## 3. 登录、加载环境与版本核对

开发机登录：

```bash
ssh astribot@10.249.22.137
```

以下均在机器人终端执行：

```bash
cd /home/astribot/astribot_projects/current
source tools/robot/env_deployed.sh
readlink -f /home/astribot/astribot_projects/current
ros2 pkg prefix astribot_s1_navigation
ros2 pkg prefix astribot_s1_path_tracking
ros2 pkg prefix astribot_trajectory_bridge
```

包前缀应指向选定 release 的 `ws_robot/install`；`PROJECT` 与 `SDK` 分别指向应用和厂家目录。运行环境为域 25、`ROS_LOCALHOST_ONLY=0`、`rmw_fastrtps_cpp` 及厂家 UDP profile。不要在机器人加载开发机根目录 `env.sh`，也不要仅加载厂家旧 workspace 后认为运行的是新代码。

删除旧自主包后的版本还应检查：

```bash
ros2 pkg prefix astribot_s1_perception_components
ros2 pkg prefix astribot_s1_exploration
ros2 pkg prefix astribot_autonomy_core
ros2 pkg prefix astribot_s1_autonomy
```

前三项应属于当前 release，最后一项应提示找不到包。若旧包仍可见，先排查旧 overlay 或复用的安装产物，不要在机器人正在运行时删除其 release。旧入口替换关系见[迁移表](../AUTONOMY_PACKAGE_RETIREMENT_20260917.md)。开发机删包不会自动迁移机器人；本次未 SSH、部署或执行真机运动。

## 4. 只读检查与感知启动

若现场感知已运行，先只读检查：

```bash
bash tools/robot/run_deployed.sh check
```

若感知未运行，检查失败是预期结果；需要重建感知链时先预览、再启动：

```bash
bash tools/robot/run_deployed.sh sensors --dry-run
bash tools/robot/run_deployed.sh sensors
```

**sensors 也会结束识别到的旧导航及感知，不是只添加缺失节点。** SLAM 重启会重新建立坐标与地图；不能无条件复用上一轮 map 目标或测试原点。它仅打开读取状态与里程计的 SDK 接口，不新建导航/底盘指令桥，但停止旧运动任务仍会执行软件停车流程。

在另一个已加载环境的终端保存检查：

```bash
python3 tools/robot/hardware_readiness.py > /tmp/astribot_inputs.json
cat /tmp/astribot_inputs.json
```

| 数据 | 当前检查口径 |
|---|---|
| `/scan_from_cloud`、`/odom` | 各一个发布端，实际收到数据，至少 3 个推进的源时间戳，≥5 Hz、龄期不超过 0.5 s |
| `/map` | 一个发布端，实际非空地图，允许静态/低频，地图 frame 能转换到 map |
| TF | `map → astribot_torso_base` 可查且新鲜 |
| conflicts | 现有导航/桥接冲突列表；普通 check 列出，带 require-idle 参数才要求为空 |

`ready` 不是 StaticLayer 已初始化的完整证明，也不是场地可通行或底盘已使能。先对齐实际进程 DDS 环境再解释发布者数量；检测有发布端不等于收到有效消息。

当前地图/里程计链：

```text
双雷达 PointCloud2 + IMU → 进程内自滤 → Voxel-SLAM
Voxel 关键帧 → nav_prob_grid → /map
Voxel 世界点云 → 一次切片 → /scan_from_cloud
厂家 SDK → chassis_odom_node → /odom 与 odom→底盘
Voxel 全局位姿 + 里程计 → map_odom_tf → map→odom
```

前两个辅助脚本已纳入项目版本。`deployed_sensors.json` 虽保留旧 `helper_directory` 字段，当前启动命令不再从那里加载这两个节点；实际旧进程仍需核对，不能仅看源码断言机器人已迁移。

自滤参数由 SLAM 和切片共用；栅格只在新鲜定位下处理当前机身内区域，不保留历史轨迹清除。地图直接输出 map 坐标系，不再使用 camera_init 恒等别名。

## 5. 人工导航与到位验证

正常使用只启动一个完整入口。precision 将结束旧感知实例，因此由 sensors 切到 precision 也会重启 SLAM。

```bash
bash tools/robot/run_deployed.sh precision --dry-run
bash tools/robot/run_deployed.sh precision
```

启动顺序：旧任务停车与退出→Livox 实际数据检查→厂家关节反馈/模型→SLAM、扫描、地图、里程计→输入和地图坐标检查→Nav2 与停用状态的桥接→七个生命周期 active→底盘使能→等待人工目标。

已有厂家模型和关节发布者会被复用，缺失时补齐。厂家反馈模式不以虚构零姿态代替真实关节状态。扫描自滤需要连杆 TF；扫描无数据需结合日志区分模型、反馈、急停和感知故障，不能直接判为急停。

机器人图形桌面的另一终端：

```bash
bash /home/astribot/astribot_projects/current/tools/robot/run_deployed.sh rviz
```

确认现场允许移动、机械臂/负载处于已评估包络且独立急停可用，再通过 Nav2 Goal 单个派发空旷区域约 0.3～0.5 m 的同向目标。确认停稳后再测返回、转向与横向。SSH 终端没有图形会话时不要把 RViz 失败当成导航失败。

自动小范围精度工具先预检，不加 `--execute` 不发送运动目标：

```bash
HW_TEST="$HOME/.ros/log/astribot/precision_check_$(date +%Y%m%d_%H%M%S)"
python3 tools/robot/navigation_precision_check.py \
  --distance 0.25 --directions x+ --angles --output "$HW_TEST/preflight"
```

这里空的 `--angles` 列表取消角度测试，只测选定平移方向及工具定义的返回目标。核对 `preflight.json`、原点和目标清单，在**同一 SLAM 会话**中才执行：

```bash
python3 tools/robot/navigation_precision_check.py --execute \
  --distance 0.25 --directions x+ --angles \
  --origin "$HW_TEST/preflight/origin.json" --output "$HW_TEST/execute"
```

执行版本会请求底盘使能，退出时取消所持目标并请求停用；完成后不要假设 RViz 新目标还能立即运动。输出目录中的 `STOP` 文件或 Ctrl+C 会结束该测试。正式正常关停仍使用第 8 节入口并核对反馈。

默认不指定方向/角度时会包含 x+/x−/y+/y− 和 ±60°，不是单点测试；先从上述有界子集开始。位置、yaw、停稳与漂移均记录，SLAM 相对精度不等于独立地面真值精度。

## 6. 两点持续验证与自主探索

两点工具仅供已经确认的空旷试验区域使用，固定为原点坐标系内 `x=+1 m,yaw=+90°` 与 `x=−1 m,yaw=−90°` 往返。其中心轨迹区域 `|x|≤1.35 m, |y|≤0.30 m` 是脚本活动边界，**不是实体通道宽度或机身净空保证**。

需要当前 SLAM 会话中核对过的 origin.json，脚本的 boot_id 和可选进程身份检查不能代替坐标连续性的现场确认。先预检：

```bash
ORIGIN_JSON=/绝对路径/本次已核对的origin.json
ENDURANCE_RUN="$HOME/.ros/log/astribot/two_goal_$(date +%Y%m%d_%H%M%S)"
python3 tools/robot/two_goal_endurance.py --origin "$ORIGIN_JSON" \
  --cycles 3 --output "$ENDURANCE_RUN/preflight"
```

确认规划与区域条件后才添加 `--execute`，使用新的输出目录。每批 cycles 为 1～20，检查结果后再开下一批；不要跨 SLAM 重启沿用旧原点。该工具不是任意路线执行器，也没有沿用仿真六点坐标。

自主探索使用单独入口，会重启链路并自动行走：

```bash
bash tools/robot/run_deployed.sh explore --dry-run
bash tools/robot/run_deployed.sh explore
```

暂停/恢复探索，在另一个已加载环境的终端执行并检查响应：

```bash
ros2 service call /exploration_coordinator_node/pause std_srvs/srv/Trigger '{}'
ros2 service call /exploration_coordinator_node/resume std_srvs/srv/Trigger '{}'
```

两条命令对应不同操作时刻。暂停保留进程，完整结束用正常停止入口；探索时不要并行运行路线工具或人工目标。在 deployed_sensors.json 配置 save_path/map_name；停止前用 slam_session save 完成保存并校验 manifest。结束进程不代表地图已保存。

## 7. 当前参数与真机能力限制

| 项目 | precision / explore 入口 |
|---|---|
| 时钟 | `use_sim_time=false` |
| 跟踪器/策略 | MPPI / off |
| 到位精度档 | hardware：3 cm / 1.5°，来源默认 nav2_pose |
| 地图/扫描 | `/map`、transient-local 地图订阅；`/scan_from_cloud` |
| 线速度 | x/y 轴上限 0.35 m/s，实际受曲率、阶段、误差及上游约束影响 |
| 到位运动参数 | 与 simulation_precision 共用 arrival_motion.yaml |
| 当前精调预算 | 135 s；应查询运行参数确认部署是否更新 |
| 制动适配 | 硬件档的响应时间及平移余移模型 |
| 低速响应模式 | monitor，只记录；compensate 不默认开放 |

在运行环境核对有效值：

```bash
ros2 param get /controller_server precise_goal_checker.xy_goal_tolerance
ros2 param get /controller_server precise_goal_checker.yaw_goal_tolerance
ros2 param get /controller_server FollowPath.arrival.source
ros2 param get /controller_server FollowPath.arrival.refine_timeout
ros2 param get /controller_server FollowPath.slip.mode
ros2 param get /global_costmap/global_costmap static_layer.map_topic
ros2 param get /global_costmap/global_costmap static_layer.map_subscribe_transient_local
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.expected_update_rate
```

期望扫描缓冲时效为 0.3 **秒**；它不改变雷达发射率，也不替代桥接输入时效及制动性能。配置在 configure 阶段读取的参数需重新配置/启动才生效，不能默认 `ros2 param set` 已改变算法。

真机暂不通过修改阶段开关启用 P2～P5：还需硬件 profile、制动/时延/包络/负载/覆盖证据、地图话题及 QoS 接线、起步恢复专项验收。85 cm 通行和通道内终点禁止原地旋转仍有未完成项。

接入视觉/mark 精调前，要保证输入是导航基座位姿，完成相机/标记外参、坐标、时间和有效性处理，再按[架构手册](ARCHITECTURE_REFERENCE.md)选择 arrival.source。不能将图像中的标记坐标直接当 map 下基座坐标。

## 8. 正常停车与关停

机器人任意终端可用绝对路径，无需预先加载环境：

```bash
bash /home/astribot/astribot_projects/current/tools/robot/stop_robot_tasks.sh --dry-run
bash /home/astribot/astribot_projects/current/tools/robot/stop_robot_tasks.sh
```

第一条只列出目标，第二条实际关停。顺序为取消/暂停任务→零速度与桥接停用→感知仍运行时检查停车反馈→关闭导航/桥接→关闭识别到的感知/雷达→复查选定进程。

也可在主管前台 Ctrl+C，或给**已核对归属的会话**创建 STOP 文件：

```bash
touch "$HOME/.ros/log/astribot/latest_hardware_precision/STOP"
```

explore/sensors 使用对应索引。`latest_*` 不是活动性证据，使用前核对 session.json；只关闭指定会话可用 `stop_robot_tasks.sh --session /实际会话目录`。

| 结果 | 含义 |
|---|---|
| `tasks_closed=true, feedback_stopped=true` | 选定任务结束，关闭感知前的反馈满足停稳要求 |
| `feedback_stopped=null` | 没有匹配运动任务或仅关闭非运动组件，不代表整机一定静止 |
| 返回 1 / remaining 非空 | 仍有残留或流程错误，检查原主管/系统服务 |
| 返回 2 / feedback_stopped=false | 反馈缺失、过旧、多源或未满足判据，需现场确认 |

停车反馈要求 `/odom` 单一来源、至少 4 帧推进时间戳且覆盖 0.5 s，速度 ≤0.01 m/s、角速度 ≤0.02 rad/s、位移变化 ≤5 mm、角变化 ≤0.01 rad。SDK odom 是软件证据，不代替现场观察或独立急停。紧急情况用独立急停，不等待 SSH/脚本。

清理保留厂家本体驱动、外部已有模型/状态链、无关摄像头、SSH 等；不删除地图。只作用于识别到的本机任务，不能关闭其他主机的控制源。完整识别范围见[部署记录](../ROBOT_DEPLOYMENT_20260915.md)。

## 9. 日志、诊断与回滚

完整入口默认日志：`~/.ros/log/astribot/hardware/<模式>_<时间>.../session.log`。索引为 `latest_hardware_precision`、`latest_hardware_explore`、`latest_hardware_sensors`。

```bash
HW_SESSION="$(readlink -f "$HOME/.ros/log/astribot/latest_hardware_precision")"
cat "$HW_SESSION/session.json"
rg 'PATH_TRACKING/|ARRIVAL_REACHED|SLIP_METRICS|ERROR|拒绝|超时' "$HW_SESSION/session.log"
```

检查 `lidar_check.json`、`input_check.json`、相应 stderr 日志及部署清单。周期控制诊断默认 2 Hz；性能验证优先只录所需速度、定位、阶段、TF 及厂家反馈，避免全话题录包负载改变控制/观测时序。

| 现象 | 排查方向 |
|---|---|
| `/map` 没发布者 | 统一入口必须提供 `/map`；检查 Voxel 状态、nav_prob_grid、QoS 与 StaticLayer 初始化 |
| 地图错位 | map/aft_mapped/odom、grid origin、TF、雷达外参；仅改 frame 名无法修复栅格坐标 |
| `/scan_from_cloud` 无有效消息 | 雷达实际数据→模型/关节反馈→连杆 TF→自滤/扫描；不能只数进程 |
| SDK 控制权拒绝 | 找原控制方释放，不能叠加第二个桥接或关闭保护 |
| 到点后漂移/反复修正 | 位姿源龄期、真实停稳窗口、制动模型、执行余移、低速响应日志 |
| monitor 报低响应 | 先区分定位/执行/轮地响应，不把 SLAM 与厂家差异直接诊断为机械打滑 |
| 重启后固定路线异常 | 原点和地图是否重建；重新核对，不强制复用旧坐标 |
| 包前缀不是当前 release | 新终端重新加载应用环境，核对已运行进程版本 |

回滚：使用当前运行版本入口正常停止并确认停车→核对旧 release 的安装产物→把 `current` 指回已保存的旧目录→新终端加载旧环境→只读检查→precision 小目标复验。不要在运动中切链接或替换动态库。

回滚不能自动撤销厂家 SDK、设备网络/外参、地图状态和现场标定；当前 SLAM 和驱动代码随应用回退。必须保存外部依赖与配置版本，不能只保存 Git commit。

## 10. 真机验收顺序

先确认输入/坐标与零运动状态，再做空旷短直线、四向低速、起步对齐、曲线/接近段及停稳复测；最后才考虑动态避障和窄通道。每次保留相同基线与候选的现场、负载、定位、参数和测试原点。

验收同时覆盖 ≤3 cm / 1.5° 到位、停止后的漂移、直线横向/航向误差、速度波动、加减速与 jerk、非预期旋转和停车/恢复。跟踪耗时不计质量排名。低速自动补偿、视觉/mark 和策略阶段分别验收，不能由普通 MPPI 到点通过一次性放行。

## 11. 双臂运输与整机功能的当前边界

现有人工导航/探索入口不等于正式搬运任务入口。导航仲裁只管理导航 Action；臂桥接的本地锁不管理底盘，也不提供双臂轨迹同步。包络服务已有停车变更/双 costmap 确认，但双臂实际姿态和载荷尚未自动提交到该服务。

因此，后续搬运验收必须单独验证抓持成功、实际运输姿态、载荷附着、包络版本与足迹确认、导航停稳、放置及异常保持。不要简单删除臂执行锁来开启双臂并行，也不要把 `use_wbc=true` 当作已完成全身协同。现有精度档和固定机身模型的仿真通过，仅支持其记录场景。

具体缺口和实施顺序见[整机架构审查](../WHOLE_ROBOT_ARCHITECTURE_REVIEW_20260917.md)。视觉抓取、力控、同步双臂、低电量回充和边走边操作均需各自的真机接口与验收记录。
