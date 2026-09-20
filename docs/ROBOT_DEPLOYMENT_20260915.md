# 10.249.22.137 到位验证与自主探索操作手册

本手册对应 2026-09-15 的源码快照。所有“机器人终端”命令都在 `astribot@10.249.22.137` 执行；开发机目录是 `/home/yjh/WorkSpace/astribot_sdk_ros2`，不要把这个路径写入机器人启动命令。

## 1. 部署位置与范围

| 内容 | 机器人上的位置 |
|---|---|
| 本次应用版本 | `/home/astribot/astribot_projects/releases/20260915_tracking_metrics` |
| 便捷入口 | `/home/astribot/astribot_projects/current`，指向本次验证后的版本 |
| ROS 源码 / ARM 编译产物 | 本版本下 `ws_robot/src` / `ws_robot/install` |
| 厂家 ARM64 SDK | `/home/astribot/Downloads/astribot_sdk_aarch64` |
| 现有 SLAM 与辅助脚本 | `/home/astribot/SLAM/vxlm-slam`、`/home/astribot/s1_tools` |
| 操作手册 | 本版本下 `docs/ROBOT_DEPLOYMENT_20260915.md` |
| 源码校验清单、构建与加载记录 | 本版本下 `deployment/` |

部署包含当前 ROS 包、机器人模型、启动及测量工具、示例、配置和文档，包括未提交的最新路径跟踪改动。厂家 `astribot_sdk`、`astribot_msgs`、`third_party` 使用机器人原有 ARM 版本，通过链接复用；没有把开发机的 x86 动态库复制过去。

开发机的 `build/install/log/runs`、Git 历史和 Python 缓存不参与部署。Gazebo 启动包、仓库仿真场景和自带 Livox 驱动源码保留，但不参加真机编译；真机继续使用已安装的厂家雷达驱动。不要删除厂家的 SDK 目录，否则原生库链接会失效。

当前默认入口为 **precision 人工目标到位验证**：重启雷达、SLAM、导航和桥接，通过既有检查并使能后等待人工目标；不启动探索协调器。`start`、`restart` 同样进入 precision。只有明确执行 `explore` 才会自动选择探索目标。无参数入口会启动并使能，不是只读检查。

初次同步部署时未启动真机探索、未申请 SDK 控制权、未给真机发送速度，也未关停现有任务。后续真机启动记录见第 10 节。`current` 链接只选择后续启动使用的版本，不会切换已经运行的进程。

## 2. 现有任务与切换范围

2026-09-15 自主探索部署复查时，旧导航与探索通过 `/home/astribot/s1_tools/roslaunch.py` 启动，使用 `/home/astribot/Downloads/astribot_sdk_aarch64/ws_robot` 中的代码。更早还发现过 `/home/astribot/Downloads/lw_test/astribot_sdk_ros2` 实例；不能仅根据目录名判断此刻运行的是哪套代码。

`precision` / `explore` / `restart` / `sensors` 都会先关停识别到的旧实例，不再使用“已运行就跳过”。如只需预览或单独关停，在机器人终端执行：

```bash
bash /home/astribot/astribot_projects/current/tools/robot/stop_robot_tasks.sh --dry-run
bash /home/astribot/astribot_projects/current/tools/robot/stop_robot_tasks.sh
```

脚本识别本项目新版本和旧工作空间中的导航/探索、速度转换、桥接、配套 RViz，以及 Livox 雷达、voxelslam、nav_prob_grid、扫描转换、定位差分里程计、地图转发与配套静态坐标别名。先结束运动任务，再关闭感知链，按 PID 与进程启动时间操作，包含已识别的重复实例。雷达/SLAM 按本机已知路径识别，不因启动在另一个 ROS 域而跳过；其他导航与辅助节点按当前真机域 25 识别。

**保留**厂家本体驱动、机械臂驱动、外部已运行的机器人模型和关节状态发布者、其他无关 TF、摄像头、SSH 和远程桌面。本次会话自己启动的状态桥和模型节点会随会话关停。不会修改雷达网络配置、SLAM 标定文件或删除已有地图文件。其他主机上的发布端、未识别的自定义程序不在清理范围内；若实例被系统服务自动拉起，脚本复查残留后退出，由现场处理其原管理服务。

SLAM 重启会重新初始化定位与地图。旧导航目标先被取消，里程计差分及地图清理轨迹也随辅助节点重启，避免跨两次定位混用坐标；不要照搬上一轮 map 坐标下的目标。现有 `mid360.yaml` 的 `is_save_map=0`，结束任务不代表已经保存本轮地图。

过去发现过 `/odom`、`/scan`、`/map_nav` 多发布端问题。本次会重启识别到的本机来源，仍要求启动检查中各有一个选定发布端；来自其他机器的重复话题会使检查失败，不能把开发机仿真并入真机 DDS 域。

## 3. 登录与加载环境

**开发机终端：**

```bash
ssh astribot@10.249.22.137
```

**机器人终端，每开一个用于操作本项目的新终端都执行：**

```bash
cd /home/astribot/astribot_projects/current
source tools/robot/env_deployed.sh
```

应看到 `PROJECT` 指向本次版本、`SDK` 指向厂家 ARM SDK。环境采用实测厂家 DDS 设置：`ROS_DOMAIN_ID=25`、`ROS_LOCALHOST_ONLY=0`、`rmw_fastrtps_cpp`，配置文件为 `/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml`。SSH 使用的 `10.249.22.137` 与机器人内部 SDK 网段是不同用途。

检查实际使用的新包：

```bash
ros2 pkg prefix astribot_s1_path_tracking
ros2 pkg prefix astribot_s1_navigation
ros2 pkg prefix astribot_trajectory_bridge
```

输出应位于本次版本的 `ws_robot/install`。不要只 source 厂家旧 `env_robot.sh`，那会加载旧工作空间；也不要在机器人上 source 项目根目录用于开发机的 `env.sh`。

## 4. 只读检查

```bash
bash tools/robot/run_deployed.sh check
```

此命令只订阅话题、读取 TF 和节点图，不创建 SDK 会话、不使能、不发速度。观测窗口默认 8 秒。雷达/SLAM 未启动时检查失败是预期结果；可先用第 5 节的 `sensors` 入口启动感知，或直接由 `explore` 完成整套启动。

| 检查 | 判据 / 含义 |
|---|---|
| `/scan`、`/odom` | 实际收到消息，接收频率至少 5 Hz、至少 3 个不同源时间戳、数据龄期不超过 0.5 s、各一个发布端 |
| `/map_nav` | 一个选定发布端，实际收到非空地图，地图坐标系能转换到 `map`；地图允许静态或低频更新 |
| TF | `map → astribot_torso_base` 可查且新鲜 |
| `navigation_conflicts` | 现有导航/调度节点；启动新导航时要求为空 |
| `bridge_conflicts` | 现有 SDK 桥接节点；启动新桥接时要求为空 |
| `ready` | 传感器与 TF 的基础检查结果；默认 check 同时列出冲突，但不因冲突本身退出 |

保存检查结果：

```bash
python3 tools/robot/hardware_readiness.py --require-idle-navigation --require-idle-bridge > /tmp/astribot_prestart.json
cat /tmp/astribot_prestart.json
```

旧脚本以 20 Hz 轮询约 10 Hz 的 TF，重复时间戳的第二帧可能带默认零速度。当前入口改用项目内 `tools/robot/tf_to_odom_node.py`：首帧建立差分基线，只在源时间戳前进时发布；重复和倒序帧跳过，真实 ROS 时钟回退时重新建立基线。原 `/home/astribot/s1_tools/tf_to_odom_node.py` 文件保留。探针分别显示 `messages` 与 `unique_stamps`。这里的 `/odom` 仍是定位派生反馈，不是独立地面真值；SDK 桥接反馈另发 `/astribot/chassis/odom_from_sdk`，不会再占用 `/odom`。

地图转发也使用本项目 `tools/hardware_nodes/grid_self_clear_node.py`，不修改同事的 `/home/astribot/s1_tools` 副本。在地图自身坐标系中，仅把完整落入当前未加 padding 的机身正方形（半宽 0.31 m）内的占据格清为自由格；边界跨越格、机身外障碍和未知格保留。沿用 20 s / 0.25 m 的历史小圆盘，历史清理范围不扩大。每次清理要求新鲜的机身 TF，按地图原点旋转和当前机身朝向计算；TF 不可用时原图转发。轮廓参数必须随实体外形和导航 footprint 一起复核，机身覆盖范围内若存在真实接触物，仍需先人工移除。


## 5. 一键启动与人工到位验证

当前到位验证在机器人终端执行：

```bash
cd /home/astribot/astribot_projects/current
bash tools/robot/run_deployed.sh precision --dry-run
bash tools/robot/run_deployed.sh precision
```

第二条实际启动命令会重启感知与导航、使能底盘，然后等待人工目标。机器人图形桌面的另一终端运行 `bash /home/astribot/astribot_projects/current/tools/robot/run_deployed.sh rviz`，使用 RViz 的 Nav2 Goal 工具发送目标。先在当前地图中设置前方约 0.3～0.5 m、同朝向的小目标；确认到位及停稳后，再逐个测试返回、45°/90° 朝向改变和小距离横移。每次只发一个目标，记录 `ARRIVAL_REACHED` 的位置与角度误差，并观察停稳后是否继续漂移。SLAM 重启后必须重新选择当前地图下的坐标。

此模式保持 `navigation_policy_stage=off`，关闭独立让行/绕行策略，保留路径碰撞检查、SDK 偏差保护和正常停车。若路径被碰撞检查拒绝，应选择可通行目标；不能把该结果当成到位精度测试。

需要恢复环境自主探索时，显式使用以下入口：

以下命令在机器人终端执行。确认现场允许机器人自主移动、机械臂周围留有空间且独立急停可用。入口会自动结束旧导航及雷达/SLAM实例，保留厂家本体驱动和外部已有模型。

```bash
cd /home/astribot/astribot_projects/current
# 只查看计划，不启动节点、不申请 SDK 控制权
bash tools/robot/run_deployed.sh explore --dry-run
# 实际启动：通过检查后将自动行走探索
bash tools/robot/run_deployed.sh explore
```

无参数、`start` 和 `restart` 均进入上面的 precision 模式。启动会占用当前终端；保留终端查看状态，另开终端执行关停。程序不安装开机自启服务。

启动管理程序依次执行：

1. 核对厂家启动文件及依赖路径，停车并取消旧目标，退出旧导航、桥接和感知实例；复查残留，未退出或被自动拉起则不叠加启动。
2. 启动厂家 Livox 驱动，等待前后雷达点云和 IMU 各自收到唯一、新鲜且持续推进时间戳的数据。默认至少 5 Hz、龄期 ≤ 0.5 s、观测跨度 ≥ 1 s，最多等 45 s。
3. 复用已有 `/joint_states` 和模型发布者；缺失时启动现有状态桥的 `manufacturer` 反馈模式及 `robot_state_publisher`。该模式直接读取厂家关节反馈并保留源时间戳，不创建 SDK 会话。随后启动 voxelslam 和其 nav_prob_grid，重启扫描转换、`tf_to_odom_node.py`、`grid_self_clear_node.py` 及地图坐标别名。沿用机器人当前参数与标定，不修改 SLAM 算法。
4. 最多等待约 120 s，让 `/map_nav`、`/scan`、`/odom` 和 TF 满足第 4 节检查。检查失败的每轮记录保留；此时不构造 SDK 会话。
5. 启动现有 MPPI 导航和初始停用的底盘桥接，等待 7 个 Nav2 生命周期节点全部 `active`，再请求并确认底盘使能。桥接服务刚出现时，其内部 TF 缓存可能尚未收到位姿；仅对已有的“位姿源不可用”回复，在原 60 s 等待范围内重试，其他拒绝立即报错。如果 SDK 控制权被其他程序占用，应从原控制方释放。
6. precision 模式等待人工目标；explore 模式启动探索协调器，自动选择前沿目标，经 `/exploration/navigate_to_pose` 与任务仲裁器交给现有 Nav2 执行；到点确认、停留后再选下一目标。

每轮等待及最终超时都会直接显示现有检查报告的 `errors` 和报告绝对路径。例如 `/scan: no messages received` 表示扫描探针未收到数据，不等于 SLAM 没启动，也不能据此判断急停状态。若检查程序自身异常，终端会给出退出码和 `input_check.stderr.log` 路径。此输出不增加检查项、观测窗口或通过条件。

只重启雷达、SLAM 与关联感知，不进入导航探索：

```bash
bash /home/astribot/astribot_projects/current/tools/robot/run_deployed.sh sensors
```

此入口也会先结束旧导航，随后保持前台管理感知子进程，不启动新的导航、SDK 或底盘使能。可用于检查建图和雷达数据；退出仍使用 Ctrl+C 或一键关停。再次运行 `sensors` 或 `explore` 会替换上一轮，不能把它当成“保留当前地图继续导航”。其只读预览为 `sensors --dry-run`。

传感器部署参数在 `tools/robot/config/deployed_sensors.json`：雷达使用 `/opt/astribot_ros/software/livox_ros_driver2`，SLAM 使用 `/home/astribot/SLAM/vxlm-slam`。启动环境分别在子进程中加载，避免 SLAM 工作空间覆盖新导航包。厂家本体驱动及真实关节反馈仍由原系统提供。本入口补齐状态桥和模型发布节点，不负责重新启动整台机器人。部件顺序、关节名和单位换算沿用 `astribot_trajectory_bridge/config/bridge.yaml`；不会用默认零姿态替代反馈。

现场急停可能使关节状态不可用。扫描自滤依赖机器人连杆 TF，连杆 TF 全部不可用时，原有感知节点会停止输出有效 `/scan`；启动入口仍按原条件等待，超时后结束本次会话。脚本不会补造零关节状态或自动解除急停。需结合 `session.log` 中的连杆 TF / 自滤错误定位原因，仅凭 `/scan` 无消息不能确定是急停还是模型发布链缺失。状态发布链现已纳入启动流程；若厂家反馈中断，对应连杆不会被其他部件的新反馈刷新时间戳。

`/map_nav` 的 `camera_init` 帧必须与 `map` 存在恒等变换。若改为带非零平移/旋转的地图坐标映射，需要先归一化栅格；入口会拒绝混用坐标。

探索参数在 `tools/robot/config/deployed_exploration.yaml`，桥接参数在 `tools/robot/config/deployed_chassis.yaml`。探索协调器只发目标。真机和仿真均通过 `navigation.launch.py` 读取 `nav2_params_mppi.yaml`，入口线速度均为 `max_linear_speed:=0.35`，不再叠加硬件专属的低速配置。到位运动配置共用 `arrival_motion.yaml`；硬件配置仅保留定位判据、桥接状态订阅和执行器延迟适配。

MPPI 的 x/y 轴上限各为 0.35 m/s，角速度上限为 2.0 rad/s；初始大角度对齐上限为 0.6 rad/s，到位微调上限为 0.06 m/s、0.15 rad/s。平滑器 x/y 加速度为 0.5 m/s²、减速度为 2.5 m/s²，角加减速度为 3.2 rad/s²。这些与当前仿真基线相同，是各阶段上限，实际指令由跟踪误差、曲率和阶段决定；0.35 m/s 是轴向上限。桥接不再二次限速或限加速度。

桥接已删除速度限幅及加减速限制，不再二次改变导航输出。速度日志显示输入与实际用于积分的线/角速度、停车置零拍数及 SDK 指令/反馈位移。机械臂/夹爪写通路关闭；SDK 偏差保护、指令超时、扫描/定位时效检查继续生效。直接向桥接输入速度也不会再受桥接限幅，上游运动指令应经过导航平滑器。

如需暂时暂停探索、稍后继续，可在已加载环境的另一终端使用：

```bash
ros2 service call /exploration_coordinator_node/pause std_srvs/srv/Trigger '{}'
ros2 service call /exploration_coordinator_node/resume std_srvs/srv/Trigger '{}'
```

检查服务返回。暂停会保留进程；需要结束本轮任务时使用下一节的一键关停。

可选 RViz：在机器人图形桌面终端执行，普通 SSH 终端不一定具备图形显示环境。

```bash
bash /home/astribot/astribot_projects/current/tools/robot/run_deployed.sh rviz
```

`navigation`、`bridge` 两个独立角色入口保留供调试；正常探索只运行 `explore`，无需再手工启动第二套导航或使能桥接。探索过程中也不要并行运行路线脚本或从 RViz 派发额外目标。

## 6. 一键关停当前任务

**机器人任意终端，一条命令，无需先 source 环境：**

```bash
bash /home/astribot/astribot_projects/current/tools/robot/stop_robot_tasks.sh
```

等价入口：

```bash
bash /home/astribot/astribot_projects/current/tools/robot/run_deployed.sh stop
```

**从开发机远程一键关停：**

```bash
ssh astribot@10.249.22.137 \
  'bash /home/astribot/astribot_projects/current/tools/robot/stop_robot_tasks.sh'
```

停止流程：暂停探索/取消已知导航动作 → 发布零速度并请求桥接停用 → **在 SLAM 与里程计仍运行时检查停车反馈** → 退出导航/探索/桥接 → 结束 SLAM、关联感知和雷达 → 核对选定进程是否退出。正常退出无效时，仅对这批确认身份的进程升级终止信号；控制任务仍未退出时会保留感知用于停车反馈。不会关闭 SSH、关机或删除日志。

配套静态 TF 的识别同时兼容 `/opt/ros/humble` 和厂家 `/opt/astribot_ros/middle_ware` 中的 `static_transform_publisher`，限定已知项目环境及 `map → camera_init`、`camera_init → odom`、`aft_mapped → astribot_torso_base` 三组坐标关系；其他静态 TF 和本体驱动保留。

在启动终端按 **Ctrl+C** 会清理本次完整会话，包括雷达和 SLAM。也可创建该会话的 `STOP` 文件；人工到位模式示例如下，探索/感知分别使用 `latest_hardware_explore` / `latest_hardware_sensors`：

```bash
touch "$HOME/.ros/log/astribot/latest_hardware_precision/STOP"
```

只停止指定探索会话、保留其他匹配任务时：

```bash
bash /home/astribot/astribot_projects/current/tools/robot/stop_robot_tasks.sh \
  --session "$(readlink -f "$HOME/.ros/log/astribot/latest_hardware_explore")"
```

| 返回码 / 输出 | 含义 |
|---|---|
| `0`，`tasks_closed=true`、`feedback_stopped=true` | 选定任务进程已退出，收到满足停车判据的反馈 |
| `0`，`feedback_stopped=null` | 没有匹配的运动任务，或仅关闭感知/检查/显示进程；未判定机器人整体运动状态 |
| `1` | 仍有选定进程未退出，或命令/启动流程失败；查看错误与 `remaining` |
| `2`，`feedback_stopped=false` | 选定进程已退出，但停车反馈缺失、过旧、多源或尚未满足停稳判据；必须现场确认 |

停车反馈在关闭传感器之前采样，报告中的 true 是当时的停车判定；SLAM 关闭后不会继续产生新反馈。停车判据要求 `/odom` 恰好一个发布端，源时间推进，至少 4 帧、覆盖至少 0.5 s；窗口内速度 ≤ 0.01 m/s、角速度 ≤ 0.02 rad/s，位姿变化 ≤ 5 mm / 0.01 rad。源时间重复帧不重复计入。SLAM 差分里程计只能作为软件反馈，不能代替观察机器人或独立急停。

紧急情况直接使用独立急停，不等待 SSH 或服务响应。一键脚本是正常软件关停，不代表机械急停性能，也不能关闭其他机器上的控制方。

## 7. 到位、路径跟踪和重规划设置

| 项目 | 本次真机入口 |
|---|---|
| 时钟 | `use_sim_time=false` |
| 到位配置 | `arrival_precision_profile=hardware`；与 simulation_precision 共用 arrival_motion.yaml |
| 位置/角度判据 | 欧氏误差 ≤ 0.03 m，最短角度误差 ≤ 1.5° |
| 到位位姿来源 | `nav2_pose`，由当前导航 TF 提供定位位姿 |
| 微调速度 | 两端统一：线速度 0.008～0.06 m/s；角速度 0.02～0.15 rad/s；无需运动时输出零 |
| 防振荡 | 两端增益 0.5 / 0.8；真机用 1.5 s 执行器响应预测提前制动，越点或反向切换先输出零，连续新反馈停稳后再修正 |
| 粗细阶段与滞回 | XY/yaw 独立保持；先收敛 XY 后检查最终 yaw；65% 容差进入保持、90% 容差恢复微调。新目标清除制动状态 |
| 微调时间上限 | 与仿真统一为 45 s；仍保留源时效、停稳和无进展判定 |
| 控制器稳定保持 | 0.6 s，并检查速度、位姿源时间推进及真机残余运动裕量；重复时间戳不能积累停稳时长 |
| 探索到点复核 | 同一 3 cm / 1.5° 判据，通过后停留 1.5 s 再选下一目标 |
| 周期诊断 | 2 Hz：`TRACKING_METRICS` / `ARRIVAL_METRICS` |
| 最终到位事件 | `ARRIVAL_REACHED`，立即记录 |
| 全局规划 | 沿用当前按新目标/路径风险触发的行为树；探索 `replan_policy=on_invalid`、`path_max_age_sec=0`，不增加定时重规划 |
| 动态避障/窄通道策略 | 代码完整部署；本入口 `navigation_policy_stage=off`，现有阶段配置仍是仿真标定，需要按硬件验证方案准入 |

仿真真值定位保持 2 mm / 0.1°，真机 SLAM 使用 3 cm / 1.5°。用户已确认只按定位源区分到位判据，运动参数共用。真机测量证据位于 `tools/robot/config/chassis_measured_parameters_20260914.json`；1.5 s 制动预测是模型验证的执行器适配起点，不能视为已经测得的全速度范围延迟或真机复测结论。

修改运动参数时审查 `nav2_params_mppi.yaml`、共同的入口上限 0.35 和 `arrival_motion.yaml`；不要在硬件入口再叠加一份低速配置。修改后构建 `astribot_s1_navigation` 并重启导航。真机 `arrival_precision_hardware.yaml` 的制动时间只在终点区域生效；正常 FOLLOW 和 ALIGN_START 不受它影响。`ARRIVAL_COAST` 记录制动进入/退出，2 Hz 的 `ARRIVAL_METRICS phase=COAST` 可用于定位余转或余移；进展判据包含残余运动，避免高速过零点被当成稳定收敛。

真机“3 cm / 1.5°”是当前选定定位源内的控制验收条件。对物理地面的准确程度还需要用外部标记、视觉或测量工具独立验证。详细定义见 `docs/PATH_TRACKING_METRICS.md`，硬件验证见 `docs/NAVIGATION_POLICY_HARDWARE_VALIDATION.md`。

## 8. 查看日志与故障原因

雷达、SLAM、感知、导航和桥接输出汇入同一个 `session.log`。人工到位模式的便捷入口为（探索将 precision 改为 explore）：

```bash
readlink -f "$HOME/.ros/log/astribot/latest_hardware_precision"
tail -F "$HOME/.ros/log/astribot/latest_hardware_precision/session.log"
```

关注路径跟踪与到点事件：

```bash
tail -F "$HOME/.ros/log/astribot/latest_hardware_precision/session.log" \
  | grep --line-buffered -E 'TRACKING_METRICS|ARRIVAL_METRICS|ARRIVAL_REACHED|BRIDGE_|PATH_TRACKING'
```

| 会话文件 | 用途 |
|---|---|
| `session.log` 及轮转文件 | 雷达、SLAM、感知和导航各角色的统一日志 |
| `restart_stop_result.json` | 启动前关停的旧实例、停车反馈与残留情况 |
| `lidar_check.json` | 前后点云/IMU 的唯一发布端、实际拍率与新鲜度 |
| `session.json` | 本次版本、运行状态、进程身份和最终停车反馈 |
| `input_check.json` / `input_check_attempt_N.json` | 重启后的最新检查与失败轮次：数据源、TF 和控制冲突 |
| `input_check.stderr.log` | 检查程序的 ROS 错误输出 |
| `stop_result.json` | 取消/停用请求返回、剩余进程与停稳判定 |
| `STOP` | 人工或关停脚本写入的会话停止请求 |

仅感知模式的日志链接为 `~/.ros/log/astribot/latest_hardware_sensors/session.log`。

导航订阅现有 `/astribot/bridge/status` 的底盘锁存故障。SDK 偏差保护、定位/扫描失效停车或桥接停用会直接终止当前目标并保留原因，例如 `BRIDGE_LEASH`；不再等待 15 s 无运动超时才报泛化错误。底盘状态恢复不会自动续跑失败目标，需要现场确认后重新发送目标；原保护阈值未放宽。

机器人没有 `rg`，所以手册使用 `grep`。横向统计应区分正常跟踪段与终点接近段，`travel_sample=1` 对应 FOLLOW 且距终点超过 0.5 m。2 Hz 日志适合现场观察，不能代替高频原始数据的峰值分析。

## 9. 重新编译、再次部署与回退

**机器人上重新编译：** 使用干净环境，不沿用运行终端中厂家 SDK 的库搜索路径。

```bash
env -i HOME=/home/astribot USER=astribot LANG=C.UTF-8 \
  PATH=/usr/local/bin:/usr/bin:/bin CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  bash --noprofile --norc \
  /home/astribot/astribot_projects/current/tools/robot/build_robot.sh \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

**开发机再次部署：** 生成新的版本目录，不覆盖本次目录。

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
bash tools/robot/deploy_project.sh
```

脚本使用 SSH + rsync，同步当前源码快照、核对 SHA256，再在 ARM 机器人编译。凭据不写入脚本。编译成功并完成新版本检查后，才修改 `current` 链接；部署脚本自身不会重启任何服务。

**回退：** 先正常停车、退出本版本导航和桥接，再从旧项目原入口启动；需要切换到其他已验证部署版本时，将 `current` 改指向那个完整目录。不能在机器人运动或 SDK 桥接仍启用时同时启动两套导航。保留旧目录是为回退提供文件，不表示旧软件与新软件可以同时接管底盘。

实际安装的厂家命令名为 `ros2 launch livox_ros_driver2 msg_MID360_launch.py`，以及在 `/home/astribot/SLAM/vxlm-slam` 加载 `install/setup.bash` 后执行 `ros2 launch voxel_slam vxlm_mid360.launch.py`。部署入口调用同一组 launch 文件的绝对路径，避免不同 overlay 选错包；不需要在另一个终端重复启动。

## 10. 本次验证记录与边界

证据保存在本版本 `deployment/`。基础部署记录包括 `source_manifest.json`、`build_native.log`、`isolated_check.json`、`hardware_inputs.json` 及 `deployment_status.json`；自主探索增量记录为 `exploration_deployment_status.json` 与 `exploration_validation/`。

基础部署已完成 17 个真机应用包的 ARM 编译、厂家 SDK 类与桥接模块导入，以及路径跟踪动态库依赖加载。自主探索增量只修改运行工具、配置和手册，沿用这些构建产物。

前一版自主探索验证：

- 20 项离线检查通过，覆盖新旧任务识别、传感器/厂家进程保护、PID 复用保护、实际子进程退出和停车反馈判定。
- 在机器人 localhost、ROS 域 127 的隔离环境中，原生探索协调器经真实任务仲裁器顺序派发 2 个前沿目标；导航最大并发数为 1。规划与执行后端、地图、里程计是测试替身，未使用 SDK。取消/暂停、桥接停用和进程退出通过验证，停车判定为 true。
- 在 localhost、ROS 域 128 中，真实 Nav2 的 7 个生命周期节点全部 active 后，启动管理程序成功使能模拟桥接并进入探索运行状态；分别验证 Ctrl+C 与独立关停命令，两种方式均返回 0、任务退出且停稳反馈通过。SDK 桥接和传感器为测试替身。
- 真机当前任务的关停清单已执行只读预览，未对其发送终止信号或停车指令。

本次增量已通过 27 项离线检查，以及完整探索、仅感知、雷达数据超时三个隔离场景。真机只读雷达检查测得前后点云约 10 Hz、IMU 约 200 Hz，点云龄期约 0.14 s；检查读取序列化消息头，避免全量转换每帧约 2 万点拖慢探针。

本次雷达/SLAM 启停增量证据为 `deployment/sensor_restart_validation/` 与 `deployment/sensor_restart_deployment_status.json`。验证使用独立 ROS 域中的传感器/SDK 替身，覆盖重复实例退出、雷达未就绪阻止导航、仅感知启动、完整导航启动，以及在感知退出前判定停稳。机器人上的厂家路径、消息类型和启动文件另做只读核查；本次同步未执行真实雷达/SLAM重启。

这些记录验证软件接线与退出流程，不代表真实地面上的探索覆盖率、避障效果或停车距离已经验收。本次未在真机 DDS 域 25 派发探索目标、发送底盘速度或构造 SDK 会话。

2026-09-15 启动脚本修正记录为 `deployment/startup_fix_validation/` 和 `startup_fix_deployment_status.json`。修正范围为现有输入检查错误的展示，以及厂家中间件中的配套静态 TF 漏识别；未增加检查项，未更改跟踪、到位或急停相关逻辑。48 项离线检查通过，机器人 ARM 环境的 localhost / ROS 域 139～142 完成完整探索启动、仅感知启动、缺少 `/scan`、检查程序异常四个隔离场景；正常场景能启动并关停，异常场景输出具体原因且不进入使能。导航使用原生 Nav2，传感器和 SDK 桥接使用替身。本次仅同步文件和执行真实进程的只读识别预览，没有重启真实感知或验证急停解除后的完整真机链路。

加载环境时，厂家 `env.sh` 仍可能打印 `astribot_robotics_install/local_setup.bash` 或 `third_pkg/local_setup.bash` 不存在的旧打包提示。本次在这些提示出现的环境中完成了上述 SDK 导入和动态库加载；若后续出现新的 ImportError 或 `ldd` 的 `not found`，应按实际错误排查，不能仅凭“脚本执行到最后”判定可用。


### 2026-09-15 13:37 真机启动记录

本次已实际启动雷达、SLAM、模型状态链、Nav2、SDK 桥接和探索协调器，并成功使能底盘。会话为 `/home/astribot/.ros/log/astribot/hardware/explore_20260915_133709_bbmwfbx4`。

修复了两处启动问题：原流程缺少真实关节反馈到连杆 TF 的连接；桥接服务刚出现时过早请求使能，内部 TF 缓存尚未收到位姿。复用现有状态桥新增 `manufacturer` 反馈来源，原 SDK 读取模式保持可用；启动程序对“位姿源不可用”这一明确的暂态回复限时重试，没有关闭原保护。

17 项状态桥检查和模拟 TF 延迟的完整启动/关停验证通过。真实输入检查中 `/scan` 从 0 帧恢复到约 9.83 Hz；运行期间另一次 10 s 观察收到 80 帧扫描，最新帧龄期约 0.13 s，厂家底盘反馈收到 2,501 帧。Nav2 的 7 个生命周期节点全部为 active。

探索已实际派发目标，但执行路径被完整足迹碰撞检查拒绝，连续失败后进入 PAUSED。一次只读复查中，到 `(0.98, 8.18)` 的规划路径共 164 个点，路径检查在第 153 个索引处判定不可通行，对应地图位置约 `(0.475, 7.675)`。这说明启动及使能链路已接通，不能据此声称自主巡航或探索覆盖效果验证通过。没有缩小足迹、清除障碍或关闭路径碰撞保护。

记录中的运行状态是当时快照；后续查看会话 `session.log` 中最新的探索状态。传感器和导航仍由同一个一键关停入口管理。
