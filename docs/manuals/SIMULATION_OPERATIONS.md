# 仿真操作手册

核对：2026-09-29。开发机仓库 `/home/yjh/WorkSpace/astribot_sdk_ros2`，ROS 2 Humble，当前仓库 Gazebo/Ignition 链。总览见[架构](ARCHITECTURE_REFERENCE.md)。本轮只验证静态接线、帮助和 dry-run，不启动仿真；下列运动步骤仍须在所属会话中验收。

## 1. 先选操作类型

| 目的 | 入口与前提 |
|---|---|
| 建图并手动导航 | `--mode mapping --navigation-policy p3`，共享六相机默认保留 |
| 自主探索 | `--mode explore --navigation-policy p3`；就绪后自动发目标，会移动 |
| 已有 Voxel 会话定位 | `--mode localize --map <完整会话目录> --navigation-policy p3` |
| 静态地图控制对照 | baseline 需要独立核对实测 SLAM 配准，见第 5 节；旧无参数 baseline 不能直接沿用 |
| 固定工位搬运 | fixed_v2 原生任务 + 正式保持/账本/分层源，见第 7 节 |
| 仅看界面/证据 | RViz 操作台/离线回放；打开界面不代表导航已就绪 |

**当前默认启动缺口：** 主管默认策略 off，navigation launch 默认上肢限速 true，而该组合被明确拒绝。本文显式选择 p3，保留上肢约束；这属于策略启用的仿真，不称为旧 off 性能基线。不改用关闭保护的方式消除报错。

## 2. 构建与环境

仅在该安装目录没有被运行会话使用时构建。新终端先加载系统 ROS，勿混入厂家 SDK 环境。

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
source /opt/ros/humble/setup.bash
bash ws_robot/src/astribot_s1_perception/scripts/prepare_livox_driver2.sh
bash tools/robot/build_slam_dependencies.sh
source tools/setup_mtc_humble.sh
export CMAKE_PREFIX_PATH="$PWD/ws_robot/deps/gtsam:${CMAKE_PREFIX_PATH:-}"
rosdep install --from-paths ws_robot/src --ignore-src -r -y
cd ws_robot
colcon build --base-paths src --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DGTSAM_DIR="$PWD/deps/gtsam/lib/cmake/GTSAM"
```

这些是构建步骤，不是本轮已执行结果。MTC 脚本可能下载其固定版本依赖；若版本不可取得，应记录失败，不能静默换版本。GraspNet worker 另需独立 LibTorch/模型，见[推理包说明](../../ws_robot/src/astribot_graspnet_runtime/README.md)。

已有会话占用共享安装时，另选唯一 build/install/log 目录进行 scoped build；启动器支持 `ASTRIBOT_OVERLAY_SETUP=/绝对路径/local_setup.bash`。不要只 source 私有 overlay 后假定启动器不会重新加载基础安装。检查 `ros2 pkg prefix`、ELF 实际链接和本次运行清单。

普通查询终端：

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
source /opt/ros/humble/setup.bash
source ws_robot/install/setup.bash
```

继续加载与所属会话完全相同的 MTC/私有 overlay 和 DDS 配置。`session/env.sh` 仅为起点，不能保证重建全部 overlay。

## 3. 会话隔离与启动

先确认现有 Gazebo、ROS、GPU 使用者和目标域是否空闲。下面 61 只是示例，**必须换成确认未使用的域**；隔离实例支持 1–101，排除 25。instance 自动派生 partition、发现端口和锁，仍共享 CPU/GPU。

```bash
SIM_INSTANCE="manual_mapping_$(date +%Y%m%d_%H%M%S)"
SIM_DOMAIN=61
SIM_RUN="$HOME/.ros/log/astribot/$SIM_INSTANCE"
SLAM_SAVE="$HOME/astribot_maps/$SIM_INSTANCE"
bash tools/launch_sim_stack.sh --instance "$SIM_INSTANCE" --ros-domain-id "$SIM_DOMAIN" \
  --mode mapping --navigation-policy p3 --save-session "$SLAM_SAVE" \
  --log-dir "$SIM_RUN" --dry-run
```

SIM_RUN 和 SLAM_SAVE 应是新目录。核对输出中的 mode、传感器 profile、domain、partition、导航策略、地图保存目录。dry-run 不占资源，不检查全部子 launch 条件，不证明运行就绪。

核对完成后，在所属前台终端启动同一配置：

```bash
bash tools/launch_sim_stack.sh --instance "$SIM_INSTANCE" --ros-domain-id "$SIM_DOMAIN" \
  --mode mapping --navigation-policy p3 --save-session "$SLAM_SAVE" --log-dir "$SIM_RUN"
```

无显示环境可加 `--headless --no-rviz`；性能实验可加 `--exclusive-performance`，它不会自动停止其他人的仿真。不要无参数启动后猜采用的配置。

ROS 风格与 CLI 风格二选一，不混用。统一 launch 现已补齐地图保存等转发，例如以下**预览**与上面的意图一致：

```bash
bash tools/launch_sim_stack.sh instance:="$SIM_INSTANCE" ros_domain_id:="$SIM_DOMAIN" \
  mode:=mapping navigation_policy:=p3 save_session:="$SLAM_SAVE" log_dir:="$SIM_RUN" dry_run:=true
```

## 4. 分层就绪与人工导航

查询环境必须来自本次会话的真实进程：ROS_DOMAIN_ID、RMW、FASTRTPS profile、localhost、partition、overlay。控制链采用 loopback UDP 时可能将 ROS_LOCALHOST_ONLY 设为 0，由 XML 限定网络，不能单凭该值判定对外广播。相机桥接可能使用独立 profile。

在环境已对齐的另一个终端：

```bash
cat "$SIM_RUN/session.json"
python3 tools/sim_stack_probe.py --phase data --timeout 20 --scan /scan_from_cloud
python3 tools/sim_stack_probe.py --phase navigation --timeout 30 \
  --scan /scan_from_cloud --costmap-scan /navigation_policy/costmap_scan --require-policy
ros2 param get /omni_effort_drive_node idle_position_hold
ros2 param get /omni_effort_drive_node idle_position_kp
ros2 param get /controller_server use_sim_time
ros2 topic info /slam/pose --verbose
```

期望轮控为 true / 3.0、仿真时钟为 true。探针需实际接收推进中的 clock、扫描、odom，并检查导航生命周期；此外核对 `/slam/pose` 唯一来源、源时间推进和 map 原点。话题有发布者、RViz 有画面或单帧 echo 均不足以证明就绪/停稳。

人工操作：RViz Fixed Frame 设 map，在已观测空旷区先选一个近距离目标。通过操作台时先申请控制会话；默认 WorkstationPanel 关闭后租约过期会请求取消。导航成功仍需 Action 终态、实际 SLAM 到位/停稳；不能只看机器人图标重合。

独立 UI（仅当本会话没有已合并的工作站窗口时）：

```bash
ros2 launch astribot_operator_station operator.launch.xml use_sim_time:=true
```

“循环路线”中逐点绘制并确认，再执行；只让一个任务来源持有目标。`tools/run_waypoint_route.py` 仍有 TF/odom 测量路径，不能用其旧停稳/漂移统计支持当前 SLAM 监测契约。

## 5. 地图保存、重定位及静态基线

结束探索/取消导航后先等动作终态与 SLAM 停稳，保留 SLAM 进程，然后保存：

```bash
ros2 run astribot_s1_perception slam_session save --timeout 120
ros2 run astribot_s1_perception slam_session inspect "$SLAM_SAVE"
```

保存要求启动时已设置唯一保存路径，完成检查 manifest、alidarState.txt、关键帧和二维地图，不把进程退出当作存图完成。UI 的结束建图/保存使用受管 mapping_session，优先保持该会话所有权。

结束旧会话后，新的定位会话可先预览：

```bash
bash tools/launch_sim_stack.sh --instance manual_localize --ros-domain-id 62 \
  --mode localize --map "$SLAM_SAVE" --navigation-policy p3 --dry-run
```

62 同样是待确认空闲的示例域；真正启动时使用新的日志目录并移除 dry-run。`--map` 是完整 Voxel 会话，不是单个 PGM/YAML；定位成功后才可派发地图目标。

**静态基线的现有限制：** baseline 选 `static_map`，当 launch_slam=true 时必须提供实测的 `--initial-chassis-pose 'x,y,z,qx,qy,qz,qw'`。姿态来自实际落稳测量并与地图注册，不能复制 spawn、填零或伪造 SLAM 样本。若由本会话另一个受管节点拥有 Voxel，才可显式 `--launch-slam false`；该参数不免除 SLAM 停稳依赖。当前没有在通用主管中自动完成这段两阶段测量/配准，因此不提供假装开箱即用的 baseline 一行命令。

## 6. 探索、禁行区与诊断

新建探索会话将上述 mapping 改为 explore，保留保存目录，启动后会自动行走。暂停/恢复是不同操作时刻：

```bash
ros2 service call /exploration_coordinator_node/pause std_srvs/srv/Trigger '{}'
```

确认目标终态及 SLAM 停稳后，按需要恢复：

```bash
ros2 service call /exploration_coordinator_node/resume std_srvs/srv/Trigger '{}'
```

暂停不是保存完成；结束建图后通过新会话继续，不复活旧目标。禁行区先绑定活动地图/版本，保存后检查消费者应用，详见[区域手册](VIRTUAL_WALLS_AND_KEEP_OUT.md)。

诊断录制可通过工作站“开始/停止记录”；默认不自动录制。同一图只运行一个 recorder，output_root 设持久目录。结束后检查状态、manifest、bag metadata 和 topic 计数，再打开离线回放；禁止读取活动 SQLite。

## 7. 操作规划与固定工位搬运

普通规划服务可在已有、确认归属的仿真上启动，模型和相机参数须与该世界一致：

```bash
source tools/setup_mtc_humble.sh
ros2 launch astribot_s1_transport transport_skills.launch.py allow_trajectory_execution:=false
```

此命令启动 MoveIt/MTC/守卫，**只提供规划服务，不执行完整搬运**。不与同名服务或已有 MoveIt 叠加。

fixed_v2 正式流程要求：

1. p4 或 p5 导航、geometry、真实附件库存及 ledger、同一高度地图来源；若 p4 还需真实通道文件。使用 `--payload-source-id` 时还须隔离 instance 和 fixed_v2。
2. READY 姿态与全部受管控制器、源身份/版本、资源 journal 均一致；无未决事务。不用 fixture 伪造在线 Hold、ACK 或 EMPTY。
3. 原生 trajectory_executor 使用仿真 commissioning；MoveIt 保持 planning-only；根据 [FixedStationTransfer](../../ws_robot/src/astribot_s1_transport_native/action/FixedStationTransfer.action)提交已核对的箱体、工位、抓放目标、退出候选、导航目标、touch_links、限值与预算。
4. 客户端按原协议续约并持有 Action handle；取消自己的父目标，等待子 UUID 终态、SLAM 停稳、账本及资源处置。未确认释放不能再次提交。
5. 结果须同时具备 TRANSFER_COMPLETE、resources_released、权威 EMPTY/Scene 读回、轨迹交接证据、停稳和完整采集；运动学附着单独标注。

目前产品入口尚未收敛：旧 `transport_task` 拒绝 fixed_v2，UI transport_session 仍走旧入口。不要复制一个裸 `ros2 action send_goal` 后遗漏续约/上下文。

成功记录为 [scene96](../evidence/mainline_20260929/PLACE_ACCEPTANCE.md)，原始[acceptance.json](../../runs/mainline_20260928/workstation_alignment/full_transfer_runner/front_transfer_scene96_transfer_budget_place_3cm_world44_20260928/acceptance.json)及同目录 runtime manifests 保存实际安装身份；[专用运行器](../../runs/mainline_20260928/workstation_alignment/full_transfer_runner/run_full_transfer.py)是本机复现入口，非通用安装命令。必须重核其固定路径、hash、case/domain、模型与现场；旧 run_candidate.bash 指向 scene68，不是 scene96 重放命令。本机 runs 不在普通 Git/真机发布包中。

scene96 为临时 3 cm / 0.1°、指定工位、3.5 倍轨迹时间缩放；严格 2 mm 和接触/力控不在通过范围。FoundationPose、GraspNet 模型闭环、插装和 VLA 不因此获得放行。

## 8. 正常关停与验收产物

先在目标所有者取消导航/路线/搬运，等待终态并用新鲜 `/slam/pose` 确认停稳，再在所属主管前台 Ctrl+C。后台接管需核对 PID 启动身份和子树，不能凭旧 session.json PID 或 latest_sim 发信号。不用全局 pkill、clean_sim_stack.sh 或删 DDS 共享内存。

停稳监测对 Δx/Δy/最短角 Δyaw 有符号累计，检查窗口净偏差；反向抖动抵消，不累加绝对路程。缺样本不算静止；不得在非导航 PICK/PLACE 重新加入位移取消门禁。各模块窗口阈值按源码分别使用，不把一个模块阈值推广为全系统标准。

保留 session.log、session.json、实际 overlay/参数/hash、Action 结果、SLAM 源/时间、payload/资源事件、封口 bag 和视频。latest_sim 只定位最近目录，不证明当前会话。进程退出、资源释放、停稳、任务成功分别记录。

| 现象 | 先查什么 |
|---|---|
| 启动立刻退出 | off+上肢限速冲突、static_map 缺实测初值、fixed_v2 不是 p4/p5 |
| 图可见但不收数据 | 真实 DDS 环境、QoS、推进中的 clock、TF 和采集时间 |
| fixed_v2 不放行 | 库存来源、ledger/geometry、正式 ArmHold、五方 ACK、版本与期限 |
| 工位失败 | 原因是否允许交接、分层图源/坐标、实际包络与停稳，不先改目标或缩小几何 |
| 新代码无效果 | 实际 package prefix、库/hash/overlay，不覆盖运行中的安装 |
| bag/录像不完整 | writer 返回码、metadata、终态覆盖；不将部分采集记为验收通过 |
