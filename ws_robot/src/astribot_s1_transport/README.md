# 移动抓放仿真任务

固定场景：机器人从原点，通过 RGB-D 定位，用左臂从取货台抓取 6 × 6 × 12 cm、配置质量 0.2 kg 的箱体；
收臂并更新导航包络；通过现有导航仲裁器移动到放置工位；放置、松爪、退臂。
取货前头部朝向侧面的箱体；取货台中心为 map 中 (0.10, 0.70, 1.095) m。
基线搬运路线先沿 +X 方向到 0.45 m 的离台点，再导航到 1.10 m 的放置工位；放置台中心 (1.15, 0.72, 1.095) m。两工位采用侧向操作，放置台外移以避开导航预测扫掠，两段导航均使用原有仲裁 action。
右臂进入紧凑屈肘运输姿态并保持，躯干保持当前姿态。所有新入口只用于仿真。

机械臂 PICK/PLACE 使用原生 MoveIt Task Constructor 完整序列规划，再执行现有碰撞、奇异、限位和时间参数化校验；准备姿态与已放置恢复仍使用 `DualArmPlanner`。夹爪开合宽度
使用现有 `GripperCommander` 的几何反解。没有新增底盘速度控制器，也没有绕过
导航仲裁、路径保护、不可达超时或包络门控。

**物体采用运动学附着**：Gazebo 中的箱体有可见模型，MoveIt 中有参与碰撞检测的
碰撞体；抓取时确认夹爪关节到位，再将箱体附着并同步 Gazebo 位姿。箱体不参与
Gazebo 夹爪接触求解；质量只传给载荷接口，未施加刚体惯性。因此本场景不验证
夹持力、摩擦、滑落、力控或带载动力学。

## 图片模块对应

| 模块 | 当前接入/补齐 | 边界 |
|---|---|---|
| 整机任务与资源协调 | 整机互斥租约覆盖底盘、双臂、头部、躯干、夹爪；阶段事务；取消等待底层 action 终态 | 租约约束本任务入口；旧 demo/直接控制器客户端仍需由运行方隔离，未宣称全系统强制抢占 |
| 运输姿态与载荷管理 | 抓取确认→抬升→收臂实测→URDF 碰撞体与物体包络→SetRobotEnvelope→两张 costmap 确认→导航 | 使用仿真运输速度/加速度边界，非实机标定值 |
| 整机模式与能力健康 | ADMISSION/MANIPULATION/TRANSPORT/FAULT/完成阶段；时钟、扫描、里程计、关节、TF、载荷同步及包络检查 | 复用现有导航健康与保护；不增加重复传感器驱动 |
| 操作场景与物体管理 | 同一 object_id；WORLD/ATTACH_PENDING/ATTACHED/RELEASE_PENDING/PLACED 和单调版本；Gazebo/MoveIt 同步（持久 ROS–Gazebo 位姿服务） | 固定工位场景；非通用视觉物体数据库 |
| 正式操作技能 | 可重复调用的具名姿态/笛卡尔规划/夹爪技能；ExecuteTrajectory 与 JTC action 可取消；反馈、失败停止、保载 | 采用左臂拾取；已有双臂闭链规划继续保留，双臂共同抓物单独验收 |
| 标定与定位质量管理 | RGB/深度/CameraInfo 同步、拍摄时刻 TF、配置摘要、连续三帧稳定观测及原点准入；移动后重投影 MoveIt 障碍物 | map 使用仿真真值；相机外参为场景设计值，实机标定和定位精度待验收 |
| 长期运行能力 | 逐事件落盘、原子状态快照、故障保载、拒绝覆盖旧账本、明确恢复边界 | 电池状态/回充、任意阶段自动断点续作和长时间耐久运行仍需独立业务与验收场景 |

## 构建与运行

在已经正常构建过的仓库中，加载 ROS 与工作空间后构建新增接口、操作包和仿真入口。
若本地 install 落后于源码，也要更新感知组件及其 core 依赖。可使用独立 build/install
目录验证，不必覆盖正在被其他任务使用的 install。

```bash
source /opt/ros/humble/setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/install/setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/tools/setup_mtc_humble.sh
cd /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot
colcon build --packages-up-to astribot_s1_transport astribot_s1_navigation \
  --cmake-args -DGTSAM_DIR=/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/deps/gtsam/lib/cmake/GTSAM
source install/setup.bash
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
```

先按仓库 `ros2-stack-ops` 检查栈归属。后续统一使用导航仓库仿真；搬运只往该世界添加工位、物体与任务节点。专用空场景、空地图和独立仿真入口已删除。已有仓库栈时复用它，禁止重复启动。所有终端均加载相同 ROS、工作空间和 MTC 前缀。

```bash
# 终端 1：唯一导航仿真入口；baseline 地图与场景坐标一致
/home/yjh/WorkSpace/astribot_sdk_ros2/tools/launch_sim_stack.sh \
  --mode baseline --navigation-policy p3 --max-linear-speed 0.2 --headless --no-rviz

# 终端 2：确认 /clock、/joint_states 推进以后启动规划服务
ros2 launch astribot_s1_transport transport_skills.launch.py

# 终端 3：复用导航仿真的 RGB-D 数据，添加任务观察器和位姿服务
ros2 launch astribot_s1_transport transport_support.launch.py

# 终端 4：执行任务；output 必须是新的目录
ros2 run astribot_s1_transport transport_task \
  --scenario /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport/config/warehouse_transfer.json \
  --output /tmp/astribot_transport_run1

# 另一个终端请求取消；success 表示接到请求，非已经停止
ros2 service call /transport/cancel std_srvs/srv/Trigger '{}'
```

`/transport/status` 发布当前阶段、物体状态和版本；最终结果以 `state.json` 为准，
完整事件在 `events.jsonl`。CLI 的 SIGINT/SIGTERM 同样请求取消。底层 action 超时会
发送取消并等待终态，不能把取消受理当成资源已经归还。

任务结束确认空载包络并保持底盘 HOLD。执行过程中出现错误时，不自动松爪、不删物体、不把
RELEASE_PENDING 伪装成 PLACED。不要通过删除账本后重试解决状态冲突。

## 接口与控制归属

| 接口 | 本模块职责 |
|---|---|
| `/transport/plan_manipulation` (`PlanManipulation` action) | MTC 只规划；返回完整有序阶段、预计起始状态和 context_id；支持取消规划 |
| `/transport/plan_skill` (`astribot_transport_msgs/srv/PlanSkill`) | 规划具名姿态、TCP 位姿、头部关节目标或夹爪宽度；返回经过校验的轨迹，不直接执行 |
| `/execute_trajectory`、夹爪及头部 `follow_joint_trajectory` | 复用已有执行器；任务层等待结果和关节实测收敛 |
| `/navigation/set_robot_envelope`、`/navigation/robot_envelope` | 提交运输姿态/载荷边界，等待同一 epoch 的 ready 确认 |
| `/navigate_to_pose` | 复用整机已有导航仲裁器；任务层不发布底盘速度 |
| `/transport/cancel`、`/transport/status` | 请求取消、阶段反馈；执行器终态和账本共同决定是否停止 |
| `/transport/execution_guard/set`、`/transport/execution_guard/status` | C++ 执行守卫：独立上下文编号、关节期望/实测误差、底盘位移、源时效与故障锁存；fixed_v2 任务消费状态后通过现有 action 取消，不另发底盘速度 |
| `/transport/object_observation`、`/transport/camera_health` | 视觉观测、帧时效和标定版本；抓取准入消费观测 |

`transport_compact` 是此仿真技能服务的关节目标：两臂肩部外展归零，第三关节左右
分别为 −0.6/+0.6 rad，肘部保持 1.0 rad，其他关节为零。该目标仍经过现有规划器
的碰撞和奇异点校验。导航包络从执行后的实测姿态重新计算，不使用固定缩小的半径。

fixed_v2 的 MTC 机械臂阶段先开启原生 `execution_guard`，收到同一上下文的健康状态后才执行。
守卫检查 300 ms 内的关节/TF 源证据，最大关节跟踪误差 0.05 rad、底盘位置/转角偏移
0.02 m / 0.02 rad；数据缺失、越界或时钟回退会锁存故障，恢复正常测量不会自动放行。
任务仍负责取消确认与保载。守卫是运行中偏离检测，不证明任意机械臂轨迹的连续碰撞安全或动力学稳定。
新运行时计算优先 C++；Python 保留现有任务编排接口和验证脚本。

`transport_skills.launch.py` 提供 `mtc_velocity_scaling`、`mtc_acceleration_scaling`，默认均为 0.1。
较低比例可用于隔离仿真的跟踪回归，不能通过提高守卫误差门限补偿执行偏离。每轮记录实际参数与二进制版本。

## 恢复与重试

- WORLD：检查是否已生成工位/物体、是否仍有 action 在执行，再决定清理本任务添加的对象；不重建另一套仿真。
- ATTACH_PENDING：附着请求结果可能不确定；核对 MoveIt attached object、Gazebo 位姿和夹爪关节。
- ATTACHED：保载停止；检查最后的 TCP→物体变换以及导航失败原因。禁止自动释放。
- RELEASE_PENDING：夹爪可能已经张开；检查世界物体与 attached object 两侧，禁止盲目再夹取。
- PLACED 但任务失败：物体已经释放，可能是退臂或放置观测验收失败，不能重放抓取序列。

提供仅针对失败/取消且物体已确认 PLACED 的恢复入口：

```bash
ros2 run astribot_s1_transport transport_task \
  --scenario /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport/config/warehouse_transfer.json \
  --output /tmp/astribot_transport_run1 --resume-placed
```

恢复先核对账本与事件末条一致、底盘到站且 HOLD、没有附着物、工位匹配、夹爪实测张开、
MoveIt 与 Gazebo 放置位置正确，再仅执行向内 8 cm/抬升 3 cm 的退臂、收臂、空载确认和放置验收。
原事件保留并追加恢复过程；不重新抓取、导航、松爪或解除附着。轨迹继续执行碰撞与奇异点检查。
其他不确定物理状态仍需人工核对，不自动继续。重置仅限本任务拥有的 Gazebo/MoveIt
进程与对象；真机不能使用此恢复方式。

## 摄像头数据接入

仿真使用 Gazebo 原生 `rgbd_camera`，参数在描述包的 `camera_rgbd_transport.yaml`。
640 × 480、目标 15 Hz、水平视场 1.5 rad、深度范围 0.08–5 m。安装位姿是为本场景
设计的虚拟值，不代表真机相机型号或外参。导航仓库默认采用同一虚拟 RGB-D 配置；真机配置仍单独审计。

`camera_observer` 接收 `/camera/color/image_raw`、`/camera/depth/image_raw` 和
`/camera/color/camera_info`，等待同一拍摄时间的 TF，把有效深度投影到 map。
本场景在配置取货区域内用橙色模板识别单个箱体，忽略区域外仓库标线，拒绝区域内多目标、无效深度、未对齐图像和过期数据。
`/transport/object_observation` 输出中心、尺寸、时戳和标定配置摘要；
`/transport/camera_health` 输出观测状态。任务必须拿到三帧稳定观测才允许抓取，
不会在视觉失败时回退到物体生成坐标。该检测器仅用于受控仿真，不是通用真机识别器。
相机用于抓取定位；导航避障仍复用现有激光感知和保护，未宣称已经完成视觉障碍融合。

机器人开机后可在机器人 ROS 环境运行仓库 `tools/robot/camera_audit.py --help` 所示
只读审计，采集设备、图像元数据、CameraInfo 和 TF。CameraInfo 是内参；外参必须
结合安装 frame/TF 和标定来源确认。确认实际型号后再选择厂商驱动及标定配置。

## 验收

```bash
PYTHONPATH=/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport \
  python3 -m unittest discover \
  -s /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_transport/test -v
```

离线测试验证事务、几何和 RGB-D 算法，不等于 Gazebo 成功。仿真成功必须同时满足：

1. 机械臂及夹爪 action 成功，多个不同时间戳的关节样本到位且静止。
   抓取点来自当前 RGB-D 三帧稳定观测。
2. 物体附着确认，搬运姿态稳定，包络 epoch 已确认。
3. 现有 NavigateToPose 仲裁入口返回成功，底盘实测到站。
4. 放置位姿在释放前满足容差，MoveIt 移除附着并保留世界物体。
5. Gazebo 连续两次观测物体位姿稳定，放置误差不超过配置的 2.5 cm。

新仓库/MTC 验证记录见 `docs/MTC_NAVIGATION_SIMULATION_20260919.md`。旧 `docs/TRANSPORT_SIMULATION_20260919.md` 仅为已退役空场景证据。

固定非 home 姿态的集成验证使用同一导航仓库，将栈和任务的
`navigation_geometry_mode` 同时设为 `fixed_v2`，导航策略使用 P4/P5。
该模式要求实测关节几何、保持控制权、附着过滤版本和六方消费者确认；
操作前先撤销导航，导航期间机械臂保持固定，不能通过 VLA 直接绕过。
默认仍为 `legacy`。施工、复现入口与每次成功/失败记录见
`docs/NONHOME_NAVIGATION_IMPLEMENTATION_20260919.md`；带载重复运行和完整通道矩阵尚未验收通过。

SLAM 默认 Release 构建已修正，示例使用导航环境默认的 1 倍仿真步进。旧的 0.05 倍联调记录不能用于实时性能验收；墙钟超时仍保留调试余量（机械臂 240 秒、每段导航 1200 秒），不代表真机参数。后续只在这一导航环境中验证，不修改传感器时间戳或时效门槛。

## 规划与执行边界

VLA 扩展已接入 PICK_PLAN / PLACE_PLAN：通过 `--vla-config` 选择本地示例或 HTTP 策略。
末端目标建议经过上下文和范围校验后进入 MTC；末端增量／关节动作块仅支持影子评估。
算法输入、动作语义、适配器示例、只读探针及运行命令见
[`VLA_MANIPULATION_INTERFACE_20260919.md`](../../../docs/VLA_MANIPULATION_INTERFACE_20260919.md)。
算法不接管夹爪、导航、attach/detach 或保载事务。省略配置保持原流程。

`FixedState → SerialContainer(PICK)` 在当前固定底盘快照中规划；PLACE 使用 `GeneratePose + ComputeIK` 搜索最多 32 个预放置逆解，通过 `Connect` 与当前姿态连接，再规划接近、释放和退臂。退让候选使用 `Alternatives`。PICK 和 PLACE 分别在各自工位规划，不缓存跨导航位移的关节轨迹。

放置序列含接近、张爪、预测解除附着、退让、收臂，整段通过校验才开始执行。预测场景变更仅发生在 MTC 内部；实际 attach/detach 仍等待 MoveIt/Gazebo 确认并落盘。每段执行前检查 HOLD、底盘偏移、关节起始状态、场景摘要、标定版本、附着状态和计划有效期，不接受部分序列或隐式规划回退。

仿真 MTC 默认速度和加速度缩放均为 0.1，规划与最终时间参数化保持一致，可通过规划节点的 `max_velocity_scaling` / `max_acceleration_scaling` 配置。规划前要求至少 0.5 秒的不同时间戳底盘样本，位移不超过 1 mm、姿态变化不超过 0.003 rad；执行前原有 2 cm / 0.02 rad 底盘变化保护仍保留。这些是当前移动底盘固定工位操作的仿真条件，不是实机标定值。

`pick_mtc_plan.json` / `place_mtc_plan.json` 保存计划。完整验收以新仓库场景报告为准；旧空场景视频和 task20 日志仅保留为历史证据，不能证明此次 MTC/仓库验收。
