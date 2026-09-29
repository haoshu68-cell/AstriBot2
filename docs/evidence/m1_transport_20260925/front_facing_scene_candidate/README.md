# 正对操作台的搬运场景候选

状态：离线坐标与数值运动学检查通过；ROS、碰撞规划、完整搬运及录像尚未运行。此目录归档场景参数和操作脚本，不新增 C++ 算法实现或兼容入口。

可运行候选：`/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/front_facing_transfer_candidate_20260925`。
来源：scene39 实际归档的 `used_*` 文件，逐文件 SHA256 见 `manifest.json`。旧场次保留为实验来源，不作为新候选的运行分支。

## 场景数值

| 项目 | PICK | PLACE |
| --- | --- | --- |
| 操作台中心 world XYZ，m | (0.10, 0.70, 0.5175) | (1.15, 0.72, 0.5175) |
| 机器人停靠 world XY/yaw，m/rad | (0.10, 0.15, π/2) | (1.15, 0.17, π/2) |
| 物体中心 world XYZ，m | (0.10, 0.70, 1.095) | (1.15, 0.72, 1.095) |
| 操作面向外法向 world | (0, −1, 0) | (0, −1, 0) |
| 物体相对底盘前向/侧向距离，m | 0.55 / 0 | 0.55 / 0 |

操作台尺寸仍为 0.10 × 0.10 × 1.035 m；物体尺寸仍为 0.06 × 0.06 × 0.12 m，质量 0.2 kg。机器人前方 +X 对着操作面。底盘中心到台前沿 0.50 m，按底盘半长 0.31 m 计算的名义间隙为 0.19 m；这一数值不能证明全身、载荷或运动路径无碰撞。

头部初值为 yaw 0 / pitch 0.95 rad。双臂、躯干、夹爪 READY 初值不变；相机 URDF 外参、传感器配置不变。抓取姿态 world XYZW 仍为 (0.5, 0.5, 0.5, 0.5)，TCP 的 pregrasp/grasp/lift 高度分别为 1.225/1.195/1.255 m。保持 idle_position_hold=true、idle_position_kp=3.0；不修改安全预算或停止、放置门槛。

## 修改范围与主线接入

1. `scenario.json` 直接采用 `spawn_world_xyyaw` 和 `nav_goal_world`；移除旧 `nav_goal`、`nav_waypoints`，没有兼容分支。
2. `run_full_transfer.py` 将 spawn XY/yaw 传给实际启动器，归档本轮头部初值 YAML。主线应传入新 ABI 的实际 overlay 与二进制哈希。
3. `prepare_transfer_goal.py` 使用本场次同时间戳实测的 map_from_world 转换世界系停靠目标，PLACE 撤离方向在 world 内计算。实测初始世界位姿与配置不符时拒绝，门槛 0.002 m / 0.1°。
4. `sim_initial_transport_ready.yaml` 必须由主线放入独立 description 测试 overlay 的同名 profile；仅写 JSON 的 head_pick_joints 不会使头部运动。Gazebo 与 MoveIt 必须解析同一 profile。`prepare_scene.py` 校验实际解析的头部 profile 与候选一致，未接入则报 FRONT_HEAD_PROFILE_NOT_INSTALLED；原关节实测检查仍执行。
5. 地图准备由主线独占接入：在 `run('prepare_transfer_goal', ...)` 成功后、`m5_observer` 启动前加入 `run('prepare_height_map', ['python3', W/'runs/layered_stack_20260925/prepare_height_map.py', '--case', b, '--session', isolation.instance], 45)`。本交付不放置占位 hook。地图装载后由父验证重新取得 ready/initial_stop/scene EMPTY 证据，不能续用之前的停止证据。

既有 verify_full_transfer、verify_full_pick、read_full_executor_parameters、观察话题、QoS、技能 launch、录像脚本共 7 项保持来源字节不变。主线接手修改 runner 后须重新归档实际 used_* 与哈希；本目录记录的是接手前冻结版本。

## 离线验证

复现：`python3 /home/yjh/WorkSpace/astribot_sdk_ros2/docs/evidence/m1_transport_20260925/front_facing_scene_candidate/validate_front_facing_scene.py`。

- 两站面向关系、物体高度、名义间隙通过。
- 实际候选的导航目标转换代码在单位变换、旋转加平移的 map 注册下回转误差均为 0。
- 初始 0.70 m 候选在单种子数值 IK 中位置残差 0.1242 m，未采用；不据此声称全局不可达。
- 最终 0.55 m 的 pregrasp/grasp/lift 数值 IK 均通过：最大位置残差 4.16e−12 m，最大姿态残差 9.28e−13 rad，最小关节限位余量 0.1323 rad，高于既有 0.1 rad 要求。这是方程求解残差，不是机器人实测精度。
- 原 URDF 光学链预测目标中心约在像素 (318.25, 179.70)，与光轴夹角 0.001073 rad；不包含遮挡、成像或识别验收。

离线高度引用 scene39 的 base world Z=0.1292237616 m；新场次必须重新测量。数值 IK 没有检查碰撞、路径连续性或实际规划器收敛，不能代替 MTC 验证。

## 主线运行验收

- 核实实际消息 ABI、可执行文件、profile、相机基线与场次哈希；实测 idle 为 true/3.0，头部及其余关节符合初值检查。
- 在 PICK、PLACE 分别保存实际 world 底盘位姿及操作面方向；位置误差 ≤0.002 m、yaw 误差 ≤0.1°，并在实际图像中核实操作区域可见。
- ThreePhase 起终点姿态对齐使用有效分层几何和本场次地图；缺失、陈旧、身份不匹配必须拒绝，无二维回退。搜索与 MPPI 沿用既有契约。
- 实际 MTC 的 IK、碰撞、关节限位、轨迹检查全部通过，随后核实 PICK、挂载账本、LIFT、运输姿态、ArmHold/包络确认、NAV、PLACE、脱离账本与 PlanningScene 独立读回、最终停止和资源释放。
- 放置实测误差仍须 ≤0.025 m。保存全程日志、失败原因和视频；阶段失败即保留失败证据，不降低阈值换取通过。

scene39 只证明已记录的 PICK/运输姿态阶段，不证明 NAV 或 PLACE。此候选没有启动 ROS、Gazebo 或硬件；主线统一运行单一场次并管理 Git 提交。
