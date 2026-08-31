# 实机 SLAM → nav 数据链路验证记录

**日期** 2026-08-31 · 机器人 `orin`(`aarch64`)
**范围** 雷达 → Voxel-SLAM → 栅格投影 → TF → nav2 规划,全程**不发速度**
**结论** 生产拓扑全链打通(D1 解决后)

---

## 0 · 生产拓扑长什么样

```
两台 Livox MID-360 (192.168.0.12/.13)
    │ /livox/lidar_front  CustomMsg 10.0Hz
    ▼
Voxel-SLAM (LIO + 回环 + GBA)
    │ /map_scan_filtered  PointCloud2 9.995Hz   （已按 0.05~1.63m 高度过滤）
    │ TF camera_init → aft_mapped
    ▼
cloud_to_grid_node                          map_odom_tf_node
    │ /map  OccupancyGrid                       │ TF map→odom（REP-105 分解）
    │  frame_id = camera_init                    │ TF map→camera_init（恒等）
    ▼                                            ▲
nav2 planner_server                              │ 需要 odom→base
    │ global_costmap（StaticLayer 吃 /map）        │
    │ ComputePathToPose → /plan                  │
                                          chassis_odom_node（只读）
                                              │ /odom 50Hz
                                              │ TF odom→astribot_torso_base

另一条只读支线（机器人模型用）：
  state_bridge_node（只读）→ /joint_states 50Hz → robot_state_publisher → 各连杆 TF
```

**本机器人没有 `base_link`**,根 frame 是 `astribot_torso_base`。

---

## 1 · 实测数据(生产拓扑,假桥已全部撤除)

| 话题 | pub | sub | 实测频率 |
|---|---|---|---|
| `/livox/lidar_front`(CustomMsg) | 1 | 1 | **10.002 Hz** |
| `/livox/imu_front` | 1 | 1 | **199.846 Hz** |
| `/map_scan_filtered` | 1 | 1 | **9.995 Hz** |
| `/map` | 1 | 2 | 0.476 Hz(节点主动限流) |
| `/odom` | 1 | 0 | **49.964 Hz** |
| `/joint_states` | 1 | 1 | **50.135 Hz** |
| `/robot_description` | 1 | 0 | latched |
| `/global_costmap/costmap` | 1 | 1 | 0.667 Hz |
| `/tf` | **4** | 5 | **92.060 Hz** |
| `/tf_static` | **1** | 9 | latched |

`cloud_to_grid` 的投影统计:

```
投影完成 236x168 占据=275 空闲=7188 未知=32185 用点=1681 切片外=0 耗时=0.03s
收=200 投影=10 限流跳过=190 发布=10 雕刻失败=0
```

`0.476 Hz` **不是故障** —— 节点自己限流(收 200 帧投影 10 帧),`雕刻失败=0`。

`map_odom_tf_node`:

```
已发 map→camera_init 恒等变换。含义：开机点即地图原点（camera_init 的原点是底盘启动位姿）
map→odom=(-0.000, 0.001, -0.001) 更新=474 跳变=0(最大 0.000m) 倾角超限=0
```

**`/tf_static` 发布者只有 1 个(`robot_state_publisher`)** —— 这是"不再依赖临时假桥"的判据,见 §4。

nav2 规划:`ComputePathToPose` 目标 (1.5, 0.0) → **`SUCCEEDED`**,规划耗时 **0.95 ms**,
`planner_server` 状态 `active [3]`,global_costmap 的 StaticLayer 每 2 秒跟着 SLAM
地图重设尺寸(`Resizing costmap to 227 X 168 at 0.050000 m/pix`)—— 证明它**真的吃到了我们的 `/map`**。

---

## 2 · ⚠️ 反直觉:两个只读桥可以共存,只需回答一次会话提示

**这是本次最容易再踩的一个点。**

`chassis_odom_node`(出 `/odom`)和 `state_bridge_node`(出 `/joint_states`)
**各自建一个厂商 SDK 会话**。第二个启动时会卡住并打印:

```
[WARN] Another user currently controls the robot. Acquiring control will immediately
stop the robot's current motion. Please ensure the robot is stationary and in a safe
state before proceeding.
Enter 'yes' to forcibly acquire control, or press 'Enter' to continue without control rights:
```

进程状态是 `Sl` / wchan `pipe_read` —— **它在等 stdin,不是死锁、不是崩溃**。
`/joint_states` 此时会显示 `pub=1` 但**测不到频率** ——
话题注册了、没有数据,是个"假可用"。

### 怎么处理

1. **先确认机器人静止**(提示自己要求的前置条件):
   ```bash
   grep -oP "帧=.*" /tmp/P_odom.log | tail -3     # 行程应 ≈ 0
   ros2 topic echo /odom --once | grep -A8 twist   # 线速度应 ≈ 0
   ```
   实测值:34500 帧累计行程 **0.008 m**,线速度 ~0。

2. 回答提示。`nohup`/`ros2 launch` 起的进程 stdin 是管道,直接写它的 fd:
   ```bash
   echo yes > /proc/<pid>/fd/0
   ```

3. 之后 `/joint_states` 立刻到 **50.135 Hz**,且节点自己打印:
   ```
   本节点是**只读**桥接：未申请控制权，不接受轨迹、不下发任何指令。
   ```
   **抢到的是会话,不是控制权。**

> `press 'Enter'`(不取控制权继续)在语义上更贴合只读桥,但本次没有实测过,
> 不能断言它同样可行。下次可以先试 Enter。

### 三个曾被考虑、又被实测作废的方案

| 方案 | 为什么废掉 |
|---|---|
| 停 `chassis_odom_node`,只跑 `state_bridge_node` | **`state_bridge_node` 只有一个 `create_publisher`(`:75`),只发 `/joint_states`,不发 `/odom` 也不发 TF。**停掉前者 = `map→odom` 断 = nav2 代价地图垮掉。拿导航换模型,不可接受。 |
| 用现成的 `bridge_container` | **它装的是写侧桥**:`bridge_container.py:28-29,78,83` 建的是 `ChassisCmdBridgeNode`(消费 `cmd_vel` 下发 SDK)+ `ArmTrajBridgeNode`。两重致命:会让机器人能动;而且里面**没有** StateBridge/ChassisOdom,根本给不了 `/joint_states` 或 `/odom`。 |
| 新写一个只读容器 | 它要解决的"两个只读节点抢一个会话"**实测不是硬冲突**,只是首次需要一次人工确认。为省一次交互新写代码不值得。 |

**教训:三个方案都是在"两者不能共存"这个未验证前提上推出来的。前提一验就全废。**

---

## 3 · D1:厂商本体驱动未启动(已解决)

### 症状

```
astribot_arm_left / arm_right / gripper_left / gripper_right / torso / chassis / head
  ... is not alive                              ← 7 个部件全部
[astribot_client.py:1174] Interface is not alive, timeout.
[astribot_client.py:64]   No simulation or real robot is started.
```

### 判定链条(逐层读源码)

```
Astribot.__init__()                              astribot_client.py:47
  → wait_for_interface_alive(timeout_s=2)                     :1169
      → astribot_interface.is_alive()                         :154
          → 遍历 7 个部件 robot_dict[name].joint_interface_.IsAlive()   ← C++ 层
            任一 False 即整体 False
  → 2 秒超时 → raise RuntimeError("No simulation or real robot is started.")
```

⚠️ `timeout_s=2` 只有 2 秒,慢启动场景下这个超时本身偏紧。

### 为什么不在这台机器上

**Astribot S1 是双机结构**:

| 机器 | 地址 | 角色 |
|---|---|---|
| Jetson Orin | `192.168.0.11` | 感知、SLAM、导航、我们的代码 |
| **x86 伴机** | `192.168.0.10` | **本体运动控制(WBC / 关节接口)** |
| 两台 Livox | `192.168.0.12` / `.13` | MID-360 |

证据:

1. 厂商 `robot_system_ctrl/` 下有**独立的** `x86_robot_env.sh`(以及 `orin_robot_env.sh`、`rk3588_robot_env.sh`)
2. 从 Orin 能看到 `/astribot_system_monitor_x86`、`/storage_x86` 节点
3. **Orin 的 `/opt/astribot_ros/log/` 下没有任何 hardware 日志目录**(只有 camera/lidar/recorder/monitor/led)
4. `192.168.0.10` ping 通

### 一个有用的诊断信号

本体未启动时,厂商 lifecycle 节点是"空壳":

```bash
$ ros2 lifecycle get /astribot_hardware_node
（返回空）
$ ros2 service call /astribot_hardware_node/get_state lifecycle_msgs/srv/GetState
requester: making request: ...
（挂住，永不返回）
```

对比我们自己的节点秒回 `active [3]`。**服务注册了但没有实现在响应** ——
说明 x86 侧进程处于"DDS 已注册、业务逻辑未运行"的状态。

⚠️ 注意:**本体驱动启动后,这两个命令的表现没有变化**(仍返回空/挂住)。
所以它只能作为"未启动"的辅助信号,**不能反过来当"已启动"的判据**。
唯一可靠判据是跑 `chassis_odom_node` 看 SDK 会话能不能建立。

### 解决后

```
[astribot_client.py:49] Interface is alive.
[astribot_client.py:55] Astribot S1 is connected! Robot mode is safe
[chassis_odom] SDK 会话已建立：freq=250.0Hz 模式=safe
  部件=[chassis, torso, arm_left, gripper_left, arm_right, gripper_right, head]
```

从 Orin **没有** ssh 到 x86 的凭据(`Permission denied (publickey,password)`),
所以 x86 侧的启动步骤本文无法记录 —— 需要厂商文档或运维流程补上。

---

## 4 · 临时 LIO-only 假桥(已撤除,仅作记录)

D1 未解决期间,为验证 nav 侧本身是否可用,曾用 3 个**单位** `static_transform_publisher`
假接 TF 链:

```
map → odom → camera_init → aft_mapped(SLAM 发) → astribot_torso_base
```

**这不是生产拓扑。**`map` 与 `odom` 刚性相等 → Voxel-SLAM 的回环修正会**直接窜到 odom 上**。
生产上 `map→odom` 必须由 `map_odom_tf_node` 按 REP-105 分解:

```
map→odom = (map→base)_SLAM ∘ (odom→base)_SDK⁻¹
```

`aft_mapped→astribot_torso_base` 用单位变换的依据:**以 aft_mapped 为准**,
它就是底盘中心(与 `astribot_torso_base` 同一物理点)。

已全部 `pkill -f static_transform_publisher` 撤除。
**判据:`/tf_static` 发布者数从 3 降到 1**(只剩 `robot_state_publisher`)。

---

## 5 · 无速度:实测核对

```
/cmd_vel                 话题不存在
/cmd_vel_nav_body_raw    话题不存在
/cmd_vel_smoothed        话题不存在
```

三层保证:

1. **只起 `planner_server` + `lifecycle_manager`**。刻意不起:
   - `controller_server` —— `navigation.launch.py:134-143`,它 remap `cmd_vel`→`cmd_vel_nav_body_raw`
   - `velocity_smoother`、`cmd_vel_body_to_world_node`、`bt_navigator`、探索协调器
   - ⚠️ 因此 **`/local_costmap/costmap` 发布者是 0** —— 局部代价地图属于 `controller_server`。
     rviz 里那个 Display 是空的。
2. 用 `ComputePathToPose`(纯规划),**不是** `NavigateToPose`
3. ~~厂商本体服务未启动,SDK 拒写~~ ← **D1 解决后这一层失效了**

> ### ⚠️ 安全边界在 D1 解决后变了
>
> | 保护层 | D1 前 | D1 后 |
> |---|---|---|
> | 不启动 cmd_vel 那一端 | ✅ | ✅ |
> | 只用 ComputePathToPose | ✅ | ✅ |
> | SDK 拒绝一切写操作 | ✅ | ❌ **失效** |
>
> 现在唯一挡住运动的是"没有任何进程在发/消费 `cmd_vel`"。
> 起 `controller_server` 会引入 `/cmd_vel` 发布者 —— 需要显式决策。

---

## 6 · 已知缺口:轮子关节不在 `/joint_states` 里

状态桥发 **22** 个关节:

```
astribot_torso_joint_1..4
astribot_arm_left_joint_1..7      astribot_gripper_left_joint_L1
astribot_arm_right_joint_1..7     astribot_gripper_right_joint_L1
astribot_head_joint_1..2
```

**没有任何 wheel 关节** → `wheel_LF_Link` 等 4 个轮子的 TF 不存在。

这是设计使然:底盘在 SDK 侧只暴露 3-DOF `[x, y, theta]`,厂商不给单个轮子转角。
**车体在 rviz 里是完整的,只是轮子不会转。**

夹爪每侧只发主动关节 `joint_L1`,另外 5 个是 URDF mimic 从动关节,
由 `robot_state_publisher` 按 mimic 关系算 —— 这是刻意的,见
`state_bridge_node.py` 头部文档第 2 条。

---

## 7 · 两次判据本身写错(比结论错更危险)

### (a) 用 `ros2 topic list` 判断话题存在 → 假阳性

`/map`、`/map_scan_filtered`、`/plan` 在列表里"都有",实际**发布者全是 0** ——
它们出现在列表里只是因为 **rviz 正订阅着它们**。

**判据必须是 `Publisher count`,不是话题是否出现在列表里。**

```bash
ros2 topic info <topic> | grep -oP "Publisher count: \K[0-9]+"
```

### (b) 探针用了错的 frame 名 → 假阴性

查连杆 TF 时写的是 `astribot_torso_link1`,URDF 里真名是
**`astribot_torso_link_1`**(带下划线)。得到"连杆 TF 一条都没有",
据此差点去排查一个**根本没有问题**的 `robot_state_publisher`。

正确做法:frame 名从 `robot_state_publisher` 的 `got segment <name>` 日志里取,
或从展开后的 URDF 里 grep,**不要凭记忆拼**。

### (c) 把厂商日志当成自己节点的行为 → 错误归因

看到 `acquiring control rights... acquired control rights` 就断言
`chassis_odom_node` 持有控制权。**那是厂商 SDK 内部无条件打的日志。**

真实边界在代码里写死:

```python
# sdk_session.py:292
bot = Astribot(freq=freq, high_control_rights=False, node_name=node_name)
# :271-275  永不申请高控制权……这是物理边界，不是可配置项 —— 所以不提供参数让调用方改
```

**会话独占 ≠ 持有控制权。**两个进程各建 `Astribot()` 会撞会话,与 `high_control_rights` 无关。

---

## 8 · 完整启动顺序

每层一个终端,**顺序不能颠倒**。

```bash
SDK=/home/astribot/Downloads/astribot_sdk_aarch64

# ① 雷达 —— 必须用厂商真入口
source /opt/astribot_ros/robot_system_ctrl/robot_env.sh
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# ② Voxel-SLAM —— 对着厂商 middle_ware 编译，且缺 GTSAM 库路径
source /opt/astribot_ros/robot_system_ctrl/robot_env.sh
export LD_LIBRARY_PATH=/home/astribot/SLAM/ThirdParty/GTSAM/install4.1.0/lib:$LD_LIBRARY_PATH
cd ~/SLAM/vxlm-slam && source install/setup.bash
ros2 launch voxel_slam vxlm_mid360.launch.py      # 包名是 voxel_slam（下划线）

# ③ 栅格投影
source $SDK/env_robot.sh
ros2 run astribot_s1_perception cloud_to_grid_node --ros-args --params-file \
  $SDK/ws_robot/install/astribot_s1_perception/share/astribot_s1_perception/config/cloud_to_grid_params.yaml

# ④ 里程计（只读）—— 必须 cd 到 SDK 根，厂商 proxy 用 cwd 相对路径
source $SDK/env_robot.sh && cd $SDK
ros2 run astribot_trajectory_bridge chassis_odom_node

# ⑤ map→odom 分解
source $SDK/env_robot.sh
ros2 run astribot_s1_perception map_odom_tf_node

# ⑥ 状态桥 + robot_state_publisher（机器人模型）
#    会卡在会话提示 —— 见 §2，确认静止后 echo yes > /proc/<pid>/fd/0
source $SDK/env_robot.sh && cd $SDK
ros2 launch astribot_trajectory_bridge state_bridge.launch.py

# ⑦ nav2 只起规划侧（不要用 navigation.launch.py，它含 controller_server）
source $SDK/env_robot.sh
ros2 run nav2_planner planner_server --ros-args \
  --params-file $SDK/ws_robot/src/astribot_s1_navigation/config/nav2_params_rpp.yaml \
  -p use_sim_time:=false
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p node_names:="[planner_server]" -p autostart:=true -p use_sim_time:=false \
  -r __node:=lifecycle_manager_planonly

# ⑧ 可视化（在 VNC 桌面里）
$SDK/view_chain.sh
```

rviz 里要**手动勾上 `RobotModel`**(配置里默认关闭)。
`Global Costmap` 也默认关闭,勾上能看到代价地图。

---

## 相关文档

- [vnc_visualization_tutorial.md](vnc_visualization_tutorial.md) —— 怎么从 PC 看到这些
- [pc_rviz_visualization.md](pc_rviz_visualization.md) —— 方案 A/B 技术方案
- [chassis_slam_integration_plan.md](chassis_slam_integration_plan.md) —— SLAM 接口对齐设计
