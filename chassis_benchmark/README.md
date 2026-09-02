# chassis_benchmark —— 底盘性能测试代码

判据与实验设计见 [`docs/chassis_performance_test_plan.md`](../docs/chassis_performance_test_plan.md)。
本目录只放**可执行的测试代码**，不重复方案里的论证。

刻意放在 `ws_robot/src/` 之外：这不是要 colcon 构建的 ROS 包，是一组独立脚本。

---

## 已核实的管线事实（写代码的前提，别凭直觉改）

这几条都是读代码核出来的，每条带出处。搞错任何一条，A/B 两组的数字全是错的。

### 1. 仿真里 `/cmd_vel` 是**车体系**

`VelocityControl`/`MecanumDrive` 已整体移除
（[`warehouse_sim.launch.py:363-364,420-424`](../ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py#L363-L364)），
底盘完全靠 `omni_effort_drive_node` 的全向轮逆解驱动，它吃的是车体系 `(vx, vy, wz)`。

`cmd_vel_body_to_world_node` 的 `enable_body_to_world` **默认 False**
（[`cmd_vel_body_to_world_node.py:53`](../ws_robot/src/astribot_s1_navigation/astribot_s1_navigation/cmd_vel_body_to_world_node.py#L53)），
该节点已退化为「直通转发 + 姿态安全监控」。

> 仓库里 `README_NAVIGATION.md` §4.2 仍写着"必须做 body→world 转换"，**那段文字是旧的**，
> 代码是对的。别照那段去改。

### 2. 仿真里 `/odom` **是真值**；真机里不是

Gazebo 的 `/odom` 来自 `OdometryPublisher` 插件，而它**基于模型真实位姿**计算、
与底盘运动学解耦（[`astribot_s1.gazebo.xacro:62-64`](../ws_robot/src/astribot_s1_description/urdf/astribot_s1.gazebo.xacro#L62-L64)），
`odom_publish_frequency` 50 Hz。所以打滑时 `/odom` 会**跟着真实位姿一起不动**，
不会像轮式里程计那样跟指令一起错。

真机上完全相反：厂商栈不发布任何 TF，SDK 的 `get_current_joints_position([chassis])`
是底盘驱动自己的航迹推算 —— **那不是真值**，真机 A 组必须靠地面贴标 + 卷尺。

`truth.py` 用两个不同的类把这个区别写死，不给"顺手用 odom"的机会。

### 3. 指令必须持续发流

* 仿真：`omni_effort_drive_node` 的 `cmd_vel_timeout_sec: 0.5`，发一条不会持续运动。
* 真机：`set_joints_position` 发一次抓不住正在运动的关节（实测漂 0.045~0.37 rad）。

`driver.py` 的两个实现都强制以固定频率重发。

### 4. 开环测试必须关掉 nav2

`arm_chassis_speed_coupling_node` 也发 `/cmd_vel`，与测试脚本抢同一个话题。
A/B/F 组一律在**只起 `warehouse_sim.launch.py`**（`enable_effort_drive:=true`）的环境下跑。
`session.py` 的预检会主动查这件事并拒绝启动。

### 5. 姿态安全监控会静默吞掉指令

`cmd_vel_body_to_world_node` 在 z/roll/pitch 越界时会**停止转发并持续发零速**
（`normal_height 0.134`、`max_height_deviation 0.06`、`max_tilt_rad 0.12`）。
若它在链路里而测试"没反应"，先查它是不是 tripped —— 不然会误判成底盘不动。

### 6. 现成的诊断话题

`omni_effort_drive_node` 已经在发：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/wheel_effort/<wheel>` | `Float64` | F2 净偏航力矩、F3 饱和计数 |
| `/wheel_velocity_setpoint/<wheel>` | `Float64` | 轮级跟踪误差 |
| `/wheel_effort_controller/commands` | `Float64MultiArray` | 下发给 ros2_control 的力矩 |

---

## 目录

```
bench/                共享库。纯函数部分可离线单测，不需要机器人
  kinematics.py       全向轮系数矩阵与由它导出的量（纯函数）
  stats.py            N/均值/σ/p95 汇总与判据比对（纯函数）
  truth.py            真值源抽象：仿真 /odom / 真机人工录入
  driver.py           底盘指令抽象：仿真 /cmd_vel / 真机 SDK 位置积分
  session.py          预检 + 运行上下文（域、仿真时钟、话题抢占、进程卫生）
  recorder.py         rosbag2 录制封装

f_health/             F 组：一票否决，每轮先跑
a_kinematics/         A 组：开环运动学精度
b_dynamics/           B 组：动态响应
c_interface/          C 组：指令接口保真度（真机）
d_load/               D 组：负载与臂-底盘耦合
e_nav/                E 组：闭环导航

tests/                bench 里纯函数的离线单测（pytest，不需要机器人）
results/              运行产物：<test_id>_<timestamp>.{json,md}
```

## 跑法

```bash
# 离线单测（不需要机器人，先跑这个确认库本身没坏）
cd chassis_benchmark && python3 -m pytest tests/ -q

# 仿真：先只起仿真，不要起 nav2
ros2 launch astribot_s1_gazebo_bringup warehouse_sim.launch.py enable_effort_drive:=true

# F 组（一票否决，先跑）
python3 f_health/f1_control_loop_rate.py --env sim
python3 f_health/f2_static_yaw_torque.py --env sim
python3 f_health/f3_effort_saturation.py --env sim

# A 组：先做 45° 各向同性
python3 a_kinematics/a5_isotropy.py --env sim --n 10
```

每个脚本都写 `results/<test_id>_<timestamp>.json`（原始样本）和同名 `.md`（汇总表）。
**json 里保留每一次的原始值**，不只是统计量——事后想换判据不用重跑。
