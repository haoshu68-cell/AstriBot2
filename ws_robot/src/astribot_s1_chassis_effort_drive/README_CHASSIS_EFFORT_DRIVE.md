# astribot_s1_chassis_effort_drive

麦克纳姆底盘力矩闭环驱动——治本方案，替代"VelocityControl 本体直控 +
MecanumDrive 轮子仅视觉转动"两套独立运动学并行架构。

## 链路

```
/cmd_vel(不变) → 麦克纳姆逆解 → 四轮目标转速 → 轮速PID+摩擦前馈
  → /wheel_effort_controller/commands(Float64MultiArray)
  → ros2_control effort command_interface → gz_ros2_control GazeboSimSystem
  → 轮子关节(DART物理) → 轮地各向异性摩擦接触力 → 车身真实动力学响应 → /odom
```

对外 `/cmd_vel`、`/odom`、`/joint_states` 接口不变，Nav2/巡游/臂-底盘耦合等
上层节点零改动。

## 必须先标定，不能直接信任默认值

- **逆解符号方案**（`config/mecanum_effort_drive_params.yaml` 里的
  `kinematics_sign_{vx,vy,wz}`）：默认值是按现有 `fdir1` 对角线摩擦分组倒推
  的初始假设，真实符号取决于每个轮关节各不相同的物理安装朝向。跑完单轮测试、
  原地旋转测试才能确认对不对。
- **PID增益/摩擦前馈/力矩限幅**：全部是起始估算值，不是实测数据。
- 详细的分阶段测试步骤、判定指标、风险清单，见项目 plan 文件里"麦轮力控
  重构方案"章节。

## 轮径扫描测试

```bash
ros2 launch astribot_s1_chassis_effort_drive mecanum_effort_drive.launch.py \
  wheel_radius:=0.12
```

只改这一个launch参数不够——SDF 里轮子碰撞体半径、spawn 高度、轮关节 origin
的 z 偏移是几何模型的一部分，必须跟这里的 `wheel_radius` 同步改，否则物理引擎
里轮地实际接触几何跟这里算的运动学半径不一致，扫描结果没有意义。

## 观测话题

- `/wheel_velocity_setpoint/{RF,LF,RR,LR}`：逆解目标转速
- `/wheel_effort/{RF,LF,RR,LR}`：PID+前馈输出力矩
- `/joint_states` 里对应关节的 velocity/effort：真实反馈

推荐用 rqt_plot/PlotJuggler 同时订阅目标转速和真实反馈画跟踪误差曲线。
