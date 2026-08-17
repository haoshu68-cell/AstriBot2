# Astribot S1 仓储仿真 —— 故障排查清单

覆盖任务书要求的 7 类异常场景，按"现象 → 原因 → 排查/修复命令"给出。

## 1. 货架/纸箱/托盘模型不显示、贴图丢失

**现象**：Gazebo 里仓储场景整体是空的，或者货架变成了灰色/纯色方块。

**原因**：`aws_robomaker_small_warehouse_world` 官方 README 只给了 Gazebo Classic 的
`GAZEBO_MODEL_PATH`，没配新版 Gazebo(Ignition/Gz Sim) 的资源搜索路径
`GZ_SIM_RESOURCE_PATH`/`IGN_GAZEBO_RESOURCE_PATH`。

**修复**：
- `warehouse_sim.launch.py` 内部已经用 `SetEnvironmentVariable` 自动设置了这两个变量，正常不需要手动处理；
- 如果你是手动 `gz sim xxx.world` 单独调试，需要先执行：
  ```bash
  source ws_robot/install/setup.bash
  source ws_robot/src/astribot_s1_gazebo_bringup/scripts/setup_gazebo_env.sh
  ```
- 排查命令：`echo $GZ_SIM_RESOURCE_PATH`，确认里面包含
  `.../install/aws_robomaker_small_warehouse_world/share/aws_robomaker_small_warehouse_world/models`。

## 2. 机器人生成后下坠 / 穿透地面

**现象**：机器人出生瞬间往下掉，或者轮子陷进地板里不动。

**原因**：
- 原始 `astribot_whole_body_with_wheel.urdf` 里 4 个轮子完全没有 `<collision>`（已在
  `astribot_s1_torso_wheel.xacro` 里补上圆柱碰撞体，见
  `astribot_s1_description/config/collision_overrides.yaml`）；
- 出生高度 `spawn_z` 不够，轮子还没接触地面就已经"贯穿"进地板碰撞体。

**排查**：
```bash
ros2 topic echo /tf --once   # 看 astribot_torso_base 的 z 是否在持续下降
```
**修复**：确认 `spawn_z >= 0.095`（默认 `0.10`），或者
`ros2 launch astribot_s1_gazebo_bringup warehouse_sim.launch.py spawn_z:=0.15` 加大余量。

## 3. 机械臂/头部/躯干关节无法受控

**现象**：`ros2 control list_controllers` 里控制器是 `inactive`/`unconfigured`，
或者发轨迹指令关节不动。

**排查**：
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```
**常见原因与修复**：
- `gz_ros2_control` 插件没装：`sudo apt install ros-humble-gz-ros2-control`；
- 控制器配置文件路径没传对：检查 `xacro astribot_s1.xacro controllers_config:=...` 输出的
  `<parameters>` 标签内容是不是一个真实存在的绝对路径；
- controller_manager 还没起来就 spawn 控制器：launch 文件里已经用
  `RegisterEventHandler(OnProcessExit(...))` 等待 `spawn_robot` 完成后再拉起控制器 spawner，
  正常不会遇到这个问题；如果手动分步启动，请确保顺序正确。

## 4. TF 树断裂 / robot_state_publisher 报错

**排查**：
```bash
ros2 run tf2_tools view_frames         # 生成 frames.pdf，检查是否有孤立分支
ros2 run robot_state_publisher robot_state_publisher --ros-args --log-level debug
```
**常见原因**：
- xacro 语法错误（如括号未闭合、`${}` 引用了不存在的属性）——用
  `xacro astribot_s1.xacro | check_urdf /dev/stdin` 先本地校验一遍；
- `use_sim_time` 没有对齐（部分节点用了系统时间、部分用了仿真时间导致 TF 插值失败）——
  确认 `warehouse_sim.launch.py` 里所有相关 Node 的 `use_sim_time` 都是同一个值；
- `/clock` 没有桥接——确认 `ros_gz_bridge` 里包含
  `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` 这一条。

## 5. Gazebo 启动卡顿 / 资源加载超时

**修复**：
- 用 `world_name:=no_roof_small_warehouse` 换成无屋顶版本世界，渲染负担更小；
- 首次启动时 Gazebo 需要联网获取部分 fuel 资源，可能较慢，属正常现象，
  确认没有网络代理/防火墙拦截 `fuel.gazebosim.org`；
- 通过 `use_rviz:=false` 关掉 RViz2，减少同时渲染的窗口数。

## 6. spawn 机器人失败 / 模型名称冲突

**现象**：`ros_gz_sim create` 报 "already exists" 或生成失败。

**修复**：
- 已在 `spawn_robot` 节点里加了 `-allow_renaming true`，一般会自动改名而不是直接失败；
- 也可以显式指定：
  ```bash
  ros2 launch astribot_s1_gazebo_bringup warehouse_sim.launch.py robot_name:=astribot_s1_2
  ```
  注意：`robot_name` 同时决定了 `<robot name="">`、Gazebo 里的实体名和传感器话题前缀
  `/model/<robot_name>/...`，改了 `robot_name` 后 `ros_gz_bridge` 会自动桥接到新前缀下的话题，无需手改。

## 7. 终端提示"包找不到"（Package 'xxx' not found）

**修复**：确认按顺序完整执行过一次编译与 source：
```bash
cd astribot_sdk_ros2/ws_robot
rosdep install --from-paths src --ignore-src -r -y      # 见顶层 README 的依赖清单
colcon build --symlink-install
source install/setup.bash
```
每次新开终端都要重新 `source install/setup.bash`（或写进 `~/.bashrc`）。
