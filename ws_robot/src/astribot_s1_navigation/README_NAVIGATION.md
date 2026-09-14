# Astribot S1 导航

## 项目链路

`astribot_s1_perception` 提供雷达、地图和定位；`astribot_s1_autonomy` 提供扫描切片、探索目标
及调度；本包负责 Nav2 配置、行为树、速度链路和 RViz。
`astribot_s1_path_tracking` 提供精确终点规划、路径跟踪和到位精调。
底盘命令由仿真驱动或 `astribot_trajectory_bridge` 执行。

```text
导航目标 → ExactGoalPlanner（Smac）→ ArrivalController（MPPI / RPP）
        → 终点精调与停稳 → 速度平滑/车体系转换 → 底盘
```

## 入口

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/setup.bash
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim launch_gazebo:=true headless:=false use_rviz:=true \
  controller_plugin:=mppi exploration:=false
```

也可运行 `tools/launch_sim_stack.sh`。该脚本只负责启动；不自动清理进程、重启、修复生命周期
或下发测试目标。启动前应明确停止已有仿真，避免重复的时钟和速度发布者。

| 参数 | 用途 |
|---|---|
| `env` | `sim` 或 `hardware` |
| `launch_navigation` | 是否随总入口启动 Nav2，默认 `true`；设为 `false` 可独立启动导航 |
| `controller_plugin` | `mppi`（全向）或 `rpp` |
| `use_rviz` / `headless` | RViz 和 Gazebo GUI |
| `max_linear_speed` | 跟踪线速度上限 |
| `exploration` | 是否启动自动探索协调器 |
| `scan_source` | `slice_scan` 或 `laserscan` |
| `map_source` / `localization` | 地图提供方式与定位来源 |

地图和定位默认值来自 perception 的 `config/map_source.yaml`。仿真静态地图与真值定位可显式指定：

```bash
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim controller_plugin:=mppi exploration:=false \
  map_source:=real_file localization:=ground_truth map_yaml_path:=/absolute/path/map.yaml
```

## 文件与接口

- `config/nav2_params_mppi.yaml`、`nav2_params_rpp.yaml`：完整节点参数。
- `launch/nav2_full_bringup.launch.py`：仿真/感知/Nav2/RViz 总入口。
- `launch/navigation.launch.py`：Nav2 生命周期节点及速度链路。
- `behavior_trees/`：周期规划、FollowPath 和有限恢复行为。
- `rviz/nav2_view.rviz`：地图、足迹、全局路径与局部轨迹。
- `astribot_s1_navigation/`：速度坐标转换、机械臂展开限速及运行诊断。

业务通常调用 `/navigate_to_pose`，或在 RViz 使用 Nav2 Goal。
直接调用 `/follow_path` 时，控制器选 `FollowPath`，检查器选 `precise_goal_checker`。
基座坐标系是 `astribot_torso_base`，不是 `base_link`。
速度输出按车体系解释，并经过既有速度平滑与底盘桥接；不要混用 world 系命令。

## 到位精度与异常

位置欧氏误差 ≤ 3 cm、航向误差 ≤ 1.5°，且连续停稳 0.6 秒才成功。
支持导航 TF、SLAM、视觉和 Mark 基座位姿；定位失效、碰撞受阻、无进展或超时明确失败。
详见 [path_tracking 代码及参数说明](../astribot_s1_path_tracking/README.md)。

仿真检查应依次确认 Gazebo 步进、时钟前进、TF 新鲜、Nav2 active，再发目标。
验收同时记录 action 结果和实际停车误差，不能仅凭“Goal succeeded”判定精度。

## 本轮实测启动方式

2026-09-13 曾出现整体冷启动时 controller_server 配置未结束、其余 Nav2 节点未激活。
Gazebo 已正常步进。以下分开启动方式已完成目标点验证，是规避手段，根因尚未确认；
没有恢复自动重启或生命周期修复门控。

终端一（Gazebo、感知和 RViz）：

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/setup.bash
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 DISPLAY=:1
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim launch_gazebo:=true launch_navigation:=false headless:=false \
  use_rviz:=true exploration:=false controller_plugin:=mppi \
  map_source:=real_file localization:=ground_truth \
  map_yaml_path:=/home/yjh/WorkSpace/astribot_sdk_ros2/maps/warehouse_baseline.yaml
```

确认物理步进、时钟和定位正常后，终端二启动 Nav2：

```bash
source /opt/ros/humble/setup.bash
source ws_robot/install/setup.bash
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1
ros2 launch astribot_s1_navigation navigation.launch.py \
  use_sim_time:=true controller_plugin:=mppi max_linear_speed:=0.5 \
  scan_topic:=/scan_from_cloud
```

验收数据见 [仿真验证记录](../astribot_s1_path_tracking/VALIDATION.md)。
