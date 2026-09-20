# MPPI 全局与局部代价地图配置校对

日期：2026-09-17。范围：当前源码、安装目录配置、启动覆盖、相关碰撞实现，以及本机 Nav2 Humble 1.1.20 对应上游源码。没有修改控制参数，没有启动导航或发送运动指令；没有读取真机当前运行参数。

结论：优先处理 **ObstaclesCritic 与局部膨胀参数不匹配**、**OFF 模式无回波清障链缺失**。不要先通过缩小全局膨胀半径解决停车问题。另有占据阈值、观测高度、足迹预算和更新时效需要统一。

## 1. 配置实值和启动覆盖

本次逐项比较 `ws_robot/src/astribot_s1_navigation/config/nav2_params_mppi.yaml` 与 `ws_robot/install/astribot_s1_navigation/share/astribot_s1_navigation/config/nav2_params_mppi.yaml`，文件内容一致。运行中节点仍可能有其他覆盖，文件一致不能代替运行参数快照。

| 项目 | 全局代价地图 | 局部代价地图 |
|---|---|---|
| 坐标系 | `map` | `odom` |
| 机器人基坐标系 | `astribot_torso_base` | 同左 |
| 地图形式 | 非滚动，StaticLayer 接入地图 | 6 × 6 m 滚动窗口 |
| YAML 分辨率 | 0.05 m；实际会随 StaticLayer 输入地图调整 | 0.05 m，即 120 × 120 格 |
| 更新 / 发布频率 | 1 / 1 Hz | 5 / 2 Hz |
| 层 | static + obstacle + inflation | obstacle + inflation |
| 名义足迹 | ±0.31 m 方形 | 同左 |
| `footprint_padding` | 未写，Nav2 默认每侧 0.01 m | 同左 |
| 膨胀半径 / 衰减系数 | 1.0 m / 3.0 | 0.65 m / 3.0 |
| 标记 / 射线清除距离 | 5.5 / 6.0 m | 同左 |
| `expected_update_rate` | 0.2 s | 0.2 s |
| `track_unknown_space` | true | 未写，默认 false |
| `inf_is_valid` | 未写，默认 false | 同左 |
| 全图发布 | true | true |

源文件：[全局/局部代价地图配置](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation/config/nav2_params_mppi.yaml:338)。默认值核对：[Nav2 1.1.20 Costmap2DROS](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp)。

启动覆盖不能遗漏：

- `navigation_policy_stage=off`：两个障碍层订阅传入的 `scan_topic`；P2 及以后改订阅 `/navigation_policy/costmap_scan`。
- 仿真默认 `scan_source=slice_scan`，原始输入为 `/scan_from_cloud`；部署脚本使用 `/scan`。
- 真机部署入口传入 `/map_nav`、`map_transient_local=false`、`use_sim_time=false`，并使用策略 OFF、MPPI、最大轴向线速度 0.35 m/s。
- YAML 的 MPPI `vx_max=vy_max=1.0` 会被启动参数改写；当前仿真和部署入口默认 0.35。直接调用底层 `navigation.launch.py` 则仍默认 1.0。

依据：[参数重写](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation/launch/navigation.launch.py:62)、[真机部署入口](/home/yjh/WorkSpace/astribot_sdk_ros2/tools/robot/run_deployed.sh:31)、[仿真入口](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation/launch/sim_stack.launch.py:65)。

## 2. 优先修复：ObstaclesCritic 的距离模型与地图不匹配

**确定的配置不一致。** `FollowPath.inner.ObstaclesCritic` 没有配置 `cost_scaling_factor` 和 `inflation_radius`。本机版本对应实现从 critic 自己的命名空间读取，默认分别为 **10.0、0.55 m**；并不会自动把局部 InflationLayer 的 **3.0、0.65 m** 同步进来。全局的 1.0 m 与这个局部控制评分器无关。

因此，它从代价值反算距离和计算排斥代价时，使用了与实际地图不同的模型。在中心点评分分支，以填充后内切半径约 0.32 m、机器人中心距障碍 0.50 m 为例，局部地图代价约 146，当前 critic 估计边缘余距约 0.055 m，匹配系数后约 0.183 m。这里只复现数学关系，不代表某次实测的真实距离。

最小候选修改是补齐以下参数；**本次没有应用**：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      inner:
        ObstaclesCritic:
          cost_scaling_factor: 3.0
          inflation_radius: 0.65
```

依据：[当前 critic 配置](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation/config/nav2_params_mppi.yaml:141)、[同版本 ObstaclesCritic 实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/obstacles_critic.cpp)。

还需留意该版本的另一个边界：接近障碍时它切换到足迹边界最高代价反算距离；边界进入 253 平台后，代价不再能表达连续的真实净空。补齐参数能修正不匹配，但不能保证消除这个评分平台。应对固定墙距和航向做评分连续性检查，再判断是否保留该 critic。

## 3. 优先修复：OFF 模式缺少无回波清障处理

**确定的链路差异，实际残留程度取决于场景。** 两个障碍层虽设 `clearing=true`，却没设 `inf_is_valid`。真机 `/scan` 的发布端设 `use_inf=true`，仿真切片扫描的无回波值是 `range_max`。Nav2 使用的投影只保留小于 `range_max` 的有效量程；这些无回波束不能自动形成清除射线。

仓库已有 `CostmapScanAdapter` 将 `+inf` 和恰好 `range_max` 转为略小于最大量程、但大于 5.5 m 标记范围的端点。问题是它仅在策略非 OFF 时启动。关闭策略时，感知清障语义也改变了，这会污染 OFF/P3 对照。

本机离线 `LaserProjection` 复现结果：

| 输入 | 投影后有效点 |
|---|---:|
| `[1.0, +inf, 20.0, 19.9999]`，最大量程 20 m | 2 / 4 |
| `[+inf, 20.0]` | 0 / 2 |
| `[19.9999, 19.9999]` | 2 / 2 |

当障碍移走、该方向只剩无回波时，缺少对应清除射线可能留下旧障碍。`observation_persistence=0` 是观测缓存语义，并不代表栅格占据自动过期。

建议把经过有效性验证的清障适配独立于策略开关复用，并确保端点仍在标记范围之外。仅增加 `inf_is_valid=true` 可处理正无穷，不能同时修复恰好等于 `range_max` 的有限值。未知/失效观测不能伪装成自由空间。

依据：[启动条件](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation/launch/navigation.launch.py:90)、[端点转换](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/protection.py:13)、[真机扫描配置](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception/config/pointcloud_to_laserscan_params.yaml:13)、[Nav2 ObstacleLayer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/obstacle_layer.cpp)、[LaserGeometry 投影条件](https://github.com/ros-perception/laser_geometry/blob/ros2/src/laser_geometry.cpp)。

## 4. 全局静态层与策略层的占据阈值不同

**配置语义不一致，是否触发需看地图数值分布。** 全局图未显式配置 `lethal_cost_threshold`、`trinary_costmap`，默认 100 / true；策略通道和起步恢复却将原始占据值 **≥65** 当作障碍。因此对 65～99 的地图单元，全局 StaticLayer 可解释为自由，策略解释为占据。

这能造成“规划路径有效，但策略不允许通行/转向”。若输入地图只有 -1、0、100，此差异暂时不触发；`/map_nav` 来自概率栅格链，必须采样直方图确认，不能假定它已经二值化。

建议在地图接口层定义统一占据阈值，核实上游概率语义后再配置 StaticLayer 和策略消费者。不能未经确认就把阈值从 100 改成 65。

依据：[策略阈值](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/start_maneuver_adapter.py:26)、[StaticLayer 的数值解释及尺寸调整](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/static_layer.cpp)。全局实际分辨率也以收到的 OccupancyGrid 为准，YAML 的 0.05 m 不能单独证明运行中也是 5 cm。

## 5. 真机扫描高度不足以覆盖整个双臂机器人

**确定的配置覆盖限制。** 代价地图 `max_obstacle_height=2.0` 不代表输入覆盖了 2 m 高度。真机部署的 `/scan` 来自 pointcloud_to_laserscan，其高度窗口只有基坐标系 **0.05～0.60 m**。投影完成后原始点的 Z 信息已丢失，代价地图不能恢复更高处障碍。

局部图又没有 StaticLayer，且只有这一路扫描：桌沿、悬挂物或上半身/双臂高度的障碍若未被扫描链保留，局部动态避障无法靠 `max_obstacle_height` 补救。仿真多高度 `/scan_from_cloud` 与这个真机输入不是同等覆盖。

建议以实际运动包络确定高度范围，选择同一多高度输出，或增加经过滤波的点云/视觉障碍源；先验证地面、自身回波与高处障碍，不直接把高度窗口放宽后上线。

另有近侧低障碍盲区需复核：切片节点的圆柱自滤半径 0.44 m、低高度范围 -0.20～0.20 m，超过方形机身侧向半宽 0.31 m。位于两者之间的真实侧方低矮障碍点也可能被剔除。它属于上游几何模型问题，调整 inflation 无法补回。

依据：[真机投影高度](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception/config/pointcloud_to_laserscan_params.yaml:5)、[感知接入](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception/launch/hardware_perception.launch.py:134)、[自滤参数](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception_components/config/pointcloud_slice_scan_params.yaml:66)。

## 6. 两个障碍评分器叠加，且接近目标时切换点不同

当前同时启用 `CostCritic` 与 `ObstaclesCritic`，两者都检查足迹并累计障碍相关代价。这不是语法错误，但会重复增加计算，并改变路径跟踪与障碍排斥的权重平衡。

`CostCritic.near_goal_distance=1.0`，另一个为 0.5；因此到点接近过程中，两个一般排斥项在不同位置停止生效。这可能影响贴路径程度和接近段平顺性，尚不能凭静态配置认定是历史波动的根因。

建议先修正第 2 项，再进行单 critic / 双 critic 的同路径对照。若只保留 CostCritic，应保留足迹碰撞检查，并验证净空、横向误差、航向误差和速度平顺性。`critical_cost=300` 在这个版本是惩罚值，不是“地图最大值只有 255，阈值永远触发不了”。

依据：[critic 列表与参数](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation/config/nav2_params_mppi.yaml:71)、[CostCritic 实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/cost_critic.cpp)。

## 7. 更新频率与观测时效值得调整，但要先量实际延迟

- 局部更新 5 Hz，控制 20 Hz：名义上每次地图更新之间有 4 次控制计算。按直行 0.35 m/s，200 ms 内机器人移动约 7 cm。该值只是更新周期对应的位移，不等于每帧一定滞后 7 cm。
- 全局更新 1 Hz：动态障碍进入全局路径证据可能有接近一个更新周期的等待；增加 BT 检查频率不能让旧地图变新。全局地图更新与全局重新规划是两个独立开关，提高前者不会自动恢复定时重规划。
- `expected_update_rate=0.2` 的单位是秒，即要求约每 200 ms 内收到更新，不是 0.2 Hz。扫描约 10 Hz 时余量有限，丢帧/TF 等待/处理抖动可能使 costmap 判为不新鲜。
- 两图未显式配置 `transform_tolerance`，默认 0.3 s；MPPI 路径变换为 0.2 s，真机点云投影为 0.02 s。三者服务不同环节，不要求机械地取同值，但需用同一条时间线审查丢帧和等待。

建议先记录 scan 采集龄期、实际输入间隔、costmap 更新耗时及是否 current，再将局部 10 Hz、全局 2～5 Hz 作为对照候选。不要只提高 `publish_frequency`：它主要影响可视化/订阅数据，不会直接提高 controller_server 内部共享地图的更新率。也不要先扩大时效阈值掩盖链路延迟。

依据：[ObservationBuffer 时效实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/observation_buffer.cpp)。

## 8. 足迹安全预算需要显式化

未覆盖默认 padding 时，±0.31 m 方形实际成为约 **0.64 × 0.64 m**，外接直径约 **0.905 m**。85 cm 通道居中直行时，几何单侧余距约 10.5 cm，而不是按 0.62 m 算出的 11.5 cm。

padding、栅格误差、策略 8 cm 净空、定位与跟踪预算是不同项，而且当前不同模块并非全部采用同一个足迹。不能在所有位置机械相加，也不能为凑 85 cm 指标直接删除 padding。建议显式配置它，并给每一层列出使用的几何及误差预算。

P2 及以后有 EnvelopeNode 同步两张地图足迹的握手；默认真机 OFF 不启动这个节点。臂底盘耦合限速不等于足迹自动扩展，必须另行保证机械臂处于匹配固定足迹的运输姿态。

依据：[包络发布与确认](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation_policy/astribot_s1_navigation_policy/envelope_node.py:32)、[启用条件](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_navigation/launch/navigation.launch.py:346)。

## 9. 目前不应直接判为错误的配置

1. **全局膨胀 1.0 m、局部 0.65 m 可以保留。** 膨胀层主要生成代价梯度，不表示每侧必须空出这个距离。局部半径也覆盖当前填充足迹约 0.453 m 的外接半径，不能为了窄通道盲目减到外接半径以下。
2. 仓库 Arrival 和 ExactGoalPlanner 的检查为足迹代价 ≥254 或中心代价 ≥253 等条件，不是“足迹碰到任意非零膨胀格就报碰撞”。`consider_footprint=true` 对方形底盘应保留。来源：[执行扫掠](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_path_tracking/src/arrival_controller.cpp:329)、[路径扫掠](/home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/src/astribot_s1_path_tracking/src/exact_goal_planner.cpp:226)。
3. 5 cm 分辨率不直接把终点位姿精度限制成 5 cm；到位控制是连续位姿闭环。但它限制几何边界的表达，85 cm 通道必须按不同栅格对齐位置验证。
4. MPPI 时域 56 × 0.05 = 2.8 s。当前 0.35 m/s 的单轴预测距离 0.98 m，6 m 局部窗口不算小。底层启动默认 1.0 m/s 时，2.8 m 加足迹已接近/超出半窗 3 m；全向组合和旋转还会改变投影，需要额外审查该高速入口。
5. 局部地图没有 StaticLayer 是常见设计选择，不是必然错误；本项目更需要确认扫描覆盖及未知空间处理。仅把 `track_unknown_space` 改为 true 并不能保证所有版本的 MPPI 都按同一未知禁行语义处理，必须核对完整消费链。
6. `always_send_full_costmap=true` 对当前 120 × 120 的局部图负担有限。大尺寸全局图、MPPI 轨迹可视化与 RViz 显示项可能叠加开销，但本次没有 CPU/消息体测量，不能据此归因当前卡顿。

## 10. 建议修改与验证顺序

| 顺序 | 内容 | 验证重点 |
|---|---|---|
| 1 | 显式同步 ObstaclesCritic 的局部膨胀参数 | 不同墙距/航向评分，固定路径与接近段质量 |
| 2 | OFF/P2/P3 共用有效无回波清障链 | 障碍移走后栅格清除、无回波不误标障碍、无效帧不清空环境 |
| 3 | 核对 `/map_nav` 数值分布并统一占据阈值 | 65～99 概率格，规划与策略同判 |
| 4 | 补齐真机高度覆盖与运输包络 | 低障碍、桌沿、悬挂物、双臂姿态变化 |
| 5 | 测量后优化更新率与时效 | 静态/横穿/持续阻挡场景的延迟与误停 |
| 6 | 对照障碍 critic 组合与权重 | 横向/航向误差、净空、加减速平顺性，不以时长排名 |

最终回归应同时保留普通路径、窄直通道、短斜入口、接近终点、障碍移除、动态横穿和反向起步。不能只验证“到达成功”；也不能因配置审计通过就宣称实机或完整窄通道验收通过。
