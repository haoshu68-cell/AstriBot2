# astribot_s1_dynamics_coupling —— 臂-底盘动力学耦合动态调速

独立、无侵入的中间控制层：根据双臂展开幅度 + 关节角速度实时计算 0~1.0 连续限速
系数，在"世界坐标系速度输出之后、gz-sim `VelocityControl` 插件接收之前"这个
位置对最终 `Twist` 做缩放，缓解机械臂运动诱发的重心偏移/姿态漂移/倾倒风险。

```
Nav2/自主巡游 → cmd_vel_body_to_world_node(既有，未修改) → cmd_vel_pre_arm_coupling
    → 【本包：arm_chassis_speed_coupling_node】 → 真正的 /cmd_vel → VelocityControl
```

## 1. 一键启用/关闭

已经默认接入 `astribot_s1_navigation` 的 Nav2 启动链路，不需要额外命令：

```bash
# 默认就是打开的
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=mapping launch_gazebo:=true controller_plugin:=mppi

# 关掉（回到接入耦合节点之前完全一样的行为）
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=mapping enable_arm_chassis_coupling:=false
```

## 2. 无侵入接入方式

**没有修改任何既有文件的逻辑**——`cmd_vel_body_to_world_node.py`、既有的静态
`arm_speed_limiter_node.py`、`VelocityControl`/TF/SLAM/雷达滤波/Nav2 核心配置
全部原样保留。接入方式只是在 `navigation.launch.py` 里：
1. 把 `cmd_vel_body_to_world_node` 本来就有的 `output_topic` 参数（默认就是
   `/cmd_vel`）**覆盖**成中间话题 `cmd_vel_pre_arm_coupling`；
2. include 本包的 `arm_chassis_coupling.launch.py`，桥接
   `cmd_vel_pre_arm_coupling → /cmd_vel`，中间做动态缩放。

`enable_arm_chassis_coupling:=false` 时这两步都不生效，行为跟接入之前完全一样。

跟既有的**静态**臂限速节点（`astribot_s1_navigation/arm_speed_limiter_node`，走
Nav2 官方 `/speed_limit` 机制、二值判断"展开/未展开"）是两套完全独立、互不知道
对方存在的保护：一个作用于 `controller_server` 的速度上限（更早的阶段），
一个直接在最终 world 系速度上做连续缩放（最后的阶段）。两者同时生效，不冲突，
也不是谁替代谁。

## 3. 动态限速系数怎么算的

展开幅度、运动速率两个维度各自独立换算成 0~1.0 的"活跃度比例"（分别除以
`extension_full_rad`/`velocity_full_rad_s` 这两个"满量程"阈值，超过就封顶在1.0），
取两者**较严格**（活跃度更高）的一个作为最终活跃度，不做加权平均——宁可保守，
不要因为"平均下来还凑合"而放过某一个维度的剧烈运动。

```
scale_raw = 1.0 - activity * (1.0 - min_speed_scale)
scale = 低通平滑(scale_raw)   # scale_smoothing_alpha，避免关节噪声导致速度顿挫
```

`/joint_states` 超过 `joint_state_timeout_sec`（默认0.5s）没更新，判定数据陈旧，
自动降级到固定的 `degraded_scale`（默认0.3），不是不限速也不是直接停机——
对应任务书"话题超时不中断导航任务"的要求。

## 4. 关于"静默失效、保持原生全速运动"的能力边界（如实说明）

**运行时逻辑层面**做到了：`arm_chassis_speed_coupling_node.py` 里每一步计算
（解析关节数据、算系数、发布）都包在 `try/except` 里，任何单次异常都 fall back
到 `scale=1.0`（原样转发，不限速），不会 fall back 到 0（那样反而更危险）。

**但**如果本节点的**进程整体崩溃退出**，因为它是"中间话题→真正/cmd_vel"这条链路
唯一的桥接者，进程不在了，新指令确实没法再送到真正的 `/cmd_vel`——这是"无侵入
中间人"架构在拓扑上无法回避的限制（除非直接改上游节点的输出目标，那样就不是
"无侵入"了）。这里用 launch 里的 `respawn=True` 做进程级兜底（挂了自动重启，
有短暂中断，不是零感知无缝切换）。如实记录这个边界，不假装是完美的容错。

## 5. 参数（`config/arm_chassis_coupling_params.yaml`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `extension_full_rad` | 1.2 | 任意关节偏离收纳姿态达到这个值(rad)，展开维度活跃度封顶1.0 |
| `velocity_full_rad_s` | 2.0 | 任意关节角速度达到这个值(rad/s)，速率维度活跃度封顶1.0 |
| `min_speed_scale` | 0.15 | 活跃度满量程时的限速系数下限（不会降到0，避免完全卡死） |
| `scale_smoothing_alpha` | 0.25 | 限速系数一阶低通滤波系数 |
| `joint_state_timeout_sec` | 0.5 | `/joint_states` 陈旧判定超时 |
| `degraded_scale` | 0.3 | 数据陈旧时的固定安全限速 |
| `folded_reference_rad` | 全0 | 双臂14关节"收纳姿态"参考角度，**部署前需按实际收纳姿态核对** |

## 6. 调试指令

```bash
# 确认节点在跑、看实时限速日志
ros2 node list | grep arm_chassis_speed_coupling
ros2 topic echo /cmd_vel_pre_arm_coupling   # 缩放前(上游真实想要的速度)
ros2 topic echo /cmd_vel                    # 缩放后(实际发给VelocityControl的)

# 手动给一个大幅展开的关节角，观察限速系数变化(日志里能看到"机械臂活跃度=...限速系数=...")
ros2 action send_goal /arm_left_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [astribot_arm_left_joint_2], points: [{positions: [-1.2], time_from_start: {sec: 2}}]}}"
```

## 7. 验证步骤

1. `colcon build --symlink-install --packages-select astribot_s1_dynamics_coupling astribot_s1_navigation`
2. 正常拉起 Nav2 导航（见第1节），确认 `ros2 node list` 里有
   `arm_chassis_speed_coupling_node`，且导航目标点仍能正常送达（跟接入前行为一致）。
3. 导航过程中给手臂发一个大幅展开的轨迹，对比 `/cmd_vel_pre_arm_coupling` 和
   `/cmd_vel` 的幅值差异，确认展开时后者明显更小。
4. `enable_arm_chassis_coupling:=false` 重跑一遍，确认导航行为跟打开时（未展开
   机械臂的情况下）一致，证明"关掉即等同于没接入"。
5. （可选）手动 `kill` 掉节点进程，确认 `respawn=True` 生效、几秒内自动重启。
