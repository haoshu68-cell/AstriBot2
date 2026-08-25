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

展开幅度、运动速率两个维度各自独立换算成 0~1.0 的"活跃度比例"，取两者**较严格**
（活跃度更高）的一个作为最终活跃度，不做加权平均——宁可保守，不要因为"平均下来还
凑合"而放过某一个维度的剧烈运动。

- **展开维度**（C1 修正后）：查 TF 求 `monitored_links` 里各连杆相对
  `chassis_base_frame` 的**水平距离**，取最大值当"伸展量"，在
  `reach_folded_m ~ reach_full_m` 之间线性映射到 0~1.0。
- **运动速率维度**（C1 未改动）：任意关节角速度绝对值 / `velocity_full_rad_s`。

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
| `extension_metric` | `horizontal_reach` | 展开度量。`joint_deviation` 是已被证伪的旧度量，只用于 A/B 回归或临时回退 |
| `chassis_base_frame` | `astribot_torso_base` | 底盘回转轴所在 frame（本机没有 `base_link`） |
| `monitored_links` | 双臂 TCP + 夹爪指尖 | 实测最大伸展总落在这几个连杆上，换夹爪要改这里 |
| `reach_folded_m` | 0.42 | 低于此伸展量不限速。取值 = Nav2 `robot_radius`：还在足迹以内时规划器已算进去了 |
| `reach_full_m` | 0.8865 | 达到此伸展量活跃度封顶 1.0。取值 = 实测全工作空间最大水平伸展 |
| `reach_update_period_sec` | 0.05 | TF 查询节流（20Hz），机械臂运动远慢于此 |
| `velocity_full_rad_s` | 2.0 | 任意关节角速度达到这个值(rad/s)，速率维度活跃度封顶1.0（C1 未改动） |
| `min_speed_scale` | 0.15 | 活跃度满量程时的限速系数下限（不会降到0，避免完全卡死） |
| `scale_smoothing_alpha` | 0.25 | 限速系数一阶低通滤波系数 |
| `joint_state_timeout_sec` | 0.5 | `/joint_states` 陈旧判定超时 |
| `degraded_scale` | 0.3 | 数据陈旧时的固定安全限速 |
| `folded_reference_rad` / `extension_full_rad` | 全0 / 1.2 | **仅 `extension_metric=joint_deviation` 时生效**，保留只为回归对比 |

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

## 8. C1 修正：为什么换掉"关节偏差"度量（实测取证 + 倾覆余量重算）

所有数字都由**活的 URDF**（`/robot_state_publisher` 的 `robot_description`）采样得到，
不含任何硬编码的 DH 参数、连杆长度或质量常数。

### 8.1 旧度量与物理量反相关

| 姿态 | 关节偏差(旧度量) | 水平伸展 | 臂质心水平偏移 | 旧耦合系数 | 旧静态限速 | 旧合计 |
|---|---|---|---|---|---|---|
| 全0（原来的"收纳基准"） | 0.0000 | **0.4205 m** | 0.0117 m | 1.000 | 1.00 | **1.000** |
| `ready` | 1.0000 | 0.4790 m | 0.0226 m | 0.292 | 0.50 | **0.146** |
| 实测（左臂作业中） | 2.1104 | 0.4698 m | 0.0202 m | 0.150 | 0.50 | **0.075** |
| 候选收纳（肘部折回） | 2.4000 | **0.3532 m** | **0.0025 m** | 0.150 | 0.50 | **0.075** |

- 真正最收拢的姿态（伸展 0.3532 m）拿到最狠的限速；而基准姿态"全0"其实是肘部完全
  伸直、伸展 0.4205 m 已经顶到足迹半径的姿态，却完全不限速。**换基准姿态改变不了
  这个反相关**，所以 C1 换的是度量本身，不是基准。
- 全工作空间随机采样 4000 次：**最大伸展 0.8865 m 时关节偏差 3.062，最小伸展
  0.1997 m 时关节偏差 3.079**——偏差几乎相同，伸展差 4.4 倍。旧度量基本不携带伸展信息。

### 8.2 倾覆余量重算（换阈值前必须做的那一步）

| 量 | 值 | 来源 |
|---|---|---|
| 支撑多边形 | 4 轮 xy = (±0.2163, ±0.2163) | URDF |
| 临界方向余量 | **边中点 0.2163 m**（不是轮心 0.306 m） | 取保守方向 |
| 整机质量 | 78.595 kg（双臂+夹爪 17.963 kg，占 22.9%） | URDF |
| 全0 姿态整机质心 | xy = (−0.0359, +0.0027)，z = **0.4321 m** | URDF |
| 最差静态余量 | 质心已偏后 0.036 m → **向后 0.180 m** | 计算 |
| 倾覆所需水平加速度 | `a_tip = g·d/h = 9.81 × 0.180 / 0.4321 = 4.09 m/s²` | 计算 |
| Nav2 指令加速度上限 | `max_accel: 2.5 m/s²`（= 倾覆阈值的 61%） | `nav2_params_rpp.yaml` |

**倾覆风险是真实存在的（余量只有 39%），C1 没有否掉它。** 但可归因于**机械臂姿态**
的那一部分很小——整机质心偏移 = (17.963/78.595) × 臂质心偏移 = 0.2285 × 臂质心偏移：

| 姿态 | 臂质心偏移 | 整机质心偏移 | 余量 0.180 → | a_tip | 相对变化 |
|---|---|---|---|---|---|
| 候选收纳 | 0.0025 | 0.0006 m | 0.179 m | 4.07 m/s² | −0.5% |
| 全0 | 0.0117 | 0.0027 m | 0.177 m | 4.03 m/s² | −1.5% |
| `ready` | 0.0226 | 0.0052 m | 0.175 m | 3.97 m/s² | −2.9% |
| 全工作空间最差 | 0.0868 | 0.0198 m | 0.160 m | 3.64 m/s² | **−11.0%** |

即：**机械臂姿态最多让倾覆阈值降 11%（全工作空间极限），实际用到的姿态只降
1.5~2.9%，而修正前的方案为此把底盘速度砍到 1/7~1/13。** 这个不成比例是 C1 要修的
真正问题。修正后 `min_speed_scale` 仍是 0.15（限到多少没放松），只修正了"什么时候
该限速"。

### 8.3 同一个坏基准在两个包里

`astribot_s1_navigation/arm_speed_limiter_node`（走 Nav2 官方 `/speed_limit`、二值
判断）用的是同一套全0 基准 + 0.5 rad 阈值，任何作业姿态都判"展开"、恒定砍到 50%，
和本节点的系数**串联叠乘**：0.5 × 0.15 = 0.075，把 `desired_linear_vel: 0.5` 压到
约 0.037 m/s（实测导航平均 0.031 m/s，对得上）。所以 C1 两个包都改了，只改一边无效。

### 8.4 修正后各姿态的系数

静态限速节点的阈值取 **0.64**，来历是一条可解释的物理判据：

```
0.42   costmap robot_radius —— 机械臂开始伸出规划足迹的位置
+0.2163 支撑多边形边中点到回转轴的距离 —— 底盘自己的倾覆力臂
=0.6363 → 取 0.64
```

含义：机械臂**伸出足迹之外的那一段**长到跟底盘自己的倾覆力臂相当时，才动用这个
粗粒度二值保护。0.42~0.64 这一段由本包的耦合节点连续调速处理，不再叠第二刀。

| 姿态 | 伸展 | 新活跃度 | 新耦合系数 | 新静态限速 | 新合计 | 相对修正前 |
|---|---|---|---|---|---|---|
| 候选收纳 | 0.353 | 0.000 | 1.00 | 1.00 | **1.00** | 0.075 → **13.3×** |
| 全0 | 0.4205 | 0.001 | 1.00 | 1.00 | **1.00** | 1.000 → 不变 |
| 实测作业 | 0.470 | 0.107 | 0.91 | 1.00 | **0.91** | 0.075 → **12.1×** |
| `ready` | 0.479 | 0.127 | 0.89 | 1.00 | **0.89** | 0.146 → **6.1×** |
| 工作空间极限 | 0.8865 | 1.000 | **0.15** | **0.50** | **0.075** | 保持最严 |

> **这里跟最初的方案有一处偏离，如实记录。**
> 方案里静态节点的阈值原定 0.42（= `robot_radius`）。实际算下来发现：`ready`
> (0.479)、实测作业姿态(0.470)、甚至全 0 姿态(0.4205) 的伸展**全都超过 0.42**，
> 50% 那一刀等于常开，合计只能从 0.146 升到 0.44——那还是"限速与风险不成比例"这个
> 同一个问题，只是换了个判据。所以把静态节点的定位明确成**粗粒度 backstop**
> （耦合节点被 `enable_arm_chassis_coupling:=false` 关掉、或崩溃重启期间兜底），
> 阈值按上面的物理判据抬到 0.64。这是一次**额外的安全阈值放松**，依据是 §8.2 的
> 倾覆余量算术（机械臂姿态最多贡献 11% 阈值下降）。要退回更保守的行为，改
> `extended_reach_m` 一个值即可，不需要改代码。

### 8.4.1 迟滞

全 0 姿态伸展 0.4205 m 与 `robot_radius` 0.42 只差 **0.5 mm**（纯巧合）。这类"伸展
恰好压在阈值上"的情形会让二值判据在静置时来回翻转，每次翻转都发一条 `SpeedLimit`，
`velocity_smoother` 的上限就在 50%/100% 之间反复跳。静态限速节点因此带
`extended_reach_hysteresis_m: 0.03`：一旦判展开，要降到 0.61 以下才解除。触发阈值
不受迟滞影响（迟滞只放宽解除，不放宽触发，否则等于偷偷抬高了保护阈值）。

### 8.5 限速不解决碰撞包络（C1b，尚未实现）

实测机械臂水平伸展最大可达 **0.8865 m**，比 `robot_radius: 0.42` 多伸出 **0.47 m**。
**限速只降低速度，完全不缩小碰撞包络**——这部分是规划器彻底看不见的真实碰撞风险，
限速一点也没减少它。原来 `nav2_params_rpp.yaml` 里有一句注释声称这个风险"靠
`arm_speed_limiter_node` 的展开限速缓解"，那句是错的，已更正。正确机制是姿态相关的
动态足迹（`nav2_collision_monitor` 订阅机械臂 TF 实时改 footprint），**立项为 C1b，
本次未实现**，如实标为未解决。
