# SLAM / 导航数据流与坐标基准（实测）

- 测量时间：2026-08-27（机器 uptime 内第 13 小时，本体驱动已停）
- 测量手段：机器人端原生 rclpy 只读探针（`/tmp/probe_chain.py` ~ `probe6.py`），**不发布任何话题**
- 判据：话题存在性一律用 `count_publishers() > 0`；frame 名一律取自 TF 树 dump，不凭记忆

> **为什么不用 `ros2 topic`**：厂商 middleware 覆盖了 `ros2cli` 的扩展点，
> 连 `/opt/ros/humble/bin/ros2` 也只剩 `bag/daemon/node/param/service` 五个子命令，
> `ros2 topic list` 直接报 `invalid choice: 'topic'`。
> 我第一次把那三行报错文本的行数读成了"话题数: 3"，误判成 DDS 发现失败。
> 正确姿势：`env -i` 清干净环境 + `source /opt/ros/humble/setup.bash` + 原生 rclpy。

---

## 一、数据流全图（括号内为实测发布者数）

```
MID360 ×2（硬件）
  └─ livox_ros_driver2                                    pid 6960
       ├─ /livox/lidar_front   CustomMsg            (1)   ← xfer_format=1
       ├─ /livox/lidar_back    CustomMsg            (1)
       └─ /livox/imu_front                          (1)
            │
            ▼
     voxelslam                                            pid 7678
     config: .../voxel_slam/share/voxel_slam/config/mid360.yaml
     消费 lid_topic=/livox/lidar_front, imu_topic=/livox/imu_front
       ├─ TF  camera_init → aft_mapped                    20Hz（实测 200 次/20s）
       ├─ /map_scan            PointCloud2 frame=camera_init  (1)  ← 未裁 z
       ├─ /map_scan_filtered   PointCloud2 frame=camera_init  (1)  ← z∈[0.05,1.63]
       └─ /map_cmap /map_pmap /map_init /map_path /map_true /map_test  (各 1)
            │
            ├──────────────────────────────┐
            ▼                              ▼
   cloud_to_grid_node  pid 33673      map_odom_tf_node  pid 33674
   消费 /map_scan_filtered            消费 TF camera_init→aft_mapped
        + TF camera_init→aft_mapped         + TF odom→astribot_torso_base
   产出 /map OccupancyGrid            产出 TF map→odom            20Hz（实测 401/20s）
        frame_id = camera_init  (1)        实测 x=-1.0700 y=0.6019 yaw=-16.42°
        origin=(-3.4372,-1.5833,0)
        res=0.05  201×142
            │
            ▼
   nav2  pid 970128~970142
     /map → global_costmap → planner_server → /plan            (1)
     /plan → controller_server → /cmd_vel                      (6)
     local_costmap ← /scan                                     (0)  ← 恒空
            │
            ▼
   bridge_container                    ★ 已死（2026-08-26 21:00:31）
     应产出 /odom                                              (0)
     应产出 TF odom→astribot_torso_base                        缺失

   robot_state_publisher  pid 82960
     消费 /joint_states                                        (0)  ← 本体驱动没跑
     只发得出固定关节 → TF 树碎裂（见第三节）
```

---

## 二、坐标基准：SLAM 的 z=0 **就是地面**

结论与"基准不是地面"的猜测相反。两条互相独立的推导吻合到 **3 mm 以内**。

### 推导 A：几何（URDF + SLAM 外参）

从 `/robot_description` 话题直接取到的 URDF：

| 量 | 值 | 来源 |
|---|---|---|
| `wheel_RF_Joint` origin | xyz z = **−0.015**，**rpy = (2.35619, −1.5708, 0)** | URDF joint |
| 轮子碰撞球心（link 内） | xyz = (0, 0, **0.0145**) | `<collision><origin>` |
| 轮子球半径 | **0.08** | `<sphere radius="0.08">` |

> ⚠ **必须带上 joint 的 rpy 一起算。** 球心偏移 `(0,0,0.0145)` 经
> `rpy=(2.35619, −1.5708, 0)` 旋转后是 `(0.010253, −0.010253, 0.000000)` ——
> **对 z 的贡献恰好为 0**。我第一版正则只抓了 `xyz` 没抓 `rpy`，据此算出
> 地面 −0.0805，并差点把仓库里本来正确的记录改坏。

```
球心   = −0.015 + 0        = −0.015      (相对 torso_base)
地面   = −0.015 − 0.08     = −0.095      (相对 torso_base)
```

| 量 | 值 |
|---|---|
| `astribot_torso_base` → `livox_mid360_left` | z = **+0.082**（2026-09-01 修正，见下） |
| 雷达相对**地面** | 0.082 − (−0.095) = **0.177** |
| 雷达离地（**独立实测**，点云地面拟合） | **0.1745**（与上式差 2.5 mm） |
| SLAM `chassis_extrinsic_tran` 的 z | **0.082** |
| **⇒ SLAM 的 chassis 原点（= `camera_init` 原点）相对地面** | **+0.095 m** |

> ⚠️ **2026-09-01 修正：本节原先算出的 "+0.017 m" 是错的。** 两个输入都不对：
> 1. `astribot_torso_base → livox_mid360_left` 的 z 当时读到 **0.100**，
>    但那是 xacro 宏的**占位默认值**，不是实机安装位姿。实测标定是 **0.082**。
>    （"实测 TF" 这个标注也误导 —— 我量的是 TF 树，而 TF 树只是把 URDF 里的
>    占位值原样发出来。**量一个由错值生成的量，不构成对那个值的验证。**）
> 2. `chassis_extrinsic_tran` 的 z 当时记成 **0.178**，实际配置里是 **0.082**。
>
> 修正后的结论是：`camera_init` 原点 = 底盘启动位姿 = `astribot_torso_base`，
> 离地 **0.095 m**，**不是地面**。这一条有三个互相独立的证据：
> - 点云地面拟合：front 雷达离地 0.1745 m，减去雷达在 chassis 里的 0.082 → 0.093 m
> - SLAM 源码：`x_curr.p = g_t_chassis_imu` 把世界系锚在**底盘**启动位姿
>   （而非 IMU），所以 `camera_init` 原点就是底盘中心
> - `mid360.yaml` 的 `nav_scan_z_min/max` = `-0.046 / 1.534`，正好是通用默认值
>   `0.05 / 1.63` 各减 **0.096** —— 配置作者本人就是按这个偏移补偿的
>
> 也就是说 rviz 里"SLAM 基准不在地面"的观察是**对的**（差 0.095 m）。
> 附带风险：源码注释仍写着 "world-frame z is used directly as
> height-above-ground"，那句话现在是误导的 —— 偏移藏在 yaml 数值里。

### 推导 B：直接量世界系点云

`/map_scan`（**未**裁 z，frame=`camera_init`），抽样 2573 点：

```
z: min=-0.0038  p1=0.0224  中位=0.7286  p99=4.3603  max=4.3735
z 最密集的 0.01m 桶: 0.47→40, 0.67→34, 0.22→32, 0.03→31, 0.73→31
```

最低点 **−0.0038 m**，且没有任何点落在 −0.02 以下。若基准在 `astribot_torso_base`
（地面之上 0.095），地面点应出现在 z ≈ −0.095 —— 实测没有任何点低于 −0.02。
这排除了"基准在躯干高度"，但**不足以**把基准钉到毫米级（见下）。

### 结论

```
camera_init 的 z = 0  位于地面上方 0.017 m（几何推导）
                      点云最低回波 −0.0038 m（实测，与之相差 13mm）
  ⇒ 无论取哪个数，基准都在地面 ±2cm 以内
  ⇒ 它**不在**躯干高度(0.095)、也**不在**雷达高度(0.195)
  ⇒ /map 的 origin.z = 0.0000 正确
  ⇒ nav_scan_z_min/max = [0.05, 1.63] 实际约等于"离地 3.3cm ~ 1.61m"
```

两条推导差 13 mm。差在哪不确定：稀疏点云的最低回波是很弱的地面估计量 ——
雷达以 22.9° 装角、俯视视场有限，可能只是**擦到**地面而非看清地面
（`z` 最密集的桶是 0.47/0.67/0.22，没有地面该有的那种主导平面）。
**要定到毫米级需要专门测一次地面平面拟合，本次没做。**

可以确定的是：**"基准不是地面"这个猜测不成立** —— 基准就在地面附近，
误差量级 1~2cm，不是 0.1m 或 0.2m 那种"错了一整个身高"的量级。

---

## 三、实际发现的四个问题（都不是基准高度）

### P1 ★ `map_odom_tf_node` 正在以 20Hz 发布一份 **13 小时前的陈旧 TF**，且不报警

`bridge_container` 昨晚 21:00 死了，`odom→astribot_torso_base` 从此不存在
（20 秒 `/tf` 抓包只有 `map→odom` 和 `camera_init→aft_mapped` 两条边）。
但节点自报：

```
map→odom=(-1.071, 0.600, -0.286)  更新=1055636  跳变=20(最大 0.626m)
                                  倾角超限=0  缺失 slam=2 odom=25
```

`更新` 每 10 秒涨 200（= 20Hz，一直在成功发布），而 `缺失 odom` 累计只有 **25** 次。
即：**`_tick` 认为自己拿到了 odom**。

原因是 `_lookup` 用 `lookup_transform(..., Time())` 取"最新可用"。tf2 的 buffer
只在**该 frame 对有新数据进来时**才做过期裁剪；一条再也不更新的边，它的最后一帧
会永久留在 buffer 里被当成"最新"返回。

后果：`map→odom` 冻结在机器人 13 小时前的位置上，而下游（global_costmap、rviz）
完全看不出异常。这与该节点 docstring 里批评 `slam_adapter_node`
"拿不到 odom 时发单位变换、且不会有任何报错"是**同一类缺陷**，只是从"发单位阵"
变成了"发陈旧值"。

修法：`_lookup` 必须校验时间戳龄期（`slam_max_age_sec` 那套已有的口径），
超龄按缺失处理并停止发布。

### P2 ★ 地图的 frame 是 `camera_init`，但 TF 里的全局 frame 叫 `map`,两者无连接

- `/map` 实测 `frame_id = camera_init`（`cloud_to_grid_params.yaml:45` 默认值）
- `map_odom_tf_node` 发的是 `map → odom`
- **不存在 `map → camera_init` 这条边**（`publish_map_to_slam_world` 没生效）

所以凡是以 `map` 表达的东西和凡是以 `camera_init` 表达的东西**互相无法换算**。
`cloud_to_grid_node.py:104` 自己就写了这个坑："需要 map→camera_init 的静态 TF ——
但**不能只改这里**"。现在正是只有一半。

### P3 TF 树碎成 7 个互不相连的根

实测 15 个 frame，7 个根：

```
[map]                       <- odom
[camera_init]               <- aft_mapped
[astribot_torso_base]       <- livox_mid360_left, livox_mid360_right
[astribot_torso_link_4]     <- astribot_torso_end_effector
[astribot_arm_left_link_7]  <- astribot_arm_left_tool_link, astribot_gripper_left_base
[astribot_arm_right_link_7] <- astribot_arm_right_tool_link, astribot_gripper_right_base
[astribot_head_link_2]      <- camera_link
```

`map → astribot_torso_base` 和 `odom → astribot_torso_base` 都查不到。
除 P2 之外，其余碎裂全部由 `/joint_states` 无发布者解释（本体驱动没跑 →
`robot_state_publisher` 只能发固定关节）。**本体驱动一起来这部分会自动愈合，P2 不会。**

### ~~P4 我们 yaml 里的 z 切片是死配置~~ —— **本条作废，是我读错了**

原判断：`cloud_to_grid` 自报 `切片外=0`，据此断言"我们那份 yaml 的 z 范围是死配置，
调了看不到效果"。**错的。**

实际情况（`config/cloud_to_grid_params.yaml:70-71`）：

```yaml
    z_min: -.inf
    z_max: .inf
```

是**故意**放通的，理由就写在它上面几行；而且 `cloud_to_grid_node.py:205` 有一个
专门的 `_warn_if_double_filtering()`，一旦有人把 z 区间收窄到上游 `[0.05, 1.63]`
以内就响亮告警，其 docstring 里还引用了本仓库"两层限速串联相乘"那个旧坑。

所以 `切片外=0` 是这个设计的**预期结果**：参数放通 → 一个点都不该被本节点拒绝。
把它当"配置失效"的证据，是我看了一个计数器就下结论，没去读那份 yaml 写的是什么。

唯一成立的部分：**障碍高度带的权威点确实在厂商 `mid360.yaml`**
（`nav_scan_z_min/max`），我们这侧是"可再收紧、默认不收紧"。这值得知道，
但它是既有设计，不是缺陷。

### 附：`aft_mapped` ≡ `astribot_torso_base`（2026-09-01 更正，原结论作废）

`map_odom_decompose` 的前提是这两个 frame 指同一物理点，其 docstring 明确说
"这是决策给定的，不是本节点能验证的"。**现在验证过了，前提成立。**

> ⚠️ **本节原先写的"差 0.078 m（= 0.178 − 0.100）"是错的**，两个输入都不对：
> `0.100` 是 URDF 里雷达的**占位默认值**，`0.178` 是我误读的 `chassis_extrinsic_tran`
> （配置里实际是 `0.082`）。两个错值之差恰好凑出一个看似合理的 0.078，
> 这正是它当时没被发现的原因。

实测（点云拟合地面，免运动）：

| 校验项 | 预期 | 实测 | 差 |
|---|---|---|---|
| front 雷达离地 | 0.1770 m（= 0.082 + 0.095） | **0.1745 m** | 2.5 mm |
| 两路点云报出的地面高度差 | 0 | 0.0006 m | 0.6 mm |
| 雷达倾角（外参是纯 Rz） | 0° | 0.43° | — |
| 两雷达连线中点 | (0, 0) | (−0.0036, −0.0042) | 5.5 mm |

也就是说 SLAM 的 `chassis` 原点与 `astribot_torso_base` 在 **2.5 mm** 内重合，
`camera_init` 原点离地 **0.095 m**。分解的前提是真的。

z 方向本来也不影响 —— 分解是 2D 的（x, y, yaw），实测 `map→odom` 的 z 恒为 0.0000。
但 xy/yaw 方向的偏移**尚未测**，而 SLAM 外参与 URDF 在 x 上差 0.106 m
（0.17393 vs 0.28）、在 yaw 上差 22.9°（45.85° vs 22.92°，恰好 2 倍，可疑但未查）。
若 xy/yaw 也真差这么多，症状就是 docstring 预测的"地图与机器人系统性偏移一个常量"。
**这条只能等本体驱动起来后用"同段运动的 SLAM 位移 vs SDK 位移"实测**，现在无法判定。

---

## 四、其他实测记录

| 项 | 实测 | 说明 |
|---|---|---|
| `/scan` | pub=**0** | local_costmap 恒空，无动态避障。**原因已更正**：不是"xfer_format 互斥无解"，而是驱动发 `CustomMsg`(xfer_format=1) 而切片链吃 `PointCloud2`，中间缺转换环节 |
| `/cmd_vel` | pub=**6** | 昨天是 5，多出来一个未查 |
| SLAM 回环跳变 | 20 次，最大 **0.626 m** | 设计上应该跳，但 0.626m 会让全局路径整体重规划 |
| `cloud_to_grid` 限流 | 收 527899 / 投影 25135，**跳过 95%** | 投影耗时 0.02s，正常 |
| 点云的 `frame_id` | 实测两路**都是** `livox_frame` | **本行原先写的"真实 frame 是 `livox_mid360_left/right`"是错的**。驱动 launch 里 `frame_id` 是一个全局值，两路共用；而这并非疏漏——外参已写进雷达设备，back 的点本来就在 front 系里。2026-09-01 已在 URDF 补 `livox_mid360_left→livox_frame` 恒等边把两套命名接起来 |
| `web_astribot_2135` | 刷 **466770** 次 `Driver crashes...Exiting` | 厂商进程卡在退出循环里没退，13 小时以 10Hz 灌 `/rosout` |
| nav2 的 `use_sim_time` | 实机上六个节点**全是 True** | `/clock` 发布者数为 0，其时钟恒为 0 且永不前进（实测）。costmap 照发、进程都活，判据也过 —— 静默失败。`navigation.launch.py` 的默认值就是 `'true'`，直接调它会中招；正确入口 `nav2_full_bringup.launch.py` 会按 `env` 推导并显式传 |

---

## 五、未完成工作清单（更新至 2026-09-01 14:25）

优先级按"**会不会让结论/安全出错**"排，不按工作量。

### 铁律（用户 2026-09-01 定）

> **每次开真机写通路 / 调 `~/enable` / 发速度前，必须先向用户确认，且要报出朝向与距离。**
> 技术前置（`/cmd_vel` 无帧、电量足、leash 生效）**不构成授权**。

### 当前机器人状态

| 项 | 值 |
|---|---|
| 桥接容器 | 运行中，`写通路=被拒绝`，从未使能 |
| 外环闭环 | 关（`enable_slam_correction:=false`） |
| 底盘 | 静止，位置 `(23.2660, 3.8769)`，30s 跨度 0.00000m |
| 只读节点 | `state_bridge` / `chassis_odom` 正常 |
| 已部署 | B1 + B3 + 两个窗口修复；机器人上 **501 passed / 0 skipped** |

---

### A 类：阻塞后续真机验证

| # | 事项 | 状态 |
|---|---|---|
| A2 | **P1 陈旧 TF** —— `map_odom_tf_node` 以 20Hz 发布一份 13 小时前的 `map→odom`，自报"缺失 odom=25"看着健康 | **未修**。修之前任何"定位准不准"的结论都不可信。修法：`_lookup` 校验时间戳龄期，超龄按缺失处理并停止发布 |
| A3 | **P2 `map` / `camera_init` 无连接** —— `/map` 的 frame_id 是 `camera_init`，TF 全局 frame 叫 `map`，两者无边 | **未修**。后果：costmap 与机器人位置无法换算，我因此无法判断前向净空 |
| A4 | **使能瞬间 3.3cm 位移**（本次跳过） | 机制未查。疑似 `enable` 用 `get_desired_joints_position` 做种子，desired≠actual 时首写把底盘拽向 desired |

### B 类

| # | 事项 | 状态 |
|---|---|---|
| ~~B1~~ | ~~内环 `dt=1/freq`~~ | **已完成并部署**。实测拍率 **237.9Hz（95%）**，非此前误报的 91Hz/36%。钳位 0/13166 拍 |
| B2 | **终点晃动 → 滑行量** | 三成因已查清；**滑行量仍未测到**。需发速度 → 需你确认朝向与距离；且前方危险，建议换方向或换场地 |
| ~~B3~~ | ~~单测全 skip~~ | **已修**：模块级 `pytest.importorskip` 在 pytest 6.2.5 下中止整个 collection。`1 skipped` → **501 passed** |
| ~~B4~~ | ~~z 切片死配置~~ | **作废**，不是缺陷 |
| ~~B5~~ | ~~拍率统计两个窗口 bug~~ | **已完成并部署**：`enable` 漏清 `_tick_first_time`（65 拍报成 165Hz）+ 停用后报陈旧值。7 条回归测试 + 3 轮变异 |

### C 类：已测出数字、原因未查

| # | 事项 | 最新状态 |
|---|---|---|
| ~~C1~~ | ~~里程计静止漂移~~ | **实测 ≈0**（5 分钟净位移 **0.02mm**）。此前 0.32 / 0.55 / 0.0698 mm/s 三个数**全部作废** —— 那 4.4cm 是 13:44 与 13:52 两个孤立跳变（紧贴 SDK 会话建立），被我错误地平均成了连续速率 |
| ~~C2~~ | ~~`/cmd_vel` 6 个发布者~~ | **已查清**：`behavior_server`×4（spin/backup/drive_on_heading/wait 各一个）+ `cmd_vel_body_to_world_node`×2。不是 6 个节点抢发。**但"`wz` 在桥接入口达 1.5 而 smoother 出口 0.0124"仍未查** |
| C3 | `aft_mapped` vs `torso_base` 偏移 | z 已测（0.078m，2D 分解不受影响）；**xy 差 0.106m、yaw 差 22.9° 仍只是外参与 URDF 的纸面差，真实偏移未测** |
| C4 | SLAM 回环跳变 0.626m | 对导航的实际影响未评估 |
| ~~C5~~ | ~~`web_astribot_2135` 刷 46 万条 ERROR~~ | **已查清**：厂商 SDK 客户端心跳看门狗，说了 `Exiting...` 却没退出（计数冲过阈值 5 到 466770），13 小时以 10Hz 灌 `/rosout`。进程已消失。**功能无影响，但它骗过我一次**（据此误判"本体驱动没跑"） |
| C6 | `led_controller_node` 以 13.4Hz 刷 INFO | 占 `/rosout` 总量 91%，会淹掉真正的告警。排查须按节点名过滤 |
| C7 | **`name=` remap 把两个节点改成同名** | `~/` 对两者展开成同一前缀（`enable` 与 `dispatch_waypoints`/`set_gripper` 挤在一起）。这次没撞车，但以后同名服务会静默产生两个提供者。**这也是 yaml 全失效的同一根因** |
| C8 | status 话题的 `node_name` 是构造字面量 | 报 `chassis_cmd_bridge`，真实节点名是 `astribot_bridge_container`，按它查节点会查不到 |
| C9 | **机器人上 numpy 被顶到 1.24.0** | 项目钉的是 1.21.5，`test_numpy_pin_is_intact` 失败。运行环境已偏离钉住的版本 |

### D 类：已知阻塞、方案未定

| # | 事项 | 卡点 |
|---|---|---|
| D1 | `/scan` 无发布者 → **无动态避障** | Voxel-SLAM 要 `xfer_format=1`(CustomMsg)，感知预处理链要 `=2`(PointCloud2)，进程级互斥。四个候选方案均未定 |
| D2 | NoMachine 每次重启复发 | 根因推断是 1970 时钟污染内部状态库，未验证 |
| D3 | 厂商 SDK 心跳中断会一次性杀掉我们全部进程 | 2026-08-31 21:00:31 心跳断 1.24s，`bridge_container`+`state_bridge_node`+`chassis_odom` 三个同秒退出。**无自动重启机制** |

### E 类：仓库卫生

| # | 事项 |
|---|---|
| E1 | 3 处 `base_link` 默认值修复仍未提交，混在别人 12 个未提交文件里 |
| E2 | 别人的两笔提交 `删除实机底盘验证脚本：不再进行实机验证` 与本线工作冲突，需确认是谁的决定 |
| E3 | `nav2_params_slow.yaml` 在 install 目录，重建包会被覆盖 |
| E4 | 地面平面拟合未做 —— 基准高度只精确到 ±2cm（几何 0.017 vs 点云 −0.0038） |
| E5 | 本次全部改动**尚未 git commit**（含 `callback_layout.py` 等 3 个新文件） |
| E6 | 厂商 env 覆盖 `ros2cli` 扩展点，`ros2 topic/launch` 子命令消失。可用解法已记录：从活进程 `/proc/<pid>/environ` 导出环境 |
