# S1 实机验证链路启动手册

> 目标机 `astribot@10.249.22.137` · Jetson Orin aarch64 · ROS 2 Humble · `ROS_DOMAIN_ID=25`
> 工作区 `/home/astribot/Downloads/astribot_sdk_aarch64/ws_robot`
>
> 本手册覆盖两段:
> - **§0–§9 只读验证链路**:建图 + 定位 + 感知 + nav2 规划 + rviz 可视化,**不发任何速度**
> - **§10 使能与发速度**(2026-09-03 新增,已实机验证):写通路、SDK 控制权、
>   rviz 点目标让机器人真走。这一段每次执行都必须**逐次与现场人员确认朝向和距离**。
>
> §10 之前的所有内容都建立在"绝不碰运动"的前提上;进入 §10 等于放弃那个前提,
> 所以它单独成节,并在开头列出全部前置条件。

---

## 0 · 一页速查

```bash
# ── 只读链路（机器人上，按顺序）──
bash /tmp/bringup_stages.sh 1 2      # SLAM + 栅格投影
bash /tmp/bringup_stages.sh 3 4      # 状态桥 + 里程计（只读，会补 TF）
bash /tmp/bringup_stages.sh 7        # 动态避障链 → /scan
bash /tmp/bringup_stages.sh 5        # nav2（默认 rpp）
bash /tmp/start_rviz.sh              # 可视化（DISPLAY=:0）
bash /tmp/guard_readonly.sh &        # 可选：只读进程守护

# ── MPPI + 限速（替代上面的阶段⑤，见 §10.2）──
bash /tmp/start_nav2_mppi.sh         # MPPI + Omni + max_linear_speed + 姿态监控禁用

# ── 使能发速度（§10，需现场确认）──
bash /tmp/enable_write.sh            # 起桥接 + 发 yes 取控制权（**不使能**）
python3 /tmp/do_enable.py            # 调 ~/enable（这一步之后机器人会动）
```

**每一步之后都必须按拍率验收**,不能只看进程或发布者数 —— 见 [§4 判据](#4--判据为什么不能看进程和发布者数)。

**只读阶段绝不执行**:阶段⑥(`bringup_stages.sh` 里那个 `allow_write_to_real:=false`
的桥接)与 §10 的任何命令。

---

## 1 · 依赖顺序与它的理由

```
厂商雷达驱动（不是我们的）
  │ /livox/lidar_front, /livox/lidar_back   CustomMsg 10Hz
  │ /livox/imu_front                        200Hz
  ▼
① Voxel-SLAM（独立工作区 /home/astribot/SLAM/vxlm-slam）
  │ /map_scan_filtered  PointCloud2 10Hz
  │ TF camera_init → aft_mapped
  ▼
② cloud_to_grid_node          /map  OccupancyGrid（frame_id=camera_init）
  │
③ state_bridge_node（只读）    /joint_states 47Hz → robot_state_publisher → 各连杆 TF
④ chassis_odom_node（只读）    /odom 47Hz + TF odom→astribot_torso_base
   map_odom_tf_node            TF map→odom（REP-105 分解）+ map→camera_init（静态恒等）
  │
⑦ livox_custom_to_pc2_node    /livox/lidar_{front,back}_pc2   ← CustomMsg→PointCloud2
   livox_preprocess ×2         /livox/{left,right}/cloud_filtered
   livox_fusion_node           /livox/fused_points
   pointcloud_slice_scan_node  /livox/cloud_self_filtered（含本体自滤）
   pointcloud_to_laserscan     /scan  10Hz
  │
⑤ nav2                        global/local costmap + planner + controller
   rviz2                       可视化
```

### 顺序不能乱的三处

| 约束 | 违反后的表现 |
|---|---|
| ④ 必须在 `/odom` 就绪**之后**才起 `map_odom_tf` | `map_odom_tf` 的 60s 看门狗触发自杀,日志报"里程计侧 0" |
| ⑦ 必须在 ③ 之后 | 自滤要查 36 个连杆的 TF,`/joint_states` 没有就查不到 → 自滤剔除 0 点、机器人把自己的手臂当障碍 |
| ⑤ 必须在 ⑦ 之后 | nav2 两个 costmap 的 `obstacle_layer` **唯一**数据源是 `/scan`,没有它就完全没有动态避障,而 costmap 照发、判据照过 |

**本机器人没有 `base_link`。** 整机根坐标系是 `astribot_torso_base`。

---

## 2 · 环境:为什么不能直接 source 厂商 env.sh

启动脚本用的是 `/tmp/robot_env.sh` —— 从厂商进程 `environ` 导出的 26 个变量,**不是** `$SDK_ROOT/env.sh`。

原因:那个 `env.sh` 的第 6 节会因为本机是 `192.168.0.11`(而不是它判定"机器人端"的 `.10`)而**生成并导出一份 interfaceWhiteList 的 fastdds XML**,覆盖掉厂商的
`FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml`。

厂商的双网卡隔离用的是**多播路由 + iptables**,不是白名单。在上面再叠一层本项目已经吃过亏:话题能列出来但 pub 恒 0。

### 五个启动期环境坑(都已写进脚本)

| 症状 | 真因 |
|---|---|
| `Package 'voxel_slam' not found` | SLAM 是独立工作区,不在 `ws_robot` 下 |
| `libmetis-gtsam.so` 缺失(exit 127) | 自建 GTSAM 4.1.0 需**前置** `LD_LIBRARY_PATH` |
| `ModuleNotFoundError: astribot_sdk` | `PYTHONPATH` 缺 `$SDK_ROOT` |
| `ValueError: Invalid robot type` | 厂商 environ 里没有 `ROBOT_TYPE` |
| `TypeError: str expected, not NoneType` | 缺 `ASTRIBOT_SDK_ROOT`,**报错文本里不含任何变量名** |

### 厂商的 `ros2` CLI 是残缺的

`PATH` 里 `/opt/astribot_ros/middle_ware/bin` 排在 `/opt/ros/humble/bin` **前面**,而厂商那份 `ros2` 只有 `bag / daemon / node / param / service`:

```
$ ros2 topic list
ros2: error: argument ...: invalid choice: 'topic'
```

**这不是"话题不存在"。** 要用 `ros2 topic` 必须在 source 厂商 env 之后再 source `/opt/ros/humble/setup.bash`,或直接调 `/opt/ros/humble/bin/ros2`。本手册的探针都用 rclpy 脚本,绕开这个问题。

---

## 3 · SDK 控制权提示:会阻塞在 stdin

阶段 ③④⑥ 起的进程会连厂商 SDK,SDK 可能弹一个提示并**阻塞在 stdin 上等输入**:

```
Acquiring control will immediately stop the robot's current motion.
Enter 'yes' to forcibly acquire control, or press 'Enter' to continue without control rights
```

**只发回车,绝不发 `yes`。**

- 回车 = 继续但不取控制权(只读,验证链路要的就是这个)
- `yes` = 夺取控制权 **并立刻停止机器人当前运动** —— 这是运动性质的动作

脚本里的 `answer_sdk_prompt()` 只写 `printf '\n'`。可以自查:

```bash
grep -c "echo yes" /tmp/bringup_stages.sh    # 必须是 0
```

### 不应答的后果(实测)

状态桥曾卡在这个提示上约 **3 小时**,表现是一条完整的静默故障链:

```
/joint_states 0Hz → 无动态 TF → 自滤剔除 0 点 → /scan 一帧不出
```

而进程活着、发布者数是 1、判据看着都过。

---

## 4 · 判据:为什么不能看进程和发布者数

这一节是本手册最重要的部分。以下每一条都是实测踩过的假成功。

### 4.1 `count_publishers` 恒为真

链路上**每个节点启动时就会建发布者**,与它能不能出数据无关。实测过五个话题全是 `pub=1 / 0.0 Hz`。

> **中段一律按拍率验。**

### 4.2 costmap 有发布者 ≠ 能定位

曾出现六阶段全报 `[OK]`、脚本打印"全部阶段完成",而那一刻 `map→odom` 和 `map→camera_init` **都不存在** —— nav2 有 costmap 发布者,但根本无法定位。

判据必须真查 TF 边(脚本里的 `check_tf_edge map odom`),不能查 `/tf` 有没有发布者(那永远 >0)。

### 4.3 tf2 的陈旧陷阱

tf2 只在某 frame pair 有**新**数据时才修剪缓冲。停更的 pair 会**永久返回最后一条记录**,且 `lookup_transform(..., Time())` 一直"成功"、不报错。

所以 TF 查得到 ≠ TF 是新的。`map_odom_tf_node` 里的 A2 源龄期检查(`MAX_SOURCE_AGE=1.0s`)就是为此存在,它曾抓到一次真实的 SLAM 停更 1.66s 并正确拒绝发布。

### 4.4 nav2 的 `use_sim_time` 默认是 `'true'`

`navigation.launch.py` 的 `DeclareLaunchArgument` 默认 `'true'`,且 yaml 里硬写了 11 处 `use_sim_time: True`。

实机 `/clock` 发布者为 **0** → 六个 nav2 节点的时钟**恒为 0、永不前进**,而 costmap 照常发布、进程都活着、判据也过。

> **必须显式传 `use_sim_time:=false`。** 绕过 `nav2_full_bringup.launch.py` 直调内层 launch 就会中招。

### 4.5 数进程数的三个坑

**(a) 模式命中自己的命令行。** 同一条命令里既写模式又读 `ps`,`ps -eo args` 会把模式串抓进来。偏差方向**永远只会高**,于是"0 个进程"被读成"1 个,已在跑"。

正确做法是分两步:

```bash
# 第一步：把模式写进文件
printf '%s\n' state_bridge_node chassis_odom_node > /tmp/pat.txt
# 第二步：另一次调用里先落进程快照，再用快照计数
ps -eo pid,args > /tmp/snap.txt
while read p; do echo "$p $(grep -c -- "$p" /tmp/snap.txt)"; done < /tmp/pat.txt
```

**(b) 调用方的 argv 也算。** 即使先落快照,只要**外层 shell** 的命令行含模式串就照样中招。实测 `proc_count 'zzz_definitely_not_running_zzz'` 对一个根本不存在的进程返回 **1**。`tools/robot/guard_readonly.sh` 里的 `proc_count()` 显式排除了本进程、父进程、含脚本名的进程和 grep/ps 自身。

**(c) 模式在一行里出现多次。** `grep -c` 数的是**行数**,但一个进程的 args 里模式出现几次不影响;真正的问题是**别的进程的参数**里含该模式。实测 `robot_state_publisher` 数到 3,其中一条是状态桥的参数 `use_robot_state_publisher:=false` —— 差点被误报成"有重复 RSP"。

> **数出异常之后,先看真实命令行(`cut -c1-140`),再下结论。**

### 4.6 大批 kill 之后 DDS 发现要重新收敛

停掉十几个进程后,新起的探针进程要很久才能发现完整的图。实测:

```
t=10s  topics=164  nodes=14
t=25s  topics=164  nodes=20
t=45s  topics=164  nodes=31     ← 正常状态是 85
```

**这期间任何"话题全 DEAD"的读数都不可信。** 一次 45 秒窗口的扫描报了包括 SLAM 和状态桥在内的"全部 DEAD",而那两个我根本没碰、进程表里活得好好的。改成 150 秒窗口后全部回来。

> 判据窗口:平时 20~30s 够;**大批重启后给 120s 以上**。
> 拿不准时用进程表交叉验证 —— 它是地面真相,不受 DDS 影响。

### 4.7 相等性断言在"两边同时失效"时会假通过

一条测试比较两个位移是否相等,两者都被压成 `0.0` 时 `0 == 0` 成立、测试通过。真正抓到问题的是比**绝对值**的那条。

> 判据尽量压在绝对值/物理量上,不要只比两个量是否相等。

---

## 5 · 逐阶段启动与验收

### 阶段 ①② SLAM + 栅格

```bash
bash /tmp/bringup_stages.sh 1 2
```

| 判据 | 期望 |
|---|---|
| `/map_scan_filtered` | **≈10 Hz** |
| TF `camera_init → aft_mapped` | 可解且**在更新** |
| `/map` | 0.476 Hz —— **不是故障**,节点主动限流(收 200 帧投影 10 帧) |
| `cloud_to_grid` 日志 | `雕刻失败=0`、`切片外=0`、单帧耗时 ≈0.03s |

**SLAM 基准高度**:`camera_init` 原点离地 **0.095 m**,不是 0。世界系锚在底盘中心。
不要拿 `tf2_echo` 读到的 z 当"实测" —— TF 只是把 URDF 里的值原样发出来,
**量一个由错值生成的量不构成对那个值的验证**。

### 阶段 ③④ 状态桥 + 里程计(只读)

```bash
bash /tmp/bringup_stages.sh 3 4
```

| 判据 | 期望 |
|---|---|
| `/joint_states` | **≈47 Hz**,22 个关节(躯干4 + 双臂7×2 + 夹爪主动1×2 + 头2) |
| `/odom` | **≈47 Hz** |
| TF `map→odom` | 可解 |
| `chassis_odom` 日志 | 静止时 `行程=0.000m 跳变=0` |

**`/joint_states` 只有 22 个关节是正常的。** URDF 有 36 个可动关节:22 个来自 SDK,
10 个夹爪从动关节由 RSP 按 mimic 关系算出,**剩下 4 个轮关节没有任何数据源** ——
SDK 的 `whole_body` 不含底盘(底盘是 3 个虚拟关节 `chassis_x/y/z_rot`),轮子自转角厂商侧从来没有这个量。

后果:rviz 的 RobotModel 会显示 4 条红色 `No transform from [wheel_*_Link]`。
**这是已知的、当前为纯显示性质的问题**,自滤链和 nav2 足迹都不依赖这 4 个 link。
(未验证:`move_group` 可能因缺这 4 个 active 关节而拒绝规划。)

### 阶段 ⑦ 动态避障链

```bash
bash /tmp/bringup_stages.sh 7
```

| 判据 | 期望 |
|---|---|
| `/livox/lidar_{front,back}_pc2` | **≈10 Hz**,空回波剔除 33%/35% |
| `/livox/{left,right}/cloud_filtered` | ≥5 Hz |
| `/livox/fused_points` | **≈10 Hz** |
| `/livox/cloud_self_filtered` | **≈10 Hz** |
| **`/scan`** | **≈10 Hz** |
| 切片节点日志 | `自身剔除 87~99 点/帧` ← **不是 0** |
| 切片节点日志 | `连杆 TF 查询失败: 0` |

**`自身剔除 0` 是红旗**,说明自滤没生效、点云里含机器人自身结构。

**空回波假点团**:无回波点被雷达设备的外参一起变换,堆成固定坐标的密集点团 ——
front 在 `(0,0,0)`,back 在 `(0.001,−0.496,0.084)`。back 那团距原点 **0.4968 m**,
而 `range_min=0.35`,**球面距离门挡不住它**,只能按"等于外参平移量"这个特征剔除。
转换节点做的就是这件事,输出里这两个坐标附近的点数必须为 **0**。

### 阶段 ⑤ nav2

```bash
bash /tmp/bringup_stages.sh 5
```

| 判据 | 期望 |
|---|---|
| 六个节点的 `use_sim_time` | 全为 **False** |
| 时钟 | 真的在**前进**(不是恒为 0) |
| TF `map→odom` | 可解 |
| `/global_costmap/costmap` `/local_costmap/costmap` | 有数据 |
| `obstacle_layer.scan.expected_update_rate` | **0.2**(不是 0.0) |

`controller_plugin` 默认 `rpp`,加载 `nav2_params_rpp.yaml`。

### rviz 可视化

```bash
bash /tmp/start_rviz.sh
```

三个坑:

**(a) locale。** ssh 会把客户端的 `LC_*` 转发过去,机器人上没有 `zh_CN.UTF-8`,
Qt 直接抛 `locale::facet::_S_create_c_locale name not valid`。启动脚本必须
`unset LANGUAGE LC_*` 并 `export LC_ALL=C.UTF-8`。

**(b) 必须直接 GLX。** `ssh -X` 走不通(Ogre 建不出转发 display 上的 GL 窗口,
100 次重试后 core dump),offscreen 也救不了。可用路径:NoMachine 连物理桌面 `:0`,
或 x11vnc 只绑回环 + ssh 隧道。

**(c) 配置里不能有能让机器人动的工具。** 校验时**先剥注释再 grep**,否则会命中说明文字:

```bash
sed 's/#.*//' <rviz配置> | grep -cE "SetGoal|goal_pose|PublishPoint"   # 期望 0
```

`nav2_view.rviz` 的 `Tools:` 段只有 `Interact/MoveCamera/Select/SetInitialPose`。
另注意 `setup.py` 只 glob `rviz/*.rviz`,放在 `config/` 下的 `.rviz` 永远不会被安装。

**(d) rviz 自己的 QoS。** rviz 对 `/scan` `/odom` 的订阅默认是 `RELIABLE`,
而这两个话题的发布端是 `BEST_EFFORT` → **rviz 一帧都收不到**,日志里只有一条
`incompatible QoS` WARNING。`/map` 双侧都是 `RELIABLE+TRANSIENT_LOCAL`,所以地图是真的。
要在 rviz 里看到激光,需要在显示项里把 Reliability 显式改成 Best Effort。

---

## 6 · 感知陈旧的三道防线(2026-09-02 新增)

起因是 2026-09-01 21:26 按下物理急停后实测到的一条链路:

```
急停 → 厂商控制驱动停 → 三个持有 SDK 会话的进程在同一秒退出（fail-fast，正确）
  → /joint_states 0Hz → robot_state_publisher 停发连杆 TF
    → 自滤逐连杆等满 TF 超时 → /scan 掉到 0.56Hz、数据龄期 2.03s
      → nav2 照常规划、照常发速度，全程零告警
```

算术闭合:36 个连杆 × 50ms = 1800ms,实测自滤级增量 1910ms(94.2% 是纯等待),
单帧 1.91s → 吞吐 1/1.91 = 0.52Hz,实测 `/scan` 0.56Hz。

| 防线 | 参数 | 作用 | 边界 |
|---|---|---|---|
| ① TF 总预算 | `tf_total_budget_sec: 0.06` | TF 全断时自滤开销从 1800ms 压到 60ms | 健康态完全不改变行为 |
| ② `hold_last` 上限 | `hold_last_max_frames: 5` | 限制"旧几何 + 新时间戳"的最长隐身时间为 0.5s | 超过转停止输出 |
| ③ costmap 陈旧告警 | `expected_update_rate: 0.2` | 陈旧时打告警 | **只是告警,不是联锁** |
| ④ 写通路联锁 | `scan_max_age_sec: 0.5` `scan_loss_grace_sec: 2.0` | 超阈置零 / 持续超宽限闩锁 | 盲走 0.5s×速度 ≈ 0.25m |

### 为什么 ③ 只能是告警

已确认 `libnav2_costmap_2d_core.so` 里有对应告警字符串,但
`controller_server` 二进制里**没有任何检查 costmap currency 的字符串** ——
陈旧时它仍会继续发速度。真正能拒绝下发的只有 ④。

### 为什么 ② 不可省

`republishLastScan()` 会把上一帧的几何**配上 `now()` 的新时间戳**发出去。
没有上限时,陈旧对一切时效性判据隐身:龄期探针读到 ~0ms、`expected_update_rate`
看到缓冲一直在更新。**真实故障下它比"不发"更危险** —— 不发能被发现,
发一份戴着新时间戳的旧数据不能。

### ④ 的可观测状态

| 事件码 | 含义 |
|---|---|
| `SCAN_STALE` | 龄期超阈,本拍速度已置零(可自动恢复) |
| `SCAN_LOST_STOPPED` | 持续陈旧超宽限,已闩锁 `STOPPED_STALE_SCAN`,需 `~/enable` 复位 |
| `SCAN_NEVER_RECEIVED` | 从未收到 `/scan`。**先查话题名与 QoS** |

`SCAN_NEVER_RECEIVED` 单独设一个码,是因为"收不到"与"变旧了"排查方向完全不同:
BEST_EFFORT 发布 + RELIABLE 订阅会一帧都收不到且只有一条 WARNING。

> ⚠️ 联锁默认开启。**重启 `bridge_container` 之前 `/scan` 必须是通的**,
> 否则它会拒绝下发,表现为"使能了但不动",事件是 `SCAN_NEVER_RECEIVED`。
> 台架无雷达时用 `require_fresh_scan:=false` 显式关闭(会打 WARNING)。

---

## 7 · 只读进程守护(可选)

```bash
bash /tmp/guard_readonly.sh &
tail -f /tmp/bringup/guard.log
```

守护 `state_bridge_node` 和 `chassis_odom_node` 两个**只读**进程。

**写通路是结构性排除的**,不是"忘了加":白名单里出现
`bridge_container` / `arm_traj_bridge` / `chassis_cmd_bridge` / `gripper`
任何一个,脚本启动期直接 `exit 2`。

理由:急停之后让写通路自动回来,等于**绕过人按急停的意图**。
恢复写通路必须人工确认后单独执行阶段⑥。

两个设计点:

- **重启前先查厂商控制驱动**,判据是 `/astribot_error_code/control_driver` 发布者 >0
  而不是进程在不在(实测过"进程活着而话题 pub 恒 0")。驱动没回来时重启必然
  1.2s 内再死,只会把真实故障刷成一片重启日志
- **计"窗口内"重启次数**(600s 内 5 次),不是总次数。用总次数的话跑一天累计到上限
  就永久放弃,之后每次真实故障都不再自愈

---

## 8 · 故障速查

### `/scan` 是 0 Hz

按依赖链**从上游往下**查,第一个断掉的地方就是根因:

```bash
# 厂商雷达（最上游，不是我们的代码）
/livox/lidar_front   应 10Hz
/livox/imu_front     应 200Hz
```

**厂商雷达可能进程活着但不出数据。** 识别方法:看转换节点的日志,它自带龄期:

```
/livox/lidar_front 已 8513.70s 没有新帧（上限 2.00s）——
  这一路停了，上面那些计数是历史值，不是当前速率。
```

这条日志是刻意这么写的:累计计数在数据停止后**看起来仍然很正常**,
必须同时给出龄期才不会把历史值读成当前速率。

停止前的征兆(实测):SLAM 日志出现 IMU/雷达时间戳发散
`IMU/LiDAR timestamp gap large | diff_ms:-1030.7`,指向时间同步。

### 三个 SDK 进程同时退出

```
Driver heartbeat timeout detected! Count: 5/5
Driver crashes. Reached timeout threshold (5). Exiting...
```

三者引用**同一个** `Last error_code_timestamp` = 同一个上游事件。

**最先排除:物理急停是否处于按下状态。** 按下急停后厂商控制驱动停止是
**完全正常**的表现,我们的进程退出是正确的 fail-fast。

判据(不是看进程):

```
/astribot_error_code/control_driver      发布者 > 0
/astribot_*/joint_space_states（6 个）    发布者 > 0
```

分层形状能区分故障类型:

| 层 | 急停时 |
|---|---|
| 设备层 `error_code/device_driver` | 仍活 |
| 末端状态 `endpoint_current_states` | 仍活 |
| **控制驱动 / `joint_space_states`** | **消失** |

设备层活着而只有运动控制层消失 → 安全停机,不是崩溃。

**控制驱动没恢复之前重启我们的进程毫无意义**,会在 1.2s 内以同样原因再退出。

### `map_odom_tf` 反复自杀

它有 60s 看门狗。八成是启动顺序错了 —— 它必须在 `/odom` 就绪**之后**才起。
它自己的日志是准确的(会报"里程计侧 0"),错的是调用它的时机。

### 探针报"话题全 DEAD"

**先查仪器,再查系统。** 见 [§4.6](#46--大批-kill-之后-dds-发现要重新收敛)。
进程表是地面真相。

---

## 9 · 本手册未覆盖 / 未验证

### 已在 2026-09-03 验掉的(从本表移出)

| 项 | 结论 |
|---|---|
| 使能写通路、发底盘速度 | **已实机验证,见 §10**。到位误差 0.092m,安全机制零触发 |
| rviz 点目标 → 规划 → MPPI 跟踪 → 机器人实走 | **全链打通**,实测走行 1.8556m |

### 仍未覆盖

| 项 | 状态 |
|---|---|
| **自主探索** | **被阻塞** —— `/map_scan_filtered` 无未知区,没有前沿可找。见 §11 |
| 健康态下 `/scan` 的端到端龄期 | **未测**(降级态实测 2.03s) |
| 新节点从启动到 DDS 发现的耗时 | **未测准**。手册里用的 45s 窗口是沿用旧说法,不是本机实测值;唯一一次尝试因 `/tmp` 被重启清空、探针落在 domain 0 而作废 |
| 桥接钳位的成因 | 只能推断。日志只打钳位**次数**,不打被钳的量与是哪个分量 —— 真实可观测性缺口 |
| 臂-底盘耦合限速系数 | 2026-09-02 见过 `限速系数=0.15`(监控连杆 TF 查不到 → 按最保守取 1.0 活跃度)。**根因未查清**;若真跑时复现,`0.1 × 0.15 = 0.015 m/s`,走 1m 要 67s |
| `move_group` 是否因缺 4 个轮关节而拒绝规划 | 机制清楚,**未测** |
| `/livox/right/cloud_filtered` 比 left 慢约 20% | **原因未查** |
| rviz Navigation 2 面板的 `Feedback: aborted` 来源 | **未查** |
| Fast DDS endpoint 级发现失效 | 已记录,**未解决** |
| 厂商雷达为何会进程活着但停止出数据 | **未查清**,疑似时间同步 |
| `wz_max` 的现场标定 | `max_linear_speed` 刻意不管角速度,仍是 yaml 的 2.0 rad/s(≈115°/s),**未在现场标定过** |

---

## 10 · 使能与发速度(2026-09-03 实机验证)

> **这一节放弃了 §0–§9 的"绝不碰运动"前提。** 执行之前必须满足全部前置条件,
> 并逐次与现场人员确认朝向和距离 —— 技术前置条件(电量、`/cmd_vel` 无帧)
> **不构成授权**。

### 10.1 前置条件(缺一项都不要往下走)

| 条件 | 怎么验 | 实测基线 |
|---|---|---|
| §0–§9 只读链路全绿 | `probe_sweep.py` 按拍率 | `/scan` 9.9~10.0Hz、`/tf` 93~96Hz、`/odom` ~50Hz |
| nav2 **7 个节点 ACTIVE** | `probe_nav2_lifecycle.py` | 见 §10.5,**进程数判据无效** |
| `/cmd_vel` 当前无人发 | 订阅数帧,不是查发布者数 | 无目标时应为 0 帧 |
| 现场有人手持急停 | 口头确认 | —— |
| 已确认本次的朝向与距离 | 口头确认 | —— |

### 10.2 起 nav2:MPPI + 限速

只读链路默认用 `rpp`(`RegulatedPurePursuitController`)—— 它是纯路径跟踪器,
**只会停、不会绕**。要真正利用全向底盘做避障必须用 MPPI:

```bash
bash /tmp/start_nav2_mppi.sh
```

它显式传四个参数,每一个都对应一次实测踩过的坑:

| 参数 | 不传的后果 |
|---|---|
| `use_sim_time:=false` | launch 默认 `true`,yaml 里也硬写多处 `True`。实机 `/clock` 发布者为 0 → 六个节点时钟**恒 0、永不前进**,而 costmap 照发、判据照过 |
| `controller_plugin:=mppi` | 默认 `rpp`,退化成"遇障就停" |
| `enable_posture_monitor:=false` | 见 §10.3 |
| `max_linear_speed:=0.1` | 见 §10.4 |

### 10.3 姿态监控必须在实机显式禁用

`cmd_vel_body_to_world_node` 带一个姿态止损监控,判据是
`|z - normal_height| > max_height_deviation` 或 roll/pitch 超 `max_tilt_rad`。

**`normal_height=0.134` 是仿真值**(Gazebo 里 world z=0 不是地面)。
实机 `/odom` 是轮式里程计、只暴露 3-DOF,**z/roll/pitch 恒等于 0**
(2026-09-02 实测 509 帧 min=max=0.0000)→ `|0-0.134|=0.134 > 0.06`
→ 判为异常姿态 → **永久**把 `/cmd_vel` 归零(`safety_tripped` 无复位路径)。

此前之所以没炸:该节点的 `/odom` 订阅原是默认 **RELIABLE**,而实机 `/odom`
发布者是 **BEST_EFFORT**,回调**一帧都没执行过**(实测 RELIABLE 0 帧 /
BEST_EFFORT 704 帧@50Hz)—— **两个缺陷互相掩盖**。QoS 已修成 `sensor_data`,
所以这个开关是必需的。

节点启动时会自述状态,日志里应看到:

```
姿态监控**已显式禁用** —— 实机 /odom 是轮式里程计(3-DOF)，z/roll/pitch 恒为 0 ...
```

**边界**:实机上把它设成 `true` 并把 `normal_height` 改成 0.0 只是让它永不触发,
**检测不出真的倾倒** —— 那是假的安全感。真要做倾倒检测得换 IMU
(`/astribot_whole_body/chassis_imu`、`/livox/imu_{front,back}`)。

### 10.4 限速:必须同时压四个量

`max_linear_speed:=<v>` 一键压住 MPPI 的 `vx_max` / `vy_max` / `vx_min`。

**只压 `vx_max` 是漏洞**:`vx_min` 仍是负的满量程,MPPI 可以全速**倒车**。

`velocity_smoother` 的 `max_velocity`/`min_velocity` 是**串联的第二层**限速,
刻意**不**在 launch 里重写 —— 它们是 double 数组,而 `RewrittenYaml.convert()`
只尝试 int/float/bool(已读 `/opt/ros/humble` 下实现确认),写进去会变成**字符串**,
`velocity_smoother` 配置期抛 `invalid type`,然后 `lifecycle_manager`
`Aborting bringup`。**那次失败时 9 个进程全都活着、进程数判据照过。**

它保持 yaml 的 1.0,是 MPPI 输出的**上界**;两层串联取 `min`,所以安全侧成立。

实测生效值(读活节点,不是读 yaml)。⚠️ **键名带 `.inner`**:2026-09-07 起
`FollowPath` 是 `ThreePhaseController`,MPPI 的限速参数由
`inner_->configure(parent, name_ + ".inner", ...)` 配到内层命名空间,
`FollowPath.vx_max` 这个键**不存在**。查参一律**枚举** `*.vx_max`,
不要按名字硬查(硬查的现象是"读不到"、看着像限速失效):

```
FollowPath.inner.vx_max = 0.1   vx_min = -0.1   vy_max = 0.1
FollowPath.inner.plugin = nav2_mppi_controller::MPPIController
FollowPath.inner.motion_model = Omni
```

⚠️ **`wz_max` 不在这个开关的管辖内**,仍是 yaml 的 2.0 rad/s(≈115°/s)。
刻意如此:静默改角速度会让原地对齐段和 Spin 恢复行为跟着变。

### 10.5 判据:nav2 必须查**生命周期状态**,不是进程数

```bash
python3 /tmp/probe_nav2_lifecycle.py
```

期望:

```
controller_server  ACTIVE      bt_navigator       ACTIVE
smoother_server    ACTIVE      waypoint_follower  ACTIVE
planner_server     ACTIVE      velocity_smoother  ACTIVE
behavior_server    ACTIVE
LIFECYCLE_OK  7 个节点全部 ACTIVE
```

**为什么进程数不能用**(2026-09-02 实测):参数写错时
`velocity_smoother` 配置期失败、`lifecycle_manager` 报
`Failed to bring up all requested nodes. Aborting bringup.`,
结果**整套 nav2 一个节点都没 activate**,而:

- 9 个进程全都活着
- "nav2 进程数 >= 5" 的判据照过
- 那几行 ERROR 混在一堆 TF WARN 中间

只有参数服务不可达才暴露出来。**进程数在这种失败上完全无效。**

### 10.6 起写通路 + 取 SDK 控制权

```bash
bash /tmp/enable_write.sh
```

这一步做两件事,**不使能、不发速度**:

1. 以 `allow_write_to_real:=true` 起 `bridge_container`
2. 向它 stdin 发 **`yes`** 夺取控制权

⚠️ **发 `yes` 之前必须让现场人员知道**:提示原文是
`Acquiring control will immediately stop the robot's current motion`。
`yes` = 夺权 + **立刻停止机器人当前运动**。机器人静止时无实际影响,但这是
"使能"性质的动作。只读链路里的 `answer_sdk_prompt` 刻意**只发回车**,与此相反。

期望日志:

```
已确认它停在控制权提示上
[OK] 写通路 = 允许
桥接已起、控制权已取。**但还没有使能，也没有发任何速度。**
```

桥接自述里核对安全参数:

```
leash=(0.250m, 0.350rad)  闭环=False  写通路=允许。启动即停用，需调 ~/enable。
```

还要确认 `/scan` 联锁**开着** —— 日志里 `require_fresh_scan=False` 的出现次数必须是 **0**。

**`enable_slam_correction:=false`(纯开环 + leash)的理由**:开着校正会让外环往指令里
注入修正量,测出来的位移/转角就不再只反映所发指令。leash 判据是
`pos_cmd vs SDK 实际位置`,与校正开关无关,所以硬保护没被关掉。

### 10.7 使能

```bash
python3 /tmp/do_enable.py
```

它调 `~/enable` 并做**静止自检**:使能后若没有导航目标,机器人必须完全不动。

```
使能成功：enabled
使能后静止自检（10s）...
  /cmd_vel  0 帧  最大幅值 0.0000
  位移 0.0001 m   转角 0.0001 rad
ENABLE_OK  已使能，当前无目标、保持静止。
```

⚠️ **使能前若 `/cmd_vel` 上已有帧,机器人会在使能瞬间立刻按那些指令动起来** ——
不需要点任何目标。脚本会把帧数和最大幅值报出来。常见来源是**上一个导航目标还在执行**
(`bt_navigator` 未收到取消),此时应先让它结束或取消,再使能。

### 10.8 点目标 → 验收

在 rviz 里用工具栏的 **Nav2 Goal** 在地图上点目标。

`nav2_view.rviz` 已含 `nav2_rviz_plugins/GoalTool`。
⚠️ 即使把 `GoalTool` 从 `Tools:` 段删掉,**`nav2_rviz_plugins/Navigation 2` 面板
自己也带一个 `/goal_pose` 发布者** —— 想做"绝对点不出目标"的只读配置时,
删工具栏是不够的(2026-09-02 我据此得出过错误结论)。

**验收必须量实际位姿,不能只看 nav2 说"到位"**:

```bash
# 机器人 map 系位姿 vs 目标坐标，直接相减
python3 - <<'PY'
import math, time, rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("acc")
b = Buffer(); TransformListener(b, n, spin_thread=True)
t0 = time.time()
while time.time() - t0 < 45: rclpy.spin_once(n, timeout_sec=0.05)
tr = b.lookup_transform("map", "astribot_torso_base", rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=5.0))
x, y = tr.transform.translation.x, tr.transform.translation.y
gx, gy = 1.90, -0.04          # ← 换成日志里 "Begin navigating ... to (gx, gy)" 的值
print("实际(map) %.3f, %.3f   目标 %.3f, %.3f" % (x, y, gx, gy))
print("到位误差  %.3f m   (xy_goal_tolerance=0.18)" % math.hypot(x-gx, y-gy))
rclpy.shutdown()
PY
```

⚠️ **必须在 map 系里比。** `/odom` 是 odom 系、目标是 map 系,直接相减会造出假误差 ——
2026-09-03 我拿 `/odom` 的 `y=0.3863` 减目标的 `y=-0.04`,算出 0.43m 横向偏差,
还据此说"和 `Reached the goal!` 矛盾";而 map 系里 `y` 只有 `-0.052`,真实误差 **0.092m**。

**2026-09-03 实测基线**(目标正前方约 1.9m,`vx_max=0.1`):

| 项 | 实测 |
|---|---|
| 到位误差 | **0.092 m**(`dx=-0.092` `dy=-0.012`),容差 0.18 ✓ |
| 全局路径 | `/plan` 36 个路径点、长 1.754m |
| MPPI 输出 | `/cmd_vel_nav_body_raw` 172 帧、最大幅值 0.2215 |
| 重规划 | `Passing new path` ×6,间隔 1.03~1.05s = 默认 BT 的 1Hz `RateController` |
| 桥接内环 | 211~214Hz(标称 250 的 85%),钳位 5 次/38226 拍 |
| leash / `/scan` 联锁 | **零触发** |
| 到位后漂移 | `dx/dy/dyaw` 全 0.0000,`/cmd_vel` 归零 |

**钳位那 5 次**全部发生在目标结束**之后**、且随后不再增长 —— 与"看门狗归零 + slew"
的时间尺度一致。这是**推断**:桥接只打钳位次数,不打被钳的量,现有日志无法直接证实。

### 10.9 收工

```bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
rclpy.init(); n = Node("dis")
c = n.create_client(Trigger, "/astribot_bridge_container/disable")
c.wait_for_service(timeout_sec=15.0)
f = c.call_async(Trigger.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=15.0)
print(f.result().message if f.result() else "超时")
rclpy.shutdown()
PY
```

**若做过临时配置改动(例如缩小 `footprint`),必须在收工时还原。**
2026-09-02 缩到外接 0.15m(真实 0.42m 的 36%)的那份改动**活过了机器重启**
—— 它改的是 `install/` 下的实体文件,而 `/tmp` 里的备份随重启清空了。

还原(`src` 是真值):

```bash
W=/home/astribot/Downloads/astribot_sdk_aarch64/ws_robot
cp $W/src/astribot_s1_navigation/config/nav2_params_mppi.yaml \
   $W/install/astribot_s1_navigation/share/astribot_s1_navigation/config/nav2_params_mppi.yaml
md5sum $W/src/.../nav2_params_mppi.yaml $W/install/.../nav2_params_mppi.yaml   # 两行必须相同
```

⚠️ `colcon build` **不一定会覆盖** install 下的配置(实测跳过了),所以用 `cp` 并核对 md5。

## 11 · `/map_scan_filtered` 已变成 OccupancyGrid(2026-09-03,阻塞探索)

厂商在 SLAM 侧改了接口:`/map_scan_filtered` 从 `sensor_msgs/PointCloud2`
改成了 **`nav_msgs/OccupancyGrid`**(其余 7 个 `/map_*` 话题仍是 PointCloud2)。

### 11.1 实测契约

| 项 | 实测值 | 对 nav2 `/map` 的契约 |
|---|---|---|
| 类型 | `nav_msgs/msg/OccupancyGrid` | ✓ |
| 取值 | 只有 `0`(97.83%) 与 `100`(2.17%) | ✗ **未知区 `-1` 为 0 格** |
| `frame_id` | `camera_init` | ✗ nav2 的 `global_frame` 是 `map` |
| QoS | RELIABLE + **VOLATILE** | ✗ 静态层按 TRANSIENT_LOCAL 订阅 → 零消息 |
| 分辨率 | 0.05 m | ✓ 与 costmap 一致 |
| 更新率 | 10.01 Hz,间隔 0.096~0.104s | ✓ |

QoS 那一项是实测出来的,不是推演:同一话题上
**TRANSIENT_LOCAL 订阅 0 帧 / VOLATILE 订阅 451 帧@10.01Hz**。

### 11.2 "未知区为 0"是最严重的一项

整图 97.83% 标成"自由",**包括从未扫过的区域**。后果:

- `allow_unknown: false` 这条保护**完全失效** —— 规划器会穿过没扫过的地方
- **前沿(frontier)的定义是"自由区与未知区的边界"** → 没有未知区就没有前沿
  → 自主探索立刻判"无前沿 = 探索完成"

**所以在厂商补上 `-1` 之前,自主探索验证无法进行。**

### 11.3 连带影响:`cloud_to_grid_node` 已收不到数据

`cloud_to_grid_params.yaml` 的 `cloud_topic` 仍指向 `/map_scan_filtered`(两处),
而该话题已不是点云 → **类型不匹配 → 该节点现在一帧都收不到**。

它原本的存在前提是"Voxel-SLAM 只出 PointCloud2、`OccupancyGrid` 引用数 0",
这个前提已被打破,所以定位需要重新判定,而不是把它的订阅类型改一改。

### 11.4 改用 `/map_scan` 的可行性(已实测,**待厂商侧决定后再动**)

`/map_scan` 仍是 `PointCloud2`、10.00Hz、4510 点/帧、`frame=camera_init`、
字段 `x/y/z/intensity/normal_*/curvature` 齐全 —— 技术上可用。

但它**没有** SLAM 侧那层高度 ROI,而当前 yaml 的 z 切片是"放通"
(因为原本依赖上游已过滤)。实测 `/map_scan` 的 z 分布:

```
大量点集中在 2.6 ~ 4.3 m（天花板/高处结构），最远 29.6 m
```

候选切片的保留率(175895 点样本):

| 切片 | 保留 | 说明 |
|---|---|---|
| `[0.05, 1.63]` | 53.67% | SLAM 侧原本的 ROI |
| 放通 `(-inf, inf)` | 100% | 当前 yaml 默认 —— **会把天花板投影成地面障碍** |
| `[0.05, 2.00]` | 56.22% | 放宽上限 |
| `[-0.05, 0.60]` | 38.04% | 旧默认(注释记载会丢 60% 障碍点) |

所以改用 `/map_scan` **必须同时**把 z 切片补成 `[0.05, 1.63]`。

### 11.5 需要厂商侧补的三项

1. **未知区 `-1`** —— 未扫描区域不能标成自由(否则探索不可用)
2. **`frame_id` 改 `map`** —— 或补一条 `map → camera_init` 静态 TF
3. **QoS 改 TRANSIENT_LOCAL** —— 否则 nav2 静态层零消息

**当前决定(2026-09-03):等厂商侧修改,我们不改代码。**

## 12 · MPPI 限速 × 探索到位精度 扫描评测(2026-08-27 新增工具)

跟随自主探索运行,逐轮记录 map 系指标,跨限速档位汇总对比。

```
scripts/run_speed_sweep.sh            扫描驱动(逐档重启 nav2 + 起录制器)
explore_metrics_recorder_node         录制器（只读，零发布）
scripts/aggregate_speed_sweep.py      跨档位汇总 -> sweep.md / sweep.csv
astribot_s1_navigation/explore_metrics/   全部判据(纯函数,192 条单测)
```

### 12.1 两个 launch 漏项(已修,但必须知道为什么)

`nav2_full_bringup.launch.py` 的 `launch_arguments` 是**白名单**:没列进去的名字
不会传到子 launch,而子 launch 的 `default_value` 照常生效。此前漏了两项:

| 漏项 | 症状 | 后果 |
|---|---|---|
| `max_linear_speed` | 不报错、不告警 | `max_linear_speed:=0.2` 下 `vx_max` 仍是 **1.0**,整轮扫描产出若干档位数字**完全相同**的表,而每行看起来都正常 |
| `enable_posture_monitor` | 同上 | 实机上**根本没办法从这个入口关掉它**,而它会**永久**把 `/cmd_vel` 归零(见 §10.3) |

`enable_posture_monitor` 的外层默认值现在**跟着 `env` 走**:
`env:=sim → true`(仿真里 `/odom` 有真的 z/roll/pitch)、`env:=real → false`。
两条由 `TestSpeedCapForwarding` 钉住,4 个变异全部会被抓住。

### 12.2 用法

```bash
# 只观测（默认）：不使能写通路、机器人不动。用来验证录制管线通不通
CAPS="0.1 0.2 0.4" MINUTES_PER_CAP=10 \
  ros2 run astribot_s1_navigation run_speed_sweep.sh

# 真跑：**每个档位都会停下来要求输入 YES**
ENABLE_WRITE=true OBSERVE_ONLY=false CAPS="0.1 0.2 0.4" \
  ros2 run astribot_s1_navigation run_speed_sweep.sh
```

| 环境变量 | 默认 | 说明 |
|---|---|---|
| `CAPS` | `0.1 0.2 0.4` | 限速档位,m/s |
| `MINUTES_PER_CAP` | `10` | 每档时长 |
| `ROUNDS_PER_CAP` | `0` | >0 时按轮数收工,优先于时长 |
| `OBSERVE_ONLY` | `true` | **默认只观测,不使能写通路** |
| `ENABLE_WRITE` | `false` | 与上一项都要显式改才会真跑 |
| `LOW_OBSTACLE_TRUTH` | 空 | 低矮障碍人工真值 yaml,不给则该项记 `n/a` |
| `COLLISIONS_MANUAL` | `-1` | 真实碰撞次数,人工填(无碰撞传感器) |

### 12.3 起停判据(和 §4 同一条原则)

- nav2 就绪 = **生命周期 ACTIVE**,不数进程(实测 9 进程全活而 `Aborting bringup`)
- 起档位后**先核对在线查参的限速 == 请求值**,不符立即停、不采数。
  查法是**枚举** `ros2 param list /controller_server | grep '\.vx_max$'` 后逐个查,
  **不要**硬编码 `FollowPath.vx_max` —— 那个键在三段式控制器下不存在,
  硬查的现象是"读不到限速",与"限速失效"完全同形。
  枚举必须带"一个键都没枚举到就算失败"的守卫:0 个键时"全部一致"是空真。
- 录制器的就绪自检按**实测消息数**,不按"有没有发布者"
  (实测过 `pub=1` 但 `0Hz`:BEST_EFFORT 发布者 + RELIABLE 订阅者)
- 零消息时它会打印上游话题的**实际类型与 QoS**,不用靠猜

### 12.4 输出

```
<OUT_ROOT>/capXX/capXX_rounds.csv     一轮一行，120 列
<OUT_ROOT>/capXX/capXX_run.json       实测限速、足迹半径、时钟源、各话题消息数、
                                      阈值、**代理量定义原文**、口径边界
<OUT_ROOT>/capXX/samples/round_NNN.csv.gz   原始时序（可离线复算）
<OUT_ROOT>/sweep.md  /  sweep.csv     跨档位汇总
```

汇总器的退出码:`0` 正常 / `1` 有 UNUSABLE 档位 / `2` 一个档位都读不到。
**非 0 不要当成成功**。

### 12.5 三个会让整张表变成混淆数据的坑(汇总器会主动拒绝出表)

1. **实测限速 ≠ 请求档位** → 该档位标 `UNUSABLE` 并从对比中剔除
2. **两个档位的实测 `vx_max` 相同** → 它们不是两个档位,并列对比会得出
   "限速对精度没有影响"的假结论
3. **耦合衰减在档位间差异显著** → 臂-底盘耦合实测能把底盘压到 **15%**,
   两个不同档位完全可能产生**同一个实际速度**。所以每轮都记
   `coupling_atten_p50/p05`;比较前先把双臂构型固定下来

### 12.6 指标的三类口径(读表前必须知道)

| 类别 | 数量 | 命名 | 说明 |
|---|---|---|---|
| 直接可测 | 22 | 无后缀 | map 系 TF / `/plan` / `/scan` / cmd 链 / action 状态 |
| 代理量 | 4 | `*_proxy` | 定义随数据落进 `run.json`,不只写在代码注释里 |
| 需人工真值 | 1 | — | 低矮障碍检出率,缺真值记 `n/a`,**不编造** |

四个代理量及其**不叫什么**:

- `narrow_success_rate_proxy` —— 窄段 = 连续 ≥0.5s 满足
  `left_min+right_min+2×外接半径 < 1.62m`(1.62 是实测 MPPI 足迹代价饱和宽度);
  段数为 0 时是 `None` 而**不是 1.0**
- `zero_progress_events_proxy` —— 按 `PoseProgressChecker` 判据从实测数据复算。
  **不叫"误判次数"**:follow_path 模式没有 BT,nav2 不发布 trip 原因,真伪判不了。
  另给 `downstream_dead_ratio`(窗内最终 `/cmd_vel` 也≈0 的占比),那部分是
  链路没执行,不该算规划器的账
- `geometric_intrusion_episodes_proxy` —— **不是碰撞次数**。本机没有碰撞传感器,
  真实碰撞由 `COLLISIONS_MANUAL` 人工填(急停在操作者手里)
- `self_filter_residual_ratio_proxy` —— `range < 外接半径` 的光束占比,理应恒为 0。
  非 0 即自滤漏了机器人自身结构(历史故障:夹爪不在自滤链里)

### 12.7 几个容易读错的列

- **到位误差只有一列** `arrival_error_xy_m`:"到位精度差距"与"位置到位误差"
  是同一个量。位姿一律来自 **map 系 TF**,不是 `/odom`(它是 3-DOF 轮式里程计,
  拿它的 y 与 map 系目标相减会造出假误差,详见 §10.8)
- **航向到位误差仅供观察**:本机 `yaw_goal_tolerance ≈ 3.15`(等于不约束朝向)
- **`heading_error` 大不代表跟踪差**:全向底盘可横移。真正该看
  `motion_dir_error`(运动方向 vs 路径切向),两列都给了
- **`vel_track_rms_at_zero_lag` 不是跟踪误差**:链路是位置积分式、实测滞后约
  0.6s,零时延残差的主要成分是时延。看 `rms_at_best_lag` + `best_lag_s`;
  `hit_boundary=True` 时那个"最佳时延"不可信。另看 `vel_track_gain`——
  明显 < 1 是被下游限速压过,不是跟踪差
- **`local_plan_hz` 是中位间隔的倒数**,不是均值。一次停顿看 `gap_max`
  (拿均值会把 157Hz 说成 91Hz,本项目犯过)
- **`traveled_m` 有两个口径**:`traveled_m` 带 2mm 抖动门限、
  `traveled_m_ungated` 不带。位姿 20Hz + 限速 0.1m/s 时单步 5mm,是门限的
  2.5 倍;若把采样提到 50Hz 就变成 2mm、与门限同量级,真实位移会被当抖动丢掉。
  `traveled_gate_suspicious=True` 就是在报这件事
- **`corridor_sample_ratio` 要和 `center_bias_*` 一起看**:只有 2/100 帧处在
  通道里时,那个居中度均值没有统计意义
- **`counter_mismatch=True`** —— 协调器自己的 `dispatched=` 与录制器看到的
  目标数不一致。差值就是漏录/多录的轮数,不查清楚成功率不可信

### 12.8 当前阻塞

探索本身仍被 §11 卡住(`/map_scan_filtered` 无未知区 → 无前沿 → 探索立刻判
COMPLETED)。**本工具可以先用 `OBSERVE_ONLY=true` 空跑验证管线**,
厂商修好当天即可直接采数。

## 附 · 相关文档

- `docs/two_day_hardware_session_report.md` —— 8.31–9.1 两天工作与卡点汇总
- `docs/real_robot_chain_verification.md` —— 生产拓扑首次贯通的实测记录
- `docs/vnc_visualization_tutorial.md` / `docs/pc_rviz_visualization.md` —— 可视化两套方案
- `tools/robot/bringup_stages.sh` —— 七阶段启动脚本(判据都在里面)
- `tools/robot/guard_readonly.sh` —— 只读进程守护
- `ws_robot/src/astribot_s1_navigation/scripts/run_speed_sweep.sh` —— 限速扫描驱动(§12)
- `ws_robot/src/astribot_s1_navigation/astribot_s1_navigation/explore_metrics/` ——
  评测判据(纯函数,可离线复核)
