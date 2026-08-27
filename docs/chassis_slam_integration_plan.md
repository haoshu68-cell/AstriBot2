# 底盘验证 → 外部 SLAM 接入 → 探索/导航验证

> 前置文档：[`real_robot_deployment.md`](real_robot_deployment.md)（怎么装上去）·
> [`sim_real_alignment.md`](sim_real_alignment.md)（对齐什么）
> 本文接在它们之后：**怎么按正确顺序把底盘、外部 SLAM、导航逐段验通。**
>
> 与前两份文档的区别：本文从 §1.C3 起**涉及真实运动**。每个会动的阶段都标了
> ⚠️ 并列出前置安全条件。§0 之前的所有内容不产生运动。

---

## 0 · 结论先行：顺序不是偏好，是被一个缺口锁死的

### 0.1 缺口：实机上没有 `odom → astribot_torso_base`

这条已逐项取证（不是推测）：

| 证据 | 结论 |
|---|---|
| `grep -rn "TransformBroadcaster\|sendTransform\|send_transform"` 全 `ws_robot/src` | **零命中**——本工作空间没有任何节点用代码广播 TF |
| 只有两处 `static_transform_publisher` | `map_provider.launch.py:172`（`map→odom`）与 `warehouse_sim.launch.py:490`（三个仿真传感器别名）。**都不是** `odom→base` |
| 唯一的 `odom→base` 产出者 | `astribot_s1.gazebo.xacro:71-78` 的 `gz-sim-odometry-publisher-system`，**只能通过 Gazebo** |
| 桥接刻意不承担这个角色 | `chassis_cmd_bridge_node.py:267` `frame_id='sdk_chassis'  # 刻意不写 odom：它不是 odom`，且**只发 Odometry 消息、不广播 TF** |
| `chassis_odom_node` | 只存在于 `sim_real_alignment.md:956` 的计划里。**无源文件**，`setup.py:29-36` 只有 `state_bridge_node` / `joint_map_probe` / `bridge_container` |

**这一条边缺失会让下面全部同时失效**（都是同一个根因，但症状分散在四个地方，极易误判成四个独立 bug）：

| 受害者 | 症状 |
|---|---|
| `local_costmap`（`global_frame: odom`） | 无法变换，局部代价地图空 |
| `behavior_server`（`global_frame: odom`） | 恢复行为全废 |
| 任何 2D SLAM | 它需要 `odom→base` 才能算 `map→odom` |
| 探索协调器 | `map→astribot_torso_base` 查不到 → 永久"定位丢失" |
| `bt_navigator` / `velocity_smoother` / 协调器停留判定 | `/odom` **一个发布者都没有** → 全部饿死 |

> **所以：只提供 `/map` + `map→odom` 的适配器在实机上仍然跑不起来。**
> 这是本文顺序编排的唯一约束来源。

### 0.2 被锁死的依赖链

```
C · 底盘开环验证（含 leash / 停车距离）
    └── 不依赖 SLAM，可以立刻开始 ◀── 从这里入手是对的
              ↓
S · 分类 /home/astribot/SLAM，确定它提供什么
              ↓
        ┌─────┴─────┐
   LIO 型            2D 型
   自带里程计         只有 map→odom
        │                 │
        │            需新增 chassis_odom_node
        └─────┬───────────┘
              ↓
      odom→base 就位（缺口填上）
              ↓
      map→odom 就位（外部 SLAM）
              ↓
      /scan 就位（现有链路，已验证形态）
              ↓
N · 导航（单点）→ 探索
```

**关键判断：`C` 与 `S` 可以并行**——底盘开环验证不需要任何 SLAM。
而 `N` 严格在两者之后。先去验探索/导航是白费，因为 TF 树根本不完整。

### 0.3 两个会让你白跑一轮的陷阱（都已核实）

**① 三个 C++ 节点的 `robot_base_frame` 默认值是 `base_link`**

```
exploration_coordinator_node.cpp:217   declare_parameter("robot_base_frame", "base_link", ...)
frontier_explorer_node.cpp:100         declare_parameter("robot_base_frame", "base_link", ...)
pointcloud_slice_scan_node.cpp:134     declare_parameter("base_frame",       "base_link", ...)
```

本仓库**没有 `base_link` 这个 frame**（根是 `astribot_torso_base`）。
yaml 里都写对了，但一旦 params 文件没加载上——而这恰好是本仓库已知会发生的事
（[`real_robot_deployment.md` §3.7②](real_robot_deployment.md) 的节点名 remap
和 `params_file` 共享上下文泄漏）——症状是：
协调器永久"定位丢失"、前沿器永久 `WAITING_TF`、`/scan_from_cloud` **静默为空**。
三个症状都不提 `base_link`。

**② `/scan` 与 `/scan_from_cloud` 是两条不同的链，喂给不同的消费者**

```
/livox/fused_points ─▶ pointcloud_slice_scan_node ─▶ /scan_from_cloud   （4 层切片）→ nav2 obstacle_layer
                    └▶ cloud_self_filtered ─▶ pointcloud_to_laserscan ─▶ /scan   （单层）→ SLAM
```

`mapper_params_online_async.yaml:18` 是 `scan_topic: /scan`（单层）；
`navigation.launch.py:101-108` 把 nav2 的 `scan_topic` 默认改写成 `/scan_from_cloud`。
**外部 SLAM 替换 slam_toolbox 时继承的是 `/scan`，不是 `/scan_from_cloud`。**
搞反了的表现是地图里凭空多出一堆高处障碍（4 层切片把 0.6 m 以上的东西也压进去了）。

---

## 1 · 底盘控制验证顺序（C0 → C5）

**编排原则：离线已覆盖的不重做。** 底盘逻辑已有 **123 个离线用例**
（`test_chassis_bridge_core` 52 + `test_chassis_feedback` 40 + `test_chassis_integrator` 31），
覆盖了限幅算术、积分精确性、leash 算术与冻结语义、看门狗归零、全部状态迁移、
重定位重对齐、漂移算术、配置拒绝、写入闸门判定。

**上机只测离线测不出来的五件事**：

1. 指令是否真的落到实物底盘上（跟随率 / 误差）
2. SDK 那个底盘位姿有没有累积漂移（`Gate 0-f` **明确未取证**）
3. 真实停车距离（两段：积分器惯滑 + SDK `control_way='filter'` 的二次收敛）
4. 真实打滑量级（用来定 `odom_drift_warn_m`，当前 0.15 是未验证值）
5. 250 Hz 内环在真机 GIL 下的实际节拍

### C0 · 离线复跑（不上机，10 分钟）

```bash
# 【机器人端执行】确认下发到 Orin 的代码与本地一致，不是只在 x86 上过
cd ~/astribot_sdk_ros2/ws_robot
colcon test --packages-select astribot_trajectory_bridge --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/astribot_trajectory_bridge
```

**判据**：123 个底盘用例 + 38 个写入闸门用例全过。
任何一个失败都先别上机——aarch64 上的浮点/依赖差异要在这里暴露。

### C1 · 只读取证（不使能、不写入，**无运动**）

这一步专门解决三个"上机才知道"的未知量，且**全部不需要写权限**。

```bash
# 【机器人端执行】前置：按 real_robot_deployment.md §4 起到 Phase 1（只读状态桥接）
cd ~/astribot_sdk_ros2 && source env_robot.sh
```

**① 量 `desired − current` 的初始差值**（防止第一次 enable 立刻触发 leash）

这是个真实风险：种子取自 `get_desired_joints_position`，而 leash 判据用
`get_current_joints_position`。两者在 enable 时刻若差超过
`leash_xy_m`(0.25 m) / `leash_theta_rad`(0.35 rad)，**第一 tick 就跳闸**。

```bash
# 【机器人端执行】纯读，无任何写调用
python3 - <<'PY'
import os
os.environ.setdefault('ASTRIBOT_LOG', '1'); os.environ.setdefault('ROBOT_TYPE', 'S1')
from astribot_sdk.core.astribot_api.astribot_client import Astribot
bot = Astribot(freq=250, high_control_rights=False, node_name='chassis_probe_c1')
part = 'astribot_chassis'
des = bot.get_desired_joints_position([part])[0]
cur = bot.get_current_joints_position([part])[0]
import math
print(f'desired = {[round(v,4) for v in des]}')
print(f'current = {[round(v,4) for v in cur]}')
dxy = math.hypot(des[0]-cur[0], des[1]-cur[1])
dth = abs(math.atan2(math.sin(des[2]-cur[2]), math.cos(des[2]-cur[2])))
print(f'Δxy = {dxy:.4f} m   (leash_xy_m = 0.25)')
print(f'Δθ  = {dth:.4f} rad (leash_theta_rad = 0.35)')
print('⚠️ enable 会立刻跳闸' if (dxy > 0.25 or dth > 0.35) else 'enable 不会因初始差值跳闸')
print(f'chassis_dof = {len(cur)}  (必须是 3，否则 ROBOT_TYPE 不对，enable 会被拒)')
PY
```

**② 量 SDK 底盘位姿的累积漂移**（`Gate 0-f`，决定它能不能当 odom 用）

```bash
# 【机器人端执行】人工手推机器人一段已知距离（例如沿直线 2 m），期间只读
python3 - <<'PY'
import os, time, math
os.environ.setdefault('ASTRIBOT_LOG', '1'); os.environ.setdefault('ROBOT_TYPE', 'S1')
from astribot_sdk.core.astribot_api.astribot_client import Astribot
bot = Astribot(freq=250, high_control_rights=False, node_name='chassis_probe_c1b')
part = 'astribot_chassis'
p0 = bot.get_current_joints_position([part])[0]
print(f'起点 = {[round(v,4) for v in p0]}')
print('现在人工推动机器人（或用手柄），60 秒后自动读终点。Ctrl+C 可提前结束。')
try:
    t0 = time.time()
    while time.time() - t0 < 60:
        p = bot.get_current_joints_position([part])[0]
        print(f'\r t={time.time()-t0:5.1f}s  x={p[0]:+.4f} y={p[1]:+.4f} θ={p[2]:+.4f}', end='')
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
p1 = bot.get_current_joints_position([part])[0]
print(f'\n终点 = {[round(v,4) for v in p1]}')
print(f'SDK 认为位移 = {math.hypot(p1[0]-p0[0], p1[1]-p0[1]):.4f} m')
print('↑ 与卷尺实测比对。差值就是这段距离上的累积误差。')
PY
```

**判据**：SDK 位移与卷尺实测的相对误差。
这个数字决定 §2.5 的 `chassis_odom_node` 能不能直接把它当 odom
（odom 只要求**局部连续**，允许缓慢漂移；但如果它会**跳变**，就不能用）。

**③ 量实际生效的控制频率**（yaml 写 100，client 默认 250，未取证）

```bash
# 【机器人端执行】
python3 - <<'PY'
import os, time
os.environ.setdefault('ASTRIBOT_LOG', '1'); os.environ.setdefault('ROBOT_TYPE', 'S1')
from astribot_sdk.core.astribot_api.astribot_client import Astribot
bot = Astribot(freq=250, high_control_rights=False, node_name='chassis_probe_c1c')
part, n, prev, changes = 'astribot_chassis', 0, None, 0
t0 = time.time()
while time.time() - t0 < 5.0:
    p = tuple(bot.get_current_joints_position([part])[0]); n += 1
    if prev is not None and p != prev: changes += 1
    prev = p
dt = time.time() - t0
print(f'读取 {n} 次/{dt:.2f}s = {n/dt:.1f} Hz（读取速率，不是控制速率）')
print(f'位姿变化 {changes} 次 = {changes/dt:.1f} Hz（静止时应接近 0）')
print('注：真实控制周期要在 C2 用 LOOP_OVERRUN 与 status 时间戳量')
PY
```

### C2 · 使能但零速（**写入打开，但不发任何 `/cmd_vel`**）

⚠️ **这是第一个打开写通路的阶段。** 机器人**理论上不动**（速度恒为 0，
积分器只是持续重发当前位置），但写权限已经打开，必须按运动阶段对待。

**前置安全条件（缺一不可）**：

| # | 条件 |
|---|---|
| 1 | 机器人处于 `safe` 模式（`allow_unsafe_mode` 保持 false，非 safe 会被闸门拒） |
| 2 | 有人手握急停，且已实测急停有效 |
| 3 | 底盘周围 ≥ 2 m 空旷（万一动了有余量） |
| 4 | C1 的 ① 判据通过（初始差值不会立刻跳闸） |
| 5 | 限速已调小：`max_vel_xy:=0.1 max_vel_theta:=0.2` |

> **注意**：`sim_real_alignment.md` Gate 5.2 写的 `velocity_scale: 0.1`
> **这个参数不存在**——它只在 §4 设计稿里（`{sim:1.0, real:0.3}`），
> 桥接包里 grep 不到。等效且已实现的旋钮就是上面第 5 条的 `max_vel_*`。

```bash
# 【机器人端执行】
cd ~/astribot_sdk_ros2 && source env_robot.sh
ros2 launch astribot_trajectory_bridge bridge_bringup.launch.py \
    target:=real allow_write_to_real:=true \
    pose_source:=slam use_sim_time:=false \
    enable_slam_correction:=false \
    domain_id:=$ROS_DOMAIN_ID
```

> `enable_slam_correction:=false` 是刻意的：**闭环修正从未在任何后端跑过**
> （MuJoCo 无 SLAM，全程 `SLAM_UNAVAILABLE_OPEN_LOOP`）。
> 纯开环 + leash 是唯一有真后端证据的配置，先跑它。闭环留到 C5。

```bash
# 【机器人端执行】另一个窗口：先确认闸门放行，再使能
ros2 topic echo /astribot/bridge/status --once      # 不应再是 62
ros2 service call /astribot_bridge_container/enable std_srvs/srv/SetBool "{data: true}"
```

**判据**：

| 检查 | 期望 | 命令 |
|---|---|---|
| 使能成功 | `success: true` | 上面的 service call 返回 |
| 机器人不动 | 目视 + 位姿不变 | `ros2 topic echo /astribot/chassis/odom_from_sdk` |
| 看门狗在报 | `CMD_VEL_TIMEOUT` | `ros2 topic echo /astribot/bridge/status` |
| 内环节拍 | `LOOP_OVERRUN` 不刷屏 | 同上 |

> ⚠️ **`CMD_VEL_TIMEOUT` 会以 250 Hz 刷 status 话题**——核心每个 inner tick 都
> `_emit`，而 `StatusReporter` **不做节流**。这是既有行为，不是故障。
> 订阅 status 时要有心理准备，别把这个洪水当成错误。
> 同理 `LOOP_OVERRUN` 是逐次上报。

**这一步实际验证的是**：写入闸门放行后 250 Hz 流式确实在跑、位置保持有效
（对应记忆里"保持位置必须持续重发"那条）、且不产生意外运动。

### C3 · ⚠️ 手工小速度（**第一次真实运动**）

**前置**：C2 全部判据通过，且 `~/disable` 已实测能立刻停。

```bash
# 【机器人端执行】先验证 disable 能停（在还没动之前先确认刹车）
ros2 service call /astribot_bridge_container/disable std_srvs/srv/Trigger {}
# 再重新使能
ros2 service call /astribot_bridge_container/enable std_srvs/srv/SetBool "{data: true}"
```

运动指令由你按现场情况发出，本文不代写。要测的量和判据如下：

| # | 测什么 | 方法要点 | 判据 |
|---|---|---|---|
| 1 | 单轴 vx | 极小速度、极短时间、立刻停 | 实际位移方向与符号一致 |
| 2 | 单轴 vy | 同上 | **全向底盘 `joint_types [2,2,1]`，vy 是合法的** |
| 3 | 单轴 ω | 同上 | 转向符号正确 |
| 4 | `input_frame` 语义 | 让底盘转过 90° 后再发同一个 body 速度 | `body`（默认）下方向应随车身转；若表现为世界系固定方向，说明 `input_frame` 配错，**车会朝错误方向走** |
| 5 | 钳位真的生效 | **手工注入超限 `/cmd_vel`**（如 vx=5.0） | 实际速度被压到 `max_vel_xy`；xy 是**按合成模长**缩放，不是逐轴 |
| 6 | 看门狗归零 | 发一次速度后停止发布 | `cmd_vel_timeout_sec`(0.3 s) 后速度归零，**状态仍是 ENABLED** |

> **第 5 项必须手工注入**：桥接的限幅值与 nav2 的限幅值**逐字节相同**
> （都等于厂商 `joint_max_velocities [1.0,1.0,2.0]`），所以在 nav2 正常流量下
> 桥接钳位是个 **no-op**，永远测不出来。不手工超限就等于没验证这一层。

### C4 · ⚠️ leash 与停车距离（本阶段最重要）

这一条是整个底盘验证里**唯一可能伤到机器人**的机制，且它的实际行为与直觉不同。

**已核实的机制事实**：leash 触发时**不下发停止指令**。

```python
# chassis_bridge_core.py:346-355
self.pos_cmd = leash_recover_command(actual)   # 算出来了
...
return False                                    # 在 set_joints_position 之前就返回
```

`pos_cmd` 被拉回实测位置却**从未下发**。SDK 手里握着的仍是触发前那个设定点，
最远比实测超前 `leash_xy_m`(0.25 m)。**leash 限制的是"指令能超前多远"，不是"立刻停"。**

叠加两条已知事实——单次位置指令抓不住正在运动的关节、`control_way='filter'`
还会再收敛一段——**真实惯滑距离会超过积分器估算值**。

| # | 测什么 | 方法 | 判据 |
|---|---|---|---|
| 1 | leash 触发 | 极低速下用**软性障碍**（纸箱）挡住，或在低摩擦面上让它打滑 | status 出现 `LEASH_TRIPPED`，带 `err_xy` / `err_theta` |
| 2 | **触发后惯滑距离** | 触发瞬间起量到完全静止 | 记录实测值。**这是要写进文档的新数字**，不要用积分器估算值 |
| 3 | 冻结确认 | 触发后继续发 `/cmd_vel` | 不再有任何 `set_joints_position`；机器人不响应 |
| 4 | 恢复路径 | `~/reset_leash` | 仅在 `LEASH_TRIPPED` 态下成功；成功后重新播种 |
| 5 | 打滑量级 | 对比 SDK 位移与实际位移 | 用来重定 `odom_drift_warn_m`（当前 0.15 是**未验证值**，首轮只当仪表读数，不当验收阈值） |

```bash
# 【机器人端执行】恢复
ros2 service call /astribot_bridge_container/reset_leash std_srvs/srv/Trigger {}
```

> **两个不要围绕它设计测试的死配置**：
> `require_manual_reset` 声明/传参/存储各一处，**没有任何逻辑读它**——
> 行为无条件是手动恢复，设成 false 是个谎。
> `start_disabled` 同样从不被读——启动无条件是 DISABLED（因为 `core.state` 初值就是它）。
>
> 还有一条：`~/reset_leash` **不检查写入闸门**，所以它能在写入被拒时把核心推到
> ENABLED。今天无害（两个定时器都会提前返回），但意味着
> **`state == ENABLED` 不等于指令能下发**——要判断能不能动，看 status 话题。

### C5 · ⚠️ 闭环修正（第一次真正执行 outer_tick）

**前置**：§2 的 SLAM 接入已完成，`map→astribot_torso_base` 可查。

闭环从未在任何后端跑过。打开后这些代码路径第一次面对真实数据：
`detect_pose_jump`、`advance_desired_pose`、`compute_correction`、`slice_correction`、
`slam_max_age_sec`、漂移诊断。

```bash
# 【机器人端执行】把 C2 的启动命令改成
ros2 launch astribot_trajectory_bridge bridge_bringup.launch.py \
    target:=real allow_write_to_real:=true \
    pose_source:=slam enable_slam_correction:=true \
    use_sim_time:=false domain_id:=$ROS_DOMAIN_ID
```

| 测什么 | 判据 |
|---|---|
| 修正量有界 | `max_corr_vel_xy`(0.10) / `max_corr_vel_theta`(0.20) 是"位姿源发疯时的最后一道防线"，不应长期贴顶 |
| 无误判跳变 | `SLAM_RELOCALIZED` 不应频繁出现（阈值 `slam_jump_threshold_m` 0.30） |
| 位姿新鲜度 | 不应频繁 `SLAM_STALE`（`slam_max_age_sec` 0.5，**两种 pose_source 下都生效**） |
| 漂移读数 | 记录 `ODOM_DRIFT_HIGH` 出现频率与量级，用来重定阈值 |

> `require_slam_to_enable` 默认 **false**，意味着 `STOPPED_NO_POSE` 在出厂配置下
> **不可达**：SLAM 丢了会静默降级成永久开环，只在 status 上留一条
> `SLAM_UNAVAILABLE_OPEN_LOOP`。若要"定位丢失即停"的生产行为，必须把它翻成 true
> 并单独验证——而它**没有自动恢复路径**，只能靠 `~/enable`。

---

## 2 · 接入 `/home/astribot/SLAM`

### 2.1 先分类，再设计——三种可能对应三种架构

我不知道那个包是什么，而**它属于哪一类直接决定要不要写 `chassis_odom_node`**。
所以第一步是探测，不是写代码。

| 类型 | 特征 | 它提供 | 还缺什么 |
|---|---|---|---|
| **A · LIO/LIO-SAM 型** | 吃原始点云 + IMU（`/livox/lidar_*` + `/livox/imu_*`） | 高频里程计 + `odom→base` + 点云地图 | 需把点云地图转 `OccupancyGrid`；**`odom→base` 缺口自动填上** ✅ |
| **B · 2D 栅格 SLAM 型** | 吃 `/scan` | `/map` + `map→odom` | **仍缺 `odom→base`** → 必须写 `chassis_odom_node` |
| **C · 只建图不定位** | 吃点云，离线出图 | 只有地图文件 | 缺 `map→odom` **和** `odom→base` → 退回 `real_file` + 我们自己的定位 |

### 2.2 探测脚本（在机器人上跑，只读）

```bash
# 【机器人端执行】保存为 ~/probe_slam.sh
cat > ~/probe_slam.sh <<'EOF'
#!/usr/bin/env bash
# 只读探测 /home/astribot/SLAM：不启动、不修改任何东西
SLAM_DIR=/home/astribot/SLAM
echo "=================== 1. 目录结构 ==================="
[ -d "$SLAM_DIR" ] || { echo "目录不存在：$SLAM_DIR"; exit 1; }
ls -la "$SLAM_DIR"
echo; echo "--- ROS 包（找 package.xml）---"
find "$SLAM_DIR" -maxdepth 3 -name package.xml | while read -r p; do
    echo "[$(dirname "$p")]"
    grep -oP '(?<=<name>)[^<]+' "$p" | sed 's/^/    name: /'
    grep -oP '(?<=<depend>)[^<]+' "$p" | sed 's/^/    dep : /' | head -20
done

echo; echo "=================== 2. 判类型（关键）==================="
echo "--- 是否 LIO 型（点云+IMU 紧耦合）---"
grep -rniE 'fast[_-]?lio|point[_-]?lio|lio[_-]?sam|lego[_-]?loam|faster[_-]?lio|imu_topic|imu_en' \
     "$SLAM_DIR" --include=*.yaml --include=*.launch.py --include=*.launch \
     --include=*.cpp --include=*.hpp 2>/dev/null | head -15
echo "--- 是否 2D 栅格型 ---"
grep -rniE 'cartographer|slam_toolbox|gmapping|hector|occupancy|OccupancyGrid' \
     "$SLAM_DIR" --include=*.yaml --include=*.launch.py --include=*.cpp 2>/dev/null | head -15

echo; echo "=================== 3. 话题与坐标系 ==================="
echo "--- 订阅/发布的话题名（源码里的字符串）---"
grep -rhoE '"/[a-zA-Z0-9_/]{2,}"' "$SLAM_DIR" \
     --include=*.cpp --include=*.hpp --include=*.py --include=*.yaml 2>/dev/null \
     | tr -d '"' | sort -u | head -40
echo "--- frame 名 ---"
grep -rhiE '(map_frame|odom_frame|base_frame|body_frame|lidar_frame|world_frame|frame_id)\s*[:=]' \
     "$SLAM_DIR" --include=*.yaml --include=*.cpp --include=*.py 2>/dev/null | head -25
echo "--- 是否自己广播 TF ---"
grep -rlE 'TransformBroadcaster|sendTransform|send_transform|StaticTransformBroadcaster' \
     "$SLAM_DIR" 2>/dev/null | head

echo; echo "=================== 4. 消息类型 ==================="
grep -rhoE '(nav_msgs|sensor_msgs|geometry_msgs|std_msgs|livox_ros_driver2)::msg::[A-Za-z]+' \
     "$SLAM_DIR" --include=*.cpp --include=*.hpp 2>/dev/null | sort | uniq -c | sort -rn | head -15
grep -rhoE 'from (nav_msgs|sensor_msgs|geometry_msgs)\.msg import [A-Za-z, ]+' \
     "$SLAM_DIR" --include=*.py 2>/dev/null | sort -u | head

echo; echo "=================== 5. 启动方式与已有地图 ==================="
find "$SLAM_DIR" -name '*.launch.py' -o -name '*.launch' -o -name 'run*.sh' 2>/dev/null | head -10
find "$SLAM_DIR" -name '*.pcd' -o -name '*.pgm' -o -name '*.yaml' -path '*map*' 2>/dev/null | head -10
echo; echo "=================== 6. 是否已编译/在跑 ==================="
ls -d "$SLAM_DIR"/install "$SLAM_DIR"/build 2>/dev/null
pgrep -af 'lio|slam|cartographer' | grep -v probe_slam | head
EOF
chmod +x ~/probe_slam.sh && ~/probe_slam.sh 2>&1 | tee ~/deploy_check/slam_probe.txt
```

若它已经在跑，再补一次运行期探测（最可靠，胜过读源码）：

```bash
# 【机器人端执行】它在跑的时候
source /opt/ros/humble/setup.bash
echo "--- 话题 + 类型 ---";  ros2 topic list -t
echo "--- TF 树 ---";        ros2 run tf2_tools view_frames -o /tmp/slam_frames
echo "--- /map QoS（决定要不要适配）---"
ros2 topic info -v /map 2>/dev/null | grep -iE 'durability|reliability|count'
echo "--- 地图元信息 ---"
timeout 15 ros2 topic echo /map --field info --once 2>/dev/null
echo "--- 它发的里程计 ---"
for t in $(ros2 topic list | grep -iE 'odom|lio|localization|pose'); do
  printf '%-40s %s\n' "$t" "$(ros2 topic type "$t" 2>/dev/null)"
done
```

### 2.3 我们这一侧的契约（已逐项查实，不可协商）

任何适配方案都必须满足这张表。**每一行都有一个"不满足时的静默失败症状"**——
这些症状全都不指向 QoS 或 frame，所以必须提前对齐而不是上机再猜。

| # | 契约项 | 值 | 不满足时的症状 |
|---|---|---|---|
| 1 | `/map` 类型 | `nav_msgs/OccupancyGrid` | — |
| 2 | `/map` QoS | **TRANSIENT_LOCAL + RELIABLE** | **零消息**，话题可见但一条收不到，只有一行 WARN。4 个订阅者全部失效 |
| 3 | `/map` frame | `header.frame_id = "map"` | 协调器**不检查** frame_id（只用 `resolution`/`origin.x,y`/`data`），所以填错不报错、直接算错 |
| 4 | 地图分辨率 | `0.05` m | 与 `global_costmap` 的 `resolution: 0.05` 对齐；不一致会有重采样误差 |
| 5 | 栅格编码 | 标准 `-1 / 0 / 100` | `occupied_threshold: 65`、`free_threshold: 25` 按这个口径 |
| 6 | `origin.orientation` | **必须是单位四元数** | 协调器只读 `origin.position.x/y`，**忽略旋转**——非零旋转会静默算错 |
| 7 | `data.size()` | `== width*height` | 协调器 `mapCallback` 直接 ERROR 丢弃（防越界读） |
| 8 | **重发周期** | **≤ 10 s，即使地图没变** | 协调器按**本地到达时间**判 `map_timeout_sec: 10.0`，报"地图已 Ns 未更新(超时)"。**只 latch 一次就不发的地图会冻住探索** |
| 9 | `header.stamp` | 不得超前本地时钟 1 s | 时钟失同步 WARN |
| 10 | TF `map→odom` | 连续发布（参考实现 50 Hz） | nav2 全局代价地图 / `bt_navigator` / 协调器解析 `map→astribot_torso_base` 全靠它 |
| 11 | TF `odom→astribot_torso_base` | **连续发布** | §0.1 的缺口。缺了 = 局部代价地图、恢复行为、2D SLAM、协调器定位、`/odom` 五处同时失效 |
| 12 | `/odom` 话题 | `nav_msgs/Odometry`，frame `odom`→`astribot_torso_base` | `bt_navigator`（`odom_topic: /odom`）、`velocity_smoother`、协调器停留判定都要它 |
| 13 | `/odom` QoS | 协调器用 **`SensorDataQoS`(BEST_EFFORT)** 订阅 | 若发布端是 RELIABLE-only 反而兼容；但 nav2 `OdomSmoother` 用默认 RELIABLE，**发布端必须 RELIABLE** 才同时满足两者 |
| 14 | 一个子帧只能有一个父源 | 外部 SLAM 发 `map→odom` 时，**我们的 slam_toolbox 必须不启动**，且 `localization` 不能是 `ground_truth` | 两个父源 → 位姿反复跳，症状看着像"定位漂移"，极难归因 |
| 15 | 它吃的 scan | 若是 2D 型，喂 **`/scan`**（单层），不是 `/scan_from_cloud` | 喂错会把 0.6 m 以上的东西压进地图，凭空多出障碍（§0.3②） |

**顺带三个协调器的硬约束**（不是 SLAM 侧的，但会一起坑）：

- `goal_unknown_clearance_radius` 必须 **0**：任何 `>= info.resolution` 的值会让
  **每一个**前沿候选被否（协调器 `mapCallback` 直接 ERROR）。当前 yaml 已是 0，别动。
- `planner_server` 的 `allow_unknown: false` → **未知栅格是硬阻塞**。
  外部 SLAM 怎么标未探索区（`-1`）直接决定路径可行性。
- 协调器订阅的 `/global_costmap/costmap_raw` QoS 是 **VOLATILE**，
  而 nav2 发布端也是默认 VOLATILE，这一对是匹配的——别"顺手"改成 transient_local。

### 2.4 数据结构与话题设计

**设计原则：不改上层一行。** §7.2 的坐标系决策已经保证了"`odom` 之上的东西
（nav2、探索、代价地图、诊断）一行都不用改"。所以适配层的职责就是
**把外部 SLAM 归一化成上表 15 项**，而不是让上层去适应它。

```
                        ┌──────────────────────────────────────┐
/livox/lidar_left  ──┬─▶│ 现有感知链（不改）                    │
/livox/lidar_right ──┘  │ preprocess×2 → fusion → self_filter  │
                        │   ├─▶ /scan            （单层，给 SLAM）│
                        │   └─▶ /scan_from_cloud （4 层，给 nav2）│
                        └──────────────────────────────────────┘
                                      │
                    ┌─────────────────┴──────────────────┐
                    ▼                                     ▼
        ┌───────────────────────┐            ┌────────────────────────┐
        │ /home/astribot/SLAM   │            │ chassis_odom_node      │
        │ （外部，我们不改它）   │            │ （新增，§2.5）          │
        │                       │            │  SDK 底盘位姿/速度      │
        │  出: 它自己的话题/frame │            │  出: /odom + TF        │
        └───────────┬───────────┘            │      odom→torso_base   │
                    │                         └────────────┬───────────┘
                    ▼                                      │
        ┌───────────────────────────┐                      │
        │ slam_adapter_node（新增）  │                      │
        │  · 话题改名 + QoS 归一化   │                      │
        │  · frame 改名              │                      │
        │  · ≤10s 心跳重发           │                      │
        │  · 契约校验，不合格响亮失败 │                      │
        │  出: /map (transient_local)│                      │
        │      TF map→odom           │                      │
        └───────────┬───────────────┘                      │
                    └──────────────┬───────────────────────┘
                                   ▼
                    map → odom → astribot_torso_base 完整
                                   ▼
                        nav2 / 探索协调器（零改动）
```

#### `slam_adapter_node` 的数据结构

```python
# 建议落点：ws_robot/src/astribot_s1_perception/astribot_s1_perception/slam_adapter_node.py
# 理由：与 map_domain_relay.py / map_source_config.py 同包，复用 latched_qos() 与校验风格

class MapContract:
    """§2.3 那 15 项里与 /map 有关的部分，做成可单测的纯逻辑。

    刻意与 ROS 解耦（照 ports.py 的既有风格），这样契约校验能离线测，
    不需要真的跑一个 SLAM。
    """
    resolution_expected: float = 0.05
    resolution_tol: float = 1e-6
    republish_period_sec: float = 5.0      # 必须 < 协调器的 map_timeout_sec=10.0
    max_stamp_future_sec: float = 1.0

    @staticmethod
    def validate(grid) -> list[str]:
        """返回违规项列表，空列表 = 合格。不抛异常，让调用方决定响亮失败的方式。"""
        # 检查项（每一条都对应 §2.3 表里的一行）：
        #   width/height != 0
        #   len(data) == width*height
        #   resolution > 0 且 ≈ 0.05
        #   origin.orientation 是单位四元数（w≈1, x≈y≈z≈0）
        #   data 取值只出现在 {-1, 0, 100}（或落在 [-1,100]）
        #   header.stamp 不超前本地时钟 max_stamp_future_sec
        ...
```

```python
class SlamAdapter:
    """外部 SLAM → 我们的契约。三件事，各自独立可测。

    1) 话题/frame 改名：外部名 → map / odom / astribot_torso_base
    2) QoS 归一化：不论上游怎么发，下游一律 TRANSIENT_LOCAL + RELIABLE
    3) 心跳重发：缓存最后一帧，按 republish_period_sec 重发，
       满足协调器"按本地到达时间判超时"的要求（§2.3 第 8 项）
    """
```

#### 话题与参数设计

| 方向 | 参数名 | 默认 | 说明 |
|---|---|---|---|
| 入 | `source_map_topic` | `/slam/map` | 外部 SLAM 的地图话题（由 §2.2 探测填） |
| 入 | `source_map_qos` | `auto` | `auto` \| `transient_local` \| `volatile`。`auto` 先试 transient_local，超时无消息再退 volatile 并 WARN |
| 入 | `source_odom_topic` | `''` | 空 = 外部不提供里程计（B/C 型），由 `chassis_odom_node` 负责 |
| 入 | `source_map_frame` | `map` | 外部的 map frame 名 |
| 入 | `source_base_frame` | `base_link` | 外部的 base frame 名（LIO 包常用 `body` 或 `base_link`） |
| 出 | `map_topic` | `/map` | 固定，下游硬依赖 |
| 出 | `odom_topic` | `/odom` | 固定 |
| 出 | `map_frame` / `odom_frame` / `base_frame` | `map` / `odom` / `astribot_torso_base` | 固定 |
| 行为 | `republish_period_sec` | `5.0` | **必须 < 10.0**，启动时校验 |
| 行为 | `publish_map_to_odom_tf` | `true` | 外部若已自己发 `map→odom`，置 false 避免双父源（§2.3 第 14 项） |
| 行为 | `strict_contract` | `true` | 契约违规时**拒绝启动并非零退出**，而不是静默转发坏数据 |
| 行为 | `source_timeout_sec` | `30.0` | 收不到源地图就 ERROR + 非零退出（照 `map_domain_relay.py` 的"不允许静默等待"） |

> **`strict_contract` 默认 true 是刻意的**：本仓库反复踩过"配置静默不生效"的坑
> （params_file 泄漏、节点名 remap、QoS 不兼容零消息）。适配层是新引入的一层，
> 必须让它**响亮失败**，否则又多一处静默故障源。

#### 扩展 `map_source_config.py`：新增 `localization: external`

现有三个合法组合里**没有**"外部 SLAM 同时提供 `/map` 和 `map→odom`"这一种。
最小改动是给 `localization` 轴加一个取值，而不是新造一个 mode 参数：

```python
# map_source_config.py:20 现状
VALID_LOCALIZATION = ('slam', 'ground_truth')
# 改为
VALID_LOCALIZATION = ('slam', 'ground_truth', 'external')

# 新增合法组合（validate_combination 里加）
#   real_live + external   ← 外部 SLAM 在线建图/定位，这是本次要用的
#   real_file + external   ← 用外部 SLAM 存下来的图 + 它的定位
# 仍然拒绝
#   sim_slam + external    ← 无意义：仿真里没有外部 SLAM
```

`external` 的语义要在 `map_provider.launch.py` 里落实为三条：
① **不启动** slam_toolbox（`needs_slam_toolbox` 已经只认 `sim_slam`，天然满足）；
② **不发** `map→odom` 静态 TF（`ground_truth` 那条分支不能走）；
③ 启动 `slam_adapter_node`，并沿用既有的 `map_start_cell_check` 做出生点校验。

### 2.5 `chassis_odom_node`（填 §0.1 的缺口）

**只有 B / C 型才需要**（A 型 LIO 自带里程计）。职责就是 `sim_real_alignment.md:956`
计划的那件事，一直没实现。

```python
# 落点：ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/chassis_odom_node.py
# 复用：sdk_session.open_session()（只读会话，high_control_rights 硬编码 False）
#       ros_ports.py 的风格；纯数学部分抽出来单测

class ChassisOdomSource:
    """SDK 底盘位姿/速度 → Odometry + TF。纯逻辑，不含 rclpy。

    输入（每周期一次，两个 SDK 只读调用）：
        pos = get_current_joints_position(['astribot_chassis'])[0]   # [x, y, theta]
        vel = get_current_joints_velocity(['astribot_chassis'])[0]   # [vx, vy, w]

    输出：
        Odometry(frame_id='odom', child_frame_id='astribot_torso_base')
        TF     odom → astribot_torso_base

    ⚠️ 前提，必须由 §1.C1② 先取证：
       odom 只要求**局部连续**，允许缓慢漂移；但**不允许跳变**。
       若 C1② 测出 SDK 位姿会跳（厂商内部重定位），这个方案不成立，
       必须退回"用外部 SLAM 的里程计"或"轮式里程计自己积分"。
    """
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `part_name` | `astribot_chassis` | 与 `chassis_bridge.yaml:17` 一致 |
| `publish_rate` | `50.0` | 对齐 Gazebo `OdometryPublisher` 的 50 Hz，下游行为不变 |
| `odom_frame` / `base_frame` | `odom` / `astribot_torso_base` | 固定 |
| `odom_topic` | `/odom` | 固定，`bt_navigator` 硬依赖 |
| `publish_tf` | `true` | 置 false 可只发话题（对照实验用） |
| `jump_threshold_m` | `0.30` | 检测到跳变就 **WARN + 上报**，不静默平滑掉——照 §7.10.5 的测量纪律 |

> **不要复用 `chassis_cmd_bridge_node` 里那个 `_publish_odom`**：
> 它 `frame_id='sdk_chassis'`、不发 TF、且**只在成功 enable 之后才发**
> （`pos_cmd is None` 就提前返回，写入闸门拒绝时 `_outer_tick` 也提前返回）。
> 这意味着 Gate 5.1 写的"只读、手推机器人"用今天的代码**做不到**。
> 新节点必须独立于写通路——这也是它该放在单独节点而不是塞进 cmd bridge 的原因。

---

## 3 · 探索 / 导航验证顺序（N0 → N4）

**N0 / N1 无运动**，是纯静态校验；从 N2 起涉及运动。

### N0 · TF 树完整性（无运动，但是所有后续的前提）

```bash
# 【机器人端执行】三条边必须全部有解
for pair in "map odom" "odom astribot_torso_base" "map astribot_torso_base"; do
  set -- $pair
  echo "--- $1 → $2 ---"
  timeout 6 ros2 run tf2_ros tf2_echo "$1" "$2" --ros-args -p use_sim_time:=false 2>&1 | head -6
done

# 完整树留档
ros2 run tf2_tools view_frames -o ~/deploy_check/tf_full
# 必须无 base_link（§0.3① 的判据）
ros2 topic echo /tf_static --once 2>/dev/null | grep -c base_link || echo "无 base_link OK"
# 必须每条边只有一个父源
ros2 topic echo /tf --once 2>/dev/null | grep child_frame_id | sort | uniq -d \
  && echo "⚠️ 有子帧被重复发布（双父源）" || echo "无重复父源 OK"
```

**判据**：三条边全部有解、无 `base_link`、无重复父源。
最后一条尤其重要——双父源的症状是"位姿反复跳"，看着像定位漂移。

### N1 · 地图与代价地图（无运动）

```bash
# 【机器人端执行】① 适配后的 /map 是否满足契约
ros2 topic info -v /map | grep -iE 'durability|reliability|count'
#   期望 TRANSIENT_LOCAL + RELIABLE，Publisher count: 1

timeout 15 ros2 topic echo /map --field info --once
#   核对 resolution=0.05、origin.orientation 是单位四元数

# ② 心跳：10 秒内必须有新的到达（§2.3 第 8 项）
echo "开始 25 秒心跳观测…"
timeout 25 ros2 topic hz /map
#   期望 ≥ 0.1 Hz（即 ≤10s 一次）。若只有一条消息就停 = 探索会冻住

# ③ 起代价地图（nav2 的一部分，此时还不发 cmd_vel）
#    注意：完整 nav2 会起 controller_server，它一旦收到目标就会发 /cmd_vel
#    所以这一步只看代价地图，不发任何目标
ros2 topic list | grep -E 'global_costmap|local_costmap'
ros2 topic echo /global_costmap/costmap_raw --field metadata --once
ros2 topic echo /local_costmap/costmap_raw --field metadata --once
```

**判据表**：

| 检查 | 期望 | 不满足的后果 |
|---|---|---|
| `/map` QoS | TRANSIENT_LOCAL + RELIABLE | 4 个订阅者全部收不到，零消息 |
| `/map` 重发 | ≤ 10 s | 协调器报"地图已 Ns 未更新"，探索冻住 |
| `resolution` | 0.05 | 与 global_costmap 重采样误差 |
| `origin.orientation` | 单位四元数 | 协调器忽略旋转，静默算错 |
| `global_costmap` 有内容 | 非空、静态层已填充 | `static_layer` 没收到地图 |
| `local_costmap` 有内容 | 非空 | `odom→base` 缺失（回 N0） |
| 出生点校验 | `map_start_cell_check` 通过 | 出生在墙里 → `Starting point in lethal space!` |

### N2 · ⚠️ 单点导航（空场地，有人守急停）

**前置**：C4 完成（leash 与停车距离已实测）、N0/N1 全绿。

**必须先重算两个按仿真速度定的参数**——不改会得到"局部规划器走不动"的假结论：

| 参数 | 现值 | 为什么要改 |
|---|---|---|
| `required_movement_radius` | 0.5 | 按仿真速度定的。真机 `joint_max_velocities` 只有 1.0 m/s，臂-底盘耦合再乘 0.15 → **0.15 m/s**，比仿真更容易触发进度检查器 |
| `movement_time_allowance` | 10.0 | 同上 |

> 臂-底盘耦合那条：C1 已修 `folded_reference_rad` 基准（导航 78.1s→13.4s），
> 但真机限速更低，**首次上机务必先把手臂收到 `ready` 并确认 `min_speed_scale` 不是瓶颈**。

验证顺序（目标由你按现场发，本文不代写运动指令）：

| # | 内容 | 判据 |
|---|---|---|
| 1 | 极近目标（< 1 m，正前方） | 到达 `xy_goal_tolerance` 内；leash 全程不触发 |
| 2 | 需要转向的目标 | 转向方向正确（验 `input_frame` 在闭环下也对） |
| 3 | 侧向目标（验全向 vy） | 能横移；`joint_types [2,2,1]` 支持 |
| 4 | 速度链路逐级核对 | `preCpl→cmd` 比值符合 `min_speed_scale`；用 `path_tracking_diagnostics_node` 逐段量 |
| 5 | 中途取消 | 立刻停；`~/disable` 也能停 |

```bash
# 【机器人端执行】速度链路逐级观测（只读）
ros2 topic hz /cmd_vel_nav_body_raw   # controller_server 原始输出
ros2 topic hz /cmd_vel_nav_body       # velocity_smoother 之后
ros2 topic hz /cmd_vel_pre_arm_coupling
ros2 topic hz /cmd_vel                # 臂-底盘耦合之后，桥接的输入
ros2 topic echo /astribot/bridge/status   # leash / 超限 / 节拍
```

### N3 · ⚠️ 自主探索

**前置**：N2 通过。探索 = nav2 + 协调器，运动幅度和不可预测性都更大。

**探索特有的三个前置检查**（每一条都有历史踩坑记录）：

```bash
# 【机器人端执行】
# ① 前沿净空半径必须是 0 —— 任何 >= 分辨率的值会否掉每一个候选
ros2 param get /exploration_coordinator goal_unknown_clearance_radius   # 必须 0.0

# ② 自滤是否把夹爪算进去了 —— 不算 = 机器人把指尖当障碍 = 零派发
ros2 topic echo /scan --once 2>/dev/null | head -20
#    最近距离不应显著小于 range_min(0.35)

# ③ 协调器的 frame 是否真的是 astribot_torso_base（不是 C++ 默认的 base_link）
ros2 param get /exploration_coordinator robot_base_frame   # 必须 astribot_torso_base
ros2 param get /frontier_explorer robot_base_frame          # 同上
ros2 param get /pointcloud_slice_scan_node base_frame       # 同上
```

| # | 内容 | 判据 |
|---|---|---|
| 1 | 前沿被识别 | `/exploration/state` 不停在 `尚未收到占据栅格地图` / `WAITING_TF` |
| 2 | 派发次数 > 0 | 有 `NavigateToPose` 目标发出（历史上"零派发"是最常见的失败） |
| 3 | 地图在长 | `/map` 的 `known%` 单调上升（`map_domain_relay.py` 的指纹日志格式可复用） |
| 4 | 无误判完成 | `/exploration/complete` 不在早期就 true |
| 5 | leash 不触发 | 探索路径贴墙更近，这里最容易触发 |

### N4 · ⚠️ 全流程

`mobile_transport` 场景（搬运→导航→放货）。判据按 Gate 5.4：
**与仿真侧同一份配置，只换 `target`，业务层 diff 为空。**

> 这一步依赖 §3.7① 的 MoveIt↔桥接 action 命名对齐**先解决**
> （`moveit_controllers.yaml` 期望 `/arm_left_controller/...`，桥接给
> `/astribot/arm_left_controller/...`；且桥接缺 `torso` 与夹爪的 action 服务端）。
> 在那之前 N4 跑不起来，与底盘无关。

---

## 4 · 需要你先给我的三个输入

这三项定了，§2 的适配器就能落成代码；不定就只能是设计稿。

| # | 问题 | 怎么得到 | 影响什么 |
|---|---|---|---|
| 1 | **`/home/astribot/SLAM` 是 A / B / C 哪一类** | 跑 §2.2 的 `probe_slam.sh`，把输出发我 | 决定要不要写 `chassis_odom_node`（B/C 要，A 不要） |
| 2 | **它的话题名、frame 名、`/map` 的 QoS** | §2.2 运行期探测那段 | 适配器的 `source_*` 参数全部由此填 |
| 3 | **`ROS_DOMAIN_ID` 实测值（25 还是 42）** | `real_robot_deployment.md` §1.3 | 你给的是 42，但 §7.7 写"已全栈统一 25（真机 SDK 后端改不动）"，而 `bridge_bringup.launch.py:59` 默认还是 42——**仓库内部就不一致**，只能实测 |

### 建议的并行推进方式

```
今天就能开始（互不依赖）：
  ├─ C0 离线复跑                      ← 10 分钟，先做
  ├─ C1 只读取证（三个探测脚本）        ← 无运动，回答"SDK 位姿能不能当 odom"
  └─ §2.2 probe_slam.sh               ← 无运动，回答上表第 1、2 项

C1② 与 probe 结果出来之后：
  ├─ 定 chassis_odom_node 要不要写
  ├─ 写 slam_adapter_node + 扩 map_source_config
  └─ C2→C5 底盘上机（可与适配器开发并行，因为 C2/C3/C4 都不需要 SLAM）

两条汇合后才进 N0→N4。
```

---

> **本文所有关于代码现状的结论都来自静态勘察，没有连接过机器人。**
> 标"已核实"的是我逐条跑过命令验证的（缺 `odom→base`、`require_manual_reset` 是死配置、
> leash 不下发停止指令、`velocity_scale` 不存在、三处 `base_link` 默认值、两条 scan 链）；
> 标"未取证"的是真的未知（SDK 位姿漂移、真实停车距离、实际控制频率、闭环行为）。
> §4 那三项现场输入拿到之前，§2 的适配器只能是设计稿。
