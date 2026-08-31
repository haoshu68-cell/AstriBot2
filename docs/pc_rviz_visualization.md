# PC 端 RViz 可视化：方案 A / B 完整落地

**目标** 在开发 PC 上看到机器人的建图、导航、探索信息
**日期** 2026-08-31 · **本文所有网络/系统事实均为实测,来源已标注**

---

## 0 · 为什么不能直接在 PC 上跑 rviz2

实测的三层阻断:

```
PC        10.249.22.160/24   eno1        （只在 WiFi 网段）
机器人    10.249.22.137/24   wlP1p1s0    ← PC 只能走这条
          192.168.0.11/24    eno1        ← 厂商 DDS 只用这张网卡

① PC → 192.168.0.11              100% packet loss（私有段，PC 无路由）
② 厂商 fastdds_udp.xml 白名单     只有 192.168.0.11 + 127.0.0.1
   且 useBuiltinTransports=false（连内置传输都禁了）
③ iptables 在 WiFi 上封 DDS 发现
   OUTPUT  DROP all -- * wlP1p1s0  0.0.0.0/0 -> 239.255.0.1
   INPUT   DROP all -- wlP1p1s0 *  0.0.0.0/0 -> 239.255.0.1
   239.255.0.1 = Fast DDS 默认多播发现地址
```

第 ③ 条是**厂商刻意**封掉 WiFi 侧 DDS 发现的 —— 规则精确绑定 `wlP1p1s0`,`eno1` 不受影响。
这不是配置疏漏,是设计意图:业务流量只走 eno1。

> ⚠️ **一个重要的技术纠正**:`interfaceWhiteList` 过滤的是**本机使用哪些网络接口**,
> **不是**"允许哪些远端地址"。所以方案 B 里 PC 接入 `192.168.0.x` 后,机器人经
> 已在白名单里的 `eno1` 就能与它通信 —— **不需要修改厂商任何文件**。
> (我一开始说"要往白名单加 PC 地址",那是错的。)

---

## 方案 A · NoMachine 远程桌面(零改动,今天可用)

### A.0 可行性已实测

你提到"机器人没有图形界面",实测**与此不符**:

| 项 | 实测结果 | 依据 |
|---|---|---|
| 默认启动目标 | `graphical.target` | `systemctl get-default` |
| 显示管理器 | **gdm3 active** | `systemctl is-active gdm3` |
| X server | Xorg PID 2517 @ vt2 | `ps` |
| 你的图形会话 | **`X1001` 已存在**(属 astribot) | `/tmp/.X11-unix/` |
| NX 服务 | **active,`0.0.0.0:4000` 监听** | `ss -ltnp` |
| rviz2 | **两套 ROS 里都有** | `middle_ware/lib/rviz2/` 与 `humble/lib/rviz2/` |
| GPU | `/dev/nvidia0` + `nvidia-modeset` | Orin 集成 GPU |

`0.0.0.0:4000` 是关键 —— NoMachine 在**所有网卡**上监听,包括 WiFi。

### A.1 为什么这条路一定通

rviz2 与话题**同机**,DDS 走 `127.0.0.1` —— 而厂商白名单里**恰好有** `127.0.0.1`。
不碰 DDS 配置、不接线、不违反"业务走 eno1"。NX 传的是画面,与 DDS 无关。

### A.2 PC 端步骤

1. 装 NoMachine 客户端(免费版够用):<https://www.nomachine.com/download>
2. 新建连接:主机 `10.249.22.137`,端口 `4000`,协议 NX
3. 用 `astribot` 账号登录 → 会接到已存在的 `X1001` 会话

### A.3 机器人端:启动可视化(在 NX 桌面的终端里)

⚠️ **rviz2 有两份,必须用与话题匹配的那套**,否则撞上"两个 ROS"问题
(重叠 184 个包、其中 150 个版本不同)。

```bash
source /opt/astribot_ros/robot_system_ctrl/robot_env.sh
source /home/astribot/Downloads/astribot_sdk_aarch64/ws_robot/install/setup.bash
rviz2 -d /home/astribot/astribot_chain.rviz      # 配置见 A.5
```

这会用到 `middle_ware` 那份 rviz2(它在 AMENT 路径前面),而厂商栈与 SLAM
都在那套上 —— 一致。

### A.4 先把数据链路起来(三层环境,缺一不可)

```bash
# ① 雷达驱动 —— 必须用厂商真入口，env_robot.sh 里看不到这个包
source /opt/astribot_ros/robot_system_ctrl/robot_env.sh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
#   实测：/livox/lidar_front CustomMsg 9.994Hz，/livox/imu_front 200Hz

# ② Voxel-SLAM —— 对着厂商 middle_ware 编译，且缺 GTSAM 库路径
source /opt/astribot_ros/robot_system_ctrl/robot_env.sh
export LD_LIBRARY_PATH=/home/astribot/SLAM/ThirdParty/GTSAM/install4.1.0/lib:$LD_LIBRARY_PATH
cd ~/SLAM/vxlm-slam && source install/setup.bash
ros2 launch voxel_slam vxlm_mid360.launch.py     # 包名是 voxel_slam（下划线）
#   实测：/map_scan_filtered PointCloud2 9.982Hz，TF camera_init→aft_mapped

# ③ 我们的节点 —— 厂商环境 + 我们的 overlay 两层叠加
source /opt/astribot_ros/robot_system_ctrl/robot_env.sh
source <SDK>/ws_robot/install/setup.bash
ros2 run astribot_s1_perception cloud_to_grid_node --ros-args \
  --params-file <SDK>/ws_robot/install/astribot_s1_perception/share/astribot_s1_perception/config/cloud_to_grid_params.yaml
ros2 run astribot_s1_perception map_odom_tf_node
#   实测：投影 193x170 占据 262 空闲 7337 耗时 0.03s

# ④ nav2（可选，要看代价地图/路径时）
ros2 launch astribot_s1_navigation navigation.launch.py use_sim_time:=false
```

> ⚠️ `odom→astribot_torso_base` 生产上由 `chassis_odom_node` 提供,但它依赖厂商
> **本体运动控制服务**,当前未启动(SDK 报 "No simulation or real robot is started",
> 7 个部件全 not alive)。在那之前 TF 树缺这一环,nav2 起不来。
> 上次验证用的是"把 SLAM 位姿当 odom 转发"的临时拓扑(LIO-only),
> 见 §C 的说明 —— **那不是生产拓扑**。

### A.5 rviz 配置要点(手点也可以,但这几处必须对)

| Display | 话题 | **必须设的 QoS/参数** | 不设的后果 |
|---|---|---|---|
| Map | `/map` | **Durability = Transient Local**,Reliability = Reliable | **一片空白**,且无任何报错 |
| Map | `/global_costmap/costmap` | 同上 | 同上 |
| Map | `/local_costmap/costmap` | 同上 | 同上 |
| PointCloud2 | `/map_scan_filtered` | Reliability = **Best Effort**;Size 0.03 | 收不到或卡顿 |
| TF | — | 勾 `map` `odom` `astribot_torso_base` `camera_init` `aft_mapped` | — |
| Path | `/plan` | 默认 | — |
| Fixed Frame | — | **`map`**(不是 `base_link` —— 本机器人没有 `base_link`) | 全部 Display 报 frame 错误 |

`/map` 的 QoS 是最容易踩的一处:我们的节点按 `TRANSIENT_LOCAL + RELIABLE` 发布,
rviz2 默认订阅是 `VOLATILE` —— **TRANSIENT_LOCAL 发布方与 VOLATILE 订阅方是兼容的**
(订阅方要求更弱),所以这一项其实能收到;但反过来(订阅方要 TRANSIENT_LOCAL 而发布方
是 VOLATILE)就零消息零报错。设成 Transient Local 更稳,且能收到 latched 的历史帧。

**机器人模型显示不出来** —— `RobotModel` 需要 `/robot_description` 与 `/joint_states`,
后者要 SDK 读关节角,同样卡在厂商本体服务。地图、点云、TF、代价地图、路径都不受影响。

### A.6 方案 A 的优缺点

| 优点 | 缺点 |
|---|---|
| 零配置改动,不碰厂商 DDS | 传画面而非数据,网络差时卡顿 |
| rviz 与话题同机,DDS 走 loopback,**一定通** | 大点云渲染吃机器人自己的 GPU |
| 不违反"业务走 eno1" | 多人同时看会互相抢会话 |
| 今天就能用 | 截图/录屏不如本地方便 |

---

## 方案 B · PC 接入 192.168.0.x 业务网段(正统拓扑)

### B.0 关键前提:不需要改厂商任何文件

如 §0 所述,`interfaceWhiteList` 管的是**本机网卡**。机器人经已白名单的 `eno1`
就能与同网段的 PC 通信。所以:

- ❌ 不需要往 `fastdds_udp.xml` 加 PC 地址
- ❌ 不需要动 `robot_env.sh`(它第 32 行的 `sed -i 's/192\.168\.0\.10/192.168.0.11/g'`
  只替换那一个地址,但我们**根本不用改这个文件**)
- ✅ 只需给 PC 一个 `192.168.0.x` 地址,并让 PC 的 DDS 配置与机器人兼容

### B.1 网段现状(实测)

```
192.168.0.10   疑似 x86 主机（厂商有 /astribot/x86_system_monitor 话题）
192.168.0.11   Orin，eno1                    ← 机器人 ROS 业务
192.168.0.12   Livox Mid-360 前              ← ARP 可达
192.168.0.13   Livox Mid-360 后              ← ARP 可达（REACHABLE）
```

建议给 PC 用 **`192.168.0.100`** —— 远离现有设备,不会撞。

### B.2 物理连接:机器人有空闲网口

```
ip -o link show → wlP1p1s0  can0  enP5p1s0  can1  eno1  l4tbr0  usb0  usb1
                              ↑ 空闲
ip link show enP5p1s0 → <NO-CARRIER,BROADCAST,MULTICAST,UP> state DOWN
```

`enP5p1s0` 未配地址、无载波。**两条接法:**

**接法 1 · 接入现有交换机(推荐)**
`192.168.0.x` 网段上已有 .10/.11/.12/.13 四个设备,必然有交换机。
把 PC 用网线接到该交换机的空闲口 —— 机器人侧零改动。

**接法 2 · PC 直连 `enP5p1s0`**
网线直连 PC 与机器人的 `enP5p1s0`,机器人侧给它配 `192.168.0.101/24`。
但这样 PC 与 `.12/.13` 雷达不在同一广播域,DDS 多播发现需要机器人做转发 ——
**更复杂,不推荐**。

> ⚠️ PC 现在只有一个 `eno1` 且已用于公司网(`10.249.22.160`)。接入 192 段需要:
> **第二张网卡**(USB 千兆网卡即可),或改用接法 2。
> 用同一张网卡配双地址(`ip addr add`)在同一物理链路上做不到 —— 两个网段
> 物理上是分开的。

### B.3 PC 端配置

```bash
# ① 给第二张网卡配地址（假设是 enx001122334455）
sudo ip addr add 192.168.0.100/24 dev enx001122334455
sudo ip link set enx001122334455 up

# ② 验证能通机器人业务口
ping -c3 192.168.0.11        # 必须通，不通就先解决物理层

# ③ DDS 环境（这几项必须与机器人一致或兼容）
export ROS_DOMAIN_ID=25              # 实机是 25，不是 42
export ROS_LOCALHOST_ONLY=0          # ⚠️ =1 与 =0 的参与者互相发现不了，同机同域也不行
unset RMW_IMPLEMENTATION             # 或显式设 rmw_fastrtps_cpp，与机器人一致
```

**PC 侧是否需要 Fast DDS XML?** 分两种情况:

- **PC 只有一张网卡在 192 段** → 不需要,默认配置即可
- **PC 有多张网卡**(你的情况:`eno1` 在公司网 + `tun0` VPN) → **建议加白名单**,
  否则 PC 的 DDS 会在所有网卡上广播发现包,既浪费也可能干扰公司网:

```xml
<!-- ~/fastdds_pc.xml -->
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>pc_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>192.168.0.100</address>   <!-- 只用这张网卡 -->
          <address>127.0.0.1</address>
        </interfaceWhiteList>
        <!-- 与机器人一致的大缓冲：雷达点云是大消息 -->
        <sendBufferSize>134217728</sendBufferSize>
        <receiveBufferSize>134217728</receiveBufferSize>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="pc_profile" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>pc_udp</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_pc.xml
```

> 机器人那份 `fastdds_udp.xml` 有 `sendBufferSize/receiveBufferSize = 128MB`。
> PC 侧不配大缓冲时,大点云可能丢包 —— 表现为点云闪烁或稀疏,而**不会报错**。

### B.4 验证顺序(每步都要过)

```bash
# 【PC】① 网络层
ping -c3 192.168.0.11

# 【PC】② DDS 发现（最关键的一步）
ros2 node list          # 应看到厂商的 40+ 个节点
ros2 topic list | wc -l # 应看到 42+ 个话题

# 【PC】③ 真的收到数据（不只是发现）
ros2 topic hz /astribot_error_code/soc     # 实测机器人侧 30.003Hz
ros2 topic echo /map --once --field info   # 起了 cloud_to_grid_node 后

# 【PC】④ TF 树完整
ros2 run tf2_ros tf2_echo map astribot_torso_base

# 【PC】⑤ 起 rviz2
rviz2
```

**第 ② 步失败的排查顺序**(按可能性):

| 症状 | 原因 | 判据 |
|---|---|---|
| 节点列表空 | `ROS_DOMAIN_ID` 不是 25 | `echo $ROS_DOMAIN_ID` |
| 节点列表空 | `ROS_LOCALHOST_ONLY=1` | `=1` 与 `=0` 的参与者互不可见,**同机同域也不行**(已实测) |
| 节点列表空 | 多播被拦 | `ping -c2 239.255.0.1`;PC 侧防火墙 |
| 能看到节点但收不到数据 | QoS 不兼容 | `ros2 topic info <t> --verbose` 对比两端 |
| 点云稀疏/闪烁 | PC 侧缓冲太小 | 加 §B.3 的 `receiveBufferSize` |

### B.5 安全影响(必须权衡)

PC 接入 `192.168.0.x` 后,**PC 与机器人在同一个 DDS 域里双向可见**。这意味着:

- PC 上任何进程都能**发布** `/cmd_vel`、`/cmd_vel_nav_body` 等指令话题
- 你之前定的约束是"WiFi 只做 ssh 运维,业务走 eno1" —— 方案 B 符合这条,
  但它把 PC 变成了业务网段的一等参与者

**建议的缓解措施:**

1. **PC 上不要装/跑底盘桥接与 controller** —— 从源头上没有速度发布者
2. **只订阅,不发布**:可视化只需订阅;要发目标点时用 `ComputePathToPose`
   (纯规划)而非 `NavigateToPose`
3. 若要更强隔离,可让 PC 用**不同 domain** + 在机器人上跑单向中继
   (仓库里有 `map_domain_relay`),但那多一层静默故障源,不推荐

### B.6 方案 B 的优缺点

| 优点 | 缺点 |
|---|---|
| 拿到**真数据**,rviz 可交互(发目标点、看 TF 细节) | 需要第二张网卡 + 物理接线 |
| 渲染在 PC,不占机器人 GPU | PC 成为业务网段参与者,暴露面变大 |
| 多人可各自连接 | 大点云需调缓冲,否则静默丢包 |
| 可用 PC 上的工具链(rqt、plotjuggler、录包) | |
| **不需要改厂商任何文件** | |

---

## 附 · 两个方案的选择

| | 方案 A(NoMachine) | 方案 B(接入业务网段) |
|---|---|---|
| 今天可用 | ✅ | ❌ 等网卡/接线 |
| 改动风险 | 零 | 零(不改厂商文件),但拓扑变了 |
| 数据保真 | 画面 | 真数据 |
| 交互能力 | 完整(在机器人桌面里) | 完整,且能用 PC 工具链 |
| 网络依赖 | WiFi 带宽(画面流) | 千兆有线 |
| 安全暴露面 | 不变 | PC 进入业务域 |

**建议:A 先用起来,B 作为长期方案并行推进。** A 零风险且立刻见效;B 才是
正确拓扑,而且 rviz 可交互、能配合 PC 上的调试工具。

---

## C · 一个必须记住的临时替代

上次全链路验证时 `odom→astribot_torso_base` 用的是"把 SLAM 位姿转发成 odom"
(LIO-only 拓扑),**因为厂商本体运动服务未启动**。它转发的是 SLAM **实测真实位姿**,
不是编造数值,但**不是生产拓扑**:

```
生产：odom 来自 SDK（chassis_odom_node），局部连续不跳
      map→odom 吸收 SLAM 的回环跳变（REP-105 分解）
临时：odom 直接等于 SLAM 位姿
      → 回环修正直接反映到 odom 上，而 odom 的契约是不允许跳
      → local_costmap（global_frame: odom）会随之瞬移
      验证数据链路够用，长期运行不行
```

这也解释了当时 `map→odom≈(0,0,0)` 且跳变 0 —— odom 就是 SLAM 位姿,分解结果
理应恒等,那本身是数学自证,不代表分解逻辑没在工作。

要恢复生产拓扑,需要先启动厂商本体运动服务(D1)。
