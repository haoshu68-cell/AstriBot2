# 教程:从 PC 通过 VNC 连到机器人做可视化

**目标** 在开发 PC 上看到机器人的 rviz(建图 / 导航 / 探索)
**适用** Astribot S1 + Jetson Orin(`aarch64`),Ubuntu 22.04
**日期** 2026-08-31 · 本文所有命令与报错都是实测,来源已标注

---

## 0 · 一句话概括

**画面在机器人上渲染,经 ssh 隧道传到 PC。**
VNC 服务只监听 `127.0.0.1`,WiFi 和业务网段上都**不开任何新端口**。

```
PC (10.249.22.160)                    机器人 orin
┌──────────────────┐                 ┌────────────────────────────┐
│ remmina          │                 │  rviz2  ──┐                │
│   ↓              │                 │           │ 渲染在 :0      │
│ localhost:5900 ──┼── ssh 隧道 ──────┼→ 127.0.0.1:5900 (x11vnc)   │
└──────────────────┘   (加密)         │  ROS 话题走 loopback DDS    │
                                     └────────────────────────────┘
        ⚠️ 连 localhost:5900，不是 10.249.22.137:5900
           后者是故意连不通的
```

**为什么 rviz 必须跑在机器人上、不能跑在 PC 上**:厂商 Fast DDS 配置
`useBuiltinTransports=false` + `interfaceWhiteList` 只有 `192.168.0.11` 和
`127.0.0.1`,且 iptables 在 `wlP1p1s0` 上 DROP 了 DDS 多播发现地址
`239.255.0.1`。PC 走 WiFi 段**发现不到任何话题**。rviz 与话题同机时 DDS
走 loopback,而 `127.0.0.1` 恰在白名单里。

---

## 1 · 前提

| 项 | 要求 | 怎么确认 |
|---|---|---|
| ssh 免密登录机器人 | 公钥已在机器人上 | `ssh astribot@10.249.22.137 hostname` → `orin` |
| PC 上有 VNC 客户端 | `remmina` | `which remmina` |
| 机器人图形会话在跑 | gdm3 + 物理桌面 `:0` | `ssh astribot@10.249.22.137 'systemctl is-active gdm3'` → `active` |

两个地址分工要记住,别搞混:

- **`10.249.22.137`**(WiFi `wlP1p1s0`)—— **只做 ssh 运维**。VNC 也是从这里进,
  但是**藏在 ssh 隧道里**,不额外开端口。
- **`192.168.0.11`**(有线 `eno1`)—— ROS 业务流量。PC 到不了这个网段。

---

## 2 · 一次性准备(装一次,以后不用再做)

### 2.1 在机器人上装 x11vnc

```bash
ssh astribot@10.249.22.137
sudo apt-get install -y x11vnc
```

只动 1 个包(`0.9.16-8`,来自 `ports.ubuntu.com jammy/universe arm64`),
`apt-get -s` 预演结果是 `1 newly installed, 0 to remove` ——
**不碰任何 ROS 包**,也不会连带升级那几百个 pending 的包。

### 2.2 设 VNC 口令

```bash
x11vnc -storepasswd     # 会交互问两遍，写到 ~/.vnc/passwd
```

> ⚠️ **VNC 协议规定口令最多 8 个字符,第 9 位起被静默截断**。
> 设了长口令而不知道被截断,是个典型的假安全感。
> 真正的防线是 `-localhost` + ssh,不是这 8 个字符。

---

## 3 · 每次连接(三步)

### 步骤 1 — 在机器人上起 VNC 服务

```bash
ssh astribot@10.249.22.137 \
  '/home/astribot/Downloads/astribot_sdk_aarch64/vnc_view.sh'
```

脚本是**幂等**的:已经在跑就只报告、不重起(两个 x11vnc 抢同一端口时,后起的
那个失败但前一个还在,很容易误读成"起不来")。

期望输出:

```
[vnc_view][OK] 物理桌面 :0 可访问
[vnc_view][OK] 已监听 127.0.0.1:5900 [::1]:5900
[vnc_view][OK] 仅回环，网络上不可见
```

最后那行是脚本里的**可执行自检**:起完之后检查 5900 是否只绑在回环上,
发现绑到别的地址就自己杀掉并报错退出。"只绑回环"不是靠注释保证的。

### 步骤 2 — 在 PC 上开 ssh 隧道

```bash
ssh -f -N -L 5900:localhost:5900 astribot@10.249.22.137
```

- `-f` 转后台(命令立刻返回,隧道继续活着)
- `-N` 不执行远程命令,只做端口转发

确认:

```bash
ss -tln | grep 5900
# 期望：LISTEN  127.0.0.1:5900
```

### 步骤 3 — 在 PC 上连

```bash
remmina -c vnc://localhost:5900
```

口令就是 2.2 里设的那个。

图形界面的等价操作:remmina 左上 `+` → 协议选
`VNC - Virtual Network Computing` → 服务器填 `localhost:5900` → 填密码 → 连接。

---

## 4 · 连上之后:起 rviz

在 VNC 桌面里开一个终端:

```bash
/home/astribot/Downloads/astribot_sdk_aarch64/view_chain.sh
```

这个脚本替你处理四个坑:

| 它做什么 | 防的坑 |
|---|---|
| 先查 `DISPLAY` | 没有显示时 rviz 抛 Ogre GLX 异常并 core dump,报错完全不指向"没显示" |
| `set +u` 包住 `source env_robot.sh` | ROS 的 `setup.bash` 引用未定义的 `AMENT_TRACE_SETUP_FILES`,与 `set -u` 冲突 |
| 用 `ros2 pkg prefix` 定位 rviz 配置 | `setup.py` 只 glob `rviz/*.rviz`,放在 `config/` 下的 `.rviz` **不会被安装** |
| 校验 rviz 的 `Tools:` 段 | 见 §7 —— 防误触发运动 |

期望看到 `OpenGl version: 4.6 (GLSL 4.6)` —— 走的是 Orin 的 GPU。

---

## 5 · rviz 里一片空白怎么办

**先分清是 VNC 的问题还是数据的问题**:如果你能看到 rviz 窗口、菜单、坐标网格,
那 VNC 是好的,空白是**没有数据**。

`/map`、点云、代价地图要有内容,机器人上必须先起三层(每层一个独立终端,
**顺序不能颠倒**):

1. 雷达驱动 `livox_ros_driver2`
2. Voxel-SLAM
3. 我们的节点(`cloud_to_grid_node` + `map_odom_tf_node`)

完整命令、每层的环境变量、以及各层的实测频率,见
[pc_rviz_visualization.md](pc_rviz_visualization.md) 的 §A.4 —— 那里是单一来源,
这里不复制,避免两处漂移。

另外两个已知的"看不到"原因:

- **`RobotModel` 显示不出来**是预期的。它需要 `/joint_states`,而那要 SDK 读关节角,
  依赖厂商本体运动控制服务(当前未启动)。配置里这个 Display 默认是关的。
- **`/map` 和代价地图必须用 `Durability = Transient Local`**。设成 Volatile
  会**一片空白且无任何报错** —— `chain_view.rviz` 里已经设好了,手点的话别漏。

---

## 6 · 故障排查

按这个顺序查,每一步都有明确判据:

### a) 隧道断了(最常见)

```bash
ss -tln | grep 5900        # 没输出就是断了
ssh -f -N -L 5900:localhost:5900 astribot@10.249.22.137   # 重建
```

### b) 隧道在,但打不到对端

```bash
timeout 5 python3 -c "import socket;s=socket.create_connection(('127.0.0.1',5900),timeout=4);print(s.recv(12))"
# 期望：b'RFB 003.008\n'   ← 拿到这个就说明真的打到了另一端的 x11vnc
```

比"能 telnet 上"强:ssh 隧道的本地端口**即使对端已死也照样 LISTEN**,
只有真读到 RFB 版本串才证明链路是通的。

### c) 机器人侧 x11vnc 挂了

```bash
ssh astribot@10.249.22.137 'pgrep -a -x x11vnc'
ssh astribot@10.249.22.137 '/home/astribot/Downloads/astribot_sdk_aarch64/vnc_view.sh'
```

### d) 连 `10.249.22.137:5900` 连不上

**这是正确行为,不是故障。**网络上没开这个端口。要连 `localhost:5900`。

```bash
# 验证它确实不可达（应该失败）
timeout 5 bash -c 'cat < /dev/null > /dev/tcp/10.249.22.137/5900' \
  && echo "意外：暴露了" || echo "正常：连接被拒绝"
```

### e) 卡顿

x11vnc 没有 NoMachine 那套编码器,**点云满屏刷新时会明显卡**。按代价从低到高:

1. 关掉 `SLAM Scan (/map_scan_filtered)` 这个 Display(最吃带宽)
2. 把 rviz 的 `Frame Rate` 从 15 降到 5(Global Options 里)
3. x11vnc 加 `-ncache 10`(会有视觉残影,是已知副作用)

---

## 7 · 安全边界(必读)

### rviz 的默认工具栏能让机器人动起来

**rviz2 的配置里没有 `Tools:` 段时会装载默认工具集,其中
`rviz_default_plugins/SetGoal`(工具栏上的 "2D Goal Pose")往 `/goal_pose`
发目标 —— nav2 起着的话,在"只看不动"的会话里点错一下机器人就走了。**

`chain_view.rviz` 显式声明了 `Tools:`,只留四个不发布任何话题的工具:
`MoveCamera` / `Select` / `FocusCamera` / `Measure`。
`view_chain.sh` 每次启动都校验,不满足就拒绝启动。

这是 **UI 层防误触,不是硬联锁**。

### VNC 给的是完整桌面

桌面里当然能开终端做任何事 —— 这一点和 NoMachine 没有区别,
运动闸门不在这一层。真正的闸门是:

- 不启动 `cmd_vel` 那一端(`cmd_vel_body_to_world_node` / 底盘桥)
- 只用 `ComputePathToPose`(纯规划,不发速度),不用 `NavigateToPose`
- 厂商本体运动服务未启动 → SDK 拒绝一切写操作

### 清理进程时的一个陷阱

```bash
# ❌ 不要这样 —— 这个模式出现在你自己的命令行里，pkill 会把当前 shell 一起杀掉
pkill -f "ssh -N -L 5900:localhost:5900"

# ✅ 用不含完整模式的查法，拿到 pid 再 kill
pgrep -a -f 'L 5900'
kill <pid>
```

停 VNC 与 rviz:

```bash
ssh astribot@10.249.22.137 'pkill -x x11vnc; pkill -x rviz2'
```

---

## 8 · 附:为什么不用别的方案(都实测过)

### ❌ 在 PC 上直接跑 rviz2

三层阻断,见 §0。PC 发现不到任何话题。

### ❌ `ssh -X` / `ssh -Y`

X 转发本身是通的(远端 `DISPLAY=localhost:10.0` 正确设置),**卡在 GL**:

```
InvalidParametersException: Window with name 'OgreWindow(0)' already exists
  in GLRenderSystem::_createRenderWindow
Unable to create the rendering window after 100 tries  → core dump
```

根因:现代 Xorg 默认关闭 indirect GLX(要 `+iglx` 才开),而 rviz 的 Ogre
必须拿到直接 GLX 上下文。

同理 `QT_QPA_PLATFORM=offscreen` **也救不了** —— Ogre 绕过 Qt 直接开 GLX,
报 `Couldn't open X display in GLXGLSupport::getGLDisplay` 并 core dump。
看到这个报错不要往 Qt/配置方向查,就是没有 DISPLAY。

### ❌ NoMachine(机器人上已装 8.13.1,但当前坏)

登录成功之后,服务端自己的会话启动流程失败,五次尝试完全一致:

```
NXSERVER User 'astribot' logged in ... NX-password.              ← 认证通过
NXSERVER ERROR! Wrong session type physicalDesktop. Cannot set limits for this session.
NXSERVER ERROR! Cannot specify is local node base on ':'
                (SessionStartStateMachine.pm, 1258, NXNodes::isRemoteNode)
→ 客户端看到 "The connection with the server was lost."
```

第二条是致命的:服务端判断 node 是本地还是远程时,拿到的地址是**一个裸冒号** ——
host 和 port 都是空的。

时间线指向那次重启:8 月 28 日还有一堆正常的长会话(来自 `192.168.0.100`),
之后重启,日志里留下 `Server '7037...' finished without cleaning`,
且**开机时钟是 1970**(PTP 同步之前),`/usr/NX/var/db/server/` 下
`cookie`/`port`/`redis` 三个文件时间戳全是 `Jan 1 1970`。

已排除:`server.cfg` 与 `node.cfg` 的 `AvailableSessionTypes` **完全一致**;
当前 4000 端口零个外部连接(不是连接数超限)。
待验:`/usr/NX/var/db/limits/` **整个目录不存在**,而报错第一条正是
"Cannot set limits"。

> ⚠️ "1970 时钟污染了内部状态库导致 node 地址变空"是**推断**,不是已验证的事实。
> 若推断成立,则**每次重启都会复发** —— 这也是不把 NoMachine 当主路的原因。
> 修复思路(未实施):`sudo /etc/NX/nxserver --restart`,
> 或让 nxserver 在时钟校正后再启动(改厂商 systemd 依赖顺序)。

### ⚠️ gnome-remote-desktop(机器人上已装 42.9,可用但不推荐)

不用装东西,keyring daemon 也在跑。但:

- 监听 **`0.0.0.0:5900`** —— WiFi 和业务网段上**都**暴露,与"WiFi 只做 ssh 运维"冲突
- 无传输加密(GNOME 42 的 VNC 后端)
- 同样 8 字符口令上限,但**直接面对网络**

选 x11vnc + `-localhost` + ssh 隧道,就是为了避开这三条。

---

## 9 · 验证记录(2026-08-31)

| 检查 | 结果 |
|---|---|
| x11vnc 监听地址 | `127.0.0.1:5900` + `[::1]:5900`,无其他 |
| 从网络侧连 `10.249.22.137:5900` | **连接被拒绝**(符合设计) |
| PC 端隧道 | `127.0.0.1:5900` LISTEN |
| 隧道端到端 | 读到 `RFB 003.008`(真打到了对端 x11vnc) |
| rviz2 | `OpenGl version: 4.6 (GLSL 4.6)`,GPU 加速 |
| rviz 工具栏自检 | 通过,无 `SetGoal` / `SetInitialPose` |

> 尚未由人工确认的一环:**remmina 实际连上并看到桌面**。
> 以上验证到 RFB 握手为止,再往后是客户端渲染,需要人眼确认。

---

## 相关文档

- [pc_rviz_visualization.md](pc_rviz_visualization.md) —— 方案 A/B 完整技术方案、数据链路三层启动命令
- [real_robot_deployment_review.md](real_robot_deployment_review.md) —— 实机部署整体状态
- 脚本:`tools/robot/vnc_view.sh`、`tools/robot/view_chain.sh`
