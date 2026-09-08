# 实机部署技术落地方案 · Astribot S1 / Orin

> 目标机：`astribot@10.249.22.137`（WiFi 运维口）· 业务口 `eno1 = 192.168.0.11` · `ROS_DOMAIN_ID=25`（全栈统一值；现场曾给 42，见 §1.3）
> 适用分支：`chassis-effort-drive`（HEAD `8a3c3a9`）
> 本文配套 [`docs/sim_real_alignment.md`](sim_real_alignment.md)：那份定"对齐什么"，这份定"怎么装上去"。
>
> **本方案范围严格限定为：部署、配置、环境校验、状态诊断。全文不含任何使机器人本体移动或夹爪动作的命令。**
> 会引起运动的 launch / 参数 / 服务 / 示例，集中列在 §4.9 禁止清单里，作为**反向清单**使用。

---

## 0 · 先读：四条会改变方案形态的既有事实

这四条都不是推测，是本次勘察在仓库里逐条取证的结果。任何绕过它们的部署步骤都会在半路失败。

### 0.1 仓库里的厂商 SDK 全部是 x86-64，Orin 是 aarch64

```
astribot_sdk/core/common/robotics_library_py/_robotics_library_py.so   ELF 64-bit, x86-64
astribot_sdk/core/common/robotics_library_py/libdrake.so.2025113_1636  ELF 64-bit, x86-64
astribot_msgs/lib/libastribot_msgs__rosidl_typesupport_*.so            ELF 64-bit, x86-64  (8 个全部)
```

结论：

* `astribot_sdk/`、`third_party/`（drake / pinocchio / hpp-fcl / robotics_library_py）、`astribot_msgs/lib`
  **一律不下发到 Orin**。闭源 `.so` 我们也无法重编。
* 实机必须使用**机器人自带的原生 aarch64 SDK**。[`env.sh:29-41`](../env.sh#L29-L41) 的
  `ASTRIBOT_MIDDLEWARE_PY` 变量本来就是为这件事留的（注释写明"真机由真机自己的环境提供"）。
* `astribot_msgs` 幸好 ship 了 `CMakeLists.txt` + 72 个 `.msg/.srv/.action` 源定义，
  Orin 上缺失时可原生重编；但**优先用机器人自带的那份**（§1.5）。
* 顺带：`astribot_rclcpp_py_ext_pybind11.cpython-38-*.so` 是 **Python 3.8** ABI，
  而其余 36 个是 `cpython-310`。这条只影响 x86 开发机，记录备查。

### 0.2 桥接层的全部实测数据来自 MuJoCo 后端，真机侧为零

[`sim_real_alignment.md §7.10.6`](sim_real_alignment.md) 明确列出"已在真后端验证"的清单——
那个"真后端"指的是 MuJoCo，不是真机。**本次是这套栈第一次接触真实硬件。**

因此本方案把写入通路全程关闭（§3.6），只做"能起来 + 状态正确"的验证。
打开写入是下一个独立 Gate，需要单独的风险评估，不在本文范围内。

### 0.3 你给的三个网络参数与仓库现状有冲突

| 现场参数 | 仓库现状 | 冲突后果 |
|---|---|---| | [`env.sh:115`](../env.sh#L115) 是 `25`，各 launch 默认也已统一为 `25` | SDK 侧与本栈不同 domain 时，**话题互不可见**，且报错只表现为"节点全都 Node not found"。**仓库这边已统一 25，仍需 §1.3 现场实测确认控制器就在 25** |
| `eno1 = 192.168.0.11` | [`env.sh:142-163`](../env.sh#L142-L163) 的 lan 分支把 `192.168.0.10` 当"本机即机器人"，其余 IP 一律按"远程工作站"处理 | Orin 是 `.11` → 走错分支 → 生成的白名单 XML 带 `useBuiltinTransports=false`，**丢掉共享内存**，Livox 点云退化为回环 UDP |
| `ROS_IP` 固定用 eno1 | 仓库从不设 `ROS_IP` | `ROS_IP` 是 **ROS1 变量，ROS2/Fast DDS 完全忽略**。等价手段是 Fast DDS `interfaceWhiteList`（§3.2） |

### 0.4 `astribot_config` 是构建期硬依赖，不是可选数据

[`astribot_s1_description/CMakeLists.txt:30-46`](../ws_robot/src/astribot_s1_description/CMakeLists.txt#L30-L46)
用**相对路径上溯三层**去取 mesh，找不到就 `FATAL_ERROR`：

```
ws_robot/src/astribot_s1_description/CMakeLists.txt
../../../astribot_config/robot_config/astribot_s1/meshes          ← 必须存在
../../../astribot_config/robot_config/astribot_s1/model/meshes/s1_gripper  ← 必须存在
```

所以 `astribot_config/`（178 MB）**必须下发**，且必须保持与 `ws_robot` 的相对目录关系。
URDF 里是 `package://astribot_s1_description/meshes/...`（运行时无绝对路径），
但那些 mesh 是 `colcon build` 阶段从 `astribot_config` 装进 `share/` 的。

---

## 1 · 前置环境校验清单

**执行顺序有意义**：1.3（domain 判定）和 1.5（原生 SDK）是两个"生死项"，
不通过就不要继续往下装。每一步都给出**通过判据**，不要只看命令不报错。

建议先在 Orin 上建一个校验记录文件，边跑边填：

```bash
# 【机器人端执行】
mkdir -p ~/deploy_check && cd ~/deploy_check
{ echo "# Orin 部署前校验 $(date -Is)"; echo; } > report.md
```

### 1.1 SSH 连通性与身份

```bash
# 【本地工作站执行】
ssh -o ConnectTimeout=5 astribot@10.249.22.137 \
  'echo "--- 基本身份 ---"; hostname; whoami; uname -m; uname -r; \
   lsb_release -ds; nproc; free -g | head -2; df -h / | tail -1'
```

**通过判据**：`uname -m` 必须是 `aarch64`（这是 §0.1 整条推论的前提）；
`lsb_release` 应为 Ubuntu 22.04；根分区剩余 ≥ 20 GB（构建 + mesh 安装需要）。

配免密登录，避免后续 rsync / 批量命令反复输密码：

```bash
# 【本地工作站执行】
ssh-keygen -t ed25519 -C "deploy-to-orin" -f ~/.ssh/id_ed25519_orin -N ''
ssh-copy-id -i ~/.ssh/id_ed25519_orin.pub astribot@10.249.22.137
# 写进 ~/.ssh/config，后面所有命令就能用 `ssh orin`
cat >> ~/.ssh/config <<'EOF'

Host orin
    HostName 10.249.22.137
    User astribot
    IdentityFile ~/.ssh/id_ed25519_orin
    ServerAliveInterval 30
    ServerAliveCountMax 6
EOF
```

`ServerAliveInterval` 不是可选项：后面的 `colcon build` 要跑十几分钟，
公司 WiFi 掉一次连接就会连带杀掉编译进程（缓解手段见 §2.5 的 tmux）。

### 1.2 双网卡与路由 metric 校验

```bash
# 【机器人端执行】
echo "--- 地址 ---";      ip -brief addr show
echo "--- 默认路由 ---";  ip route show default
echo "--- 全部路由 ---";  ip route show
echo "--- NM 设备 ---";   nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
echo "--- NM metric ---"
for c in $(nmcli -t -f NAME connection show --active | cut -d: -f1); do
  printf '%-24s ' "$c"
  nmcli -t -f ipv4.route-metric,ipv4.never-default connection show "$c" | tr '\n' ' '; echo
done
```

**通过判据**：

| 检查项 | 期望 |
|---|---|
| `eno1` 地址 | `192.168.0.11/24` |
| `wlP1p1s0` 地址 | `10.249.22.137/x` |
| eno1 metric | 100 |
| wlP1p1s0 metric | 600 |

**逐目标验证实际选路**（比读 metric 可靠，因为它走的是内核真实决策）：

```bash
# 【机器人端执行】
ip route get 192.168.0.10   | head -1   # 期望: dev eno1
ip route get 10.249.22.1    | head -1   # 期望: dev wlP1p1s0
ip route get 223.5.5.5      | head -1   # 出网(apt/git)：期望 dev wlP1p1s0
ip route get 239.255.0.1    | head -1   # DDS 多播默认走哪张卡
```

> ⚠️ **一个必须现场定夺的风险**：你给的 metric 是 eno1=100 / WiFi=600，
> 这意味着 **eno1 上挂着一条 metric 更低的默认路由**。如果 eno1 不通外网
> （通常它只连机器人控制器），那么 `apt` / `git clone` / DNS 会全部尝试从 eno1 出去而超时——
> 表现为"网络明明是通的，装包却卡死"。
>
> 上面第三条 `ip route get 223.5.5.5` 就是判据：**如果它返回 `dev eno1`，必须处置。**
> 推荐处置是让 eno1 不再提供默认路由，而不是去调 metric：
>
> ```bash
> # 【机器人端执行】仅当上面判据命中时
> ENO1_CONN=$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: '$2=="eno1"{print $1}')
> sudo nmcli connection modify "$ENO1_CONN" ipv4.never-default yes
> sudo nmcli connection up "$ENO1_CONN"
> ip route get 223.5.5.5 | head -1   # 复验：应变为 dev wlP1p1s0
> ip route get 192.168.0.10 | head -1 # 复验：仍须是 dev eno1
> ```
>
> 这样 `192.168.0.0/24` 仍走 eno1（链路路由），出网走 WiFi，两者不再互相干扰。
> 注意 **DDS 不依赖这条默认路由**——我们用 Fast DDS 白名单把它钉在 eno1 上（§3.2），
> 这正是不靠路由表来保证业务链路的原因。

**Livox 的第三个网段**（这条大概率会命中）：

```bash
# 【机器人端执行】
ip -4 addr show | grep -q '192\.168\.1\.' \
  && { echo "存在 192.168.1.x："; ip -4 addr show | grep '192\.168\.1\.'; } \
  || echo "⚠️ 没有 192.168.1.x 地址 —— 两颗 Livox 的主机地址缺失"
```

仓库里两颗雷达的配置期望**主机** `192.168.1.5` / `192.168.1.6`，**雷达**
`192.168.1.12` / `192.168.1.13`（[`MID360_config_left.json:14-28`](../ws_robot/src/astribot_s1_perception/config/MID360_config_left.json#L14-L28)），
既不是 eno1 的 `192.168.0.x` 也不是 WiFi 的 `10.249.x`。
[`hardware_livox.launch.py:7-13`](../ws_robot/src/astribot_s1_perception/launch/hardware_livox.launch.py#L7-L13)
自己就注明这些是**占位值**。现场必须确认：雷达挂在哪张网卡、真实 IP 是什么（§3.5）。

```bash
# 【机器人端执行】确认雷达在线（纯 ICMP，不触发任何设备动作）
for ip in 192.168.1.12 192.168.1.13; do
  ping -c2 -W1 "$ip" >/dev/null 2>&1 && echo "$ip 在线" || echo "$ip 不可达"
done
```

### 1.3 【生死项】确认 SDK 后端在哪个 domain

这是最可能让整套栈"看起来全都没起来"的一条。**本栈现已统一到 domain 25**
（与厂商 `env.sh` 一致，见下面的取舍），而 SDK 与机器人控制器之间走的就是 DDS。
历史上仓库里流传过两个值（厂商 25 / 本栈 42），现在只剩 25，但**现场给的参数是 42**，
所以这一步不是"选一个"而是**确认机器人控制器确实在 25**。

```bash
# 【机器人端执行】先确认控制器主机可达
ping -c3 -W1 192.168.0.10 && echo "192.168.0.10 可达" || echo "⚠️ 192.168.0.10 不可达"

# 【机器人端执行】逐 domain 扫描（--no-daemon 避免 ros2 daemon 缓存串味）
source /opt/ros/humble/setup.bash
for d in 25 42; do
  echo "===== ROS_DOMAIN_ID=$d ====="
  ROS_DOMAIN_ID=$d ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    timeout 10 ros2 node list --no-daemon 2>/dev/null | sed 's/^/  node: /'
  ROS_DOMAIN_ID=$d ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    timeout 10 ros2 topic list --no-daemon 2>/dev/null | grep -i astribot | sed 's/^/  topic: /'
done

# 【机器人端执行】看厂商进程是否在跑，以及它自己怎么设的 domain
pgrep -af 'astribot|ast_' | head
for p in $(pgrep -f 'astribot' | head -5); do
  echo "--- pid $p ---"
  tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_|FASTRTPS'
done
```

**通过判据**：有且只有一个 domain 能看到 `astribot` 相关话题/节点。那个值就是全栈必须采用的值。

* 若结果是 **25** → 与本栈现在的默认值一致，什么都不用改（这也是预期结果：
  控制器的 domain 由厂商启动脚本固定、改不动，所以是本栈迁到 25 而不是反过来）。
* 若结果是 **42** → 现场给的参数才是对的，硬件与仓库默认值不一致。domain 只是频道号、
  不提供隔离，改哪边都行，但必须统一：在 `env_robot.sh`（§3.1）里覆盖成 42，
  并给各 launch 传 `domain_id:=42` / `ros_domain_id:=42`。这是需要你现场拍板的第一件事。
* 若两个 domain 都空 → 厂商侧服务没起来，或 DDS 被网卡/防火墙挡住。先查 §6.1。

### 1.4 ROS 2 环境与依赖版本

```bash
# 【机器人端执行】
source /opt/ros/humble/setup.bash
echo "ROS_DISTRO      = $ROS_DISTRO"
echo "python3         = $(python3 --version 2>&1)"
echo "RMW             = ${RMW_IMPLEMENTATION:-<unset，默认 rmw_fastrtps_cpp>}"
echo "ros-humble 包数 = $(dpkg -l 'ros-humble-*' 2>/dev/null | grep -c '^ii')"
echo "colcon          = $(colcon version-check 2>/dev/null | head -1)"
```

**通过判据**：`ROS_DISTRO=humble`、`python3` 为 3.10.x（middleware 路径写死
`lib/python3.10/site-packages`，见 [`env.sh:29`](../env.sh#L29)）。

逐包核对本栈运行期真正需要的 ROS 包：

```bash
# 【机器人端执行】
for p in \
  rclpy rclcpp robot_state_publisher xacro tf2_ros tf2_tools \
  rmw_fastrtps_cpp \
  moveit_ros_move_group moveit_planners_ompl moveit_simple_controller_manager \
  moveit_kinematics moveit_configs_utils \
  slam_toolbox pointcloud_to_laserscan pcl_conversions \
  nav2_bringup nav2_lifecycle_manager \
  nav2_regulated_pure_pursuit_controller nav2_mppi_controller \
  control_msgs trajectory_msgs nav_msgs sensor_msgs_py \
  ; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then printf 'OK      %s\n' "$p"
  else printf 'MISSING %s\n' "$p"; fi
done
```

> `nav2_*` 与 `moveit_*` 里除 `move_group` 外的部分，本次**不启动**（会运动），
> 但它们是 `package.xml` 的构建依赖，缺了 `colcon build` 会失败，所以要一起查。

SDK 侧的 pip 依赖——**这三个 rosdep 覆盖不到**，只在
[`sdk_session.py:41`](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/sdk_session.py#L41)
的运行期自检里出现：

```bash
# 【机器人端执行】
python3 - <<'PY'
import importlib
for m in ('filterpy', 'tabulate', 'h5py', 'numpy'):
    try:
        mod = importlib.import_module(m)
        print(f'OK      {m:10s} {getattr(mod, "__version__", "?")}')
    except Exception as e:
        print(f'MISSING {m:10s} {type(e).__name__}')
PY
```

**缺失时的安装方式（必须带 `--no-deps`）**：

```bash
# 【机器人端执行】
pip install --no-deps filterpy tabulate h5py
```

> ⚠️ **不要整段跑仓库根目录的 `install.sh`**。它第 78-88 行用
> `pip install -U ... numpy==1.22.4`，**不带 `--no-deps`**，会顶掉被测试锁死的
> numpy 版本；这与 [`sdk_session.py:97-100`](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/sdk_session.py#L97-L100)
> 和桥接包 README 的明确要求直接冲突。它还会往 `~/.bashrc` 追加 source 行（第 59-62 行）。
> 该脚本只有 §1.6 那三行目录创建是实机需要的，单独执行即可。

### 1.5 【生死项】机器人自带的原生 aarch64 SDK

```bash
# 【机器人端执行】
echo "--- 厂商安装目录 ---"
ls -d /opt/astribot* 2>/dev/null || echo "(无 /opt/astribot*)"

echo "--- 原生 .so 架构（必须是 aarch64）---"
find / -xdev -name '_robotics_library_py*.so' 2>/dev/null | while read -r f; do
  printf '%s\n  ' "$f"; file -b "$f" | cut -c1-60
done

echo "--- middleware（Rate/ok/spin，250Hz 内环依赖）---"
find / -xdev -type d -name 'astribot_ros_middleware*' 2>/dev/null | head

echo "--- 已注册的 astribot_msgs ---"
ros2 pkg prefix astribot_msgs 2>/dev/null || echo "(未注册)"

echo "--- SDK python 包位置 ---"
python3 -c "import astribot_sdk, os; print(os.path.dirname(astribot_sdk.__file__))" 2>&1 | tail -1
```

**通过判据**：至少要找到一份 `_robotics_library_py*.so` 且 `file` 报 **ARM aarch64**。

**只导入、不建会话**的验证（这一步绝对不会让机器人动，因为它不创建 `Astribot()` 实例）：

```bash
# 【机器人端执行】把下面两个路径换成上一步实测到的真实位置
export ASTRIBOT_LOG=1 ROBOT_TYPE=S1
export ASTRIBOT_MIDDLEWARE_PY=/实测/astribot_ros_middleware/lib/python3.10/site-packages
export PYTHONPATH="$ASTRIBOT_MIDDLEWARE_PY:/实测/astribot_sdk_root/astribot_sdk/core/common:/实测/astribot_sdk_root:$PYTHONPATH"

python3 - <<'PY'
import sys
try:
    from astribot_sdk.core.astribot_api.astribot_client import Astribot
    print('SDK import OK ->', Astribot.__module__)
except Exception as e:
    print('SDK import FAILED:', type(e).__name__, e)
    sys.exit(1)
PY
```

> **为什么只 import**：`Astribot()` 构造即建立会话并加入 ROS 图。本阶段不需要它。
> 真正需要会话时走 §4.2 的只读状态桥接，那条路径里
> [`sdk_session.py:283`](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/sdk_session.py#L283)
> 把 `high_control_rights` **硬编码为 False**，不可通过参数打开。

失败时按这个顺序查（都是本仓库踩过的真实链条）：

| 报错特征 | 根因 | 处置 |
|---|---|---|
| `No module named robotics_library_py` | `astribot_sdk/core/common` 不在 `PYTHONPATH`；编译过的 `util.py:26` 用**裸名**导入 | 补这一层路径 |
| 同名 `.so` 与包互相遮蔽 | 厂商打包缺陷 | 桥接已内置 `prewarm_robotics_library_py()` 规避 |
| `libdmumps_seq-5.4.so => not found` | 缺原生库 | `sudo apt install libmumps-seq-5.4`（x86 机上已验证有效） |
| `spdlog_ex` + `create log config directory failed` | `/opt/astribot_ros` 不存在 | §1.6 |
| import 无输出直接静默 | `ASTRIBOT_LOG` 未在 import 前置入环境（`astribot_interface.py:34-41` 会 `os.dup2` 重定向 fd） | 先 export 再 import |

### 1.6 权限与 `/opt/astribot_ros`

SDK 的 C++ 日志层把路径写死在 `/opt/astribot_ros/`，目录建不出来就直接
`terminate` 抛 `spdlog_ex`（**不是**降级到 stderr）。

```bash
# 【机器人端执行】先查
ls -ld /opt/astribot_ros /opt/astribot_ros/log /opt/astribot_ros/robot_config 2>&1
[ -w /opt/astribot_ros/log ] && echo "log 可写 OK" || echo "⚠️ log 不可写"

# 【机器人端执行】缺失时补（等价于 install.sh:92-98，需要 sudo）
sudo mkdir -p /opt/astribot_ros/log /opt/astribot_ros/robot_config
sudo chown -R "$USER:$USER" /opt/astribot_ros
```

其余权限项：

```bash
# 【机器人端执行】
groups                                     # 需要 sudo 权限做 §1.6 / §7.2
ulimit -n                                  # 250Hz 多话题，建议 ≥ 4096
ulimit -r                                  # 实时优先级上限（厂商内环可能需要）
sysctl net.core.rmem_max net.core.rmem_default   # 大点云 UDP 收包缓冲
```

Livox 是**以太网设备，不是串口**——全仓库 `ws_robot/src` 内没有任何
`/dev/tty*` 引用，所以不需要 `dialout` 组。

### 1.7 时间同步

```bash
# 【机器人端执行】
timedatectl status
systemctl is-active chrony systemd-timesyncd 2>&1 | paste -sd' '
command -v chronyc >/dev/null && chronyc tracking | head -5
```

**通过判据**：`System clock synchronized: yes`，且 RTC/本地时区一致。

两颗雷达的时钟必须同步，否则
[`livox_fusion_params.yaml:10`](../ws_robot/src/astribot_s1_perception/config/livox_fusion_params.yaml#L10)
的 `sync_slop_sec: 0.05` 时间窗对不上，融合点云**持续丢帧**——
[`README_PERCEPTION.md:211-213`](../ws_robot/src/astribot_s1_perception/README_PERCEPTION.md#L211-L213) 已记录这一条。

还要确认**没有仿真时钟残留**（实机全链路 `use_sim_time=false`）：

```bash
# 【机器人端执行】任何节点起来之后再跑
ros2 topic info /clock    # 期望 Publisher count: 0
```

`/clock` 一旦有发布者，而节点又是 `use_sim_time=false`，TF 会表现为
时间跳变 / 外推失败，且报错完全不提时钟（§6.6）。

### 1.8 校验结果登记表

全部填完再进 §2。任何一行是 ✗ 都要先处置。

| # | 检查项 | 通过判据 | 结果 |
|---|---|---|---|
| 1.1 | SSH + 架构 | `aarch64` / Ubuntu 22.04 / 根分区 ≥20G | ☐ |
| 1.2 | eno1 地址 / metric | `192.168.0.11`，`ip route get 192.168.0.10` → `dev eno1` | ☐ |
| 1.2 | 出网不走 eno1 | `ip route get 223.5.5.5` → `dev wlP1p1s0` | ☐ |
| 1.2 | Livox 网段 | `192.168.1.x` 存在且雷达可达 | ☐ |
| 1.3 | **SDK domain** | 有且仅有一个 domain 能看到 astribot 话题，值已记录 | ☐ |
| 1.4 | ROS 环境 | humble / py3.10 / 上述包全 OK | ☐ |
| 1.4 | pip 依赖 | filterpy·tabulate·h5py 就位，numpy 未被顶版本 | ☐ |
| 1.5 | **原生 SDK** | `.so` 是 aarch64，且 import 成功 | ☐ |
| 1.6 | `/opt/astribot_ros` | log 与 robot_config 存在且可写 | ☐ |
| 1.7 | 时间同步 | synchronized: yes；`/clock` 无发布者 | ☐ |

---

## 2 · 代码下发 / 同步与构建

### 2.1 下发清单：哪些下、哪些绝对不下

| 路径 | 下发 | 理由 |
|---|---|---|
| `ws_robot/src/`（自有包） | ✅ | 主体 |
| `astribot_config/`（178 MB） | ✅ **必须** | §0.4，`description` 构建期 `FATAL_ERROR` 依赖 |
| `config/fastdds_*.xml.template` | ✅ | §3.2 的基础 |
| `docs/` | ✅ 建议 | 现场排障要查 |
| `tools/joy_tools.py` | 可选 | 与本次部署无关 |
| `maps/*`（若要复用既有地图） | ⚠️ 单独传 | 被 `.gitignore` 忽略，git 通路带不走 |
| `env.sh` | ⚠️ 下发但**不直接用** | domain=25 + lan 分支假设 `.10`（§0.3），实机用 §3.1 的 `env_robot.sh` |
| `astribot_sdk/` | ❌ | x86-64 |
| `third_party/`（drake·pinocchio·hpp-fcl·middleware） | ❌ | x86-64 |
| `astribot_msgs/{lib,include,share,local}` | ❌ | x86-64 预编译产物 |
| `astribot_msgs/{msg,srv,action,CMakeLists.txt,package.xml}` | ⚠️ 仅当机器人没有 | 源定义，可原生重编（§2.4 分支 B） |
| 仓库根 `build/ install/ log/` | ❌ | x86-64 产物 |
| `ws_robot/{build,install,log}` | ❌ | 同上 |
| `ws_robot/src/*/{build,install,log}`（嵌套残留 ~100 MB） | ❌ **务必排除** | 源码树里的 x86-64 陈旧产物，见 §2.5 |
| `ws_robot/src/aws-robomaker-small-warehouse-world` | ❌ | 纯仿真世界，且是唯一的构建阻塞项（§2.4） |

### 2.2 下发方式：先解决"git 通路会丢代码"这件事

```bash
# 【本地工作站执行】现状核对
git -C /home/yjh/WorkSpace/astribot_sdk_ros2 status --porcelain | awk '{print $1}' | sort | uniq -c
git -C /home/yjh/WorkSpace/astribot_sdk_ros2 remote -v
```

实测结果：**30 个未跟踪条目 + 10 个已修改文件，且没有配置任何 remote。**
其中未跟踪的部分包含**整个桥接层**（`astribot_trajectory_bridge` 的 13 个 py 模块、
`astribot_bridge_msgs` 整包、`livox_ros_driver2`）——也就是这次部署最核心的代码。

> ⚠️ **任何基于 `HEAD`（`8a3c3a9`）的 git 下发都会丢掉桥接层。**
> `git bundle` / `git archive` / 新建 remote 后 push 都一样。必须先处置。

三种通路，按可追溯性排序：

**方式 A · 提交后走 git remote（推荐，生产部署应当如此）**

```bash
# 【本地工作站执行】
cd /home/yjh/WorkSpace/astribot_sdk_ros2
git add ws_robot/src/astribot_trajectory_bridge ws_robot/src/astribot_bridge_msgs \
        ws_robot/src/astribot_s1_gazebo_bringup ws_robot/src/astribot_s1_navigation \
        ws_robot/src/astribot_s1_perception env.sh
git status --short                       # 复核范围，别把 .coverage / 嵌套 build 加进去
git commit -m "桥接层 + 实机部署配置入库"
git remote add origin <公司 GitLab 地址>
git push -u origin chassis-effort-drive
```

```bash
# 【机器人端执行】
cd ~ && git clone -b chassis-effort-drive <公司 GitLab 地址> astribot_sdk_ros2
cd astribot_sdk_ros2 && git submodule update --init ws_robot/src/livox_ros_driver2
git rev-parse --short HEAD > ~/deploy_check/deployed_sha.txt   # 留痕，排障时第一件要问的事
```

注意 `astribot_config/`（178 MB）是否在库内。若它不在版本库里，仍需用方式 C 单独同步。

**方式 B · git bundle（无服务器，离线单文件）**

```bash
# 【本地工作站执行】提交之后
git bundle create /tmp/astribot.bundle --all
scp /tmp/astribot.bundle orin:/tmp/
# 【机器人端执行】
git clone -b chassis-effort-drive /tmp/astribot.bundle ~/astribot_sdk_ros2
```

同样受"未提交内容不进 bundle"的约束。

**方式 C · rsync（最快，能带走未提交内容；但丢版本信息）**

```bash
# 【本地工作站执行】先 --dry-run 看清范围，确认无误再去掉它
rsync -avhn --delete \
  --exclude='.git/' \
  --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='__pycache__/' --exclude='*.pyc' --exclude='.coverage' \
  --exclude='astribot_sdk/' \
  --exclude='third_party/' \
  --exclude='astribot_msgs/lib/' --exclude='astribot_msgs/include/' \
  --exclude='astribot_msgs/share/' --exclude='astribot_msgs/local/' \
  --exclude='ws_robot/src/aws-robomaker-small-warehouse-world/' \
  /home/yjh/WorkSpace/astribot_sdk_ros2/ \
  orin:~/astribot_sdk_ros2/
```

`--exclude='build/'` 等是**无锚定**模式，会同时命中仓库根和
`ws_robot/src/*/build`，正好覆盖 §2.5 那 100 MB 嵌套残留。

`--delete` 会删除目标端多余文件，第一次同步或目标端有手工改动时**务必先看 `-n` 的输出**。

地图单独传（被 gitignore，且 `.posegraph` 单个 18 MB）：

```bash
# 【本地工作站执行】仅当要复用既有地图
rsync -avh /home/yjh/WorkSpace/astribot_sdk_ros2/maps/ orin:~/astribot_sdk_ros2/maps/
```

### 2.3 构建前置

**① Livox 驱动源码准备**（仓库自带脚本，**不要**用 Livox 官方 `build.sh`）

```bash
# 【机器人端执行】
cd ~/astribot_sdk_ros2/ws_robot
bash src/astribot_s1_perception/scripts/prepare_livox_driver2.sh
ls -l src/livox_ros_driver2/package.xml src/livox_ros_driver2/launch | head
```

驱动包只 ship 了 `package_ROS1.xml` / `package_ROS2.xml`，该脚本负责生成 `package.xml`
与 `launch/`。**官方 `build.sh` 里有 `rm -rf ../../build/ ../../install/`**，
在本工作空间里正好命中 `ws_robot/build` 与 `ws_robot/install`，
会连带删掉其他所有包的产物——脚本注释里已写明这就是它存在的原因。

**② Livox-SDK2（原生库，任何 package.xml 都没声明它）**

[`livox_ros_driver2/CMakeLists.txt:249`](../ws_robot/src/livox_ros_driver2/CMakeLists.txt#L249)
是 `find_library(... /usr/local/lib REQUIRED)`，缺了直接构建失败。

```bash
# 【机器人端执行】走 WiFi 拉代码
cd ~ && git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir -p build && cd build
cmake .. && make -j"$(nproc)" && sudo make install
sudo ldconfig
ls -l /usr/local/lib/liblivox_lidar_sdk_shared.so   # 通过判据
```

**③ 排除唯一的构建阻塞包**

```bash
# 【机器人端执行】
touch ~/astribot_sdk_ros2/ws_robot/src/aws-robomaker-small-warehouse-world/COLCON_IGNORE
```

`aws-robomaker-small-warehouse-world` 的 `CMakeLists.txt:11` 是
`find_package(gazebo_ros REQUIRED)`（Gazebo **Classic**），在没有 Gazebo 的实机上必然失败。
它是**唯一**的硬阻塞——`astribot_s1_gazebo_bringup` 自身只 `find_package(ament_cmake)`，
所有 gz 依赖都是 `exec_depend`，**能正常构建**。

> **不要把 `astribot_s1_gazebo_bringup` 也 ignore 掉。**
> [`hardware_livox.launch.py:45-47`](../ws_robot/src/astribot_s1_perception/launch/hardware_livox.launch.py#L45-L47)
> 在**运行时**用 `FindPackageShare('astribot_s1_gazebo_bringup')` 去取
> `astribot_s1_controllers.yaml`——实机 launch 依赖仿真包的 `share/`。
> 这是既有的耦合，本次按"保留构建"处理；清理方案见 §3.7。

**④ rosdep**

```bash
# 【机器人端执行】
sudo rosdep init 2>/dev/null; rosdep update
cd ~/astribot_sdk_ros2/ws_robot
rosdep install --from-paths src --ignore-src -y --rosdistro humble \
  --skip-keys "gazebo_ros gazebo gazebo_plugins ros_gz_sim ros_gz_bridge gz_ros2_control livox_ros_driver2"
```

`--skip-keys` 里的 gz 系列是仿真 `exec_depend`，实机不装；`livox_ros_driver2`
是工作空间内的包，rosdep 不该去外部找它。

> rosdep **不覆盖** `filterpy` / `tabulate` / `h5py`（§1.4）——它们只出现在
> `install.sh` 和运行期自检里，必须手工装。

### 2.4 构建

先确定 `astribot_msgs` 走哪个分支：

```bash
# 【机器人端执行】
ros2 pkg prefix astribot_msgs && echo "→ 分支 A：用机器人自带的" \
  || echo "→ 分支 B：需要从源定义重编"
```

**分支 A（推荐）**：机器人已注册 `astribot_msgs`，什么都不用做。
**分支 B**：把源定义软链进工作空间再编（**不要**拷 `lib/`，那是 x86-64）：

```bash
# 【机器人端执行】仅分支 B
ln -s ~/astribot_sdk_ros2/astribot_msgs ~/astribot_sdk_ros2/ws_robot/src/astribot_msgs
cd ~/astribot_sdk_ros2/ws_robot
colcon build --symlink-install --packages-select astribot_msgs
```

**分层构建**（一次全量失败会淹没真正的错误；分层能定位到层）：

```bash
# 【机器人端执行】
cd ~/astribot_sdk_ros2/ws_robot
source /opt/ros/humble/setup.bash

# 层 1：接口。后面所有包都依赖它
colcon build --symlink-install --packages-select astribot_bridge_msgs

# 层 2：桥接层（实机控制通路，零 Gazebo 依赖）
source install/setup.bash
colcon build --symlink-install --packages-up-to astribot_trajectory_bridge

# 层 3：模型 + 规划（这一层会触发 astribot_config mesh 安装，§0.4）
colcon build --symlink-install \
  --packages-select astribot_s1_description astribot_s1_moveit_config astribot_s1_manipulation

# 层 4：感知 / 自主 / 导航配置 / 雷达驱动
colcon build --symlink-install \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  --packages-select livox_ros_driver2 astribot_s1_perception astribot_s1_autonomy \
                    astribot_s1_navigation astribot_s1_dynamics_coupling \
                    astribot_s1_gazebo_bringup astribot_s1_chassis_effort_drive
```

**编译注意点**

| 项 | 说明 |
|---|---|
| `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble` | **强制**。`livox_ros_driver2/CMakeLists.txt` 用这两个开关分支 ROS1/ROS2，缺了会编成 ROS1 形态 |
| `--symlink-install` | 仓库既有约定（`ws_robot/README.md:75`）。注意 `astribot_trajectory_bridge/setup.cfg` 用了旧式 `script_dir`，在新 setuptools 上会刷 deprecation 警告——是警告，不是错误 |
| Orin 内存 | 4 个包真正编 C++（`astribot_s1_manipulation` 带 MoveIt/OMPL/Eigen，`astribot_s1_autonomy` 带 PCL）。Orin 上并行度过高会 OOM，建议 `--parallel-workers 2 --executor sequential` |
| 首次构建耗时 | MoveIt + PCL 模板重，预留时间；务必在 tmux 里跑（§2.5） |
| mesh 安装 | 层 3 若报 `找不到 mesh 源目录`，就是 `astribot_config` 没同步或相对路径不对（§0.4） |

```bash
# 【机器人端执行】低内存构建变体
colcon build --symlink-install --parallel-workers 2 --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

**构建通过判据**

```bash
# 【机器人端执行】
colcon list --names-only | sort > /tmp/pkgs_src.txt
ls install/ | sort > /tmp/pkgs_built.txt
diff /tmp/pkgs_src.txt /tmp/pkgs_built.txt || true    # 差集应只有被 ignore 的 aws 世界包
grep -rn "error:" log/latest_build/*/stdout_stderr.log 2>/dev/null | head
ls install/astribot_s1_description/share/astribot_s1_description/meshes   # mesh 已装
ros2 pkg executables astribot_trajectory_bridge   # 期望含 bridge_container / state_bridge_node
ros2 pkg executables livox_ros_driver2            # 期望含 livox_ros_driver2_node
```

### 2.5 编译缓存清理

```bash
# 【机器人端执行】常规重编（改了 CMakeLists / package.xml / xacro 安装内容后必做）
cd ~/astribot_sdk_ros2/ws_robot
rm -rf build install log

# 【机器人端执行】只重编单个包
rm -rf build/astribot_s1_manipulation install/astribot_s1_manipulation
colcon build --symlink-install --packages-select astribot_s1_manipulation
```

**源码树内的嵌套残留**（本地工作站上实测存在约 100 MB，全是 x86-64 产物）：

```bash
# 【机器人端执行】核查是否被误同步进来
find ~/astribot_sdk_ros2/ws_robot/src -maxdepth 2 \
     \( -name build -o -name install -o -name log \) -type d -print
# 若有输出，删掉——它们不属于源码
find ~/astribot_sdk_ros2/ws_robot/src -maxdepth 2 \
     \( -name build -o -name install -o -name log \) -type d -exec rm -rf {} +
```

这些目录里带 `COLCON_IGNORE`，所以不会污染构建，但会：
① 让 `grep` 命中过时的旧版 launch/配置副本（`astribot_s1_manipulation/install/` 下就有一份
`planning_demo.launch.py` 与现行文件不一致）；② 白占 100 MB；③ 在 aarch64 机器上完全无用。

**其他缓存**

```bash
# 【机器人端执行】
find ~/astribot_sdk_ros2 -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null
rm -rf ~/.ros/log/*            # ros2 launch 的历史日志，会长期累积
ros2 daemon stop               # 换 domain / 换网络配置后必做，否则 ros2 CLI 读到旧缓存
```

> `ros2 daemon stop` 这一条在 §1.3 换 domain 扫描前后特别重要——
> daemon 会缓存发现结果，导致"命令输出与实际图不符"。

**始终在 tmux 里跑构建和后续启动**，避免 WiFi 掉线杀掉进程：

```bash
# 【机器人端执行】
tmux new -s build      # 断线后 tmux attach -t build 回来
```

---

## 3 · 参数 / URDF / SRDF / launch 配置部署

### 3.1 环境脚本：新增 `env_robot.sh`，不改 `env.sh`

`env.sh` 有三处与实机不符（§0.3），且它同时被 x86 开发机使用。
**不要在 Orin 上改它**，而是新增一份实机专用脚本——这样两边可以各自演进，
`git pull` 也不会反复冲突。

```bash
# 【机器人端执行】按 §1.5 实测到的真实路径填写下面两个变量
cat > ~/astribot_sdk_ros2/env_robot.sh <<'EOF'
#!/usr/bin/env bash
# =====================================================================
# Astribot S1 / Orin 实机环境（aarch64）
#
# 与 env.sh 的区别，逐条都有原因：
#   1) 不挂仓库内的 astribot_sdk / third_party —— 那些是 x86-64（见 docs/real_robot_deployment.md §0.1）
#   2) ROS_DOMAIN_ID 取 §1.3 实测值（预期 25，与 env.sh 一致）
#   3) 不走 env.sh 的 lan 分支：它把非 192.168.0.10 的机器当远程工作站，
#      生成的白名单 XML 带 useBuiltinTransports=false，会丢掉共享内存
#   4) ROS_IP 不设 —— 那是 ROS1 变量，ROS2/Fast DDS 不读它；
#      业务网卡绑定靠 FASTRTPS_DEFAULT_PROFILES_FILE
# =====================================================================
SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 必填：§1.5 实测到的机器人原生 SDK 位置 ----------------------------
export ASTRIBOT_SDK_ROOT="__填写：机器人原生 SDK 根__"
export ASTRIBOT_MIDDLEWARE_PY="__填写：.../astribot_ros_middleware/lib/python3.10/site-packages__"

for p in "$ASTRIBOT_SDK_ROOT" "$ASTRIBOT_MIDDLEWARE_PY"; do
    [ -d "$p" ] || echo "[env_robot][ERROR] 路径不存在: $p"
done

export PYTHONPATH="${ASTRIBOT_MIDDLEWARE_PY}:${ASTRIBOT_SDK_ROOT}/astribot_sdk/core/common:${ASTRIBOT_SDK_ROOT}:${PYTHONPATH}"

# ---- SDK 运行期约定 ---------------------------------------------------
export ROBOT_TYPE='S1'
export ASTRIBOT_LOG=1          # 必须在 import SDK 之前生效（astribot_interface.py 会 dup2 重定向 fd）

# ---- ROS 2 -----------------------------------------------------------
source /opt/ros/humble/setup.bash
[ -f "${SDK_ROOT}/ws_robot/install/setup.bash" ] && source "${SDK_ROOT}/ws_robot/install/setup.bash"

export ROS_DOMAIN_ID=25        # ← 全栈统一值，与 env.sh:115 一致；若 §1.3 实测是 42 就改成 42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0    # 必须 0：SDK 与控制器之间的 DDS 要跨机走 eno1
unset IGN_IP GZ_IP             # Gazebo 专用，实机无意义

# ---- 业务网卡绑定（替代 ROS_IP）---------------------------------------
export FASTRTPS_DEFAULT_PROFILES_FILE="${SDK_ROOT}/config/fastdds_orin_eno1.xml"
[ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ] || \
    echo "[env_robot][ERROR] 缺 Fast DDS 配置: $FASTRTPS_DEFAULT_PROFILES_FILE"

echo "[env_robot] DOMAIN=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[env_robot] FASTRTPS_PROFILES = $FASTRTPS_DEFAULT_PROFILES_FILE"
EOF
chmod +x ~/astribot_sdk_ros2/env_robot.sh
```

### 3.2 Fast DDS 多网卡隔离（本节替代"设 ROS_IP"）

**必须先明确**：`ROS_IP` 是 ROS1 变量，ROS 2 / Fast DDS **完全忽略**它。
把业务流钉在 eno1 的正确手段是 `interfaceWhiteList`。

仓库既有的 [`config/fastdds_whitelist_192.xml.template`](../config/fastdds_whitelist_192.xml.template)
有一处不适合实机：它设了 `<useBuiltinTransports>false</useBuiltinTransports>`
但**只挂了一个 UDPv4 传输**，等于**关掉共享内存（SHM）**。
实机上所有业务节点都在 Orin 本机，Livox 点云是大消息，丢掉 SHM 会明显掉性能。

```bash
# 【机器人端执行】
cat > ~/astribot_sdk_ros2/config/fastdds_orin_eno1.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<!--
  Orin 实机 Fast DDS profile
    · UDPv4 只允许 192.168.0.11（eno1）—— 业务流不上 WiFi
    · 同时保留 SHM —— 本机节点间的大点云走共享内存，不退化成回环 UDP
      （仓库原模板只挂 UDP 且关了 builtin，等于丢掉 SHM）
-->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

  <transport_descriptors>
    <transport_descriptor>
      <transport_id>eno1_udp</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>192.168.0.11</address>
      </interfaceWhiteList>
    </transport_descriptor>

    <transport_descriptor>
      <transport_id>local_shm</transport_id>
      <type>SHM</type>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="astribot_orin_eno1" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports>
        <transport_id>local_shm</transport_id>
        <transport_id>eno1_udp</transport_id>
      </userTransports>
    </rtps>
  </participant>

</profiles>
EOF
```

**诚实的边界说明**：`interfaceWhiteList` 只过滤**单播** locator，
**不阻止 SPDP 发现报文从其他物理网卡多播出去**——本仓库在 x86 机上用
`/proc/net/igmp` 逐网卡数多播加入次数实测过这个现象（见 [`env.sh:118-133`](../env.sh#L118-L133) 的注释）。
所以上面的配置保证的是"**业务数据**不上 WiFi"，不是"WiFi 上一个包都没有"。

验证（这是判据，不要只看配置文件写了什么）：

```bash
# 【机器人端执行】起了节点之后再跑。WiFi 上应当**没有** DDS 数据流
sudo timeout 15 tcpdump -ni wlP1p1s0 -c 20 'udp and portrange 7400-7700' \
  && echo "⚠️ WiFi 上有 DDS 流量" || echo "WiFi 无 DDS 数据流 OK"

# 【机器人端执行】eno1 上应当**有**
sudo timeout 15 tcpdump -ni eno1 -c 20 'udp and portrange 7400-7700' | tail -5

# 【机器人端执行】逐网卡看多播加入情况
cat /proc/net/igmp
```

若需要**更强隔离**（完全不在 WiFi 上做发现），可加不依赖多播的单播初始对等体。
代价是必须枚举所有对端，新增机器要改配置：

```xml
  <!-- 可选加固：放进上面 <participant><rtps> 里。本次未实测，启用后需按上面 tcpdump 复验 -->
  <builtin>
    <initialPeersList>
      <locator><udpv4><address>192.168.0.10</address></udpv4></locator>
      <locator><udpv4><address>192.168.0.11</address></udpv4></locator>
    </initialPeersList>
  </builtin>
```

### 3.3 `use_sim_time`：仿真配置与实机配置的分界

这套栈的 sim/real 分叉**不在 xacro 里，而在 launch 层**。没有 `use_gazebo` / `use_fake_hardware` 之类的开关。

| 轴 | 仿真路径 | 实机路径 |
|---|---|---|
| 感知 | `sim_perception.launch.py`（`use_sim_time` 默认 true） | `hardware_perception.launch.py`（默认 false）+ `hardware_livox.launch.py`（硬编码 False） |
| 选择器 | `perception_slam_bringup.launch.py` 的 `env:=sim` | 同文件 `env:=hardware`，第 187 行据此推导 `use_sim_time` |
| 控制 | `warehouse_sim.launch.py` → gz_ros2_control | `astribot_trajectory_bridge`（`bridge_bringup.launch.py`） |

**能用 launch 参数覆盖的，就不要改 yaml**（改 yaml 会在 `git pull` 时冲突，且容易漏）：

| 文件 | 位置 | 实机处置 |
|---|---|---|
| `astribot_s1_moveit_config/launch/move_group.launch.py` | L53 默认 `true` | 启动时传 `use_sim_time:=false`（§4.6） |
| `astribot_s1_perception/**` | 已由 `env:=hardware` 推导 | 无需改 |
| `astribot_s1_manipulation/config/manipulation_params.yaml` | L14 `use_sim_time: true` | 本次不启动执行层；启用前需改 |
| `astribot_s1_navigation/config/nav2_params_rpp.yaml` | L19,69,80,119,127,139,143,158,182,243 | **本次不启动**（nav2 会发 `/cmd_vel`）。且 `navigation.launch.py:76` 用 `RewrittenYaml` 覆盖它，届时传 launch 参数即可 |
| `astribot_s1_navigation/config/nav2_params_mppi.yaml` | L13,163,174,211,219,229,233,237,248,269,336 | 同上 |

```bash
# 【机器人端执行】部署后自查：实机不该有任何节点报 use_sim_time=true
# （节点起来之后跑，§5.4 有完整版本）
for n in $(ros2 node list); do
  v=$(ros2 param get "$n" use_sim_time 2>/dev/null | awk '{print $NF}')
  [ "$v" = "True" ] && echo "⚠️ $n use_sim_time=True"
done; echo "扫描完成"
```

### 3.4 URDF / SRDF：实机要不要改

**结论：本次的只读 / 诊断部署，URDF 不需要改。**
`robot_state_publisher` 只关心 link/joint 几何，会忽略它不认识的
`<ros2_control>` / `<gazebo>` 标签——
[`hardware_livox.launch.py:49-51`](../ws_robot/src/astribot_s1_perception/launch/hardware_livox.launch.py#L49-L51)
的注释已明确记录这一点。缺 Gazebo 时那些标签是惰性的，不报错。

但下面这些**在打开执行通路之前必须处理**，现在先登记：

| 文件 · 位置 | 内容 | 实机含义 |
|---|---|---|
| `astribot_s1_ros2_control.xacro:63` | `<plugin>gz_ros2_control/GazeboSimSystem</plugin>` | 模型里最"仿真"的一行。本栈实机走桥接、不用 ros2_control，所以它只是惰性存在 |
| `astribot_s1_ros2_control.xacro:113-124` | `<gazebo><plugin filename="gz_ros2_control-system">` | 同上 |
| `astribot_s1.gazebo.xacro:71-78` | `OdometryPublisher`（**地面真值 odom**） | ⚠️ 实机不存在此发布者。实机 odom 只能来自 SLAM / 桥接，**不要**有任何配置指望它 |
| `astribot_s1.gazebo.xacro:110-217` | 4 个轮子的 `<surface>` 摩擦/接触参数 | 纯 Gazebo 接触模型，实机无意义 |
| `astribot_s1_sensors.xacro:51-88` | `<sensor type="gpu_lidar">`（Mid-360 的近似） | 实机数据来自 `livox_ros_driver2`；**frame_id `livox_mid360_left/right` 必须保持不变**（`hardware_livox.launch.py:88` 硬编码） |
| `astribot_s1_sensors.xacro:116` | 头部相机位姿 | 文件自注"出生后可在 RViz 里核对朝向再微调"——未标定 |
| `astribot_s1.xacro:57-58` | `wheel_effort_limit=15.0` / `wheel_velocity_limit=40.0` | 自注为未标定的仿真起始值，需真实电机参数 |
| `astribot_s1.srdf:205-206` | `virtual_joint type="fixed" parent_frame="world"` | 刻意不是 `planar`，所以 MoveIt 不需要 `odom→astribot_torso_base`。**代价：导航与操作不能并发**。要并发就得改 `planar` + 可靠 odom TF |
| `moveit_config/config/joint_limits.yaml` | 全部 `max_acceleration` | 全是 `max_velocity / 0.35s` 的估算值，文件自注未标定（L30-35）。**执行前必须在实机上辨识** |

`warehouse_sim.launch.py:485-506` 里那三个**仿真专用静态 TF 别名**
（把 gz 的 `astribot_s1/astribot_torso_base/livox_mid360_left_sensor` 映射回短名）
在实机上不能跑——我们不启动该 launch，天然规避。

> 根 frame 是 `astribot_torso_base`，**没有 `base_link`**。
> 任何配置里出现 `base_link` 都是错的（§5.2 有 TF 校验命令）。

### 3.5 Livox 硬件配置项

必须与现场实测对齐的项（三处，且**互不校验**，改一处不报错但会静默漂移）：

| 位置 | 项 | 当前值（占位） |
|---|---|---|
| `astribot_s1_perception/config/MID360_config_left.json:14-21` | 主机 IP | `192.168.1.5` |
| 同上 `:28` | 雷达 IP | `192.168.1.12` |
| `MID360_config_right.json:14-21` / `:28` | 主机 / 雷达 | `192.168.1.6` / `192.168.1.13` |
| `MID360_config_*.json:31-38` | 外参（**mm / 度**） | left `yaw 22.9, x 280, y 180, z 100`；right `yaw 157.0, x −280, y −180, z 100` |
| `astribot_s1.xacro:36-39` | 挂载位姿（**m / rad**） | `livox_left_xyz/rpy`、`livox_right_xyz/rpy` |
| `astribot_s1_perception/config/livox_extrinsics.yaml` | 同一组外参 | **纯文档，无消费者**（grep 确认），是漂移隐患 |
| `hardware_livox.launch.py:90` | `cmdline_input_bd_code` | `livox0000000001`（占位广播码） |

```bash
# 【机器人端执行】按现场实测改 IP（示例，值需替换）
cd ~/astribot_sdk_ros2/ws_robot/src/astribot_s1_perception/config
cp MID360_config_left.json MID360_config_left.json.bak
python3 - <<'PY'
import json, pathlib
f = pathlib.Path('MID360_config_left.json')
d = json.loads(f.read_text())
print(json.dumps(d, indent=2)[:800])   # 先看结构，确认要改的键位再动手
PY
```

改完必须重装（配置在 `share/` 下，`--symlink-install` 对 `install(DIRECTORY)` 装的
json **不一定**是软链）：

```bash
# 【机器人端执行】
cd ~/astribot_sdk_ros2/ws_robot
colcon build --symlink-install --packages-select astribot_s1_perception
# 复验实际生效的那一份
cat install/astribot_s1_perception/share/astribot_s1_perception/config/MID360_config_left.json | head -30
```

### 3.6 写入闸门：本次部署全程保持关闭

这是"不触发运动"的**机制性**保证，不是靠"我们不去调那个服务"的自觉。

| 参数 | 值 | 声明位置 | 作用 |
|---|---|---|---|
| **`allow_write_to_real`** | **`false`** | `arm_traj_bridge_node.py:198` · `chassis_cmd_bridge_node.py:138` · launch 默认 `bridge_bringup.launch.py:64-67` | 后端是真机且此项为 false → 闸门判 `REAL_WRITE_NOT_AUTHORIZED`，**六条写通路全部拒绝** |
| `declared_target` | `sim` 或 `real` | 同上 | 与 SDK 实测后端不一致 → `TARGET_MISMATCH`，同样拒绝 |
| `allow_unsafe_mode` | `false` | 同上 | 真机上仅 `robot_mode == 'safe'` 才允许写 |
| `high_control_rights` | **硬编码 `False`** | `sdk_session.py:283` | **不可通过参数打开**。SDK 层就不接受我们的指令 |
| `bridge.sdk_high_control_rights` | `false` | `config/bridge.yaml:40` | 只读状态桥接的边界 |
| `enable_waypoints_service` | `false` | `arm_traj_bridge_node.py:184` | 阻塞式 `move_joints_waypoints` 通路关闭 |
| `start_disabled` | `true` | `chassis_cmd_bridge_node.py:111` | 底盘积分器在 `~/enable` 被调用前不工作 |

六条被覆盖的写通路：手臂 Action goal（`arm_traj_bridge_node.py:256` → `REJECT`）、
`~/dispatch_waypoints`（:428）、`~/set_gripper`（`gripper_core.py:206-209`）、
底盘 250 Hz 内环（`chassis_cmd_bridge_node.py:218`）、外环（:241）、`~/enable`（:279）。

> ⚠️ **安全保证的落点很重要**：`allow_write_to_real` 被**故意不写进 yaml**
> （[`chassis_bridge.yaml:104-105`](../ws_robot/src/astribot_trajectory_bridge/config/chassis_bridge.yaml#L104-L105)
> 注明"必须在命令行给出，以免存进配置后被遗忘"）。
> 加上 §3.7 那条 yaml 不生效的缺陷，实际结论是：
> **本次的不动保证来自「代码默认值 + launch 内联字典」，不来自 yaml。**
> 所以不要试图通过编辑 yaml 来加强安全——那样反而可能什么都没生效。

闸门是**构造时评估一次**，没有运行期重开/关闭的服务；被拒绝时节点**不退出**，
继续上报状态（这正是我们要的诊断形态）。

诚实的局限（[`chassis_cmd_bridge_node.py:191-208`](../ws_robot/src/astribot_trajectory_bridge/astribot_trajectory_bridge/chassis_cmd_bridge_node.py#L191-L208) 自注）：
`_discover_backends()` 只可能返回一个元素，所以"多后端"那条闸门是死路径。
**"绝不同时拉起 MuJoCo 和真机"是操作纪律，不是代码强制**。
`write_gate.py:12-14` 也写明这是准入检查，**不是网络层隔离**。

### 3.7 已知不一致项：登记，本次不擅自"修好"

这几条都是勘察中查实的。它们不影响本次只读部署，但**打开执行通路前必须解决**。
现在改动它们会引入未经验证的变更，所以只登记。

**① MoveIt 与桥接的 action 命名不匹配**

```
moveit_controllers.yaml 期望 : /arm_left_controller/follow_joint_trajectory
桥接默认提供             : /astribot/arm_left_controller/follow_joint_trajectory
```

`moveit_controllers.yaml:36-38` 用 `arm_left_controller` + `action_ns: follow_joint_trajectory`，
而桥接的 `action_name.<grp>` 默认值带 `/astribot` 前缀。
此外 MoveIt 还要 `torso_controller`、`gripper_left/right_controller` 三个
`FollowJointTrajectory` 服务端，而桥接**只提供两个手臂组**，夹爪走的是
`~/set_gripper` **服务**（不是 action）。
→ **实机 MoveIt 执行通路目前没有接通。** 本次只做规划（§4.6），不涉及。

**② 桥接的两个 yaml 实际不生效**

`bridge_bringup.launch.py:136` 给容器 Node 设了 `name='astribot_bridge_container'`，
而 rcl 的节点名 remap 是**进程级**的，会把容器内两个节点都改名；
`chassis_bridge.yaml` / `arm_bridge.yaml` 是按 `chassis_cmd_bridge:` / `arm_traj_bridge:`
键的，于是**两个 yaml 段都匹配不上、全部回落到代码默认值**。
launch 的内联字典用的是 `/astribot_bridge_container` 键，**是生效的**。

当前两边取值逐项相同，所以没有行为差异——但**编辑这两个 yaml 不会有任何效果**。
§5.4 给出了一条能证实/证伪它的命令。修法（本次不做）：去掉 `name=`，或把两个 yaml 改成 `/**:` 键。

**③ 其他**

| 项 | 现状 |
|---|---|
| `arm_bridge.yaml:24` `dispatch_mode: "stream"` | 代码里从未 `declare_parameter`，是死配置 |
| `map_source.yaml` 头部注释 | 注释说默认 `real_file + ground_truth`，实际值是 `sim_slam` / `slam`。信注释会跑错分支 |
| `hardware_livox.launch.py:45-47` | 实机 launch 依赖 `astribot_s1_gazebo_bringup` 的 `share/`。清理方案：把 `astribot_s1_controllers.yaml` 移到 `astribot_s1_description`。本次按"保留构建"处理（§2.3） |
| `navigation.launch.py:250` | 唯一一处没有 `scoped=True` 的 include（`params_file` 泄漏隐患）。本次不启动 nav2 |
| `nav2_full_bringup.launch.py:241` | RViz 节点的 `use_sim_time` 写成 Python 字面量 `True`，`env:=hardware` 时只有 RViz 是 true。本次不启动 |

---

## 4 · 实机启动流程（分步，全程不产生运动）

**每个 Phase 都给出"为什么它不会让机器人动"的机制依据**，而不是只说"这个是安全的"。
每个 Phase 用独立 tmux 窗口，起完先跑该 Phase 的验证（§5）再进下一步。

```bash
# 【机器人端执行】每个新终端窗口都先做这两件事
cd ~/astribot_sdk_ros2 && source env_robot.sh
ros2 daemon stop     # 换过 domain/网络配置后必做，否则 CLI 读旧缓存
```

### 4.0 Phase 0 · 启动前的安全前提确认

```bash
# 【机器人端执行】
echo "--- domain / 网络 ---"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "FASTRTPS=$FASTRTPS_DEFAULT_PROFILES_FILE"

echo "--- 图上现在有没有别的东西在跑（防止双后端 / 残留进程）---"
ros2 node list 2>/dev/null
pgrep -af 'astribot_simulation|gz sim|gzserver|ign gazebo' || echo "无仿真进程 OK"

echo "--- /cmd_vel 当前发布者数（必须是 0）---"
ros2 topic info /cmd_vel 2>/dev/null || echo "话题尚不存在（正常）"
```

**通过判据**：无 MuJoCo / Gazebo 进程；`/cmd_vel` 无发布者。

> 残留进程清理时注意：**按工作空间路径杀进程会漏掉 `/opt/ros/humble` 下的二进制**
> （`parameter_bridge`、`nav2_*` 等），留下的僵尸进程会污染后续测量结果。
> 且 `pkill -f <pattern>` 若 pattern 出现在命令行里会**杀掉自己所在的 shell**——
> 用变量拼接或 `pgrep` 先看清单再逐个 kill。

### 4.1 启动顺序总览

```
Phase 0  安全前提确认            —— 无节点
Phase 1  只读状态桥接            —— SDK 会话（只读）+ /joint_states + RSP
Phase 2  雷达驱动                —— livox ×2
Phase 3  感知链                  —— 预处理 ×2 + 融合 + 点云转 scan + 多层切片
Phase 4  SLAM                    —— 建图 或 定位
Phase 5  规划                    —— move_group（只规划，不执行）
Phase 6  桥接容器（闸门关闭）    —— 诊断真机后端识别与闸门判定
```

Phase 1 与 Phase 2 都会拉起 `robot_state_publisher`。**两者只能有一个开**，
否则 `/robot_description` 与 TF 会有两个发布者。下面用
`use_robot_state_publisher:=false` 让 Phase 1 让位给 Phase 2。

### 4.2 Phase 1 · 只读状态桥接

```bash
# 【机器人端执行】窗口 1
cd ~/astribot_sdk_ros2 && source env_robot.sh
ros2 launch astribot_trajectory_bridge state_bridge.launch.py \
    use_robot_state_publisher:=false
```

**为什么不会动**：`state_bridge_node` 是 Gate-2 的只读节点，
只做 `SDK 部件数组 → 逐关节 /joint_states`，**没有任何写调用**；
其 SDK 会话由 `config/bridge.yaml:40` 的 `sdk_high_control_rights: false` 约束，
且 `sdk_session.py:283` 把 `high_control_rights` 硬编码为 `False`。

若只想**先单独验证 URDF/TF 而完全不碰 SDK**，可以只起 RSP：

```bash
# 【机器人端执行】可选：纯模型验证，不建 SDK 会话
ros2 launch astribot_trajectory_bridge state_bridge.launch.py \
    use_robot_state_publisher:=true &
# 但这仍会起 state_bridge_node；只要 RSP 的话用下面这条更干净：
xacro "$(ros2 pkg prefix astribot_s1_description)/share/astribot_s1_description/urdf/astribot_s1.xacro" \
      robot_name:=astribot_s1 > /tmp/s1.urdf
ros2 run robot_state_publisher robot_state_publisher \
      --ros-args -p use_sim_time:=false -p robot_description:="$(cat /tmp/s1.urdf)"
```

### 4.3 Phase 2 · 雷达驱动

```bash
# 【机器人端执行】窗口 2
cd ~/astribot_sdk_ros2 && source env_robot.sh
ros2 launch astribot_s1_perception hardware_livox.launch.py \
    robot_name:=astribot_s1 use_lidar:=true use_camera:=true publish_freq:=10.0
```

**为什么不会动**：只有 `robot_state_publisher` + 两个 `livox_ros_driver2_node`，
全是传感器发布者，无任何指令话题/动作客户端。`use_sim_time` 在
`hardware_livox.launch.py:69` 硬编码为 `False`。

`publish_freq` 官方推荐 5/10/20/50；`10.0` 与
`livox_fusion_params.yaml:10` 的 `sync_slop_sec: 0.05` 匹配，先别改。

### 4.4 Phase 3 · 感知链

```bash
# 【机器人端执行】窗口 3
cd ~/astribot_sdk_ros2 && source env_robot.sh
ros2 launch astribot_s1_perception hardware_perception.launch.py use_sim_time:=false
```

这条会 include `sim_perception.launch.py`（**名字有误导性**，它其实是共用管线），
起 2 个预处理 + 1 个融合 + 1 个 `pointcloud_to_laserscan`。

多层切片扫描（自主/导航要用的 `/scan` 源）：

```bash
# 【机器人端执行】窗口 4
cd ~/astribot_sdk_ros2 && source env_robot.sh
ros2 launch astribot_s1_autonomy slice_scan.launch.py use_sim_time:=false
```

**为什么不会动**：这两条都是点云→点云 / 点云→scan 的纯数据变换节点，
无 `/cmd_vel` 发布者、无 action 客户端。

> ⚠️ **自滤参数必须复核**：`pointcloud_slice_scan_params.yaml:170-274` 的
> `self_filter` 是按仿真模型的机械包络配的（footprint 半径 0.42、
> 6 条 TF 链半径 0.20/0.18/0.15/0.15/0.10/0.10）。
> 实机机械包络若不同，机器人会**把自己的指尖当障碍**，
> 表现为"Starting point in lethal space"、探索零派发。§5.1 有验证方法。

### 4.5 Phase 4 · SLAM

**建图**（新场地）：

```bash
# 【机器人端执行】窗口 5
cd ~/astribot_sdk_ros2 && source env_robot.sh
ros2 launch astribot_s1_perception slam_mapping.launch.py use_sim_time:=false
```

**定位**（已有地图）：

```bash
# 【机器人端执行】窗口 5（二选一）
ros2 launch astribot_s1_perception slam_localization.launch.py \
    use_sim_time:=false \
    map_file_name:=/home/astribot/astribot_sdk_ros2/maps/demo_warehouse
```

`map_file_name` 不带扩展名（slam_toolbox 序列化格式，会去找 `.posegraph` + `.data`）。
`map_start_pose` 参数在该 launch 的 L54-60 自注**不起作用**，别依赖它。

> **不要**用 `perception_slam_bringup.launch.py` 做这一步。它的
> `autonomous_patrol` **默认是 `true`**，会拉起
> `autonomous_patrol_node` 直接往 `/cmd_vel` 发 `Twist`（`autonomous_patrol_node.py:97`）——
> **单独跑这个入口就会让机器人动起来**。
> 必要时只能这样调用，且三个参数一个都不能省：
> ```bash
> # 若确实要用聚合入口，autonomous_patrol:=false 是强制的
> ros2 launch astribot_s1_perception perception_slam_bringup.launch.py \
>     env:=hardware mode:=mapping launch_gazebo:=false \
>     autonomous_patrol:=false use_rviz:=false
> ```
> 本方案推荐上面的**逐叶子 launch**方式，从机制上避开这个默认值。

`max_laser_range` 需与实际 scan 的 `range_max` 对齐：
`mapper_params_online_async.yaml:39` 是 `20.0`，
而 `pointcloud_to_laserscan_params.yaml:31` 也是 `20.0`。
两者**相等**时，Karto 的可碾区间 `[threshold, maxRange)` 为空，
**SLAM 永不主动碾出自由空间**。实机 Mid-360 量程 70 m，
这两个值现场需要重新定，且必须一起定。

### 4.6 Phase 5 · 规划（只规划，不执行）

```bash
# 【机器人端执行】窗口 6
cd ~/astribot_sdk_ros2 && source env_robot.sh
ros2 launch astribot_s1_moveit_config move_group.launch.py \
    use_sim_time:=false use_rviz:=false
```

**为什么不会动**：`move_group` 提供 `/execute_trajectory` 服务端，
但**从不主动发起**执行。而且按 §3.7 ①，实机侧的
`FollowJointTrajectory` 控制器名/命名空间与桥接不匹配，
即使有人误调执行也会因找不到控制器而失败——**这不是设计的安全措施，只是当前的客观状态**。

> 已知：`move_group` 退出时会 SIGSEGV（析构链问题，与业务无关，但会刷一屏，掩盖真实错误）。

### 4.7 Phase 6 · 桥接容器（闸门关闭，纯诊断）

这一步的目的是**验证闸门本身工作正常**，同时取证"SDK 确实识别到真机后端"。

```bash
# 【机器人端执行】窗口 7
cd ~/astribot_sdk_ros2 && source env_robot.sh
ros2 launch astribot_trajectory_bridge bridge_bringup.launch.py \
    target:=real \
    allow_write_to_real:=false \
    pose_source:=slam \
    enable_waypoints_service:=false \
    use_sim_time:=false \
    domain_id:=$ROS_DOMAIN_ID
```

**为什么不会动**：`target:=real` + `allow_write_to_real:=false` 使闸门判定为
`REAL_WRITE_NOT_AUTHORIZED`（枚举值 **62**），六条写通路全部拒绝（§3.6）。
节点保持存活并持续上报状态，这正是我们要观察的。
`sim_root` 留空 → 不会拉起 MuJoCo（`bridge_bringup.launch.py:121-128`）。

**期望观察到的结果**（这是正向判据，不是错误）：

```bash
# 【机器人端执行】
ros2 topic echo /astribot/bridge/status --once
# status_code 应为 62 (REAL_WRITE_NOT_AUTHORIZED)
```

若 `status_code` 是 `SDK_NOT_ALIVE` → 后端没连上，回 §1.3；
若是 `TARGET_MISMATCH`（61）→ SDK 实测后端不是真机（可能是 MuJoCo 残留），回 Phase 0。

### 4.8 完整启动脚本（只读诊断栈）

```bash
# 【机器人端执行】保存为 ~/astribot_sdk_ros2/start_diag_stack.sh
cat > ~/astribot_sdk_ros2/start_diag_stack.sh <<'EOF'
#!/usr/bin/env bash
# 只读诊断栈：不含任何写入通路，不会产生运动
# 用法: ./start_diag_stack.sh   然后 tmux attach -t astribot
set -e
cd "$(dirname "$0")"

tmux has-session -t astribot 2>/dev/null && { echo "会话已存在，先 tmux kill-session -t astribot"; exit 1; }
tmux new-session -d -s astribot -n state

run() { tmux send-keys -t "astribot:$1" "cd $(pwd) && source env_robot.sh && $2" C-m; }

run state "ros2 launch astribot_trajectory_bridge state_bridge.launch.py use_robot_state_publisher:=false"
sleep 8

tmux new-window -t astribot -n lidar
run lidar "ros2 launch astribot_s1_perception hardware_livox.launch.py publish_freq:=10.0"
sleep 6

tmux new-window -t astribot -n perception
run perception "ros2 launch astribot_s1_perception hardware_perception.launch.py use_sim_time:=false"
sleep 4

tmux new-window -t astribot -n slice
run slice "ros2 launch astribot_s1_autonomy slice_scan.launch.py use_sim_time:=false"
sleep 4

tmux new-window -t astribot -n slam
run slam "ros2 launch astribot_s1_perception slam_mapping.launch.py use_sim_time:=false"
sleep 6

tmux new-window -t astribot -n movegroup
run movegroup "ros2 launch astribot_s1_moveit_config move_group.launch.py use_sim_time:=false use_rviz:=false"

tmux new-window -t astribot -n check
echo "已启动。tmux attach -t astribot；在 check 窗口跑 §5 的验证命令。"
echo "注意：桥接容器(Phase 6)刻意不在本脚本内，需人工确认前面全绿后手动启动。"
EOF
chmod +x ~/astribot_sdk_ros2/start_diag_stack.sh
```

### 4.9 禁止清单（本次部署绝对不执行）

**launch 层**

| 入口 | 会发生什么 |
|---|---|
| `nav2_full_bringup.launch.py` | nav2 全栈 → `controller_server` → `/cmd_vel`；`exploration:=true` 时还有 `NavigateToPose` 客户端 |
| `navigation.launch.py` | 同上核心部分 |
| `perception_slam_bringup.launch.py` **不带** `autonomous_patrol:=false` | `autonomous_patrol_node` 直接发 `Twist` 到 `/cmd_vel` |
| `warehouse_sim.launch.py` | 拉 Gazebo + spawn 7 个控制器（含 `wheel_effort_controller` 与两个夹爪控制器）+ 力矩驱动节点 |
| `omni_effort_drive.launch.py` | 订阅 `/cmd_vel`，往 `/wheel_effort_controller/commands` 发轮力矩 |
| `arm_chassis_coupling.launch.py` | 最终 `/cmd_vel` 发布者 |
| `exploration_coordinator.launch.py` | `NavigateToPose` + `ComputePathToPose` 动作客户端 |
| `planning_demo.launch.py execute:=true` | MoveIt `execute()` + `GripperCommander` 的 FollowJointTrajectory 客户端 |
| `bridge_bringup.launch.py allow_write_to_real:=true` | **打开写入闸门** |
| `autonomy_bringup.launch.py` | 本身不发指令，但与探索协调器同用时链路会通 |

**服务 / 话题 / 动作层**

```
ros2 service call /astribot_bridge_container/enable ...          ← 底盘积分器使能
ros2 service call /astribot_bridge_container/set_gripper ...     ← 夹爪
ros2 service call /astribot_bridge_container/dispatch_waypoints  ← 阻塞式轨迹下发
ros2 action send_goal /astribot/arm_*_controller/follow_joint_trajectory ...
ros2 topic pub /cmd_vel ...
ros2 topic pub /cmd_vel_pre_arm_coupling ...
ros2 topic pub /wheel_effort_controller/commands ...
```

**厂商示例层**（`examples/`）

| 可用（只读） | 禁止（会运动） |
|---|---|
| `100-get_robot_properties.py`（仅 `get_*`，已逐行确认） | `103`~`110` 全部 `move_*` / `set_*` |
| `101-get_joint_states.py`（仅 `get_*`） | `201-head_follow` · `202/203-chassis_joy_control` |
| `102-get_cartesian_states.py`（仅 `get_*`） | `206/207-waypoints` · `208-traj_replay` |
| `204-forward_kinematics.py`（纯计算） | `210-arm_gravity_compensation` · `211-whole_body_control_test` |
| `209-get_closest_dist.py` | `999-stop_robot.py`（虽是停止，但仍是控制调用） |
| `300-get_images.py` | — |
| ⚠️ `301-get_lidar_scan.py` | 会 `activate_lidar()` / `deactivate_lidar()`——**改变设备状态但不产生本体运动**。需要时再用，用完注意它会 deactivate |

> 这些示例都需要 `Astribot()` 会话。它们**默认 `high_control_rights` 由示例自身决定**，
> 与桥接的硬编码 False 不同。用之前先看该文件的构造调用。

---

## 5 · 运行时状态验证

原则：**每条判据都要能被一条命令证实或证伪**。看不到数据不等于"还在启动中"。

### 5.1 话题验证

```bash
# 【机器人端执行】
ros2 topic list | sort
```

**期望存在**（按 Phase 累积）：

| Phase | 话题 | 类型 | 期望频率 |
|---|---|---|---|
| 1 | `/joint_states` | `sensor_msgs/JointState` | ~50 Hz（`bridge.yaml:28` `publish_rate`） |
| 2 | `/livox/lidar_left` `/livox/lidar_right` | `PointCloud2` | ~10 Hz |
| 2 | `/livox/imu_left` `/livox/imu_right` | `Imu` | 高频 |
| 2 | `/robot_description` · `/tf` `/tf_static` | — | — |
| 3 | `/livox/fused_points` | `PointCloud2` | ~10 Hz |
| 3 | `/scan` | `LaserScan` | ~10 Hz |
| 4 | `/map` · `/slam_toolbox/...` | `OccupancyGrid` | 低频 |
| 6 | `/astribot/bridge/status` | `astribot_bridge_msgs/BridgeStatus` | 持续 |
| 6 | `/astribot/chassis/odom_from_sdk` | `nav_msgs/Odometry` | 250 Hz 内环产物 |

```bash
# 【机器人端执行】逐条量频率（每条会阻塞几秒）
for t in /joint_states /livox/lidar_left /livox/lidar_right /livox/fused_points /scan; do
  echo "--- $t ---"
  timeout 6 ros2 topic hz "$t" 2>&1 | head -3
done
```

**安全相关的反向判据**（必须为 0）：

```bash
# 【机器人端执行】/cmd_vel 不能有任何发布者
ros2 topic info /cmd_vel 2>/dev/null | grep -E 'Publisher count'
# 期望 "Publisher count: 0"，或话题根本不存在

# 【机器人端执行】仿真时钟不能有发布者
ros2 topic info /clock 2>/dev/null | grep -E 'Publisher count'
```

**QoS 与发布者数**（重复发布者是 TF/状态诡异问题的常见根因）：

```bash
# 【机器人端执行】
ros2 topic info -v /joint_states | head -20
ros2 topic info -v /robot_description | grep -E 'count|Durability|Reliability'
# robot_description 的 Publisher count 必须是 1（Phase 1/2 双 RSP 会变成 2）
```

> 关于 QoS 告警：SDK 侧会刷 `joint_space_command` 的 incompatible QoS 警告，
> 那是 **SDK 自己的 RELIABLE 订阅收不到自己的 BEST_EFFORT 发布**，两端都在 SDK 内部，
> 不是我们的配置问题，也不影响指令落地（x86 侧实测 100% 落地、误差 0.0 mm）。

**点云自滤有效性**（§4.4 的遗留风险）：

```bash
# 【机器人端执行】看融合点云里有没有落在机身包络内的点
timeout 10 ros2 topic echo /livox/fused_points --field height --once
# 更实际的判据：/scan 里最近距离不应显著小于 range_min(0.35)
timeout 8 ros2 topic echo /scan --once 2>/dev/null \
  | python3 -c "
import sys,re
d=sys.stdin.read()
m=re.search(r'ranges:\s*\n((?:\s*-\s*[-\d.einf]+\n)+)', d)
if m:
    v=[float(x) for x in re.findall(r'-?[\d.]+(?:e-?\d+)?', m.group(1)) if x not in ('inf','-inf')]
    v=[x for x in v if x>0]
    print(f'最近 {min(v):.3f} m / 最远 {max(v):.3f} m / 有效点 {len(v)}')
    print('⚠️ 最近距离小于 range_min，疑似自身点未滤除' if min(v)<0.34 else '自滤看起来正常')
else: print('未解析到 ranges')
"
```

### 5.2 TF 校验

```bash
# 【机器人端执行】
# 根 frame 必须是 astribot_torso_base，且**不存在** base_link
ros2 run tf2_tools view_frames -o /tmp/frames 2>&1 | tail -3
ros2 topic echo /tf_static --once 2>/dev/null | grep -E 'child_frame_id|frame_id' | sort -u | head -30

echo "--- base_link 检查（应无输出）---"
ros2 topic echo /tf_static --once 2>/dev/null | grep -c 'base_link' || echo "无 base_link OK"
```

```bash
# 【机器人端执行】逐条查关键变换
ros2 run tf2_ros tf2_echo astribot_torso_base livox_mid360_left  --ros-args -p use_sim_time:=false
ros2 run tf2_ros tf2_echo astribot_torso_base livox_mid360_right --ros-args -p use_sim_time:=false
ros2 run tf2_ros tf2_echo astribot_torso_base camera_link        --ros-args -p use_sim_time:=false
# Phase 4 起来之后
ros2 run tf2_ros tf2_echo map astribot_torso_base --ros-args -p use_sim_time:=false
```

**通过判据**：

| 检查 | 期望 |
|---|---|
| 根 frame | `astribot_torso_base`（**不是** `base_link`，**不是**任何传感器 frame） |
| 两颗雷达外参 | 与 §3.5 实测值一致（注意 json 是 mm/度、xacro 是 m/rad） |
| `map → astribot_torso_base` | Phase 4 起来后应存在且不报外推 |
| 地面高度 | 地面在 `astribot_torso_base` 下方约 `z ≈ -0.095`（**不是** -0.129；odom/Gazebo 的静态高度 0.1292 不是地面偏移） |
| 时间戳 | `ros2 topic echo /tf --once` 的 stamp 应是墙钟时间，不是 0 |

> 若 `tf2_echo` 带 timeout 却查不到**动态** TF，而静态 TF 正常——
> 那是"从工作线程做带 timeout 的 lookupTransform"的已知坑，不是 TF 真的缺。

### 5.3 模型加载校验

```bash
# 【机器人端执行】① xacro 能否离线展开（最先排除的一层）
xacro "$(ros2 pkg prefix astribot_s1_description)/share/astribot_s1_description/urdf/astribot_s1.xacro" \
      robot_name:=astribot_s1 > /tmp/s1.urdf && echo "xacro 展开 OK ($(wc -l < /tmp/s1.urdf) 行)"

# 【机器人端执行】② URDF 结构自洽 + link/joint 计数
command -v check_urdf >/dev/null || sudo apt install -y liburdfdom-tools
check_urdf /tmp/s1.urdf | head -20
echo "link 数 = $(grep -c '<link ' /tmp/s1.urdf)   joint 数 = $(grep -c '<joint ' /tmp/s1.urdf)"

# 【机器人端执行】③ mesh 是否真的装进 share（§0.4 的直接判据）
MESH="$(ros2 pkg prefix astribot_s1_description)/share/astribot_s1_description/meshes"
ls "$MESH" && echo "--- 夹爪 mesh（必须存在，缺了碰撞体积会静默少算）---" && ls "$MESH/s1_gripper" | head

# 【机器人端执行】④ 检查 URDF 里引用的每个 mesh 文件是否都在
grep -o 'package://astribot_s1_description/meshes/[^"]*' /tmp/s1.urdf | sort -u | while read -r m; do
  f="${MESH}/${m#package://astribot_s1_description/meshes/}"
  [ -f "$f" ] || echo "缺失: $m"
done; echo "mesh 引用检查完成"

# 【机器人端执行】⑤ 运行期实际加载的那一份
ros2 param get /robot_state_publisher robot_description 2>/dev/null | head -c 300; echo
```

```bash
# 【机器人端执行】⑥ MoveIt 模型（Phase 5 之后）
ros2 param get /move_group robot_description_semantic 2>/dev/null | head -c 300; echo
ros2 param get /move_group planning_plugin
# 期望: astribot_s1_manipulation/OmplPlannerExtension

echo "--- 规划组 ---"
ros2 service list | grep -E 'plan_kinematic_path|get_planning_scene|compute_fk|compute_ik'
```

> **OMPL 插件必须确认注册成功**：MoveIt2 Humble 自带的 OMPL 插件只注册 25 个规划器，
> **不含 BIT\* / Informed RRT\***。yaml 里写了未注册的名字**不报错**，
> 会静默回落到组默认规划器（通常 RRTConnect）——"以为在跑 BIT\*"其实没跑。
> 本仓库的 `OmplPlannerExtension` 就是为注册这两个而存在的，所以上面那条
> `planning_plugin` 的返回值是硬判据。

```bash
# 【机器人端执行】⑦ 关节限位一致性（仓库自带测试，机器化了"以厂商 per-part 模型为准"这条规则）
cd ~/astribot_sdk_ros2/ws_robot
colcon test --packages-select astribot_s1_description --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/astribot_s1_description
```

该测试需要 `astribot_config` 在位（§0.4）。它会校验限位、TCP 偏置、
夹爪两源交叉一致性、以及"夹爪确实进了碰撞模型"。

### 5.4 参数打印与配置生效验证

```bash
# 【机器人端执行】节点清单——顺便证实/证伪 §3.7 ② 那条 yaml 缺陷
ros2 node list | sort
```

**这就是 §3.7 ② 的判据**：

* 若看到 `/chassis_cmd_bridge` 和 `/arm_traj_bridge` → 节点名正常，两个 yaml 生效。
* 若只看到 `/astribot_bridge_container`（或它出现两次）→ **节点名 remap 缺陷已确认**，
  两个 yaml 完全未加载，全部参数走代码默认值。

```bash
# 【机器人端执行】写入闸门的实际取值（安全核心，必须逐条确认）
N=$(ros2 node list | grep -E 'astribot_bridge_container|chassis_cmd_bridge' | head -1)
echo "查询节点: $N"
for p in allow_write_to_real declared_target allow_unsafe_mode \
         enable_waypoints_service start_disabled use_sim_time pose_source; do
  printf '%-26s = %s\n' "$p" "$(ros2 param get "$N" "$p" 2>&1 | awk '{print $NF}')"
done
```

**通过判据**：`allow_write_to_real = False`、`allow_unsafe_mode = False`、
`enable_waypoints_service = False`、`use_sim_time = False`。

```bash
# 【机器人端执行】全图 use_sim_time 扫描（实机不允许出现 True）
for n in $(ros2 node list); do
  v=$(ros2 param get "$n" use_sim_time 2>/dev/null | awk '{print $NF}')
  [ "$v" = "True" ] && printf '⚠️ %-45s use_sim_time=True\n' "$n"
done; echo "扫描完成"

# 【机器人端执行】导出全部参数留档，便于与下次部署做 diff
mkdir -p ~/deploy_check/params
for n in $(ros2 node list); do
  f=~/deploy_check/params/$(echo "$n" | tr '/' '_').yaml
  ros2 param dump "$n" > "$f" 2>/dev/null || true
done
ls ~/deploy_check/params/
```

`ros2 param dump` 的输出正是排障时最有用的东西——
"参数到底是不是我以为的那个值"这个问题，靠读 yaml 永远答不了（§3.7 ②就是例子）。

### 5.5 写入闸门状态确认

```bash
# 【机器人端执行】
ros2 interface show astribot_bridge_msgs/msg/BridgeStatus | head -40
ros2 topic echo /astribot/bridge/status --once
```

**状态码对照**（`astribot_bridge_msgs/msg/BridgeStatus.msg`）：

| 码 | 名称 | 含义 | 本次期望 |
|---|---|---|---|
| 60 | `MULTIPLE_BACKENDS` | 多后端（实为死路径，见 §3.6） | — |
| 61 | `TARGET_MISMATCH` | `declared_target` 与实测后端不符 | 若出现 → 后端不是真机，查 Phase 0 |
| **62** | **`REAL_WRITE_NOT_AUTHORIZED`** | 真机后端 + 未授权写入 | ✅ **这就是期望值** |
| 63 | `POSE_SOURCE_INVALID` | `ground_truth` + `real` 组合非法 | 用 `pose_source:=slam` 可避免 |
| — | `SDK_NOT_ALIVE` | 后端没连上 | 若出现 → 回 §1.3 |

**看到 62 是好事**：它同时证明了两件事——SDK 认出了真机后端，且闸门确实在拦。

### 5.6 就绪判定表

| # | 项 | 判据命令 | 期望 |
|---|---|---|---|
| 1 | 网络绑定 | `tcpdump -ni wlP1p1s0 'udp portrange 7400-7700'` | WiFi 无 DDS 数据流 |
| 2 | domain 一致 | `ros2 node list` | 能看到 SDK 侧节点 |
| 3 | 关节状态 | `ros2 topic hz /joint_states` | ~50 Hz |
| 4 | 雷达 ×2 | `ros2 topic hz /livox/lidar_{left,right}` | 各 ~10 Hz |
| 5 | 融合点云 | `ros2 topic hz /livox/fused_points` | ~10 Hz，无持续丢帧 |
| 6 | scan | `ros2 topic hz /scan` | ~10 Hz |
| 7 | TF 根 | `tf2_echo astribot_torso_base livox_mid360_left` | 有解，且无 `base_link` |
| 8 | map TF | `tf2_echo map astribot_torso_base` | 有解，不报外推 |
| 9 | URDF | `check_urdf /tmp/s1.urdf` + mesh 引用检查 | 全部通过，无缺失 mesh |
| 10 | MoveIt 插件 | `ros2 param get /move_group planning_plugin` | `OmplPlannerExtension` |
| 11 | **闸门** | `ros2 param get <node> allow_write_to_real` | **False** |
| 12 | **闸门状态** | `ros2 topic echo /astribot/bridge/status --once` | **62** |
| 13 | **无运动源** | `ros2 topic info /cmd_vel` | Publisher count: 0 |
| 14 | 无仿真时钟 | `ros2 topic info /clock` | Publisher count: 0 |
| 15 | 无 sim time | 全图 `use_sim_time` 扫描 | 无 True |

---

## 6 · 常见部署故障排查清单

每一条按 **现象 → 根因 → 定位命令 → 处置** 组织。
标 🔬 的是本仓库**已实测取证**过的坑，不是理论推演。

### 6.1 🔬 双网卡 DDS：话题有发布者但收不到数据

| 现象 | 可能根因 |
|---|---|
| `ros2 topic list` 能看到话题，`ros2 topic echo` 永远无输出 | 两端选了不同网卡的 locator；或 SHM 段权限/残留 |
| `ros2 node list` 全空，像"系统没起来" | `ROS_DOMAIN_ID` 不一致（§1.3）或 `ros2 daemon` 缓存 |
| 时通时不通、重启后表现变化 | 多网卡上同时做发现，对端选到了不可路由的地址 |
| WiFi 一断，本机节点之间也失联 | 业务流误走了 WiFi |

```bash
# 【机器人端执行】① 先排除 CLI 缓存——这一步能省掉很多冤枉路
ros2 daemon stop && sleep 2

# ② 环境三件套是否如预期
env | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|FASTRTPS_DEFAULT_PROFILES_FILE'

# ③ 进程实际绑了哪些地址（比看配置文件可靠）
for p in $(pgrep -f 'bridge_container|livox_ros_driver2_node|robot_state_publisher'); do
  echo "--- pid $p $(tr '\0' ' ' < /proc/$p/cmdline | cut -c1-60) ---"
  ss -aunp 2>/dev/null | grep -w "pid=$p" | awk '{print "   ", $5}' | sort -u | head
done

# ④ 逐网卡看多播加入
cat /proc/net/igmp

# ⑤ 数据流实际走哪张卡
sudo timeout 10 tcpdump -ni eno1       'udp portrange 7400-7700' -c 10
sudo timeout 10 tcpdump -ni wlP1p1s0   'udp portrange 7400-7700' -c 10   # 期望：没有数据流

# ⑥ SHM 残留（换配置/异常退出后可能留下坏段）
ls -l /dev/shm | head -20
```

**处置**

| 判定 | 动作 |
|---|---|
| ③ 里出现 `10.249.x` | Fast DDS profile 未生效。查 `FASTRTPS_DEFAULT_PROFILES_FILE` 是否被 `env.sh` 的 lan 分支覆盖（§0.3），确认用的是 §3.2 那份 |
| ⑤ 中 WiFi 有大量数据流 | 白名单地址写错，或 XML 语法错被静默忽略。Fast DDS 对坏 XML 常常**不报错只忽略** |
| ⑤ 中 eno1 也没有流量 | 先回 §1.3 查 domain；再查防火墙 `sudo ufw status` |
| ⑥ 有陈旧 `fastrtps_` 段 | 全部节点停掉后 `rm -f /dev/shm/fastrtps_*`，再重启 |
| 只有跨机不通、本机正常 | `ROS_LOCALHOST_ONLY` 被设成 1 了。实机必须是 **0**（§3.1） |

> 🔬 **一个已实测的反直觉结论**：给 Fast DDS 挂一个
> `interfaceWhiteList=127.0.0.1` 的 XML 来"限制到本机"是**无效且有害**的——
> eno1 的多播加入数照涨。原因：`interfaceWhiteList` 只过滤单播 locator，
> 不阻止 SPDP 从物理网卡多播出去；而带 `is_default_profile` 的 XML 还会盖掉
> rmw 层的 localhost 处理。要"仅本机"只能用 `ROS_LOCALHOST_ONLY=1`
> 并且**主动 `unset FASTRTPS_DEFAULT_PROFILES_FILE`**。
> 实机不适用这条（我们要跨机连控制器），但排障时容易被这个思路带偏。

### 6.2 网卡 metric 异常

| 现象 | 根因 |
|---|---|
| `apt` / `git clone` / DNS 超时，但 `ping 10.249.x` 正常 | eno1 抢了默认路由，出网流量走了不通外网的业务口（§1.2 的风险项） |
| 重启或 WiFi 重连后业务链路又坏了 | NetworkManager 重新协商时把 metric/路由改了 |
| `192.168.0.10` 忽然不可达 | eno1 掉线，或路由被后加的连接覆盖 |

```bash
# 【机器人端执行】
ip route show
ip route get 223.5.5.5     | head -1    # 出网：应 dev wlP1p1s0
ip route get 192.168.0.10  | head -1    # 业务：应 dev eno1
nmcli -t -f NAME,DEVICE,STATE connection show --active
journalctl -u NetworkManager --since '-20 min' --no-pager | tail -30
```

**处置（固化，重启后仍有效）**：

```bash
# 【机器人端执行】
ENO1=$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: '$2=="eno1"{print $1}')
WIFI=$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: '$2=="wlP1p1s0"{print $1}')

sudo nmcli connection modify "$ENO1" ipv4.method manual \
     ipv4.addresses 192.168.0.11/24 ipv4.route-metric 100 \
     ipv4.never-default yes connection.autoconnect yes
sudo nmcli connection modify "$WIFI" ipv4.route-metric 600 connection.autoconnect yes

sudo nmcli connection up "$ENO1"
ip route show default        # 复验：默认路由只应来自 WiFi
```

`ipv4.never-default yes` 是关键：eno1 保留 `192.168.0.0/24` 链路路由（业务可达），
但不再提供默认路由（出网不受影响）。**DDS 不依赖默认路由**——靠 §3.2 的白名单绑定。

### 6.3 URDF / 模型加载失败

| 现象 | 根因 |
|---|---|
| `colcon build` 报 `找不到 mesh 源目录` | `astribot_config` 没同步，或相对目录关系被破坏（§0.4） |
| RViz 里模型是一堆方块 / 部件缺失 | mesh 没装进 `share/`，或 `package://` 解析失败 |
| `robot_state_publisher` 起不来 | xacro 展开失败（参数拼写、include 路径） |
| TF 树断裂 / 有孤立分支 | 两颗 Mid-360 的 `side` 参数重复，或传感器 frame 名不一致 |
| **碰撞检测悄悄少算体积** | 夹爪 collision mesh 缺失——⚠️ 这一条不报错，会撞机器 |

```bash
# 【机器人端执行】按"从源头到运行期"的顺序逐层排除
# ① xacro 能否展开（脱离 launch 单独试，报错最清晰）
xacro "$(ros2 pkg prefix astribot_s1_description)/share/astribot_s1_description/urdf/astribot_s1.xacro" \
      robot_name:=astribot_s1 > /tmp/s1.urdf; echo "exit=$?"

# ② 结构自洽
check_urdf /tmp/s1.urdf | head

# ③ mesh 源（构建期依赖）
ls ~/astribot_sdk_ros2/astribot_config/robot_config/astribot_s1/meshes
ls ~/astribot_sdk_ros2/astribot_config/robot_config/astribot_s1/model/meshes/s1_gripper | head -3

# ④ mesh 目标（运行期）
ls "$(ros2 pkg prefix astribot_s1_description)/share/astribot_s1_description/meshes"

# ⑤ 逐个核对 URDF 引用的 mesh 是否都存在（§5.3 ④ 的完整版）
MESH="$(ros2 pkg prefix astribot_s1_description)/share/astribot_s1_description/meshes"
grep -o 'package://astribot_s1_description/meshes/[^"]*' /tmp/s1.urdf | sort -u | while read -r m; do
  [ -f "${MESH}/${m#package://astribot_s1_description/meshes/}" ] || echo "缺失: $m"
done

# ⑥ 夹爪是否真的进了碰撞模型（仓库自带测试，机器化了这条）
cd ~/astribot_sdk_ros2/ws_robot && colcon test --packages-select astribot_s1_description \
  --event-handlers console_direct+ 2>&1 | tail -20
```

**处置**

| 判定 | 动作 |
|---|---|
| ③ 缺失 | 同步 `astribot_config`（§2.1），保持 `ws_robot` 的相对位置 |
| ③ 有、④ 无 | 没重编 description。`rm -rf build/astribot_s1_description install/astribot_s1_description` 后重编 |
| ① 失败 | 看报错行号；注意 `--symlink-install` 下改了 xacro 会立即生效，但改 `CMakeLists.txt` 必须重编 |
| ④ 有但 RViz 仍空 | `package://` 需要包已 `source`；确认 `source ws_robot/install/setup.bash` |

> 🔬 **改 URDF 必须同步点云自滤**：夹爪不在自滤链里 →
> 机器人把自己的指尖当障碍 → `Starting point in lethal space` → 探索零派发。
> 自滤配置在 `astribot_s1_autonomy/config/pointcloud_slice_scan_params.yaml:170-274`，
> 与 URDF **没有任何自动校验关系**。两条 scan 链都要查，且 SLAM 的地图**有记忆**——
> 改完要重新建图，否则旧地图里的错误障碍还在。

### 6.4 🔬 launch 参数错误（本仓库踩得最多的一类）

| 现象 | 根因 |
|---|---|
| 参数明明写在 yaml 里，节点行为却是默认值 | ① 节点名与 yaml 键不匹配（§3.7 ②）；② `params_file` 共享上下文泄漏 |
| 节点加载了**别的**包的 yaml，且**零报错** | `params_file` 是共享 launch 上下文名，第一个 include 抢到该名，后续节点静默加载错文件。`/**:` 键的文件"能加载但全落默认值" |
| 机器人自转 / 控制周期变成 2 Hz | 🔬 已实测：底盘节点加载了探索协调器的 yaml，拿到 `control_period_sec: 0.5` |
| 重复的 action server、`MoveItErrorCode=-7`、`Invalid Trajectory`、`PREEMPTED` | launch 没装 shutdown handler，泄漏了上一个 `move_group` 进程，两个实例抢同名 action |
| RViz 里 TF 冻结但其他节点正常 | `nav2_full_bringup.launch.py:241` 把 RViz 的 `use_sim_time` 写成了字面量 `True` |

```bash
# 【机器人端执行】① 先数进程——重复实例是最容易被误判成"并发 bug"的原因
pgrep -af 'move_group|bridge_container|robot_state_publisher|slam_toolbox' | sort

# ② 节点名是否与 yaml 键一致
ros2 node list | sort

# ③ 参数的**实际**取值（唯一可信来源，不要读 yaml 推断）
ros2 param dump /astribot_bridge_container 2>/dev/null | head -40

# ④ 对比 yaml 里写的键
grep -nE '^[a-z_/*]+:' ~/astribot_sdk_ros2/ws_robot/src/astribot_trajectory_bridge/config/*.yaml
```

**处置**

| 判定 | 动作 |
|---|---|
| ② 只见 `/astribot_bridge_container` | §3.7 ② 缺陷确认。**不要改 yaml**（改了也不生效），要么去掉 launch 里的 `name=`，要么把 yaml 键改成 `/**:` |
| ① 有重复进程 | 全部 kill 后重启。注意杀进程的坑（§4.0）：按工作空间路径杀会漏掉 `/opt/ros/humble` 下的二进制 |
| ③ 与预期不符 | 检查该 launch 是否在 `GroupAction(scoped=True)` 里显式传了 `params_file` |

> **写新 launch 时的硬规矩**：每个 `IncludeLaunchDescription` 都必须显式传
> `params_file`，并包在 `GroupAction(scoped=True)` 里。
> 仓库里 `warehouse_sim.launch.py:366-405` 有一整段注释记录这个 bug 的现场。
> 目前 `navigation.launch.py:250` 是唯一还没加 `scoped=True` 的地方。

### 6.5 依赖缺失

| 现象 | 根因 | 处置 |
|---|---|---|
| `find_library(LIVOX_LIDAR_SDK_LIBRARY ... REQUIRED)` 失败 | Livox-SDK2 没装到 `/usr/local`，且**任何 package.xml 都没声明它** | §2.3 ② |
| `find_package(gazebo_ros REQUIRED)` 失败 | `aws-robomaker-small-warehouse-world`（Gazebo Classic） | §2.3 ③ `COLCON_IGNORE` |
| `livox_ros_driver2` 编成 ROS1 形态 / 找不到可执行文件 | 缺 `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble` | §2.4 |
| `No module named filterpy / tabulate / h5py` | rosdep **不覆盖**这三个 | `pip install --no-deps filterpy tabulate h5py` |
| numpy 版本被顶、测试大面积失败 | 跑了 `install.sh`（它用 `pip install -U` 不带 `--no-deps`） | 重装原 numpy；以后只单跑 `install.sh` 的目录创建段 |
| `No module named robotics_library_py` | `astribot_sdk/core/common` 不在 `PYTHONPATH`（编译过的 `util.py:26` 用裸名导入） | §3.1 的 `env_robot.sh` 已含该路径 |
| `libdmumps_seq-5.4.so => not found` | 缺原生库 | `sudo apt install libmumps-seq-5.4` |
| `spdlog_ex` + `create log config directory failed` | `/opt/astribot_ros` 不存在，SDK 直接 `terminate` | §1.6 |
| `ImportError` 但架构不符（`wrong ELF class`） | 误用了仓库里的 x86-64 `.so` | §0.1，改用机器人原生 SDK |

```bash
# 【机器人端执行】依赖体检一条龙
echo "--- Livox-SDK2 ---"; ls -l /usr/local/lib/liblivox_lidar_sdk_shared.so 2>&1 | head -1
echo "--- 原生库 ---";     ldconfig -p | grep -E 'dmumps|gsl' | head
echo "--- pip ---";        python3 -c "
import importlib
for m in ('filterpy','tabulate','h5py','numpy'):
    try: print(f'  OK      {m:10s}', importlib.import_module(m).__version__)
    except Exception as e: print(f'  MISSING {m:10s}', type(e).__name__)"
echo "--- SDK 架构 ---"
find / -xdev -name '_robotics_library_py*.so' 2>/dev/null | while read -r f; do file -b "$f" | cut -c1-45; done
```

> 🔬 **断言"ABI 冲突"之前先跑一条 `ldd`**：本仓库自带 pinocchio 3.7.0、
> 系统装 4.0.0，soname 不同即无冲突。曾经有过"ABI 阻塞在线验证"的错误结论，
> 一条 `ldd` 就否证了。错根因会扩散进文档并让排查停止。

### 6.6 时间同步问题

| 现象 | 根因 |
|---|---|
| TF 报 `extrapolation into the future/past`，但节点都在跑 | 有节点 `use_sim_time=true` 而实机无 `/clock` |
| `tf2_buffer: Detected jump back in time` | 两套栈抢同一个 `ROS_DOMAIN_ID`，或系统时钟被 NTP 猛拽 |
| 融合点云持续丢帧 | 两颗雷达时钟不同步，`sync_slop_sec: 0.05` 窗口对不上 |
| 所有话题有发布者但零消息，日志被 THROTTLE 静音 | 🔬 sim time 恒为 0（仿真侧问题，实机不应出现——若出现说明混入了 sim 配置） |

```bash
# 【机器人端执行】
timedatectl status
chronyc tracking 2>/dev/null | head -5
ros2 topic info /clock                       # 必须 Publisher count: 0
for n in $(ros2 node list); do
  v=$(ros2 param get "$n" use_sim_time 2>/dev/null | awk '{print $NF}')
  [ "$v" = "True" ] && echo "⚠️ $n use_sim_time=True"
done
# 消息时间戳是否是墙钟
ros2 topic echo /joint_states --once 2>/dev/null | grep -A2 stamp | head -4
date +%s
```

**处置**：`use_sim_time` 全线 false（§3.3）；确保没有任何 Gazebo/MuJoCo 残留在发 `/clock`；
`chrony` 保持运行且与雷达同源。若雷达支持 PTP，优先 PTP。

### 6.7 ROS_DOMAIN_ID 不一致（最可能的首次现场故障）

| 现象 | 根因 |
|---|---|
| "节点全都 Node not found"，像整个系统没起来 | 查询 shell 没带 `ROS_DOMAIN_ID`，或与 launch 侧不一致 |
| 自己的节点互相可见，但看不到 SDK / 控制器 | 本栈与厂商 domain 不一致（本栈已统一 25，见 §0.3；现场若真是 42 就两边都改成 42） |

```bash
# 【机器人端执行】
echo "当前 shell: ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<未设置！>}"
for d in 25 42; do
  echo "--- domain $d ---"
  ROS_DOMAIN_ID=$d timeout 8 ros2 node list --no-daemon 2>/dev/null | head
done
```

**处置**：统一到 §1.3 实测出的那个值，写进 `env_robot.sh`，
并确认所有 launch 的 `domain_id` 参数与之一致（`bridge_bringup.launch.py` 会
`SetEnvironmentVariable('ROS_DOMAIN_ID', ...)` 覆盖子进程）。

---

## 7 · 实机长期运行固化建议

### 7.1 环境变量固化

需要两份形态：交互式 shell 用 `source`，systemd 用 `EnvironmentFile`（**不支持 shell 语法**）。

```bash
# 【机器人端执行】① systemd 用的纯 KEY=VALUE（不能有 $ 展开、不能有 source）
sudo mkdir -p /etc/astribot
sudo tee /etc/astribot/orin.env >/dev/null <<'EOF'
ROS_DOMAIN_ID=25
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_LOCALHOST_ONLY=0
FASTRTPS_DEFAULT_PROFILES_FILE=/home/astribot/astribot_sdk_ros2/config/fastdds_orin_eno1.xml
ROBOT_TYPE=S1
ASTRIBOT_LOG=1
ASTRIBOT_SDK_ROOT=__填写：机器人原生 SDK 根__
ASTRIBOT_MIDDLEWARE_PY=__填写：.../astribot_ros_middleware/lib/python3.10/site-packages__
EOF
sudo chmod 644 /etc/astribot/orin.env
```

```bash
# 【机器人端执行】② 交互式 shell：显式手动 source，不要自动注入
grep -q 'astribot_sdk_ros2/env_robot.sh' ~/.bashrc || cat >> ~/.bashrc <<'EOF'

# Astribot 实机环境。刻意**不自动 source** —— 自动注入会让每个新 shell 都带上
# ROS_DOMAIN_ID/DDS 配置，一旦配置有误就到处复现，且 scp/rsync 等非交互场景也受影响。
# 需要时手动执行： astribot-env
alias astribot-env='source ~/astribot_sdk_ros2/env_robot.sh'
EOF
```

> ⚠️ 仓库根的 `install.sh:59-62` 会往 `~/.bashrc` **追加** `source /opt/ros/humble/setup.bash`。
> 若已被追加过多次会重复 source（无害但混乱）：`grep -n 'setup.bash' ~/.bashrc` 自查去重。

**固化后必须复验一次**（环境变量类问题最容易"以为写了其实没生效"）：

```bash
# 【机器人端执行】模拟 systemd 的环境加载方式做一次干净验证
env -i bash -c 'set -a; . /etc/astribot/orin.env; set +a;
  . /opt/ros/humble/setup.bash;
  echo "DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION LOCALHOST=$ROS_LOCALHOST_ONLY";
  test -f "$FASTRTPS_DEFAULT_PROFILES_FILE" && echo "DDS XML 在位" || echo "⚠️ DDS XML 缺失"'
```

### 7.2 开机自启（分级：只读可自启，写入通路一律手动）

**分级原则**：能自启的必须是**不可能产生运动**的部分。
桥接容器即使闸门关闭也**不自启**——因为它是唯一一个"改一个命令行参数就能写真机"的进程，
自启会让那个参数脱离人工确认。

```bash
# 【机器人端执行】等 eno1 真正就绪的辅助脚本（DDS 绑定 192.168.0.11，网卡没起来就会绑失败）
sudo tee /usr/local/bin/astribot-wait-eno1 >/dev/null <<'EOF'
#!/usr/bin/env bash
# 等 eno1 拿到 192.168.0.11，最多 60 秒
for i in $(seq 1 60); do
    ip -4 addr show eno1 2>/dev/null | grep -q '192\.168\.0\.11' && { echo "eno1 就绪"; exit 0; }
    sleep 1
done
echo "eno1 未在 60s 内就绪" >&2; exit 1
EOF
sudo chmod +x /usr/local/bin/astribot-wait-eno1
```

**单元 1 · 只读状态桥接**

```bash
# 【机器人端执行】
sudo tee /etc/systemd/system/astribot-state.service >/dev/null <<'EOF'
[Unit]
Description=Astribot 只读状态桥接 (SDK -> /joint_states)
# 网络就绪是硬前提：DDS 要绑 192.168.0.11
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=astribot
Group=astribot
EnvironmentFile=/etc/astribot/orin.env
WorkingDirectory=/home/astribot/astribot_sdk_ros2
ExecStartPre=/usr/local/bin/astribot-wait-eno1
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && \
    source /home/astribot/astribot_sdk_ros2/ws_robot/install/setup.bash && \
    exec ros2 launch astribot_trajectory_bridge state_bridge.launch.py \
         use_robot_state_publisher:=false'
Restart=on-failure
RestartSec=5
# 只读节点重启无副作用，可以自动重启
StandardOutput=journal
StandardError=journal
SyslogIdentifier=astribot-state

[Install]
WantedBy=multi-user.target
EOF
```

**单元 2 · 雷达 + 感知**

```bash
# 【机器人端执行】
sudo tee /etc/systemd/system/astribot-perception.service >/dev/null <<'EOF'
[Unit]
Description=Astribot 雷达驱动 + 感知链
After=network-online.target astribot-state.service
Wants=network-online.target

[Service]
Type=simple
User=astribot
Group=astribot
EnvironmentFile=/etc/astribot/orin.env
WorkingDirectory=/home/astribot/astribot_sdk_ros2
ExecStartPre=/usr/local/bin/astribot-wait-eno1
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && \
    source /home/astribot/astribot_sdk_ros2/ws_robot/install/setup.bash && \
    exec ros2 launch astribot_s1_perception hardware_livox.launch.py publish_freq:=10.0'
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=astribot-perception

[Install]
WantedBy=multi-user.target
EOF
```

```bash
# 【机器人端执行】启用与管理
sudo systemctl daemon-reload
sudo systemctl enable --now astribot-state.service
sudo systemctl enable --now astribot-perception.service

systemctl status astribot-state astribot-perception --no-pager | head -30
journalctl -u astribot-state -f            # 跟日志
```

**明确不做自启的单元**（写下来，避免以后有人"顺手补齐"）：

| 组件 | 为何不自启 |
|---|---|
| `bridge_bringup`（桥接容器） | 唯一"改一个参数就能写真机"的进程，必须人工确认前置状态后手动起 |
| 任何 nav2 / 探索 / patrol | 会发 `/cmd_vel` |
| `move_group` | 可自启（不主动执行），但它退出时 SIGSEGV 会刷屏，建议按需手动起 |

> `bridge_bringup.launch.py:133-153` 刻意设了 `respawn=False`：底盘是**开环位置积分**，
> 重启后可能从错误的积分种子继续。**不要给桥接加 `Restart=always`**——
> 这与只读节点的处理方式必须区别对待。

### 7.3 网络稳定性保障

```bash
# 【机器人端执行】① 业务口静态化 + 自动重连（§6.2 的固化命令，这里是完整版）
ENO1=$(nmcli -t -f NAME,DEVICE connection show | awk -F: '$2=="eno1"{print $1; exit}')
sudo nmcli connection modify "$ENO1" \
     ipv4.method manual ipv4.addresses 192.168.0.11/24 \
     ipv4.route-metric 100 ipv4.never-default yes \
     connection.autoconnect yes connection.autoconnect-retries 0
#                                                        ^ 0 = 无限重试

# ② 运维口：关掉省电，避免 ssh 掉线打断长任务
WIFI=$(nmcli -t -f NAME,DEVICE connection show | awk -F: '$2=="wlP1p1s0"{print $1; exit}')
sudo nmcli connection modify "$WIFI" \
     802-11-wireless.powersave 2 ipv4.route-metric 600 connection.autoconnect yes
#                              ^ 2 = disable

sudo nmcli connection up "$ENO1"; sudo nmcli connection up "$WIFI"
```

```bash
# 【机器人端执行】③ UDP 收包缓冲加大（大点云 + 250Hz 多话题）
sudo tee /etc/sysctl.d/60-astribot-dds.conf >/dev/null <<'EOF'
# Fast DDS 大消息（Livox 点云）与高频小消息共存，默认缓冲偏小会丢包
net.core.rmem_max = 16777216
net.core.rmem_default = 8388608
net.core.wmem_max = 16777216
net.core.wmem_default = 8388608
net.ipv4.ipfrag_time = 3
net.ipv4.ipfrag_high_thresh = 134217728
EOF
sudo sysctl --system | tail -5

# ④ 文件描述符上限
sudo tee /etc/security/limits.d/60-astribot.conf >/dev/null <<'EOF'
astribot soft nofile 8192
astribot hard nofile 16384
EOF
```

```bash
# 【机器人端执行】⑤ 业务链路健康巡检（只读，可挂 cron / timer）
sudo tee /usr/local/bin/astribot-netcheck >/dev/null <<'EOF'
#!/usr/bin/env bash
# 只读巡检：不改配置、不重启服务，只报告
ok=0
ip -4 addr show eno1 | grep -q '192\.168\.0\.11' || { echo "FAIL eno1 地址丢失"; ok=1; }
ip route get 192.168.0.10 2>/dev/null | grep -q 'dev eno1' || { echo "FAIL 业务路由不走 eno1"; ok=1; }
ip route get 223.5.5.5 2>/dev/null | grep -q 'dev eno1' && { echo "WARN 出网走了 eno1"; }
ping -c1 -W1 192.168.0.10 >/dev/null 2>&1 || echo "WARN 192.168.0.10 不可达"
[ $ok -eq 0 ] && echo "OK 网络链路正常"
exit $ok
EOF
sudo chmod +x /usr/local/bin/astribot-netcheck
astribot-netcheck
```

**长任务一律在 tmux 里跑**（WiFi 是运维口，掉线不该杀掉业务）：

```bash
# 【机器人端执行】
tmux new -s astribot        # 断线后 tmux attach -t astribot
```

### 7.4 日志与留痕

```bash
# 【机器人端执行】① journald 限容，避免长期运行把磁盘写满
sudo mkdir -p /etc/systemd/journald.conf.d
sudo tee /etc/systemd/journald.conf.d/astribot.conf >/dev/null <<'EOF'
[Journal]
SystemMaxUse=2G
SystemMaxFileSize=200M
MaxRetentionSec=2week
EOF
sudo systemctl restart systemd-journald
```

```bash
# 【机器人端执行】② 三个会长期增长的目录，都要纳入巡检
du -sh ~/.ros/log /opt/astribot_ros/log ~/astribot_sdk_ros2/ws_robot/log 2>/dev/null

# ③ 定期清理（保留 7 天）
sudo tee /etc/cron.daily/astribot-logclean >/dev/null <<'EOF'
#!/bin/sh
find /home/astribot/.ros/log -maxdepth 1 -type d -mtime +7 -exec rm -rf {} + 2>/dev/null
find /opt/astribot_ros/log   -type f -mtime +7 -delete 2>/dev/null
exit 0
EOF
sudo chmod +x /etc/cron.daily/astribot-logclean
```

```bash
# 【机器人端执行】④ 部署留痕：出问题时第一件要问的就是"装的是哪个版本"
cat > ~/deploy_check/deployed.txt <<EOF
部署时间   : $(date -Is)
git SHA    : $(git -C ~/astribot_sdk_ros2 rev-parse --short HEAD 2>/dev/null || echo 'rsync 下发，无 SHA')
分支       : $(git -C ~/astribot_sdk_ros2 rev-parse --abbrev-ref HEAD 2>/dev/null || echo '-')
架构       : $(uname -m)
ROS_DISTRO : $ROS_DISTRO
DOMAIN_ID  : $ROS_DOMAIN_ID
eno1       : $(ip -4 -brief addr show eno1 | awk '{print $3}')
EOF
cat ~/deploy_check/deployed.txt
```

> 用 rsync 下发（方式 C）时没有 SHA，排障会明显更难——这是推荐方式 A 的实际理由。

---

## 8 · 现场待定清单（本方案无法在离线状态下确定的事）

这些项**必须现场取证**，不要按仓库里的值直接假定。前三条会直接决定整套栈能否跑通。

| # | 待定事项 | 判定命令 | 若与预期不符的影响 |
|---|---|---|---|
| 1 | **SDK 后端实际的 `ROS_DOMAIN_ID`**（仓库已统一 25，现场曾给 42，要实测确认） | §1.3 | 不一致则本栈与 SDK **完全互不可见**，表象是"节点全都不存在" |
| 2 | **`192.168.0.10` 是否存在、是否就是 SDK 后端** | §1.3 第一条 | 决定 §3.2 白名单是否够用、是否需要 initialPeers |
| 3 | **Orin 上是否有原生 aarch64 SDK + `astribot_msgs`** | §1.5 | 没有则整个方案需要厂商先提供 aarch64 构建（§0.1） |
| 4 | 两颗 Livox 的真实 IP 与所在网卡（`192.168.1.x` 是第三网段） | §1.2 末段 | 雷达起不来 → 无 `/scan` → SLAM/导航全链路无输入 |
| 5 | eno1 是否抢了默认路由 | `ip route get 223.5.5.5` | 抢了则 apt/git 全部超时，表现为"网络通但装不上包" |
| 6 | `/opt/astribot_ros` 是否已创建（需 sudo） | §1.6 | 缺失则 SDK 直接 `terminate`（不是降级） |
| 7 | 桥接的两个 yaml 是否真的加载（节点名 remap 缺陷） | §5.4 第一条 | 影响所有 yaml 调参；安全参数不受影响（走内联字典） |
| 8 | 实机机械包络与自滤参数是否匹配 | §5.1 末段 | 不匹配 → 机器人把自己当障碍 → 探索零派发 |
| 9 | `max_laser_range` 与实际 scan `range_max` 的口径 | §4.5 末段 | 两者相等时 Karto **永不碾出自由空间**，且对 SLAM 静默无效 |
| 10 | `joint_limits.yaml` 的全部 `max_acceleration` | — | 全是 `max_velocity/0.35s` 的估算值，文件自注未标定。**执行前必须实机辨识** |

### 本方案已明确划在范围外的事

| 事项 | 状态 |
|---|---|
| 打开写入闸门（`allow_write_to_real:=true`） | 独立 Gate，需单独风险评估 |
| MoveIt ↔ 桥接的 action 命名对齐（§3.7 ①） | 执行通路前置项，**目前未接通** |
| 桥接缺 `torso` / 夹爪的 `FollowJointTrajectory` 服务端 | 同上（夹爪走 service 而非 action） |
| nav2 / 探索 / patrol 上机 | 全部会产生运动 |
| SRDF `virtual_joint` 改 `planar`（导航与操作并发） | 需可靠 odom TF，属后续设计 |
| 实机夹持力验证 | `set_effector_max_force` 在仿真下是空操作，实机行为未知 |

---

## 附:文档关系

| 文档 | 定什么 |
|---|---|
| [`docs/sim_real_alignment.md`](sim_real_alignment.md) | 对齐**什么**：真值源规则、决策 D1-D6、Gate 0-5 验收标准、附录 B 差异实测数据 |
| **本文** | 怎么**装上去**：环境校验、下发构建、配置部署、无运动启动、诊断、固化 |
| `ws_robot/src/astribot_trajectory_bridge/README.md` | 桥接层内部设计与端口抽象 |
| `ws_robot/src/astribot_s1_perception/README_PERCEPTION.md` | 感知链细节与其自身的排障清单 |

> **本文的所有结论都来自对当前工作树（分支 `chassis-effort-drive`，HEAD `8a3c3a9`）的静态勘察，
> 没有连接过任何机器人。** 凡是标注"实测"的，指的是仓库内已记录的历史实测（多数在 x86 开发机 +
> MuJoCo 后端上完成）；凡是标注"待定"的，就是真的未知。§8 的十项必须现场取证后才能开始 §4 的启动流程。
