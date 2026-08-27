# 同步到实机 aarch64 SDK 目录：技术方案与流程

> 目标：`astribot@10.249.22.137:/home/astribot/Downloads/astribot_sdk_aarch64`
> 本文所有事实均在实机实测取证（2026-08-27），不是按仓库推断。
> 配套：[`real_robot_deployment.md`](real_robot_deployment.md) · [`chassis_slam_integration_plan.md`](chassis_slam_integration_plan.md)

---

## 0 · 结论先行：这是一次 3 MB 的同步，难点不在传输

实测数据把问题重新定义了：

| 量 | 值 |
|---|---|
| 本地仓库总大小 | 1.9 GB |
| **实机真正需要的源码** | **2.97 MB / 381 文件** |
| 其余 | x86-64 构建产物、日志、地图、仿真世界包 |

明细（`find` 逐文件统计，排除 `build`/`install`/`log`/`__pycache__`/`.git`）：

```
全部源码      :    18.60 MB (512 文件)
其中 aws 世界 :    15.64 MB (131 文件)  <- 仿真专用，排除
实机需要      :     2.97 MB (381 文件)
```

所以**方案的价值不在"怎么传"，而在"排除什么"和"怎么不破坏目标端已有的 aarch64 SDK"。**

### 0.1 目标目录已经是一个独立的 git 仓库

```
origin  https://gitlab.astribot.com/public_repo/astribot_sdk_aarch64
分支 master   HEAD 2644cdf
未跟踪: examples/212-chassis_spin_in_place.py
        examples/frames_2026-08-27_17.11.08.gv / .pdf
```

**这与我们本地的仓库是两个不同的 repo**（我们本地无 remote，是 x86-64 变体）。
所以**不能用 `git pull` / `git push` 做同步**——两边历史无关。
只能把 `ws_robot/` 当作**载荷**注入进去。

### 0.2 目标端的 SDK 确实是原生 aarch64（部署方案 §0.1 的阻塞项到此解除）

```
_robotics_library_py.so                       ELF 64-bit LSB shared object, ARM aarch64
libastribot_msgs__rosidl_typesupport_cpp.so   ELF 64-bit LSB shared object, ARM aarch64
```

`third_party/software/astribot_ros_middleware/lib/python3.10/site-packages` **在位**。

> **因此 `astribot_sdk/` `third_party/` `astribot_msgs/` 三个目录一律不同步。**
> 它们在目标端是能跑的 aarch64 版本，我们本地那份是 x86-64，覆盖过去等于把机器人搞坏。

### 0.3 `astribot_config` 逐项一致，**不需要传那 178 MB**

| | 文件数 | 总字节 |
|---|---|---|
| 本地 | 112 | 186,406,891 |
| 目标 | 112 | 186,406,891 |

7 个 per-part yaml（限位真值源）md5 **全部逐字节相同**：

```
477752036ffaca8802ba20de513ff18e  astribot_arm_left.yaml
d94896431ce0168cc144a63e03d1e9c2  astribot_arm_right.yaml
c6dbedd41849278c95d53127f602ffb5  astribot_chassis.yaml
7f757b0ba9723ac9c3ae73ea0a745ad4  astribot_gripper_left.yaml
2b57481bd0d19a1d22a519f0926bd698  astribot_gripper_right.yaml
b1ee90789438856b9db146bb6c11d02e  astribot_head.yaml
5779ba394d6cd06811e449bc41c76d5c  astribot_torso.yaml
```

构建期硬依赖的两个 mesh 目录也都在位（22 + 24 文件），
所以 [`astribot_s1_description/CMakeLists.txt:30-46`](../ws_robot/src/astribot_s1_description/CMakeLists.txt#L30-L46)
那个 `FATAL_ERROR` 不会触发——**只要 `ws_robot` 放在 SDK 根目录下、保持三层上溯关系**。

### 0.4 ⚠️ 绝对不要覆盖目标端的 `env.sh`

目标端 `env.sh` 有两处我们本地已修的缺陷：

| 目标端现状 | 问题 |
|---|---|
| `env.sh:17` `PYTHONPATH=.../third_party/astribot_ros_middleware_py` | **该目录不存在**（真实路径是 `third_party/software/astribot_ros_middleware/lib/python3.10/site-packages`，已确认在位）。不存在的路径挂在 PYTHONPATH 上不报错、静默忽略 |
| 无 `astribot_sdk/core/common` | 编译过的 `util.py:26` 用**裸名** `import robotics_library_py.robotics_library_py`，缺这一层 import 必失败 |

**但直接把我们的 `env.sh` 覆盖过去会引入更严重的回归**：
我们本地 `env.sh` 默认 `ASTRIBOT_NET_MODE=localhost` → `ROS_LOCALHOST_ONLY=1`。
实机上 SDK 与控制器（`192.168.0.10`）之间走的就是 DDS，
**`ROS_LOCALHOST_ONLY=1` 会把这条链路直接切断**。
而且目标端 `env.sh` 已经正确设了 `ROS_DOMAIN_ID=25` / `ROS_LOCALHOST_ONLY=0` / `RMW=rmw_fastrtps_cpp`。

> **正解：不动 `env.sh`，新增一份 `env_robot.sh`（§3）。**
> 两边各自演进，同步流程也不必处理冲突。

### 0.5 ⚠️ `/opt/ros/humble` 是**最小安装**，我们的包编不过

这是本次同步之后最大的一道工作量，实测：

```
/opt/ros/humble/share 下只有 118 个包，已装 ros-humble-* apt 包 116 个
```

| 包 | 状态 |
|---|---|
| `xacro` · `tf2_ros` · `pcl_conversions` | ✅ 存在 |
| `robot_state_publisher` | ❌ **缺失** |
| `moveit_ros_move_group` · `moveit_planners_ompl` | ❌ 缺失 |
| `nav2_bringup` · `nav2_mppi_controller` | ❌ 缺失 |
| `slam_toolbox` · `pointcloud_to_laserscan` | ❌ 缺失 |
| `rviz2` | ❌ 缺失 |

`colcon` 与 `rosdep` 都在。磁盘充裕（1.7 T 可用）。

**所以流程必须是「先补依赖，再构建」，不能同步完就直接 `colcon build`。**

---

## 1 · 目录布局决策

```
/home/astribot/Downloads/astribot_sdk_aarch64/     ← 目标端既有 git 仓库，不动它的历史
├── astribot_sdk/          ← 目标端 aarch64，❌ 不同步
├── third_party/           ← 目标端 aarch64，❌ 不同步
├── astribot_msgs/         ← 目标端 aarch64，❌ 不同步
├── astribot_config/       ← 逐项一致，❌ 不同步（§0.3）
├── config/                ← ⚠️ 只增不覆盖（放 Fast DDS profile，见 §3.2）
├── env.sh                 ← ❌ 不覆盖（§0.4）
├── examples/              ← ❌ 不同步（目标端有本地改动）
├── install.sh             ← ❌ 不同步
├── tools/                 ← 可选
│
├── ws_robot/              ← ✅ 本次同步的载荷（2.97 MB）
│   └── src/               ← 12 个自有包（排除 aws 仿真世界）
├── docs/                  ← ✅ 建议同步（312 KB，现场排障要查）
└── env_robot.sh           ← ✅ 新增，不覆盖 env.sh
```

**`ws_robot` 必须放在 SDK 根目录下**，这不是风格选择：
`astribot_s1_description/CMakeLists.txt` 用
`${CMAKE_CURRENT_SOURCE_DIR}/../../../astribot_config/robot_config/astribot_s1/meshes`
三层上溯取 mesh，放错位置会在构建期 `FATAL_ERROR`。

对应关系必须是：

```
<SDK_ROOT>/ws_robot/src/astribot_s1_description/CMakeLists.txt
<SDK_ROOT>/astribot_config/robot_config/astribot_s1/meshes          ✓ 三层上溯命中
```

---

## 2 · 同步方案

### 2.1 为什么不用 git

| 方式 | 结论 |
|---|---|
| `git pull` / `git push` | ❌ 两边是**不同的 repo**（`astribot_sdk_aarch64` vs 我们本地无 remote），历史无关 |
| 把 `ws_robot` 提交进目标 repo | ❌ 那是厂商的 `public_repo`，不该把我们的工作推进去 |
| `git bundle` | ❌ 本地有 **30 个未跟踪条目**（整个桥接层都未提交），bundle 带不走 |
| **`rsync` 白名单同步** | ✅ 载荷只有 3 MB，且能精确控制排除范围 |

> 若要版本可追溯（生产部署应当如此），正确做法是给 `ws_robot` **单独建一个我们自己的 git 仓库**
> （公司 GitLab），目标端 clone 它到 `<SDK_ROOT>/ws_robot`。
> 那样 §2.2 的 rsync 只用于首次落地或应急，日常走 git。见 §5。

### 2.2 rsync 同步（先 dry-run）

`rsync` 需要目标端有 `rsync`。先确认：

```bash
# 【本地工作站执行】
ssh orin 'command -v rsync || echo "目标端缺 rsync，走 §2.3 的 tar 方案"'
```

```bash
# 【本地工作站执行】① 先 dry-run，把要传的文件看清楚
SRC=/home/yjh/WorkSpace/astribot_sdk_ros2
DST=orin:/home/astribot/Downloads/astribot_sdk_aarch64

rsync -avhn --delete \
  --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='__pycache__/' --exclude='*.pyc' --exclude='.coverage' \
  --exclude='.git/' \
  --exclude='maps/' \
  --exclude='aws-robomaker-small-warehouse-world/' \
  "$SRC/ws_robot" "$DST/"
```

**`-n` 的输出必须逐条看过**。确认无误后去掉 `-n`：

```bash
# 【本地工作站执行】② 真正同步 ws_robot
rsync -avh --delete \
  --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='__pycache__/' --exclude='*.pyc' --exclude='.coverage' \
  --exclude='.git/' \
  --exclude='maps/' \
  --exclude='aws-robomaker-small-warehouse-world/' \
  "$SRC/ws_robot" "$DST/"

# ③ 文档（现场排障要查）
rsync -avh "$SRC/docs" "$DST/"
```

**逐条排除的理由**（不是抄模板）：

| 排除项 | 理由 |
|---|---|
| `build/` `install/` `log/` | 无锚定模式，同时命中仓库根与 `ws_robot/src/*/` 下的**嵌套残留**（本地实测 `astribot_s1_autonomy` 里有 103 MB、`astribot_s1_manipulation` 18 MB）。这些是 **x86-64 产物**，在 aarch64 上完全无用 |
| `maps/` | 本地 `ws_robot/maps` 122 MB + 仓库根 `maps/`，都是仿真跑出来的地图。实机要用的是 Voxel-SLAM 的 `sessions/1floor` |
| `.git/` | 目标端有自己的 git 仓库，把我们的 `.git` 传过去会造成两个仓库嵌套 |
| `aws-robomaker-...` | 15.64 MB 纯仿真世界，且是唯一的构建阻塞包（`find_package(gazebo_ros REQUIRED)`，Gazebo Classic） |
| `.coverage` | 本地测试产物 |

> **`--delete` 的作用范围**：只作用于 `$DST/ws_robot/`（rsync 的 `--delete` 限于被同步的目录树），
> 不会碰 `astribot_sdk/` `third_party/` 等同级目录。首次同步仍建议先看 `-n` 输出。

### 2.3 备用：tar over ssh（目标端无 rsync 时）

```bash
# 【本地工作站执行】
cd /home/yjh/WorkSpace/astribot_sdk_ros2
tar czf - \
  --exclude='build' --exclude='install' --exclude='log' \
  --exclude='__pycache__' --exclude='*.pyc' --exclude='.coverage' \
  --exclude='.git' --exclude='maps' \
  --exclude='aws-robomaker-small-warehouse-world' \
  ws_robot docs \
| ssh orin 'cd /home/astribot/Downloads/astribot_sdk_aarch64 && tar xzf - && echo "解包完成"'
```

### 2.4 同步后立即校验

> **本方案的 rsync 命令已对实机 dry-run 实测过**（未写入任何文件）：
>
> ```
> total size is 3.13M   speedup is 156.12 (DRY RUN)
> 待传条目      : 502      （381 个文件 + 目录项）
> 产物泄漏      : 0        （build/install/log）
> aws 世界泄漏  : 0
> pycache 泄漏  : 0
> maps 泄漏     : 0
> .so/.o 泄漏   : 0
> 目标端将删除  : 0        （首次同步应为 0）
> ```
>
> 目标端 `rsync 3.2.7` 已确认在位。上面这组数字就是 §2.2 命令的预期输出，
> 真同步前用 `-n` 复现一遍，任何一项不为 0 都要先查清再传。

```bash
# 【机器人端执行】
D=/home/astribot/Downloads/astribot_sdk_aarch64
echo "=== 载荷到位 ==="
find $D/ws_robot/src -maxdepth 1 -mindepth 1 -type d | sort | sed "s|$D/ws_robot/src/||"
echo "文件数: $(find $D/ws_robot/src -type f | wc -l)   (本地是 381，不含 aws 世界)"

echo "=== 三层上溯关系（构建期硬依赖）==="
ls $D/ws_robot/src/astribot_s1_description/../../../astribot_config/robot_config/astribot_s1/meshes >/dev/null 2>&1 \
  && echo "mesh 三层上溯 OK" || echo "❌ 位置不对，构建会 FATAL_ERROR"

echo "=== 目标端 aarch64 SDK 未被污染 ==="
file -b $D/astribot_sdk/core/common/robotics_library_py/_robotics_library_py.so | cut -c1-42
#   必须仍是 ARM aarch64；出现 x86-64 说明误覆盖了，立刻 git checkout 恢复

echo "=== 没有 x86 产物混进来 ==="
find $D/ws_robot -name '*.so' -o -name '*.o' 2>/dev/null | head
find $D/ws_robot -maxdepth 2 \( -name build -o -name install -o -name log \) -type d 2>/dev/null
```

**通过判据**：12 个包目录、381 个文件、mesh 三层上溯 OK、SDK 仍是 aarch64、无 `.so`/`.o`/产物目录。

---

## 3 · 环境脚本：新增而不覆盖

### 3.1 `env_robot.sh`

放在 SDK 根目录，**与目标端 `env.sh` 并存**。

> **已在实机落地并验证通过**（2026-08-27）。两处是实测才发现、光看代码想不到的：
>
> 1. **厂商 `env.sh` 不 source `/opt/ros/humble`** —— 它只 source SDK 内部三个
>    setup.bash（`third_party/software`、`astribot_msgs`、`third_pkg`）。
>    不先 source ROS 的话 `astribot_interface.py:17` 的 `import rclpy` 直接
>    `ModuleNotFoundError`，SDK 整个起不来。**顺序也重要**：
>    `astribot_msgs/local_setup.bash` 是 ROS overlay，要 ROS 在前。
> 2. **厂商 `env.sh` 自己就设了 Fast DDS 白名单**（`env.sh:101-102`，实测生成
>    `config/fastdds_whitelist_192.xml`、绑 `192.168.0.11`）。所以 §3.2 那句
>    "不要再叠白名单"要这样理解：**用它这一份，不要再加第三种机制**。
>
> 实机验证输出：
>
> ```
> ROS_DOMAIN_ID      = 25       ROS_LOCALHOST_ONLY = 0
> RMW                = rmw_fastrtps_cpp        ROS_DISTRO = humble
> ROBOT_TYPE         = S1
> FASTRTPS profile   = .../config/fastdds_whitelist_192.xml
> middleware 在 PYTHONPATH : 3   core/common 在 PYTHONPATH: 1
> SDK import OK                 ← 只 import，未建会话，机器人未动
> ```

它先 source ROS，再 source 厂商 `env.sh` 拿到正确的 domain/RMW/白名单，
然后补 §0.4 那两处缺陷，最后挂我们的工作空间。

```bash
# 【机器人端执行】
cat > /home/astribot/Downloads/astribot_sdk_aarch64/env_robot.sh <<'EOF'
#!/usr/bin/env bash
# =====================================================================
# 实机环境（aarch64）。**不修改厂商 env.sh**，只在其之上补齐。
#
# 为什么不直接改 env.sh：
#   · 它属于厂商 git 仓库(public_repo/astribot_sdk_aarch64)，改了每次 pull 都冲突
#   · 我们本地那份 env.sh 默认 ASTRIBOT_NET_MODE=localhost -> ROS_LOCALHOST_ONLY=1，
#     会切断 SDK 与控制器(192.168.0.10)之间的 DDS，覆盖过来是严重回归
# =====================================================================
SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 1. 先用厂商 env.sh（它的 ROS_DOMAIN_ID=25 / LOCALHOST_ONLY=0 / RMW 是对的）----
source "${SDK_ROOT}/env.sh"

# ---- 2. 补厂商 env.sh 的两处缺陷 ----
# (a) env.sh:17 指向的 third_party/astribot_ros_middleware_py 根本不存在，
#     真实路径在 third_party/software/... 下（已实测在位）。
#     不存在的路径挂 PYTHONPATH 上不报错、静默忽略，所以这条缺陷能活很久。
_MW="${SDK_ROOT}/third_party/software/astribot_ros_middleware/lib/python3.10/site-packages"
if [ -d "$_MW" ]; then
    export PYTHONPATH="${_MW}:${PYTHONPATH}"
    export ASTRIBOT_MIDDLEWARE_PY="$_MW"
else
    echo "[env_robot][ERROR] middleware 缺失: $_MW"
fi

# (b) 编译过的 util.py:26 用**裸名** import robotics_library_py.robotics_library_py，
#     需要 common 这一层可见。只有 SDK_ROOT 是不够的。
export PYTHONPATH="${SDK_ROOT}/astribot_sdk/core/common:${PYTHONPATH}"

# ---- 3. SDK 运行期约定 ----
# ASTRIBOT_LOG 必须在 import SDK **之前**生效：astribot_interface.py 在 import 时
# 就 os.dup2 重定向 fd 1/2，晚了会把我们自己的 ERROR 一起吞掉。
export ASTRIBOT_LOG=1
export ROBOT_TYPE="${ROBOT_TYPE:-S1}"     # 不是 S1 时底盘自由度会是 2，enable 会被拒

# ---- 4. 我们的工作空间 ----
if [ -f "${SDK_ROOT}/ws_robot/install/setup.bash" ]; then
    source "${SDK_ROOT}/ws_robot/install/setup.bash"
    echo "[env_robot] ws_robot overlay 已挂载"
else
    echo "[env_robot] ws_robot 尚未构建（见 §4）"
fi

echo "[env_robot] DOMAIN=$ROS_DOMAIN_ID  LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY  RMW=$RMW_IMPLEMENTATION"
EOF
chmod +x /home/astribot/Downloads/astribot_sdk_aarch64/env_robot.sh
```

**验证（关键是 domain 必须是 25、localhost_only 必须是 0）**：

```bash
# 【机器人端执行】
cd /home/astribot/Downloads/astribot_sdk_aarch64 && source env_robot.sh
python3 -c "
from astribot_sdk.core.astribot_api.astribot_client import Astribot
print('SDK import OK')"    # 只 import，不建会话，不会让机器人动
```

### 3.2 Fast DDS：用厂商已有的两层，不要加第三层

⚠️ **对我在 [`real_robot_deployment.md` §3.2](real_robot_deployment.md) 的方案做修正。**
实机上 DDS 隔离**已经有两层**，都不需要我们再做：

**第一层 —— 厂商启动脚本的路由 + 防火墙**（`/home/astribot/astribot_orin_startup.sh` 实测）：

```bash
sudo /sbin/route add -net 224.0.0.0 netmask 224.0.0.0 dev eno1   # 多播整段钉到 eno1
sudo iptables -I INPUT  -i wlP1p1s0 -d 239.255.0.1 -j DROP       # WiFi 上的 DDS 多播丢掉
sudo iptables -I OUTPUT -o wlP1p1s0 -d 239.255.0.1 -j DROP
```

这一层管的是厂商栈那些进程（它们用
`FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml`）。
它恰好绕开了 `interfaceWhiteList` 只过滤单播、挡不住 SPDP 多播那个已知缺陷。

**第二层 —— SDK 自己 `env.sh` 的接口白名单**（`env.sh:101-102`，实测生效）：

```
[env.sh] Fast DDS whitelist = 192.168.0.11 (只用 192.168.0.x 网段)
FASTRTPS_DEFAULT_PROFILES_FILE=.../config/fastdds_whitelist_192.xml
```

这一层管的是**我们**这条链路上的进程（因为 `env_robot.sh` 会 source 它）。

> **所以我们什么都不用配。** 不要另外 export 自己的 profile——
> 三份 profile 互相覆盖时，`is_default_profile` 谁最后生效很难说清，
> 而症状是"话题可见但零消息"，排查成本极高。
>
> 现场只需确认这两层都在：
>
> ```bash
> # 【机器人端执行】
> sudo iptables -L INPUT  -n | grep 239.255
> sudo iptables -L OUTPUT -n | grep 239.255
> ip route show | grep 224.0.0.0
> echo "$FASTRTPS_DEFAULT_PROFILES_FILE"      # source env_robot.sh 之后
> ```

---

## 4 · 构建流程

### 4.1 先补依赖（§0.5，这是最大的一块工作量）

`/opt/ros/humble` 只有 118 个包，`robot_state_publisher`/MoveIt/Nav2/slam_toolbox 全缺。
走 WiFi 装（apt 出网走 `wlP1p1s0`，不要指望 eno1）。

```bash
# 【机器人端执行】① 核对出网确实不走 eno1，否则 apt 会卡死
ip route get 223.5.5.5 | head -1        # 期望 dev wlP1p1s0

# ② 补依赖。ros-humble-desktop 会带上 rviz2 等一大堆，
#    实机若不需要图形界面可只装下面这些具体包。
sudo apt update
sudo apt install -y \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-tf-transformations \
  ros-humble-moveit ros-humble-moveit-planners-ompl \
  ros-humble-moveit-simple-controller-manager ros-humble-moveit-kinematics \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-smac-planner \
  ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-tf2-tools ros-humble-tf2-sensor-msgs \
  liburdfdom-tools

# ③ 复核
source /opt/ros/humble/setup.bash
for p in robot_state_publisher xacro moveit_ros_move_group moveit_planners_ompl \
         nav2_bringup nav2_mppi_controller slam_toolbox pointcloud_to_laserscan; do
  ros2 pkg prefix $p >/dev/null 2>&1 && printf 'OK      %s\n' "$p" || printf 'MISSING %s\n' "$p"
done
```

SDK 侧的 pip 依赖（rosdep 覆盖不到，只在 `sdk_session.py:41` 的运行期自检里出现）：

```bash
# 【机器人端执行】必须带 --no-deps，否则会顶掉被测试锁死的 numpy 版本
python3 -c "
import importlib
for m in ('filterpy','tabulate','h5py','numpy'):
    try: print('OK     ', m, importlib.import_module(m).__version__)
    except Exception as e: print('MISSING', m, type(e).__name__)"
# 缺哪个装哪个：
# pip install --no-deps filterpy tabulate h5py
```

> ⚠️ **不要整段跑 `install.sh`**：它第 78-88 行用 `pip install -U ... numpy==1.22.4`
> 且**不带 `--no-deps`**，会顶掉 numpy；还会往 `~/.bashrc` 追加 source 行。
> 而它唯一实机需要的那段（建 `/opt/astribot_ros` 目录）在这台机器上**已经不需要**了——
> 该目录已存在且厂商栈正在用。

### 4.2 排除仿真专用包

```bash
# 【机器人端执行】
D=/home/astribot/Downloads/astribot_sdk_aarch64
# aws 世界包我们没同步；若将来同步了，必须 ignore 掉（find_package(gazebo_ros REQUIRED)）
[ -d $D/ws_robot/src/aws-robomaker-small-warehouse-world ] && \
  touch $D/ws_robot/src/aws-robomaker-small-warehouse-world/COLCON_IGNORE

# livox 驱动：实机**已有两份**，不要再编我们那份
#   厂商:      /opt/astribot_ros/software/livox_ros_driver2
#   Voxel-SLAM: /home/astribot/SLAM/vxlm-slam/install/livox_ros_driver2
[ -d $D/ws_robot/src/livox_ros_driver2 ] && \
  touch $D/ws_robot/src/livox_ros_driver2/COLCON_IGNORE
```

> **不要 ignore `astribot_s1_gazebo_bringup`**：它自身只 `find_package(ament_cmake)`，
> 能正常构建；而 [`hardware_livox.launch.py:45-47`](../ws_robot/src/astribot_s1_perception/launch/hardware_livox.launch.py#L45-L47)
> 在**运行时**要用它的 `share/astribot_s1_controllers.yaml`——实机 launch 依赖仿真包的 share，
> 这是既有耦合。

### 4.3 分层构建

```bash
# 【机器人端执行】
cd /home/astribot/Downloads/astribot_sdk_aarch64/ws_robot
source /opt/ros/humble/setup.bash

# 层 1：接口，后面全依赖它
colcon build --symlink-install --packages-select astribot_bridge_msgs

# 层 2：桥接层（实机控制通路，零 Gazebo 依赖）
source install/setup.bash
colcon build --symlink-install --packages-up-to astribot_trajectory_bridge

# 层 3：模型 + 规划（这一层触发 astribot_config mesh 安装，验证 §0.3 的三层上溯）
colcon build --symlink-install \
  --packages-select astribot_s1_description astribot_s1_moveit_config astribot_s1_manipulation

# 层 4：感知 / 自主 / 导航
colcon build --symlink-install \
  --packages-select astribot_s1_perception astribot_s1_autonomy \
                    astribot_s1_navigation astribot_s1_dynamics_coupling \
                    astribot_s1_gazebo_bringup astribot_s1_chassis_effort_drive
```

**Orin 内存**：4 个包真正编 C++（`astribot_s1_manipulation` 带 MoveIt/OMPL/Eigen、
`astribot_s1_autonomy` 带 PCL）。并行度过高会 OOM：

```bash
colcon build --symlink-install --parallel-workers 2 --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**务必在 tmux 里跑**——WiFi 是运维口，掉线会杀掉编译：

```bash
tmux new -s build
```

### 4.4 构建后校验

```bash
# 【机器人端执行】
cd /home/astribot/Downloads/astribot_sdk_aarch64/ws_robot
grep -rn "error:" log/latest_build/*/stdout_stderr.log 2>/dev/null | head

# mesh 是否真的装进 share（§0.3 的直接判据）
ls install/astribot_s1_description/share/astribot_s1_description/meshes
ls install/astribot_s1_description/share/astribot_s1_description/meshes/s1_gripper | head -3

# 可执行文件
source install/setup.bash
ros2 pkg executables astribot_trajectory_bridge   # 期望 bridge_container / state_bridge_node / joint_map_probe

# 离线测试（aarch64 上重跑一遍，暴露架构相关差异）
colcon test --packages-select astribot_trajectory_bridge astribot_s1_perception \
            --event-handlers console_direct+
colcon test-result --verbose
```

**通过判据**：无 `error:`、mesh 两个目录都在、可执行文件齐、
桥接 123 个底盘用例 + 38 个写入闸门用例 + 感知 48 个 `cloud_to_grid` 用例全过。

---

## 5 · 日常同步流程（固化）

### 5.1 一键同步脚本

```bash
# 【本地工作站执行】保存为 ~/bin/sync-to-orin.sh
cat > ~/bin/sync-to-orin.sh <<'EOF'
#!/usr/bin/env bash
# 同步 ws_robot + docs 到实机 aarch64 SDK 目录。
# 默认 dry-run；加 --go 才真传。
set -euo pipefail

SRC=/home/yjh/WorkSpace/astribot_sdk_ros2
DST_HOST=orin
DST_DIR=/home/astribot/Downloads/astribot_sdk_aarch64

DRY="-n"
[ "${1:-}" = "--go" ] && DRY=""

EXCLUDES=(
  --exclude='build/'   --exclude='install/'  --exclude='log/'
  --exclude='__pycache__/' --exclude='*.pyc' --exclude='.coverage'
  --exclude='.git/'    --exclude='maps/'
  --exclude='aws-robomaker-small-warehouse-world/'
)

echo "=== 同步 ws_robot ${DRY:+（dry-run，加 --go 才真传）} ==="
rsync -avh $DRY --delete "${EXCLUDES[@]}" "$SRC/ws_robot" "$DST_HOST:$DST_DIR/"
echo "=== 同步 docs ==="
rsync -avh $DRY "$SRC/docs" "$DST_HOST:$DST_DIR/"

if [ -z "$DRY" ]; then
  echo "=== 同步后校验 ==="
  ssh "$DST_HOST" "
    D=$DST_DIR
    echo \"包数: \$(find \$D/ws_robot/src -maxdepth 1 -mindepth 1 -type d | wc -l)\"
    echo \"文件: \$(find \$D/ws_robot/src -type f | wc -l)\"
    file -b \$D/astribot_sdk/core/common/robotics_library_py/_robotics_library_py.so | cut -c1-42
  "
fi
EOF
chmod +x ~/bin/sync-to-orin.sh
```

用法：

```bash
sync-to-orin.sh          # 先看要传什么
sync-to-orin.sh --go     # 确认后真传
```

### 5.2 增量重编（只改了 python 就不用重编）

| 改了什么 | 要做什么 |
|---|---|
| 纯 python（`astribot_trajectory_bridge` / `astribot_s1_perception` 等 ament_python 包） | `--symlink-install` 下**改完即生效**，不用重编。但**改了 `setup.py` / `package.xml` 必须重编** |
| yaml / launch / xacro（`install(DIRECTORY)` 装的） | 需 `colcon build --packages-select <包>`；`--symlink-install` 对 `install(DIRECTORY)` 装的文件**不一定**是软链 |
| C++（manipulation / autonomy / bridge_msgs） | 必须 `colcon build --packages-select <包>` |
| `CMakeLists.txt` | `rm -rf build/<包> install/<包>` 后重编 |

### 5.3 建议：给 `ws_robot` 单独建 git 仓库

rsync 无版本信息，出问题时第一句话"装的是哪个版本"就答不上来。
本地当前有 **30 个未跟踪条目**（整个桥接层未提交），这个状态本身就是风险。

推荐路径：

```bash
# 【本地工作站执行】
cd /home/yjh/WorkSpace/astribot_sdk_ros2
git add ws_robot docs && git commit -m "桥接层 + 实机部署方案入库"
git remote add origin <公司 GitLab>/astribot_ws_robot
git push -u origin chassis-effort-drive
```

```bash
# 【机器人端执行】此后日常
cd /home/astribot/Downloads/astribot_sdk_aarch64/ws_robot && git pull
git rev-parse --short HEAD > ~/deploy_check/ws_robot_sha.txt   # 留痕
```

注意：`ws_robot` 作为独立 repo clone 进 SDK 目录后，
目标端 SDK 的 `.gitignore` 里要加 `ws_robot/`，否则厂商 repo 会看到一堆未跟踪文件。

---

## 6 · 风险清单

| # | 风险 | 后果 | 规避 |
|---|---|---|---|
| 1 | 误覆盖 `astribot_sdk/` `third_party/` `astribot_msgs/` | **把机器人的 aarch64 SDK 换成 x86-64，SDK 直接不能用** | 同步白名单只含 `ws_robot` + `docs`；§2.4 有 `file` 校验 |
| 2 | 覆盖 `env.sh` | 我们那份默认 `ROS_LOCALHOST_ONLY=1`，**切断 SDK 与 192.168.0.10 的 DDS** | 只新增 `env_robot.sh`（§3.1） |
| 3 | `ws_robot` 放错位置 | mesh 三层上溯失败，构建期 `FATAL_ERROR` | 必须在 SDK 根目录下；§2.4 有判据 |
| 4 | 把 x86 产物传过去 | 污染 aarch64 构建，症状是链接期怪错 | 排除 `build/install/log`（含嵌套残留） |
| 5 | 直接 `colcon build` | `/opt/ros/humble` 是最小安装，大批依赖缺失 | 先做 §4.1 |
| 6 | 跑 `install.sh` | 顶掉 numpy 版本 + 改 `~/.bashrc` | 只手工装 pip 包，带 `--no-deps` |
| 7 | 叠加自己的 Fast DDS 白名单 | 与厂商已有的路由+iptables 方案打架 | 先读 §3.2，不 export 我们的 xml |
| 8 | 编译时 WiFi 掉线 | 编译中断 | tmux |
| 9 | Orin 上高并行编译 | OOM | `--parallel-workers 2` |
| 10 | rsync 无版本信息 | 排障时不知道装的哪版 | §5.3 单独建 repo |

---

## 7 · 完整流程速查

```
【本地】① sync-to-orin.sh              # dry-run 看清范围
【本地】② sync-to-orin.sh --go         # 真传（约 3 MB）
【机器】③ 校验载荷 + SDK 仍是 aarch64   # §2.4
【机器】④ 建 env_robot.sh              # §3.1，不覆盖 env.sh
【机器】⑤ 读厂商 fastdds/iptables 现状  # §3.2，不叠加
【机器】⑥ apt 补 ROS 依赖 + pip --no-deps # §4.1  ← 最大工作量
【机器】⑦ COLCON_IGNORE 仿真/重复包     # §4.2
【机器】⑧ tmux 里分层 colcon build      # §4.3
【机器】⑨ 构建校验 + 离线测试重跑        # §4.4
```

> 前置条件：`real_robot_deployment.md` §1 的校验清单里，
> **§1.3（domain）已实测为 25、§1.5（原生 SDK）已确认 aarch64、§1.6（`/opt/astribot_ros`）已存在**——
> 这三项不必再做。剩下要现场确认的是 §1.2 的出网选路（`ip route get 223.5.5.5` 必须走 WiFi，
> 否则第 ⑥ 步的 apt 会卡死）。
