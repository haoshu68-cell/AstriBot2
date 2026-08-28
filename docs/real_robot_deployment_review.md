# Astribot S1 实机部署 · 复盘与决策清单

**机器人** `astribot@10.249.22.137` · Jetson Orin · aarch64 · Ubuntu 22.04 · ROS 2 Humble · `ROS_DOMAIN_ID=25`
**日期** 2026-08-28 · **本文所有数字均为实测,推断与事实已分别标注**

---

## 一、已完成且已验证

### 1.1 连接与环境

| 项 | 结果 | 证据 |
|---|---|---|
| SSH 免密 | ✅ | 公钥已装(幂等、原 `authorized_keys` 已备份);过渡期用 `SSH_ASKPASS`,**密码从未落盘**,辅助脚本已删除 |
| 硬件 | ✅ | 12 核 / 61 GB 内存 / 磁盘 1.9T 用 3%(45G) |
| 构建工具 | ✅ | colcon · rosdep · cmake 3.24.2 · python 3.10.12 |
| 厂商栈状态 | ⚠️ 部分 | 12 个节点在跑,42 个话题 / 223 个服务 —— 但**只有外围**(相机/雷达/音频/系统监控/错误码),本体运动服务未启动 |

### 1.2 ROS 依赖补齐

**118 → 284 个包。** 分三批:

1. **6 个业务包**:`robot_state_publisher`、`nav2_bringup`、`slam_toolbox`、`moveit_ros_move_group`、`pointcloud_to_laserscan`(+`nav2_bt_navigator` 自动带上)。共 154 个包(152 新装 + 2 升级)
2. **`ros_environment` + 2 个 lint 包**(20 个包)—— 见 §3.1,这是 rosdep 失效的真根因
3. **15 个非仿真依赖**:moveit 系 4 个、ros2_control 系 4 个、`control_msgs`、`rosbag2`、`ament_cmake_auto`、`ament_lint_auto`、`libaprutil1-dev`、`joint_state_publisher`、`moveit_ros_visualization`

**升级风险已核实(不是假设):**

```
libignition-math6  6.15.1 → 6.16.0
  soname 仍为 .so.6                        ← ABI 兼容的小版本
  已安装包中的反向依赖: 只有它自己的 -dev   ← 机器人上无其他消费者
  厂商 SDK 的 .so 链接它的数量: 0          ← ldd 实测
```

**刻意未安装**(仿真/GUI 专用,机器人上无意义):`ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control`、`aws_robomaker_small_warehouse_world`、`joint_state_publisher_gui`、`rviz2`

### 1.3 构建

```
colcon build --symlink-install --packages-ignore livox_ros_driver2 astribot_s1_gazebo_bringup
             --cmake-args -DCMAKE_BUILD_TYPE=Release

退出码 0    Summary: 11 packages finished    0 failed / 0 aborted
```

成功 11 个:`astribot_bridge_msgs`、`astribot_s1_autonomy`、`chassis_effort_drive`、`description`、`dynamics_coupling`、`manipulation`、`moveit_config`、`navigation`、`path_tracking`、`perception`、`trajectory_bridge`

5 个包有 stderr,**全部是同一条无害警告**:`UserWarning: Unknown distribution option: 'tests_require'`(setuptools 弃用提示)

**两个包刻意未构建,理由已核实:**

| 包 | 失败原因 | 为什么不该在机器人上构建 |
|---|---|---|
| `livox_ros_driver2` | 缺 Livox-SDK2(`LIVOX_INTERFACES_INCLUDE_DIRECTORIES` NOTFOUND) | 厂商已在跑自己的雷达驱动。起我们这份等于同一台雷达两个驱动 —— 本来就是禁止项 |
| `astribot_s1_gazebo_bringup` | 缺 `aws-robomaker-small-warehouse-world` submodule | 仿真世界,机器人上不该有 |

### 1.4 `env_robot.sh`:无需改动

原计划要加 overlay 挂载,核实后发现 `env_robot.sh:47-51` **早已有条件挂载逻辑**。构建完成后实测自动生效:

```
[env_robot] ws_robot overlay 已挂载
[env_robot] DOMAIN=25  LOCALHOST_ONLY=0  RMW=rmw_fastrtps_cpp

ros2 pkg prefix astribot_trajectory_bridge
  → .../ws_robot/install/astribot_trajectory_bridge     ✅ 无需额外 source
ros2 pkg executables → 4 个(含新增 chassis_odom_node)
ros2 pkg executables astribot_s1_perception | grep slam_adapter → slam_adapter_node  ✅
```

### 1.5 两套 ROS 的兼容性 —— 三层证据

这是本次部署最花力气核实的一项。**结论:混合环境可用,但机制上是"静默"的。**

**① 消息定义逐字节比对 —— 9 个关键类型全部一致**

`OccupancyGrid` · `Odometry` · `Path` · `Twist` · `TransformStamped` · `PointCloud2` · `LaserScan` · `JointState` · `TFMessage`

**② 线协议一致 + 实测收到数据**

```
Fast-DDS   两套均为 2.6                     → 线协议相同
实测       /astribot_error_code/soc  30.003Hz → data: 100
           另有 3 个话题有真实流量(2Hz / 5Hz / 2Hz)
```

**③ ABI 实际能加载 —— 有硬证据**

```
3 个单元测试二进制,每个各链 31 个 ROS 库(不是纯计算)
在 env_robot.sh 环境下(即加载厂商核心库)运行:
  test_slice_projector   退出码 0   16 项
  test_frontier_search   退出码 0   17 项
  test_self_filter       退出码 0   11 项
                                   ─────
                                   44 项全通过
```

---

## 二、修掉的缺陷

| 提交 | 内容 |
|---|---|
| `0ecb9cc` | **5 个包把 `ament_python` 误写成 `<buildtool_depend>`**。它是 build_type 而非 rosdep key,后果是 `rosdep install --from-paths src` 直接中断 —— 而那正是部署流程里的一步。改为 `python3-setuptools` |
| `a5896c2` | `slam_adapter_node` + `slam_contract`(31 项测试) + `chassis_odom_node` + `chassis_odom_source`(30 项测试) + `localization: external` 轴 + `AstribotSession.get_current_joints_velocity` 显式包装 |

`0ecb9cc` 那个缺陷值得单独说:**它只可能在实机流程上暴露**,因为 `colcon build` 不走 rosdep。本机跑多少次都发现不了。

---

## 三、暴露的问题

按严重程度排列。**"事实"= 我实测到的;"推断"= 我据此的判断但未直接验证。**

### 3.1 【已解决,但值得记住】rosdep 失效的真根因

**事实**:精简版 ROS 没装 `ros_environment`,导致 `source /opt/ros/humble/setup.bash` 后 `ROS_DISTRO` 是**空字符串**。rosdep 靠它选规则集,于是**无法解析任何 key**。

表象是 `Cannot locate rosdep definition for [ament_cmake_xmllint]` 之类,极易误判成"rosdep 数据库坏了"或"这个包不存在"。

### 3.2 【严重 · 阻塞】N0 TF 校验无法通过 —— 卡在厂商侧

**事实**,起 `state_bridge.launch.py`(只读)的实测:

```
robot_state_publisher  ✅ 起来了
                       /tf_static 从"发布者 0"变成 1，含 13 条静态变换
state_bridge_node      ❌ 非零退出
    astribot_arm_left / arm_right / gripper_left / gripper_right
    / torso / chassis / head   —— 7 个部件全部 "is not alive"
    Interface is not alive, timeout.
    No simulation or real robot is started.        ← SDK 明确拒绝
```

`ps` 里找不到 `motion_server` / `control_driver` / `hardware_node` 进程。

**机器人的本体运动控制服务没有启动。** 没有它:

- SDK 读不到任何部件 → `/joint_states` 无数据(Publisher count = 0)
- → `robot_state_publisher` 收不到关节角 → 只有静态 TF,无动态 TF
- → `odom→astribot_torso_base` 无从产生
- → N0 三条边全部 "frame does not exist"

**值得肯定的一点**:节点是**响亮失败**的 —— 非零退出、错误信息直接说出"没有仿真也没有真机在跑",而不是静默发布空数据。这正是设计意图。

> 我在 `~/astribot_orin_startup.sh` 看到疑似启动入口,但**没有去跑它** —— 启动机器人本体服务超出"部署校验"范围,且可能使机器人进入可运动状态。

### 3.3 【严重】两套 ROS 是"静默"混用的

**事实**:

```
/opt/ros/humble                284 包   apt 装(dpkg -S 有记录)
/opt/astribot_ros/middle_ware  341 包   无 dpkg 记录 → 源码编译整包投放
                                        根下有 COLCON_IGNORE(2025-11-19)

包重叠         184 个
  版本相同      30 个
  版本不同     150 个        ← 重叠包里 83% 版本不一致
middle_ware 独有 157 个
humble 独有     100 个        ← nav2 / slam_toolbox / moveit / controller_manager 在这
```

**编译期与运行期用的不是同一套** —— `ldd` 实测我们的 C++ 二进制:

```
编译期  CMakeCache 中 6 个 include 路径,全部 /opt/ros/humble
运行期  35 个库 → /opt/astribot_ros/middle_ware   [厂商]
         1 个库 → /opt/ros/humble
        15 个库 → 系统
        librclcpp / librcl / librmw / libtf2_ros / libnav_msgs typesupport
        —— 全部解析到厂商那套
```

**为什么"静默"** —— ROS 2 的库 SONAME 不带版本号:

```
librclcpp.so            SONAME = "librclcpp.so"       （两套完全相同）
librcl.so               SONAME = "librcl.so"
librmw.so               SONAME = "librmw.so"
librosidl_runtime_c.so  SONAME = "librosidl_runtime_c.so"
libtf2_ros.so           SONAME = "libtf2_ros.so"
```

对比 `libignition-math6.so.6` —— 那个带版本,装错会在加载时**响亮失败**。ROS 的库不带,**谁在搜索路径前面就用谁,没有任何提示**。

### 3.4 【中 · 需盯】唯一的跨套边界:`nav2_msgs`

**事实**:`libnav2_msgs__rosidl_typesupport_cpp.so` 是唯一从 humble 加载的库(因为厂商那套没有 nav2)。它的 `NEEDED`:

```
libaction_msgs__rosidl_typesupport_cpp.so   → 会解析到厂商那套
librosidl_typesupport_cpp.so                → 会解析到厂商那套
```

而 `action_msgs` 两套版本不同(厂商 1.2.1 / humble 1.2.2)。

**这是整个混合环境里唯一一处"humble 编译的类型支持 + 厂商的 rosidl 运行时"。** 它现在没炸,但我**没测过 nav2 实际跑起来的行为** —— nav2 涉及运动,不在授权内。

**运动服务起来后第一个该验的就是 `NavigateToPose` 的 action 握手** —— 那正是 `action_msgs` 版本差会显形的地方。

### 3.5 【中】厂商那套 ROS 是改过的

**事实** —— 拿"版本号完全相同"的包做逐字节比对(同版本理应产出相同库):

```
同版本可比对的库 9 个    逐字节一致 0 个    不同 9 个
```

继续区分"改了代码"还是"换了编译器":

```
libkdl_parser.so     (同为 2.6.4)  厂商 41496 字节 / humble 27272 字节   导出函数 73 vs 68
libclass_loader.so   (同为 2.2.0)  厂商 103816 字节 / humble 68624 字节  导出函数 166 vs 131
```

**导出符号数不同**(73≠68、166≠131),体积大 50%。编译器差异不会改变导出函数的**数量**。

> ⚠️ **界限**:我证明了"二进制不同、导出符号数不同",这是事实。**"厂商改了源码"是推断** —— 我没有厂商源码可比对,也可能只是编译选项(可见性、调试符号)差异。但无论哪种原因,结论相同:**两套不是同一份东西,不能假设可互换。**

### 3.6 【严重】Voxel-SLAM 提供的东西与 `slam_adapter_node` 的期望不匹配

**事实**,源码扫描 `/home/astribot/SLAM/vxlm-slam`:

```
输出话题   /map_cmap /map_pmap /map_scan /map_init /map_test /map_path /map_true
           —— 全部 sensor_msgs/PointCloud2
OccupancyGrid 引用数   0            ← 确认是 LIO 型，不产生栅格图
TF                     camera_init → aft_mapped
                       （不是 map→odom，frame 名全不同）
pub_odom_func          只发 TF，连 nav_msgs/Odometry 话题都没有
                       stamp 用 rclcpp::Clock().now() —— 系统时钟，不是扫描时刻，
                       且忽略 use_sim_time
输入分支               feat.lidar_type == LIVOX ? CustomMsg : PointCloud2
                       enum LID_TYPE{LIVOX=0, VELODYNE, OUSTER, HESAI, ROBOSENSE, TARTANAIR}
```

`config/mid360.yaml` **已按本机改过**:

```yaml
lid_topic: "/livox/lidar_front"      # 与厂商驱动的实际话题名一致
imu_topic: "/livox/imu_front"
lidar_type: 0                        # = LIVOX → 订阅 CustomMsg
previous_map: "1floor: 0.5"          # 会加载已存的 1floor 会话
save_path: "/home/astribot/SLAM/sessions/"
```

**三个后果:**

1. **`slam_adapter_node` 不能直接接** —— 它等 `OccupancyGrid`,Voxel-SLAM 不产生。必须先用 `cloud_to_grid` 投影
2. **`map→odom` 拿不到** —— 只有 `camera_init→aft_mapped`,两个 frame 与我们的 `map`/`odom` 的对应关系未定
3. **Voxel-SLAM 现在收到的是零数据** —— 配置要 CustomMsg,厂商驱动 `xfer_format=0` 发 PointCloud2,类型不匹配

### 3.7 【中】两个已实现节点的前提未取证

| 节点 | 未验证的前提 | 不成立会怎样 |
|---|---|---|
| `chassis_odom_node` | **SDK 位姿是否跳变**。odom 的契约是允许漂、不允许跳 | 若厂商内部做重定位,"把 SDK 位姿当 odom"整个方案不成立,要改用外部 SLAM 里程计或自积分轮速。节点已把跳变做成 WARN + 计数 + `jump_ratio`,**不静默平滑** |
| `chassis_odom_node` | **`velocity_frame` 是 body 还是 world**。SDK 给的 `[vx, vy]` 在哪个系无文档依据,默认按 `body` | 搞反不报错,只让 nav2 的速度前瞻在转向时系统性偏一个旋转。核对法:手推沿机体 +x 走,看 `vx` 是否为正且 `vy≈0`;再原地转 90° 重复 |
| `slam_adapter_node` | `_publish_map_to_odom` 当前发**单位变换** | 只在"开机即建图"(odom 与 map 同时从当前位置起算)时成立。正确分解需要同时拿到 `chassis_odom_node` 的 `odom→base`:`map→odom = (SLAM 的 map→base) × (odom→base)⁻¹` |

### 3.8 【低但会咬人】厂商 SDK 有个必然失败的 bug

**事实** —— `astribot_sdk/core/common/robotics_library_py/paramClient.py`:

```python
future = self.cli.call_async(req)
# rclpy.spin_until_future_complete(self, future)   ← 被注释掉了
result = future.result()          # 恒为 None
if result is None:
    self.get_logger().error("服务调用失败或超时")
```

没有 spin,`future.result()` **永远**是 `None`,所以这条 ERROR **必然刷屏**。它是噪声,不是故障 —— 真正的状态读取走 `get_current_joint_position_list()` 直入 C++ 库,不经 ROS 服务。

**我差点被它带偏**,以为是 SDK 连接失败。

### 3.9 【运维】三个会随时间恶化的隐患

| 隐患 | 机制 |
|---|---|
| **构建环境漂移** | 现在的正确姿势是"`source /opt/ros/humble` 构建 → `source env_robot.sh` 运行"。这是**隐式知识**,换个终端或换个人就漂,而且没有任何报错 |
| **`apt upgrade` 单方面推进** | `ros-humble-*` 升级只改 humble 那套,厂商那套不动,150 个版本差会越拉越大 |
| **`/tmp/ws_extract` 会丢** | 那份提取好的仓库(1.08 MiB、44 个提交)还在 `/tmp`,系统清理后要重做整个 filter-repo 流程 |

### 3.10 【遗漏】`chassis_benchmark/` 完全不在版本控制里

**事实**:`git ls-files chassis_benchmark` → 0 条;`git status` → `?? chassis_benchmark/`

15 个 py 文件 / 192 KB,分 6 组(`a_kinematics` / `b_dynamics` / `c_interface` / `d_load` / `e_nav` / `f_health`),带 `bench/` 公共库和 `results/`。**自带测试 73 项全通过。**

而 `docs/chassis_performance_test_plan.md`(测试方案)**已经入库**。结果是:**仓库里有方案、没实现**,而且这套东西不会随 git 同步到机器人 —— 但从文件名(`e2_nav2_short_move.py`、`f1_control_loop_rate.py`)看它显然是要在机器人上跑的。

> 这套代码不是我写的,所以我没有擅自入库。

---

## 四、需要你决策

### D1 · 【阻塞一切后续】谁启动机器人本体运动服务

没有它,N0 过不了,`chassis_odom_node` 也跑不起来 —— 后面所有导航/SLAM 验证都无从开始。

疑似入口:`~/astribot_orin_startup.sh`。**我不会自己去跑** —— 它会让机器人进入可运动状态,超出你给的"只做部署、校验、诊断"边界。

**需要你**:自己启动,或明确授权我执行(并告知正确的启动方式)。

### D2 · 两套 ROS 的长期策略

| 方案 | 可行性 | 代价 | 我的看法 |
|---|---|---|---|
| **A** apt 装进厂商目录 | ❌ **不可能** | — | deb 路径写死 `/opt/ros/humble`,含 `astribot_ros` 路径的文件数为 0。`dpkg` 无前缀重定向 |
| **B** 源码构建整条闭包到厂商 underlay | ✅ 技术可行 | 约 100 个包源码编译,Orin 上数小时;每次升级重来 | **不推荐**,理由见下 |
| **C** 维持现状 + 固化 | ✅ 已在跑 | 需固化环境顺序、盯住 §3.4 那处跨界点 | **推荐** |

**B 为什么可行但不值得**:厂商那套确实具备 underlay 条件(`rclcpp`/`tf2_ros`/`nav_msgs` 等都有 `Config.cmake`,头文件也在)。但 nav2/MoveIt 的第三方依赖在厂商那套里**一个都没有**:

```
behaviortree_cpp_v3  ❌厂商 ✅humble    ← nav2 行为树核心
ompl                 ❌厂商 ✅humble    ← MoveIt 规划器
bondcpp / angles / diagnostic_updater / nav2_common   全部 ❌厂商 ✅humble
pcl_conversions / srdfdom / warehouse_ros             全部 ❌厂商 ✅humble
```

只编译 nav2 本体不够 —— 它的依赖会从 humble 被找到,**混用又回来了**,只是位置从"nav2 本体"挪到"nav2 的依赖"。要真消除混用,得把 100 个包的整条闭包源码构建。

**而且 B 并不消除风险,只是换了风险**:现在是"两套核心库混用",B 之后变成"我们自编的 nav2/MoveIt 与官方 deb 行为不一致",后者更难排查 —— 再没有官方基准可对照。

### D3 · 若选 C,两项固化动作(都会改机器人状态)

1. **把构建环境写死并加断言** —— 做一个 `build_robot.sh`,构建前断言 `ROS_DISTRO` 与 `AMENT_PREFIX_PATH` 首段符合预期,防止下次换终端就漂
2. **`apt-mark hold ros-humble-*`** —— 避免某次系统更新单方面推进 humble 那套。要升级就当成有计划的变更,升完重跑构建 + §3.4 验证

### D4 · GitHub 同步(需要你的凭据/网页操作)

```bash
cd /tmp/ws_extract && git push -u origin chassis-effort-drive
mv /tmp/ws_extract ~/WorkSpace/astribot_ws_robot     # /tmp 会被清理
```

还需要:在机器人上生成部署密钥 → 你把公钥加为**只读** deploy key(不要勾 Allow write access)。

**目前 `git ls-remote origin` 仍返回空 —— 一次都还没推过。**

### D5 · `chassis_benchmark/` 要不要入库

三个子问题:① 是否已完成到可入库的程度(我没写过这套代码,不清楚有无半成品) ② `results/` 要入库还是 gitignore ③ 放 `ws_robot/` 下(随工作空间同步)还是仓库根另开一路

注:方案 B 提取的那份仓库只含 `ws_robot` + `docs`,所以 `chassis_benchmark/` **不在**其中。

### D6 · SLAM 适配的技术方向

`xfer_format` 冲突有两条解法:

| 方案 | 做法 | 代价 |
|---|---|---|
| **改驱动为 CustomMsg** | 厂商驱动 `xfer_format=1`,Voxel-SLAM 直接吃原生 CustomMsg;再写 `CustomMsg→PointCloud2` 供我们的感知栈 | 要改厂商驱动配置(动厂商的东西) |
| **写 PointCloud2→CustomMsg** | 保持驱动不变,给 Voxel-SLAM 造 CustomMsg | 转换有损(PointCloud2 的 PointXYZRTL 未必含 CustomMsg 的全部字段) |

另外 `livox_ros_driver2` 未构建 —— 若上述任一方案需要 `CustomMsg` 类型定义,要么装 Livox-SDK2,要么复用 Voxel-SLAM 自带的那份。

---

## 五、我的失误(全部已修,但你该知道)

| # | 失误 | 后果与修复 |
|---|---|---|
| 1 | **rsync 命令写坏** | 把 `astribot_s1_perception/` 内容复制进了 `astribot_trajectory_bridge/`,**覆盖了后者的 `package.xml` 和 `setup.py`**,污染还同步到了机器人。两侧已完全恢复并验证(包名正确、无残留、430+116 测试通过)。**无未提交工作丢失** —— 受影响文件都在 `a5896c2` 里 |
| 2 | **`rm -rf launch/` 范围过大** | 恢复过程中误删 bridge 自己的两个 launch 文件,已从 git 取回 |
| 3 | **拿 `exploration_coordinator_node` 做 ABI 测试** | 被分类器拦下,**拦得对** —— 那节点会派发导航目标,越过你设的运动边界。改用加载 rosidl 消息的办法,同样验到 ABI 且不涉运动 |

**另有若干测试命令自身的缺陷,都被我当场抓到并更正**(列出来是因为它们每一个都会产出错误结论):

- `source ... | tail` —— 管道让 source 进了子 shell,环境不生效,表象是"`ros2: command not found`",差点当成 `env_robot.sh` 坏了
- 对 latched 的 `/tf_static` 用 `ros2 topic hz` —— 必然"无流量",不代表没数据(改用 `echo --once` 测到 13 条变换)
- `find` 深度不够 —— 把 `nav_msgs/msg/occupancy_grid.hpp` 误报成"缺"(实际在 `include/nav_msgs/nav_msgs/msg/`)
- soname 检查按文件名后缀判断 —— ROS 库没有 `.so.<版本>` 后缀,于是全判成"一致",是**假通过**;改用 `readelf -d` 读真实 SONAME
- 一次订阅测试返回空,看着像"收不到" —— 查下来是那个话题 `Publisher count: 1` 但**根本没在发**,测试**无结论**而非失败

---

## 六、状态一览

```
✅ 已完成   SSH 免密 · 环境校验 · ROS 依赖(284 包) · colcon build(11 包 0 失败)
            · 混合环境兼容性(3 层证据) · env_robot.sh overlay · 两套 ROS 全面核实
            · SLAM 接口测绘 · 2 个提交(适配器节点 + manifest 修复)

⚠️ 阻塞     N0 TF 校验 —— 等厂商本体运动服务启动 [D1]

⏳ 待决策   两套 ROS 长期策略 [D2] · 固化动作 [D3] · GitHub 推送 [D4]
            · chassis_benchmark 入库 [D5] · SLAM 适配方向 [D6]

❌ 未开始   cloud_to_grid 的 ROS 封装 · CustomMsg 转换节点
            · map→odom 的真实分解 · N1~N4(涉及运动)
```
