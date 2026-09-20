# ws_robot 模块状态与整理记录

> 后续变更（2026-09-17）：旧 `astribot_s1_autonomy` 包已退役，当前源码为 19 个包。下文拆分/兼容验收是当时记录；当前入口见[包退役迁移说明](AUTONOMY_PACKAGE_RETIREMENT_20260917.md)。

核对日期：2026-09-15。依据为当前 `package.xml`、CMake/setup 安装入口、launch 和代码引用。源码共 20 个 ROS 包：17 个项目实现/接口包、1 个兼容包、2 个第三方包。没有证据支持把某个完整源码包直接作为废弃包删除。以下“在用”表示有有效依赖或可执行入口，不表示本次逐包完成了仿真或真机验收。

## 包级清单

| 包 | 状态 | 实际职责及使用方 |
|---|---|---|
| `astribot_autonomy_core` | 在用；内部有退役算法残留 | 无 ROS 的感知几何/探索算法；由感知组件、探索包链接 |
| `astribot_bridge_msgs` | 在用 | SDK 桥接状态与接口；桥接及路径跟踪依赖 |
| `astribot_navigation_msgs` | 在用 | 任务、感知健康、包络、策略与规划接口；探索、策略、跟踪依赖 |
| `astribot_logging` | 在用 | Python/ROS/C++ 统一日志；多包依赖 |
| `astribot_s1_description` | 在用 | 机器人几何、关节及碰撞模型；仿真、感知、MoveIt 使用 |
| `astribot_s1_gazebo_bringup` | 仿真专用 | Gazebo、机器人和 RViz 启动；感知仿真分支使用 |
| `astribot_s1_chassis_effort_drive` | 仿真专用 | 全向轮力矩驱动；Gazebo 启动链调用 |
| `astribot_s1_perception` | 在用 | Python 预处理、融合、SLAM/地图/TF；导航及真机感知入口使用 |
| `astribot_s1_perception_components` | 在用 | C++ 自身剔除、多高度切片、Livox 转换；感知和导航调用 |
| `astribot_s1_exploration` | 在用；内部有退役分支 | 前沿搜索与探索任务；统一导航入口及真机探索脚本调用 |
| `astribot_s1_navigation_policy` | 在用，分阶段启用 | 任务仲裁、世界状态、让行/绕行、包络与最终防护；导航集成 |
| `astribot_s1_navigation` | 在用 | Nav2 配置、行为树、launch、速度链路及诊断 |
| `astribot_s1_path_tracking` | 在用 | ThreePhase、Arrival、目标检查器、规划适配和 BT；导航插件加载 |
| `astribot_s1_dynamics_coupling` | 在用 | 根据机械臂状态约束底盘速度；导航速度链路调用 |
| `astribot_s1_manipulation` | 按功能选用 | 双臂闭链规划、轨迹处理、碰撞/奇异性检查、夹爪；MoveIt 插件及独立入口 |
| `astribot_s1_moveit_config` | 按功能选用 | SRDF、IK、规划器和控制器配置；双臂规划入口加载 |
| `astribot_trajectory_bridge` | 真机链路使用 | SDK 状态、里程计、轨迹 action 和底盘命令桥接；真机启动脚本调用 |
| `astribot_s1_autonomy` | 兼容门面 | 旧 launch、4 个 ros2 run、配置路径和组件发现转发；不编译业务实现 |
| `aws_robomaker_small_warehouse_world` | 第三方、仿真专用 | 仓储场景；源码目录为 `aws-robomaker-small-warehouse-world` |
| `livox_ros_driver2` | 第三方、按硬件选用 | Livox 驱动及 CustomMsg 类型；仿真通常不运行该驱动 |

`astribot_s1_perception` 与 `astribot_s1_perception_components` 按集成层和 C++ 适配层分工，并非新旧副本。`astribot_autonomy_core` 是纯算法库，也不是旧 autonomy 包的重复实现。

## 迁移与兼容

| 旧归属 | 当前实现包 | 旧入口处理 |
|---|---|---|
| autonomy 内的几何/探索算法 | `astribot_autonomy_core` | 保留 include 路径与 C++ 命名空间 |
| 点云切片、Livox CustomMsg 转换 | `astribot_s1_perception_components` | 旧 launch / ros2 run 转发 |
| 前沿搜索、探索协调 | `astribot_s1_exploration` | 旧 launch / ros2 run 转发 |
| `libastribot_s1_autonomy_components.so` | 新感知、探索两个组件库 | 旧单体库已停止构建；不承诺旧二进制 ABI |

`astribot_s1_autonomy::...` 类名和 `include/astribot_s1_autonomy/...` 路径保留是源码接口兼容决定，不能按字符串全仓替换。新启动与新依赖直接使用对应实现包。本次仅将新包 launch 的示例命令更新为实际包名。

兼容门面目前没有其他项目包通过 package.xml 依赖，但它本身仍是公开启动入口，且拥有组合启动和 RViz 配置。其退役条件应是明确停止旧入口支持、迁移外部使用方，再单独删除；当前没有完成这一迁移。

## 确认退役或需要进一步收敛的内容

| 内容 | 当前证据 | 整理方式 |
|---|---|---|
| 旧 autonomy 单体库及其安装导出 | 当前 CMake 不构建；旧文件不在当前安装清单 | 清除本地失效安装产物，保留新组件注册和转发入口 |
| autonomy 原源码位置的安装头文件链接 | 14 个链接目标不存在；头文件已分属新包 | 清除断链，不删除新包头文件 |
| 旧速度扫参脚本的构建链接 | `aggregate_speed_sweep.py` / `run_speed_sweep.sh` 源文件已删除 | 清除两个断链 |
| 探索自举 Bootstrap | 构造阶段拒绝启用；`shouldBootstrap()` 恒 false，发布函数为空 | 后续移除不可达内部字段、参数读取和空分支，保留旧配置的明确拒绝提示 |
| 探索直接 FollowPath | 当前只接受 NavigateToPose，旧模式解析后被拒绝 | 后续集中旧参数迁移检查，保留统一任务仲裁和导航执行链 |
| 旧探索 Escape | 构造阶段拒绝启用；tick 只转 PAUSED；core 仍编译 `escape_logic.cpp` | 后续删除无调用算法时同步收敛 header 类型、成员和导出；不能删掉新策略的倒退恢复 |

后三项是源代码内部的退役残留，本次已定位但没有做控制状态机/公共 C++ 接口变更。删除这些内容应以探索启动、取消/恢复及导航故障回归为验证边界；窄通道策略的新 `start_maneuver` 与旧探索 Escape 属于不同实现。

### 可选功能不等于弃用

- `autonomous_patrol_node` 仍在 Python 入口和独立感知 launch 中，且该独立 launch 默认可启动它；统一 Nav2 总入口显式传 `autonomous_patrol=false`。这是旧的可选反应式巡游能力，不是无引用死文件。后续若统一退役，需要同时迁移独立启动契约。
- `map_domain_relay`、`cloud_to_grid_node`、`map_odom_tf_node` 按地图和定位方案启用；不因本次仿真没启动而删除。
- `map_start_cell_check`、运行指标记录器、关节映射探查和碰撞对生成器是诊断/配置工具；保留安装入口不等于每次运行。
- logging 的两个测试文件仍由 CMake 注册，不是失效的安装模块。
- `ThreePhaseController` 仍是 ArrivalController 的控制基类；SLAM、视觉和 Mark 到位接口、独立防护及桥接保护均继续保留。

## 本次整理范围

1. 重写 `ws_robot/README.md`，补齐 20 包分组、目录性质、主启动入口及文档索引，移除过时的“五包目录树”和历史故障叙事。
2. 更新导航说明中的包归属、事件触发规划及受监督启动说明；更新四个新包 launch 的示例命令。
3. 修正 navigation、dynamics_coupling、trajectory_bridge 的包描述，移除旧 VelocityControl 和“桥接仅只读”的过时表述；不修改依赖、参数或执行代码。
4. 按当前安装清单清理失效产物，并在仓库外保留清单与恢复副本。不清空整个 build/install，不删除地图和验证日志，不重建运行中的控制库。

本次状态依据源码与安装一致性检查，不构成新的窄通道、路径跟踪或真机性能验收。窄通道开发进度仍见 [设计与验证手册](NARROW_PASSAGE_REBUILD_20260914.md)。

## 整理结果与验证

- 删除 23 个本地失效产物：14 个旧头文件断链、1 个旧单体库、6 个旧 CMake 导出文件、2 个脚本构建断链。清理后 install/build 断链均为 0。
- 删除旧库前检查了 43 个安装 ELF 文件（含符号链接指向的产物），没有旧库依赖；运行进程也未映射该旧库。当前安装清单中的文件、3 个兼容组件注册及 4 个转发可执行入口保持。
- 8 个新旧 launch 的 `--show-args` 均成功；3 个组件注册均指向存在的新库。
- 独立 C++ 消费方通过兼容包发现全部新导出目标，头文件编译成功；旧单体目标不再导出。
- 4 个 launch 和 1 个 setup.py 排除文档字符串/description 后，AST 与修改前一致。20 份 package.xml 可解析，文档本地链接和 `git diff --check` 通过。
- 本次没有启动导航目标、发送运动命令或重建运行控制库。

外部证据与恢复副本：`/home/yjh/WorkSpace/astribot_validation/ws_robot_module_audit_20260915/`。其中 `cleanup_manifest.json` 记录原路径、文件摘要或链接目标；`removed/` 保存原文件，`packages.json` 保存包级依赖快照，`launch_validation.json`、`static_validation.json` 和 `consumer_*` 保存验证结果。
