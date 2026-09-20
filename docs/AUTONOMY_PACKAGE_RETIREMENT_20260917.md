# astribot_s1_autonomy 包退役与迁移

日期：2026-09-17。按用户要求删除旧 ROS 兼容包，保留已拆分的业务实现及组合调试功能。当前 `ws_robot/src` 为 19 个 ROS 包：17 个项目实现/接口包、2 个第三方包。

## 变更范围

- 删除 `ws_robot/src/astribot_s1_autonomy`：包清单、CMake、4 个单功能 launch 转发、4 个可执行别名、4 个配置别名及旧包组件发现索引。
- 将独有的 `autonomy_bringup.launch.py` 和 `rviz/autonomy_debug.rviz` 迁入 `astribot_s1_exploration`，补充安装规则及感知组件/RViz 运行依赖。
- 组合入口直接读取感知组件包和探索包的配置，保留原节点名、组件/独立进程模式、开关、话题覆盖、参数与原组件类 ID。
- 更新当前架构、操作说明和失效链接；历史报告标注后续退役，不把旧验证记录改写成新验证结果。

本次没有修改任何 C++ 算法或 YAML 参数。`astribot_s1_autonomy::` 命名空间和 `include/astribot_s1_autonomy/...` 是新实现包导出的源码接口，继续保留；它们不表示旧 ROS 包仍存在。不再支持 `find_package(astribot_s1_autonomy)` 或以旧包名查找组件资源。

## 命令迁移

以下名称均保持，替换所属包即可：

| 功能 | 新包 | launch / ros2 run |
|---|---|---|
| 点云切片 | `astribot_s1_perception_components` | `slice_scan.launch.py` / `pointcloud_slice_scan_node` |
| Livox 转换 | `astribot_s1_perception_components` | `livox_custom_to_pc2.launch.py` / `livox_custom_to_pc2_node` |
| 前沿建议 | `astribot_s1_exploration` | `frontier_explore.launch.py` / `frontier_explorer_node` |
| 探索任务协调 | `astribot_s1_exploration` | `exploration_coordinator.launch.py` / `exploration_coordinator_node` |
| 感知与前沿建议组合 | `astribot_s1_exploration` | `autonomy_bringup.launch.py` |
| 调试 RViz | `astribot_s1_exploration` | `share/astribot_s1_exploration/rviz/autonomy_debug.rviz` |

```bash
ros2 launch astribot_s1_exploration autonomy_bringup.launch.py
# 或采用独立进程模式；两条命令不要同时执行
ros2 launch astribot_s1_exploration autonomy_bringup.launch.py use_composition:=false use_rviz:=true
```

组合入口只启动扫描和前沿位姿建议，不启动 Nav2 或探索任务协调器。正常导航/自主探索继续使用[仿真](manuals/SIMULATION_OPERATIONS.md)或[真机](manuals/HARDWARE_OPERATIONS.md)系统入口。

外部 C++ 消费者按所需功能直接 `find_package` 并链接 `astribot_autonomy_core`、`astribot_s1_perception_components` 或 `astribot_s1_exploration`，不通过旧包间接导出。组件加载的 package 参数切换到新包，类 ID 保持；旧单体库 ABI 不恢复支持。

## 本地安装产物

确认没有进程以参数、已映射文件或打开文件使用旧包路径后，将 `ws_robot/build/astribot_s1_autonomy`、`ws_robot/install/astribot_s1_autonomy` 移至仓库外证据目录保存。源码删除后不会再由新终端的包索引发现旧包。

共享安装空间只补充迁移后的 launch/RViz 资源链接和探索包依赖索引，未替换正在使用的任何共享库。已有 shell 的环境可能残留旧前缀，应重新打开终端加载环境。构建日志作为历史记录保留，不以删除所有含旧包名字的文本为目标。

本轮未部署机器人。机器人上已有 release 不会因开发机删包自动变化；需要新部署并使用新命令。外部私有脚本未代为改写，调用旧入口会失败，应按上表迁移。

## 验证结果

| 项目 | 结果与边界 |
|---|---|
| 包与依赖 | 19 份 package.xml；无旧包反向依赖，无旧包发现项/兼容组件索引 |
| 独立构建 | core、logging、navigation_msgs、perception_components、exploration 共 5 包通过；构建环境未包含旧包 |
| 组合启动等价性 | 32 组对照全部一致：24 组有效组合、8 组双节点关闭的预期拒绝；包括独立/组件模式、RViz、时钟、级别及话题覆盖 |
| 实际节点启动 | 独立 ROS 域 197 下，组件模式与独立进程模式均启动成功；两个节点服务可用、时钟参数正确，组件容器实际加载两个预期类，未发现 Twist 发布端；两次正常退出码均为 0 |
| 当前安装入口 | 新包 5 个 launch 的 `--show-args` 通过；迁移的 RViz 资源可找到 |
| 外部 C++ 消费者 | 直接依赖新包，保留旧 include/类名的编译、链接和运行通过；无需旧兼容包 |
| 行为变更范围 | 源码哈希对照确认 C++ 与 YAML 未变，RViz 配置逐字节一致 |

这是包迁移、参数等价和组件集成验证，**没有重新运行 Gazebo 导航路线、RViz GUI 或真机运动回归**，不作为新的跟踪性能验收。原 Gazebo/导航栈未停止或重启，未发送导航目标或底盘速度。

验证脚本与证据未安装进产品包，保存在：

[/home/yjh/WorkSpace/astribot_validation/autonomy_retirement_20260917_201657](/home/yjh/WorkSpace/astribot_validation/autonomy_retirement_20260917_201657/README.md)

其中 `source_scope.json`、`launch_equivalence.json`、`launch_smoke.json`、`main_install_checks.json`、`artifact_migration.json`、构建日志及外部消费者记录可分别复核上述结论。`before/` 与 `retired_artifacts/` 保存退役前副本。

## 后续仿真回归

同日后续另行构建并运行了 Gazebo/RViz，短路线 5/5、完整路线 6/6 通过。新证据、过程误差和探索验证边界见[仿真回归记录](AUTONOMY_SIMULATION_REGRESSION_20260917.md)。上面的“未运行 Gazebo”描述保留为退役操作当轮的事实，不与后续实验混合。
