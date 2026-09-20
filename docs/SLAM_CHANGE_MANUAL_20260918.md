# 本次 SLAM 修改手册与完整 Git diff

范围：2026-09-18 的仿真/真机 SLAM 统一、存图/载图、Eigen 纳管、源码目录迁移和接口分包收尾。此次生成手册只读取业务源码，不修改控制逻辑、不重新启动机器人、不提交或暂存主仓库修改。

**逐文件原因和完整应用 diff：**[ANNOTATED_DIFF.md](../runs/slam_unification_20260918/diff_review/ANNOTATED_DIFF.md)。每个文件按“状态 → 修改原因 → 基线边界 → `diff --git`”展开，没有截断补丁。

**全部文件索引：**[ALL_FILES.md](../runs/slam_unification_20260918/diff_review/ALL_FILES.md)；机器可读版本为 [file_inventory.csv](../runs/slam_unification_20260918/diff_review/file_inventory.csv) 和 [manifest.json](../runs/slam_unification_20260918/diff_review/manifest.json)。后者含生成时间、基线、每个文件前后 SHA-256、是否被主仓库 ignore、补丁位置与校验结果。

## 1. 基线及“本次”的准确含义

主仓库 HEAD 为 `9adddc4d78b24a9893e06ad63492eb4537f24666`（2026-09-14）。本次 SLAM 开工前工作区已经存在其他未提交的路径跟踪、窄通道、探索和清理改动，不能把当前 `git diff HEAD` 全部算成本次 SLAM。

本手册按本任务修改记录和现有代码选择 SLAM 关联文件，导出这些文件的 **HEAD → 生成时快照** 差异：

- 对共享文件，保留完整真实 Git diff，逐文件说明明确哪些变化属于 SLAM、哪些是历史累计改动；没有伪造一个不存在的“SLAM 开工前提交”。
- 原 `ws_robot/src/SLAM` 未受主仓库 HEAD 跟踪，未留存完整修改前快照。迁入的 Voxel、栅格、消息和 GTSAM 在 Git 中显示为 **A（新增纳管）**，不代表几千行原算法是本轮编写。无法从主仓库生成该旧目录的精确逐行删除/重命名补丁，其物理迁移和删除在第 4 节单列。
- Eigen 的 540 个文件是本轮纳入的官方头文件/许可证/版本标记；未改写上游头文件。
- Livox 和 Gazebo 世界是独立 Git 子模块，分别使用自己的 HEAD；不能把它们的 diff 冒充主仓库同一提交的改动。

因此，这套文件是**完整审阅资料**，不是已经切分干净、可独立构建的单个 SLAM 提交。应用补丁中的部分共享文件依赖工作区已有的历史功能；临时目录的补丁校验不等于在干净 HEAD 上完成构建。当前工作区已包含这些改动，不要再次直接应用。

## 2. 补丁分册

| 文件 | 文件数 | 内容与原因 |
|---|---:|---|
| [01_application.patch](../runs/slam_unification_20260918/diff_review/01_application.patch) | 135 | 统一算法、栅格、领域消息、启动/部署、旧链删除、Eigen 包装、消费者接线和文档；逐文件原因见注释版 |
| [02_eigen_sources.patch](../runs/slam_unification_20260918/diff_review/02_eigen_sources.patch) | 540 | Eigen 3.4.0 官方文件完整纳管，避免多份 Eigen 源码和编译配置漂移 |
| [03_gtsam_sources.patch](../runs/slam_unification_20260918/diff_review/03_gtsam_sources.patch) | 3057 | 已有 GTSAM 4.2.0 源码/资源迁入；`HandleEigen.cmake` 禁止内置 Eigen，其余第三方内容按迁移处理 |
| [04_livox_driver.patch](../runs/slam_unification_20260918/diff_review/04_livox_driver.patch) | 1 | 驱动 CMake 的两处统一 Eigen 接线；prepare 脚本可重现 |
| [05_world_residual.patch](../runs/slam_unification_20260918/diff_review/05_world_residual.patch) | 1 | IMU 插件从 world 移至模型后留下的一个空行，无运行行为影响，仍如实列出 |

合计 **3734 个文件条目**，其中绝大部分是第三方原有内容迁入/纳管；应用与两个子模块共 **137 个条目**。不把这个数字称为“修改了 3734 个算法文件”。GTSAM 中有 49 个第三方数据文件被父仓库 ignore 规则匹配，本审阅包仍完整列出，manifest 中单独标识；未悄悄改变 ignore 或暂存状态。

完整压缩包：[SLAM_DIFF_REVIEW_20260918.tar.gz](../runs/slam_unification_20260918/SLAM_DIFF_REVIEW_20260918.tar.gz)。校验文件：[SHA256SUMS](../runs/slam_unification_20260918/diff_review/SHA256SUMS)。

## 3. 按功能理解修改原因

| 功能 | 关键文件 | 修改原因及行为边界 |
|---|---|---|
| 标准化传感器输入 | `astribot_s1_slam/src/feature_point.hpp`、`voxelslam.hpp/cpp` | 驱动直接提供 PC2/IMU，算法内部解析字段、时间并同步双雷达；删除 CustomMsg 转换、独立 Python 预处理与融合 |
| 仿真惯性输入 | `astribot_s1_sensors.xacro`、`astribot_s1.gazebo.xacro`、`warehouse_sim.launch.py` | 添加 200 Hz IMU 传感器、模型系统插件和 ROS 桥接，使仿真可运行与真机相同的激光惯性后端 |
| 坐标及里程计所有权 | `map_odom_tf_node.py`、`map_odom_decompose.py` | 算法和地图直接使用 map，独立连续 odom 负责底盘局部运动；不再依赖 map→camera_init 恒等别名或 TF 差分里程计 |
| 位姿输出 | `highrate_odom.hpp` | 新增 /slam/pose 及底盘协方差，TF 外推保留源时间；过期或未完成注册时不放行导航输出 |
| 到位接口 | `arrival_controller.cpp` | 只将 slam_pose 默认话题改为 /slam/pose；本轮不重调到位控制器，完整 diff 中其他控制变动是历史累计 |
| 机身回波处理 | `robot_self_filter.hpp`、`self_filter.yaml` | 在估计前剔除机身，SLAM 与切片共享几何；缺失/陈旧 TF 丢弃点云，不把机器人回波积累到地图 |
| 唯一导航地图 | `nav_prob_grid_node.cpp` | 栅格节点同时管理在线图和最终 PGM/YAML；仅在新鲜位姿下清理当前机身内部，移除独立自清及历史轨迹清除 |
| 地图一致性 | `nav_prob_grid_node.cpp` | 暂存先到的位姿修正，最终导出等待关键帧到齐，避免回环修正或迟到数据丢失 |
| 保存会话 | `slam_session.py`、`voxelslam.cpp` | 停稳后触发最终优化，等待三维/二维输出，校验文件并原子提交 manifest；用户仍须先结束导航/探索，停稳检测不是任务互斥锁 |
| 载图与短会话 | `voxelslam.cpp`、`voxel_map.hpp`、`BTC.cpp` | 用持久化关键帧重建描述子，避免重新按十帧分组丢弃短会话；修复单关键帧/稀疏残差存图流程；优化后才标记全局注册完成 |
| 一次切片 | `sim_perception.launch.py`、`hardware_perception.launch.py`、切片组件 | 统一消费 /map_scan_filtered 并输出 /scan_from_cloud，删除第二次 pointcloud_to_laserscan |
| 静态仿真基线 | `cloud_pose_frame`、`map_provider.launch.py` | 用 SLAM 自身位姿还原点云，再由真值 TF 投影，避免 SLAM 原点高度差制造假障碍；该模式仍是静态地图基线，不是第二套 SLAM |
| 共用启动链 | `voxel_slam.launch.py`、`perception_slam_bringup.launch.py`、`nav2_full_bringup.launch.py` | 仿真与真机只注入设备/时间参数，建图和载图复用同一套估计与地图契约；拒绝非法目录、阈值和未提交地图 |
| 真机 release 自包含 | `hardware_sensors.py`、`hardware_exploration.py`、`deployed_sensors.json` | 驱动、SLAM、栅格和配置随 release 部署，统一 /map 与 /scan_from_cloud；删除外部 SLAM/helper 工作空间依赖，不保留旧机中间层 |
| Eigen 统一 | `astribot_eigen_vendor`、`build_slam_dependencies.sh`、各消费者 CMake | 同一份 3.4.0 源码，校验 SHA-256，目标级 include/link，避免目录注入和两种编译配置；预编译 PCL/ROS/SDK 不会自动重建 |
| GTSAM 构建 | `third_party/gtsam`、`HandleEigen.cmake` | 将依赖纳入项目源码，去掉 GTSAM 内置 Eigen，固定外部项目 Eigen 和非本机专属指令集选项 |
| 消息领域边界 | `astribot_slam_msgs`、`astribot_bridge_msgs/CMakeLists.txt` | 按 SLAM/导航/桥接分包，补导出运行依赖；SLAM 类型命名空间随包名迁移，字段布局不顺带重设计 |
| 构建/部署可复现 | `build_robot.sh`、`prepare_livox_driver2.sh`、`deploy_project.sh` | 干净克隆准备驱动 manifest，按顺序构建 Eigen/GTSAM，快照包含第三方源码，避免依赖开发机残留安装 |

每个具体文件的原因、A/M/D 状态和全部代码差异见 [ANNOTATED_DIFF.md](../runs/slam_unification_20260918/diff_review/ANNOTATED_DIFF.md)，第三方逐文件清单见 [ALL_FILES.md](../runs/slam_unification_20260918/diff_review/ALL_FILES.md)。

## 4. 原目录迁移、删除及生成产物

以下是文件系统层面的迁移，不伪装为 Git 已识别的 rename：

| 原路径 | 当前路径 / 处理 | 原因 |
|---|---|---|
| `ws_robot/src/SLAM/vxlm-slam/src/Voxel-SLAM/VoxelSLAM` | `ws_robot/src/astribot_s1_slam` | 遵循现有 ROS 包命名与源码布局，保留 Voxel 估计核心 |
| `ws_robot/src/SLAM/vxlm-slam/src/nav_prob_grid` | `ws_robot/src/astribot_s1_mapping` | 栅格与定位按职责独立 |
| `ws_robot/src/SLAM/vxlm-slam/src/voxel_slam_msgs` | `ws_robot/src/astribot_slam_msgs` | 保持领域接口包，统一命名 |
| `ws_robot/src/SLAM/ThirdParty/GTSAM/sc4.2.0` | `ws_robot/third_party/gtsam` | 第三方源码不再放在嵌套 ROS 工作空间 |
| 原 Voxel-SLAM 顶层 `LICENSE` | `astribot_s1_slam/LICENSE` | 保留上游 GPL-2.0 授权 |
| 原嵌套 `src/livox_ros_driver2` | 删除 | 与现有项目 Livox 子模块重复，只保留一个驱动源码来源 |
| 原 GTSAM `gtsam/3rdparty/Eigen` | 删除 | 统一依赖 astribot_eigen_vendor，不维护第二份 Eigen |
| 原 SLAM 工作空间余下启动/安装脚本、编辑器配置与示例资源 | 删除 | 不保留另一套工作空间管理与旧机启动方案 |
| 迁入 Voxel 的 `launch/`、`rviz_cfg/` 及除 `mid360.yaml` 外的设备示例配置 | 删除 | 启动归属 perception，保留当前设备参数，不维护重复入口 |

上述未跟踪旧文件缺少完整 before 字节，无法提供可逆的逐文件删除补丁。当前保留文件全部收入对应 patch；旧路径映射和物理删除范围已如实说明，未从上游下载另一版本冒充本地修改前版本。

以下为生成/验证产物，不属于源码补丁：

- `build/`、`install/` 内旧 `voxel_slam`、`nav_prob_grid`、`voxel_slam_msgs` 和已删感知节点安装残留已清理；`ws_robot/deps` 是本机构建产物。
- Livox 的 `package.xml` 和 `launch/` 由 `prepare_livox_driver2.sh` 从上游 ROS2 模板生成，不作为主仓库源码新增文件；其 Eigen 依赖变更由脚本重现。
- Gazebo/Nav2 日志、路线采样、关键帧 PCD、地图和 manifest 位于 `runs/slam_unification_20260918`，作为验收证据保存，不混入 C++/Python 源码差异。

## 5. 怎样查看和校验

优先打开注释版，直接查找文件名；需要纯 patch 时打开对应分册。例如：

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
less runs/slam_unification_20260918/diff_review/01_application.patch
rg -n '^diff --git|^@@' runs/slam_unification_20260918/diff_review/01_application.patch
cd runs/slam_unification_20260918/diff_review
sha256sum -c SHA256SUMS
```

`manifest.json` 的 `patch_line` 可定位文件的 diff 起始行。子模块 patch 的路径相对于各自子模块根目录；01～03 相对于主仓库根目录。

生成时使用独立临时 Git 仓库：先还原选中文件的主仓库 HEAD 内容，再对捕获的当前文件生成 `--binary --full-index` patch；三个主仓库分册依次通过 `git apply --check` 和实际应用，最终 Git tree 与捕获快照一致。两个子模块分册分别在独立基线检查并应用，内容与当前文件逐字节一致。此过程未向主仓库提交、不改变其 index，也没有依赖“只看 diff 能打开”作为完整性证明。

## 6. 验证结果与未完成项

本轮源码验证结果见[SLAM 仿真验证报告](SLAM_SIMULATION_VALIDATION_20260918.md)：建图 3/3、冷启动载图 3/3、载图复测 3/3；最大到点误差 2.690 cm / 1.395°，测量来源为 SLAM map 坐标。存图含 44 个关键帧，完整性检查通过；探索完成两个目标及驻留后在候选校验阶段暂停。

首轮载图有一次采样缺口和瞬时时效告警，复测未复现，不能宣称根因已修复。退化 `system_reset` 的坐标连续性/代次恢复、长时实时性、完整历史路线、窄通道和真机验证仍待完成；详见[架构评审](SLAM_ARCHITECTURE_REVIEW_20260918.md)。本次文档/补丁整理没有新增运动验证。
