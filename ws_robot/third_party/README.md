# 数值依赖源码

- `gtsam/`：从本仓库原 `src/SLAM/ThirdParty/GTSAM/sc4.2.0` 迁移，版本 4.2.0，保留原版权、许可证和算法实现。项目改动为禁止启用其内置 Eigen，删除重复 Eigen 副本。
- Eigen 的唯一源码在 `../src/astribot_eigen_vendor/vendor/eigen`，版本及文件摘要由该包管理。

`tools/robot/build_slam_dependencies.sh` 在目标架构本机构建，生成目录为 `ws_robot/deps`，不提交二进制。此目录不作为 colcon 工作空间扫描，也不包含另一套机器人 launch 或兼容启动脚本。
