# 统一 Eigen 依赖

唯一源码为 `vendor/eigen`，来自 Eigen 3.4.0 官方归档。`vendor.lock.json` 记录来源、归档 SHA-256 和每个保留文件的 SHA-256；保留 Eigen/unsupported 头文件及全部上游许可证，未修改头文件，不携带上游测试或示例。具体文件的许可证以其头部及 COPYING 文件为准。

ROS C++ 包在 PCL、MoveIt、GTSAM 等依赖之前调用 `find_package(astribot_eigen_vendor REQUIRED)`，并在 package.xml 声明 `<depend>astribot_eigen_vendor</depend>`。创建目标后调用 `astribot_target_eigen(target_name)`：该函数链接 `Eigen3::Eigen` 并仅在指定目标上优先选择项目头文件，不修改整个目录的 include 设置。不要手工指定 `/usr/include/eigen3`。若其他 Eigen target 已先被载入，配置直接失败。

依赖脚本通过 `ASTRIBOT_EIGEN_STANDALONE=ON` 将同一份头文件先安装至 `ws_robot/deps/eigen` 以构建 GTSAM；colcon 再安装 ROS vendor 包。两处安装均来自这里的同一份源码，没有第二份维护分支。GTSAM 的 `GTSAM_USE_SYSTEM_EIGEN=ON` 表示使用外部提供的 Eigen，此处实际指向本包，不能再启用 GTSAM 内置副本。

版本升级必须同时重建 GTSAM 和所有源码消费者，并核对系统 PCL、ROS/MoveIt 与厂家闭源 SDK 的 ABI。当前选用与本机系统 PCL 配套的 Eigen 3.4.0；本包不会重写已经编译好的第三方二进制。不要单独改变 Eigen SIMD/内存对齐宏。
