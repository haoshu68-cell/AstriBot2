# M1 依赖：导出现有场景占据语义库

2026-09-24，供 C++ 正式任务执行器复用 MTC 已有 `canonical_octomap`。不复制算法、不按兄弟包源码路径编译、不修改占据/概率判别语义。

仅修改 `astribot_s1_transport_mtc/CMakeLists.txt`：将现有实现构建为共享库，MTC 自身链接该库，安装其现有头文件并导出目标 `astribot_s1_transport_mtc::transport_scene_signature`。原 Python 兼容绑定及其构建方式保持。

独立覆盖层：`runs/mainline_20260924/mtc_export/install/local_setup.bash`。源码、产物及日志哈希见同目录 `manifest.json`；共享安装没有修改。

验证：

- 包含 MTC planner 的独立 Release 构建通过。
- 原包 CTest 3/3：执行保护、关节裕量、占据语义边界。
- 独立安装消费者仅通过 `find_package` 和导出目标，编译链接现有占据边界测试，1/1 通过；证明安装接口可消费，不是复制源码后测试。
- 首次配置缺少项目已有提取依赖 `py_binding_tools` 的前缀，已按原成功构建缓存补 `ws_robot/deps/mtc_humble/opt/ros/humble`；首次失败日志保留。没有下载或更换依赖版本。

未启动 ROS/Gazebo，不代表 M1 轨迹或整条搬运已通过。M1 后续在其独立构建中链接此目标，并按正式执行事务完成验证。
