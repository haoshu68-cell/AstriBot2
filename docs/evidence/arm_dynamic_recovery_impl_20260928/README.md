# 新分支首段轨迹与参数实验

参见 [summary.json](summary.json) 的来源/二进制哈希及 [参数依据](../../ARM_DYNAMIC_RECOVERY_PARAMETERS_20260928.md)。基线 `1e5c7857`，本轮几何实现提交 `dca8d1c2`。新工作目录及私有构建/安装均未覆盖旧分支、主工作区或共享部署。

## 验证结果

- [parameter_regression.log](parameter_regression.log)：6/6 CTest 目标通过，68 个注册用例中 67 通过，`ObservedCloud.SameCloudProcessingBenchmark` 跳过。安装版本的 gtest XML 未编码该跳过；以 [ctest_details.log](ctest_details.log) 中的 SKIPPED 为准。不将跳过记为性能通过。
- [coverage_test.log](coverage_test.log)：5 cm 地图中 20 mm 试棒的有效回波入图与射线空闲体素反例，两项通过。
- [depth_decimation_final/result.json](depth_decimation_final/result.json)：实际安装的 C++ CPU 投影进程、完整 640×360 合成深度图、同帧对照 12 组。抽稀 4 有 4 组完全丢失试棒；抽稀 1 在全部 12 组保留试棒，逐组点数符合输入几何真值。这是**现有抽稀配置不满足所选目标的失败证据**，不是相机检测或动态避障通过。
- 原图用有限圆柱解析求交生成，保留图像哈希、内参文件哈希、实际二进制哈希、进程命令和加载库路径。逐组记录所有已发帧及已收到的两侧输出，只比较相同采集时间的结果。5 Hz 合成输入流用于功能测试，不能当作相机/整栈实时性能证据。
- 两个投影子进程退出码均为 0；没有机器人动作。仿真资源曾在只读检查时被其他任务使用，之后 [shared_resource_check.json](shared_resource_check.json) 显示锁可取得，检查后立即释放；本轮未取得持续租约或启动仿真。

## 失败与诊断记录

- `build.log`：初次编译误用本机不支持的 Duration 构造，修正为 `from_nanoseconds`；`build2.log`、`final_build.log` 为后续成功记录。
- `depth_decimation01`–`07`：单帧实验没有成对输入，计数表现为 received=1、paired=0；增加端点确认、健康信息、局部只读输入观测，并试过本会话 UDP 配置。没有据此断言投影算法错误或相机漏检。
- `depth_crop01`：64×64 裁剪只用于定位消息传输问题，仍未完成两进程对照。它不是正式矩阵，不替代原完整分辨率场景。对应临时 `crop_probe.py` 仅作为失败实验记录保存。
- `depth_decimation08`：改为有界连续输入并保留每帧情况；头部 6 组完成，躯干首组超时。健康信息显示两端已处理输入，但抽稀 1 的完整输出未被观察到；不把发送或处理计数当接收成功。
- `depth_decimation09`、`depth_decimation_final`：使用夹具专属 [depth_fixture_transport.xml](depth_fixture_transport.xml)，显式配置本机 UDP 发现及 16 MiB SHM 段，完整矩阵完成。仍有启动阶段未同时收到输出的帧，已保留在每组 `captures` 中。本次是限定场景的可用配置，不宣称已定位全部丢帧根因或修好原部署。未删除任何全局 SHM 文件、修改系统缓冲区或接管外部进程。

## 复现

私有构建环境快照为 [env.sh](env.sh)。它依赖原有本机 MoveIt、MTC 和旧分支依赖安装；不能当作可跨机器的完整部署包。构建本分支的 `astribot_s1_manipulation` 到独立目录，使用 `BUILD_TESTING=ON`，在空闲隔离 domain 下运行该包 CTest。

投影实验使用仓库中的验证脚本；`--binary` 为报告中实际使用的 C++ 可执行程序路径，两个 profile 为报告中的冻结 YAML。输出目录必须不存在，两个投影进程由脚本独占创建并收尾。

```bash
source /tmp/astribot_arm_dynamic_recovery_impl/env.sh
export ROS_DOMAIN_ID=147 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/astribot_arm_dynamic_recovery_impl/depth_fixture_transport.xml
python3 tools/validation/measure_arm_depth_decimation.py \
  --binary /home/yjh/WorkSpace/astribot_sdk_ros2/runs/mainline_20260928/departure_no_deviation/deployment/prefixes/astribot_s1_perception_components/lib/astribot_s1_perception_components/rgbd_pointcloud_node \
  --head-profile /tmp/astribot_arm_dynamic_recovery/simulation/frozen/astribot_s1_description/share/astribot_s1_description/config/simulation_navigation_full/camera_head_rgbd.yaml \
  --torso-profile /tmp/astribot_arm_dynamic_recovery/simulation/frozen/astribot_s1_description/share/astribot_s1_description/config/simulation_navigation_full/camera_torso_rgbd.yaml \
  --output /tmp/arm_depth_decimation_reproduction
```

先核对 domain 未被其他会话使用及报告中的文件哈希。脚本返回成功仅表示功能实验完成，必须检查 `lost_with_decimation_*`；当前配置 4 的丢失是预期复现的缺陷，不能将 `completed=true` 当作运行范围验收通过。

尚未覆盖：真实镜头/深度误差/材质与遮挡、所有像素相位、动态目标、整栈感知时延、首控制周期的在线遥测绑定、物理跟踪和停止范围、自动恢复/重规划及真机。原分支其他包的 12 项基线回归分歧未在本轮处理。整体未合并。
