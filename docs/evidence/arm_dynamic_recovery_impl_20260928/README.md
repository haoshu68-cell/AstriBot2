# 新分支首段轨迹与参数实验

参见 [summary.json](summary.json) 的来源/二进制哈希及 [参数依据](../../ARM_DYNAMIC_RECOVERY_PARAMETERS_20260928.md)。基线 `1e5c7857`，本轮几何实现提交 `dca8d1c2`。新工作目录及私有构建/安装均未覆盖旧分支、主工作区或共享部署。

## 按参数续做：实际部署与首周期绑定

实现提交 `df8a6042`。运行时只扩展已有 C++ 控制器/执行器和一个消息，没有新增运行节点或替代插值器。

- [controller_start/summary.json](controller_start/summary.json)：操作包 6/6 CTest 目标通过；74 个注册用例，73 通过、1 个性能用例跳过。控制器专项 19 项通过，包括新增 6 项首周期测试。并发 25 个 Goal 的每个 BOUND 记录均与该周期实际采用的轨迹端点和状态一致；这一轮 UNKNOWN 记录数为 0，不声称覆盖了所有交叠时序。
- 执行器的三个隔离 ROS 场景通过：domain150 六路首周期记录与 UUID、原始命令摘要和阶段代次绑定（含先收到首周期、后收到 Goal 响应，以及不相关 UUID）；domain152 风险后立即恢复正向状态仍撤销；domain153 保护心跳中断仍停止。均确认释放资源。domain149 是验证脚本把 NumPy uint8 当成普通 int 构造消息的失败，保留结果且未复用该 domain。
- 首周期记录为影子证据。执行器没有因收到它而批准在线 CLEAR、恢复或重规划。保留初始状态的关节顺序；将来接到几何检查时必须按最终原始命令的关节顺序使用。
- [dense_simulation/summary.json](dense_simulation/summary.json)：自有 domain95、固定六相机，把原有两路投影的抽稀改成 1，观察到 230400 点/帧。实际采集到入图的最大仿真时间差为头部 557 ms、躯干 527 ms。短窗口和观察者接收数不能代表完整帧率、最坏时延上界或覆盖验收。
- 此次导航诊断配置存在 policy-off/arm coupling 冲突，supervisor 两次拒绝后自行收尾，**整栈就绪失败**。20 mm 真实渲染试棒与动态障碍尚未运行，没有机器人运动指令。最后停稳观察没有样本，不作成功判据。
- [move_group_teardown/summary.json](move_group_teardown/summary.json)：新增更新器和原更新器，在无 Gazebo 的相同 MoveGroup 参数下均以 -11 退出。旧实验日志也有同样问题。launch 返回 0 不代表节点正常退出，原报告已更正；具体依赖生命周期修复仍待验证。

密集输入的源数据、部署清单、参数、世界统计、进程启动身份及库映射以 `.gz` 保存；[artifacts.json](dense_simulation/artifacts.json) 记录解压后的 SHA-256 和大小，逐项校验通过。这些是历史实验产物；后续私有安装已加入首周期记录，重新启动须生成新的部署清单。

构建诊断保留了：命令全局参数位置错误、旧 CMake 缓存选到旧消息包、混用两种迭代器的编译错误及修复；开环初测试图运行中改变 JTC 的只读参数，被框架拒绝。最后使用配置阶段启用开环的有效用例通过，没有为该测试增加运行时回退或修改只读契约。

复现控制器回归：按 [env.sh](env.sh) 使用私有安装，在空闲隔离 domain 运行 `astribot_s1_manipulation` 的 CTest。协议入口为 `ws_robot/src/astribot_s1_transport_native/test/verify_plan_to_hold.py`，必须选择无历史账本的新 domain；其 `--output` 的同名 `.log` 是子进程日志，外层输出应另存 `.console.log`，不能重定向覆盖它。感知只读采集入口为 `tools/validation/measure_arm_scene_input.py --output <新目录>`，先加载目标会话的实际 overlay/domain/传输配置。

## 前一阶段验证结果

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
