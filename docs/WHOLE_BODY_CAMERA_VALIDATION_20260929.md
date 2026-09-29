# 双臂碰撞检查：相机接入验证与录屏（2026-09-29）

## 结论

**当前相机点云尚未接入新增的双臂分层碰撞检查，不能通过相机驱动的整机避障验收。**

相机点云用于普通 Nav2 局部代价地图的配置存在；WholeBodyCollisionCritic 实际读取 `/height_maps/snapshot`。本分支没有相机点云到该分层地图的在线生产链路。存档 SLAM 点云分层节点只读取关键帧 PCD，不订阅实时相机。

本次只做验证及录屏，没有新增感知功能、发送导航目标或修改生产控制逻辑。实现分支为 `codex/fixed-posture-whole-body`，被测实现提交 `e9d871b9`，没有合并开发主线。

## 成功标准与实际结果

| 检查 | 结果 | 证据层 |
|---|---|---|
| 相机点云持续输出 | 通过：头部、躯干均有持续消息和可见环境点云 | 实际 Gazebo 传感器 + ROS |
| 相机进入双臂分层地图 | 不通过：源码无在线转换链；观测期间目标地图无发布者、无消息 | 源码 + 实际 ROS 图 |
| 已有分层碰撞逻辑 | 4 项定向测试通过 | 真实 C++ 插件、ROS loopback、合成地图 |
| 相机障碍导致导航避让/停车 | 未执行，缺少相机分层地图链路；导航部署也未就绪 | 整栈阻塞 |
| 实际停车、接触、动态障碍、真机 | 未验证 | 不以插件测试替代 |

## 相机观测

独立会话 `whole_body_camera_20260929`，ROS domain 91，Gazebo partition `astribot_whole_body_camera_20260929`。使用独占性能租约；本次未干扰其他会话。相机基线为 `joint_navigation_full_v1`，无相机开关覆盖。

观察窗口为 15.002 秒墙钟时间：

| 话题 | 消息数 | 按源时间戳频率 | 每帧点数 | frame |
|---|---:|---:|---:|---|
| `/camera/head_rgbd/points` | 277 | 20 Hz | 14400 | `astribot_torso_base` |
| `/camera/head_rgbd/points_raw` | 277 | 20 Hz | 14400 | `astribot_torso_base` |
| `/camera/torso_rgbd/points` | 276 | 20 Hz | 14400 | `astribot_torso_base` |
| `/height_maps/snapshot` | 0 | 无 | 无 | 无 |

20 Hz 是模拟源时间频率；按整个墙钟窗口计数约为 18.4 Hz。点数为消息中的 width×height，不代表每个点均为有效障碍。RViz 实录确实显示了环境点云。`/clock` 在窗口内持续推进。

目标地图缺失不仅是一次 ROS 查询结果：`LayeredCollisionReader` 订阅的是 HeightSliceMaps，当前静态生产节点 `height_slice_map_node.cpp` 从 SLAM 存档读取，未找到实时相机生产者。由于完整导航启动失败，本次不能宣称普通局部 costmap 已在运行中消费相机；该连接只确认到源码/参数配置层。

## 运行环境与启动阻塞

- 被测 critic、geometry、recovery 及导航配置来自隔离目录 `runs/whole_body_nav/install`。
- 仿真/传感器基础安装来自 `/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_20260923_065115/install`。
- 初次尝试使用默认安装构建导航包，缺少两个 native 依赖；改用上述已有基础安装后导航包构建成功。日志完整保留。
- 导航启动明确失败：基础安装内缺少 `navigation_constraint_cpp`。120 秒就绪检查失败，不能把传感器运行成功记为整栈成功。
- 默认安装与基础安装里的旧底盘参数是 false；本次使用独立配置前缀将 `idle_position_hold` 设为 true，`idle_position_kp` 保持 3.0，未修改共享安装。未成功完成启动后的参数服务读回；因此该运行也不计为运动验收。
- 没有发送运动任务。未使用 odom/TF 计算运动、漂移或停稳结果；未给出实测停稳结论。
- supervisor 在失败后收回自有进程，最终 `remaining_owned_pids=[]`；另行启动的 RViz、录屏进程和独立显示也已退出。

## 定向测试

在独立 domain 92 运行并通过：

1. `whole_body_collision_critic`：真实 pluginlib 加载；空闲、旋转中间扫掠、同平面不同高度、未知、部分碰撞零权重、全无解、撤销和过期包络；包含实际 Arrival safeCommand 调用。
2. `layered_alignment_collision`：分层几何与扫掠、静态来源/未知语义、几何对照。
3. `layered_alignment_reader`：ROS 地图/包络读取。
4. `departure_controller`：Departure 每周期检查。

为录制真实进程输出又运行了同样四项测试，均通过。这是同一套测试的两次运行，不计为八项独立覆盖。测试中的分层地图由夹具构造，没有接入相机，也没有真实机器人运动。

## 录像与原始证据

所有文件位于 `/home/yjh/WorkSpace/astribot_whole_body_nav/runs/camera_validation_20260929/`。

- [46 秒录屏](../runs/camera_validation_20260929/whole_body_camera_validation.mp4)：1280×800、15 fps；前 21 秒为实际 Gazebo 相机点云的 RViz 画面，后 25 秒为真实测试进程输出。前段裁去开头 4 秒，并加事实说明字幕；没有合成机器人运动。
- [原始相机录屏](../runs/camera_validation_20260929/camera_pointcloud_screen.mp4)、[原始测试录屏](../runs/camera_validation_20260929/plugin_tests_screen.mp4)。
- [相机观测](../runs/camera_validation_20260929/camera_probe.json)：每帧时间戳、点数、发布者、订阅者。
- [测试原始输出](../runs/camera_validation_20260929/recorded_test_output.log)、[首次定向测试](../runs/camera_validation_20260929/critic_tests.log)、[Departure 测试](../runs/camera_validation_20260929/departure_test.log)。
- [会话记录](../runs/camera_validation_20260929/stack/session.json)、[完整会话日志](../runs/camera_validation_20260929/stack/session.log)。
- [版本及环境身份](../runs/camera_validation_20260929/validation_identity.json)、[查询环境](../runs/camera_validation_20260929/env.bash)。
- 同目录 `probe.py`、`camera_view.rviz`、`record_tests.bash` 和 `record_test_window.py` 为此次观测/录屏的复现入口；运行前须重新确认 domain 与仿真资源空闲。

下一步缺口是将实时相机观测真正写入分层地图，并验证空闲清除、未知区域与数据时效，再进行机器人避障运动和停车验收。当前没有为演示而将相机点云替换成仿真真值地图。
