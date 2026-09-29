# 传感器时间与硬同步触发方案

状态：2026-09-22；设计 + C++ 同步证据检查已实现；物理触发后端、相机厂商驱动绑定、真机曝光验收未实现/未验证。仅在独立 overlay 编译，未改生产默认启动。沿用既有 250 ms 新鲜度预算。

## 当前事实

- Gazebo 传感器使用共同 `/clock`，但各自按 update_rate 采样，桥接和传输独立。共同时间轴不等于共同曝光时刻。
- `camera_health_node.cpp` 对同一相机 color/depth/CameraInfo 做有界历史配对，默认最大跨度 30 ms。它不是跨相机曝光同步器。
- Livox 驱动从包的 `time_type` 分辨未同步、PTP/GPS，并保留设备时间戳和点内偏移；当前机器人关机，无法证明实际锁定状态、源时钟质量或主机对齐。
- `livox_ros_driver2/src/comm/pub_handler.cpp:265` 的 `GetEthPacketTimestamp` 在非PTP/GPS时回退到主机 `high_resolution_clock`。这种时间不是已确认的设备采集时间；`SensorTiming.clock_locked` 必须为false。驱动现有同步flag是共享状态，后续适配必须逐设备逐包读取time_type，不以另一台设备锁定推导全体已锁定。
- 本轮相机运动负载仍出现 wall-clock 新鲜度不足。触发同步解决采集关联；不会修复 USB、DDS、渲染或推理积压。两项检查必须同时保留。
- 目前不能回答“全部传感器已经对齐”，也不能从显示相同 header.stamp 推导硬同步已成立。

## 设备分工与同步网络

```text
统一时间源（PTP grandmaster / PPS + 绝对时间）
 ├─ 支持硬件时间戳的主机网卡 PHC -> 受控主机时间映射
 ├─ Livox MID-360 -> 原生 PTPv2 / GPS 同步 -> 点时间 + 内置 IMU 时间
 ├─ 独立 IMU -> 按型号选择 PPS/FSYNC/原生同步，不假设支持
 └─ 定时板 MCU/FPGA 硬件定时器 + 捕获回读
      ├─ 头 RGB-D：color、depth 分开列能力
      ├─ 头 stereo：左右成像器的厂家硬件配对能力
      ├─ 腹 RGB-D：color、depth 分开列能力
      └─ 左/右腕 RGB-D：color、depth 分开列能力
```

MID-360 官方规格明确 PTPv2/GPS；它不是按相机每一帧触发来采样。PTP 同步时钟也不意味着相机曝光相位一致。雷达帧内每点时间、IMU 每个采样时间应保留，供去畸变/插值使用。

相机型号未知，照片不能确认传感器型号、RGB 外触发能力、电压或插针。不能按“RGB-D”整体宣称两路都支持外触发。按 stream_id 建能力表，不支持的一路标为软配对或不可用，不允许作为严格硬同步组成员。

## 硬件后端落地顺序

1. 登记每台设备序列号、固件、SDK、触发输入/输出、极性、电压、脉宽范围、频率范围、曝光/读出模式、RGB/深度各自能力、硬件计数器与时间域。
2. 选一个时序控制器所有者。频率和脉宽由 MCU/FPGA **硬件定时器**输出；Linux/ROS 定时器仅用于仿真或下发配置，不作物理脉冲源。电气隔离、扇出延迟与回读线按设备规格确定，当前不写虚构 GPIO/引脚值。
3. C++ 厂商后端实现 `HardwareTriggerPort`：configure -> readback -> arm -> edge readback；disarm 在正常停机、心跳超时及错误后关闭输出。接口在 `sync_core.hpp`；当前没有绑定任一实际板卡。
4. 设备驱动先切外触发并确认 ACK/锁定，随后启用触发板。每次重新配置产生新 trigger_epoch；板卡重启/时间源变化产生新 clock_epoch。腕相机动态激活必须在新 epoch 内重新确认，不复用旧帧。
5. 触发板逐边沿输出 `{group, trigger_epoch, clock_epoch, sequence, actual_edge_stamp}`；驱动从设备帧元数据关联 trigger_sequence。绝不能仅按最近 ROS 时间戳生成“硬件关联”。设备无可信回读/计数器时保持未验证。
6. 相机原始设备时间、映射后的采集时间、曝光开始/中点定义、曝光时长、映射不确定度与版本一并保存在驱动原始日志。当前侧带接口消费归一化的 capture_stamp，原始 Image/Cloud/Imu header 不被监控器改写。
7. 工程初始检查预算：相对触发边沿的采集时差 <=2 ms，映射不确定度 <=1 ms，采集与接收新鲜度各 <=250 ms。2 ms 为候选单流预算，**不是已测精度，也不等于两路间 2 ms**。最终应以运动速度、相机滚动快门、视觉算法要求和实测预算确定；不自动放宽。
8. RGB-D 干扰要独立验证。投射器互扰可能需要分组相位错开；跨组采样此时不叫同时曝光，融合须使用各自实际时间。改变相位产生新配置版本并重新验收。

## 数据与控制流

| 生产者 -> 消费者 | 接口 | 时序与 QoS | 失效与权限 |
|---|---|---|---|
| 时间板后端 -> sync_monitor | TriggerEdge | 实际边沿；reliable，depth 256 | 缺失/冲突撤销同步证据；不控制运动 |
| 相机/雷达/IMU 驱动 -> sync_monitor | SensorTiming | 每采样侧带信息；reliable，depth 128 | 原始采集时间、源/触发/时钟 epoch、序号、锁定及不确定度 |
| sync_monitor -> 健康/融合/日志 | SyncStatus | 逐帧结果 + 20 ms 断流检查；reliable，depth 64 | valid 是时间契约结果，不是标定/识别/整机运动许可 |
| 驱动 -> 已有视觉/导航链 | Image、CameraInfo、PointCloud2、Imu | 原有 sensor-data QoS | 不复制大图，不改原时间戳，不额外成为控制器 |
| 任务 -> 同步控制器后端（后续） | 配置/arm/disarm 有界事务 | 确认配置及唯一所有者后启用 | 超时关闭触发；MTC/Nav2 仍拥有执行与取消 |

实现包：`ws_robot/src/astribot_sensor_sync`。核心无 ROS/厂商依赖；ROS 适配仅校验和有界缓存。触发历史 256，上限4096；跨话题早到帧最多64，等待50 ms，原接收时间不刷新。配对不允许延长样本有效期。

启动采用 opt-in `sync_monitor.launch.py`；默认拒绝 simulated 证据，参考时钟 epoch 默认 `UNCONFIGURED`，部署必须显式配置真实会话的时间域。当前不把新状态接到运动放行，因为设备侧带生产者尚不存在，强行启用会永久阻断任务。

未来接入健康门控必须要求同一 source_id、source_epoch、frame_sequence、capture_stamp、clock_epoch，对应 CameraHealth/WorldSnapshot 的**那一帧**有效；不能用最近一次全局 true 给其他帧授权。传感器固件与标定版本变化使关联缓存失效。

## 模块边界

- `Monitor`：只判断时间契约，状态为未观测/有效/失效；时钟回退、触发冲突锁存，需受控重启清理；源 epoch 变化需重新配置/重启，旧消息不得恢复授权。
- `HardwareTriggerPort`：未来唯一物理时序写入者，configure/arm/disarm/read_edge。板卡能力不明时不能激活；没有用 POSIX sleep 冒充硬件定时器。
- `sync_monitor`：不改变设备、不启动脉冲、不操作底盘/机械臂。阶段一影子检查已实现；设备驱动绑定和健康融合接线属于后续设备接入阶段。
- 仿真测试只发布合成边沿/采集元数据来验证消息流和故障语义，**未改变 Gazebo 相机曝光行为，不是物理硬同步验收**。

## 多场景确认矩阵

| 场景 | 预期 | 当前验证层 |
|---|---|---|
| 正常触发 + 模拟同组帧 | SIMULATED_TRIGGER_MATCH，hardware=false | C++ / ROS 合成消息 |
| 帧先于边沿到达 | 50 ms 内有界等待，不刷新 receipt | ROS 合成消息 |
| 边沿缺失/重复冲突、重复帧/触发复用 | 明确错误；冲突锁存 | C++ / ROS 合成消息 |
| 采集时间过期、未来帧、不确定度超限 | 拒绝 | C++ / ROS 合成消息 |
| 源断流、/clock 停滞 | wall watchdog 拒绝 | ROS 合成消息 / 后续 Gazebo |
| 时钟回退、设备重启、epoch 不一致 | 拒绝旧时间线，需要重新配置 | C++；真实设备待验 |
| 雷达/IMU 仅同时间域 | CLOCK_ONLY，hardware_trigger_verified=false | C++ / ROS 合成消息 |
| 多相机满负载 + 导航 + 推理 | 同步与新鲜度同时达标 | 现有新鲜度回归未放行 |
| RGB-D 投射器互扰、不同曝光/帧率 | 单独分组并检查原始图像质量 | 待设备型号/物理验证 |
| 真实曝光延迟/抖动/掉线与恢复 | 示波器边沿 + LED 图像 + 驱动计数器一致 | 未开展真机验收 |

## 后续交付门槛

1. 确认型号和板卡后补厂商 C++ 适配、能力检查、实际采集元数据；PPS/PTP 锁定是实测状态，不是配置项 true。
2. 单独验证实际边沿计数与曝光，再把对齐证明接到 WorldSnapshot 和相机健康层；保留软同步兼容但明确 provenance。
3. 用静止、机械臂运动、底盘运动、遮挡、弱纹理、相机重启、时钟失锁、网络/USB 拥塞多场景测分位数、最大偏差、丢帧、恢复时间。
4. 硬件仍关闭，本轮不会改接线、切驱动外触发或声称全传感器已同步。

参考：[Livox MID-360 官方规格](https://www.livoxtech.com/mid-360/specs)，[Livox 官方同步文档](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/common/time_sync.rst)。相机型号未确认，因此不套用 RealSense 或其他厂商的引脚/寄存器配置。

监控器重启时要求显式参考时钟 epoch；`UNCONFIGURED` 即使与输入字符串相等也拒绝。默认帧元数据没有时钟锁定权限。ROS `/clock` 停滞/回退测试使用私有命名空间时钟，不向导航仿真 `/clock` 写入数据。

### 名称映射（适配器契约，尚未接驱动）

- 当前 `head_rgbd`、`torso_rgbd`、`left_wrist_rgbd`、`right_wrist_rgbd` 相机ID保持不变，侧带信息用 `相机ID/color`、`相机ID/depth` 区分采集流。
- 当前 `head_stereo_left` 与 `head_stereo_right` 分别映射为侧带 `head_stereo/left` 与 `head_stereo/right`；不改原图像话题或 TF frame 名称。
- `lidar_front`、`lidar_back`、`imu` 是预留的设备逻辑身份；内置 IMU 和外部 IMU 必须按实际设备再拆身份，不能重复把同一数据当成两个独立来源。
