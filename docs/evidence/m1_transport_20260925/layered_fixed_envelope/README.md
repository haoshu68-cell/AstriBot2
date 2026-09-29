# ThreePhase 姿态对齐的分层包络元数据

最终范围以主线转达的用户纠正为准：分层切片仅用于 ThreePhase 起点/终点姿态对齐，不修改搜索、MPPI、到位调整的传统几何契约。该子任务于 2026-09-25 07:20 完成离线核心验证；ROS 和运动验证由主线统一负责。

## 最终改动

- geometry_state_node 对共享 `height_slices.yaml` 一次读取的原始字节计算 SHA256，并从相同字节解析配置。`height_profile_revision`、`ground_in_base_m` 随同一次 RobotGeometryState 发布，异步采样、模型/载荷账本、旧覆盖门槛与有效期不变。
- fixed_envelope_core 从请求指定的 reference_state_sequence 复制分层、profile 和 ground；对每个非空层施加一次原有环境 clearance，空层保留槽位。续租只更新原先允许更新的时效等字段，不重算或重复膨胀参考层。
- `installed_geometry_hash` 保持原二维算法；`height_geometry_hash` 独立绑定已发布的二维轮廓、所有层的边界和 float32 轮廓、空层、frame、clearance、profile 和 ground。仅修改单层时前者不变、后者改变。
- 共享头 `layered_envelope.hpp` 提供 `layeredGeometryHash` 与 `validateLayeredEnvelope`。后者校验层连续性/有效多边形/原始配置 SHA 形式/ground/高度覆盖/分层 hash，供对齐消费者在系统边界调用；源租约、资源权限和地图匹配仍由消费者负责。缺层和超高不使用二维回退，但不为普通 Nav 增加新的准入或撤销条件。

本轮曾计划的整栈 coverage 替换及撤销服务更换已按新范围撤回。fixed_envelope_node.cpp、geometry_state_core.hpp 均与本轮 before **逐字节相同**，保留既有 hold_observed 修复、撤销服务和四段 coverage 流程。superseded_full_stack 中仅保存撤回时的证据。没有新增地图订阅、CMake/package 依赖或 RevokeFixedEnvelope 入口。

## 已验证

| 证据 | 结果 | 范围 |
| --- | --- | --- |
| layered_envelope_hash_test | 5/5 PASS | 空层、所有身份字段、单个 float32 ULP 改变、顶点顺序规范化、缺层/非有限/非平面输入拒绝 |
| layered_fixed_envelope_test | 8/8 PASS | 同参考状态、一次 clearance、续租不重算、普通准入保持、对齐缺层/超高拒绝、独立双 hash、合法 hold 偏差 |
| fixed_envelope_core_test | exit 0 | 原权限、五方 ACK、负 ACK、源有效期与时钟边界不变量 |
| 协议验证脚本语法 | PASS | producer 两个既有协议测试新增原始 profile SHA/ground 断言；尚未执行 ROS |

首轮 hash 测试 fixture 编译因 `-Werror=misleading-indentation` 失败，拆分该语句后重新编译通过；未将首轮编译记为通过。XML、核心输出日志、独立编译完整 argv、源码 before/after、差分与 SHA256 见本目录和 manifest.json。

## 编译环境与未验证范围

独立 g++ C++17/O2 使用主线已生成的新导航消息头：

`/home/yjh/WorkSpace/astribot_sdk_ros2/runs/layered_stack_20260925/alignment_install/astribot_navigation_msgs/include/astribot_navigation_msgs`

载荷消息头：

`/home/yjh/WorkSpace/astribot_sdk_ros2/runs/task_chain_20260923_payload_offline/install/astribot_payload_msgs/include/astribot_payload_msgs`

标准消息头来自 `/opt/ros/humble/include` 各包。纯核心只链接 crypto/gtest/pthread，没有启动 ROS，也没有链接/运行节点；完整可复现命令在 `core_build_commands.json`。二进制在本目录 `bin/`。

未运行共享 colcon、ROS producer/coordinator 协议、ThreePhase 集成、Gazebo、硬件或性能基准。RobotGeometryState 新字段改变 C++ 布局，fixed_hold_flow 需要主线以同一新消息重编 transport_native 后验证，不能混链旧 ArmHold 库。本子任务未改共享 Git/index，统一构建、测试注册和 Git 提交由主线负责。
