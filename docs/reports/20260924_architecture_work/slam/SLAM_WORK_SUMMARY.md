# 引入 SLAM 后的工作汇总

主线：**接入统一 → 坐标与自滤 → 导航地图 → 存图载图 → 探索收尾 → 地图资产管理 → 禁区与会话绑定 → 工程化与稳定性。**

依据 2026-09-18 至 09-24 已有材料按依赖关系归纳；部分工作并行或交叉推进，并非严格串行开发。工程基础中包含沿用能力，不把它们全部计为本期新增。此次只整理证据，没有启动 SLAM、Gazebo 或真机。

[打开流程图](index.html) · [下载 PNG](slam_work_flow.png) · [下载 SVG](slam_work_flow.svg)

## 01  统一 SLAM 接入

将 Voxel-SLAM 作为仿真与真机共用后端。驱动直接提供 PointCloud2/IMU，由算法内部解析字段、时间并同步双雷达；补齐仿真 200 Hz IMU 及桥接。删除 CustomMsg 转换、独立预处理/融合和旧专用工作空间入口。**收益：减少重复链路，让仿真与真机遵守共同输入契约；实际真机输入质量另验。**

↓

## 02  理顺坐标与机器人回波

算法和地图直接使用 map，由独立里程计维持局部 odom；明确 map→odom 的单一发布职责。新增 /slam/pose 与底盘协方差，TF 外推保留原始时间。SLAM 前剔除机身回波，缺失或陈旧 TF 时丢弃输入，避免把机器人本体累积成障碍。此项不能替代 SLAM 退化后坐标连续性的验证。

↓

## 03  打通 SLAM → 导航地图

由 astribot_s1_mapping 管理在线概率栅格和最终 PGM/YAML，SLAM 通过关键帧与优化位姿接口供图。补齐先到位姿修正缓存与最终关键帧等待；仅在新鲜位姿下清理当前机身内部，删除历史轨迹清除和独立自清转发。统一 /map 和 /scan_from_cloud，只做一次切片，为 Nav2 提供地图与扫描。

↓

## 04  建图 → 保存 → 冷启动载图

保存流程增加最终优化、二维/三维输出等待、文件与哈希检查、原子提交 manifest。载图用持久关键帧重建描述子，修正短会话、单关键帧与稀疏残差保存边界。

已有仿真证据：run15 建图 3/3、run16 冷启动载图 3/3、同会话复测 3/3；最大到点误差 2.690 cm / 1.395°，基于 SLAM 位姿；保存 44 个关键帧、1911 条扫描位姿。载图是在相同仿真出生位置，不是任意位置重定位；首轮有采样缺口与时效告警，复测未复现不等于根因已修复。

↓

## 05  探索 → 停稳 → 会话收尾

新增 C++ mapping_session：冻结探索新目标 → 等待其导航 Action 终态 → 新鲜里程计持续停稳 → 检查存图配置 → 一次触发 finish → 等待最终数据 → 校验并提交。暂停不自动结束 SLAM，取消保存标为 CANCELED_PARTIAL，不把不可达前沿当作全图完成。

r25 使用真实 Voxel 完成静止取消保存，得到 1349 条扫描位姿、1 个关键帧和 278×416 栅格；重复取消未重复保存。该场景因 ENVELOPE_V2_NOT_READY 没有派发探索导航，**只能证明静止收尾，不能证明运动中取消或自然遍历完成。** 另外补了探索固定包络就绪检查，避免错误展示 READY。

↓

## 06  地图变为可管理的资产

增加 C++ 地图管理器、RViz 地图工位页和受管 Voxel 适配器：校验保存会话 → 复制为不可变资产 → 绑定地图/工位版本 → 停稳与控制权检查 → 受管加载 → 定位、TF、地图及双 costmap 连续就绪 → 提交活动地图。失败转为需显式处理的恢复状态，外部 SLAM 占用时拒绝接管。

导入、持久化、假依赖切图和界面门控有隔离测试；实际两地图切换/重定位尚未验收。跨楼层所需的载荷、运输与交接证据未接齐，不能因界面存在就称跨楼层可用。

↓

## 07  虚拟墙与禁区绑定 SLAM

虚拟墙/禁区随建图会话、保存 manifest 和地图版本管理；原始 SLAM 地图保留，探索和 costmap 消费叠加约束。保存完成不等于已应用，必须等待消费者确认；从建图继承的非空约束在最终地图中重新核对。

排查并修复非探索 mapping 模式遗漏 mapping_session 导致的 ZONES.NO_CONTEXT。两次独立冷启动各 120 秒，建图会话唯一、区域 token 稳定、双 costmap 与最终保护同版本确认通过；全过程静止，未执行存载图事务或区域穿越。SLAM 回环引起的旧墙坐标漂移不自动修正。

↓

## 08  工程化与稳定性补齐

按估计、导航栅格、感知组合和 SLAM 消息职责分包，迁入 Eigen 3.4.0 / GTSAM 源码与构建约束，移除嵌套工作空间及重复 Eigen；目标机器自行构建，已有预编译库 ABI 不由此自动保证。沿用统一 spdlog/session.log，补足会话状态、事件/参数、独立日志目录和仿真归属记录。

后续 map→odom 和跨域地图中继迁为 C++，相关 4/4 CTest 与安装 smoke 有记录。SLAM 与 RGB-D/推理并发压力做了分段观测、独占协议和处理健康恢复：一轮 1800 秒静止 SLAM+Nav2+重复推理通过，另一轮仍有 STALE 失败。**不把地图中继优化称为 SLAM 核心算法提速，也不把静止压力通过当作运动全负载通过。**

↓

## 后续验收出口

- 运动中 pause/resume、cancel 后停稳存图，以及真实封闭场景自然探索完成。
- 两份真实地图导入、切换、定位与双 costmap 刷新；跨楼层载荷/交接条件单独验收。
- SLAM 退化/reset、时钟回退、传感器断流后的坐标代次与恢复授权。
- 回环后的禁区坐标复核、长时运动并发、真实传感器标定与真机外部真值验收。

## 证据来源

- [docs/SLAM_CHANGE_MANUAL_20260918.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/SLAM_CHANGE_MANUAL_20260918.md)
- [docs/SLAM_ARCHITECTURE_REVIEW_20260918.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/SLAM_ARCHITECTURE_REVIEW_20260918.md)
- [docs/SLAM_SIMULATION_VALIDATION_20260918.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/SLAM_SIMULATION_VALIDATION_20260918.md)
- [docs/EXPLORATION_SLAM_FINALIZATION_20260919.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/EXPLORATION_SLAM_FINALIZATION_20260919.md)
- [docs/EXPLORATION_MAPPING_ACCEPTANCE_20260919.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/EXPLORATION_MAPPING_ACCEPTANCE_20260919.md)
- [docs/P2_MAP_STATION_TRANSACTIONS_20260919.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/P2_MAP_STATION_TRANSACTIONS_20260919.md)
- [docs/P2_VOXEL_SESSION_ADAPTER_20260919.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/P2_VOXEL_SESSION_ADAPTER_20260919.md)
- [docs/MAP_SELECTION_LOAD_INTEGRATION_20260920.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/MAP_SELECTION_LOAD_INTEGRATION_20260920.md)
- [docs/manuals/VIRTUAL_WALLS_AND_KEEP_OUT.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/manuals/VIRTUAL_WALLS_AND_KEEP_OUT.md)
- [docs/RGBD_CUDA_PROCESS_RECOVERY_20260922.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/RGBD_CUDA_PROCESS_RECOVERY_20260922.md)
- [docs/RGBD_EXCLUSIVE_REGRESSION_20260922.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/RGBD_EXCLUSIVE_REGRESSION_20260922.md)
- [docs/CPP_MIGRATION_COMPLETED_WORK_BRIEF_20260924.md](/home/yjh/WorkSpace/astribot_sdk_ros2/docs/CPP_MIGRATION_COMPLETED_WORK_BRIEF_20260924.md)
