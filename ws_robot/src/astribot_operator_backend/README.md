# astribot_operator_backend

C++ 上位机控制网关、受管建图进程生命周期及隔离假导航后端。
接口、启动配置、信任边界和验收限制见
[P0/P1 交付文档](../../../docs/P0_P1_COMPLETE_IMPLEMENTATION_20260919.md)。

生产导航 launch 集成网关退出联动；backend.launch.xml 仅供开发/独立集成。
默认 mapping_runtime 不启用新建 SLAM，会话配置须显式选择部署 profile。

开发假后端通过 `navigation_action:=/operator_fake/navigate_to_pose` 注入假 Action；生产默认仍使用 `/navigate_to_pose`，避免把演示 Action 混入实际导航图。

新增地图领域命令和地图状态门控，见 [P2 事务实现](../../../docs/P2_MAP_STATION_TRANSACTIONS_20260919.md)。
