# AstriBot 导航栅格

接收 `astribot_slam_msgs` 关键帧、回环位姿修正和当前扫描，生成 `/map` 与最终 PGM/YAML。节点 `nav_prob_grid_node` 保留运行名称，地图所有权与定位算法分离；不提供第二套定位或点云转发链。

通过感知包的统一 SLAM launch 启动。参数、会话提交和验证见 [统一 SLAM 参考](../../../docs/SLAM_INTEGRATION_REFERENCE_20260918.md)。
