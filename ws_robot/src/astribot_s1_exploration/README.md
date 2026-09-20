# astribot_s1_exploration

前沿候选建议与顺序探索任务。仅通过 NavigateToPose 提交导航任务，不持有速度 publisher 或 FollowPath client。默认使用导航栈的策略行为树；旧直控配置被拒绝。

[架构、迁移及质量验收](../../../docs/AUTONOMY_ARCHITECTURE_INTEGRATION.md)

[自主探索优化、默认参数及离线验证](../../../docs/EXPLORATION_OPTIMIZATION.md)

## 感知与前沿建议组合入口

```bash
ros2 launch astribot_s1_exploration autonomy_bringup.launch.py
ros2 launch astribot_s1_exploration autonomy_bringup.launch.py use_composition:=false use_rviz:=true
```

两条是替代启动方式。入口由旧 autonomy 包迁入，保留组件/独立进程、感知/前沿开关、话题覆盖及原参数；它提供扫描和前沿位姿建议，不启动 Nav2 或探索任务协调器。实际自主导航使用系统导航入口和 exploration_coordinator。

感知参数来自 `astribot_s1_perception_components/config`，探索参数来自本包 `config`；RViz 配置位于本包 `rviz/autonomy_debug.rviz`。旧包名不再提供兼容命令；C++ 命名空间、头文件路径和组件类 ID 保持不变。[完整迁移与验证说明](../../../docs/AUTONOMY_PACKAGE_RETIREMENT_20260917.md)。

## 探索结束自动存图

`~/cancel` 现在表示结束探索并保存已有建图成果；`~/pause` 仍然只是暂停。
协调器确认完成或取消后，等待自己的导航 Action 终态，再由本包 C++
`mapping_session_node` 等待停稳、触发 Voxel finish、检查文件并提交 manifest。
结果以 `/mapping_session/status` 的 `SAVED` 为准；请求受理不等于保存成功。
进入收尾后不能 resume，新的探索必须使用新的 SLAM 会话。

启动、失败重试、默认目录及验证边界见
[探索与 SLAM 会话收尾](../../../docs/EXPLORATION_SLAM_FINALIZATION_20260919.md)。
