# astribot_s1_autonomy：兼容入口

实现已拆分为 `astribot_autonomy_core`（纯算法）、`astribot_s1_perception_components`（感知适配）和 `astribot_s1_exploration`（探索任务）。

详见 [架构、关注点、决策与质量验收](../../../docs/AUTONOMY_ARCHITECTURE_INTEGRATION.md)。

旧 launch、4 个 ros2 run 入口、配置路径和组件类 ID 保留转发。源文件和默认配置只有一个归属；旧 C++ 二进制依赖需重新构建。新项目应直接依赖对应包，不再链接原单体组件库。

探索任务只走 NavigateToPose → 导航策略行为树。旧的直接 FollowPath、自举旋转、脱困速度配置会被拒绝；没有有效地图时等待，不自行移动。

拆分前的调参记录移至 [历史参考](../../../docs/legacy/AUTONOMY_BEFORE_SPLIT.md)，其中旧执行模式不再可用。
