# 地图选择、加载与现有插件联动

## 已实现

- RViz 操作台“地图与工位”页新增“选择目录”，通过 Qt 文件选择器选择机器人上的 SLAM 会话目录。
- 既有 `map_import` 负责校验会话 manifest、哈希和地图栅格，并归档到不可变 catalog；地图版本下拉框显示归档版本。
- 既有 `map_switch_begin`、人工换层确认/放弃/恢复事务接入 `map_manager`，事务提交后由 `voxel_session_adapter` 调用现有 Voxel-SLAM 定位插件并验证 TF、地图和 Nav2 costmap。
- 隔离仿真默认生成实例专属 `voxel_session_adapter.yaml`，启用 `profile=sim`、受管资产目录和运行目录，避免多个仿真实例互相覆盖。
- 地图页状态文字明确显示“选择会话→校验归档→选择版本→准备切换→人工确认（如需）→适配器加载验证”的流程；适配器未就绪时导航按钮继续禁用。

## 验证

- RViz 操作台 CTest：7/7 通过，包含地图页事务按钮与状态门控测试。
- 地图管理器 CTest：4/4 通过，包含 catalog 导入、事务恢复、Voxel 适配器和无速度约束测试。
- `navigation.launch.py` 通过 Python 语法检查；`sim_stack_supervisor.py --dry-run` 已确认实例导航命令携带 `enable_voxel_adapter:=true` 与实例专属参数文件。
- 现有仿真域 213 已观察到 `/map_session_adapter/status`；本次未执行真实切图，因为当前运行中的 Voxel-SLAM 图由其他进程持有，适配器会按设计拒绝外部 SLAM 接管，避免覆盖正在运行的定位会话。

## 使用顺序

1. 启动带实例的仿真，上位机打开“地图与工位”。
2. 点击“选择目录”，选择已完成并具有 `manifest.json` 的 SLAM 会话。
3. 点击“校验并归档已保存地图”，等待地图版本出现在列表。
4. 选择地图版本并点击“准备切换所选地图 / 进入人工转运等待”。同楼层直接进入受管加载；跨楼层勾选人工换层并在停稳后确认。
5. 等待“活动地图已验证”、适配器 `READY`、TF/costmap 就绪后再导航。

该流程不会向底盘直接发布速度；地图切换事务要求停稳、控制租约和插件证据全部满足。
