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

## “开始新建图会话”的两种入口

### 已经有图或已有受管建图进程

“有图”不能直接覆盖当前地图文件。面板点击“开始新建图会话”后，实际调用的是
`/operator_backend/command` 的 `new_mapping_session`，后端再调用
`/mapping_runtime/start`。只有以下条件满足时才会接受：

这里要区分两种“有图”状态：

- **当前受管快速建图会话已有结果**：`mapping_runtime` 自己的子进程仍在运行，且
  `/mapping_session/status` 已报告 `SAVED`。这就是“重新开始快速建图”的直接入口；运行时
  只向自己拥有的进程组发送 `SIGINT`，确认旧图进程和 `/map` 发布者退出后，再生成新的唯一
  `map_<时间>_<PID>` 会话目录。
- **地图目录中已有活动定位图**：`map_manager` 的 `active_map` 已提交，当前系统在用它做
  定位。此时按钮不会绕过地图事务直接覆盖它；必须先停止导航/路线/探索并完成受管地图切换，
  再由部署流程显式切到 mapping 模式。外部 Voxel-SLAM 或静态 `map_server` 由别的进程拥有
  时，系统拒绝抢占，要求通过原拥有者完成停止。

1. 已申请控制权，里程计连续停稳，导航/路线/探索没有未确认目标。
2. 当前运行时由 `mapping_runtime` 自己拥有；外部 SLAM、静态 `map_server` 或其他探索节点不会被强杀。
3. 如果当前受管会话仍在运行，必须已经进入可重启条件；旧会话先收到 `SIGINT`，等待旧进程组和 `/map` 发布者退出。
4. 旧会话的地图保存状态为 `SAVED`，然后创建新的唯一 `map_<时间>_<PID>` 目录，使用 `save_map=1` 启动 Voxel-SLAM，并以暂停状态等待资源就绪。

因此“重新开始快速建图”是一次新的 SLAM 会话，不会在旧地图目录中追加，也不会把旧图当作新图的初始地图。旧目录仍保留，可先在“地图与工位”页导入归档。

### 仅启动 RViz、后端尚未运行

面板会把按钮切换为“启动默认仿真并开始建图”。确认后由 C++ 面板通过继承的 ROS 环境启动：

```text
ros2 launch astribot_s1_navigation nav2_full_bringup.launch.py \
  env:=sim mode:=mapping slam_backend:=voxel launch_gazebo:=true \
  launch_slam:=false \
  use_rviz:=false exploration:=false \
  operator_runtime_params_file:=.../astribot_operator_backend/config/mapping_runtime_sim.yaml
```

`launch_slam:=false` 是关键：Gazebo、传感器和 Nav2 先启动，但不启动一个无主的 Voxel-SLAM；随后由
`mapping_runtime` 独占启动带 `save_map=1` 的快速建图会话。这样“重新开始”可以安全停止并重建自己拥有的旧会话。
仿真使用当前 RViz 的 `ROS_DOMAIN_ID`，强制 `ROS_LOCALHOST_ONLY=1`，默认世界为
`small_warehouse`，不会连接真机，也不会再打开第二个 RViz。启动完成后面板等待
`operator_backend`，自动申请本地控制权，再提交 `new_mapping_session`。如果发现已有后端、多个状态发布者或控制权被其他客户端持有，则不会自举或抢占。

标准仿真启动入口现在会根据 `env` 自动选择 `mapping_runtime_sim.yaml`；硬件入口仍使用空 profile，必须显式提供标定参数，不会因为打开 RViz 而启动真机建图。

## 地图加载的完整实现

当前生产链路加载的是已经完成的 Voxel-SLAM 会话，而不是直接让 RViz 读取一张 PGM：

1. **选择会话目录**：目录必须位于 `map_manager.import_root` 下，并包含 `manifest.json`、地图 YAML/PGM、关键帧和轨迹等文件。
2. **导入校验**：`map_import` 校验 manifest 版本、`backend=voxel_slam`、`world_frame=map`、完成结果（`COMPLETED` 或 `CANCELED_PARTIAL`）、每个文件 SHA-256、地图 YAML 和 PGM 几何信息，然后复制到不可变 catalog 目录。
3. **生成地图版本**：版本取 manifest 的哈希；同一 `map_id` 不允许被覆盖。面板地图列表显示 `map_id + floor + version`。
4. **开始切换事务**：选择版本后提交 `map_switch_begin`。后端检查控制租约、停稳证据、导航/路线/探索均为空闲，以及地图版本和 catalog revision 没有变化。
5. **人工换层**：跨楼层或勾选人工转运时，事务进入 `WAIT_TRANSFER`；人工完成搬运并确认后才进入 `LOAD_INTENT`。同楼层可以直接进入加载阶段。
6. **适配器加载**：`map_manager` 将事务交给 `/map_session_adapter/command` 的 `load_session`。适配器负责调用现有 Voxel-SLAM 定位插件，并回报 `map_id`、版本、TF、地图、全局/局部 costmap 是否就绪。
7. **连续验证并提交**：适配器状态连续满足 `READY`、`localization_ready`、`tf_ready`、`map_ready`、两个 costmap ready 后，catalog 才转为 `COMMITTED`，并更新 `active_map`。此时面板才重新开放导航和路线。

加载失败会进入 `RECOVERY_REQUIRED`，保留旧活动地图和事务现场；人工核对后使用“人工核对后重新加载并验证”，不会自动重复加载或盲目恢复运动。

`map_server` 的 `yaml_filename` 直读方式只保留给仿真静态地图基线（`slam_backend=static_map`）。它适合验证 Nav2 的静态地图显示，不替代上面的 Voxel 会话归档、定位插件和 costmap 验证流程。
