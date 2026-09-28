# 不计为通过的尝试

- `controller_stop_fixture_invalid.xml`：初始化后设置框架只读参数，夹具启动失败；改为初始化参数后重跑。
- `initial_regression/`：定站导航测试遗漏专用 domain 确认变量，18 个导航前置失败不代表功能失败；后续完整回归带变量重跑通过。
- `protocol_normal_admission_failed.*`：验证夹具只等待约 0.4 秒，未覆盖当前源代码要求的 0.6 秒 SLAM 停稳窗口；增加验证侧等待并保留实际停稳准入，正常协议随后通过。
- `baseline_invalid_overlay/`：基线动态库被当前 overlay 覆盖；修正库搜索顺序后独立复现原始 12 项失败。
- `sweep_initial_compile_failure.log`：调用了本机 MoveIt 不存在的碰撞后端名称接口。修正为已安装头文件中的接口后重建。该次失败后误运行的旧测试可执行文件不计为新扫掠功能验证。

以上没有通过放宽运行时条件或删除失败断言解决。

- `full_pick_normal.*`、`full_place_normal.*`：旧夹具没有模拟真实 MoveIt 附着时移除同名世界物体，也未返回现行服务要求的 `remaining_stages`。参照已通过的真实 PlanningScene 单测和服务定义修正验证夹具；`*_normal_final.json` 的全流程断言通过，生产负载代码未改。
- `lease_readiness_before.*`：新增边界测试复现了尚无控制周期确认就接受新租约目标的行为；修复后的 13 项插件测试及 100 次故障/并发重复全部通过。
# 点云更新器阶段补充

- `observed_octomap/observation_initial_build.log`：直接链接感知包的全部导出库时遇到未定义 `GLUT::GLUT` 导入目标。实际仅需现有 ShapeMask，改为链接该库的公开导出目标；没有添加无关图形库依赖或替代运行路径。
- `observed_octomap/observation_teardown_failure.*`：首轮 10 项测试中 9 项在夹具收尾时失败。销毁订阅回调组后才从 executor 移除节点，触发 ROS executor 的组归属异常。修正为先移除仍存活的节点，再销毁订阅和更新器；完整重跑 11/11 通过。原失败未计作通过。
- 普通包回归中的点云性能用例为显式跳过；设置 `ASTRIBOT_OCTOMAP_BENCHMARK=1` 的独立结果才作为性能实验与该项通过证据。

## 相机原点与隔离仿真

- `camera_origin_fix/observation_origin_before.*`：现场实际 RGB-D 点云的 frame 为底盘坐标系。以点云 frame 原点发射射线会制造错误的自由空间，并错误计算量程。新增测试在修复前失败；修复后必须有源时刻相机 optical frame 的 TF，无 TF 不退回点云 frame 原点，13/13 通过。既有上游对照限定在点云系与相机系相同的输入，不能外推为原错误地图保持不变。
- 第一轮自有 `session01` 的启动曾选 `navigation-policy off`，与现有 arm-speed coupling 的前置条件不匹配。保留启动失败日志，使用现有 p2 配置重新启动，没有关闭安全条件。该会话只读，未下发机械臂目标。
- `session01` 暴露相机原点问题后已安全关闭，再使用修复版本创建新 `session02`。域和安装均独立于主线，未在正在运行的进程下替换动态库。
