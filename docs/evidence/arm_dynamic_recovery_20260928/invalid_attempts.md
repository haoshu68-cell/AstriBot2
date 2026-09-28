# 不计为通过的尝试

- `controller_stop_fixture_invalid.xml`：初始化后设置框架只读参数，夹具启动失败；改为初始化参数后重跑。
- `initial_regression/`：定站导航测试遗漏专用 domain 确认变量，18 个导航前置失败不代表功能失败；后续完整回归带变量重跑通过。
- `protocol_normal_admission_failed.*`：验证夹具只等待约 0.4 秒，未覆盖当前源代码要求的 0.6 秒 SLAM 停稳窗口；增加验证侧等待并保留实际停稳准入，正常协议随后通过。
- `baseline_invalid_overlay/`：基线动态库被当前 overlay 覆盖；修正库搜索顺序后独立复现原始 12 项失败。
- `sweep_initial_compile_failure.log`：调用了本机 MoveIt 不存在的碰撞后端名称接口。修正为已安装头文件中的接口后重建。该次失败后误运行的旧测试可执行文件不计为新扫掠功能验证。

以上没有通过放宽运行时条件或删除失败断言解决。

- `full_pick_normal.*`、`full_place_normal.*`：旧夹具没有模拟真实 MoveIt 附着时移除同名世界物体，也未返回现行服务要求的 `remaining_stages`。参照已通过的真实 PlanningScene 单测和服务定义修正验证夹具；`*_normal_final.json` 的全流程断言通过，生产负载代码未改。
- `lease_readiness_before.*`：新增边界测试复现了尚无控制周期确认就接受新租约目标的行为；修复后的 13 项插件测试及 100 次故障/并发重复全部通过。
