# 虚拟墙 / 禁区与 SLAM 地图集成验证

日期：2026-09-21。范围：本次 navigation_zones 功能及关联 C++ 模块。使用 /tmp/astribot_zones_build 与 /tmp/astribot_zones_install，不覆盖共享安装目录或重启现有仿真。

## 结果

8 个相关 C++ 功能包构建成功，另构建了公共导航启动/配置包。8 包共 35 个 CTest 项通过，0 失败；其中可用 GTest XML 记录 97 个子用例，其他原生/差分测试按 CTest 项统计，不重复相加。详见 results.json 与各包日志。

| 证据 | 覆盖内容 | 不能推出 |
|---|---|---|
| zone_geometry / zone_state | 薄墙、凹多边形、自交/退化拒绝、扫掠、失效与旧消息拒绝、锁/幂等/版本、迁移清单继承 | 物理停止距离 |
| zone_ros | 真实 LayeredCostmap 更新、清图保持、删除后恢复、odom→map 平移、缺 TF/过期阻塞；假地图/里程计/消费者的保存与生效屏障；SAVED 会话不可编辑 | Gazebo 行驶轨迹或真机实时性 |
| exploration_end | ROS 协调器接收地图、里程计、TF 与约束，无前沿时报告 BOUNDARY_LIMITED，不触发整图完成 | 动态环境探索覆盖率 |
| mapping_session | 假 SLAM 的停稳、finish、最终事件与真实文件写入；约束缺失时等待，快照写入 manifest | Voxel 回环后的墙位置物理正确 |
| loop_route / backend | 有约束时准入；旧 token 拒绝、当前导航取消；路线 DWELL 阶段版本变化后不发下一点 | 所有外部调用方及整机任务全部通过验收 |
| operator_station | Qt offscreen 墙/矩形绘制、版本冲突、重载、地图切换、观察者禁写及既有页面回归；实际页面截图 zone_page.png | 实际 Ogre 视口鼠标交互或现场屏幕布局验收 |
| native policy / task arbiter | 修改后编译；既有数学、策略差分及所有权回归 | 新保护接线的整车闭环验证 |

公共 RPP / MPPI 配置均检查 zone_layer 位于 obstacle_layer 后、inflation_layer 前；Python launch / XML / YAML 通过解析。回放仅新增 Marker 可视化白名单，不重放在线约束输入。

## 审查修复

1. 接受新路线前只暂存 token；拒绝竞争请求时不得覆盖活动路线的 token。
2. 建图结束后冻结禁区编辑，避免后续编辑被存图清单里的旧快照覆盖。修改已存图的区域需先加载地图。
3. 单独的 SLAM 存图验证 launch 没有导航或区域服务，显式关闭区域依赖，仅用于原有存图测试；它不构成本功能验收。生产公共启动与 owned_mapping 仍要求区域服务。

## 待完成的现场验收

- 独立 Gazebo + RViz：实际点击绘制、绕墙导航、禁止进入区域、区域隔断探索、保存并重新加载、共享地图切换业务场景。
- 实际轨迹与完整机器人包络到边界的最小距离、约束更新和停止时延；最大地图与 64 区域的负载测试。
- 启用 final_protection 的整条速度链失联/重定位场景，之后才安排真机低速验证。

未向真机发送速度，也未把离线/假后端结果描述为真机或 Gazebo 闭环通过。当前共享运行栈不会因为源码变更自动加载新插件。
