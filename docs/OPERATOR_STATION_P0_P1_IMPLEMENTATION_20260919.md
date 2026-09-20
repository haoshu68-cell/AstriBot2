# P0/P1 首批 C++ 功能包交付

新增包：`ws_robot/src/astribot_operator_station`。本次没有删除旧采集脚本，没有改动 SLAM 或现有驱动。

## 已落地

| 组件 | 技术 | 能力 |
| --- | --- | --- |
| OperatorPanel | C++17 / Qt5 / rviz_common / pluginlib | 目标坐标输入、确认下发、取消自己的导航目标、探索暂停/恢复、录制启停/标记/快照 |
| diagnostics_recorder | C++ / rclcpp / rosbag2_cpp / spdlog | 显式话题白名单、SQLite bag 分段、事件记录、参数变更订阅、10 s 回读、低磁盘退出 |
| evidence / inspect_incident | C++ / JSON | 接收序号参数历史重建、缺口及删除处理、非有限浮点和整数类型保真 |
| 离线回看窗口 | C++ / Qt | 异步打开文件、事件选择、前后步进、每 0.5 s 顺序播放、暂停、参数详情，无 ROS 发布 |
| 启动与配置 | XML / YAML | 工作站默认仅 RViz，可显式启用同机采集器，无自定义 Python 启动/采集脚本 |

初始参数清单包含 50 项，覆盖控制器与检查器类型、到点容差、接近阶段限速、MPPI 速度/加速度和预测参数、探索失败预算、代价地图/包络相关配置。参数缺失不使用默认值替代。对实际生效只能确认 observed，未实现设备/算法 effective revision。

控制接口：`/navigate_to_pose` Action；探索节点 pause/resume Trigger；`/diagnostics_recorder/{start,stop,mark,snapshot}` Trigger。关闭面板不自动取消全部任务；取消只作用于本面板 handle。服务/Action 均异步，GUI 不阻塞等待 ROS。

## 验证与边界

在 `/tmp/astribot-operator-build` 隔离构建，未覆盖共享 ws_robot/install。C++ 单元/集成测试覆盖：参数时序/删除、缺口不沿用旧值、坏事件拒绝、真实 rosbag 写入/读取、录制重复启动及停止/重启、缺失节点和参数、参数变更、非有限浮点/大整数、Qt 按钮向假 Action 服务提交/取消、探索暂停与终态显示。

测试使用 localhost 独立域，并将导航/探索控制名称重映射到测试进程专属路径，不向机器人发送速度或生产导航目标。Qt 使用 offscreen，并生成面板截图检查。XML/YAML 可解析；launch --show-args 通过。

## 启动方式

本轮已构建版本可用于开发检查：

```bash
source /opt/ros/humble/setup.bash
source /tmp/astribot-operator-build/install/setup.bash
ros2 launch astribot_operator_station operator.launch.xml
```

该命令仅启动工作站 RViz。不要同时在多个采集主机启动同名 recorder。正式部署依包 README 修改持久化目录、robot_id、deployment_revision、域和参数/话题白名单，并通过正常构建部署流程安装。

## 尚未完成的验收项

这不是全部 P1 或完整上位机交付。当前没有生产身份鉴权/控制租约、地图事务、通用搬运启动、真机抓放、参数写入、3D bag 时间同步回放、自动故障前后窗口、设备 boot UUID/实际生效 revision、完整故障包哈希导出或 Orin 资源预算实测。UI 明确标识未接入，防误操作勾选不冒充权限系统。

下一步顺序：接入 3D 只读回放与缓存重建、资源验收 → P2 自动故障窗口/地图事务 → P3 抓放控制适配 → P4 搬运后端。原脚本仅在功能覆盖及验收等价后退役，不以重写语言为由一次性替换现行设备链。

## P1 增量：参数身份与数据质量

- 事件/manifest schema v2：以参数事件发布端 DDS GID 跟踪实例；重启或同名冲突清除旧值。设备 boot UUID 仍未知，服务读值仅为 observed。
- 请求按会话、请求令牌和参数事件 revision 校验；超时、采样期间变更、录制中止均记录缺口。回读发现未观察到的变化时，将上次已知证据之后的不确定历史区间标 unknown。
- 每秒话题接收质量：发布者数、消息/字节数、数据年龄、最大接收间隔；区分无发布者、无数据、过期和静态数据已收到。丢包数保持未知。
- C++ 测试新增实例切换和历史缺口重建；集成场景覆盖同名节点重启/重复实例、无事件参数变化、服务超时、话题过期及发布者消失。共 8 个 gtest 用例，分属 3 个测试程序。未运行真机动作或共享仿真。
