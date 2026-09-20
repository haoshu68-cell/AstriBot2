# P0/P1 增量：探索场景交互与版本化命令

本次落实探索这一条控制链的 P0 契约、假后端与 P1 场景交互；不是全系统 P0/P1 验收完成。保留现有探索算法与 SLAM 实现/注释，不新增 Python 运行节点。

## 已实现接口

- `/exploration_coordinator_node/operator_status`：schema_version=1 JSON，可靠 transient-local，带 boot_id、revision、原始状态、reason_code、readiness_detail、progress、transition_reason、导航未终结标记及可用操作。
- `/exploration_coordinator_node/command`：`astribot_operator_msgs/srv/ExplorationCommand`，输入 command_id、expected_boot_id、expected_revision、operation（pause/resume/cancel_save）。后端在状态锁内检查能力并执行，响应仅表示受理或拒绝，不表示停稳、存图成功。
- `/exploration_coordinator_node/command_events`：记录请求关联 ID、预期/返回版本、操作与响应。同步写 ROS 日志，沿用已有 spdlog 启动采集链，并加入诊断 bag 白名单；不新增独立日志脚本。
- `/mapping_session/status` 与 `/mapping_session/retry`：沿用既有接口；新状态页展示保存阶段、失败原因、实际目录和保存类型。

状态 revision 对操作前提变化递增，不因普通心跳刷新递增。已受理命令在本进程内去重，最多保留 4096 个，满后拒绝新版本化命令；同 ID 不同操作/预期版本拒绝。记录不跨重启持久化；旧 boot 请求拒绝。重复受理响应保留原受理事实，当前执行状态以最新状态 Topic 为准。

固定响应码：COMMAND.ACCEPTED、REQUEST.BOOT_MISMATCH、REQUEST.INVALID_ID、REQUEST.CONFLICT、REQUEST.CAPACITY、STATE.REVISION_MISMATCH、REQUEST.UNSUPPORTED、EXPLORATION.OPERATION_BLOCKED。

兼容 Trigger pause/resume/cancel 仍保留，复用同一内部操作函数；旧 resume 保持旧行为。新的版本化 resume 要求暂停、导航无未终结目标、地图/定位/里程计/校验图就绪、已知区域可用于决策。不应把新接口的防误操作机制宣称为全系统鉴权或独占控制租约。

## 场景矩阵

| 场景 | 显示与可执行操作 |
| --- | --- |
| 未连接、状态超过 3 s 未更新、多发布者冲突 | 禁止新探索操作，显示状态不可用；不推断任务已停止 |
| 地图未收到、地图过期、定位/里程计/校验图不足 | 显示具体就绪原因；不允许恢复；可暂停、可按后端能力结束保存 |
| 地图已知区域不足 | 显示等待地图，不显示建图完成 |
| 正常探索 | 可暂停、结束并保存；不提供重复恢复入口 |
| 人工暂停或导航被接管 | 展示原始转换原因；就绪且无未终结导航时可确认恢复 |
| 取消导航尚无终态 | 显示等待终态，禁止恢复；受理不代表物理停稳 |
| 有边界但不可达 | 明确不可达，不误判完成 |
| 候选点冷却、失败预算暂停 | 展示冷却/预算处理提示；人工恢复需确认重置失败预算 |
| 无候选且正在确认完成条件 | 显示“正在确认”，由后端判断完成，UI 不决定完成 |
| 当前地图探索完成 | 保留 current_map 完成范围，等待收尾；不是全部楼层覆盖完成 |
| 主动取消探索 | 确认本会话不能恢复 → 等待导航终态与存图；保存标 CANCELED_PARTIAL |
| 等待停稳、读取配置、SLAM 收尾或文件校验 | 展示地图收尾阶段，不开放恢复或重复保存 |
| 保存失败 | 展示原因；仅在 retry 服务可用时开放“重试地图保存” |
| 保存成功 | 展示目录与 COMPLETED/CANCELED_PARTIAL；不把部分保存标为探索完成 |
| 请求响应超时 | 结果未知；只允许同 command_id 重试确认，不生成新的动作请求 |
| 确认框打开期间状态改变、过期或重启 | 本次确认无效，重新核对；不自动补发 |
| 新建图 | 明确暂未接入新 SLAM 会话启动，不能以 resume 冒充新会话 |

暂停仍可在地图保存状态不可用时请求冻结探索；恢复和结束保存要求新鲜的 IDLE 地图会话状态。保存重试超时后等新地图状态再操作。UI 对同名状态多发布者阻止提交，但目前地图状态没有 boot/session ID，不能据此声称跨地图实例绑定完成。

## C++ 假后端

新增 `fake_exploration_backend` 和 `operator_fake.launch.xml`。固定提供 `/operator_fake/*` 端点；示例启动将 Panel 控制入口重映射至假端点，导航、循环路线和记录控制均重映射至未实现的假入口。不会调用机器人或写入真实地图。

安装隔离构建后，可在独立开发域运行（不使用仓库仿真 launch）：

```bash
source /opt/ros/humble/setup.bash
source /tmp/astribot-route-build/install/setup.bash
ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1 ros2 launch astribot_operator_station operator_fake.launch.xml scenario:=waiting_map
```

支持场景 waiting_map、running、paused、cancel_pending、unreachable、completing、saving、save_failed、saved、disconnected。可以在相同测试域修改假节点的 scenario 参数；这是测试注入入口，不部署到真机运行 launch。正常工作站 launch 未默认启动假后端。

## 验证与后续边界

测试新增真实探索节点的 boot 校验、命令幂等、冲突/旧版本拒绝、未就绪恢复拒绝；保留暂停不存图、取消仅收尾一次及结束后不能恢复测试。Qt 假服务测试覆盖实际按钮调用与不同场景门控。构建目录 `/tmp/astribot-route-build`；测试使用专属 localhost 域。未覆盖共享安装目录、未驱动共享仿真或真机。

还需继续完成的 P0/P1 范围：全系统控制租约和身份、导航/录制等接口统一迁移、新建 SLAM 会话事务、总览聚合、原始时间同步的只读 3D 回放、资源预算和完整 Nav2/Gazebo 行为验收。既有离线事件/关键参数回看保持原功能，不宣称本次已交付 3D 回放。

本轮完成的测试：operator_station 9 个 gtest 用例、exploration 6 个 gtest 用例通过。新增场景在这些集成用例中覆盖；确认框期间后端重启和非法状态字段的 Qt 回归通过。测试没有验证真实导航运动或覆盖率。
