# P3：机械臂规划预览接入（2026-09-19）

## 当前交付与边界

在现有 C++ operator_backend 内增加 ArmPreview 组件，工作站增加“机械臂预览”页签。没有新增 Python 运行节点或采集脚本，也没有建立第二个机械臂执行控制器。

复用 `/transport/plan_skill` 的 `named` 规划请求，目前支持 `arm_left` / `arm_right` 的 `transport_compact`。这只是轨迹规划，不代表机械臂已回到收起位。底盘、手臂、夹爪均不接收本组件的运动命令。

现有 `astribot_s1_transport/ros_backend.py` 的执行前置条件包含资源锁、导航 hold、包络、控制器、场景与载荷台账；其 admission 还依赖 Gazebo 位姿服务和指定世界原点。因此不能把原生 `/execute_trajectory` 直接接到上位机按钮，绕过这套事务。

## 操作与状态

所有请求走 `/operator_backend/command`，沿用 schema 1、boot_id、command_id、租约、查询与幂等语义：

| operation | payload | 行为 |
|---|---|---|
| arm_plan | group、named_target | 异步请求既有规划器，成功后查询返回 SUCCEEDED；状态中显示 READY |
| arm_discard | plan_id | 只丢弃匹配计划；PLANNING 中请求则忽略其迟到响应 |
| arm_execute | 任意 | 明确拒绝 ARM.EXECUTION_ADAPTER_UNAVAILABLE |

状态：EMPTY → PLANNING → READY / FAILED；控制权丢失、主动丢弃、关节或地图上下文变化、超时 → INVALIDATED。服务本身不支持取消，丢弃不会中止下游规划计算，也不表示任何运动已停止。计划仅用于预览，不能执行。

关节反馈要求单一发布者、完整有限的位置值、唯一关节名、ROS 时间戳与接收时间均在 2 秒内。规划返回要求起点与采集状态一致，时间严格递增，位置/速度/加速度/力矩数组维度合法且有限，最多 10000 点，时长最多 300 秒；不支持 multi-DOF 轨迹。所有已采集关节变化均使计划失效，当前采用保守策略。

关键参数由节点声明并进入 recorder 的参数白名单：

| 参数 | 默认 | 允许范围 |
|---|---:|---|
| arm_preview_ttl_sec | 30 | (0,120] 秒 |
| arm_planning_timeout_sec | 20 | (0,20] 秒 |
| arm_start_tolerance_rad | 0.025 | (0,0.025] rad |

参数在启动时读取并设为只读，状态中发布实际使用值；运行期间参数修改会被拒绝。调整后需重启后端并重新申请租约、规划。

## 显示、反馈与记录

- `/operator/arm_preview`：transient-local `moveit_msgs/DisplayTrajectory`；失效时发送空轨迹。可配合 MoveIt RViz 显示插件，需部署端安装与配置。当前没有宣称完成真实机器人模型的 3D 渲染验收。
- 工作站显示计划状态、失效原因、计划 ID、实际参数；执行按钮禁用。
- `/transport/status` 作为独立观测反馈进入总览：VALID / STALE / CONFLICT，不把已有仿真 object_state 当成真机夹持证据。载荷可信度保持 UNKNOWN。
- recorder 增加轨迹与运输状态采集，关节状态沿用已有采集项；命令请求记录 group、named_target、plan_id。沿用现有 ROS/spdlog 日志链路，不新建日志系统。
- 当前只读回放的发布白名单未扩展到 MoveIt 轨迹。bag 保存轨迹原文，工作站状态/事件可供诊断；后续补齐专用机械臂回放显示，禁止向执行 action 回放。

## 验证与后续

验证使用 isolated install `/tmp/astribot-route-build/install`。新增单进程 ROS 假规划服务测试运行于 localhost domain 214；覆盖合法预览、NaN 轨迹、关节偏移、地图切换、取消后的迟到响应、租约失效、计划过期，并检查未发布 cmd_vel。网关测试检查 arm_execute 明确拒绝，Qt 测试检查租约下的规划请求与禁用执行按钮。测试不会启动 Gazebo、MoveIt 执行器或真机驱动。

后续 P3 尚需：统一资源事务的 C++ 执行适配器、规划场景/载荷/标定版本绑定、控制器 readiness 与底盘 hold 确认、单次执行消费与取消终态屏障、真实夹持反馈、MTC 分阶段计划与失败恢复，以及真实模型 3D 显示和隔离仿真执行验收。完成这些之后才能开放执行按钮；本次不是完整 P3 验收。

### 本轮验证产物

构建日志：`/tmp/astribot-p3-build.log`；测试日志：`/tmp/astribot-p3-test.log`。源码已更新；当前运行中的工作空间 install 与共享 ROS 栈未替换。部署需在计划停机窗口重新构建 `astribot_transport_msgs`、`astribot_operator_backend`、`astribot_operator_station` 及其依赖，并确保规划器与后端使用相同的时钟配置。

本轮最终构建通过；backend 3 项、station 13 项，共 16 项 gtest 通过，0 失败。`git diff --check` 通过。验证覆盖假后端和离屏 Qt 交互，不包含真实 MoveIt 模型渲染、机械臂执行或夹持物理验收。

后续执行事务核心与多场景异常验收见 [P0—P5 异常验证矩阵](P0_P5_EXCEPTION_VALIDATION_MATRIX_20260919.md)。核心已通过纯 C++ 故障测试，尚未接入真实执行 action。
