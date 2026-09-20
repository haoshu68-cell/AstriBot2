# 社会导航场景验证

本目录的脚本用于本机 Gazebo，不用于真机。先按仓库 ros2-stack-ops 流程确认已有仿真的归属；每个用例从冷启动的初始位姿开始，不在另一个任务的仿真上直接发送目标。

完整规控与判据见 `docs/SOCIAL_NAVIGATION_SCENARIO_DESIGN.md`；阶段状态见 `docs/evidence/social_navigation_20260919/implementation_status.json`。有配置文件不代表该项已经通过。

统一原策略后的施工顺序和逐项回归要求见 [详细落地计划](../../docs/UNIFIED_NAVIGATION_IMPLEMENTATION_PLAN_20260919.md)及 [50 项验证清单](../../docs/plans/unified_navigation_rollout_20260919.json)。清单中的新增用例需要相应夹具和验证器扩展，不能直接当作当前 `run_episode.py` 的 case 输入。

## 单场景运行

先安装可选依赖并构建社会消息、观察器、Gazebo 插件、导航策略和路径跟踪包，source 对应 overlay。当前工作站已构建的 overlay 是 `/home/yjh/WorkSpace/astribot_validation/social_navigation_20260919/h1_overlay.bash`。

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
export ASTRIBOT_OVERLAY_SETUP=/home/yjh/WorkSpace/astribot_validation/social_navigation_20260919/h1_overlay.bash
bash tools/launch_sim_stack.sh --mode baseline --social-policy h2 \
  --social-scenario ws_robot/src/astribot_s1_gazebo_bringup/config/social/h2_cross_left.yaml \
  --log-dir /tmp/social_cross_left
```

另一个终端，启动器显示 READY 后：

```bash
bash /tmp/rosops/align_env.sh
source /tmp/rosops/query_env.sh
set +u
source /home/yjh/WorkSpace/astribot_validation/social_navigation_20260919/h1_overlay.bash
python3 tools/social_navigation/run_episode.py \
  --case tools/social_navigation/cases/h2_cross_left.json \
  --output /tmp/social_cross_left_episode
```

目录必须未存在。配置中的机器人目标为 `[2,0,0]` 及返回 `[0,0,0]`，格式为 `[x_m,y_m,yaw_rad]`，角度使用弧度；例如 90° 应填写 `1.5707963267948966`。脚本先检查行人名称、初始位置、暂停状态和独立几何，再启动既有导航测试器，按机器人位置放行行人。Ctrl+C 取消本次导航并恢复观测故障模式；停止整栈按启动器所属 session.json 的 supervisor PID 操作。

## 用例

| JSON 文件 | 场景 YAML | 检查重点 |
|---|---|---|
| h2_empty | h2_empty | 无人基线，至少 3 对 off/on |
| h2_cross_left | h2_cross_left | 左侧横穿、等待、恢复、原精度到点 |
| h2_cross_right | h2_cross_right | 右侧横穿，对称性 |
| h2_two_cross | h2_two_cross | 两人冲突并集，无抢行 |
| h2_stop_ahead | h2_stop_ahead | 行人走停，保持停止间距 |
| h2_head_on | h2_head_on | 对向斜交接近、提前减速/等待；不等同于非合作正面冲撞 |
| h2_input_loss | h2_empty | 丢包 3 s、旧帧重发 3 s、恢复；重复帧不续期 |
| h2_goal_occupied | h2_goal_occupied | 已规划后行人占据目标，等待后离开 |

无人 off 对照将启动命令中的 `--social-policy h2` 改为 `--social-policy off`，测试器增加 `--policy off`。其余模式、地图、初态、目标和参数保持一致。不要用纯静止“零碰撞”代替功能通过。

H2 显式加载 `h2_simulation.json`，其停车安全预算参考此前四档、四方向真机 SLAM 最大前冲数据。它不是当前真机制动认证；实际速度超出约 0.053–0.249 m/s 时，日志明确标为历史范围外。普通 off 入口保留原配置；两组跟踪器及到位运动参数一致。

历史停车数据及完整扫掠的无运动检查：

```bash
python3 tools/social_navigation/validate_stop_reference.py --output /tmp/social_stop_reference.json
```

三对无人场景均完成后比较（路径指向各自 `summary.json` 所在目录）：

```bash
python3 tools/social_navigation/compare_empty_baseline.py \
  --off /tmp/empty_off_1 /tmp/empty_off_2 /tmp/empty_off_3 \
  --on /tmp/empty_on_1 /tmp/empty_on_2 /tmp/empty_on_3 \
  --output /tmp/social_empty_comparison.json
```

比较器检查同路线/同初始航向/同场景内容、完整控制器参数及实际加载库、全部到位、无额外中途停车、事件式规划及 FOLLOW 横向/航向/实际 jerk 的逐目标中位数。同一 episode 重复传入不能计为三轮；缺少完整参数或指标只做历史诊断，不能放行。新的冷启动排程按轮交替 off/on 次序。总耗时不参与评分。

横向/航向比较按 pose 源时间每 50 ms 最多保留一帧，避免低仿真实时率下重复查询同一位姿改变 P95 权重；旧记录缺少 pose 时间时显式记录 odom 时间代理。实际加速度/jerk 继续基于去重后的 odom 源时间。重算结果单独保存，不改写历史采集文件。

## 结果判读

- `summary.json`：逐项布尔检查和 `scenario_passed`；`status=completed` 只代表采集结束。无冲突、行人未动、无恢复、返回失败都不算通过。
- `observations.jsonl`：社会候选、输入龄期、最终速度、真实运动、独立机身/人员几何、场景事件。
- `navigation/results.jsonl`：成功/失败分别保存；FOLLOW 横向/航向/实际加速度与 jerk，末端到位误差分开统计。
- 启动目录 `session.log` / `session.json`：统一日志、生命周期与进程归属。

验收还要看 Gazebo 与 RViz 成对截图、动态残留格与静态地图对照，以及无人 A/B 指标。脚本的单场景通过不能替代整阶段验收，也不能代表真机精度。

## 逐项回归启动器（I0.1）

`run_regression.py` 支持公共六类的冷启动排程；取消包含运动中、等待中两个变体。默认重复 3 次，无人场景每次分别跑 off/h2，全部完成后调用原 A/B 比较器。另有四方向四档速度的正常置零夹具和独立几何判据正反例；新夹具实现不等于仿真通过。施工清单中的跟随、队列、超越等后续场景尚未全部接入，不能靠公共回归输出推进整阶段。

与其他任务共用机器时，使用独立冻结运行目录和安装目录，并指定空闲的实例名与 ROS Domain。以下是本次隔离实例命令；不要同时重复启动同一实例：

```bash
python3 /home/yjh/WorkSpace/astribot_validation/unified_navigation_isolated_20260919/runtime/tools/social_navigation/run_regression.py \
  --overlay /home/yjh/WorkSpace/astribot_validation/unified_navigation_isolated_20260919/frozen_h2.bash \
  --instance unified_social_67 --ros-domain-id 67 \
  --output /home/yjh/WorkSpace/astribot_validation/unified_core_$(date +%Y%m%d_%H%M%S)
```

追加 `--dry-run` 只打印排程。`--cases r01_cancel_waiting --repeats 1` 可用于单个夹具调试，但一次调试通过不满足三次回归要求。输出目录必须未存在；失败重跑使用新目录，保留原结果。

启动前基础设施失败默认最多重试一次（`--startup-retries 0` 可关闭）。仅限尚未创建 episode、确认本次进程已结束的失败；运动中失败、测量失败、碰撞或清理失败不自动重试。失败尝试保存在独立目录和 `startup_attempts`，有效重复次数不包含启动失败。这是有界恢复，不能称为已修复 Gazebo 的偶发启动根因。

启动器反读自己创建的 controller 进程环境，核对 ROS Domain、Gazebo partition 和 discovery 端口，记录实际加载的库哈希、物理步进、生命周期和数据实测速率；查询使用每例的 `query_env.sh`。每例结束先向自己的 supervisor 发 SIGINT；若超时，仅按预先绑定的本次进程句柄结束残留，并将该轮标为基础设施失败。其他会话持锁时退出，不清理对方，也不删除共享内存。Ctrl+C 同样先取消自己的导航，再停止自己的仿真。

回归中的导航超时、稳定观察和场景动作使用仿真物理时间，同时保留独立墙钟看门狗；低实时率不能被误判为到位超时。原 `run_waypoint_route.py` 默认仍使用墙钟，新增 `--timeout-clock sim` 是此回归显式选择。总时间不参与跟踪性能评分。源帧时间必须递增且新鲜，收到重复内容/旧帧不能续期。

`--cases r03_x_pos_10 --repeats 1` 运行 x+、0.10 m/s 的置零试验：先检查空闲、静止、路径有效及输入归属，发送 1.6 s 有界速度，再观察 6 s 停车，位移限于起点 0.6 m 内。`r03_{x_pos,x_neg,y_pos,y_neg}_{06,10,20,35}` 覆盖四方向四档。它经过现有平滑链，分别记录原始零指令、最终零指令、实际速度、峰值前冲及最终残差；不是机械急停验证，也不直接认证真机。

R03 v2 显式指定 `stop_boundary=final_output_zero`：历史先验只比较下游最终零指令后的尾程，正常平滑停车全程放在 `normal_stop` 中。最终 Twist 无源时间戳，事件时间是接收端 ROS 时钟，不能称为执行器确认时间。v1 将平滑器之前的零指令作为历史先验起点，边界不匹配；原 v1 失败记录保留，不能改写成通过。完整正常停车的预测仍由 I2 单独验收，不因尾程通过而算通过。

`--cases oracle_geometry --repeats 1` 仅用于隔离 Gazebo：先验证无重叠，再将仿真机器人移入已知行人轮廓，检查独立几何判据确实报告重叠。该用例的预期碰撞不能混入正常避障成功率，结束后必须冷启动新场景。

几何反例要求传送后至少 0.5 s，连续三帧机身已静止且确实重叠；不以传送造成的保守扫掠报警作为通过依据。

正常置零失败时用 `analyze_stop_chain.py --episodes /absolute/episode --output /new/directory` 对齐原始零速、下游指令及 odom 源时间，输出 JSON 和曲线。正常零速经过平滑器，最终防护的零速在平滑器之后；两条链路的停车结果必须分别报告。该分析器不会更改原失败结果或放行阶段。

已完成的社会场景若记录了 odom 位置，可用 `analyze_protection_stops.py --episode /absolute/episode --profile /absolute/h2_simulation.json --output /new/report.json` 分析最终防护 HOLD 的实际停车。它要求递增源时间、连续位置、持续零指令和停稳窗口；没有有效的运动中 HOLD 返回 `INVALID_FIXTURE`，不能把静止无事发生算作制动通过。

行人走停和目标占用用例现在显式选择 `pause.stop_contract=source_hold_budget_v2`。开始前读取最终防护节点实际使用的 profile 并保存其依赖哈希；评估带源时间的 HOLD 后停车预算、0.5 s 窗口内至少 20 帧且覆盖至少 0.45 s 的连续停稳、人员恢复前持续零指令与有效 HOLD、完整几何净空和采样连续性。已经静止的等待可以通过等待检查，但不计作制动试验。旧“行人暂停后 1.5 s”检查保留为诊断，不覆盖旧失败报告。新契约的目标占用、行人走停各一轮闭环通过，另有两次回放和 32 个负例通过；重复次数仍未齐。目标占用记录到运动中制动，行人走停本轮在人员暂停前已停止，后者仅计作等待验证。原失败记录保留。

RViz 出现疑似残留格时，在所属实例的 `query_env.sh` 环境中运行 `capture_map_snapshot.py --output /new/directory --wall-seconds 20`。它只保存有限时长内各指定地图、扫描和点云的末帧、原始 CDR、源时间、TF 及叠加图，不录制全部话题；缺帧和 TF 错误会单独记录，不能据此宣称地图无异常。

每例保存 `stack/session.log`、`stack/session.json`、`episode/`、控制器 `runtime_manifest.json`、规划器 `planner_runtime_manifest.json`、感知组件 `perception_runtime_manifest.json` 和成对 `gazebo.png` / `rviz.png`。截图仅匹配本次 supervisor 的子进程窗口，并移动 Gazebo 相机以避开墙体遮挡；截图成功仍标记 `visual_review=PENDING`，必须查看后记录结论。汇总的 `episodes_passed` 仅表示所选用例及已有 A/B 检查通过，`stage_acceptance` 不自动标通过。

`run_episode.py` 的进程退出码现在与场景断言一致：失败退出非零。取消夹具另外检查同一任务的 `CANCELED`、停止后的实际运动和指令，以及取消后旧任务未重新激活。数据缺失与夹具未触发分别记录为 `INFRA_FAILURE` 和 `INVALID_FIXTURE`，不算到位失败样本。

`s01_yaw90` 验证 90° 终点朝向，`s01_l` / `s01_u` 使用一个 NavigateThroughPoses 任务覆盖连续拐弯。它们同样各跑 off/h2 三对。中间经过点的判据读取正在运行的 `/navigation_executor/bt_navigator` 所选 BT 的 `RemovePassedGoals` 半径，同时记录文件哈希和真实最近偏离；它不是最终到位精度。原 BT 半径为 0.7 m，原控制器在首次 L 试验中距拐点约 0.384 m，必须保留这个基线现象，不以最终 2 mm 到位冒充过弯精度。`--via-tolerance` 可显式要求更严格经过距离，但不能高于实际 BT 半径。启动航向 ±90° 的独立变体为 `s01_start_{left,right}_{straight,l,u}`；使用正常 Gazebo 初始生成角度，启动后以独立仿真位姿确认，不使用瞬移来绕过起始对齐。

对已有记录做无运动回放和故障注入：

```bash
python3 tools/social_navigation/replay_evidence.py \
  --episodes /absolute/path/to/episode \
  --output /absolute/path/to/replay_checks.json
```

当前施工证据及未完成项见 [实施状态](../../docs/evidence/unified_navigation_20260919/implementation_status.json)。

`r02_fixed_p4_13` / `r02_fixed_p5_13` 使用独立构建的 MoveIt/transport 支持，复用 `verify_fixed_hold_expiry.py` 的双臂保持、完整包络握手、入口导航、物理墙体和原通道行为树。社会控制保持 off，分别运行合法的 fixed_v2 + P4/P5。记录器不发布假的几何确认或保持 ACK；最终到位仍用 2 mm / 0.1° 验证。原脚本的三种规划正反例及 1.3 m 穿越、保持过期停车一并保留。

窄通道独立观察增加 `robot_swept_bounds`：从 Gazebo 实际全部碰撞体逐物理帧计算世界坐标包围盒，并计入相邻物理帧运动界，输出区间并集。它是保守下界，出现包围盒相交不能直接称为真实接触；缺帧或不支持的几何不算无碰撞。R02 仍需重复次数、P4/P5 及 legacy 边界与质量比较齐全才能验收。


`--collect-all` 可在每例已确认退出后继续采集同一批次的其他冷启动用例，用于查明问题范围。失败仍保留为失败，不能据后续通过结果开放下一阶段；清理未确认时立即停止。

`r02_cancel_moving` 在实际运动后中断通道验证器，要求任务取消、执行器结束确认、连续一秒实际停稳及最终零指令。验证器的 SIGINT/SIGTERM 先进入原取消与撤销事务，再关闭 ROS；不能把 ROS 上下文先销毁后的取消超时当作正常清理。

R02 的隔离回归显式选择 `navigation_sim_timeout_s=180`（与无人往返夹具一致，覆盖控制器原有 135 秒精调上限），并保留 600 秒墙钟看门狗；仅导航结果等待使用物理时间，服务响应及取消确认仍保留原墙钟上限。旧工具默认的 90 秒墙钟行为不变。改变测试计时不解决轮端低速响应问题，原失败证据不改写。

`analyze_corridor_entry.py --episodes /absolute/episode ... --output /new/directory` 对齐入口导航的 TF、odom 与最终速度，报告指令存在但航向响应很小的连续区间及曲线。历史未录最终速度时明确标记覆盖率 0，不补造指令。新的通道记录器另外以最多 20 Hz 保存轮速与轮端力矩，并在实际穿越时采集一组成对 Gazebo/RViz 图；它只测量，不修改驱动控制参数。

`r02_legacy_p4_13` / `r02_legacy_p5_13` 为独立的旧几何入口，使用原 legacy 配置和正常导航仲裁，不启动 fixed_v2 保持协议。机器人通过正常生成参数从 `(-1.4,0,0)` 出发，夹具创建自身的两面实体墙，检查代价图确实观测到墙体。1.3 m 用例通过通道内经过点导航至 `(1.4,0,0)`；1.8 m 对照 `r02_legacy_p{4,5}_18` 使用单目标。准入状态、通行许可、实际完整机身扫掠和出口后到位分别录制。针对单目标轻微弯曲和经过点重复端点，独立候选补齐精确对齐直线规划和零长度段处理；P4/P5 在 1.3 m、1.8 m 各有一轮通过，原失败记录保留，重复矩阵未齐。Fixed_v2 暴露附件/策略确认的时序问题，另建候选验证；禁止放宽准入阈值或互借通过结果。

`--first-repeat 2 --repeats 2` 可以将中断后缺少的第 2、3 轮写入新目录，继续保留对应的 A/B、B/A 顺序。旧记录不覆盖；最终比较仍要求三个不同 episode 对，不能用相同目录凑数。

只读汇总已经完成的批次结果：

```bash
python3 tools/social_navigation/report_regression.py \
  --batches /absolute/path/to/batch \
  --output /absolute/path/to/report.json
```

汇总保留失败、启动异常与取消的区别，按每个目标重算最多 20 Hz 的 FOLLOW 指标，实际 jerk 使用原始去重 odom；不合并 REFINE、不把时间列入评分。重复目录排除，运行中未落盘的用例不计完成。汇总本身始终不授予阶段通过。

通道记录器另保存实际运动约束、通道请求的源时间/租约、完整参考路径的哈希及原文、最终防护状态，避免将策略诊断评估时间当作指令发布时间。端点截图前用 20 秒有界只读采样保存 `post_fixture_snapshot/` 中的地图、局部/全局代价图及扫描；结合源龄期和被移除墙体的位置复查残留。截图或快照缺失必须在阶段审查中单独列出，不自动解释为地图已清空。
