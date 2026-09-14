# 真机底盘测试操作说明（parking_v3）

本说明按 2026-09-14 实际 SSH 核对的机器人路径编写。**除了第一步登录命令，其余所有命令均在机器人 SSH 终端执行，不在开发电脑本地执行。** 本次只更新说明，未启动运动测试。

## 1. 登录与路径确认

在开发电脑终端登录：

```bash
ssh astribot@10.249.22.137
```

登录后用户名应为 `astribot`，提示符通常包含 `astribot@orin`。在真机执行：

```bash
whoami
hostname
```

已确认的路径：

| 用途 | 真机绝对路径 |
|---|---|
| 厂家 SDK | `/home/astribot/Downloads/astribot_sdk_aarch64` |
| 当前测试工具包 | `/home/astribot/chassis_tests/toolkit_20260914_parking_v3` |
| 环境入口 | `/home/astribot/chassis_tests/toolkit_20260914_parking_v3/env_test.sh` |
| 主测试程序 | `/home/astribot/chassis_tests/toolkit_20260914_parking_v3/tools/robot/chassis_characterization.py` |
| 配置模板 | `/home/astribot/chassis_tests/toolkit_20260914_parking_v3/tools/robot/config/chassis_characterization.json` |
| 本操作说明 | `/home/astribot/chassis_tests/toolkit_20260914_parking_v3/docs/CHASSIS_AUTOMATED_CHARACTERIZATION.md` |

旧目录 `toolkit_20260914_130626`、`toolkit_20260914_feedback_v2` 保留作历史版本，本文不再使用。`parking_v3` 是工具包版本；其配置内 `feedback_schema_version=2` 是反馈协议版本，两者不矛盾。

以下建议新建 `configs` 和 `runs` 子目录存放配置与结果，不覆盖现有 `preview01`、`feedback_check02` 等记录。

## 2. 加载真机环境

每个新开的真机 SSH 终端都先执行：

```bash
cd /home/astribot/chassis_tests/toolkit_20260914_parking_v3
source /home/astribot/chassis_tests/toolkit_20260914_parking_v3/env_test.sh
```

**使用 `env_test.sh`，不要直接 source 工具包内部的 `tools/robot/env_robot.sh`。** 前者显式指定真实 SDK 目录，再加载后者，避免把独立工具包误认为 SDK 根目录。

`env.sh` 中两个可选 setup 路径缺失的提示曾出现过；离线预览成功不证明 SDK 已连接，应以下面的只读预检为准。此前实际进程使用 Domain 25、Fast DDS 配置 `/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml`。不要凭 SSH 地址修改 DDS 网卡白名单；如实际接收失败，先按仓库 ros2-stack-ops 流程从正在运行的厂家/定位进程核对 DDS 环境。

## 3. 建立自己的配置副本

在真机执行。命令在文件已存在时保留原文件，不覆盖你的编辑：

```bash
mkdir -p /home/astribot/chassis_tests/configs /home/astribot/chassis_tests/runs

if [ ! -e /home/astribot/chassis_tests/configs/x_positive_v3.json ]; then
  cp /home/astribot/chassis_tests/toolkit_20260914_parking_v3/tools/robot/config/chassis_characterization.json \
    /home/astribot/chassis_tests/configs/x_positive_v3.json
fi

nano /home/astribot/chassis_tests/configs/x_positive_v3.json
```

如没有 nano，使用 `vi` 编辑同一文件；nano 保存为 Ctrl-O、回车，退出为 Ctrl-X。

### 需要填写的内容

| 字段 | 含义与填写要求 |
|---|---|
| `conditions.robot_serial` | 现场实际设备编号 |
| `conditions.posture` | 双臂、夹爪和躯干的实际运输姿态 |
| `conditions.payload` | 本次载荷质量/形状，空载也明确记录 |
| `conditions.floor` | 地面材质、平整程度、坡度等实际工况 |
| `conditions.operator` | 现场操作者 |
| `conditions.manufacturer_position_velocity_mapping` | 已核实的厂家 position/velocity 三个分量对应方向、正负号和单位；不能用“已确认”代替实际映射说明 |
| `sdk_axis_mapping_verified` | 只有 SDK 指令轴与厂家反馈轴映射已经核实时才设 true |
| `exclusive_control_verified` | 只有确认没有导航、手柄或其它程序同时发送底盘指令时才设 true |

只读预检和离线预览允许以上确认项保持 false，`conditions` 也可以尚未填全。**实际执行运动前才要求填写完整并完成确认**。此前只读验证已经证明厂家反馈可读，并没有完成轴方向的运动核验，也没有证明现场唯一控制入口。

确认轴方向时，用现场已有且确认可用的人工遥控入口，前后、左右、正反旋转分别短距离低速测试，记录实际方向及厂家反馈变化。还要检查遥控程序最终发送的 SDK 轴，因为手柄中间可能做过坐标转换。没有已确认入口就先保留待验证，不直接改成 true。一般核对的是软件映射，不是修改物理接线。

### 数据来源已经分开

- 主测量：厂家 `/astribot_chassis/joint_space_states`，`position/velocity` 按 SDK 三轴顺序解释；不是 `/joint_space_command_recv` 命令回显，也不能直接称为原始轮编码器。
- 参考：同事部署的 SLAM `/odom`。位姿按唯一源时间去重，忽略原来的交替零速度，参考速度重新差分。两路分别记录，不直接相减未标定的坐标。
- 默认 `slam_reference_required=false`：厂家数据正常即可通过主反馈预检；SLAM 缺失/冲突单独报告，不冒充地面运动已验证。设 true 后，两路均需有效。
- 同一源时间对应不同位姿、源时间回退、DDS 图发现多个发布端不能靠简单去重解决。Humble 使用每 .2 秒一次的发布端图快照检查来源，不是逐消息发布者鉴别。

## 4. 先做只读预检：机器人不会因该命令运动

```bash
PROBE_DIR="/home/astribot/chassis_tests/runs/feedback_$(date +%Y%m%d_%H%M%S)"
python3 /home/astribot/chassis_tests/toolkit_20260914_parking_v3/tools/robot/chassis_characterization.py \
  --config /home/astribot/chassis_tests/configs/x_positive_v3.json \
  --probe-only 20 --output "$PROBE_DIR"

cat "$PROBE_DIR/feedback_report.json"
```

这个入口只有 ROS 订阅，不创建 `Astribot()`，不申请控制权、不发位置/速度、不调用 stop/restart。重点查看：

- `manufacturer.error` 应为 null，`accepted` 持续增加，`backwards` 和冲突计数应为 0。
- `required_feedback_ready=true` 仅说明配置要求的反馈可读，不表示允许运动。
- 单独查看 `slam_reference.error`。此前预检发现多个 `/odom` 发布端，新脚本会将该参考标为不可用；不会自动停止同事的节点。

源时间必须与系统墙钟相容且持续前进。重复源数据不会刷新有效反馈的健康计时。命令返回 1 表示所需反馈未就绪，返回 2 表示配置或参数错误。每次输出目录必须是新目录；同秒重复启动若重名，请稍后重试。

## 5. 再做离线计划预览：仍不会运动

```bash
PLAN_DIR="/home/astribot/chassis_tests/runs/plan_x_positive_$(date +%Y%m%d_%H%M%S)"
python3 /home/astribot/chassis_tests/toolkit_20260914_parking_v3/tools/robot/chassis_characterization.py \
  --config /home/astribot/chassis_tests/configs/x_positive_v3.json \
  --output "$PLAN_DIR"

cat "$PLAN_DIR/plan.json"
```

默认是 x 正方向、从静止起步测试：7 档速度，每档斜坡 1 秒、恒速 6 秒、减速 1 秒、静止保持 4 秒，共重复 3 轮；首次另有 8 秒静止基线。总计 **260 秒，指令积分行程 1.365 米**。这不是实际停车距离，也不是场地所需安全长度；还需考虑偏移、收敛和额外停车空间。

## 6. 现场确认后才执行运动测试

以下命令**会连接 SDK 并执行运动**。仅在现场空旷区域、姿态/载荷/方向已确认、无其它写入口、独立急停有人掌握时使用。新模板的确认项默认 false，不应为了绕过检查而直接改为 true。

```bash
RUN_DIR="/home/astribot/chassis_tests/runs/x_positive_$(date +%Y%m%d_%H%M%S)"
printf '本次结果目录：%s\n停车文件：%s/STOP\n' "$RUN_DIR" "$RUN_DIR"

python3 /home/astribot/chassis_tests/toolkit_20260914_parking_v3/tools/robot/chassis_characterization.py \
  --config /home/astribot/chassis_tests/configs/x_positive_v3.json \
  --output "$RUN_DIR" --execute
```

每次只测一个方向，不自动返回起点，不自动移动双臂/躯干，不自动 restart 或回 Home。它直接使用 SDK 位置积分通道，不经过导航避障策略，不能指望它自动避开场地障碍物。

## 7. 运行中如何停车

“开始自动测试”出现后，在**正在执行测试的终端**按任意一个键：

| 按键 | 行为 |
|---|---|
| 空格、s/S、q/Q | 无需回车，请求 SDK 整机停止并结束本次测试 |
| Ctrl-C | 同样请求中止；初始化期间也可使用 |

不是暂停，没有按键恢复功能；停车后不会继续执行后续档位。重新测试需现场确认状态，并使用新的输出目录。SDK 初始化完成前热键尚未启用，使用 Ctrl-C 或 STOP 文件。

也可以在**第二个 SSH 终端**停车。先登录机器人，再把执行终端打印的完整 STOP 路径复制过去：

```bash
# 示例格式：必须将下面路径换成本次程序实际打印的完整路径！
touch /home/astribot/chassis_tests/runs/本次实际目录名/STOP
```

不要在第二个终端直接用 `$RUN_DIR`：shell 变量不会在两个 SSH 终端之间自动共享。主进程和 SDK 子进程均检查 STOP 文件；文件删除也不会让已终止的测试恢复。

键盘停车要求交互终端；推荐先正常 SSH 登录再运行，不用后台 nohup 执行首次运动测试。非交互执行仍支持信号/STOP 文件。结束后会恢复终端输入模式。

停车调用 **整机 `stop_robot()`**，不再继续追赶旧位置目标。SDK 退出等待期间持续接收反馈，管道关闭不等于 SDK 进程退出；进程退出等待上限为 5 秒，之后最多再观察厂家反馈 3 秒。仅使用停车回执之后的源数据，约 1 秒窗口内速度和位姿变化均满足静止判据才记录 `manufacturer_feedback_settled`；缺反馈或未静止会标记未确认。软件按键不是硬实时急停，SDK/网络阻塞时使用物理急停。

## 8. 四种测试怎么选

编辑配置中的 `mode`。档位数组 `levels` 始终填写严格递增的正数，零速由脚本自动安排。

| mode | 自动流程 | 测量含义 |
|---|---|---|
| `min-speed` | 每档从零起步，恒速后归零，停稳再测下一档，重复多轮 | 当前 SDK 控制链的最低稳定起步速度候选 |
| `descending` | 从最高档开始，逐档连续降低，最低档后停车；每轮间归零 | 已经运动时的最低维持速度候选；不能代替起步门槛 |
| `response` | 各档斜坡→恒速→减速→停车 | 稳态速度增益与响应曲线；完整 jerk/传递函数仍需原始数据分析 |
| `braking` | 斜坡→恒速→约 .02 秒停止增加位置目标→保持 | 正常 SDK filter 收敛/停车响应，不是机械急停或独立 watchdog 认证 |

最小速度默认档位为 `.002, .004, .006, .008, .01, .015, .02 m/s`。先采静止噪声，基线剔除 `baseline_discard_s`（默认 1 秒）启动瞬态，至少保留 2 秒；厂家速度噪声取剩余基线各 1 秒窗口按源时间加权的均值绝对值最大值，避免单帧峰值否决全部低速档。基线仍有明显漂移时不输出候选。恒速段最初 1 秒不用于稳态判断；剩余段分三段，每段均要求正确方向的持续位移，厂家报告速度也必须超过基线噪声门槛。同一档所有配置重复都满足，才给候选。

位置检出阈值为 `max(min_displacement_m, noise_multiplier × 基线位置峰峰值)`；旋转使用 `min_rotation_rad`。低速未检出可能是无法辨识，不代表真正最低速度为零。若 .006 不稳定、.008 稳定，可在两者之间增加细档位再测；不要只为得到更小结果而压低噪声阈值。

停止阈值默认为 `.01 m/s`、`.02 rad/s`，可能高于最低速度档。制动试验须使用明显高于静止阈值且已经验证的速度，或先结合静止噪声建立可辨识的阈值，不能把低于阈值的运动自动当作物理静止。

### 六个方向与边界

| 方向 | axis | direction | levels 单位 |
|---|---|---|---|
| SDK x 正 / 负 | x | 1 / -1 | m/s |
| SDK y 正 / 负 | y | 1 / -1 | m/s |
| SDK theta 正 / 负 | yaw | 1 / -1 | rad/s |

方向是已核实的 SDK 轴，不擅自将 x 正等同于物理前进。每个方向复制单独配置文件并重新预览。旋转模板可从已核实的小档位开始；修改档位/重复/时长后应重新核对总转角，线速度数值不能无条件照搬成角速度。

**旋转试验的最低档存在硬下限。** 本机厂家 `wz` 反馈静止时恒为 0.00323 rad/s，而有效静止阈值是 `min(settle_yaw_rate_rad_s, 最低档 / 4)`，因此最低档低于 **0.0129 rad/s** 时静止判据永不可能满足，程序会在静止基线阶段就中止（实测 0.005 rad/s 起档即如此）。要抬最低档，不要放宽 `settle_yaw_rate_rad_s`；证据与影响见 [四方向平移 + 原地旋转（五轮）](CHASSIS_FOUR_DIRECTION_AND_YAW_20260914.md)。

程序同时限制指令速度、斜坡加速度、计划时长、指令积分行程、厂家反馈按测量分辨率累计的行程/转角、位姿跳变、源数据新鲜度及 SDK 指令超前反馈的偏差。边界触发后中止，不会自动放宽并继续。

## 9. 查看结果

在**原执行终端**，程序退出后 `$RUN_DIR` 仍保留：

```bash
cat "$RUN_DIR/summary.json"
ls "$RUN_DIR"
```

如果换了 SSH 终端，先找本次目录，再用完整路径：

```bash
ls -lt /home/astribot/chassis_tests/runs
```

| 文件 | 内容 |
|---|---|
| `config.json`、`plan.json`、`metadata.json` | 本次配置、阶段计划和脚本哈希/环境 |
| `samples.csv` | 厂家主反馈的唯一源时间位置与速度、阶段和目标速度 |
| `slam_reference.csv` | SLAM 去重后位姿及重新差分的参考速度 |
| `events.jsonl` | SDK 目标与实际反馈、实际积分速度、停车请求/回执、异常 |
| `stop_feedback.jsonl` | 停止请求返回后的厂家反馈观察；未创建 SDK 时可能不生成 |
| `summary.json` | 各档候选、稳态增益、停车估计、双路健康及执行结果 |

`execution_status=operator_stopped` 表示人工终止，数据会保存，但不输出完整测试通过的最低速度结论。`sdk_stop_request_returned=true` 只说明 SDK 调用返回；还要看 `stop_observation` 并现场确认。

`minimum_continuous_motion_candidate` 是当前 SDK 控制链的反馈候选，不是电机原始死区。`ground_truth_verified` 和 `hardware_validated` 不会自动变成 true；排除滑移及验证 3 cm/1.5° 到点精度仍需独立测量。

### 重新离线分析（不会运动）

在原终端中使用本次保存的配置，而不是之后改过的配置：

```bash
ANALYSIS_DIR="/home/astribot/chassis_tests/runs/reanalysis_$(date +%Y%m%d_%H%M%S)"
python3 /home/astribot/chassis_tests/toolkit_20260914_parking_v3/tools/robot/chassis_characterization.py \
  --config "$RUN_DIR/config.json" --analyze "$RUN_DIR/samples.csv" \
  --output "$ANALYSIS_DIR"
```

离线分析只分析选中的 CSV，不恢复执行历史；必须连同原 `summary.json` 和 `events.jsonl` 的中止信息一起看，不能用重分析覆盖失败结论。

## 10. 可选：同时采集其它传感器

在第二个真机 SSH 终端，重新加载环境。以下全为只读采集，不发指令：

```bash
source /home/astribot/chassis_tests/toolkit_20260914_parking_v3/env_test.sh
CASE_DIR="/home/astribot/chassis_tests/runs/raw_case_$(date +%Y%m%d_%H%M%S)"
python3 /home/astribot/chassis_tests/toolkit_20260914_parking_v3/tools/robot/validate_hardware_params.py init "$CASE_DIR"
nano "$CASE_DIR/topics.json"
nano "$CASE_DIR/conditions.json"

python3 /home/astribot/chassis_tests/toolkit_20260914_parking_v3/tools/robot/validate_hardware_params.py record \
  "$CASE_DIR" --item 03_braking --environment hardware --duration 90
```

按实际启用的话题调整 `required`，明确本次未采集的项目；默认原始采集还要求 `/scan`、`/tf`、`/odom`，不能将其缺失等同于厂家主反馈失效。原始采集保留重复帧作为证据，不会自动去重改写原始数据。

## 11. 常见提示

- **找不到 env.sh**：确认 source 的是本文绝对路径 `env_test.sh`，不是工具包内的 `env_robot.sh`。
- **输出目录已存在**：使用新目录名；不要删除原结果来规避。
- **反馈协议已更新**：从 parking_v3 模板新建配置并迁移工况，不能直接使用最初 v1 的旧配置。
- **工况/轴映射/唯一控制入口未确认**：只读预检可以先做；运动执行需完成实际确认，不要为了通过检查填写不实信息。
- **SLAM 多发布端**：联系部署同事确认来源。工具不会擅自关闭 SLAM，不把冲突参考混入厂家测量。
- **停车未确认**：以现场机器人实际状态为准，必要时使用独立急停；不要马上启动下一次测试。


### `No module named 'astribot_sdk'`

每个新 SSH 终端都要重新加载环境，即使已经位于工具包目录。目录正确不代表 Python 搜索路径正确。先运行下列只读路径检查，不要直接重试运动：

```bash
source /home/astribot/chassis_tests/toolkit_20260914_parking_v3/env_test.sh
python3 -c 'import importlib.util; s=importlib.util.find_spec("astribot_sdk"); print(s); assert s is not None, "SDK package not found"'
```

工具包已补强环境入口：在 overlay 加载后再次加入 SDK 根目录；SDK 子进程也按 `ASTRIBOT_SDK_ROOT` 补充导入路径。导入阶段失败会明确标为 `initialization_failed`，不会误称已创建 SDK 后停车未确认。构造函数已经开始、控制接口可能已产生副作用的失败仍单独处理，不能将其视为单纯导入失败。


## 2026-09-14 跑机分析后的判据修正

- `motion_position_resolution_m=0.0005`、`motion_rotation_resolution_rad=0.001`：位姿从上一个累计锚点变化达到分辨率后才累计，锚点之间的剩余变化也参与越界检查。持续慢移、慢转仍会逐步累计，不能用净起终点距离替代路径长度。小于分辨率的往返变化无法可靠区分真实运动和噪声；原始逐帧总变化仍保存在 `motion_budget.raw_*` 中。配置限制分辨率最大为 1 mm / 0.002 rad，且不能大于预算的 1%，不是允许放宽场地边界。
- 平移试验有效静止速度阈值为 `min(settle_speed_m_s, 最低档速度 / 4)`；旋转试验对角速度作同样处理。默认 x 试验为 **0.0005 m/s**。约 1 秒窗口内，源时间加权的速度绝对值均值和位姿变化范围都必须满足判据；数据中断、慢速爬行、往返振荡不能仅凭平均净速度为零判静止。
- `stop_metric_status` 区分 `settled`、`unconfirmed` 和 `initial_speed_not_distinguishable`。没有可辨识的停车前运动或没有确认静止时，停车时间及确认距离为 `null`。计划减速开始时刻并非机械制动触发时刻。
- `sdk_stop_request_returned`、`sdk_worker_exited`、`stop_observation` 分别表示请求返回、SDK 进程退出和反馈静止确认；三者不能互相替代。退出超时单独记录 `sdk_exit_timeout`。
- `--analyze` 会读取 CSV 同目录原 `summary.json` 的执行状态。原试验中止或状态未知时，`minimum_continuous_motion_candidate` 保持 `null`，重算候选放在 `offline_candidate_before_execution_review`，不能把离线重算当作完整真机通过。原始文件不会被覆盖。

本次记录的详细原因和验证结果见 [跑机复盘](CHASSIS_RUN_REVIEW_20260914.md)。


### SDK 退出收尾与时序诊断补充（17:12 跑机后）

SDK 子进程在停车回执被主进程接收后，按厂家 `examples/211-whole_body_control_test.py` 的方式调用 `middleware.shutdown()`，结束本进程的 ROS 执行器；这不是关闭机器人厂家服务。事件中增加 shutdown 开始/返回/失败记录。循环 0.05 秒与指令租期 0.25 秒保护保持原值，触发时分别输出 `loop_dt_s`、`command_age_s` 和当时速度。

每档报告新增 `third_displacements` 与 `third_displacement_threshold`，可直接定位前、中、后哪段位移不足。延长恒速观察时间会改变低速检出能力；比较候选时必须同时记录恒速时长。SDK 使用位置积分和 filter 控制，测得的是此控制链的等效低速响应，不等同于厂家原生速度模式死区。
