# A4 空载正式链路交接配方

状态：A1/A2/A3 已阶段验收；本配方脚本16项离线检查通过，**新启动会话尚未执行本配方**。A4 非 home/多载动态与感知联合矩阵尚未验收。

## 所有权与启动前提

导航任务拥有仿真生命周期、环境、安装路径和所有底盘动作。本窗口不启动第二个 Gazebo。以新的 supervisor 身份文件绑定 boot_id/PID/start_ticks/exe/完整 cmdline，环境须匹配同一 instance、domain 和 partition。旧 domain94 重启前记录只用于历史证据。

唯一导航世界、六相机预设保持原样。固定包络会话启动时显式指定 `--navigation-geometry-mode fixed_v2 --navigation-policy p4 --payload-source-id gazebo_empty_v1`（p5 亦可），使用本次实际 instance/domain。原 legacy 会话不能通过另起一个协调器声称已切换 fixed_v2。当前 EMPTY 观察器只接受基准静态背景；增加未登记动态人/对象会拒绝，不将其过滤掉换取通过。

先由会话所有者完成 I0.2 门槛并切换到 fixed_v2 验证批次。不得在导航性能采样中途加进程改变对照负载。

从自己的启动句柄取得 supervisor PID，采集其身份（不搜索全机第一个同名进程）：

```bash
python3 tools/sim/capture_supervisor_owner.py \
  --pid <owned-supervisor-pid> --session <actual-instance> \
  --source gazebo_empty_v1 --output <new-supervisor-owner.json>
```

该命令拒绝 legacy/source/domain/partition 不一致，只记录明确 PID 的身份；进程匹配本身不提供控制授权。启动任务负责持有原始句柄。

## 只规划 MoveIt 与正式保持器

沿用当前 `move_group.launch.py`，无需修改 launch；同会话启动时必须 `use_sim_time:=true allow_trajectory_execution:=false use_rviz:=false`，并明确传入该会话解析后的六相机参数：

```text
camera_profile:=<description-share>/config/simulation_navigation_full/camera_head_rgbd.yaml
torso_camera_profile:=<description-share>/config/simulation_navigation_full/camera_torso_rgbd.yaml
camera_calibration_dir:=<description-share>/config/simulation_navigation_full
camera_mounts_profile:=<description-share>/config/camera_mounts_reference_sim.yaml
use_camera:=true use_wrist_cameras:=true use_stereo_cameras:=true use_lidar:=true
```

`description-share` 必须来自本次实际安装；模型检查会比较真实参数，不能只看路径名称。当前源码的 `transport_skills.launch.py` 已修正为读取同一 preset 并完整转发4个相机路径；旧安装仍可能硬编码旧 profile。此批只需 MoveIt 与保持器，不额外启动未用的 MTC/技能节点来改变采样负载。

使用已构建的 `astribot_s1_transport_native/hold_executor`，参数 `use_sim_time:=true simulation_commissioning:=true`。它独占6组控制器客户端；同时启动 MoveIt 执行管理器、旧搬运执行器或直接轨迹客户端会拒绝。日志中的 unresolved/quarantined 必须保留并处置，不能删除持久 journal 后重试。

导航 launch 只运行一份最新 `fixed_envelope_cpp`。A3 已修复 ACK 反馈放大与负 ACK 后旧正 ACK 重放，必须使用 c0d24b1b 对应源码构建的版本。

## 加载真实全量 EMPTY 观察器

`prepare_empty_inventory.py` 是一次性运维/验证脚本，运行时源、账本、几何、保持和包络全部为 C++。它读取两个真实 robot_description，检查六相机物理结构一致，转换机器人 SDF，然后调用 C++ loader。它不发布 attachment/hold/ACK，不修改 PlanningScene，不发送运动命令。

在已核对并 source 的**新会话环境**中运行：

```bash
python3 tools/sim/prepare_empty_inventory.py \
  --owner <new-supervisor-owner.json> \
  --session <actual-instance> --source gazebo_empty_v1 \
  --plugin-directory <inventory-install>/lib \
  --world-reference <actual-world-loaded-by-this-session.sdf> \
  --output <new-evidence-directory>
```

本仓库已验收 observer 安装可参考 `runs/joint_acceptance_20260923/inventory_install/lib`；使用前核对库及 overlay，不能 source 旧查询脚本后保留旧 domain。world-reference 必须是本次 Gazebo 真正加载的世界（有 social world 生成时是生成产物）。脚本拒绝不匹配的 supervisor/source/environment。

检查 `<output>/result.json` 的 `passed=true`：独立 source、完整场景回读、confirmed ledger 和同版本完整 geometry 持续2秒且每项 ROS/steady 时效满足。重复采集包不续期；回读绑定完成稳定窗口时的源/时钟/账本/附件/模型版本，查询期间失效或版本变化须重新完整观察。服务 `queued` 不算完成。加载后失败则 observer 留存，本配方失败不构成准入通过；脚本不授予或撤销既有导航许可。用新输出目录加 `--observe-existing` 重查，不能重复加载第二个源或自动清空未知附件。

## 六消费者与故障恢复

分工：bootstrap 不持有任何 HoldResources 客户端。导航任务的动态验证探针独占 Action、renew 与 SetFixedEnvelope；下面静止回归只能在动态探针退出后串行执行，禁止两者争用关节。

EMPTY 成功后运行已经实测通过 A3 的脚本：

```bash
python3 tools/sim/verify_fixed_envelope_hold.py \
  --owner <new-supervisor-owner.json> \
  --output <new-evidence-file.json>
```

它按真实 HoldResources → renew → SetFixedEnvelope → global/local costmap、planner、controller、policy、protection 六消费者确认执行9个静止场景。包括真实 consumer 失联、错误实际 footprint、场景不一致、世界暂停、取消/过期及恢复；不需要伪正 ACK，也不发底盘目标。

ACK 丢失/错误安装撤回后，后续严格更新的有效 ACK 可在同 epoch 恢复；hold、geometry、时钟故障撤销后必须新请求/新 epoch。每次资源终态都须 `resources_released=true`。这仍是静止链路验收，**不证明运动中制动距离、非 home 导航或多载荷动态成功**。

## A4 动态验证的交接门槛

动态验证需要保持租约贯穿整个导航动作，使用已有导航仲裁入口，记录实际命令、里程计、路径、到位、hold/geometry/ACK 版本。正常结束须先导航终态和连续实测停稳，再取消保持并确认资源释放。故障实验保留保持直到停车确认；保持自身失效则必须验证导航终止与恢复新 epoch，不能仅凭最终零命令判定已停稳。

当前 HoldResources 仅保持当前实测姿态。非 home 初始姿态必须有额外的已验证规划—执行—实测确认链；单/双载必须通过独立 ECM 全量登记/已应用状态和完整 PlanningScene 一致性，不复用 EMPTY 静态策略冒充有载。下一批接口演进在 C++ 完成，运动学附件验收不外推为抓取接触/摩擦/力控验收。
