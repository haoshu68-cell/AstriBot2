# FoundationPose、导航到位与搬运装配协作契约

2026-09-23。依据用户明确要求，与下列两个当前活动任务协作。本文由 FoundationPose 任务唯一编辑；其他任务通过消息提供补充，避免共享工作区同文件并发写入。

| 任务（当前准确标题） | ID | 修改职责 |
|---|---|---|
| 制定FoundationPose整合方案 | 01a0ccca-7607-7bc0-b7d9-12dc8c8e6eda | FoundationPose 新后端/快照/资产/质量门控、装配 CAD 和观察契约、本文件 |
| 完善path_tracking到位精度 | 01a09962-8e15-71e0-b9a6-a92798318698 | path_tracking 到位/停稳验证；I0.2 导航仿真；不改 foundationpose/assembly |
| 设计机器人搬运仿真流程 | 01a0c405-627f-7742-b500-e057726ed2e7 | transport_native、payload_state、fixed_envelope、任务资源/scene/attachment 提交与执行流程 |

保留单一 `chassis-effort-drive` 主线；本文不授权对整个未提交工作区做 reset、批量提交或清理。各任务仅处理自己的修改，不覆盖他方包、验证脚本和运行目录。

## 运行资源与阶段次序

- 导航任务报告本轮 I0.2 窗口为 14:05:45–15:35:45（Asia/Shanghai），使用 domain89、partition `astribot_i02_20260923_nav`、端口18178/18179、私有显示`:96`。独立目录为 `/home/yjh/WorkSpace/astribot_validation/unified_navigation_resume_20260921_01/I0_2_20260923_065115`。
- FoundationPose 在该窗口只执行 CPU 离线准备、下载和设计，不启动第二套 Gazebo、GPU推理或TensorRT engine构建；到时后仍需收到所有者释放确认。
- 用户随后授权安装 Docker Engine 与 NVIDIA Container Toolkit。导航任务明确要求本轮完成前不实际安装/启服务，因为安装可能自动启动 Docker 并改变 systemd、网桥或 iptables，影响 DDS 发现和时效测量。安装授权保留；先准备官方材料与安装脚本，收到导航任务明确释放通知后再安装，不以时钟到点推定释放。
- 搬运任务已将仿真/GPU交接给导航；A1/A2 提交分别为 `e6123515`、`09bc10d7`，A3 提交 `c0d24b1b`。A3结果见 `FIXED_ENVELOPE_ACCEPTANCE_20260923.md`，本窗口没有复跑，不能以转述代替新的运行证据。
- 保留 `I0.2 → A4` 先决顺序。FoundationPose P0 离线准备不代表该门槛已过，不开放新抓取或装配运动阶段。
- 搬运任务随后报告 A4 窗口为14:13:31–15:43:31，当前做纯运维 EMPTY bootstrap 和非home/载荷前置；动态探针 HoldResources 客户端仍由导航唯一持有。该进度不是 A4 已验收。

### 15:31 资源交接与基础安装

导航任务15:31明确释放维护/仿真/GPU资源，`I0_2_navigation_20260923_140545/current_cleanup.json` 确认其会话无残留进程、私有显示退出。搬运任务随后确认自14:13起只做离线检查，当前无构建和运行会话。FoundationPose 独立核对未见 ROS/Gazebo 进程或 GPU 计算任务后接手；没有停止其他任务进程。

15:33 Docker Engine 29.8.1 / NVIDIA Container Toolkit 1.20.1 安装完成，新增9包、既有包版本无变化、驱动仍580.178.04。随后无网络、只读临时容器内的显卡可见性检查通过，容器退出删除。证据见 `evidence/foundationpose_p0_20260923/container_installation.json`；该检查不是 FoundationPose 推理或装配验收，也不改变导航 I0.2 / 搬运 A4 的业务验收状态。

### 15:57 再次交还导航回归

搬运任务收到用户继续要求后恢复 A4；导航任务随后明确进入唯一 N4 六相机回归窗口，先 consumer_pause、后独立 90° 返程，预计16:25前清理交还。FoundationPose 收到实际启动通知后立即中止大型镜像拉取（exit130），没有启动模型构建或 GPU/ROS 容器；下载缓存保留，后续续传必须使用新日志，不能改写中止记录。

期间仅继续轻量离线脚本/文档；时间到点不等于资源交还。主机容器维护已完成，新增 docker0 网桥/路由和服务状态已通知两任务；导航重新核对 DDS 和路由，安装前后性能证据分别记账。引擎转换等待下一次明确释放，不与导航或搬运测量并跑。

### 16:08–16:36 基镜像与CPU模型检查窗口

导航16:08明确交还；`I0_2_navigation_20260923_155154/window1_cleanup.json`确认三会话stopped/remaining空、owned runtime空、两个私有显示进程exit0。本任务读回后恢复下载，16:30固定TensorRT基镜像取得，16:34两份ONNX完整语义/输入合同CPU检查通过。未分配GPU或启动ROS后端；实际TRT26与目标TRT30的版本差异仍待容器对齐。

按新全窗口一小时规则，FP-BACKEND-01在本轮检查点有界收尾并deferred；约16:36再次向导航明确交还全部下载/仿真/GPU资源，本任务容器为空、管理员终端退出。FoundationPose转独立CPU槽板S0；导航/搬运后续按最新用户方向恢复扫描或执行静止探针，统一由导航协调唯一会话，本任务不因此自动重启GPU工作。

## 导航到装配的数据/控制交接

```text
导航 Action 终态
  → 独立新鲜 TF/odom 连续停稳观测
  → 搬运任务取得操作资源，确认没有新导航执行
  → 确认适用的关节保持/对象账本/场景版本
  → 新采集板与工件观察（各自 frame、时间、标定、实例与质量）
  → 相对槽位精对准；按装配公差独立准入
  → 抓取/预插入/插入/实测验证
  → 原任务所有者提交对象、附着或放置状态
```

- `/navigation/execution_status` 的 NavigationExecutionStatus 含 task/source/sequence/state/reason/action_status；它不是停稳证书。
- `ARRIVAL_REACHED` 是控制器日志事件，不能代替 Action终态或独立实测停稳；3 cm/1.5° 是导航阶段要求，SLAM相对误差不能当作槽位相对精度。
- `/transport/hold_resources` (HoldResources)、`/transport/hold_executor/renew` (RenewHold) 当前只保持22关节/6个JTC的实测姿态；不是任意姿态规划、base锁或scene写事务锁。
- `/navigation/arm_hold`、`/navigation/geometry_state` 按原任务方的所有权、lease/epoch消费。保持失效先撤销，子动作终态与新500 ms稳定窗口后才释放，不能由感知宣称释放成功。
- `/payload/attachment_state` 为原任务方确认来源；感知只读消费。EMPTY需权威完整库存和独立完整PlanningScene读回，空检测数组不能证明空载。
- 准入快照关联 task/context、导航执行代际、base停稳采样窗口、hold/geometry epoch、scene/object/attachment revision、camera/source/clock/calibration/association revision。任何相关状态改变使旧计划失效。
- 板与工件观测必须采集于停稳后；规划和执行前分别重新核对快照龄期、资源权限及实测起点。attach/release 只能由执行环境的独立库存与完整 scene 读回完成提交。
- 正式 base settling/hold 收据仍是后续接口；在其设计和验证完成前，不给现有状态附加不存在的授权语义。

## 相机实际入口更正

搬运任务提醒并经本窗口读取历史 evidence 核实：当前统一 six-camera preset 为 `use_camera_postprocess=false`。躯干记录中的入口为：

```text
/camera/raw/torso_rgbd/image
/camera/raw/torso_rgbd/depth_image
/camera/raw/torso_rgbd/camera_info
message frame: astribot_s1/astribot_torso_link_4/torso_rgbd_sensor
```

来源 `evidence/joint_acceptance_20260923/camera/six_camera_raw_streams.json` 是当时的5秒传输记录，不证明当前新会话仍在运行，也不证明曝光同步、光学轴、K/D/P或RGB-depth配准合格。

名义配置：头/腹640×360@20Hz、腕640×320@20Hz、双目400×300@5Hz。实际帧率/端到端龄期需在具体会话实测。FoundationPose 不能继续默认等待旧后处理topic；也不能仅把原始消息 frame_id 重写成 optical frame。输入适配阶段需核对真实传感器几何与变换。

## 新装配场景的共同边界

用户确认首期保留顶部抓持段，后续增加齐平嵌入。先固定板、固定底盘、单臂执行，另一臂停放；多形状匹配场景参考开源资产组织，采用本项目参数化设计。

现有 kinematic 载荷跟随只证明该运动学仿真链，不能证明摩擦、夹持、卡滞、接触力或插入成功。新场景分别标记感知、几何可达、轨迹执行、接触装配四层证据。接触阶段需经过独立物理/反馈门槛后由搬运任务接线；FoundationPose 不直接发末端或关节指令。
