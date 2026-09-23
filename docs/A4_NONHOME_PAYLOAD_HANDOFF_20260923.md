# A4 非 home 与多载荷前置及执行顺序

这是当前源代码能力核查和后续执行配方，**不是非 home / 多载动态验收报告**。运行使用统一导航仓库与六相机 preset，所有批次由导航任务拥有。单项 A4 计时不因本配方重置。

## 当前真正可用的边界

- A1 C++ ECM 观察器可以独立枚举已登记的箱体、圆柱、球、mesh 运动学附件，并生成带质量、碰撞几何和版本的全量物理观察。独立 PlanningScene 回读后账本才确认。
- A2 `HoldResources` 只保持当前实测22关节，不接受目标姿态，也不锁住底盘或场景写入者。
- A3 可根据该实际保持姿态生成固定包络并由6个实际消费者确认；不能由 `posture_id=ready` 推断手臂已在 ready。
- `PlanSkill` / MTC 只提供规划结果。现有 PlanSkill 结果缺少完整任务权限、场景/标定版本与执行时快照绑定；不能把返回成功的轨迹直接交给任意新控制器客户端。
- 已修复有载包络仅检查质量为正的漏洞：C++ 核心现在绑定有效账本、附件版本、完整物体 ID 集合和实际质量总和；节点复用 payload::Consumer 并增加墙钟失效检查。32项C++链路测试和独立权威契约通过，**尚未进入新节点运行或有载动态验收**；当前 N4 继续使用冻结安装。详见 [质量准入证据](A4_PAYLOAD_MASS_GUARD_20260923.md)。

## 无故障夹具准备

在后续明确独占的有载批次中，使用 `--payload-source-id gazebo_kinematic_v1` 启动同一导航环境。不能在已加载 static EMPTY 源的会话中再插入第二个库存源。

```bash
python3 tools/sim/verify_kinematic_inventory.py \
  --prepare-only --owner <new-owner.json> --session <actual-instance> \
  --source gazebo_kinematic_v1 \
  --plugin-directory <inventory-install>/lib \
  --world-reference <actual-world-file> --output <new-evidence-directory>
```

新增 `--prepare-only` 的实现与14项离线检查已完成，**尚未新栈实测**。它复用原 A1 四种夹具与真实 C++ 执行/观察插件，直接为所有物体加载插件；成功路径不执行缺插件、附着、幻影对象、未知物体或暂停故障。先持有导航验证使用的 waypoint 文件互斥锁，确认 hold idle、底盘持续0.6秒停稳；位姿漂移、非零命令、无效姿态、旧里程计均拒绝。准备期间持续观察并锁存运动/失效；较长外部调用期间主线程继续接收观测，已在途调用等待有界终态后停止，禁止继续创建。这个运维互斥和停稳检查不能代替未来正式底盘租约，也不能与不遵守该锁的写入者形成原子互斥。

成功后所有已登记模型明确 detached，保存 `registry.json`、`prepared_fixtures.json` 和版本证据。实际 diagnostics 的完整执行模型集合必须与 registry 相同；缺少、重复、异物、attached或 accepted/applied 不一致拒绝。136条历史A1明确EMPTY诊断回放通过，仅证明检查器兼容真实旧输入。此阶段确认模型身份与detached状态，夹具规格文件不冒充尚未附着时的质量/几何实测。

ROS/steady 时效与独立场景回读使用和 EMPTY 相同的严格验证屏障。失败保留来源与身份，不清账本、不启动第二个源，不对既有或复用模型发送detach，不修改PlanningScene。所有在途外部操作记录返回值及 stdout/stderr；创建请求应答不是实体状态证明，后续必须查看独立库存。不得把 prepare-only 成功写成有载或动态通过。

## 载荷切换事务

1. 导航权威确认旧目标终态，采集连续新鲜停稳窗口；保持旧载荷不松开。
2. 正式保持按当前阶段要求撤销并等待真实子动作终态、实测安全状态和资源释放；如未知则隔离。
3. 只操作 registry 中本次创建的模型。读取独立 diagnostics 中的 accepted/applied 与执行 epoch；accepted大于applied时不提交下一条命令，恢复不从序号1重放。
4. 执行 C++ kinematic command，服务响应只代表接收。等待独立 ECM 观察到新 applied 序号、正确parent及实际几何；PlanningScene 尚未一致时必须拒绝准入。
5. 将实际物理观察中的 AttachedCollisionObject 投影到 MoveIt；独立全量 GetPlanningScene 回查，由账本确认，不直接发布正向 AttachmentState。
6. 新几何包含所有实际手臂、夹爪、相机及载荷的保守分层凸包。新的保持绑定 attachment_revision；新 SetFixedEnvelope 使用该确认版本，质量由账本所有对象 weight 求和，并经 C++ 准入核对。
7. 六消费者同 coordinator/session/epoch/hash 确认后，导航任务才可发目标。到站按停稳→新观测→操作的顺序交接。

## 场景顺序和失败语义

| 场景 | 真实前置 | 当前状态与预期 |
|---|---|---|
| 当前实测姿态、明确空载 | A1–A3 + 新会话 bootstrap | 导航任务执行 N4 动态准入、hold撤销、ACK失联、实测停止和恢复 |
| 左臂 / 右臂 ready | 受权姿态规划执行、另一臂保持、实测到位、新包络 | 缺正式姿态执行交接；不得仅修改posture_id |
| 左箱体0.5kg / 右圆柱0.75kg | 真实附着/scene一致、有效姿态、总质量核对 | 待前置完成；从开阔直线和侧移开始 |
| 左箱体+右圆柱双载1.25kg | 两个独立物体身份均在全量账本；不能漏掉任一侧 | 待前置完成；回转、窄通道对齐/退出、起终点朝向 |
| 偏置 sphere / mesh | 实测pose、保守几何与偏置载荷版本 | 使用A1物理几何基线；偏置变化必须新包络，禁止复用旧hash |
| 载荷丢失 / parent变化 / 状态过期 | 对应独立来源故障 | 撤销准入并停止；恢复必须重新确认版本和保持，不自动复用旧命令 |
| 遮挡 / 稀疏RGB-D / Voxel高度覆盖 | 固定姿态与载荷前提已满足，记录真实相机/点云时间和TF | 观测不足拒绝；不降低净空、碰撞或时效阈值换成功 |

每一行分别记录成功、预期拒绝、被测失败、基础设施失败与未运行。到位/跟踪用导航方固定口径；源失效到许可撤销、最终零输出、实测停稳的时延各自记录，不能互相替代。

## 非 home 运行时补齐方向

下一实现应在 C++ 正式资源所有者内加入“受权姿态准备→保持”事务：申请底盘/双臂/躯干/夹爪及场景写权限，停稳后建立快照；调用既有规划器并绑定实际起点和版本；执行前复核碰撞/限位/时间参数化和退出路径；真实子 Action UUID/result与新鲜实测到位满足后，在同一资源租约内转入 ArmHold。中途取消、单臂失败、时钟回退或未知结果均禁止新导航，未知状态保留隔离。

不能先释放全部资源再用另一个无版本执行器改变姿态，然后将最后一帧当成完整事务。扩展既有 `ResourceAuthority` / `ChildActions` / `ArmHold`，避免再维护第二套 Python 运行时保持逻辑。正式底盘/场景租约及受权执行接口尚未实现，须作为独立缺口验收。

装配的后续交接参考 [FoundationPose协作契约](FOUNDATIONPOSE_ASSEMBLY_COORDINATION_20260923.md) 与 [插孔设计](PEG_IN_HOLE_ASSEMBLY_DESIGN_20260923.md)。A1/A4 运动学附着不证明 C1刚性接触插入，更不证明 C2摩擦抓持、力控或真实载荷惯性效果。
