# 仓库仿真的受限 EMPTY 证据源

`astribot::EmptyInventory` 是 C++ 世界观察插件，只读 Gazebo ECM。它只支持
`static_world_empty_only_v1`，**不是通用有载清单或力接触抓取检测器**。

数据流：独立基线描述 + Gazebo 实际模型/插件/关节 →
`/payload/attachment_observation` → C++ 附件账本 + 独立 MoveIt 完整回查 →
`/payload/attachment_state` → C++ 几何消费者。插件没有 PlanningScene 写入或运动权限。

## 准入条件

- 世界及机器人名称唯一匹配；世界插件信息可读。
- 所有外部模型与原始仓库 SDF 基线的模型集合、父级路径、位置、静态性质和直接 link 数量相符。
  允许无直接 link 的 SDF 包装模型，本体必须来自已核对的静态背景。
- 机器人必须是基线的扁平模型，直接 link、joint 名称集合及 joint 两端必须匹配独立机器人描述。
  新增嵌套模型、link、joint 不能继承“机器人本体”豁免。
- 实例 ID 绑定后，被同名新实体替换仍拒绝；最多 4096 模型、256 插件。
- 只允许代码中明确列出的物理、传感、控制系统。`KinematicPayload`、未知插件、外部关节、
  `DetachableJoint`、额外静态/动态物体、缺元数据均不能证明 EMPTY。
- 支持范围内的完整 EMPTY 必须再经独立完整 PlanningScene 回查确认，才交给几何计算。
  即使几何完整，仍需正式 ArmHold 和六方同版本确认才能导航。

这是一种保守的封闭场景策略。新增货物、社会导航代理或别的世界均可能使其拒绝；不能通过把
这些实体加入背景白名单来代替有载来源实现。后续有载源需要直接读取实际附着执行状态和几何。

## 版本与时效

每次插件实例化生成源 epoch；物理采集序号递增，清单或完整性改变生成新 revision，时间回退增加
clock epoch。有效期从实际物理采集起 300 ms，不用发布或收包时间延长。

Gazebo PostUpdate 与 ROS `/clock` 到达顺序可能相反。观察器只缓存一个正向采集，下一次 50 ms
物理采集确认同一清单后才发布上一帧，保留原采集与截止时间；负向变化立即撤销缓存。ROS 时间和
单调时间均限制缓存，暂停后不能补发已经过期的 EMPTY。

## 启动顺序

1. 使用有归属记录的隔离导航仿真，固定 `instance / ROS_DOMAIN_ID / IGN_PARTITION / 发现端口`。
   在 supervisor 显式传 `--navigation-geometry-mode fixed_v2 --payload-source-id gazebo_empty_v1`。
   该参数只启动只读账本，身份来自 instance，日志落在会话目录；不自动构造空载。
2. 启动同一域的 MoveIt 场景服务，做只读回查时可设置 `allow_trajectory_execution:=false`。
3. 从规划/几何共同使用的可信机器人描述保存 URDF，再用同版本 `ign sdf -p` 转换为机器人 SDF。
   世界基线使用原始仓库 SDF。**不要从当前 ECM 导出带未知附件的场景作为可信基线**。
4. 在确认拥有的环境中运行 C++ `load_empty_inventory`，显式传入七项参数：

```text
load_empty_inventory --world default --robot astribot_s1 \
  --session INSTANCE --source gazebo_empty_v1 \
  --world-reference ORIGINAL_WAREHOUSE_SDF --robot-reference TRUSTED_ROBOT_SDF \
  --plugin ABSOLUTE_PATH_TO_libastribot_empty_inventory.so
```

加载器从完整世界状态查找实际 World entity ID；不能假定热加载服务会解析 entity.name。
它拒绝默认 domain 25、身份不匹配和重复加载。成功只表示 `INVENTORY_LOAD_QUEUED`，仍须观察
`/payload/simulation_inventory_diagnostics`、账本和几何输出才算接通。配置失败不会抛出异常中止
Gazebo，但会停止该观察器发布，不能把缺消息当作 EMPTY。

## 验证入口

- `inventory_gate_test`：身份、背景、机器人结构、未知机制、时钟/版本及缓存期限。
- `tools/sim/verify_empty_inventory.py`：有归属的实际仓库中注入一个临时静态物体、一个 MoveIt
  虚拟附件及短暂暂停，检查拒绝和恢复，并在 finally 清理；不发送运动目标。
- `tools/sim/verify_missing_hold.py`：真实几何下验证缺 hold、缺请求身份及旧接口无法绕过准入。
- `ws_robot/src/astribot_s1_payload_state/test/test_ros_protocol.py`：显式开启的 domain 116 ROS
  夹具测试，包含 50 ms 采集延迟、断流、暂停、重启及版本变化；这不属于实际物理清单证明。

现场结果和当前仍缺少的有载/任务保持接线见 `docs/PAYLOAD_STATE_SIMULATION_20260923.md`。
