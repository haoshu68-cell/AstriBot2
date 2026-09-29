# P0/P1 上位机功能演示与操作手册

版本：2026-09-20。本文面向开发仿真演示，默认使用独立 ROS localhost 域 224 的假后端。它不会启动 Gazebo、SLAM、真机驱动或向底盘发布速度。

## 1. 演示范围

P0 演示控制权租约、命令幂等/越权拒绝、后端状态和安全释放；P1 演示 RViz 工作站、地图显示、单点导航接口、多点循环路线、探索暂停/恢复/取消存图、诊断录制、关键参数快照和只读回放入口。

假后端会在状态中明确标出 `robot_id=FAKE`、`FAKE / NO ROBOT`，假地图/假存图不会生成真实地图文件。真实 Voxel-SLAM 存图请使用项目的仿真验收入口，不能把本手册的假后端结果当成真实导航或建图验收。

地图加载使用“地图与工位”页：点击“选择目录”选取机器人上的已保存 SLAM 会话，点击“校验并归档已保存地图”写入不可变地图目录；随后在“地图版本”中选择版本，点击“准备切换所选地图 / 进入人工转运等待”。同楼层会直接进入受管加载，跨楼层必须勾选人工换层并在停稳后确认。仿真实例会自动启用 `voxel_session_adapter`，将选中会话交给已有 Voxel-SLAM 定位插件；适配器验证未完成前，导航按钮保持禁用。

## 2. 构建与环境

先构建本次演示所需包，并使用同一安装前缀：

```bash
cd /home/yjh/WorkSpace/astribot_sdk_ros2
source /opt/ros/humble/setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/ws_robot/install/setup.bash
source /tmp/codex_transport_install/local_setup.bash
source /tmp/astribot-route-build/install/local_setup.bash
source /home/yjh/WorkSpace/astribot_sdk_ros2/tools/setup_mtc_humble.sh
export ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1
export ASTRIBOT_LOG_DIR=/tmp/astribot-p01-demo
colcon build --base-paths ws_robot/src/astribot_operator_backend ws_robot/src/astribot_operator_station \
  --build-base /tmp/astribot-route-build/build \
  --install-base /tmp/astribot-route-build/install \
  --packages-select astribot_operator_backend astribot_operator_station
source /tmp/astribot-route-build/install/local_setup.bash
```

同一域只能启动一套演示栈。启动前检查：

```bash
bash /tmp/rosops/whose_stack.sh /tmp/astribot-p01-demo
```

如果发现日志不属于本次演示，停止操作并先确认所有者。

## 3. 启动工作站

```bash
ros2 launch astribot_operator_station operator_fake.launch.xml \
  map_storage:=/tmp/astribot-p01-demo/catalog \
  map_import_root:=/tmp/astribot-p01-demo/maps
```

启动后会打开 RViz，加载 `WorkstationPanel`。顶部摘要应显示 `FAKE` 和 `OBSERVER`。假后端专用导航 Action 为 `/operator_fake/navigate_to_pose`，生产默认仍使用 `/navigate_to_pose`。

如果只需要查看接口而不打开窗口，可在另一终端执行：

```bash
ros2 topic echo /operator_backend/status --once
ros2 service list | grep -E 'operator_backend|loop_route|operator_fake'
ros2 action info /operator_fake/navigate_to_pose
```

## 4. P0 控制权演示

推荐直接运行仓库外的临时演示客户端（只调用控制服务，不发布速度）：

```bash
python3 /tmp/astribot-p01-demo/demo_p01.py
```

预期输出包含：

```text
acquire       accepted=true   reason=CONTROL.ACQUIRED
navigate      accepted=true   reason=COMMAND.ACCEPTED
navigate bad  accepted=false  reason=CONTROL.NOT_OWNER
release       accepted=true   reason=CONTROL.STOP_REQUESTED
```

`release` 后短时间显示 `STOP_UNCONFIRMED` 是安全语义：释放控制权的请求已经受理，但所属任务终态尚未确认，不能立即把控制权交给下一方。它不是急停，也不代表物理底盘已经停稳。

手工调用接口时，必须先从 `/operator_backend/status` 读 `boot_id`，先 `acquire` 获取 `lease_id`，后续命令携带同一租约。相同 `command_id` 的相同请求只返回原结果；相同 ID 改 payload 会返回 `REQUEST.CONFLICT`。

## 5. P1 导航与循环路线

在“导航与路线”页：

1. 点击“申请控制权”。
2. 点击“地图连续选点（拖动设置朝向）”，在地图上逐点点击；也可以用“添加坐标”后编辑 X/Y/Yaw。
3. 用“上移”“下移”“删除选中点”整理路线，点击“保存路线”保存 JSON。
4. 单点导航使用“导航到选中点（operator 优先级）”。取消使用“取消本后端导航，等待终态”。
5. 至少保留两个点，点击“开始导航”。未加载路线时先自动保存到 `routes/current_route.json`，随后按 `1→2→…→1` 循环，不会自动跳过失败点。
6. 点击“取消本后端循环路线”，等待状态变为 `CANCELED` 或 `PREEMPTED`。

命令行演示客户端会自动创建两个点并取消路线；预期最终路线状态为 `CANCELED`，并保留两个 waypoint。路线取消只作用于本面板/本租约创建的 route_id。

## 6. P1 探索建图交互

在“探索建图”页，按钮受状态和版本门控：

- “开始新建图会话”：需要地图事务和空闲证据；假后端默认不可用。
- “暂停探索”：暂停自己的探索目标，保留会话。
- “恢复当前探索”：必须携带最新 exploration boot/revision；版本变化会拒绝旧操作。
- “结束探索并保存部分地图”：确认后进入保存事务，不能恢复旧会话。
- “重试地图保存”：只有保存状态为 `FAILED` 时开放。

假演示客户端会执行 pause、cancel-save，并检查取消后 `can_cancel_save=false`、resume 被阻断、地图状态进入 `WAIT_STOP`。假后端的 detail 会注明 `FAKE: no files or robot actions`。

真实仿真存图另见：

```bash
ros2 launch astribot_sim_validation mapping_validation.launch.xml \
  map_name:=mapping_cancel_demo
```

该入口使用真实 Voxel-SLAM；必须先检查 clock、TF、地图推进、Nav2 生命周期和唯一包络保持所有者。取消后只有 `/mapping_session/status=SAVED`、PGM/YAML/关键帧和 manifest 校验通过才算成功。

## 7. 日志与回放

在“日志与回放”页：

1. “开始记录”启动 C++ `diagnostics_recorder`。
2. “关键参数快照”立即读取白名单参数；参数不可用时显示 `unknown`，不会用默认值冒充实效值。
3. “人工故障标记”写入业务事件。
4. “停止记录并落盘”后再打开回放。
5. 选择“打开只读 3D / 参数回放”，选择 incident 目录，使用暂停、播放、倍率和“跳转并重建”。

incident 目录格式：

```text
/tmp/astribot_incidents/incident_<时间>_<PID>/
├── session.log
├── events.jsonl
├── manifest.json
└── bag/*.db3
```

回放只发布地图、TF、路径、里程计、关节状态和 costmap 的显示话题，阻断 `/cmd_vel`、Action、Service 和关节轨迹命令；关键参数按采集序号关联，未知区间明确显示 `unknown`。

无实际 incident 时可先检查工具：

```bash
ros2 run astribot_operator_station inspect_incident \
  /absolute/path/to/incident/events.jsonl 100
ros2 run astribot_operator_station replay_viewer \
  /absolute/path/to/incident
```

## 8. 常见状态与处理

| 状态/原因码 | 含义 | 处理 |
|---|---|---|
| `OBSERVER` | 当前无控制权 | 申请控制权 |
| `HELD` | 当前租约有效 | 可提交所属操作 |
| `CONTROL.NOT_OWNER` | 租约缺失/过期/错误 | 重新读取 boot，重新 acquire；不要换 ID 重发未知请求 |
| `STOP_UNCONFIRMED` | 取消/释放终态未确认 | 等待状态终态；不要立即交接 |
| `STATE.STALE` | 后端状态过期或发布者不唯一 | 检查域、节点和发布者数量 |
| `EXPLORATION.NOT_READY` | 地图/定位/探索输入未就绪 | 查看 `readiness_detail`，不能当作建图完成 |
| `MAP.SESSION_NOT_IDLE` | 当前地图会话仍在保存/运行 | 等待 `IDLE` 或处理 `FAILED` 重试 |
| `REQUEST.CONFLICT` | 相同命令 ID 的 payload 不一致 | 使用原命令查询；不要静默换 ID |

## 9. 停止演示

先停止 RViz/launch，确认只清理本演示域：

```bash
bash /tmp/rosops/whose_stack.sh /tmp/astribot-p01-demo
# 确认日志均为本次演示后，再按 PID/启动时间停止本次进程组。
```

不要使用全局 `pkill`、不要删除共享 FastDDS 内存、不要停止其他 ROS 域的仿真。

## 10. 当前边界

已实现并演示：P0 租约/错误码/幂等门控，P1 面板状态、单点/循环导航接口、探索状态交互、C++ 记录和只读回放入口。

假后端不会证明真实导航、SLAM 覆盖率、机械臂执行、底盘制动或真机安全。真实仿真探索移动仍受 `fixed_v2` 包络保持会话接入门槛限制；完成该接入后，再做自然无前沿完成、运动中取消存图和多地图切换验收。

### 虚拟墙与禁区（2026-09-21）

新增“虚拟墙与禁区”页：墙、矩形、多边形绘制，停稳后保存，等待导航/探索应用；约束随 SLAM 存图清单与地图版本保存、恢复。详见 [操作说明](VIRTUAL_WALLS_AND_KEEP_OUT.md)。当前证据是隔离 C++ ROS / Qt / costmap 测试，现场 Gazebo 和真机链路尚待验收。
