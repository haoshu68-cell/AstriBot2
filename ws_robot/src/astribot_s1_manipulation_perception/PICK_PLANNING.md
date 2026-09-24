# M3 感知到 MTC 客户端

入口：`include/astribot_s1_manipulation_perception/pick_planning_client.hpp`，导出 CMake target `astribot_s1_manipulation_perception::pick_planning_client`。算法服务仍是既有 C++ GraspNet/CAD 配准；本库仅请求推理与完整 MTC PICK 规划。没有执行轨迹、夹爪指令、attach/detach 或全局 PlanningScene 写入。

## M1 调用边界

```cpp
// 在 M1 worker 线程调用；这个专属 node 不能同时加入 M1 executor。
astribot::perception_planning::PickPlanningClient client(dedicated_node);
// ready() 为 true 后再采集新快照；不把等待服务发现耗时转嫁给旧图像。
const auto outcome = client.plan(request, read_current_context, cancel_requested);
// 失败原因在 reason；终态不明时 terminal_confirmed=false，禁止复用 client。
// success 只代表规划完成，outcome.pick 为提案，仍须 M1 所属场景事务与执行准入。
```

`read_current_context` 必须返回 M1 **当前实际**实例绑定版本、相机/processing/clock epoch、标定/模型/场景/包络版本和全场景规范签名；不能始终返回最初 Request 的副本。每次等待以及规划返回都会检查。签名算法由 M1 与 MTC 共用，本库不建立第二套场景签名。保持原始采集戳与期限，最多采集后5秒；推理发送仍满足0.5秒准入。真实规划若超期明确拒绝，本轮合成协议测试不证明这一总时效预算可供真实 MTC 完成。

Request 还必须携带原始 `admission_deadline_steady` 与 `result_deadline_steady`。传感器接收时记录 ROS/steady 两个时钟，将 `capture+0.5s` 与 `min(capture+5s, fixture.valid_until)` 的剩余期限投影到 steady；不能在调用 `plan()` 时填写 `steady_now+5s`。`PlannedPick.result_deadline_steady` 原样交给M1继续约束。缺失或过期明确拒绝，ROS时钟暂停不会续期。旧手工云调用者必须补原接收锚点；头文件与静态库需要成套重建。

Request 中 `object_instance/identity_revision` 来自调用者的实例账本；`detection_id` 是当次检测/分割假设，不能互换。当前只接受 `segmentation_source=single_instance_fixture`、`visible_instances=1` 及调用方分离的目标云。YOLO 只有二维框及逐帧 ID，当前通用点云也没有目标实例分割；本库不宣称补全两者。

## 受控工位 RGB-D 请求源

`single_box_request_source.hpp` 导出 `astribot_s1_manipulation_perception::single_box_request_source`。它挂在 M1 已运行的节点/执行器上，由 M1 worker 调用；无需另建请求网关。所有运动、工位事实、实例和任务上下文仍由M1管理。

```cpp
namespace pp = astribot::perception_planning;
pp::SingleBoxRequestSource source(m1_node, capture_tf_buffer);
auto current = [&] { return source.bind_context(read_actual_owner_context()); };
// task_request: 当前完整scene、CAD几何/hash、map_grasp及规划参数。
// fixture: 权威station_id/单实例identity_revision、租期、工作台region及版本。
auto request = source.capture(task_request, fixture, current);
auto outcome = client.plan(request, current, cancel_requested);
```

`SingleBoxFixture` 必须包含独立工位坐标系、region_revision、AABB上下界和有效期；Context的 `station_region_revision` 与之相同。工作台尺寸/台面上方体积是允许的来源，目标实体pose、GT bbox、`warehouse_transfer.pick_xyz`不是允许的区域来源。先把真实光学XYZ按采集时刻TF变换到工位范围内，再在该范围做橙色连通分离；区域内多个候选拒绝。颜色只用于该受控工位，不能生成持续实例身份。无效深度的颜色像素若其视线穿过工作范围，仍保留在80%分母，绝不补造点。

输入复用现有 `rgbd_pointcloud_node` 的明确契约：`decimation=1`、每个像素占16字节XYZ/填充，缺深度保留NaN、height=1、width=图像宽×高。库核对完整布局、原始RGB/云/CameraInfo相同stamp/frame/尺寸和K投影回像素，任意扁平点云不能冒充该provider。只接受已注册的同光学帧RGB-D及D=0实际CameraInfo；不会忽略非零畸变。实际CameraInfo保存在Request，其规范内容hash加入 `camera_info_revision`；同源epoch/标定版本内内参变化锁存拒绝，不能悄悄换K。

订阅缓存各4帧，采集到使用最多250 ms（同时计入到达前延迟与到达后steady时间）；原stamp不重复接收续期。两个Health均需持续有效，处理epoch首帧范围必须包含该云。时钟回退清空缓存，必须有M1新的clock_epoch和新帧。立即查采集时刻TF，缺变换直接失败；ROS回调须持续运行，销毁source前须停稳其回调执行器。

真实有效目标点不足2048直接拒绝；超过12000只均匀选取原点，不填充。`config/single_box_projection.yaml` 是供场次所有者部署的独立CPU工位投影器候选，话题为 `/manipulation/single_box/head_rgbd/points` 和 `/perception/projection_health/single_box/head_rgbd`；它不改变导航降采样。两推理实例必须配置相同 `projection_health_topic`，否则既有服务会拒绝非空processing_epoch。当前只实现/验证库及合成ROS协议，尚未把当前真实工位数据送入模型。

两个推理结果必须与同一个云 Header、实例、源、模型hash、标定/场景/包络版本及有效期一致。既有结果消息没有processing_epoch字段，本库通过本次Action句柄和原始Request保持该绑定，且服务自身核对投影健康；不将camera source_epoch替代processing_epoch。

库把已知 CAD 几何按真实6D变换后写入**本次 MTC 请求的场景副本**。这只是感知提案，原场景版本不因此升级，也不表明全局场景已更新。M1 只有完成所属场景事务提交、独立读回及必要重验后，才能进入执行。PICK 包含 PREGRASP、GRASP_APPROACH、GRASP_CONFIRM、ATTACH_CONFIRM、LIFT、TRANSPORT_POSTURE；任何缺段、错序或返回context不匹配都不交付。MTC 自身负责 IK、碰撞和退路，客户端不以网络评分替代它。

取消或一端失败时，仅取消本客户端的具体Goal，最多等1秒墙钟收集终态。取消受理不算终态；无法确认时原始错误保留、无提案输出、client被禁止复用。仍未确认的远端推理/规划只能由其所有者继续追踪，不隐式cancel-all或重启服务。

## 推理部署

使用 `paired_inference.launch.py`，显式传入 `config`、`calibration_revision`、`planning_scene_revision`、`envelope_epoch`。这些值来自 M1 真实快照，必须非零；不复用历史评测固定1。

既有server共用单作业busy锁，每次Action又要求输入龄期≤0.5秒，单实例顺序6D→GraspNet会使第二次准入过期。最小适配是两个既有实例：一个仅配置pose worker，一个仅配置grasp worker；未用Action重映射，只有一个实际GPU worker。两请求同时发送同一快照，期限不续期。

server版本参数仍是启动只读：版本变化明确拒绝，由所属生命周期管理重建实例。本轮没有实现动态场景版本订阅，不能把更改请求字段当作后端已更新部署上下文。

## 抓取坐标与已知箱体

`Request.map_grasp` 必须来自注册适配，按本次候选及物体6D返回 `T_grasp_tcp`、真实物体夹持宽度及实际闭合q。GraspNet宽度包含开口裕量，depth影响指尖平面，不能用单常量变换和网络宽度直接下发MTC。

受限适配在 `known_box_grasp_mapping.hpp`，导出 `astribot_s1_manipulation_perception::known_box_grasp_mapping`。其边界是单一CAD箱体、面对齐候选、明确的当前左夹爪RobotModel与同MTC配置的GripperCommander。从CAD求宽度，复用Commander的宽度反解，再由实际RobotState与指垫碰撞mesh求远端平面映射。这里的几何注册是候选，不是摩擦/接触/夹持验收；完整接近、闭合扫掠和保持需后续所属仿真验证。

```cpp
request.map_grasp = [geometry = request.model_geometry, model, commander](
    const auto &candidate, const auto &object_pose) {
  return astribot::perception_planning::map_known_box_grasp(
      geometry, candidate, object_pose, model, *commander);
};
```

`commander` 由M1按MTC相同左夹爪配置和RobotModel创建并保持生命期；本适配只调用只读宽度反解/FK，绝不调用其运动方法。当前MTC默认预紧4 mm，不能为适配单独设置不同预紧。模型与注册表在一次调用期间不可变；注册revision必须覆盖模型、指垫几何与预紧。返回提案保存此次mapping、q及注册证据，供M1落盘与审查。

首期候选限制包括：三轴与箱体主轴面对齐（矩阵数值容差1e-6）、原生depth为10/20/30/40 mm、height为20 mm、CAD宽度小于实际全开间隙、候选开口能容纳CAD宽度、两侧面位于最终预紧覆盖范围、抽象手指与箱体有纵向/高度交叠。限制外的候选记录原因后尝试下一项；注册/模型缺失等配置错误直接失败。抽象交叠不证明真实指垫接触或完整闭合扫掠。

当前源码中的首期MTC固定左臂及 `astribot_torso_base`，库保持相同范围，不推测支持右臂/双臂。模型注册、实际预紧值、URDF/SRDF及指垫mesh必须随部署记录，变更后重新核验。
