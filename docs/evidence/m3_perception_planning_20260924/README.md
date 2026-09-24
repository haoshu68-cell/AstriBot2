# M3 C++ 感知到抓取规划：阶段交付

后续交付入口：`normal_box/README.md` 为正常箱体模型与方柱对称声明；`rgbd_source/README.md` 为10:46起的受控工位C++ RGB-D Request源。本页下列21例是初始客户端/几何阶段的历史证据，最新API和定向回归以rgbd_source为准。全包原有文件与本轮改动分类见FILE_SCOPE.md。

2026-09-24；09:50开始，按主线调度施工。**本轮接口、独立构建、离线几何及隔离ROS协议通过；整个M3模型驱动抓放尚未关闭。** 未启动Gazebo、GPU、真实推理worker或控制执行器，不涉及VLA/真机。没有修改MTC、transport_native、旧transport执行器、共享消息或共享install，没有提交共享工作树。

## 交付与接口

- [`pick_planning_client.hpp`](../../../ws_robot/src/astribot_s1_manipulation_perception/include/astribot_s1_manipulation_perception/pick_planning_client.hpp)：`PickPlanningClient::ready()/plan(Request,current,cancel)`；结果含完整MTC规划、物体6D、所选候选、原采集时间/期限、身份/epoch/版本/实际场景签名和注册映射证据。只返回提案。
- [`known_box_grasp_mapping.hpp`](../../../ws_robot/src/astribot_s1_manipulation_perception/include/astribot_s1_manipulation_perception/known_box_grasp_mapping.hpp)：按已知BOX尺寸和物体姿态求宽度，复用当前GripperCommander反解q，再用真实MoveIt RobotState/指垫mesh求逐候选depth相关TCP映射；不使用单常量猜测变换，不把模型带裕量开口当物体宽度。
- [`PICK_PLANNING.md`](../../../ws_robot/src/astribot_s1_manipulation_perception/PICK_PLANNING.md)：精确M1调用及部署边界。
- 独立安装：`runs/m3_perception_planning_20260924/install/local_setup.bash`。导出target为 `astribot_s1_manipulation_perception::pick_planning_client` 和 `astribot_s1_manipulation_perception::known_box_grasp_mapping`；仅后者需要夹爪几何组件。

目标识别映射为 `object_instance + identity_revision` → `model_id + pose_model_revision`，当次 `detection_id` 单独保留。当前YOLO只是二维框/逐帧假设，没有持久实例或通用目标分割；本轮只接收调用方提供的显式单实例fixture点云。估计物体姿态、模型原生抓取姿态、实际TCP是三种不同变换。

同一云并发进入两个既有服务实例；未用端点重映射，只有一个grasp worker。因为每个请求准入0.5秒且历史推理>1秒，单实例同帧串行不成立，未改写采集戳或放宽门槛。版本参数继续启动只读，必须用M1真实非零标定/场景/包络版本启动；变化后拒绝并由所属生命周期重建，不冒称已有动态版本接线。

## 实际检查

| 检查 | 结果 | 证据 |
|---|---|---|
| 当前消息+transport接口+物体配准依赖及本包独立构建 | PASS，包级串行；实际命令允许-j28，峰值未采集 | build_contract_red.log（依赖构建）、build_final.log |
| 纯C++上下文/几何/完整计划/终态 | 10/10 | contract_final.xml、contract_final.log |
| 合成ROS Action协议（真实客户端） | 6/6 | protocol_final.xml、protocol_final.log；初轮归属在protocol_session.json |
| 当前MoveIt/Commander的BOX映射几何 | 5/5 | box_final.xml、box_final.log、registration_inputs.json |
| 双实例launch参数展开 | PASS，未启动任何节点 | paired_launch.json |
| 安装导出库的外部CMake消费者 | 构建与调用检查通过 | consumer_configure.log、consumer_build.log、consumer_result.json |

协议测试经调度明确授权在ROS_DOMAIN_ID=100、ROS_LOCALHOST_ONLY=1单进程运行；此前只读环境盘点未发现该domain进程，见domain_100_preflight.json。测试服务仅返回合成消息，不产生实际模型证据。覆盖一端失败时另一端取消、保留原错误、同时取消等待终态、MTC首候选无解后下一项、规划时场景签名变化、取消受理但终态缺失。独立审查指出ready future的UNKNOWN不能算终态，现已把“结果可读”和“确证终态”分开：UNKNOWN立即暴露且有界清理，仍未决则禁复用。

实际几何样本：箱体真实宽0.060 m，默认预紧0.004 m，既有Commander得 `q=0.466377314821 rad`；全开间隙 `0.0975270967228 m`。depth=0.01 m时TCP映射平移 `(-0.0564122767507,-0.006,0.000000374667) m`。同一候选depth改为0.04 m时沿approach平移增加30 mm；改变CAD宽度时q/FK随之改变。mesh实际路径/hash与独立审查一致，记录在registration_inputs.json。

几何测试使用当前源码展开的**传感器关闭模型**及当前SRDF，只为夹爪FK；因此SRDF对已关闭传感器link有已记录警告。原URDF中四个连杆只有visual无collision的警告也保留，未借本次修改补造碰撞体。测试不是全机器人碰撞模型验收。复用了已安装的GripperCommander组件并记录.so/hash与一致的头文件；未宣称本轮重建该组件源码。

## 必须保留的集成前置

1. 旧registry只含三box复合体`asymmetric_union`；受限适配只接受单BOX。现已在 [normal_box](normal_box/README.md) 按正常场景实际60×60×120 mm生成CAD/visibility/registry/闭合mesh并通过离线资产检查；8元素方柱声明的C++构建/测试状态单独记录，尚未实际模型调用，不能偷换为旧复合体。
2. `map_grasp`须绑定当前RobotModel、SRDF、mesh、预紧与模型输出语义。本轮是面对齐单箱体几何候选，真实夹爪闭合全程扫掠、双侧接触/保持、IK与完整规划仍需所属场次验证；运动学附着也不能代替摩擦抓持。
3. MTC请求scene是感知更新后的**提案副本**。M1须由所属场景事务提交、独立读回及重验，才进入执行；原scene_revision不能自动充当新事实版本。current回调必须持续返回实际全场景签名，不能返回Request常量副本。
4. 总期限仍为原采集后最多5秒，调用前龄期≤0.5秒。真实双推理+MTC能否在预算内结束本轮未测，不能以合成协议耗时证明。
5. C++常驻实例分割/身份关联生产者、当前参考相机配置重采、M1/M2正式执行接线及模型驱动放置仍待集成；本轮不称通用YOLO端到端或整条M3通过。

## 复现

先加载Humble及已验证项目underlay；独立构建用 `colcon build --base-paths ws_robot/src --packages-select astribot_object_pose_core astribot_perception_msgs astribot_transport_msgs astribot_s1_manipulation_perception --executor sequential`，设置 `MAKEFLAGS=-j1`（并核对 command.log），或直接 `cmake --build --parallel 1`，使用本任务独立build/install/log目录。不要写共享install。

纯契约二进制为独立build目录内的 `astribot_s1_manipulation_perception/pick_planning_test`。协议测试须先向总调度确认domain100资源仍归本任务，再用对应环境运行 `pick_planning_protocol_test`。BOX测试另需 `M3_TEST_URDF` 指向记录的registration_robot.urdf、`M3_TEST_SRDF`指向当前实际SRDF；运行`known_box_grasp_mapping_test`。这些入口不启动Gazebo和模型。

保留红阶段：首次构建缺少本任务object_pose_core安装，补齐显式依赖；首轮契约缺实现导致3项失败；BOX缺实现导致3项失败。xacro首次缺显式robot_name、夹爪导出依赖缺control_msgs/OMPL的构建错误也保留。最终通过不覆盖这些历史错误。

## 正常箱体准备补充

10:30总调度批准限定跨包修改：object_pose_core 的 CLI/精确语义测试/CMake测试接线/README，及本包服务的 registry 白名单。新增 `square_prism_z` 复用现有 Options 对称旋转，不改配准算法与门槛。10:42经调度临时释放完成两包串行构建（22.8 s）；随后经限定授权修正测试读取器，4/4 C++检查通过，已安装CLI另有4例完整JSON解析边界验证。原读取器不支持null的失败日志保留。基础21项检查与新增检查分开留证；实际模型与抓放仍未验收。M2 first_scene02 的箱体 SDF/fullScene 尺寸质量已离线对齐，明确其static/KinematicPayload属性，未把回执位姿送入推理。

并行度收据更正：10:50只读核对发现此前colcon实际追加`-- -j28 -l28`，覆盖预期的单worker限制；因此上述构建仅能声明包级顺序，实际并发编译器峰值未知，不可用作独占性能证据。完整命令审计见`build_parallelism_audit.json`。功能构建/测试结果不变；最终单测试目标直接`--parallel 1`，后续采用显式约束并实核。
