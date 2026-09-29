# M3：GraspNet 原生抓取坐标到左夹爪 TCP 的只读几何审阅

审阅时间：2026-09-24，版本范围：本文件列出的当前源码及哈希。root 已与 M3 预留本文件；本次只新增本证据文件，没有修改运行代码，没有启动或查询 ROS/Gazebo，没有操作机器人。

**结论：轴方向旋转可以从当前模型确定；保持原生 depth 语义的平移不能无条件采用单个固定常量。** 下面给出一个可用于限定仿真夹具的“指垫远端平面对齐”注册候选：旋转固定，平移依赖候选 `depth` 和实际闭合宽度对应的夹爪关节角。它已有源码与离线 FK 依据，但尚未经过 MoveIt 完整接触几何和仿真验收，不能标记为已验收注册，更不是物理标定。

## 1. 源码和输出语义

本地官方 GraspNet baseline 位于 `/home/yjh/.cache/astribot/graspnet/rebuild_cpu_source/baseline`，Git HEAD 为 `280c215129f759ed8649cb4e89fc5dfee55f4f80`。本次核对 `models/graspnet.py`、`utils/loss_utils.py`、`utils/collision_detector.py` 相对该 HEAD 无修改；未使用网络或凭图片推断轴向。

- `models/graspnet.py:77-133`：输出每行依次为 score、width、height、depth、按行展开的 3×3 旋转、translation、object id。translation 来自 `fp2_xyz` 采样抓取点，不能按名称当物体质心或机器人 TCP。
- `models/graspnet.py:85-88,103-104,131`：approach 为 `-grasp_top_view_xyz`；width 为网络预测的 **1.2 倍**并裁剪到 0.1 m；depth 为 0.01、0.02、0.03、0.04 m；height 固定为 0.02 m。
- `utils/loss_utils.py:68-96`：旋转的第一列是 approach，因此原生抓取系 G 的 **+X 为伸入/接近方向**。
- `utils/collision_detector.py:70-93`：点先减 translation 再右乘 R 转到 G；两指内侧为 `y_G=±width/2`，高度为 `z_G∈[-height/2,+height/2]`，指体纵向为 `x_G∈[depth-0.06,depth]`。因此 **G 的 Y 为开合方向，Z 为高度方向，指尖远端平面为 x_G=depth**。G 原点不是固定掌心或固定指尖。
- 仓库 `tools/vision/graspnet_prepare.py:81-94,120-121` 原样导出 `pred_decode`；`ws_robot/src/astribot_graspnet_runtime/src/graspnet_worker.cpp:55-65` 只推理并写 Nx17，没有执行 TCP 转换；`ws_robot/src/astribot_s1_manipulation_perception/src/manipulation_perception_server.cpp:480-499` 原样填写姿态及 width/height/depth。
- `ws_robot/src/astribot_perception_msgs/msg/GraspCandidate.msg:7-9` 明文约定该姿态为模型原生 grasp frame，需要注册后才可规划。

v2 运行记录 `docs/evidence/grasp_pose_sim_20260921/graspnet/v2/runtime_paths_v2.json` 给出 CPU 模型 SHA256 `82a2d7208b38333b10fdb025ed6f4efab0f661b914f32da563c142c5048535c7`、CUDA 模型 SHA256 `5acc1ff77600a6e1b8a5a814b363956c9e12bf74abf89d85b94ba6f66cfd1b7c`；本次没有重新加载模型，注册时应绑定实际运行模型 revision，不能仅凭文件名绑定。本报告使用源码约定而非重新推理结果。

## 2. 当前机器人和 MTC 的真实目标

记 F 为 `astribot_arm_left_tool_link`（与 link_7 重合）、T 为 `astribot_arm_left_tcp_link`、H 为 `astribot_gripper_left_base`，列向量变换 `p_A = T_A_B p_B`。

当前 Xacro 明确给出：

```text
T_F_T = [ I, (0,-0.150,0) ]
T_F_H = [ Rx(alpha), (0.006,-0.048,0) ], alpha=1.5708 rad
T_T_H = [ Rx(alpha), (0.006, 0.102,0) ]
```

来源：`ws_robot/src/astribot_s1_description/urdf/astribot_s1_arm.xacro:344-357`、`astribot_s1_gripper.xacro:301-304`；厂商 `astribot_config/robot_config/astribot_s1/astribot_arm_left.yaml:11` 也给出 TCP 的 -0.15 m 偏置。

夹爪开合轴为 H 的 X，也就是 T 的 X；伸出方向为 H 的 +Z，即 T 中 `(0,-sin(alpha),cos(alpha))`，近似 -Y_T。不是 +Z_T。H 原点相对 TCP 还有 **+6 mm 的 X 偏心**。

SRDF `ws_robot/src/astribot_s1_moveit_config/config/astribot_s1.srdf:43-44,106-107` 区分 arm tip=TCP 与 eef 安装 parent=tool；MTC `ws_robot/src/astribot_s1_transport_mtc/src/mtc_planner.cpp:151-171,201-208,221,235` 要求：

- 所有目标在 `astribot_torso_base` 中；
- IK frame 和 MoveTo 目标均为 `astribot_arm_left_tcp_link`；
- `grasp_width_m` 交给当前 `GripperCommander::graspAngleForWidth`，不是直接下发张口。

因此正确的整条姿态链为：

```text
T_B_T(candidate) = T_B_C(capture_stamp) * T_C_G(candidate) * T_G_T(candidate, robot_model)
B = astribot_torso_base
C = 此候选原始 optical frame
```

不能再多乘一次 `T_F_T`；`T_G_T` 已经针对 TCP。`T_B_C` 必须使用捕获时刻、对应标定版本与本任务场景；对象 6D 位姿用于独立的物体几何与接触验证，不能替代 `T_C_G`。

## 3. 可以确定的轴旋转

采用一种明确的手指对应关系：G 的 +Y 对应 H/T 的 +X（正侧指垫），G 的 +X 对应 H 的 +Z，G 的 +Z 对应 H 的 +Y。该关系保留右手系。

令 s=sin(alpha)、c=cos(alpha)，则：

```text
R_T_G = [ H_z_in_T, H_x_in_T, H_y_in_T ]
R_G_T = transpose(R_T_G)
      = [[0, -s, c],
         [1,  0, 0],
         [0,  c, s]]
      = [[0, -0.9999999999932537, -0.0000036732051033],
         [1,  0,                   0                  ],
         [0, -0.0000036732051033,  0.9999999999932537]]
```

若把安装角理想化为 pi/2，则为 `Rz(+pi/2)`；实现应从当前 RobotModel/安装变换取值，而不是把 1.5708 和 pi/2 混用。审阅已核对 `det(R)=1`，`R_G_T * H_z_in_T = +X_G`，`R_G_T * H_x_in_T = +Y_G`。

另一个绕 G 的 X 旋转 pi、交换两指的分支也是平行夹爪可能的握姿对称，但不是上述同一分支。若首期只接受一个分支，固定并记录即可；若试另一个分支，须独立求 IK 和全身碰撞，不应默默交换。

## 4. 为什么平移依赖 width/depth

当前主动关节 q 为 L1，范围 `[0,0.93]`；L1 绕 -Y_H 转 q，L11 绕 +Y 转 q 并有 -0.03 rad 预角，故 L11 的净姿态恒为 Ry(-0.03)，R11 恒为 Ry(+0.03)，但两指位置随 q 移动。

来自 `astribot_s1_gripper.xacro:319-328,354-362,399-425` 的常数：

```text
a = 0.04125 m
b = 0.036379080527138 m
x_origin(q) = 0.0145 + a*cos(q) - b*sin(q)
z_origin(q) = 0.0511 + a*sin(q) + b*cos(q)
p_H_L11 = (+x_origin,0,z_origin)
p_H_R11 = (-x_origin,0,z_origin)
```

当前两个碰撞 mesh 的旋转后 distal support 均为 `z_mesh_max=0.06627016723755787 m`；内侧 X support 分别为 `-0.0069864518321704` 和 `+0.0069864518321704` m。可复核的当前几何关系为：

```text
gap(q) = 2*x_origin(q) - 2*0.0069864518321704
z_tip(q) = z_origin(q) + 0.06627016723755787
p_T_tip_mid(q) = (0.006, 0.102-s*z_tip(q), c*z_tip(q))
```

`tip_mid` 在此定义为两指 distal support 平面的横向中线；这是清楚定义的模型几何参考点，**不是已经测得的实际接触中心**。两个不同简化 mesh 的对称 support 恰好相同，不代表所有三角面或所有接触都精确镜像。

| q(rad) | 当前指垫包络间隙(m) | distal 中线相对 TCP 的 Y(m) |
|---:|---:|---:|
| 0 | 0.097527096 | -0.051749248 |
| 0.2 | 0.081427774 | -0.059219198 |
| 0.5 | 0.052545537 | -0.067072117 |
| 0.75 | 0.025796645 | -0.070105934 |
| 0.93 | 0.006024007 | -0.070185640 |

开合全行程的远端前后变化约 **18.44 mm**。G 的远端平面又随 candidate.depth 改变，1 cm 到 4 cm 共差30 mm。因此把一个注册样本的常量平移用于所有 width/depth 会改变插入深度。

## 5. 首期可审查的“远端平面对齐”注册候选

限定当前模型、左夹爪、固定手指分支，以及经物体几何验证的候选。把本机 `tip_mid(q*)` 对齐到原生抓取的 `(depth,0,0)`：

```text
t_G_T(q*,d) = (d,0,0) - R_G_T * p_T_tip_mid(q*)
           = (d + 0.102*sin(alpha) - z_tip(q*),
              -0.006,
              -0.102*cos(alpha))
T_G_T(q*,d) = [R_G_T,t_G_T(q*,d)]
```

这里 `d=candidate.gripper_depth_m`。q* 的定义必须在注册版本内固定，例如“当前 MTC 最终命令间隙对应 q”，不能有时使用全开 q=0、有时使用接触 q。该选择把最终命令姿态的 distal 平面对齐；全开→闭合过程的前后扫掠仍必须由实际夹爪几何检查，不能只检查两个端点。

对于当前 MTC，最小一致计算顺序是：

1. 从**已知 box/CAD 和物体姿态**取得本候选闭合方向的物体接触宽度 `w_object`。首期可限制 G 的 Y 与 box 一对相对面法向平行，排除斜夹、复合物体及不明确接触。精确平行时宽度即该轴尺寸；一般 box 的投影宽度为 `sum_i(size_i*abs((R_G_O)[Y,i]))`，但一般投影宽度不自动证明正确接触，应留到后续验收。
2. 复用与 MTC 一致的 `GripperCommander`、RobotModel、SRDF 命名状态与 preload（当前默认0.004 m），调用 `graspAngleForWidth(w_object)` 得到 q*。当前实现以61个角度样本建表，目标间隙为 `w_object-preload`，再反查线性插值；不要另写不同的近似闭合规则。
3. 将 q* 放入同一个 RobotState并更新 mimic/FK；从当前 L11/R11 碰撞几何计算 `p_T_tip_mid(q*)`。上面的封闭公式可作为本版本离线对照，不是新增另一套永久几何源。
4. 用候选 d 得出 `T_G_T`，再计算 `T_B_T`。`pre_target` 沿模型 approach 的反方向平移：`p_B_pre=p_B_target-approach_m*(R_B_C*R_C_G*[1,0,0])`，姿态保持；不能默认沿 TCP 的 Z 退让。lift 用已定义的 B/世界方向，不能混成 grasp 高度轴。
5. 将**物体接触宽度**传给 MTC `grasp_width_m`，保持 q* 与实际规划一致；另检查候选/实际全开间隙足够接近、目标处双侧指垫确实能覆盖物体，以及闭合扫掠无非许可碰撞。

数值例，仅说明公式、不是抓取接受：`w_object=0.060 m`、默认 preload=0.004 m，对照61点表得 `q*=0.466377311 rad`；当 d=0.01 m，`t_G_T=(-0.0564122794,-0.006,+0.000000374667)` m；当 d=0.04 m，X 平移改为 -0.0264122794 m。`w_object=0.100 m` 的反查虽然可能给出 q，但全开间隙约0.097527 m，小于物体厚度，仍不能接近插入，必须单独拒绝或由真实几何碰撞检查拒绝。

**模型 width 的语义不能偷换。** GraspNet width 已乘1.2，不能直接把它当 `w_object` 再仅减4 mm就声称夹住。简单除1.2也不是物体宽度真值：模型预测、裁剪和当前物体几何都有各自含义。首期已知box可用已注册尺寸和物体姿态建立独立宽度依据，candidate.width保留为候选开口/质量检查数据。

## 6. M3 最小接口建议

本次核对的 `pick_planning_client.hpp:37-46` 已有 `base_from_camera`、`model_geometry`、`geometry_revision`、单个 `grasp_from_tcp`、registration revision/evidence、width上下限。保持已有场景、身份、时效接口，不再添加第二套执行入口。

建议只改变注册所需的最小语义：

- 把“一个 Request 固定 `grasp_from_tcp` 覆盖整个候选数组”改成**按候选解析的注册结果**。可由C++注册解析器返回 `{grasp_from_tcp, object_contact_width_m, commanded_gripper_q}`，`planning_goal` 消费该已绑定candidate id/model revision/robot model revision的结果。注册解析器使用既有RobotModel与GripperCommander；不要由调用方手填宽度、q和变换三个互相矛盾的值。
- Request保留 `grasp_registration_revision/evidence`，明确模式为当前“distal_plane_at_commanded_gap”，并绑定URDF/SRDF/指垫碰撞模型与preload版本。若已有模型身份机制足够，复用它，不重复造消息。
- 首期已知box的尺寸/位姿已经在 `model_geometry`，优先由此按当前candidate算接触宽度；只在已有geometry不足以表达时增加明确 `object_contact_width_m` 及来源，且须按候选绑定。不要拿 `candidate.gripper_width_m` 填这个字段。
- candidate已有depth，无需新增另一份depth；在规划请求/证据记录里保留本次选用的d、q、变换与注册身份。若M3暂时只支持固定变换，则限于**已明确固定的width/q和depth单一夹具**并拒绝其他值；不能把该受限结果注册为通用GraspNet变换。

实现选择可以更小，但必须保证 q* 与现有 MTC 的实际闭合计算一致。`GripperCommander` 构造本身需要节点，但本次审阅没有实例化或启动节点；上述“复用”是后续实现职责，不是本次已完成的运行接线。

## 7. 已验证、尚未知与最小后续验收

本次已做离线解析：

- 从当前XML原点、轴、mimic比例和OBJ顶点构造FK；与上面解析式一致。
- 得出上表开口及distal支持位置，复现现有61点宽度表的插值口径（没有调用实际MoveIt运行库）。
- 验证轴映射为右手旋转，并对61个q×4种d核对 `R_G_T*p_T_tip_mid+t_G_T=(d,0,0)`。这是所选注册定义的代数一致性检查，不能计为244个抓取或接触通过。

尚未知/未验收：

1. 指垫真正有效接触面的范围、变形和摩擦；仅有当前简化碰撞mesh，不能把distal support称实际接触中心。
2. 本机器人指垫几何不等于GraspNet抽象平行夹爪：模型finger_length=0.06m、height=0.02m，而当前指垫mesh旋转后纵向跨度约0.071253m、厚度方向总宽0.028m；一个刚体变换不可能使全部体积一致，必须用机器人真实几何重新检查。
3. 厂商模型之间存在约2.6mm基座/7mm销轴定义差别，记录在 `astribot_s1_gripper.xacro:60-71`；本次只绑定当前仿真模型，不能声称物理TCP/指尖标定。
4. 抓取动作最终是否双侧接触且可靠保持；MTC能够规划、预测attach成功或Gazebo运动学附着成功均不能单独回答。

最小离线验收：

1. 用**实际展开URDF+SRDF和MoveIt RobotState**复核 q=0、受限box对应q、q=0.93的mimic/FK及张口表，与本表对照并绑定模型哈希。
2. 使用独立构造的box、已知物体6D位姿及非单位相机旋转/平移验证整条 `T_B_C*T_C_G*T_G_T`；同一candidate改d 0.01→0.04，TCP必须沿approach改变30mm；宽度改变时必须使用新q/FK。
3. 在PlanningScene检查接近、闭合全过程、抬升；先检查禁止接触，闭合阶段只允许本物体与两指垫。独立量测物体在两指内侧之间及纵向接触覆盖，不以允许碰撞矩阵或attach返回成功代替。
4. 设置至少一个明确反例：交换矩阵乘法顺序、误用identity旋转、忽略6mm偏心或固定d变换时应破坏几何断言；这是验证器可识别坐标错误的证据，不必为每个字符串造测试。

最小仿真验收（本次未运行）：在所属隔离仿真使用已注册box和真实相机点云→真实GraspNet候选→上述注册→MTC规划；先完成规划且检查实际仿真模型/关节/指垫位置，再由M1既有任务事务执行接近、闭合、附着读回、短距抬升、放回。记录candidate原生姿态/width/depth、T_B_C采样时刻、注册身份、q命令与实测q、两侧接触几何、非许可碰撞、物体保持和最终释放。首期单夹具通过只开放该夹具条件，不自动开放任意width/depth/物体，也不涉及真机。

## 8. 本次机器人源码身份

| 文件 | SHA256 |
|---|---|
| `ws_robot/src/astribot_s1_description/urdf/astribot_s1_gripper.xacro` | `7893da976c5ca70967c8d834474cdce230f148c7bc05989694919b1a04db651e` |
| `ws_robot/src/astribot_s1_description/urdf/astribot_s1_arm.xacro` | `7159be13feaf9bcf471a6d26907ee880bc38c6b1b2e5ba669aee8b2312625653` |
| `ws_robot/src/astribot_s1_moveit_config/config/astribot_s1.srdf` | `1a272c8af5508d0c78c40685bc1afcd2c38bbe3678bd4d3e69bac829647c668c` |
| `astribot_gripper_L11_Link_collision_mesh.obj` | `6fa7d39b316b243a50712f09a1f80f669cdfb4633215fd064cf0b3b31ac2da95` |
| `astribot_gripper_R11_Link_collision_mesh.obj` | `2eaeed10fc374830f1bed3c0ed0c59dabfbbc594d18b1ffa949a53c0e8957929` |

两个mesh路径前缀均为 `astribot_config/robot_config/astribot_s1/model/meshes/s1_gripper/obj/`。
