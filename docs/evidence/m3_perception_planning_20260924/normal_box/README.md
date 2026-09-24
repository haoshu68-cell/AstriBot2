# 正常箱体模型准备

来源是当前 `warehouse_transfer.json` 中的 `transport_box_01`：60×60×120 mm、0.2 kg。
`prepare.py` 复用现有 CAD 表面采样器生成 4000 个带外法线样本、中心位于原点的闭合 OBJ 网格、单 BOX 可见性模型、registry 和推理参数模板。尺寸来自场景配置，没有将 spawn 位姿、Gazebo 真值或历史抓姿写入推理输入。

模型 ID 为 `transport_box_60x60x120`，实例 ID 与模型 ID 分离。`geometry.json` 给出 M1 构造注册 CollisionObject 所需尺寸及模型到形状的恒等变换；实例版本仍由任务所有者生成。点云和可见性文件各自 SHA256 以冒号拼接，遵循现有服务 `pose_model_revision` 规则；完整输入和资产 hash 在 `provenance.json`。

该物体是方柱，对称群含 8 个正旋转（包含 identity）。`square_prism_z` 使用原配准 Options 的 7 个非 identity 旋转，要求 x=y、z 不同且中心在原点；cube、非方柱及 CAD 外包尺寸不匹配被拒绝。输出仍是对称等价姿态，不能宣称识别了唯一的箱体朝向。没有改变模型尺寸、配准算法、覆盖率或时间门槛。

`verify_assets.py` 独立检查 CAD 范围/法线、8 种旋转后样本集合不变、网格闭合/正体积、场景尺寸和文件绑定。它不采集相机、不运行配准、不证明真实模型推理或抓取成功。CLI 语义测试位于 object_pose_core 的 `square_prism_cli_tests`，构建和运行状态另记在本目录。

`inference.template.yaml` 是部署准备模板，使用本任务独立安装的 pose worker；须先完成本次含 square_prism_z 的重建，不能混入旧接口 overlay。首站选择规范六相机中的 head_rgbd 原始 640×360@20 Hz，frame 来自 2026-09-23 的原始流回执；本次实际流、对应 CameraHealth、TF 和 epoch 仍须由 M2/M5 核对。模板没有继承旧头部关节角度。

标定/场景/包络版本在模板中保留 0；`paired_inference.launch.py` 要求显式传入当前真实非零版本。不能把测试用 1 填入后当作事实。版本变化后既有服务仍需由所有者重建。运行还需要单实例分离点云生产者、采集时刻 TF、当前 source/processing/clock epoch、M1 场景提案提交及独立读回。原始 RGB-D topic 和历史 frame 是来源记录，不能代替这些前置。

M2 的 first_scene02 SDF/registry/full-scene 回执已离线核对：尺寸、0.2 kg质量和对角惯量与本模型一致，hash 见 `m2_fixture_binding.json`。SDF 明确为 `static=true` 并使用 KinematicPayload；虽然含 collision/inertial，仍不能称动态自由物体或摩擦夹持验证。其 scene 位姿没有进入感知输入；已结束场次的 readback 也不能用作新任务的实时场景事实。

验证结果：两包独立串行构建通过，`semantic_tests_final.xml` 中 4/4 C++ 语义检查通过；`cli_boundaries.json` 对已安装 CLI 的4个边界完整解析 JSON 并核对 exit2/pose null，合法声明进入原空场景拒绝，cube/非方柱/CAD错配在声明边界拒绝。初轮 `semantic_tests.log/xml` 中的 1 FAIL 为 OpenCV JSON 读取器不支持现有合法 `null`；原输出已核实，修正测试读取方式后通过，失败记录保留。没有运行真实配准/GraspNet或抓放，不把这些用例计为模型成功率。
