cmake_minimum_required(VERSION 3.8)
project(astribot_s1_description)

find_package(ament_cmake REQUIRED)

# 本包自身的 urdf/xacro、控制器碰撞记录、rviz 配置，安装到 share/${PROJECT_NAME}/ 下，
# 运行时统一通过 FindPackageShare('astribot_s1_description') 动态定位，不出现绝对路径。
install(DIRECTORY
  urdf
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------------------
# mesh 资源复用说明（本方案唯一使用"源码树相对路径"而非 FindPackageShare 的地方）：
#
# astribot_config 目前不是一个 ament 包（没有 package.xml/CMakeLists.txt），无法用
# FindPackageShare 在运行时定位。为了不修改/不复制原始仓库内容到本包源码里（保持“只读引用”），
# 这里在 *构建脚本*（CMakeLists.txt，只在 colcon build 时执行一次）里用相对于本文件的路径
# 找到 astribot_config 下已有的 mesh 目录，install 到本包的 share/astribot_s1_description/meshes/
# 下 —— 运行时 xacro/launch 代码里引用的仍然是 package://astribot_s1_description/meshes/...，
# 不会出现任何绝对路径。
#
# 目录关系（相对本 CMakeLists.txt）：
#   ws_robot/src/astribot_s1_description/CMakeLists.txt
#   ../../../astribot_config/robot_config/astribot_s1/meshes   <- 3 层上溯到仓库根目录
# ---------------------------------------------------------------------------------------
set(ASTRIBOT_S1_MESH_SRC_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../astribot_config/robot_config/astribot_s1/meshes"
)

if(NOT EXISTS "${ASTRIBOT_S1_MESH_SRC_DIR}")
  # 潜在报错点：如果这个包被挪到别的工作空间/仓库结构变了，这里会先给出清晰报错，
  # 而不是让 colcon build 静默跳过 mesh、直到 RViz/Gazebo 里模型变成一堆方块才发现问题。
  message(FATAL_ERROR
    "astribot_s1_description: 找不到 mesh 源目录: ${ASTRIBOT_S1_MESH_SRC_DIR}\n"
    "请确认本包位于 <astribot_sdk_ros2>/ws_robot/src/astribot_s1_description，"
    "且 <astribot_sdk_ros2>/astribot_config/robot_config/astribot_s1/meshes 存在。"
  )
endif()

install(DIRECTORY
  ${ASTRIBOT_S1_MESH_SRC_DIR}/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# ---------------------------------------------------------------------------------------
# 夹爪 mesh：源目录和上面那批不是同一个
#
# 上面的 meshes/ 目录只有 s1_arm / s1_head / s1_torso 三个子目录，**没有 s1_gripper**。
# 夹爪的 mesh 只存在于 model/meshes/ 下（那是个超集：多了 s1_gripper 和各目录的 obj/ 子目录）。
# 装夹爪 mesh 时必须从 model/meshes/s1_gripper 取，不能指望上面那条 install。
#
# 这里刻意**不**把上面的源目录整体换成 model/meshes（那样一条 install 就够了）：
# model/meshes 里每个 STL 旁边还有一份 .obj，s1_arm 的 obj 单文件就上兆，
# 全量装进 share 会让安装体积翻好几倍，而 obj 里本包只用夹爪那几个碰撞 mesh。
# 所以按需只加一条 s1_gripper。
#
# 夹爪用到两类文件：
#   *.stl                        —— visual
#   obj/*_collision_mesh.obj     —— collision（厂商自己的碰撞简化网格）
# 两类都在 s1_gripper 及其 obj/ 子目录下，一条 install 全部带上。
# ---------------------------------------------------------------------------------------
set(ASTRIBOT_S1_GRIPPER_MESH_SRC_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../astribot_config/robot_config/astribot_s1/model/meshes/s1_gripper"
)

if(NOT EXISTS "${ASTRIBOT_S1_GRIPPER_MESH_SRC_DIR}")
  # 同上：宁可在 build 阶段清晰报错，也不要让夹爪 mesh 静默缺失 ——
  # 缺 visual 只是 RViz 里看不见，缺 collision 则是**碰撞检测悄悄少算一块体积**，
  # 那是会撞机器的，必须让它在这里就停下。
  message(FATAL_ERROR
    "astribot_s1_description: 找不到夹爪 mesh 源目录: ${ASTRIBOT_S1_GRIPPER_MESH_SRC_DIR}\n"
    "夹爪 mesh 在 model/meshes/s1_gripper（不是 meshes/），请确认 astribot_config 完整。"
  )
endif()

install(DIRECTORY
  ${ASTRIBOT_S1_GRIPPER_MESH_SRC_DIR}
  DESTINATION share/${PROJECT_NAME}/meshes
)

# ---------------------------------------------------------------------------------------
# 关节限位一致性测试
#
# 把"限位以厂商 per-part 模型为准"这条规则机器化。厂商 ship 了四份互不一致的模型，
# 本包早期抄的是 whole_body_with_wheel.urdf —— 一份 SDK 运行时并不加载的模型，
# 导致躯干速度限位比真机宽 3.3 倍（方向不安全）、左臂 joint_3 白丢 1.7 rad 行程。
# 详见 docs/sim_real_alignment.md 第 0 节与 test/test_joint_limits_parity.py 的模块注释。
#
# 注意这是 `colcon test` 阶段的测试，不是 `colcon build` 阶段的检查 ——
# 因为它要展开 xacro，而 xacro 的 $(find astribot_s1_description) 需要本包先装好。
# ---------------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_joint_limits_parity
    test/test_joint_limits_parity.py
    TIMEOUT 300
  )
  # 夹爪模型一致性。厂商没有任何单一文件同时给出夹爪的几何和耦合关系，
  # 本包混用 MJCF(几何/耦合) + whole_body_with_gripper.sdf(碰撞 mesh/基座装法)，
  # 所以这里校验的是"能证明没抄错"的性质（TCP 偏置、平行夹爪不变量、两源交叉校验、
  # 腕部质量守恒、夹爪确实进了碰撞模型），不是逐字节比对。
  # 详见 test/test_gripper_model.py 与 urdf/astribot_s1_gripper.xacro 的文件头。
  ament_add_pytest_test(test_gripper_model
    test/test_gripper_model.py
    TIMEOUT 300
  )
endif()

ament_package()
