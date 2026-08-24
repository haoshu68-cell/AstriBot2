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
endif()

ament_package()
