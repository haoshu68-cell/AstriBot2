cmake_minimum_required(VERSION 3.16)
project(astribot_s1_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 任务要求"无 warning"。开满常用告警，把问题在编译期暴露出来，
# 而不是等到运行时变成奇怪的数值行为。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# OMPL：BIT* / Informed RRT* 的头文件与符号都来自这里。
# moveit_planners_ompl 提供 ompl_interface 的公开头（PlanningContextManager 等），
# ompl 本体提供 og::BITstar / og::InformedRRTstar 的实现。
find_package(ompl REQUIRED)
find_package(moveit_planners_ompl REQUIRED)

set(ASTRIBOT_MANIPULATION_DEPS
  rclcpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
)

# ---------------------------------------------------------------------------
# 核心能力库：不含任何 ROS 节点，纯算法 + MoveIt 数据结构。
# 这样单元测试可以在没有 move_group、没有仿真的环境里直接跑。
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core SHARED
  src/error_codes.cpp
  src/singularity_monitor.cpp
  src/collision_validator.cpp
  src/closed_chain_constraint.cpp
  src/trajectory_metrics.cpp
  src/trajectory_time_optimizer.cpp
  src/dual_arm_planner.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_core SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_core ${ASTRIBOT_MANIPULATION_DEPS})

# ---------------------------------------------------------------------------
# OMPL 规划器扩展插件
#
# 单独一个 shared library，被 move_group 通过 pluginlib 加载
# （planning_pipelines.yaml 里 planning_plugin 指向它）。
# 必须显式链接 ompl 本体：og::BITstar / og::InformedRRTstar 的符号在
# libompl.so 里，而 moveit_planners_ompl 只提供 ompl_interface 那一层封装。
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}_ompl_extension SHARED
  src/ompl_planner_extension.cpp
)

target_include_directories(${PROJECT_NAME}_ompl_extension PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_ompl_extension SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_ompl_extension
  ${OMPL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_ompl_extension
  ${ASTRIBOT_MANIPULATION_DEPS}
  moveit_planners_ompl
)
pluginlib_export_plugin_description_file(
  moveit_core ompl_planner_extension_plugin_description.xml)

# ---------------------------------------------------------------------------
# demo 与工具节点
# ---------------------------------------------------------------------------
add_executable(planning_demo_node src/planning_demo_node.cpp)
target_link_libraries(planning_demo_node ${PROJECT_NAME}_core)
ament_target_dependencies(planning_demo_node ${ASTRIBOT_MANIPULATION_DEPS})

# 自碰撞对生成工具：采样实测哪些 link 对"从不碰撞"或"总是碰撞"，
# 输出可直接粘进 SRDF 的 disable_collisions 片段。
# 这样 SRDF 里的碰撞关闭列表是可复现的实测结果，而不是手填。
add_executable(self_collision_pair_generator src/self_collision_pair_generator.cpp)
target_link_libraries(self_collision_pair_generator ${PROJECT_NAME}_core)
ament_target_dependencies(self_collision_pair_generator ${ASTRIBOT_MANIPULATION_DEPS})

# ---------------------------------------------------------------------------
# 安装
# ---------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ompl_extension
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 可执行节点必须装到 lib/${PROJECT_NAME}，这是 ros2 run / ros2 launch 查找
# libexec 的位置。装到 bin/ 的话 launch 会报
# "libexec directory ... does not exist"，而 ros2 run 也找不到节点。
install(
  TARGETS planning_demo_node self_collision_pair_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # 跳过理由与 astribot_s1_autonomy 一致（保持全仓库一致的 lint 策略）：
  # copyright  —— 仓库没有统一版权头格式。
  # uncrustify —— 其规则会把中文块注释重排到难以阅读。
  # cpplint    —— 行长按**字节**计数，中文字符每个 3 字节，带中文注释的行
  #               会被误报超长；本包源码都在 100 **字符**以内。
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # lint_cmake 限定扫描范围，避免把包内 build/ install/ 里 CMake 自己生成的
  # .cmake 文件也算进来（那些一律不符合 lint 规则，会淹掉真实问题）。
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_lint_cmake)
  ament_lint_auto_find_test_dependencies()
  ament_lint_cmake(CMakeLists.txt)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)

  # 这些单测只依赖 moveit_core 的数据结构和本包算法，不需要 ROS 运行时、
  # 不需要 move_group、不需要仿真，可在 CI 里直接跑。
  # 测试用的是 test/test_robot_fixture.hpp 里的合成双臂模型（两条平面 3R 臂），
  # 不加载真实 URDF —— 理由见该头文件的注释。
  ament_add_gtest(test_singularity_monitor test/test_singularity_monitor.cpp)
  target_link_libraries(test_singularity_monitor ${PROJECT_NAME}_core)
  target_include_directories(test_singularity_monitor PRIVATE test)
  ament_target_dependencies(test_singularity_monitor
    ${ASTRIBOT_MANIPULATION_DEPS} urdf srdfdom)

  ament_add_gtest(test_closed_chain_constraint test/test_closed_chain_constraint.cpp)
  target_link_libraries(test_closed_chain_constraint ${PROJECT_NAME}_core)
  target_include_directories(test_closed_chain_constraint PRIVATE test)
  ament_target_dependencies(test_closed_chain_constraint
    ${ASTRIBOT_MANIPULATION_DEPS} urdf srdfdom)

  ament_add_gtest(test_trajectory_optimizer test/test_trajectory_optimizer.cpp)
  target_link_libraries(test_trajectory_optimizer ${PROJECT_NAME}_core)
  target_include_directories(test_trajectory_optimizer PRIVATE test)
  ament_target_dependencies(test_trajectory_optimizer
    ${ASTRIBOT_MANIPULATION_DEPS} urdf srdfdom)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(${ASTRIBOT_MANIPULATION_DEPS})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
