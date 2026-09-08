cmake_minimum_required(VERSION 3.8)
project(astribot_s1_path_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ---- 纯函数层：不依赖 ROS，单独成库以便离线测试 ----
# 拆开的理由见 align_math.hpp 文件头：相位判断写错时在线现象极难反推。
add_library(align_math SHARED src/align_math.cpp)
target_include_directories(align_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 曲率-速度耦合的纯函数层（同样不依赖 ROS，理由见 curvature_speed_math.hpp 文件头）。
add_library(curvature_speed_math SHARED src/curvature_speed_math.cpp)
target_include_directories(curvature_speed_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---- 控制器插件 ----
add_library(${PROJECT_NAME} SHARED src/three_phase_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} align_math)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs nav2_core nav2_costmap_2d nav2_util nav_msgs
  pluginlib rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# ---- MPPI critic 插件（独立 so）----
# 单独一个库、不并进 ${PROJECT_NAME}：那个库是 nav2_core::Controller 插件，
# 被 RPP 档位也会加载；把 xtensor/MPPI 依赖塞进去会让不跑 MPPI 的场景
# 也必须解析 mppi 符号。两个基类本来就该分两个 so。
add_library(astribot_s1_mppi_critics SHARED src/curvature_speed_limit_critic.cpp)
target_include_directories(astribot_s1_mppi_critics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(astribot_s1_mppi_critics curvature_speed_math)
ament_target_dependencies(astribot_s1_mppi_critics
  nav2_core nav2_costmap_2d nav2_mppi_controller pluginlib rclcpp rclcpp_lifecycle)
# critics.xml 的导出标签是 nav2_mppi_controller、不是 nav2_core（见 critics.xml 注释）。
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

install(TARGETS align_math curvature_speed_math astribot_s1_mppi_critics ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml critics.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # 只链纯函数层：这些测试不需要 ROS 运行时，因此可以在没有 nav2 的环境里跑。
  ament_add_gtest(test_align_math test/test_align_math.cpp)
  target_link_libraries(test_align_math align_math)

  ament_add_gtest(test_curvature_speed_math test/test_curvature_speed_math.cpp)
  target_link_libraries(test_curvature_speed_math curvature_speed_math)

  # 只用 xtensor 头文件、不需要 nav2 运行时。守的是"累加不能换掉 nav2 的缓冲区"，
  # 一次真实 SIGSEGV 的回归测试（见 cost_accumulate.hpp 文件头）。
  find_package(xtensor QUIET)
  ament_add_gtest(test_cost_accumulate test/test_cost_accumulate.cpp)
  target_include_directories(test_cost_accumulate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_export_include_directories(include)
ament_export_libraries(align_math curvature_speed_math astribot_s1_mppi_critics ${PROJECT_NAME})
ament_export_dependencies(nav2_core nav2_costmap_2d geometry_msgs nav_msgs)
ament_package()
