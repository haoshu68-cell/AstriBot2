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
# 窄通道贴边通行的纯几何层。与 align_math 同样的理由拆开：
# 「有利朝向每 45° 复现」这条几何写错时，在线现象是「机器人在通道口卡住」，
# 从日志完全反推不出来，必须能离线逐条测。
add_library(narrow_math SHARED src/narrow_math.cpp)
target_include_directories(narrow_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(narrow_math align_math)
target_include_directories(align_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---- 控制器插件 ----
add_library(${PROJECT_NAME} SHARED src/three_phase_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} align_math narrow_math)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs nav2_core nav2_costmap_2d nav2_util nav_msgs
  pluginlib rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS align_math narrow_math ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

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

  ament_add_gtest(test_narrow_math test/test_narrow_math.cpp)
  target_link_libraries(test_narrow_math narrow_math)
endif()

ament_export_include_directories(include)
ament_export_libraries(align_math narrow_math ${PROJECT_NAME})
ament_export_dependencies(nav2_core nav2_costmap_2d geometry_msgs nav_msgs)
ament_package()
