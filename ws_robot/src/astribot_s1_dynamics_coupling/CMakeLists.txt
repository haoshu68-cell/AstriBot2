cmake_minimum_required(VERSION 3.8)
project(astribot_s1_dynamics_coupling)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(arm_chassis_speed_coupling_node src/arm_chassis_speed_coupling_node.cpp)
target_include_directories(arm_chassis_speed_coupling_node PRIVATE include)
target_compile_features(arm_chassis_speed_coupling_node PRIVATE cxx_std_17)
target_compile_options(arm_chassis_speed_coupling_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(arm_chassis_speed_coupling_node rclcpp sensor_msgs nav2_msgs tf2 tf2_ros)
install(TARGETS arm_chassis_speed_coupling_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README_DYNAMICS_COUPLING.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(dynamics_core_probe test/core_probe.cpp)
  target_include_directories(dynamics_core_probe PRIVATE include)
  target_compile_features(dynamics_core_probe PRIVATE cxx_std_17)
  add_test(NAME dynamics_core_differential COMMAND ${CMAKE_COMMAND} -E env
    "DYNAMICS_CORE_PROBE=$<TARGET_FILE:dynamics_core_probe>"
    ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_core.py)
  add_executable(arm_navigation_limit_core_test test/arm_navigation_limit_core_test.cpp)
  target_include_directories(arm_navigation_limit_core_test PRIVATE include)
  target_compile_features(arm_navigation_limit_core_test PRIVATE cxx_std_17)
  add_test(NAME arm_navigation_limit_core COMMAND arm_navigation_limit_core_test)
  add_test(NAME arm_navigation_limit_boundary COMMAND ${Python3_EXECUTABLE} -m pytest -q
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_limit_boundary.py)
  set_tests_properties(dynamics_core_differential arm_navigation_limit_boundary PROPERTIES TIMEOUT 30)
  add_test(NAME arm_navigation_limit_ros COMMAND ${CMAKE_COMMAND} -E env
    "DYNAMICS_CPP=$<TARGET_FILE:arm_chassis_speed_coupling_node>"
    ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_limit_ros.py)
  set_tests_properties(arm_navigation_limit_ros PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
  # DYNAMICS_DOMAIN/ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY are explicitly assigned by
  # the session owner; this package never silently selects a shared ROS domain.
  # Historical Twist A/B protocol remains archived/reference-only. It does not
  # describe this upstream SpeedLimit interface and is no longer an active test.
endif()
ament_package()
