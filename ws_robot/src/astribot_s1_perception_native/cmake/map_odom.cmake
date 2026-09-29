# Native map -> odom role; root package owns dependencies and global install.
add_executable(map_odom_tf_node src/map_odom_tf_node.cpp)
target_compile_features(map_odom_tf_node PRIVATE cxx_std_17)
target_compile_options(map_odom_tf_node PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_include_directories(map_odom_tf_node PRIVATE include)
ament_target_dependencies(map_odom_tf_node rclcpp geometry_msgs tf2 tf2_ros)
install(TARGETS map_odom_tf_node DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  add_executable(map_odom_core_probe test/map_odom_core_probe.cpp)
  target_compile_features(map_odom_core_probe PRIVATE cxx_std_17)
  target_compile_options(map_odom_core_probe PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off -UNDEBUG)
  target_include_directories(map_odom_core_probe PRIVATE include)
  add_test(NAME map_odom_core_smoke COMMAND map_odom_core_probe --self-test)
endif()
if(BUILD_TESTING)
  add_test(NAME map_odom_core_differential
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_map_odom_core.py
      --basetemp=${CMAKE_CURRENT_BINARY_DIR}/map_odom_core_pytest)
  set_tests_properties(map_odom_core_differential PROPERTIES
    RUN_SERIAL TRUE TIMEOUT 60
    ENVIRONMENT "MAP_ODOM_CORE_PROBE=$<TARGET_FILE:map_odom_core_probe>")
  add_test(NAME map_odom_ros_differential
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_map_odom_ros.py
      --basetemp=${CMAKE_CURRENT_BINARY_DIR}/map_odom_ros_pytest)
  set_tests_properties(map_odom_ros_differential PROPERTIES
    RUN_SERIAL TRUE TIMEOUT 180
    ENVIRONMENT "MAP_ODOM_CPP=$<TARGET_FILE:map_odom_tf_node>;MAP_ODOM_DOMAIN=160;ROS_LOCALHOST_ONLY=1")
endif()
