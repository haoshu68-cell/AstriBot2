cmake_minimum_required(VERSION 3.8)
project(astribot_trajectory_bridge_native)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
option(ASTRIBOT_BUILD_PYBIND "Build the migration-only Python bridge" ON)
if(ASTRIBOT_BUILD_PYBIND)
  find_package(ament_cmake_python REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  find_package(pybind11 REQUIRED)
  ament_python_install_package(${PROJECT_NAME})
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)
endif()

add_library(chassis_math STATIC src/chassis_math.cpp src/arm_math.cpp)
target_include_directories(chassis_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(chassis_math PUBLIC cxx_std_17)
target_compile_options(chassis_math PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(chassis_math PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)
add_library(bridge_runtime STATIC
  src/gripper_controller.cpp
  src/arm_executor.cpp
  src/chassis_controller.cpp)
target_link_libraries(bridge_runtime PUBLIC chassis_math Threads::Threads)
target_compile_features(bridge_runtime PUBLIC cxx_std_17)
target_compile_options(bridge_runtime PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(bridge_runtime PROPERTIES POSITION_INDEPENDENT_CODE ON)


if(ASTRIBOT_BUILD_PYBIND)
  pybind11_add_module(_chassis_math_native src/chassis_math_pybind.cpp
                      src/gripper_controller_pybind.cpp
                      src/arm_traj_executor_pybind.cpp
                      src/chassis_bridge_core_pybind.cpp)
  target_link_libraries(_chassis_math_native PRIVATE bridge_runtime)
  target_compile_features(_chassis_math_native PRIVATE cxx_std_17)
  target_compile_options(_chassis_math_native PRIVATE -Wall -Wextra -Wpedantic)
  install(TARGETS _chassis_math_native LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
endif()
install(TARGETS chassis_math bridge_runtime EXPORT export_bridge_core ARCHIVE DESTINATION lib)
ament_export_targets(export_bridge_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

add_executable(arm_speed_limiter_cpp src/arm_speed_limiter_node.cpp)
target_link_libraries(arm_speed_limiter_cpp chassis_math)
target_include_directories(arm_speed_limiter_cpp PRIVATE include)
target_compile_features(arm_speed_limiter_cpp PRIVATE cxx_std_17)
target_compile_options(arm_speed_limiter_cpp PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(arm_speed_limiter_cpp rclcpp rcl_interfaces nav2_msgs sensor_msgs tf2 tf2_ros)
install(TARGETS arm_speed_limiter_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(benchmark_standalone_driver test/benchmark_standalone_driver.cpp)
  target_link_libraries(benchmark_standalone_driver bridge_runtime)
  add_executable(chassis_math_test test/chassis_math_test.cpp)
  target_link_libraries(chassis_math_test PRIVATE chassis_math)
  target_include_directories(chassis_math_test PRIVATE include)
  target_compile_features(chassis_math_test PRIVATE cxx_std_17)
  target_compile_options(chassis_math_test PRIVATE -UNDEBUG)
  add_test(NAME chassis_math COMMAND chassis_math_test)
  add_executable(gripper_core_probe test/gripper_core_probe.cpp)
  target_link_libraries(gripper_core_probe bridge_runtime)
  add_executable(runtime_contract_test test/runtime_contract_test.cpp)
  target_link_libraries(runtime_contract_test bridge_runtime)
  target_compile_options(runtime_contract_test PRIVATE -UNDEBUG)
  add_test(NAME runtime_contract COMMAND runtime_contract_test)
  set_tests_properties(runtime_contract PROPERTIES TIMEOUT 10)
  add_executable(runtime_config_test test/runtime_config_test.cpp)
  target_link_libraries(runtime_config_test bridge_runtime)
  target_compile_options(runtime_config_test PRIVATE -UNDEBUG)
  add_test(NAME runtime_config COMMAND runtime_config_test)
  add_executable(chassis_core_probe test/chassis_core_probe.cpp)
  target_link_libraries(chassis_core_probe bridge_runtime)
  add_executable(arm_core_probe test/arm_core_probe.cpp)
  target_link_libraries(arm_core_probe bridge_runtime)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME chassis_standalone_replay COMMAND ${CMAKE_COMMAND} -E env
    "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../astribot_trajectory_bridge"
    "ASTRIBOT_CHASSIS_PROBE=$<TARGET_FILE:chassis_core_probe>"
    ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_chassis_standalone.py)
  add_test(NAME arm_standalone_replay COMMAND ${CMAKE_COMMAND} -E env
    "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../astribot_trajectory_bridge"
    "ASTRIBOT_ARM_PROBE=$<TARGET_FILE:arm_core_probe>"
    ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_arm_standalone.py)
  add_test(NAME gripper_standalone_replay COMMAND ${CMAKE_COMMAND} -E env
    "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../astribot_trajectory_bridge"
    "ASTRIBOT_GRIPPER_PROBE=$<TARGET_FILE:gripper_core_probe>"
    ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gripper_standalone.py)
  add_test(NAME arm_dynamic_parameters COMMAND ${CMAKE_COMMAND} -E env
    "ASTRIBOT_ARM_LIMITER_CPP=$<TARGET_FILE:arm_speed_limiter_cpp>"
    "ASTRIBOT_ARM_PARAMETER_DOMAIN=118" "ROS_DOMAIN_ID=118" "ROS_LOCALHOST_ONLY=1"
    ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_arm_dynamic_parameters.py)
  set_tests_properties(arm_dynamic_parameters PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
endif()

ament_package()
