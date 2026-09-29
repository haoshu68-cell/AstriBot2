add_library(policy_observer_core STATIC src/policy_observer_core.cpp)
target_include_directories(policy_observer_core PUBLIC include)
target_link_libraries(policy_observer_core PUBLIC policy_contracts navigation_math)
target_compile_options(policy_observer_core PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
add_library(policy_observer_runtime STATIC src/policy_observer_node.cpp)
target_include_directories(policy_observer_runtime PUBLIC include)
target_link_libraries(policy_observer_runtime PUBLIC policy_observer_core policy_envelope policy_fusion policy_health policy_risk policy_observation_adapters)
target_compile_options(policy_observer_runtime PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
ament_target_dependencies(policy_observer_runtime PUBLIC rclcpp ament_index_cpp tf2 tf2_ros geometry_msgs nav_msgs sensor_msgs std_msgs astribot_navigation_msgs)
add_executable(policy_observer_cpp src/policy_observer_main.cpp)
target_link_libraries(policy_observer_cpp PRIVATE policy_observer_runtime)
install(TARGETS policy_observer_cpp RUNTIME DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(policy_observer_core_probe test/policy_observer_core_probe.cpp)
  target_link_libraries(policy_observer_core_probe PRIVATE policy_observer_core nlohmann_json::nlohmann_json)
  target_compile_options(policy_observer_core_probe PRIVATE -ffp-contract=off)
  add_test(NAME policy_observer_core_differential
    COMMAND ${CMAKE_COMMAND} -E env
      "POLICY_OBSERVER_CORE_PROBE=$<TARGET_FILE:policy_observer_core_probe>"
      "PYTHONDONTWRITEBYTECODE=1"
      ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_policy_observer_core.py")
endif()
