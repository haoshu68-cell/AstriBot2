add_library(policy_path_evidence STATIC src/policy_path_evidence.cpp)
target_include_directories(policy_path_evidence PUBLIC include)
target_link_libraries(policy_path_evidence PUBLIC navigation_math)
ament_target_dependencies(policy_path_evidence PUBLIC nav_msgs)
target_compile_options(policy_path_evidence PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
if(BUILD_TESTING)
  add_executable(policy_behavior_probe test/policy_behavior_probe.cpp)
  target_link_libraries(policy_behavior_probe PRIVATE navigation_math nlohmann_json::nlohmann_json)
  target_compile_options(policy_behavior_probe PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
  add_test(NAME policy_behavior_differential
    COMMAND ${CMAKE_COMMAND} -E env
      "POLICY_BEHAVIOR_PROBE=$<TARGET_FILE:policy_behavior_probe>"
      "PYTHONDONTWRITEBYTECODE=1"
      ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_policy_behavior.py")
  add_executable(policy_path_evidence_probe test/policy_path_evidence_probe.cpp)
  target_link_libraries(policy_path_evidence_probe PRIVATE policy_path_evidence nlohmann_json::nlohmann_json)
  target_compile_options(policy_path_evidence_probe PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
  add_test(NAME policy_path_evidence_differential
    COMMAND ${CMAKE_COMMAND} -E env
      "POLICY_PATH_EVIDENCE_PROBE=$<TARGET_FILE:policy_path_evidence_probe>"
      "PYTHONDONTWRITEBYTECODE=1"
      ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_policy_path_evidence.py")
endif()
