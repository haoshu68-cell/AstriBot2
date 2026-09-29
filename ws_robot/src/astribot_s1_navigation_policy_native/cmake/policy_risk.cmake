add_library(policy_sweep STATIC src/policy_sweep.cpp)
target_include_directories(policy_sweep PUBLIC include)
target_compile_features(policy_sweep PUBLIC cxx_std_17)
target_compile_options(policy_sweep PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_link_libraries(policy_sweep PUBLIC navigation_math nlohmann_json::nlohmann_json)
ament_target_dependencies(policy_sweep PUBLIC astribot_s1_robot_geometry)
add_library(policy_risk STATIC src/policy_risk.cpp)
target_include_directories(policy_risk PUBLIC include)
target_compile_features(policy_risk PUBLIC cxx_std_17)
target_compile_options(policy_risk PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_link_libraries(policy_risk PUBLIC policy_sweep policy_fusion)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  foreach(component sweep risk)
    add_executable(policy_${component}_probe test/policy_${component}_probe.cpp)
    target_link_libraries(policy_${component}_probe PRIVATE policy_${component} nlohmann_json::nlohmann_json)
    target_compile_options(policy_${component}_probe PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
    string(TOUPPER "${component}" component_upper)
    add_test(NAME policy_${component}_differential
      COMMAND ${CMAKE_COMMAND} -E env
        "POLICY_${component_upper}_PROBE=$<TARGET_FILE:policy_${component}_probe>"
        "PYTHONDONTWRITEBYTECODE=1"
        ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider
        "${CMAKE_CURRENT_SOURCE_DIR}/test/test_policy_${component}.py")
  endforeach()
  add_test(NAME policy_risk_independent_replay
    COMMAND ${CMAKE_COMMAND} -E env
      "POLICY_RISK_PROBE=$<TARGET_FILE:policy_risk_probe>"
      "POLICY_SWEEP_PROBE=$<TARGET_FILE:policy_sweep_probe>"
      "PYTHONDONTWRITEBYTECODE=1"
      ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/test/verify_policy_risk_extra.py")
endif()
