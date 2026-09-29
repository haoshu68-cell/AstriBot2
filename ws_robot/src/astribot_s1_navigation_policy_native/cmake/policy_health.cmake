# Include after policy_contracts and navigation_math targets are defined.
add_library(policy_health STATIC src/policy_health.cpp)
target_include_directories(policy_health PUBLIC include)
target_compile_features(policy_health PUBLIC cxx_std_17)
target_compile_options(policy_health PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(policy_health PUBLIC policy_contracts navigation_math)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(policy_health_probe test/policy_health_probe.cpp)
  target_link_libraries(policy_health_probe PRIVATE policy_health nlohmann_json::nlohmann_json)
  target_compile_options(policy_health_probe PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME policy_health_differential
    COMMAND ${CMAKE_COMMAND} -E env
      "POLICY_HEALTH_PROBE=$<TARGET_FILE:policy_health_probe>"
      "PYTHONDONTWRITEBYTECODE=1"
      ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_policy_health.py")
endif()
