add_library(policy_observation_adapters STATIC src/policy_observation_adapters.cpp)
target_include_directories(policy_observation_adapters PUBLIC include)
target_link_libraries(policy_observation_adapters PUBLIC policy_contracts nlohmann_json::nlohmann_json)
target_compile_features(policy_observation_adapters PUBLIC cxx_std_17)
target_compile_options(policy_observation_adapters PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
if(DEFINED POLICY_FUSION_GEOMETRY_INCLUDE)
  target_include_directories(policy_observation_adapters PRIVATE "${POLICY_FUSION_GEOMETRY_INCLUDE}")
else()
  find_package(astribot_s1_robot_geometry REQUIRED)
  ament_target_dependencies(policy_observation_adapters PUBLIC astribot_s1_robot_geometry)
endif()
if(BUILD_TESTING)
  add_executable(policy_observation_adapters_probe test/policy_observation_adapters_probe.cpp)
  target_link_libraries(policy_observation_adapters_probe policy_observation_adapters)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME policy_observation_adapters_differential COMMAND ${CMAKE_COMMAND} -E env
    "POLICY_OBSERVATION_ADAPTERS_PROBE=$<TARGET_FILE:policy_observation_adapters_probe>"
    ${Python3_EXECUTABLE} -m pytest -q "${CMAKE_CURRENT_SOURCE_DIR}/test/test_policy_observation_adapters.py")
  add_test(NAME policy_json_strings_differential COMMAND ${CMAKE_COMMAND} -E env
    "POLICY_OBSERVATION_ADAPTERS_PROBE=$<TARGET_FILE:policy_observation_adapters_probe>"
    "PYTHONDONTWRITEBYTECODE=1"
    ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider "${CMAKE_CURRENT_SOURCE_DIR}/test/test_policy_json_strings.py")
endif()
