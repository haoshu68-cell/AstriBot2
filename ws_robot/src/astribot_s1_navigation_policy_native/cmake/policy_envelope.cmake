# Include after policy_contracts and policy_profile; all geometry remains in its kernel.
add_library(policy_envelope STATIC src/policy_envelope.cpp)
target_include_directories(policy_envelope PUBLIC include)
target_compile_features(policy_envelope PUBLIC cxx_std_17)
target_compile_options(policy_envelope PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_link_libraries(policy_envelope PUBLIC policy_contracts policy_profile nlohmann_json::nlohmann_json OpenSSL::Crypto)
ament_target_dependencies(policy_envelope PUBLIC astribot_navigation_msgs astribot_s1_robot_geometry)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(policy_envelope_probe test/policy_envelope_probe.cpp)
  target_link_libraries(policy_envelope_probe PRIVATE policy_envelope)
  target_compile_options(policy_envelope_probe PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
  add_test(NAME policy_envelope_differential
    COMMAND ${CMAKE_COMMAND} -E env "POLICY_ENVELOPE_PROBE=$<TARGET_FILE:policy_envelope_probe>"
      "PYTHONDONTWRITEBYTECODE=1" ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_policy_envelope.py")
endif()
