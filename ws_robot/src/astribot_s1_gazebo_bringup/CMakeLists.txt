cmake_minimum_required(VERSION 3.8)
project(astribot_s1_gazebo_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(astribot_payload_msgs REQUIRED)
find_package(astribot_s1_payload_state REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(astribot_slam_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(urdf REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-common4 REQUIRED COMPONENTS graphics)
find_package(ignition-rendering6 REQUIRED)
add_library(astribot_social_scene SHARED src/social_scene.cpp)
target_compile_features(astribot_social_scene PUBLIC cxx_std_17)
target_link_libraries(astribot_social_scene ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1 ignition-common4::graphics ignition-rendering6::ignition-rendering6)
install(TARGETS astribot_social_scene LIBRARY DESTINATION lib)
add_library(astribot_payload_execution SHARED src/kinematic_inventory.cpp)
target_compile_features(astribot_payload_execution PUBLIC cxx_std_17)
target_include_directories(astribot_payload_execution PUBLIC include)
target_link_libraries(astribot_payload_execution ignition-gazebo6::ignition-gazebo6 nlohmann_json::nlohmann_json)
install(TARGETS astribot_payload_execution LIBRARY DESTINATION lib)
add_library(astribot_payload_geometry SHARED src/payload_geometry.cpp)
target_compile_features(astribot_payload_geometry PUBLIC cxx_std_17)
target_include_directories(astribot_payload_geometry PUBLIC include)
ament_target_dependencies(astribot_payload_geometry astribot_payload_msgs urdf astribot_s1_payload_state)
target_link_libraries(astribot_payload_geometry astribot_payload_execution ignition-gazebo6::ignition-gazebo6 ignition-common4::graphics)
install(TARGETS astribot_payload_geometry LIBRARY DESTINATION lib)
add_library(astribot_kinematic_payload SHARED src/kinematic_payload.cpp)
target_compile_features(astribot_kinematic_payload PUBLIC cxx_std_17)
target_link_libraries(astribot_kinematic_payload astribot_payload_execution ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1)
install(TARGETS astribot_kinematic_payload LIBRARY DESTINATION lib)
add_library(astribot_contact_evidence SHARED src/contact_evidence.cpp)
target_compile_features(astribot_contact_evidence PUBLIC cxx_std_17)
target_link_libraries(astribot_contact_evidence ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1)
install(TARGETS astribot_contact_evidence LIBRARY DESTINATION lib)

add_library(astribot_empty_inventory SHARED src/empty_inventory.cpp)
target_compile_features(astribot_empty_inventory PUBLIC cxx_std_17)
target_compile_options(astribot_empty_inventory PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(astribot_empty_inventory PRIVATE include)
ament_target_dependencies(astribot_empty_inventory rclcpp astribot_payload_msgs std_msgs)
target_link_libraries(astribot_empty_inventory astribot_s1_payload_state::payload_state_core)
target_link_libraries(astribot_empty_inventory astribot_payload_geometry astribot_payload_execution ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1 nlohmann_json::nlohmann_json)
install(TARGETS astribot_empty_inventory LIBRARY DESTINATION lib)
add_library(astribot_height_slice_map SHARED src/height_slice_map.cpp src/height_slice_shapes.cpp)
target_compile_features(astribot_height_slice_map PUBLIC cxx_std_17)
target_compile_options(astribot_height_slice_map PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(astribot_height_slice_map PRIVATE include)
ament_target_dependencies(astribot_height_slice_map rclcpp nav_msgs astribot_slam_msgs)
target_link_libraries(astribot_height_slice_map astribot_payload_execution ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1 ignition-common4::graphics nlohmann_json::nlohmann_json yaml-cpp OpenSSL::Crypto)
set_target_properties(astribot_height_slice_map PROPERTIES INSTALL_RPATH "$ORIGIN")
install(TARGETS astribot_height_slice_map LIBRARY DESTINATION lib)
set_target_properties(astribot_kinematic_payload astribot_empty_inventory astribot_payload_geometry
  PROPERTIES INSTALL_RPATH "$ORIGIN")
add_executable(load_empty_inventory src/load_empty_inventory.cpp)
target_compile_features(load_empty_inventory PUBLIC cxx_std_17)
target_compile_options(load_empty_inventory PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(load_empty_inventory ignition-gazebo6::ignition-gazebo6)
install(TARGETS load_empty_inventory DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(inventory_gate_test test/inventory_gate_test.cpp)
  target_include_directories(inventory_gate_test PRIVATE include)
  target_compile_features(inventory_gate_test PUBLIC cxx_std_17)
  ament_add_gtest(kinematic_inventory_test test/kinematic_inventory_test.cpp)
  target_include_directories(kinematic_inventory_test PRIVATE include)
  target_compile_features(kinematic_inventory_test PUBLIC cxx_std_17)
  # Exercise the shared library's own gate instead of interposing the test's inline copy.
  target_compile_options(kinematic_inventory_test PRIVATE -fvisibility-inlines-hidden)
  target_link_libraries(kinematic_inventory_test astribot_payload_execution ignition-gazebo6::ignition-gazebo6)
  ament_add_gtest(payload_geometry_test test/payload_geometry_test.cpp)
  target_link_libraries(payload_geometry_test astribot_payload_geometry)
  ament_target_dependencies(payload_geometry_test astribot_payload_msgs)
  ament_add_gtest(height_slice_geometry_test test/height_slice_geometry_test.cpp)
  target_include_directories(height_slice_geometry_test PRIVATE include)
  target_compile_features(height_slice_geometry_test PUBLIC cxx_std_17)
endif()

find_package(ignition-gui6 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick)
add_library(SocialProxyDisplay SHARED src/social_proxy_display.cpp src/social_proxy_display.qrc)
set_target_properties(SocialProxyDisplay PROPERTIES AUTOMOC ON AUTORCC ON)
target_compile_features(SocialProxyDisplay PUBLIC cxx_std_17)
target_link_libraries(SocialProxyDisplay ignition-gui6::ignition-gui6
  ignition-rendering6::ignition-rendering6 ignition-plugin1::ignition-plugin1 Qt5::Core Qt5::Quick)
install(TARGETS SocialProxyDisplay LIBRARY DESTINATION lib)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

set(AWS_WAREHOUSE_SRC_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../aws-robomaker-small-warehouse-world/models"
)

if(NOT EXISTS "${AWS_WAREHOUSE_SRC_DIR}")
  message(FATAL_ERROR
    "astribot_s1_gazebo_bringup: 找不到 aws-robomaker-small-warehouse-world 的 models 目录: "
    "${AWS_WAREHOUSE_SRC_DIR}\n"
    "请确认已执行 git submodule update --init --recursive。"
  )
endif()

foreach(_override_model aws_robomaker_warehouse_GroundB_01 aws_robomaker_warehouse_RoofB_01)
  install(DIRECTORY ${AWS_WAREHOUSE_SRC_DIR}/${_override_model}/meshes
    DESTINATION share/${PROJECT_NAME}/models/${_override_model}
  )
  install(DIRECTORY ${AWS_WAREHOUSE_SRC_DIR}/${_override_model}/materials
    DESTINATION share/${PROJECT_NAME}/models/${_override_model}
  )
endforeach()

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/setup_gazebo_env.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
