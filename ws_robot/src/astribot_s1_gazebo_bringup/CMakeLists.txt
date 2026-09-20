cmake_minimum_required(VERSION 3.8)
project(astribot_s1_gazebo_bringup)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-common4 REQUIRED COMPONENTS graphics)
find_package(ignition-rendering6 REQUIRED)
add_library(astribot_social_scene SHARED src/social_scene.cpp)
target_compile_features(astribot_social_scene PUBLIC cxx_std_17)
target_link_libraries(astribot_social_scene ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1 ignition-common4::graphics ignition-rendering6::ignition-rendering6)
install(TARGETS astribot_social_scene LIBRARY DESTINATION lib)
add_library(astribot_kinematic_payload SHARED src/kinematic_payload.cpp)
target_compile_features(astribot_kinematic_payload PUBLIC cxx_std_17)
target_link_libraries(astribot_kinematic_payload ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1)
install(TARGETS astribot_kinematic_payload LIBRARY DESTINATION lib)
add_library(astribot_contact_evidence SHARED src/contact_evidence.cpp)
target_compile_features(astribot_contact_evidence PUBLIC cxx_std_17)
target_link_libraries(astribot_contact_evidence ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1)
install(TARGETS astribot_contact_evidence LIBRARY DESTINATION lib)

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
