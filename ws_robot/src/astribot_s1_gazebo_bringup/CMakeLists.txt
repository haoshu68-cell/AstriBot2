cmake_minimum_required(VERSION 3.8)
project(astribot_s1_gazebo_bringup)

find_package(ament_cmake REQUIRED)

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
