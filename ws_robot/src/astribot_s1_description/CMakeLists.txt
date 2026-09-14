cmake_minimum_required(VERSION 3.8)
project(astribot_s1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

set(ASTRIBOT_S1_MESH_SRC_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../astribot_config/robot_config/astribot_s1/meshes"
)

if(NOT EXISTS "${ASTRIBOT_S1_MESH_SRC_DIR}")
  message(FATAL_ERROR
    "astribot_s1_description: 找不到 mesh 源目录: ${ASTRIBOT_S1_MESH_SRC_DIR}\n"
    "请确认本包位于 <astribot_sdk_ros2>/ws_robot/src/astribot_s1_description，"
    "且 <astribot_sdk_ros2>/astribot_config/robot_config/astribot_s1/meshes 存在。"
  )
endif()

install(DIRECTORY
  ${ASTRIBOT_S1_MESH_SRC_DIR}/
  DESTINATION share/${PROJECT_NAME}/meshes
)

set(ASTRIBOT_S1_GRIPPER_MESH_SRC_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../astribot_config/robot_config/astribot_s1/model/meshes/s1_gripper"
)

if(NOT EXISTS "${ASTRIBOT_S1_GRIPPER_MESH_SRC_DIR}")
  message(FATAL_ERROR
    "astribot_s1_description: 找不到夹爪 mesh 源目录: ${ASTRIBOT_S1_GRIPPER_MESH_SRC_DIR}\n"
    "夹爪 mesh 在 model/meshes/s1_gripper（不是 meshes/），请确认 astribot_config 完整。"
  )
endif()

install(DIRECTORY
  ${ASTRIBOT_S1_GRIPPER_MESH_SRC_DIR}
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
