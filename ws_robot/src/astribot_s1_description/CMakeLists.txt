cmake_minimum_required(VERSION 3.8)
project(astribot_s1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
  # The source checkout may contain colcon output under urdf/ from an older
  # in-place build.  Those directories are build metadata, not robot assets;
  # installing them can duplicate latest_build symlinks and abort the package
  # install before camera profiles reach the share directory.
  PATTERN "build" EXCLUDE
  PATTERN "install" EXCLUDE
  PATTERN "log" EXCLUDE
  PATTERN ".pytest_cache" EXCLUDE
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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gripper_mimic_interfaces test/test_gripper_mimic_interfaces.py)
  ament_add_pytest_test(camera_profiles_installed test/test_camera_profiles.py)
  ament_add_pytest_test(camera_reference_mounts test/test_camera_reference_mounts.py)
endif()
ament_package()
