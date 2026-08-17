cmake_minimum_required(VERSION 3.8)
project(astribot_s1_gazebo_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/setup_gazebo_env.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
