cmake_minimum_required(VERSION 3.16)
project(astribot_autonomy_core)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)

# Domain libraries deliberately have no ROS, PCL or Nav2 dependency.
add_library(perception_geometry SHARED src/slice_projector.cpp src/self_filter.cpp src/livox_custom_convert.cpp)
add_library(exploration_domain SHARED src/frontier_search.cpp src/path_validator.cpp
  src/costmap_adapter.cpp src/escape_logic.cpp src/failure_budget.cpp)
foreach(target perception_geometry exploration_domain)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
endforeach()
install(TARGETS perception_geometry exploration_domain EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
