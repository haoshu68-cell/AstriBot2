cmake_minimum_required(VERSION 3.8)
project(astribot_s1_gazebo_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------------------
# models/aws_robomaker_warehouse_{GroundB_01,RoofB_01}/ 下只有我们自己写的 model.sdf/model.config
# （修正了官方仓库里两个模型的惯性张量数据问题，见各自 model.sdf 顶部注释），
# 这两个模型的 mesh/材质贴图沿用官方 submodule 里现成的文件，不重新制作/不拷贝进 git 历史，
# 构建时从 submodule 源码树只读拷贝过来，和 astribot_s1_description 里引用 astribot_config
# meshes 的做法完全一致（只在构建脚本里出现相对路径，运行时仍然全部走 GZ_SIM_RESOURCE_PATH）。
# ---------------------------------------------------------------------------------------
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
