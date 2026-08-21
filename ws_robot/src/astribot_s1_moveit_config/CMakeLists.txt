cmake_minimum_required(VERSION 3.16)
project(astribot_s1_moveit_config)

find_package(ament_cmake REQUIRED)

# 纯配置包：只安装 config / launch / rviz 三个目录，不编译任何目标。
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # 跳过原因与 astribot_s1_autonomy 保持一致：
  # copyright  —— 仓库既有文件没有统一版权头格式，单独给本包套一套只会不一致。
  # uncrustify —— 本包没有 C++ 代码，无意义。
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # lint_cmake 限定扫描范围：它的自动钩子是 file(GLOB_RECURSE "*.cmake")，
  # 会把包内 build/ install/ 里 CMake 自己生成的文件也算进来（那些一律不符合
  # lint 规则），一跑就是上百条报错且全部出自生成物，把真实问题淹掉。
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_lint_cmake)
  ament_lint_auto_find_test_dependencies()
  ament_lint_cmake(CMakeLists.txt)
endif()

ament_package()
