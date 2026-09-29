cmake_minimum_required(VERSION 3.16)
project(astribot_graspnet_runtime LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(CTest)
if(BUILD_TESTING)
  add_executable(graspnet_cloud_io_test test/test_cloud_io.cpp)
  target_include_directories(graspnet_cloud_io_test PRIVATE include)
  target_compile_options(graspnet_cloud_io_test PRIVATE -UNDEBUG)
  add_test(NAME graspnet_cloud_io COMMAND graspnet_cloud_io_test)
endif()

# Deliberately isolated from ROS and its C++ ABI. Torch is optional for normal
# workspace builds. A CUDA wheel includes its LibTorch headers and shared libs.
set(GRASPNET_TORCH_ROOT "" CACHE PATH "Path to isolated Python torch package")
if(GRASPNET_TORCH_ROOT)
  add_executable(graspnet_worker src/graspnet_worker.cpp)
  target_include_directories(graspnet_worker PRIVATE include
    "${GRASPNET_TORCH_ROOT}/include"
    "${GRASPNET_TORCH_ROOT}/include/torch/csrc/api/include")
  set(GRASPNET_TORCH_CXX11_ABI 0 CACHE STRING "torch._C._GLIBCXX_USE_CXX11_ABI")
  target_compile_definitions(graspnet_worker PRIVATE _GLIBCXX_USE_CXX11_ABI=${GRASPNET_TORCH_CXX11_ABI})
  target_link_directories(graspnet_worker PRIVATE "${GRASPNET_TORCH_ROOT}/lib")
  target_link_libraries(graspnet_worker PRIVATE -Wl,--no-as-needed torch torch_cpu c10)
  if(EXISTS "${GRASPNET_TORCH_ROOT}/lib/libtorch_cuda.so")
    target_link_libraries(graspnet_worker PRIVATE torch_cuda c10_cuda)
  endif()
  target_link_libraries(graspnet_worker PRIVATE -Wl,--as-needed)
  set_target_properties(graspnet_worker PROPERTIES BUILD_RPATH "${GRASPNET_TORCH_ROOT}/lib"
    INSTALL_RPATH "${GRASPNET_TORCH_ROOT}/lib")
  install(TARGETS graspnet_worker RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()
install(DIRECTORY include/ DESTINATION include)
