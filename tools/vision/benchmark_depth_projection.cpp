// Synthetic CPU/CUDA compute+transfer benchmark; no ROS or camera latency claim.
#include "astribot_s1_perception_components/depth_projection.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>
int main() {
  using namespace astribot::vision;
  auto cpu=make_depth_projector("cpu"),gpu=make_depth_projector("cuda");
  for(unsigned w:{320u,640u,1280u})for(int stride:{1,4}) {
    unsigned h=w*9/16;std::vector<float>depth(size_t(w)*h,1.5f);
    DepthView v{reinterpret_cast<const uint8_t*>(depth.data()),depth.size()*4,w,h,w*4,true,false};
    ProjectionConfig c{400,400,double(w)/2,double(h)/2,.2,5.,stride};
    std::vector<float>out(output_points(v,c)*4);
    for(auto backend:{cpu.get(),gpu.get()}) {
      for(int i=0;i<10;++i)backend->project(v,c,out.data());
      std::vector<double>ms;
      for(int i=0;i<100;++i) {
        auto start=std::chrono::steady_clock::now();backend->project(v,c,out.data());
        ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
      }
      std::sort(ms.begin(),ms.end());
      std::printf("{\"width\":%u,\"height\":%u,\"decimation\":%d,\"backend\":\"%s\",\"p50_ms\":%.6f,\"p95_ms\":%.6f,\"max_ms\":%.6f}\n",
        w,h,stride,backend==cpu.get()?"cpu":"cuda",ms[50],ms[95],ms.back());
    }
  }
}
