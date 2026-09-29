#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#ifdef ASTRIBOT_CUDA_PROJECTION
#include <cuda.h>
#include <dlfcn.h>
#include <atomic>
static std::atomic<size_t> fail_allocation_size{0};
extern "C" CUresult CUDAAPI cuMemAlloc_v2(CUdeviceptr* pointer,size_t bytes) {
  using Allocate=CUresult(CUDAAPI*)(CUdeviceptr*,size_t);
  static auto real=reinterpret_cast<Allocate>(dlsym(RTLD_NEXT,"cuMemAlloc_v2"));
  if(!real)return CUDA_ERROR_NOT_INITIALIZED;
  size_t expected=bytes;
  if(fail_allocation_size.compare_exchange_strong(expected,0))return CUDA_ERROR_OUT_OF_MEMORY;
  return real(pointer,bytes);
}
#endif
#include "astribot_s1_perception_components/depth_projection.hpp"
using namespace astribot::vision;
TEST(DepthProjection, CpuUnitsStrideEndianAndInvalid) {
  auto backend=make_depth_projector("cpu");
  const uint8_t bytes[]={3,232,0,0,255,255,0,0};
  DepthView view{bytes,8,3,1,8,false,true};
  ProjectionConfig config{100,100,0,0,.2,5.,1};
  float xyz[12]{};backend->project(view,config,xyz);
  EXPECT_FLOAT_EQ(xyz[2],1.f);EXPECT_FLOAT_EQ(xyz[0],0.f);
  EXPECT_TRUE(std::isnan(xyz[6]));EXPECT_TRUE(std::isnan(xyz[10]));
}
TEST(DepthProjection, RejectsUnknownBackend) {
  EXPECT_THROW(make_depth_projector("typo"),std::invalid_argument);
}
#ifndef ASTRIBOT_CUDA_PROJECTION
TEST(DepthProjection, UnbuiltCudaIsExplicitFailure) {
  EXPECT_THROW(make_depth_projector("cuda"),std::runtime_error);
}
#endif
TEST(DepthProjection, RejectsMalformedAndOversizedViews) {
  uint8_t data[8]{};DepthView v{data,8,2,1,4,false,false};ProjectionConfig c{100,100,0,0,.2,5.,1};
  v.step=3;EXPECT_THROW(output_points(v,c),std::invalid_argument);v.step=4;
  v.bytes=3;EXPECT_THROW(output_points(v,c),std::invalid_argument);v.bytes=8;
  c.decimation=0;EXPECT_THROW(output_points(v,c),std::invalid_argument);c.decimation=1;
  c.fx=NAN;EXPECT_THROW(output_points(v,c),std::invalid_argument);c.fx=100;
  v.width=0xffffffff;EXPECT_THROW(output_points(v,c),std::invalid_argument);
}
#ifdef ASTRIBOT_CUDA_PROJECTION
TEST(DepthProjection, CudaMatchesCpuAtMultipleSizes) {
  auto cpu=make_depth_projector("cpu"),gpu=make_depth_projector("cuda");
  for(auto width:{7u,640u,1280u}) for(int stride:{1,4}) {
    const unsigned height=width==7?3:width/2;
    std::vector<float> depth(width*height);
    for(size_t i=0;i<depth.size();++i)depth[i]=(i%19==0)?NAN:float(i%70)*.1f;
    DepthView view{reinterpret_cast<const uint8_t*>(depth.data()),depth.size()*4,width,height,width*4,true,false};
    ProjectionConfig c{400,410,100,110,.2,5.,stride};
    std::vector<float>a(output_points(view,c)*4),b(a.size());cpu->project(view,c,a.data());gpu->project(view,c,b.data());
    for(size_t i=0;i<a.size();++i) {
      if(std::isnan(a[i]))ASSERT_TRUE(std::isnan(b[i]));else ASSERT_NEAR(a[i],b[i],1.e-5);
    }
  }
}
TEST(DepthProjection, CudaUnitsPaddedStrideAndEndian) {
  auto cpu=make_depth_projector("cpu"),gpu=make_depth_projector("cuda");
  const uint8_t data[]={3,232,7,208,0,0,12,13,19,136,23,112,0,0,14,15};
  DepthView v{data,16,3,2,8,false,true};ProjectionConfig c{100,200,.5,.5,.2,5.,1};
  float a[24]{},b[24]{};cpu->project(v,c,a);gpu->project(v,c,b);
  for(int i=0;i<24;++i)if(std::isnan(a[i]))EXPECT_TRUE(std::isnan(b[i]));else EXPECT_NEAR(a[i],b[i],1.e-6);
}
TEST(DepthProjection, CudaRecoversAfterFailedInputGrowth) {
  auto gpu=make_depth_projector("cuda");std::vector<float>depth(17,1.f),out(68);
  ProjectionConfig c{100,100,0,0,.2,5.,1};
  DepthView small{reinterpret_cast<const uint8_t*>(depth.data()),12,3,1,12,true,false};
  auto large=small;large.bytes=large.step=68;large.width=17;
  gpu->project(small,c,out.data());fail_allocation_size=68;
  EXPECT_THROW(gpu->project(large,c,out.data()),std::runtime_error);EXPECT_EQ(fail_allocation_size.load(),0u);
  EXPECT_NO_THROW(gpu->project(small,c,out.data()));EXPECT_FLOAT_EQ(out[2],1.f);
}
TEST(DepthProjection, CudaRecoversAfterFailedOutputGrowth) {
  auto gpu=make_depth_projector("cuda");float depth[3]={1,1,1},out[12]{};
  DepthView v{reinterpret_cast<const uint8_t*>(depth),12,3,1,12,true,false};ProjectionConfig c{100,100,0,0,.2,5.,3};
  gpu->project(v,c,out);c.decimation=1;fail_allocation_size=48;
  EXPECT_THROW(gpu->project(v,c,out),std::runtime_error);EXPECT_EQ(fail_allocation_size.load(),0u);
  c.decimation=3;EXPECT_NO_THROW(gpu->project(v,c,out));EXPECT_FLOAT_EQ(out[2],1.f);
}
#endif
