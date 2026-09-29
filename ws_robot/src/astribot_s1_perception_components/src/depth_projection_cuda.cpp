// CUDA driver + NVRTC C APIs avoid the LibTorch/ROS C++ ABI boundary and nvcc dependency.
#include "astribot_s1_perception_components/depth_projection.hpp"
#include <cuda.h>
#include <nvrtc.h>
#include <algorithm>
namespace astribot::vision {
namespace {
void check(CUresult rc) {
  if(rc!=CUDA_SUCCESS) {const char* name=nullptr;cuGetErrorName(rc,&name);throw std::runtime_error(name?name:"CUDA_ERROR");}
}
void rtc(nvrtcResult rc) {if(rc!=NVRTC_SUCCESS)throw std::runtime_error(nvrtcGetErrorString(rc));}
constexpr const char* kernel=R"cuda(
extern "C" __global__ void project(const unsigned char* data,float* out,unsigned width,unsigned height,
  unsigned step,int floating,int big,int stride,double fx,double fy,double cx,double cy,double lo,double hi) {
  unsigned ow=(width+stride-1)/stride,oh=(height+stride-1)/stride;
  unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=ow*oh)return;
  unsigned x=(i%ow)*stride,y=(i/ow)*stride,bpp=floating?4:2,bits=0;
  const unsigned char* p=data+(unsigned long long)y*step+x*bpp;
  for(unsigned b=0;b<bpp;++b)bits|=unsigned(p[b])<<(8*(big?bpp-1-b:b));
  float d=floating?__uint_as_float(bits):float(bits)*.001f;
  if(isfinite(d) && d>=lo && d<=hi) {
    out[4*i]=float((double(x)-cx)*d/fx);out[4*i+1]=float((double(y)-cy)*d/fy);out[4*i+2]=d;
  }else out[4*i]=out[4*i+1]=out[4*i+2]=__int_as_float(0x7fffffff);
  out[4*i+3]=0;
})cuda";
class CudaProjector final:public DepthProjector {
  CUdevice device_{};CUcontext context_=nullptr;CUstream stream_=nullptr;
  CUmodule module_=nullptr;CUfunction function_=nullptr;
  CUdeviceptr input_=0,output_=0;size_t input_bytes_=0,output_bytes_=0;
  void cleanup()noexcept {
    if(context_)cuCtxSetCurrent(context_);
    if(stream_)cuStreamSynchronize(stream_);
    if(input_)cuMemFree(input_);
    if(output_)cuMemFree(output_);
    if(module_)cuModuleUnload(module_);
    if(stream_)cuStreamDestroy(stream_);
    if(context_)cuDevicePrimaryCtxRelease(device_);
  }
public:
  CudaProjector() {
    nvrtcProgram program=nullptr;
    try {
      check(cuInit(0));check(cuDeviceGet(&device_,0));check(cuDevicePrimaryCtxRetain(&context_,device_));
      check(cuCtxSetCurrent(context_));check(cuStreamCreate(&stream_,CU_STREAM_NON_BLOCKING));
      int major=0,minor=0;check(cuDeviceGetAttribute(&major,CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR,device_));
      check(cuDeviceGetAttribute(&minor,CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR,device_));
      std::string arch="--gpu-architecture=compute_"+std::to_string(major)+std::to_string(minor);
      const char* options[]={arch.c_str(),"--fmad=false"};
      rtc(nvrtcCreateProgram(&program,kernel,"depth_projection.cu",0,nullptr,nullptr));
      auto rc=nvrtcCompileProgram(program,2,options);
      if(rc!=NVRTC_SUCCESS){size_t n=0;nvrtcGetProgramLogSize(program,&n);std::string log(n,'\0');nvrtcGetProgramLog(program,log.data());throw std::runtime_error(log);}
      size_t n=0;rtc(nvrtcGetPTXSize(program,&n));std::vector<char> ptx(n);rtc(nvrtcGetPTX(program,ptx.data()));
      rtc(nvrtcDestroyProgram(&program));check(cuModuleLoadData(&module_,ptx.data()));
      check(cuModuleGetFunction(&function_,module_,"project"));
    }catch(...){if(program)nvrtcDestroyProgram(&program);cleanup();throw;}
  }
  ~CudaProjector()override{cleanup();}
  void project(const DepthView& v,const ProjectionConfig& c,float* out)override {
    const size_t points=output_points(v,c),bytes=points*4*sizeof(float);
    if(!out)throw std::invalid_argument("null output");
    check(cuCtxSetCurrent(context_));
    if(v.bytes>input_bytes_){if(input_){check(cuMemFree(input_));input_=0;}input_bytes_=0;check(cuMemAlloc(&input_,v.bytes));input_bytes_=v.bytes;}
    if(bytes>output_bytes_){if(output_){check(cuMemFree(output_));output_=0;}output_bytes_=0;check(cuMemAlloc(&output_,bytes));output_bytes_=bytes;}
    check(cuMemcpyHtoDAsync(input_,v.data,v.bytes,stream_));
    auto width=v.width,height=v.height,step=v.step;int floating=v.floating,big=v.big_endian,stride=c.decimation;
    double fx=c.fx,fy=c.fy,cx=c.cx,cy=c.cy,lo=c.min_depth,hi=c.max_depth;
    void* args[]={&input_,&output_,&width,&height,&step,&floating,&big,&stride,&fx,&fy,&cx,&cy,&lo,&hi};
    check(cuLaunchKernel(function_,unsigned((points+255)/256),1,1,256,1,1,0,stream_,args,nullptr));
    check(cuMemcpyDtoHAsync(out,output_,bytes,stream_));check(cuStreamSynchronize(stream_));
  }
};
}
std::unique_ptr<DepthProjector> make_cuda_depth_projector(){return std::make_unique<CudaProjector>();}
}
