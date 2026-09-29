#include "astribot_s1_perception_components/depth_projection.hpp"
#include <cmath>
#include <cstring>
namespace astribot::vision {
void validate_projection(const DepthView& v,const ProjectionConfig& c) {
  if(!v.data || !v.width || !v.height || uint64_t(v.width)*v.height>4194304 ||
     uint64_t(v.step)<uint64_t(v.width)*(v.floating?4:2) ||
     uint64_t(v.step)*v.height>v.bytes || v.bytes>33554432 ||
     c.decimation<1 || c.decimation>64 || !std::isfinite(c.fx) || !std::isfinite(c.fy) ||
     c.fx<=0 || c.fy<=0 || !std::isfinite(c.cx) || !std::isfinite(c.cy) ||
     !std::isfinite(c.min_depth) || !std::isfinite(c.max_depth) ||
     c.min_depth<0 || c.max_depth<=c.min_depth)
    throw std::invalid_argument("invalid or oversized depth projection");
}
size_t output_points(const DepthView& v,const ProjectionConfig& c) {
  validate_projection(v,c);
  return size_t((v.width+c.decimation-1)/c.decimation)*((v.height+c.decimation-1)/c.decimation);
}
class CpuProjector final:public DepthProjector {
public:
  void project(const DepthView& v,const ProjectionConfig& c,float* output)override {
    validate_projection(v,c);if(!output)throw std::invalid_argument("null output");
    size_t i=0;const unsigned bpp=v.floating?4:2;
    for(unsigned y=0;y<v.height;y+=c.decimation)for(unsigned x=0;x<v.width;x+=c.decimation) {
      const auto* p=v.data+size_t(y)*v.step+size_t(x)*bpp;uint32_t bits=0;
      for(unsigned b=0;b<bpp;++b)bits|=uint32_t(p[b])<<(8*(v.big_endian?bpp-1-b:b));
      float d;if(v.floating)std::memcpy(&d,&bits,4);else d=float(bits)*.001f;
      if(std::isfinite(d) && d>=c.min_depth && d<=c.max_depth) {
        output[i]=float((double(x)-c.cx)*d/c.fx);output[i+1]=float((double(y)-c.cy)*d/c.fy);output[i+2]=d;
      }else output[i]=output[i+1]=output[i+2]=NAN;
      output[i+3]=0.f;i+=4;
    }
  }
};
#ifdef ASTRIBOT_CUDA_PROJECTION
std::unique_ptr<DepthProjector> make_cuda_depth_projector();
#endif
std::unique_ptr<DepthProjector> make_depth_projector(const std::string& backend) {
  if(backend=="cpu")return std::make_unique<CpuProjector>();
  if(backend=="cuda") {
#ifdef ASTRIBOT_CUDA_PROJECTION
    return make_cuda_depth_projector();
#else
    throw std::runtime_error("CUDA projection was not built; select cpu or build optional CUDA backend");
#endif
  }
  throw std::invalid_argument("projection_backend must be cpu or cuda");
}
}
