#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
namespace astribot::vision {
struct DepthView {
  const uint8_t* data;
  size_t bytes;
  uint32_t width,height,step;
  bool floating,big_endian;
};
struct ProjectionConfig {double fx,fy,cx,cy,min_depth,max_depth;int decimation;};
size_t output_points(const DepthView&,const ProjectionConfig&);
void validate_projection(const DepthView&,const ProjectionConfig&);
class DepthProjector {
public:
  virtual ~DepthProjector()=default;
  // Output is x,y,z,padding for every decimated pixel, matching PointCloud2 xyz.
  virtual void project(const DepthView&,const ProjectionConfig&,float* output)=0;
  // Concurrent cancellation must not wait for a device call to return.
  virtual void cancel()noexcept {}
};
std::unique_ptr<DepthProjector> make_depth_projector(const std::string& backend);
}
