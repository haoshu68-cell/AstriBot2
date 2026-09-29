#pragma once
#include "astribot_s1_perception_components/depth_projection.hpp"
#include <type_traits>

namespace astribot::vision::wire {
// Local, same-build IPC only. Versioned native-endian POD, never a pointer or
// ROS serialized payload. Both ends independently validate byte counts.
constexpr uint32_t magic=0x41535052,version=1;
constexpr uint64_t max_input_bytes=33554432,max_output_bytes=67108864;
enum class Op:uint32_t {ready=1,request=2,result=3,error=4};
struct Header {
  uint32_t magic_value{magic},version_value{version};
  Op op{Op::request};uint32_t reserved{0};
  uint64_t boot{0},epoch{0},request{0},generation{0};
  int64_t capture_ns{0};
  uint64_t input_bytes{0},output_bytes{0};
  uint32_t width{0},height{0},step{0},floating{0},big_endian{0};
  int32_t decimation{0};
  double fx{0},fy{0},cx{0},cy{0},min_depth{0},max_depth{0};
  char reason[96]{};
};
static_assert(std::is_trivially_copyable_v<Header> && std::is_standard_layout_v<Header>);
inline bool envelope_valid(const Header& h) {
  return h.magic_value==magic && h.version_value==version && h.reserved==0 &&
    h.input_bytes<=max_input_bytes && h.output_bytes<=max_output_bytes;
}
inline ProjectionConfig config(const Header& h) {
  return {h.fx,h.fy,h.cx,h.cy,h.min_depth,h.max_depth,h.decimation};
}
inline bool request_valid(const Header& h) {
  if(!envelope_valid(h)||h.op!=Op::request||!h.boot||!h.epoch||!h.request||
     h.capture_ns<0||h.floating>1||h.big_endian>1)return false;
  const uint8_t dummy=0;
  try {
    const DepthView v{&dummy,size_t(h.input_bytes),h.width,h.height,h.step,bool(h.floating),bool(h.big_endian)};
    return output_points(v,config(h))*4*sizeof(float)==h.output_bytes;
  }catch(const std::invalid_argument&){return false;}
}
inline bool same_request(const Header& reply,const Header& request) {
  return envelope_valid(reply) && reply.boot==request.boot && reply.epoch==request.epoch &&
    reply.request==request.request && reply.generation==request.generation &&
    reply.capture_ns==request.capture_ns && reply.output_bytes==request.output_bytes;
}
}
