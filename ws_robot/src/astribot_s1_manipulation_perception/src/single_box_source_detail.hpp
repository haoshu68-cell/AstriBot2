#pragma once
#include <astribot_s1_manipulation_perception/single_box_request_source.hpp>
namespace astribot::perception_planning::detail {
void require(bool,const char*);
int64_t stamp(const builtin_interfaces::msg::Time&);
void health(const RgbdFrame&,const Context&,int64_t now_ns);
std::chrono::steady_clock::time_point deadline(const SensorReceipt&,int64_t until_ns);
bool fresh(const builtin_interfaces::msg::Time&,const SensorReceipt&,int64_t now_ns,int64_t age_ns);
}
