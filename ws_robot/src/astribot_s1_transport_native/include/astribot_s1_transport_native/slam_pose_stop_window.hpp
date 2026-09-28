#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <cstdint>
#include <deque>

namespace astribot::transport {
// Callers admit finite, strictly source-ordered SLAM poses at their input boundary.
class SlamPoseStopWindow {
public:
  void clear() {samples_.clear();}
  void observe(const geometry_msgs::msg::Pose &pose,int64_t received_steady) {
    const auto &q=pose.orientation;
    samples_.push_back({received_steady,pose.position.x,pose.position.y,
      std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))});
    while(samples_.size()>2&&samples_[1].received<=received_steady-600000000)
      samples_.pop_front();
  }
  bool stopped() const {
    if(samples_.size()<2||samples_.back().received-samples_.front().received<600000000)return false;
    double x=0.,y=0.,angle=0.;
    for(size_t i=1;i<samples_.size();++i) {
      const auto &before=samples_[i-1],&current=samples_[i];
      x+=current.x-before.x;y+=current.y-before.y;
      angle+=std::remainder(current.angle-before.angle,2*std::acos(-1.));
      if(std::hypot(x,y)>.005||std::abs(angle)>.01)return false;
    }
    return true;
  }
private:
  struct Sample {int64_t received;double x,y,angle;};
  std::deque<Sample> samples_;
};
} // namespace astribot::transport
