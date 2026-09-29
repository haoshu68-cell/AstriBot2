#pragma once
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace astribot_s1_robot_geometry {
// DDS topics and /clock have independent delivery order. Select the newest
// acquisition compatible with the observed clock; never retimestamp future
// data or extend an old sample's lease. The caller owns payload storage.
inline int selectSourceSample(const std::vector<int64_t> & stamps,int64_t now,
    int64_t maximum_age,int64_t maximum_future=10000000) {
  if(now<0 || maximum_age<0 || maximum_future<0 || maximum_future>10000000) {
    throw std::invalid_argument("invalid source time budget");
  }
  int selected=-1;int64_t newest=0;
  for(std::size_t i=0;i<stamps.size();++i) {
    const auto stamp=stamps[i];
    if(stamp<=0)continue;
    const bool valid=stamp>now ? stamp-now<=maximum_future : now-stamp<=maximum_age;
    if(valid && stamp>=newest) {selected=static_cast<int>(i);newest=stamp;}
  }
  return selected;
}
}
