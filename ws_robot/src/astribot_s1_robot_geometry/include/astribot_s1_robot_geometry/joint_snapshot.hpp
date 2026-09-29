#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace astribot_s1_robot_geometry {
class JointSnapshot {
  struct Packet {std::vector<std::string> names;std::vector<double> positions;int64_t stamp;};
  std::vector<std::string> required_;
  std::set<std::string> names_;
  std::map<std::string,std::pair<double,int64_t>> values_;
  int64_t maximum_age_,maximum_skew_,last_now_{-1};
  uint64_t epoch_{0};
  static void validate(const Packet & p) {
    if(p.stamp<=0 || p.names.size()!=p.positions.size() ||
        std::set<std::string>(p.names.begin(),p.names.end()).size()!=p.names.size() ||
        !std::all_of(p.positions.begin(),p.positions.end(),[](double v){return std::isfinite(v);})) {
      throw std::invalid_argument("invalid joint source");
    }
  }
  void apply(const Packet & p) {
    for(std::size_t i=0;i<p.names.size();++i) {
      const auto & name=p.names[i];
      if(names_.count(name) && (!values_.count(name) || p.stamp>values_.at(name).second)) {
        values_[name]={p.positions[i],p.stamp};
      }
    }
  }
public:
  JointSnapshot(std::vector<std::string> required,int64_t max_age=300000000,int64_t max_skew=100000000)
  : required_(std::move(required)),names_(required_.begin(),required_.end()),
    maximum_age_(max_age),maximum_skew_(max_skew) {
    if(required_.empty() || names_.size()!=required_.size() || max_age<=0 || max_skew<0) {
      throw std::invalid_argument("invalid joint snapshot contract");
    }
  }
  uint64_t epoch() const {return epoch_;}
  void clear() {values_.clear();}
  void observeClock(int64_t now) {
    if(now<0)throw std::invalid_argument("invalid source clock");
    last_now_=now;
  }
  // Source stamps only order samples; the local clock does not invalidate them.
  void update(const std::vector<std::string> & names,const std::vector<double> & positions,
      int64_t source,int64_t now) {
    observeClock(now);Packet p{names,positions,source};validate(p);
    apply(p);
  }
  // Retain each joint's newest observation regardless of DDS delivery order.
  void receive(const std::vector<std::string> & names,const std::vector<double> & positions,
      int64_t source,int64_t now) {
    observeClock(now);Packet p{names,positions,source};validate(p);apply(p);
  }
  auto snapshot(int64_t now) {
    observeClock(now);
    std::map<std::string,double> q;std::vector<int64_t> stamps;
    for(const auto & name:required_) {
      if(!values_.count(name))throw std::invalid_argument("JOINTS_INCOMPLETE");
      const auto & value=values_.at(name);q[name]=value.first;stamps.push_back(value.second);
    }
    const auto low=std::min_element(stamps.begin(),stamps.end());
    return std::make_tuple(q,stamps,*low+maximum_age_);
  }
};
}
