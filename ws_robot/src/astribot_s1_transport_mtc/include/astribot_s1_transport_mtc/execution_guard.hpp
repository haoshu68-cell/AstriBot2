#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace astribot_s1_transport_mtc {
inline bool executionPoseValid(double x,double y,double z,double qx,double qy,double qz,double qw) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
    std::isfinite(qx) && std::isfinite(qy) && std::isfinite(qz) && std::isfinite(qw) &&
    std::abs(qx*qx+qy*qy+qz*qz+qw*qw-1.)<=.001;
}
inline std::string trackingFault(const std::vector<std::string>& expected,
    const std::vector<std::string>& names,const std::vector<double>& desired,
    const std::vector<double>& actual,double limit,double & maximum) {
  maximum=0.;
  if(expected.empty() || names.size()!=desired.size() || names.size()!=actual.size() ||
      std::set<std::string>(names.begin(),names.end()).size()!=names.size() ||
      !std::isfinite(limit) || limit<=0.)return "MANIPULATION_TRACKING_STATE_INVALID";
  std::string worst;
  for(const auto & name:expected) {
    auto it=std::find(names.begin(),names.end(),name);
    if(it==names.end())return "MANIPULATION_TRACKING_STATE_INVALID";
    const size_t i=it-names.begin();
    if(!std::isfinite(desired[i]) || !std::isfinite(actual[i]))return "MANIPULATION_TRACKING_STATE_INVALID";
    const double error=std::abs(desired[i]-actual[i]);
    if(error>maximum){maximum=error;worst=name;}
  }
  return maximum>limit ? "MANIPULATION_TRACKING_ERROR:"+worst : "";
}

}
