#include "astribot_s1_transport_mtc/execution_guard.hpp"
#include "astribot_s1_transport_mtc/base_motion_limits.hpp"
#include <iostream>
#include <limits>
int main() {
  using namespace astribot_s1_transport_mtc;
  int failed=0;auto check=[&](bool ok,const char* message){if(!ok){++failed;std::cerr<<message<<'\n';}};
  for (const bool simulated : {false,true}) {
    const auto limits=baseMotionLimits(false,simulated);
    check(limits.translation_m==.02 && limits.rotation_rad==.02,"default base limits unchanged");
  }
  const auto relaxed_limits=baseMotionLimits(true,true);
  check(relaxed_limits.translation_m==.02 && relaxed_limits.rotation_rad==.05,"simulation changes only angular limits");
  bool non_simulation_rejected=false;
  try { (void)baseMotionLimits(true,false); }
  catch (const std::invalid_argument&) { non_simulation_rejected=true; }
  check(non_simulation_rejected,"relaxed base limits rejected without simulated time");
  double error=0.;
  check(executionPoseValid(0.,0.,0.,0.,0.,0.,1.),"valid reference pose");
  check(!executionPoseValid(std::numeric_limits<double>::quiet_NaN(),0.,0.,0.,0.,0.,1.),"nonfinite base rejected");
  check(!executionPoseValid(0.,0.,0.,0.,0.,0.,0.),"zero quaternion rejected");
  check(trackingFault({"a","b"},{"b","a"},{0.,1.},{.05,1.},.05,error).empty() && error==.05,"joint order and exact bound");
  check(!trackingFault({"a"},{"a"},{0.},{.050001},.05,error).empty(),"error above bound rejected");
  check(!trackingFault({"a"},{"a","a"},{0.,0.},{0.,0.},.05,error).empty(),"duplicate names rejected");
  check(!trackingFault({"a"},{"b"},{0.},{0.},.05,error).empty(),"missing joint rejected");
  check(!trackingFault({"a"},{"a"},{},{0.},.05,error).empty(),"missing desired positions rejected");
  check(!trackingFault({"a"},{"a"},{0.},{std::numeric_limits<double>::quiet_NaN()},.05,error).empty(),"nonfinite measurement rejected");
  check(!trackingFault({"wrist6","wrist7"},{"wrist6","wrist7"},{-.1,-.5},{-.76,1.05791},.05,error).empty(),"observed wrist excursion rejected");
  return failed?1:0;
}
