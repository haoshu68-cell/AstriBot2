#include "astribot_s1_transport_mtc/execution_guard.hpp"
#include <iostream>
#include <limits>
int main() {
  using namespace astribot_s1_transport_mtc;
  int failed=0;auto check=[&](bool ok,const char* message){if(!ok){++failed;std::cerr<<message<<'\n';}};
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
  check(executionSourceFresh(1000000000,700000000,600000000),"exact source age boundary");
  check(!executionSourceFresh(1000000000,699999999,600000000),"expired source rejected");
  check(!executionSourceFresh(1000000000,900000000,950000000),"previous execution evidence rejected");
  check(executionSourceFresh(1000000000,1010000000,950000000),"bounded clock delivery skew");
  check(!executionSourceFresh(1000000000,1010000001,950000000),"future source outside delivery bound rejected");
  return failed?1:0;
}
