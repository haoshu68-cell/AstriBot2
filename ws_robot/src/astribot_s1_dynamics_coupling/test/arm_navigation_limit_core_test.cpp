#include "astribot_s1_dynamics_coupling/core.hpp"
#include <cstdlib>
#include <limits>
#include <iostream>
using namespace astribot_s1_dynamics_coupling;
#define CHECK(c) do {if(!(c)){std::cerr<<"check failed at "<<__LINE__<<'\n';return EXIT_FAILURE;}}while(0)
int main() {
  std::vector<std::string> names;for(int i=0;i<14;++i)names.push_back("joint_"+std::to_string(i));
  const auto required=names;std::vector<double> position(14,0.),velocity(14,0.);
  CHECK(complete_joint_sample(names,position,velocity,required));
  velocity.pop_back();CHECK(!complete_joint_sample(names,position,velocity,required));velocity.push_back(0.);
  names.back()="other";CHECK(!complete_joint_sample(names,position,velocity,required));names=required;
  names.back()=names.front();CHECK(!complete_joint_sample(names,position,velocity,required));names=required;
  position[0]=std::numeric_limits<double>::quiet_NaN();CHECK(!complete_joint_sample(names,position,velocity,required));position[0]=0.;
  velocity[0]=std::numeric_limits<double>::infinity();CHECK(!complete_joint_sample(names,position,velocity,required));
  const double full_reach_scale=scale_from_activity(reach_activity(.8865,.42,.8865),.15);
  // 1 - (1 - .15) rounds one ULP above the binary representation of .15.
  CHECK(full_reach_scale>=.15 && full_reach_scale<=std::nextafter(.15,1.));
  CHECK(scale_from_activity(0.,.15)==1.);
  std::cout<<"arm navigation limit joint/value boundaries passed\n";
}
