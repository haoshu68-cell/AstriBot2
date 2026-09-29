#include <cmath>
#include <iostream>
#include "astribot_s1_chassis_effort_drive_native/wheel_math.hpp"
using astribot::chassis_effort::WheelLoop;
int fails=0;
void require(bool ok,const char* text) {if(!ok) {std::cerr<<"FAIL "<<text<<"\n";++fails;}}
WheelLoop biased(double sign) {WheelLoop x;for(int i=0;i<800;++i)x.update(sign*.1,0,.4,.1,0,.1,1,.05,15,.01);return x;}
int main(){
 for(double s:{-1.,1.}) {
  auto a=biased(-s);auto t=a.update(s*.07,0,.4,.1,0,.1,1,.05,15,.01);
  require(s*t[0]>=.128-1e-10,"stationary reverse must not be opposed by old integral");
  require(s*a.integral()>0,"stationary reverse accumulates new direction");
  auto moving=biased(-s);double old=moving.integral(); moving.update(s*.07,-s*.2,.4,.1,0,.1,1,.05,15,.01);
  require(std::abs(moving.integral()-(old+s*.27*.01))<1e-12,"moving braking retains integral");
  auto zero=biased(-s);old=zero.integral();zero.update(0,0,.4,.1,0,.1,1,.05,15,.01);
  require(std::abs(zero.integral()-old)<1e-12,"zero target retains equilibrium");
  auto small=biased(-s);old=small.integral();small.update(s*.05,0,.4,.1,0,.1,1,.05,15,.01);
  require(std::abs(small.integral()-(old+s*.05*.01))<1e-12,"deadband target retains old behavior");
  auto same=biased(s);old=same.integral();same.update(s*.07,0,.4,.1,0,.1,1,.05,15,.01);
  require(std::abs(same.integral()-(old+s*.07*.01))<1e-12,"same direction retains integral");
  auto boundary=biased(-s);boundary.update(s*.07,-s*.05,.4,.1,0,.1,1,.05,15,.01);
  require(s*boundary.integral()>0,"measured deadband boundary permits stationary recovery");
  auto moving_edge=biased(-s);old=moving_edge.integral();moving_edge.update(s*.07,-s*.050001,.4,.1,0,.1,1,.05,15,.01);
  require(std::abs(moving_edge.integral()-(old+s*.120001*.01))<1e-12,"moving just outside deadband retains braking");
  auto disabled=biased(-s);old=disabled.integral();disabled.update(s*.07,0,.4,0,0,.1,1,.05,15,.01);
  require(std::abs(disabled.integral()-(old+s*.07*.01))<1e-12,"disabled I gain preserves existing state");
  auto no_time=biased(-s);old=no_time.integral();no_time.update(s*.07,0,.4,.1,0,.1,1,.05,15,0);
  require(no_time.integral()==old,"zero dt cannot change integral");
  auto limit=biased(-s);auto result=limit.update(s*.07,0,.4,.1,0,.1,1,.05,.12,.01);
  require(std::abs(result[0])<=.12,"effort bound remains enforced");
 }
 if(!fails)std::cout<<"PASS both signs: reversal, braking, zero, deadband, same direction, effort bound\n";
 return fails?1:0;
}
