#include "corner_fixture.hpp"

int main() {
  using namespace astribot_s1_path_tracking;
  int failed=0,checks=0;
  auto check=[&](bool ok,const char * why){++checks;if(!ok){++failed;std::cerr<<why<<'\n';}};
  ArrivalController c;ArrivalGoalChecker checker;ArrivalControllerTestPeer::setup(c);
  c.setPlan(path({{0,0},{1,0}},100));
  auto accepts=[&](double now,const PolicyLease::Message * msg){
    try {ArrivalControllerTestPeer::policyTick(c,checker,now,msg);return true;}
    catch(const nav2_core::PlannerException & e){std::cerr<<e.what()<<'\n';return false;}
  };
  PolicyLease::Message m;m.epoch=1;m.sequence=1;m.stamp=rclcpp::Time(100000000000LL,RCL_ROS_TIME);
  m.lease_s=.012;m.max_linear_speed=.35;m.max_angular_speed=1.5;
  check(accepts(100.016,&m),"latest permission remains usable beyond its former timestamp TTL");
  check(accepts(100.10,nullptr),"latest valid permission is retained until replaced");
  check(accepts(100.40,nullptr),"waiting without replacement does not create a timestamp failure");
  m.sequence=2;m.stamp=rclcpp::Time(100410000000LL,RCL_ROS_TIME);m.lease_s=.2;m.hold=true;
  try {
    const auto v=ArrivalControllerTestPeer::policyTick(c,checker,100.42,&m);
    check(v.linear.x==0.&&v.linear.y==0.&&v.angular.z==0.,"new valid HOLD yields zero");
  }catch(...){check(false,"valid replacement must recover within a still-active execution");}
  m.sequence=1;m.hold=false;
  try {
    const auto v=ArrivalControllerTestPeer::policyTick(c,checker,100.43,&m);
    check(v.linear.x==0.&&v.linear.y==0.&&v.angular.z==0.,"old sequence cannot replace valid HOLD");
  }catch(...){check(false,"old sequence must not poison current valid HOLD");}
  // ROS acquisition time can move independently of the steady execution
  // budget. It must not reinstate the removed timestamp/rollback gate.
  m.sequence=3;m.stamp=rclcpp::Time(99000000000LL,RCL_ROS_TIME);
  check(accepts(99.,&m),"newer-sequence permission survives ROS time rollback");
  m.sequence=4;m.stamp=rclcpp::Time(100500000000LL,RCL_ROS_TIME);
  check(accepts(100.5,&m),"ROS time recovery has no latched timestamp failure");
  std::cout<<"checks="<<checks<<" failed="<<failed<<'\n';return failed?1:0;
}
