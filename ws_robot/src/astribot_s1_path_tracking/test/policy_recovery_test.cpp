#include "corner_fixture.hpp"

int main() {
  using namespace astribot_s1_path_tracking;
  int failed=0,checks=0;
  auto check=[&](bool ok,const char * why){++checks;if(!ok){++failed;std::cerr<<why<<'\n';}};
  ArrivalController c;ArrivalGoalChecker checker;ArrivalControllerTestPeer::setup(c);
  c.setPlan(path({{0,0},{1,0}},100));
  auto rejects=[&](double now,const PolicyLease::Message * msg){
    try {ArrivalControllerTestPeer::policyTick(c,checker,now,msg);return false;}
    catch(const nav2_core::PlannerException & e){return std::string(e.what()).find("POLICY_LEASE_EXPIRED")!=std::string::npos;}
  };
  PolicyLease::Message m;m.epoch=1;m.sequence=1;m.stamp=rclcpp::Time(100000000000LL,RCL_ROS_TIME);
  m.lease_s=.012;m.max_linear_speed=.35;m.max_angular_speed=1.5;
  check(rejects(100.016,&m),"expired permission must throw before producing motion");
  check(rejects(100.10,nullptr),"missing replacement remains rejected");
  check(rejects(100.40,nullptr),"continuous expiry still throws beyond original Nav2 tolerance");
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
  // Clock rollback remains a latched execution fault even with fresh permission.
  m.sequence=3;m.stamp=rclcpp::Time(99000000000LL,RCL_ROS_TIME);
  bool rollback=false;try{ArrivalControllerTestPeer::policyTick(c,checker,99.,&m);}
  catch(const nav2_core::PlannerException&e){rollback=std::string(e.what()).find("CLOCK_JUMP")!=std::string::npos;}
  check(rollback,"clock rollback remains rejected");
  m.sequence=4;m.stamp=rclcpp::Time(100500000000LL,RCL_ROS_TIME);
  bool latched=false;try{ArrivalControllerTestPeer::policyTick(c,checker,100.5,&m);}
  catch(const nav2_core::PlannerException&e){latched=std::string(e.what()).find("CLOCK_JUMP")!=std::string::npos;}
  check(latched,"clock rollback remains latched after a new lease");
  std::cout<<"checks="<<checks<<" failed="<<failed<<'\n';return failed?1:0;
}
