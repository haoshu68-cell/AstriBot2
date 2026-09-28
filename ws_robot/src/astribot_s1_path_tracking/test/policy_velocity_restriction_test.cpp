#include <cmath>
#include <iostream>
#include "astribot_s1_path_tracking/policy_lease.hpp"

int main() {
  using astribot_s1_path_tracking::PolicyLease;
  const rclcpp::Time now(1000000000LL,RCL_ROS_TIME);
  int failed=0,checks=0;
  auto check=[&](bool value,const char * name){++checks;if(!value){++failed;std::cerr<<name<<'\n';}};
  auto near=[](double a,double b){return std::abs(a-b)<1e-12;};
  auto command=[](double x,double y,double w){geometry_msgs::msg::Twist v;v.linear.x=x;v.linear.y=y;v.angular.z=w;return v;};
  PolicyLease policy;PolicyLease::Message m;
  m.stamp=now;m.epoch=1;m.sequence=1;m.lease_s=.3;m.max_linear_speed=.2;m.max_angular_speed=.1;
  policy.receive(m);
  auto v=command(.3,.4,1.);
  check(policy.restrict(v,now)&&near(v.linear.x,.03)&&near(v.linear.y,.04)&&near(v.angular.z,.1),
    "angular cap scales the whole coupled twist, retaining its path curvature");
  m.sequence++;m.max_linear_speed=.1;m.max_angular_speed=1.;policy.receive(m);
  v=command(-.3,.4,-.2);policy.restrict(v,now);
  check(near(v.linear.x,-.06)&&near(v.linear.y,.08)&&near(v.angular.z,-.04),
    "linear cap limits XY norm and reverse angular command together");
  v=command(.02,0.,.01);
  check(!policy.restrict(v,now)&&near(v.linear.x,.02)&&near(v.angular.z,.01),
    "already compliant command is unchanged");
  m.sequence++;m.alignment_required=true;m.max_angular_speed=.04;policy.receive(m);
  v=command(.03,.04,.2);policy.restrict(v,now);
  check(v.linear.x==0.&&v.linear.y==0.&&near(v.angular.z,.04),
    "alignment removes translation and still obeys angular cap");
  m.sequence++;m.alignment_required=false;m.centering_required=true;m.max_linear_speed=.02;policy.receive(m);
  v=command(.03,.04,.2);policy.restrict(v,now);
  check(near(std::hypot(v.linear.x,v.linear.y),.02)&&v.angular.z==0.,
    "centering removes rotation and still obeys linear cap");
  m.sequence++;m.centering_required=false;m.max_angular_speed=0.;policy.receive(m);
  v=command(.03,.04,.2);policy.restrict(v,now);
  check(v==geometry_msgs::msg::Twist(),"forbidden angular movement cannot change a curved path into a straight shortcut");
  m.sequence++;m.hold=true;policy.receive(m);
  v=command(.01,0.,0.);policy.restrict(v,now);
  check(v==geometry_msgs::msg::Twist(),"HOLD dominates nonzero speed allowances");
  m.sequence++;m.hold=false;policy.receive(m);
  v=command(.01,0.,0.);policy.restrict(v,now+rclcpp::Duration::from_seconds(.31));
  check(near(v.linear.x,.01),"positive constraint remains available beyond its observation deadline");
  std::cout<<"checks="<<checks<<" failed="<<failed<<'\n';return failed?1:0;
}
