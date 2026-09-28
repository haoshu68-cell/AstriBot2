#include <cassert>
#include <thread>
#include "astribot_s1_path_tracking/policy_lease.hpp"
#include "astribot_s1_path_tracking/corridor_lease.hpp"
#include "astribot_s1_path_tracking/arrival_settling.hpp"
int main() {
 using namespace astribot_s1_path_tracking;
 const rclcpp::Time now(1000000000LL,RCL_ROS_TIME);
 PolicyLease lease;assert(!lease.fresh(now));
 PolicyLease::Message msg;msg.stamp=rclcpp::Time(9000000000LL,RCL_ROS_TIME);
 msg.epoch=1;msg.sequence=1;msg.lease_s=.02;msg.max_linear_speed=.2;msg.max_angular_speed=.3;
 lease.receive(msg);assert(lease.fresh(now)&&lease.linearSpeedLimit(now)==.2);
 std::this_thread::sleep_for(std::chrono::milliseconds(35));
 assert(lease.fresh(rclcpp::Time(90000000000LL,RCL_ROS_TIME)));
 msg.sequence=2;msg.hold=true;lease.receive(msg);assert(lease.held(now));
 msg.sequence=1;msg.hold=false;lease.receive(msg);assert(lease.held(now));
 msg.epoch=2;lease.receive(msg);assert(!lease.held(now));
 for(bool simulated:{false,true}) {
   assert(corridorRequestFresh(simulated,-90.,1000.,.3));
   assert(!corridorRequestFresh(simulated,0.,0.,0.));
 }
 ArrivalSettling<1> stop;stop.command(true,10.);
 stop.observe(1000.,10.,{0.},.6,.5,.01,.008);
 stop.observe(1001.,10.3,{0.},.6,.5,.01,.008);
 assert(!stop.evidence().stopped);
 stop.observe(1002.,10.6,{0.},.6,.5,.01,.008);
 assert(stop.evidence().stopped);
 stop.observe(1.,10.7,{100.},.6,.5,.01,.008);assert(stop.evidence().stopped);
 stop.command(false,10.8);assert(!stop.evidence().stopped);
}
