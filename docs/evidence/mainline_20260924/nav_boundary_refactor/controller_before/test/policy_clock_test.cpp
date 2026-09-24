#include <iostream>
#include <thread>
#include "astribot_s1_path_tracking/policy_lease.hpp"
#include "astribot_s1_path_tracking/corridor_lease.hpp"

int main() {
  using astribot_s1_path_tracking::PolicyLease;
  int failed=0;
  auto check=[&](bool ok,const char * why){if(!ok){std::cerr<<why<<'\n';++failed;}};
  using astribot_s1_path_tracking::corridorRequestFresh;
  check(corridorRequestFresh(true,.044,.346424,.3),"slow simulation request is physically fresh");
  check(!corridorRequestFresh(false,.044,.346424,.3),"hardware retains reception watchdog");
  check(corridorRequestFresh(false,.044,.04,.3),"fresh hardware request remains valid");
  for (bool simulated : {false,true}) {
    check(!corridorRequestFresh(simulated,.301,.01,.3),"source expiry never extended");
    check(!corridorRequestFresh(simulated,-.001,.01,.3),"future source cannot authorize motion");
    check(!corridorRequestFresh(simulated,.01,-.01,.3),"negative wall age invalid");
    check(!corridorRequestFresh(simulated,.01,.01,.301),"corridor lease ceiling unchanged");
    check(!corridorRequestFresh(simulated,.01,.01,0.),"zero lease invalid");
    check(!corridorRequestFresh(simulated,NAN,.01,.3),"nonfinite source invalid");
    check(!corridorRequestFresh(simulated,.01,NAN,.3),"nonfinite receipt invalid");
  }
  const rclcpp::Time now(1000000000LL,RCL_ROS_TIME);
  PolicyLease lease;
  check(!lease.waitingForClock(now),"missing input must not become clock wait");
  PolicyLease::Message msg;msg.stamp=rclcpp::Time(1001000000LL,RCL_ROS_TIME);
  msg.epoch=1;msg.sequence=1;msg.lease_s=.3;msg.max_linear_speed=.2;msg.max_angular_speed=.3;
  lease.receive(msg);
  check(!lease.fresh(now)&&lease.linearSpeedLimit(now)==0.,"future constraint must not authorize motion");
  check(lease.waitingForClock(now),"reproduce one millisecond clock delivery skew");
  const rclcpp::Time caught_up(1001000000LL,RCL_ROS_TIME);
  check(lease.fresh(caught_up)&&lease.linearSpeedLimit(caught_up)==.2,"fresh grant after clock catches up");
  const rclcpp::Time expired(1401000000LL,RCL_ROS_TIME);
  check(!lease.fresh(expired)&&!lease.waitingForClock(expired),"expired source must still fail");
  PolicyLease stalled;msg.lease_s=.02;stalled.receive(msg);
  check(stalled.waitingForClock(now),"initial bounded clock wait");
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
  msg.sequence=2;stalled.receive(msg);
  check(!stalled.fresh(now)&&!stalled.waitingForClock(now),"new messages must not extend a stalled clock wait");
  PolicyLease distant;msg.stamp=rclcpp::Time(2000000000LL,RCL_ROS_TIME);distant.receive(msg);
  check(!distant.waitingForClock(now),"large future jump must not wait");
  PolicyLease wall_guard;msg.stamp=now;msg.sequence=1;msg.lease_s=.02;wall_guard.receive(msg);
  check(wall_guard.fresh(now),"final constraint initially fresh");
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
  check(!wall_guard.fresh(now)&&wall_guard.linearSpeedLimit(now)==0.,
        "final wall watchdog still stops during paused source clock");
  PolicyLease corner;msg.stamp=now;msg.sequence=1;msg.lease_s=.3;msg.max_angular_speed=.2;
  corner.receive(msg);check(corner.cornerAngularSpeedLimit(now)==.2,"normal corner angular limit");
  msg.sequence++;msg.centering_required=true;corner.receive(msg);
  check(corner.cornerAngularSpeedLimit(now)==0.,"centering forbids a corner turn");
  msg.sequence++;msg.centering_required=false;msg.corridor_tracking_required=true;corner.receive(msg);
  check(corner.cornerAngularSpeedLimit(now)==0.,"passage tracking forbids a corner turn");
  msg.sequence++;msg.corridor_tracking_required=false;msg.alignment_required=true;corner.receive(msg);
  check(corner.cornerAngularSpeedLimit(now)==0.,"dedicated alignment cannot be replaced by a corner turn");
  check(corner.cornerAngularSpeedLimit(expired)==0.,"expired policy never authorizes corner rotation");
  return failed?1:0;
}
