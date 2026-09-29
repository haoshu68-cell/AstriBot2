#include <cassert>
#include "corner_fixture.hpp"

namespace {rcl_time_point_value_t fake_steady_ns=100000000000LL;}
int main() {
  using namespace astribot_s1_path_tracking;
  ArrivalController controller;ArrivalGoalChecker checker;
  ArrivalControllerTestPeer::setup(controller);
  // Use the existing test peer to give the real base clock a deterministic
  // epoch distinct from std::chrono. No ROS graph or simulation is started.
  auto *clock=ThreePhaseControllerTestPeer::budgetClock(controller);
  const auto original_get_now=clock->get_now;
  clock->get_now=[](void*,rcl_time_point_value_t *now){*now=fake_steady_ns;return RCL_RET_OK;};
  controller.setPlan(path({{0.,0.},{1.,0.}},100));
  PolicyLease::Message lease;lease.epoch=1;lease.sequence=1;lease.hold=true;
  lease.lease_s=.3;lease.max_linear_speed=.35;lease.max_angular_speed=1.5;
  lease.stamp=rclcpp::Time(200000000000LL,RCL_ROS_TIME);
  const auto command=ArrivalControllerTestPeer::policyTick(controller,checker,200.,&lease);
  const double arrival_time=ArrivalControllerTestPeer::startedAt(controller);
  const double slam_time=ThreePhaseControllerTestPeer::latest(controller).second;
  clock->get_now=original_get_now;
  assert(command.linear.x==0.&&command.linear.y==0.&&command.angular.z==0.);
  // Executes ArrivalController::computeVelocityCommands and the real base
  // receiveSlamPose: using std::chrono in either path fails this assertion.
  assert(arrival_time==100.&&slam_time==100.&&arrival_time==slam_time);
  std::cout<<"ArrivalController and SLAM reception share the base steady clock\n";
}
