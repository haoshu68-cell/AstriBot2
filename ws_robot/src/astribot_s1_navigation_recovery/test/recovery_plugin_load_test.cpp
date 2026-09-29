#include <pluginlib/class_loader.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/controller.hpp>
int main() {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> planner("nav2_core","nav2_core::GlobalPlanner");
  pluginlib::ClassLoader<nav2_core::Controller> controller("nav2_core","nav2_core::Controller");
  return planner.createSharedInstance("astribot_s1_navigation_recovery::DeparturePlanner")&&
    controller.createSharedInstance("astribot_s1_navigation_recovery::DepartureController")?0:1;
}
