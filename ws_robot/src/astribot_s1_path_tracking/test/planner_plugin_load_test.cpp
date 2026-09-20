#include "pluginlib/class_loader.hpp"
#include "nav2_core/global_planner.hpp"
#include <iostream>
int main() {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
  for(const auto * name:{"astribot_s1_path_tracking::ExactGoalPlanner","astribot_s1_path_tracking::ExactHybridPlanner","astribot_s1_path_tracking::ExactLatticePlanner"}) {
    auto planner=loader.createSharedInstance(name);
    if(!planner) return 1;
    std::cout<<"Loaded "<<name<<'\n';
  }
}
