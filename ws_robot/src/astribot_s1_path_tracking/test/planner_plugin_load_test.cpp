#include "pluginlib/class_loader.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include <iostream>
int main() {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
  for(const auto * name:{"astribot_s1_path_tracking::ExactGoalPlanner","astribot_s1_path_tracking::ExactHybridPlanner","astribot_s1_path_tracking::ExactLatticePlanner"}) {
    auto planner=loader.createSharedInstance(name);
    if(!planner) return 1;
    std::cout<<"Loaded "<<name<<'\n';
  }
  pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core","nav2_core::Controller");
  for(const auto * name:{"astribot_s1_path_tracking::ThreePhaseController","astribot_s1_path_tracking::ArrivalController"}) {
    if(!controllers.createSharedInstance(name)) return 1;
    std::cout<<"Loaded "<<name<<'\n';
  }
  pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core","nav2_core::GoalChecker");
  if(!goals.createSharedInstance("astribot_s1_path_tracking::ArrivalGoalChecker")) return 1;
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress("nav2_core","nav2_core::ProgressChecker");
  if(!progress.createSharedInstance("astribot_s1_path_tracking::PolicyProgressChecker")) return 1;
  std::cout<<"Loaded arrival goal and execution progress checkers (no initialize/configure)\n";

}
