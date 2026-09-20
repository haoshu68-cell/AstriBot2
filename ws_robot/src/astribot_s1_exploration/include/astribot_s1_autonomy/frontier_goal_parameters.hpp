#ifndef ASTRIBOT_S1_AUTONOMY__FRONTIER_GOAL_PARAMETERS_HPP_
#define ASTRIBOT_S1_AUTONOMY__FRONTIER_GOAL_PARAMETERS_HPP_
#include "rclcpp/rclcpp.hpp"
#include "astribot_s1_autonomy/frontier_search.hpp"
namespace astribot_s1_autonomy {
inline void declareFrontierGoalParameters(rclcpp::Node & node)
{
  node.declare_parameter("search.goal_footprint",
    std::vector<double>{.32,.32,-.32,.32,-.32,-.32,.32,-.32});
  node.declare_parameter("search.goal_footprint_margin",.05);
  node.declare_parameter("search.goal_retreat_radius",.8);
}
inline bool loadFrontierGoalParameters(rclcpp::Node & node,
  FrontierSearchParams & params, std::string & error)
{
  const auto polygon=node.get_parameter("search.goal_footprint").as_double_array();
  if (polygon.size()<6 || polygon.size()%2 || polygon.size()>64) {
    error="search.goal_footprint requires 3..32 body-frame x/y pairs";return false;
  }
  params.goal_footprint.clear();
  for (std::size_t i=0;i<polygon.size();i+=2) {params.goal_footprint.push_back({polygon[i],polygon[i+1]});}
  params.goal_footprint_margin=node.get_parameter("search.goal_footprint_margin").as_double();
  params.goal_retreat_radius=node.get_parameter("search.goal_retreat_radius").as_double();
  return true;
}
}
#endif
