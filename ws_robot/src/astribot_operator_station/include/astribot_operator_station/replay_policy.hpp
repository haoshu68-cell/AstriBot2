#pragma once
#include <map>
#include <string>
namespace astribot_operator_station {
inline bool replay_allowed(const std::string & topic,const std::string & type) {
 static const std::map<std::string,std::string> allow={
  {"/tf","tf2_msgs/msg/TFMessage"},{"/tf_static","tf2_msgs/msg/TFMessage"},
  {"/map_nav","nav_msgs/msg/OccupancyGrid"},{"/map","nav_msgs/msg/OccupancyGrid"},{"/plan","nav_msgs/msg/Path"},
  {"/odom","nav_msgs/msg/Odometry"},{"/joint_states","sensor_msgs/msg/JointState"},
  {"/global_costmap/costmap","nav_msgs/msg/OccupancyGrid"},{"/local_costmap/costmap","nav_msgs/msg/OccupancyGrid"}};
 auto it=allow.find(topic);return it!=allow.end()&&it->second==type;
}
}
