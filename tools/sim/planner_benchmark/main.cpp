// Frozen observed grids only: no controller, geometry ACK or actuation publisher.
#include <fstream>
#include <iostream>
#include <chrono>
#include <nlohmann/json.hpp>
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "astribot_s1_robot_geometry/filled_collision.hpp"

using J=nlohmann::json;
namespace geo=astribot_s1_robot_geometry;
geometry_msgs::msg::PoseStamped pose(double x,double y,double yaw) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";
  p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(yaw/2);p.pose.orientation.w=std::cos(yaw/2);return p;
}
double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
J assess(const nav_msgs::msg::Path & path,nav2_costmap_2d::Costmap2D & map,
  const geo::Polygon & polygon,const J & target,double width) {
  J out;out["poses"]=path.poses.size();out["path"]=J::array();
  double length=0.,heading_change=0.;bool safe=!path.poses.empty(),crossed=false,inside_turn=false;
  double r=geo::radius(polygon),step=map.getResolution()/4.;
  for(size_t i=0;i<path.poses.size();++i) {
    const auto & p=path.poses[i].pose;double yaw=tf2::getYaw(p.orientation);
    out["path"].push_back({p.position.x,p.position.y,yaw});
    safe=safe && !geo::collision(map,polygon,p.position.x,p.position.y,yaw);
    if(i==0)continue;
    const auto & a=path.poses[i-1].pose;double ayaw=tf2::getYaw(a.orientation);
    double dx=p.position.x-a.position.x,dy=p.position.y-a.position.y,dyaw=wrap(yaw-ayaw);
    double d=std::hypot(dx,dy);length+=d;heading_change+=std::abs(dyaw);
    int n=std::max(1,int(std::ceil((d+r*std::abs(dyaw))/step)));
    for(int k=0;k<=n;++k) {
      double t=double(k)/n,x=a.position.x+t*dx,y=a.position.y+t*dy;
      safe=safe && !geo::collision(map,polygon,x,y,ayaw+t*dyaw,step);
      if(x>=-.6 && x<=.6 && std::abs(y)<width/2.) {
        crossed=true;if(d<1e-5 && std::abs(dyaw)>.005)inside_turn=true;
      }
    }
  }
  bool endpoint=false;
  if(!path.poses.empty()) {
    const auto & p=path.poses.back().pose;
    double xy=std::hypot(p.position.x-double(target[0]),p.position.y-double(target[1]));
    double yaw=std::abs(wrap(tf2::getYaw(p.orientation)-double(target[2])));
    out["endpoint_xy_m"]=xy;out["endpoint_yaw_rad"]=yaw;
    // Search discretization is reported separately from the exact execution goal.
    endpoint=xy<=std::sqrt(2.)*map.getResolution() && yaw<=.05;
  }
  out["filled_polygon_sweep_safe"]=safe;out["endpoint_compatible"]=endpoint;
  out["enters_requested_corridor"]=crossed;out["in_place_turn_inside"]=inside_turn;
  out["length_m"]=length;out["heading_variation_rad"]=heading_change;
  out["planning_candidate_only"]=true;
  out["accepted_for_comparison"]=safe && endpoint && crossed && !inside_turn;
  return out;
}
int main(int argc,char ** argv) {
  if(argc!=4) {std::cerr<<"usage: planner_benchmark input.json output.json omni_primitives.json\n";return 2;}
  J input;std::ifstream(argv[1])>>input;
  auto m=input.at("costmap").at("metadata");auto e=input.at("envelope");
  if(!e.at("navigation_allowed").get<bool>())throw std::runtime_error("snapshot not granted");
  geo::Polygon polygon;
  for(const auto & q:e.at("installed_footprint").at("points")) {
    geometry_msgs::msg::Point p;p.x=q.at("x");p.y=q.at("y");polygon.push_back(p);
  }
  if(!geo::convex(polygon))throw std::runtime_error("invalid frozen polygon");
  rclcpp::init(0,nullptr);
  auto mapros=std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions().arguments(
    {"--ros-args","-r","__ns:=/frozen_planner_evaluation"}));
  mapros->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
  mapros->set_parameter(rclcpp::Parameter("footprint_padding",0.));
  J footprint=J::array();for(const auto & p:polygon)footprint.push_back({p.x,p.y});
  // Configure polygon mode before lifecycle configure. setRobotFootprint alone
  // does not switch Costmap2DROS away from its default radius mode.
  mapros->set_parameter(rclcpp::Parameter("footprint",footprint.dump()));
  if(mapros->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS)
    throw std::runtime_error("offline costmap configure failed");
  auto map=mapros->getCostmap();
  map->resizeMap(m.at("size_x"),m.at("size_y"),m.at("resolution"),
    m.at("origin").at("position").at("x"),m.at("origin").at("position").at("y"));
  auto data=input.at("costmap").at("data").get<std::vector<unsigned char>>();
  if(data.size()!=map->getSizeInCellsX()*map->getSizeInCellsY())throw std::runtime_error("grid length");
  std::copy(data.begin(),data.end(),map->getCharMap());mapros->setRobotFootprint(polygon);
  if(mapros->getUseRadius())throw std::runtime_error("candidate must use polygon collision mode");
  auto tf=std::make_shared<tf2_ros::Buffer>(mapros->get_clock());
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
  auto start=input.at("start"),goal=input.at("goal");double width=input.at("fixture_width_m");
  auto from=pose(start[0],start[1],start[2]),to=pose(goal[0],goal[1],goal[2]);
  J results={{"input",argv[1]},{"geometry_hash",e.at("installed_geometry_hash")},
    {"epoch",e.at("epoch")},{"sweep_step_m",map->getResolution()/4.},
    {"obstacle_cost_threshold",254},{"unknown_is_collision",true},{"candidates",J::array()}};
  if(input.contains("baseline_path")) {
    nav_msgs::msg::Path path;
    for(auto & p:input["baseline_path"]["poses"]) {
      auto q=p["pose"]["orientation"];
      path.poses.push_back(pose(p["pose"]["position"]["x"],p["pose"]["position"]["y"],
        2.*std::atan2(q["z"].get<double>(),q["w"].get<double>())));
    }
    auto row=assess(path,*map,polygon,goal,width);row["planner"]="deployed_exact_2d";
    row["recorded_action_status"]=input["baseline_status"];results["candidates"].push_back(row);
  }
  for(const std::string type:{"SmacPlanner2D","SmacPlannerHybrid","SmacPlannerLattice"}) {
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("frozen_"+type,"/frozen_planner_evaluation");
    node->declare_parameter("GridBased.tolerance",0.);
    node->declare_parameter("GridBased.allow_unknown",false);
    node->declare_parameter("GridBased.downsample_costmap",false);
    node->declare_parameter("GridBased.smooth_path",false);
    node->declare_parameter("GridBased.max_planning_time",3.);
    node->declare_parameter("GridBased.angle_quantization_bins",72);
    node->declare_parameter("GridBased.minimum_turning_radius",.5);
    node->declare_parameter("GridBased.motion_model_for_search",std::string("REEDS_SHEPP"));
    node->declare_parameter("GridBased.allow_reverse_expansion",true);
    node->declare_parameter("GridBased.lattice_filepath",std::string(argv[3]));
    auto planner=loader.createSharedInstance("nav2_smac_planner/"+type);
    J row={{"planner",type}};bool configured=false;
    try {
      planner->configure(node,"GridBased",tf,mapros);configured=true;
      auto begin=std::chrono::steady_clock::now();auto path=planner->createPlan(from,to);
      double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
      row=assess(path,*map,polygon,goal,width);row["planner"]=type;row["planning_seconds"]=elapsed;
    } catch(const std::exception & error) {row["error"]=error.what();row["accepted_for_comparison"]=false;}
    if(configured)planner->cleanup();planner.reset();results["candidates"].push_back(row);
    std::cout<<row.dump()<<std::endl;
  }
  std::ofstream(argv[2])<<results.dump(2)<<'\n';
  mapros->on_cleanup(rclcpp_lifecycle::State());mapros.reset();rclcpp::shutdown();return 0;
}
