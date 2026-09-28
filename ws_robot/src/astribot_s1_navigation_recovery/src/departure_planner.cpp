#include "astribot_s1_navigation_recovery/departure_path.hpp"
#include "astribot_s1_path_tracking/envelope_guard.hpp"
#include "astribot_s1_path_tracking/layered_collision_reader.hpp"
#include "astribot_s1_navigation_recovery/navigation_start_assessment.hpp"
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>

namespace astribot_s1_navigation_recovery {
using namespace astribot_s1_path_tracking;
class DeparturePlanner : public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
      std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    node_=parent.lock();map_=std::move(map);
    guard_.configure(node_,map_,"planner");reader_.configure(node_,map_);
    assessment_=std::make_unique<NavigationStartAssessment>(node_,map_,guard_,reader_);
    nav2_util::declare_parameter_if_not_declared(node_,name+".max_distance_m",rclcpp::ParameterValue(2.));
    nav2_util::declare_parameter_if_not_declared(node_,name+".search_step_m",rclcpp::ParameterValue(.05));
    max_distance_=node_->get_parameter(name+".max_distance_m").as_double();
    step_=node_->get_parameter(name+".search_step_m").as_double();
    if(!std::isfinite(step_)||!std::isfinite(max_distance_)||step_<=0||step_>max_distance_||max_distance_>2.)
      throw std::invalid_argument("DEPARTURE_INVALID_SEARCH_BUDGET");
    evidence_=node_->create_publisher<std_msgs::msg::String>("path_tracking/departure_evidence",10);
    path_=node_->create_publisher<nav_msgs::msg::Path>("path_tracking/departure_path",10);
  }
  void activate() override {evidence_->on_activate();path_->on_activate();}
  void deactivate() override {evidence_->on_deactivate();path_->on_deactivate();}
  void cleanup() override {assessment_.reset();reader_.cleanup();guard_.cleanup();evidence_.reset();path_.reset();map_.reset();node_.reset();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override {
    if(!map_->isCurrent())throw nav2_core::PlannerException("DEPARTURE_COSTMAP_NOT_CURRENT");
    if(start.header.frame_id!=map_->getGlobalFrameID()||goal.header.frame_id!=start.header.frame_id)
      throw nav2_core::PlannerException("DEPARTURE_FRAME_MISMATCH");
    nav_msgs::msg::Path output;output.header=start.header;output.header.stamp=node_->now();
    output.poses.push_back(start);output.poses.front().header=output.header;
    // This stage belongs to the fixed-posture mainline. Existing non-fixed
    // navigation gets a stationary handoff, never an unvalidated retreat.
    if(!guard_.enabled())return output;
    const auto snapshot=reader_.snapshot(start);
    auto *map=map_->getCostmap();
    DepartureSelection selected;
    try {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      selected=selectDeparture(*snapshot,*map,snapshot->installedFootprint(),
        {start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation)},
        max_distance_,step_);
    } catch(const std::runtime_error & error) {
      std_msgs::msg::String msg;msg.data=std::string("{\"result\":\"REJECTED\",\"reason\":\"")+error.what()+
        "\",\"map_revision\":\""+snapshot->revision()+"\",\"height_geometry_hash\":\""+snapshot->geometryHash()+"\"}";
      evidence_->publish(msg);throw nav2_core::PlannerException(error.what());
    }
    if(selected.distance>0) {
      auto target=output.poses.front();target.pose.position.x=selected.target.x;target.pose.position.y=selected.target.y;
      output.poses.push_back(target);
    }
    std_msgs::msg::String msg;msg.data=std::string("{\"result\":\"")+(selected.distance>0?"RECOVERY_SELECTED":"START_CLEAR")+
      "\",\"distance_m\":"+std::to_string(selected.distance)+",\"exit_distance_m\":"+std::to_string(selected.exit_distance)+
      ",\"direction\":\""+selected.direction+"\",\"map_revision\":\""+snapshot->revision()+
      "\",\"height_geometry_hash\":\""+snapshot->geometryHash()+"\",\"envelope_epoch\":"+std::to_string(snapshot->envelopeEpoch())+"}";
    evidence_->publish(msg);path_->publish(output);
    RCLCPP_INFO(node_->get_logger(),"DEPARTURE_PLAN %s",msg.data.c_str());
    return output;
  }
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  EnvelopeGuard guard_;LayeredCollisionReader reader_;double max_distance_=2.,step_=.05;
  std::unique_ptr<NavigationStartAssessment> assessment_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr evidence_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_;
};
} // namespace astribot_s1_navigation_recovery
PLUGINLIB_EXPORT_CLASS(astribot_s1_navigation_recovery::DeparturePlanner,nav2_core::GlobalPlanner)
