#include "astribot_s1_navigation_recovery/departure_path.hpp"
#include "astribot_s1_path_tracking/envelope_guard.hpp"
#include "astribot_s1_path_tracking/layered_collision_reader.hpp"
#include "astribot_s1_navigation_recovery/navigation_start_assessment.hpp"
#include "astribot_navigation_msgs/srv/plan_start_recovery.hpp"
#include "astribot_navigation_msgs/srv/get_recovery_obstacles.hpp"
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
    policy_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    policy_executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    policy_executor_->add_callback_group(policy_group_,node_->get_node_base_interface());
    policy_obstacles_=node_->create_client<astribot_navigation_msgs::srv::GetRecoveryObstacles>(
      "/navigation_policy/recovery_obstacles",rmw_qos_profile_services_default,policy_group_);
    start_recovery_=node_->create_service<astribot_navigation_msgs::srv::PlanStartRecovery>(
      "/navigation/plan_start_recovery",[this](
        astribot_navigation_msgs::srv::PlanStartRecovery::Request::ConstSharedPtr request,
        astribot_navigation_msgs::srv::PlanStartRecovery::Response::SharedPtr response) {
        try {
          if(request->execution_id.empty()||request->header.frame_id!=map_->getGlobalFrameID()||
              !std::isfinite(request->required_heading))
            throw std::invalid_argument("START_CONNECTION_RECOVERY_INVALID_REQUEST");
          if(!guard_.enabled())throw std::runtime_error("START_CONNECTION_REQUIRES_FIXED_GEOMETRY");
          if(!map_->isCurrent())throw std::runtime_error("DEPARTURE_COSTMAP_NOT_CURRENT");
          const auto tf=map_->getTfBuffer()->lookupTransform(
            map_->getGlobalFrameID(),map_->getBaseFrameID(),tf2::TimePointZero);
          geometry_msgs::msg::PoseStamped start;start.header=tf.header;
          start.pose.position.x=tf.transform.translation.x;
          start.pose.position.y=tf.transform.translation.y;
          start.pose.position.z=tf.transform.translation.z;
          start.pose.orientation=tf.transform.rotation;
          const auto snapshot=reader_.snapshot(start);
          RecoveryObstacles obstacles;
          if(request->use_policy_obstacles) {
            using Service=astribot_navigation_msgs::srv::GetRecoveryObstacles;
            const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
            if(!policy_obstacles_->wait_for_service(std::chrono::seconds(2)))
              throw std::runtime_error("DEPARTURE_POLICY_SERVICE_UNAVAILABLE");
            auto future=policy_obstacles_->async_send_request(std::make_shared<Service::Request>());
            const auto remaining=deadline-std::chrono::steady_clock::now();
            if(remaining<=std::chrono::steady_clock::duration::zero()||
                policy_executor_->spin_until_future_complete(future,remaining)!=rclcpp::FutureReturnCode::SUCCESS) {
              policy_obstacles_->remove_pending_request(future);
              throw std::runtime_error("DEPARTURE_POLICY_SERVICE_TIMEOUT");
            }
            const auto world=future.get();
            if(!world->valid)throw std::runtime_error("DEPARTURE_POLICY_UNAVAILABLE: "+world->reason);
            if(world->header.frame_id.empty())throw std::runtime_error("DEPARTURE_POLICY_FRAME_MISSING");
            if(world->header.frame_id!=map_->getGlobalFrameID()) {
              const auto transform=map_->getTfBuffer()->lookupTransform(
                map_->getGlobalFrameID(),world->header.frame_id,tf2::TimePointZero);
              obstacles.source_in_map={transform.transform.translation.x,transform.transform.translation.y,
                tf2::getYaw(transform.transform.rotation)};
            }
            for(const auto &polygon:world->obstacles) {
              double lx=INFINITY,ly=INFINITY,ux=-INFINITY,uy=-INFINITY;
              if(polygon.points.size()!=4)throw std::runtime_error("DEPARTURE_POLICY_INVALID_BOX");
              for(const auto &point:polygon.points) {
                if(!std::isfinite(point.x)||!std::isfinite(point.y))
                  throw std::runtime_error("DEPARTURE_POLICY_INVALID_BOX");
                lx=std::min(lx,double(point.x));ly=std::min(ly,double(point.y));
                ux=std::max(ux,double(point.x));uy=std::max(uy,double(point.y));
              }
              if(lx>=ux||ly>=uy)throw std::runtime_error("DEPARTURE_POLICY_INVALID_BOX");
              obstacles.boxes.push_back({lx,ly,ux,uy});
            }
          }
          auto *map=map_->getCostmap();DepartureSelection selected;
          {
            std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
            selected=selectDeparture(*snapshot,*map,snapshot->installedFootprint(),
              {start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation)},
              max_distance_,step_,request->required_heading,obstacles);
          }
          response->path.header=start.header;response->path.header.stamp=node_->now();
          start.header=response->path.header;response->path.poses.push_back(start);
          if(selected.distance>0.) {
            auto target=start;target.pose.position.x=selected.target.x;target.pose.position.y=selected.target.y;
            response->path.poses.push_back(target);
          }
          response->success=true;
          response->reason=selected.distance>0.?"START_CONNECTION_RECOVERY_SELECTED":"START_CONNECTION_CLEAR";
          path_->publish(response->path);
          RCLCPP_INFO(node_->get_logger(),
            "START_CONNECTION_RECOVERY execution=%s reason=%s heading=%.9f distance=%.3f direction=%s map=%s geometry=%s epoch=%lu",
            request->execution_id.c_str(),response->reason.c_str(),request->required_heading,selected.distance,
            selected.direction,snapshot->revision().c_str(),snapshot->geometryHash().c_str(),
            static_cast<unsigned long>(snapshot->envelopeEpoch()));
        } catch(const std::exception &error) {
          response->success=false;response->reason=error.what();response->path={};
        }
      });
  }
  void activate() override {evidence_->on_activate();path_->on_activate();}
  void deactivate() override {evidence_->on_deactivate();path_->on_deactivate();}
  void cleanup() override {start_recovery_.reset();policy_obstacles_.reset();
    policy_executor_->remove_callback_group(policy_group_);policy_executor_.reset();policy_group_.reset();
    assessment_.reset();reader_.cleanup();guard_.cleanup();evidence_.reset();path_.reset();map_.reset();node_.reset();}
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
  rclcpp::Service<astribot_navigation_msgs::srv::PlanStartRecovery>::SharedPtr start_recovery_;
  rclcpp::CallbackGroup::SharedPtr policy_group_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> policy_executor_;
  rclcpp::Client<astribot_navigation_msgs::srv::GetRecoveryObstacles>::SharedPtr policy_obstacles_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr evidence_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_;
};
} // namespace astribot_s1_navigation_recovery
PLUGINLIB_EXPORT_CLASS(astribot_s1_navigation_recovery::DeparturePlanner,nav2_core::GlobalPlanner)
