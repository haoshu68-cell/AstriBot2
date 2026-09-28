#include "astribot_s1_navigation_recovery/navigation_start_assessment.hpp"
#include <iomanip>
#include <sstream>

namespace astribot_s1_navigation_recovery {
using namespace astribot_s1_path_tracking;
NavigationStartAssessment::NavigationStartAssessment(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map,EnvelopeGuard &guard,LayeredCollisionReader &reader)
  :node_(std::move(node)),map_(std::move(map)),guard_(guard),reader_(reader) {
  service_=node_->create_service<Service>("/navigation/assess_start",
    [this](Service::Request::ConstSharedPtr req,Service::Response::SharedPtr res){assess(*req,*res);});
}
void NavigationStartAssessment::assess(const Service::Request & req,Service::Response & res) {
  using Response=Service::Response;
  res.state=Response::UNAVAILABLE;res.header.stamp=node_->now();res.header.frame_id=map_->getGlobalFrameID();
  try {
    const auto &p=req.goal.pose.position;const auto &q=req.goal.pose.orientation;
    if(req.execution_id.empty()||req.goal.header.frame_id!=res.header.frame_id||
        !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(q.x)||!std::isfinite(q.y)||
        !std::isfinite(q.z)||!std::isfinite(q.w)||std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>1e-3)
      throw std::runtime_error("START_ASSESSMENT_INVALID_REQUEST");
    const auto transform=map_->getTfBuffer()->lookupTransform(map_->getGlobalFrameID(),map_->getBaseFrameID(),tf2::TimePointZero);
    res.evaluated_start.header=transform.header;
    res.evaluated_start.pose.position.x=transform.transform.translation.x;
    res.evaluated_start.pose.position.y=transform.transform.translation.y;
    res.evaluated_start.pose.position.z=transform.transform.translation.z;
    res.evaluated_start.pose.orientation=transform.transform.rotation;
    // Only the fixed-posture stack has the authoritative height geometry.
    if(!guard_.enabled()) {res.state=Response::READY;res.reason="START_RECOVERY_NOT_APPLICABLE";return;}
    // A fresh fixed envelope is geometry evidence even while installation ACKs
    // are pending. Motion permission is checked by the execution boundary.
    const auto layers=reader_.snapshot(res.evaluated_start);
    res.height_map_revision=layers->revision();res.geometry_hash=layers->geometryHash();res.envelope_epoch=layers->envelopeEpoch();
    auto *map=map_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    std::ostringstream metadata;metadata<<std::setprecision(17)<<map->getSizeInCellsX()<<','<<map->getSizeInCellsY()<<','<<map->getResolution()<<','<<map->getOriginX()<<','<<map->getOriginY()<<';';
    auto bytes=metadata.str();bytes.append(reinterpret_cast<const char *>(map->getCharMap()),size_t(map->getSizeInCellsX())*map->getSizeInCellsY());
    res.costmap_revision=astribot_s1_robot_geometry::sha256(bytes);
    const auto &actual=res.evaluated_start.pose;
    DeparturePose start{actual.position.x,actual.position.y,tf2::getYaw(actual.orientation)};
    if(layers->collision(start.x,start.y,start.yaw)) {
      res.state=Response::BLOCKED;res.reason="START_LAYER_COLLISION";
    } else if(navigationStartClear(*map,layers->installedFootprint(),start)) {
      res.state=Response::READY;res.reason="START_READY";
    } else {
      res.state=Response::RECOVERY_REQUIRED;res.reason="START_PLANAR_CLEARANCE_REQUIRES_DEPARTURE";
    }
  } catch(const std::exception & error) {res.state=Response::UNAVAILABLE;res.reason=error.what();}
  RCLCPP_INFO(node_->get_logger(),"START_ASSESSMENT execution=%s state=%u reason=%s start=(%.9f,%.9f,%.9f) pose_stamp=%.9f costmap=%s height_map=%s geometry=%s epoch=%lu",
    req.execution_id.c_str(),unsigned(res.state),res.reason.c_str(),res.evaluated_start.pose.position.x,
    res.evaluated_start.pose.position.y,tf2::getYaw(res.evaluated_start.pose.orientation),rclcpp::Time(res.evaluated_start.header.stamp).seconds(),
    res.costmap_revision.c_str(),res.height_map_revision.c_str(),res.geometry_hash.c_str(),static_cast<unsigned long>(res.envelope_epoch));
}
} // namespace astribot_s1_navigation_recovery
