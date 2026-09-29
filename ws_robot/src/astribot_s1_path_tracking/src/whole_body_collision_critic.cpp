#include "astribot_s1_path_tracking/whole_body_collision_critic.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics {
void WholeBodyCollisionCritic::initialize() {
  reader_.configure(parent_.lock(),costmap_ros_);
}

void WholeBodyCollisionCritic::score(CriticData &data) {
  if(!enabled_)return;
  const auto snapshot=reader_.snapshot(data.state.pose,true);
  const auto &trajectory=data.trajectories;
  const auto batches=trajectory.x.shape(0),steps=trajectory.x.shape(1);
  const auto &start=data.state.pose.pose;
  const double start_yaw=tf2::getYaw(start.orientation);
  std::size_t rejected=0;
  for(std::size_t i=0;i<batches;++i) {
    double x=start.position.x,y=start.position.y,yaw=start_yaw;
    for(std::size_t k=0;k<steps;++k) {
      const double nx=trajectory.x(i,k),ny=trajectory.y(i,k),na=trajectory.yaws(i,k);
      if(snapshot->edgeCollision(x,y,yaw,nx,ny,na)) {
        data.costs(i)=std::numeric_limits<float>::infinity();++rejected;break;
      }
      x=nx;y=ny;yaw=na;
    }
  }
  // With a finite feasible minimum, exp(-infinity) gives rejected samples
  // exactly zero weight. Humble still updates controls before handling a
  // failed batch: keep that discarded batch finite to avoid infinity-infinity.
  if(rejected==batches) {data.costs.fill(0.f);data.fail_flag=true;}
  if(rejected) {
    RCLCPP_INFO_THROTTLE(logger_,*parent_.lock()->get_clock(),1000,
      "WHOLE_BODY_COLLISION rejected=%zu batch=%zu all_blocked=%d",
      rejected,batches,rejected==batches);
  }
}
}
PLUGINLIB_EXPORT_CLASS(mppi::critics::WholeBodyCollisionCritic,mppi::critics::CriticFunction)
