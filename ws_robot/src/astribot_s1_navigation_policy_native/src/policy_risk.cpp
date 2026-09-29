#include "astribot_s1_navigation_policy_native/policy_risk.hpp"
#include "astribot_s1_navigation_policy_native/policy_fusion.hpp"
#include "astribot_s1_navigation_policy_native/navigation_math.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astribot::navigation::policy {
namespace {
// Finite measured velocities may overflow when multiplied by the forecast
// horizon. Python still uses the route endpoint (or empty-route fallback).
// Keep the existing native batch for ordinary distances and that exact scalar
// interpolation for exceptional derived distances, without clipping the input.
std::vector<double> forecast_positions(const std::vector<double>& route,
  const std::vector<double>& distances, const RobotState& robot) {
  if(std::all_of(distances.begin(),distances.end(),[](double v){return std::isfinite(v);}))
    return navigation::path_position_batch(route,distances,{robot.x,robot.y,robot.yaw});
  std::vector<double> result;
  for(double distance:distances) {
    PlanarPose pose{robot.x,robot.y,robot.yaw};
    if(!route.empty()) {
      pose={route[route.size()-2],route.back(),robot.yaw};
      for(std::size_t i=route.size();i>2;i-=2) {
        const double dx=route[i-2]-route[i-4],dy=route[i-1]-route[i-3];
        if(euclidean_norm(dx,dy)>1e-9){pose[2]=std::atan2(dy,dx);break;}
      }
      for(std::size_t i=2;i<route.size();i+=2) {
        const double dx=route[i]-route[i-2],dy=route[i+1]-route[i-1];
        const double length=euclidean_norm(dx,dy);
        if(length>1e-9&&distance<=length) {
          const double ratio=distance/length;
          pose={route[i-2]+ratio*dx,route[i-1]+ratio*dy,std::atan2(dy,dx)};break;
        }
        distance-=length;
      }
    }
    result.insert(result.end(),pose.begin(),pose.end());
  }
  return result;
}
}
RobotState::RobotState(double x_, double y_, double yaw_, double vx_, double vy_, double wz_)
  :x(x_),y(y_),yaw(yaw_),vx(vx_),vy(vy_),wz(wz_) {
  finite(x,"x"); finite(y,"y"); finite(yaw,"yaw");
  finite(vx,"vx"); finite(vy,"vy"); finite(wz,"wz");
}

Bounds obstacle_bounds(const MetricBox& box, const MetricBox* previous) {
  const auto& before = previous ? *previous : box;
  const double ux = 2*std::sqrt(std::max(box.position_covariance_m2.values[0],before.position_covariance_m2.values[0]));
  const double uy = 2*std::sqrt(std::max(box.position_covariance_m2.values[4],before.position_covariance_m2.values[4]));
  return {{std::min(box.center_m.x-box.size_m.x/2,before.center_m.x-before.size_m.x/2)-ux,
           std::min(box.center_m.y-box.size_m.y/2,before.center_m.y-before.size_m.y/2)-uy},
          {std::max(box.center_m.x+box.size_m.x/2,before.center_m.x+before.size_m.x/2)+ux,
           std::max(box.center_m.y+box.size_m.y/2,before.center_m.y+before.size_m.y/2)+uy}};
}

std::vector<PredictionRow> prediction_rows(const WorldSnapshot& world, bool include_current, bool swept) {
  std::vector<PredictionRow> out;
  for (std::size_t owner=0; owner<world.tracks.size(); ++owner) {
    const auto& track=world.tracks[owner];
    if (track.prediction_model) {
      const auto& model=*track.prediction_model;
      const auto& box=track.geometry;
      auto row=[&](std::int64_t ns,double t,double before) {
        const double cx=box.center_m.x+model.velocity.x*t, cy=box.center_m.y+model.velocity.y*t;
        double lx=cx-box.size_m.x/2, ly=cy-box.size_m.y/2;
        double ux=cx+box.size_m.x/2, uy=cy+box.size_m.y/2;
        double covx=box.position_covariance_m2.values[0]+model.variance_m2_s2*(t*t);
        double covy=box.position_covariance_m2.values[4]+model.variance_m2_s2*(t*t);
        if (swept) {
          const double px=box.center_m.x+model.velocity.x*before, py=box.center_m.y+model.velocity.y*before;
          lx=std::min(lx,px-box.size_m.x/2); ly=std::min(ly,py-box.size_m.y/2);
          ux=std::max(ux,px+box.size_m.x/2); uy=std::max(uy,py+box.size_m.y/2);
          covx=std::max(covx,box.position_covariance_m2.values[0]+model.variance_m2_s2*(before*before));
          covy=std::max(covy,box.position_covariance_m2.values[4]+model.variance_m2_s2*(before*before));
        }
        const double uncertainty_x=2*std::sqrt(covx), uncertainty_y=2*std::sqrt(covy);
        out.push_back({owner,ns,{{lx-uncertainty_x,ly-uncertainty_y},{ux+uncertainty_x,uy+uncertainty_y}}});
      };
      if(include_current)row(0,0.,0.);
      double previous=0.;
      for(const auto& step:model.steps){row(step.first,step.second,previous);previous=step.second;}
    } else {
      const MetricBox* previous=&track.geometry;
      if(include_current)out.push_back({owner,0,obstacle_bounds(*previous)});
      for(const auto& sample:track.predictions) {
        out.push_back({owner,sample.offset_ns,obstacle_bounds(sample.geometry,swept?previous:nullptr)});
        previous=&sample.geometry;
      }
    }
  }
  // Per-track steps are ordered by contract. Current + explicit offset zero
  // deliberately remains duplicated, as in NumPy's stable owner/time lexsort.
  for(const auto& row:out) {
    const auto& b=row.bounds;
    if(!std::isfinite(b.lower[0])||!std::isfinite(b.lower[1])||!std::isfinite(b.upper[0])||
       !std::isfinite(b.upper[1])||b.lower[0]>b.upper[0]||b.lower[1]>b.upper[1])
      throw std::invalid_argument("prediction bounds must be finite and ordered");
  }
  return out;
}

bool has_predictions(const TrackedObstacle& track) {
  return !track.predictions.empty() || (track.prediction_model && !track.prediction_model->steps.empty());
}
std::size_t prediction_count(const TrackedObstacle& track) {
  return track.prediction_model ? track.prediction_model->steps.size() : track.predictions.size();
}
MetricBox final_prediction(const TrackedObstacle& track) {
  if (track.prediction_model && !track.prediction_model->steps.empty()) {
    const auto& model=*track.prediction_model; const auto& b=track.geometry;
    const double t=model.steps.back().second;
    return translate(b,model.velocity,t,t*t*model.variance_m2_s2);
  }
  return track.predictions.empty()?track.geometry:track.predictions.back().geometry;
}

Risk evaluate_risk(const WorldSnapshot& world,const RobotState& robot,const Polygon& path,
  const RiskProfile& profile,std::optional<double> speed_limit) {
  double speed=speed_limit.value_or(profile.max_speed_m_s);
  if(!std::isfinite(speed)||!(speed>0.&&speed<=profile.max_speed_m_s))
    throw std::invalid_argument("risk forecast speed outside profile");
  speed=std::max(speed,euclidean_norm(robot.vx,robot.vy));
  std::vector<double> path_xy;
  for(const auto& point:path)path_xy.insert(path_xy.end(),point.begin(),point.end());
  const auto route=navigation::remaining_path(path_xy,robot.x,robot.y);
  const auto& tracks=world.tracks;
  const bool uncertain=!world.unassociated.empty();
  constexpr double inf=std::numeric_limits<double>::infinity();
  if(tracks.empty())return {uncertain,false,inf,inf,{},false,uncertain,{}};
  std::vector<Bounds> current_boxes;
  for(const auto& track:tracks)current_boxes.push_back(obstacle_bounds(track.geometry));
  const auto current=clearance_many(std::vector<double>(tracks.size(),robot.x),
    std::vector<double>(tracks.size(),robot.y),std::vector<double>(tracks.size(),robot.yaw),current_boxes,profile.sweep);
  const double minimum=*std::min_element(current.begin(),current.end());
  std::vector<bool> blocked(tracks.size()),immediate_owners(tracks.size());
  bool immediate=false;double first=inf;
  for(std::size_t i=0;i<tracks.size();++i) {
    blocked[i]=current[i]<=0.;immediate_owners[i]=blocked[i];immediate=immediate||blocked[i];
  }
  const auto batch=prediction_rows(world);
  if(!batch.empty()) {
    std::vector<double> distances,x,y,yaw;
    std::vector<Bounds> boxes;
    for(const auto& row:batch) { distances.push_back(speed*(static_cast<double>(row.offset_ns)*1e-9));boxes.push_back(row.bounds); }
    const auto poses=forecast_positions(route,distances,robot);
    for(std::size_t i=0;i<batch.size();++i) { x.push_back(poses[3*i]);y.push_back(poses[3*i+1]);yaw.push_back(poses[3*i+2]); }
    const auto gaps=clearance_many(x,y,yaw,boxes,profile.sweep);
    for(std::size_t i=0;i<batch.size();++i)if(!route.empty()&&gaps[i]<=0.) {
      first=std::min(first,static_cast<double>(batch[i].offset_ns)*1e-9);blocked[batch[i].owner]=true;
    }
    const PlanarPose command{robot.vx,robot.vy,robot.wz};
    const double stop=navigation::stopping_horizon({command.begin(),command.end()},profile.reaction_time_s,
      profile.brake_deceleration_m_s2,profile.angular_brake_deceleration_rad_s2,profile.linear_stop_delay_s);
    const auto swept=prediction_rows(world,false,true);
    std::vector<double> begin,end;
    std::vector<Bounds> active_boxes;
    std::vector<std::size_t> owners;
    double previous=0.;std::size_t previous_owner=0;
    for(std::size_t i=0;i<batch.size();++i) {
      if(i==0||batch[i].owner!=previous_owner)previous=0.;
      const double offset=static_cast<double>(batch[i].offset_ns)*1e-9;
      if(previous<stop){begin.push_back(previous);end.push_back(std::min(offset,stop));active_boxes.push_back(swept[i].bounds);owners.push_back(swept[i].owner);}
      previous=offset;previous_owner=batch[i].owner;
    }
    const auto actual=motion_clearance(command,begin,end,active_boxes,profile.sweep,{robot.x,robot.y,robot.yaw});
    immediate=immediate||stop>profile.prediction_horizon_s;
    for(std::size_t i=0;i<actual.size();++i)if(actual[i]<=0.){immediate=true;immediate_owners[owners[i]]=true;}
  }
  std::vector<std::string> selected,instant;
  bool moving=false;
  for(std::size_t i=0;i<tracks.size();++i) {
    const auto& track=tracks[i];
    if(blocked[i]) {
      selected.push_back(track.fused_track_id);
      if(track.prediction_model&&!track.prediction_model->steps.empty()) {
        const auto& m=*track.prediction_model;const double t=m.steps.back().second;
        moving=moving||euclidean_norm((track.geometry.center_m.x+m.velocity.x*t)-track.geometry.center_m.x,
          (track.geometry.center_m.y+m.velocity.y*t)-track.geometry.center_m.y)>.1;
      }else if(!track.predictions.empty()) {
        const auto& end=track.predictions.back().geometry.center_m;
        moving=moving||euclidean_norm(end.x-track.geometry.center_m.x,end.y-track.geometry.center_m.y)>.1;
      }
    }
    if(immediate_owners[i])instant.push_back(track.fused_track_id);
  }
  return {!selected.empty()||uncertain,immediate,minimum,first,std::move(selected),moving,uncertain,std::move(instant)};
}
}  // namespace astribot::navigation::policy
