#include "astribot_s1_path_tracking/workstation_alignment_core.hpp"
#include "astribot_s1_path_tracking/arrival_braking.hpp"
#include "ruckig/ruckig.hpp"
#include <algorithm>

namespace astribot_s1_path_tracking {
namespace {
bool zeroMotion(const WorkstationMotionState &s) {
  for(std::size_t i=0;i<3;++i)
    if(std::abs(s.velocity[i])>1e-9||std::abs(s.acceleration[i])>1e-8)return false;
  return true;
}
bool sweepClear(const LayeredCollisionSnapshot &map,const WorkstationPose &pose,
    const std::array<double,3> &command,double dt) {
  const double sweep=(std::hypot(command[0],command[1])+map.radius()*std::abs(command[2]))*dt;
  const int samples=std::max(1,int(std::ceil(sweep/(map.resolution()*.5))));
  // Sample the actual constant-body-twist arc, with an arc-length coverage
  // margin. This is the same integration used by full admission prediction.
  for(int i=0;i<=samples;++i) {
    const auto p=workstationIntegrate(pose,command,dt*double(i)/samples);
    if(map.collision(p.x,p.y,p.yaw,sweep/(2.*samples)))return false;
  }
  return true;
}
}
std::array<double,3> workstationCommand(const WorkstationPose &current,const WorkstationPose &goal,
    WorkstationMotionState &state,const WorkstationConfig &cfg,double dt,bool stopping) {
  std::array<double,3> target{};
  // Ruckig's velocity constraints are per axis. Use a box inscribed in the
  // planar speed circle so changing both body axes during yaw correction
  // cannot exceed the Euclidean cap. A pure-axis approach is consequently
  // limited to max_linear_speed/sqrt(2); acceleration/jerk remain unchanged.
  const double axis_speed=cfg.max_linear_speed/std::sqrt(2.);
  if(!stopping) {
    const double dx=goal.x-current.x,dy=goal.y-current.y,distance=std::hypot(dx,dy);
    const double angle=std::remainder(goal.yaw-current.yaw,2.*M_PI);
    state.xy_held=distance<=(state.xy_held?cfg.resume_tolerance_ratio:cfg.stop_tolerance_ratio)*cfg.xy_goal_tolerance;
    state.yaw_held=std::abs(angle)<=(state.yaw_held?cfg.resume_tolerance_ratio:cfg.stop_tolerance_ratio)*cfg.yaw_goal_tolerance;
    if(!state.xy_held) {
      const double speed=std::min(std::max(cfg.min_linear_speed,cfg.kp_xy*distance),referenceSpeedForDistance(
        distance,cfg.max_linear_speed,cfg.normal_acceleration,cfg.normal_jerk));
      target[0]=speed*(std::cos(current.yaw)*dx+std::sin(current.yaw)*dy)/distance;
      target[1]=speed*(-std::sin(current.yaw)*dx+std::cos(current.yaw)*dy)/distance;
      target[0]=std::clamp(target[0],-axis_speed,axis_speed);
      target[1]=std::clamp(target[1],-axis_speed,axis_speed);
    }
    if(!state.yaw_held)target[2]=std::copysign(std::min(std::max(cfg.min_angular_speed,cfg.kp_yaw*std::abs(angle)),
      referenceSpeedForDistance(std::abs(angle),cfg.max_angular_speed,
        cfg.normal_angular_acceleration,cfg.normal_angular_jerk)),angle);
  }
  ruckig::InputParameter<3> input;ruckig::OutputParameter<3> output;
  input.control_interface=ruckig::ControlInterface::Velocity;
  input.synchronization=ruckig::Synchronization::Phase;
  input.current_velocity=state.velocity;input.current_acceleration=state.acceleration;
  input.target_velocity=target;input.target_acceleration={0.,0.,0.};
  input.max_velocity={axis_speed,axis_speed,cfg.max_angular_speed};
  input.max_acceleration={cfg.normal_acceleration,cfg.normal_acceleration,cfg.normal_angular_acceleration};
  input.max_jerk={cfg.normal_jerk,cfg.normal_jerk,cfg.normal_angular_jerk};
  ruckig::Ruckig<3> otg(dt);
  if(otg.update(input,output)<0)throw std::runtime_error("WORKSTATION_MOTION_MODEL_ERROR");
  state.velocity=output.new_velocity;state.acceleration=output.new_acceleration;
  for(std::size_t i=0;i<3;++i)if(target[i]==0.&&std::abs(state.velocity[i])<1e-10&&std::abs(state.acceleration[i])<1e-9)
    state.velocity[i]=state.acceleration[i]=0.;
  return state.velocity;
}
WorkstationPose workstationIntegrate(const WorkstationPose &p,const std::array<double,3> &v,double dt) {
  const double w=v[2],a=std::abs(w)<1e-10?dt:std::sin(w*dt)/w;
  const double b=std::abs(w)<1e-10?0.:(1.-std::cos(w*dt))/w;
  const double dx=a*v[0]-b*v[1],dy=b*v[0]+a*v[1];
  return {p.x+std::cos(p.yaw)*dx-std::sin(p.yaw)*dy,
    p.y+std::sin(p.yaw)*dx+std::cos(p.yaw)*dy,std::remainder(p.yaw+w*dt,2.*M_PI)};
}
bool workstationStoppingClear(const LayeredCollisionSnapshot &map,const WorkstationPose &current,
    WorkstationMotionState state,const WorkstationConfig &cfg) {
  auto pose=current;const double dt=1./cfg.control_frequency;
  if(!sweepClear(map,pose,state.velocity,dt))return false;
  pose=workstationIntegrate(pose,state.velocity,dt);
  for(double t=0.;t<cfg.refine_timeout;t+=dt) {
    if(zeroMotion(state))return !map.collision(pose.x,pose.y,pose.yaw);
    const auto cmd=workstationCommand(pose,pose,state,cfg,dt,true);
    if(!sweepClear(map,pose,cmd,dt))return false;
    pose=workstationIntegrate(pose,cmd,dt);
  }
  throw std::runtime_error("WORKSTATION_STOP_MODEL_BUDGET");
}
WorkstationEvaluation evaluateWorkstationAlignment(const LayeredCollisionSnapshot &map,
    const WorkstationPose &start,const WorkstationPose &goal,const WorkstationConfig &cfg) {
  WorkstationEvaluation result;WorkstationMotionState state;auto pose=start;
  const double dt=1./cfg.control_frequency;
  try {
    if(map.collision(goal.x,goal.y,goal.yaw)) {result.reason="WORKSTATION_GOAL_LAYER_COLLISION_OR_UNKNOWN";return result;}
    for(;result.prediction_s<cfg.refine_timeout;result.prediction_s+=dt,++result.steps) {
      const auto cmd=workstationCommand(pose,goal,state,cfg,dt);
      if(!workstationStoppingClear(map,pose,state,cfg)) {
        result.reason="WORKSTATION_SWEEP_COLLISION_OR_UNKNOWN";return result;
      }
      if(state.xy_held&&state.yaw_held&&zeroMotion(state)) {
        result.clear=true;result.reason="WORKSTATION_ALIGNMENT_CLEAR";return result;
      }
      pose=workstationIntegrate(pose,cmd,dt);
    }
    result.reason="WORKSTATION_PREDICTION_BUDGET";
  } catch(const std::exception &error) {result.reason=error.what();}
  return result;
}
} // namespace astribot_s1_path_tracking
