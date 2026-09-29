#include "workstation_alignment_fixture.hpp"
#include "astribot_s1_path_tracking/arrival_settling.hpp"
#include <cassert>
#include <iostream>
using namespace astribot_s1_path_tracking;
int main() {
  WorkstationConfig cfg;const double dt=1./cfg.control_frequency;
  {
    WorkstationConfig fine=cfg;fine.min_linear_speed=.008;fine.min_angular_speed=.02;
    // Scene78 stopped outside the goal while proportional requests fell below
    // the effective wheel-speed range. Check the fixed-pose steady request.
    const WorkstationPose goal{.000488,.003701,.006222};
    WorkstationMotionState state;std::array<double,3> command{};
    for(int i=0;i<100;++i)command=workstationCommand({},goal,state,fine,dt);
    assert(std::abs(std::hypot(command[0],command[1])-fine.min_linear_speed)<1e-9);
    assert(std::abs(command[2]-fine.min_angular_speed)<1e-9);
    // A policy cap below the configured floor still wins, without throwing.
    fine.max_linear_speed=.001;fine.max_angular_speed=.002;state={};
    for(int i=0;i<100;++i) {
      command=workstationCommand({},goal,state,fine,dt);
      assert(std::hypot(command[0],command[1])<=fine.max_linear_speed+1e-9);
      assert(std::abs(command[2])<=fine.max_angular_speed+1e-9);
    }
    state={};command=workstationCommand(goal,goal,state,fine,dt);
    assert((state.xy_held&&state.yaw_held&&command==std::array<double,3>{}));
  }
  for(bool minimum:{false,true}) {
    auto fine=cfg;if(minimum) {fine.min_linear_speed=.008;fine.min_angular_speed=.02;}
    const WorkstationPose goal{.000488,.003701,.006222};WorkstationPose pose{};WorkstationMotionState state;
    int stopped=0;double elapsed=0.;
    for(;elapsed<fine.refine_timeout;elapsed+=dt) {
      const auto command=workstationCommand(pose,goal,state,fine,dt);
      // Scene78 wheel coefficients/radius and friction-feedforward threshold.
      // This quasi-static dead zone omits PID integral, inertia and contacts;
      // it is a low-speed regression model, not physical simulation evidence.
      constexpr double radius=.08,a=.7071;
      std::array<double,4> wheels{
        (-a*command[0]-a*command[1]-.306*command[2])/radius,
        ( a*command[0]-a*command[1]-.306*command[2])/radius,
        (-a*command[0]+a*command[1]-.3024*command[2])/radius,
        ( a*command[0]+a*command[1]-.3024*command[2])/radius};
      for(auto &wheel:wheels)if(std::abs(wheel)<=.05)wheel=0.;
      std::array<double,3> actual{};
      actual[0]=radius*(-wheels[0]+wheels[1]-wheels[2]+wheels[3])/(4.*a);
      actual[2]=-radius*(wheels[0]+wheels[1]+wheels[2]+wheels[3])/(2.*(.306+.3024));
      actual[1]=radius*(wheels[2]+wheels[3]-wheels[0]-wheels[1])/(4.*a)-(.306-.3024)*actual[2]/(2.*a);
      pose=workstationIntegrate(pose,actual,dt);
      const bool zero=std::hypot(command[0],command[1])<1e-9&&std::abs(command[2])<1e-9;
      stopped=zero&&state.xy_held&&state.yaw_held?stopped+1:0;
      if(stopped>=20)break; // One full second of zero request without a correction cycle.
    }
    const double xy=std::hypot(goal.x-pose.x,goal.y-pose.y),yaw=std::abs(goal.yaw-pose.yaw);
    if(minimum) {
      assert(stopped>=20&&elapsed<5.);
      assert(xy<=fine.xy_goal_tolerance&&yaw<=fine.yaw_goal_tolerance);
    } else {
      assert(stopped==0&&elapsed>=fine.refine_timeout);
      assert(std::abs(xy-std::hypot(goal.x,goal.y))<1e-12&&std::abs(yaw-goal.yaw)<1e-12);
    }
    std::cout<<"scene78 low-speed approximation: minimum="<<minimum<<" stopped="<<(stopped>=20)
      <<" seconds="<<elapsed<<" xy="<<xy<<" yaw="<<yaw<<"\n";
  }
  for(const auto &goal:std::vector<WorkstationPose>{{.4,.2,1.4},{-.35,.1,-1.2},{.25,.15,-M_PI+.02},{0.,0.,M_PI/2.}}) {
    workstation_test::Geometry geometry;auto map=geometry.snapshot();WorkstationPose pose{0.,0.,0.};
    const auto predicted=evaluateWorkstationAlignment(map,pose,goal,cfg);
    if(!predicted.clear)throw std::runtime_error(predicted.reason);
    WorkstationMotionState state;std::size_t i=0;
    for(;i<10000;++i) {
      const auto before=state;
      const auto command=workstationCommand(pose,goal,state,cfg,dt);
      assert(std::hypot(command[0],command[1])<=cfg.max_linear_speed+1e-8);
      assert(std::abs(command[2])<=cfg.max_angular_speed+1e-8);
      for(std::size_t axis=0;axis<3;++axis) {
        const double a=axis==2?cfg.normal_angular_acceleration:cfg.normal_acceleration;
        const double j=axis==2?cfg.normal_angular_jerk:cfg.normal_jerk;
        assert(std::abs(state.acceleration[axis])<=a+1e-8);
        assert(std::abs(state.acceleration[axis]-before.acceleration[axis])<=j*dt+1e-8);
      }
      assert(workstationStoppingClear(map,pose,state,cfg));
      if(state.xy_held&&state.yaw_held&&std::hypot(command[0],command[1])<1e-9&&std::abs(command[2])<1e-9)break;
      pose=workstationIntegrate(pose,command,dt);
    }
    assert(i==predicted.steps);
    assert(std::hypot(pose.x-goal.x,pose.y-goal.y)<=cfg.xy_goal_tolerance);
    assert(std::abs(std::remainder(pose.yaw-goal.yaw,2.*M_PI))<=cfg.yaw_goal_tolerance);
  }
  {
    workstation_test::Geometry f;f.obstacle(0,.4,0.);
    assert(evaluateWorkstationAlignment(f.snapshot(),{0.,0.,0.},{.2,0.,0.},cfg).clear);
    f.obstacle(1,.4,0.);
    assert(!evaluateWorkstationAlignment(f.snapshot(),{0.,0.,0.},{.2,0.,0.},cfg).clear);
  }
  for(int8_t value:{int8_t(100),int8_t(-1)}) {
    workstation_test::Geometry f;f.obstacle(0,.25,0.,value);
    assert(!f.snapshot().collision(0.,0.,0.)&&!f.snapshot().collision(.5,0.,0.));
    assert(!evaluateWorkstationAlignment(f.snapshot(),{0.,0.,0.},{.5,0.,0.},cfg).clear);
  }
  {
    workstation_test::Geometry f;f.obstacle(1,.20,.20);
    assert(!f.snapshot().collision(0.,0.,0.)&&!f.snapshot().collision(0.,0.,M_PI/2.));
    assert(!evaluateWorkstationAlignment(f.snapshot(),{0.,0.,0.},{0.,0.,M_PI/2.},cfg).clear);
  }
  {
    workstation_test::Geometry f;f.obstacle(0,.12,0.);WorkstationMotionState moving;moving.velocity[0]=.06;
    assert(!f.snapshot().collision(0.,0.,0.));
    assert(!workstationStoppingClear(f.snapshot(),{0.,0.,0.},moving,cfg));
  }
  {
    workstation_test::Geometry f;
    assert(!evaluateWorkstationAlignment(f.snapshot(),{0.,0.,0.},{1.9,0.,0.},cfg).clear);
    WorkstationMotionState state;
    const auto v=workstationCommand({.25,.15,M_PI/2.},{.45,.15,M_PI/2.},state,cfg,dt);
    assert(std::abs(v[0])<1e-8&&v[1]<0.); // map +X is body -Y, no odometry input.
  }
  {
    ArrivalSettling<2> xy;ArrivalSettling<1> yaw;xy.command(true,0.);yaw.command(true,0.);
    for(int i=0;i<=20;++i) {
      const double t=i*.05,e=i%2?1e-4:-1e-4;
      xy.observe(100.+t,t,{e,-e},cfg.settle_time,cfg.stopped_linear_velocity,cfg.settle_drift_ratio*cfg.xy_goal_tolerance);
      yaw.observe(100.+t,t,{std::remainder(M_PI+e*.5,2.*M_PI)},cfg.settle_time,cfg.stopped_angular_velocity,cfg.settle_drift_ratio*cfg.yaw_goal_tolerance,true);
    }
    assert(xy.evidence().stopped&&yaw.evidence().stopped);
    for(int i=21;i<=40;++i)xy.observe(100.+i*.05,i*.05,{.001*(i-20),0.},cfg.settle_time,cfg.stopped_linear_velocity,cfg.settle_drift_ratio*cfg.xy_goal_tolerance);
    assert(!xy.evidence().stopped);
  }
  std::cout<<"workstation core: shared coupled rollout, limits, layered obstacles/unknown, stopping sweep, map yaw and signed settling passed\n";
}
