#include "corner_fixture.hpp"
#include <algorithm>
#include <deque>
#include <fstream>

// Deterministic approach-only surrogate, not Gazebo/MPPI or a hardware model.
int main(int argc,char ** argv) {
  using namespace astribot_s1_path_tracking;
  using Peer=ThreePhaseControllerTestPeer;
  if(argc!=2) {return 2;}
  std::ofstream out(argv[1]);
  out<<"yaw_deg,delay_s,initial_speed_mps,accepted,elapsed_s,actual_corner_error_m,actual_speed_mps\n";
  int failures=0,cases=0;
  for(double yaw : {0.,M_PI/2.,M_PI,-M_PI/2.}) {
    for(double delay : {0.,.10,.20,.35}) {
      for(double initial : {0.,.10,.20}) {
        ArrivalController controller;ArrivalControllerTestPeer::setup(controller);
        ArrivalControllerTestPeer::referenceBrake(controller);Peer::plantInit(controller,yaw);
        double position=.6,speed=initial,source_position=position,stamp=10.,elapsed=0.;
        std::deque<std::pair<double,double>> commands;
        bool accepted=false;
        for(int tick=0;tick<300;++tick) {
          const double now=10.+tick*.05;elapsed=tick*.05;
          // 10 Hz source / 20 Hz control: duplicate frames must not count twice.
          if(tick%2==0) {source_position=position;stamp=now;}
          const auto command=Peer::plantStep(controller,now,stamp,source_position,yaw,speed);
          commands.push_back({now,command.linear.x});
          if(controller.phase()==Phase::kAlignCorner) {accepted=true;break;}
          double target=initial;
          for(const auto & sample:commands) {if(sample.first<=now-delay+1e-9) {target=sample.second;}}
          speed+=std::clamp(target-speed,-.25*.05,.25*.05);
          position+=speed*.05;
        }
        ++cases;
        const double error=std::abs(1.-position);
        const bool ok=accepted && error<=.04+1e-9 && std::abs(speed)<=.01;
        if(!ok) {++failures;std::cerr<<"FAIL plant yaw="<<yaw<<" delay="<<delay<<" initial="<<initial
          <<" error="<<error<<" speed="<<speed<<" accepted="<<accepted<<'\n';}
        out<<yaw*180./M_PI<<','<<delay<<','<<initial<<','<<accepted<<','<<elapsed<<','<<error<<','<<speed<<'\n';
      }
    }
  }
  std::cout<<cases<<" offline approach surrogate cases; failures="<<failures<<'\n';
  return failures?1:0;
}
