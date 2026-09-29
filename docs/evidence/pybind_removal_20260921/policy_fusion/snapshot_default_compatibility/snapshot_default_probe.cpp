#include "astribot_s1_robot_geometry/fusion_snapshot.hpp"
#include <iomanip>
#include <iostream>
using namespace astribot_s1_robot_geometry;
int main(){std::cout<<std::setprecision(17);for(int scenario=0;scenario<12;++scenario){SnapshotParams p;p.has_region=scenario%2;p.region_x=.2;p.region_y=-.4;p.region_travel=scenario*.3;
std::vector<SnapshotTrackInput> inputs;for(int i=0;i<250;++i){SnapshotTrackInput in;in.capture_ns=i*1000000;in.center[0]=(i%19)*.2-2.;in.center[1]=(i%17)*.17-1.;in.center[2]=.5;
in.size[0]=.2+(i%3)*.01;in.size[1]=.3;in.size[2]=1.;in.pos_cov[0]=.01;in.pos_cov[4]=.02;in.pos_cov[8]=.03;
in.track_velocity[0]=i%7?0.:.12;in.track_velocity[1]=i%9?0.:-.3;in.spatial_occupancy=i%11==0;in.velocity_observable=i%3;in.has_velocity_m_s=i%5==0;in.has_velocity_covariance=i%5==0;
in.velocity_covariance[0]=.001;in.velocity_covariance[4]=.002;in.velocity_covariance[8]=.003;
for(int j=0;j<i%12;++j)in.samples.push_back({j*.1,in.center[0]+(i%2?j*.01:0.),in.center[1]});inputs.push_back(in);}
std::vector<SnapshotTrackOutput> out;snapshotTracks(inputs,scenario*300000000LL,p,out);for(const auto& o:out){for(double x:o.center)std::cout<<x<<' ';for(double x:o.size)std::cout<<x<<' ';for(double x:o.covariance)std::cout<<x<<' ';for(double x:o.velocity)std::cout<<x<<' ';std::cout<<o.variance<<' '<<o.relevant<<'\n';}}}
