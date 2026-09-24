#include "astribot_s1_autonomy/slice_projector.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace astribot_s1_autonomy;
int main() {
  SliceProjector projector;SliceProjector::Params p;
  p.angle_min=-M_PI;p.angle_max=M_PI;p.angle_increment=M_PI/180;
  p.range_min=.35;p.range_max=20;p.no_return_value=INFINITY;
  const double edges[]={.05,.25,.68,1.18,1.63,2.30};
  for(int i=0;i<5;++i)p.slices.push_back({std::to_string(i),edges[i],edges[i+1],1,20,true});
  std::string error;assert(projector.configure(p,error));ProjectionResult output;
  projector.project({{10,0,.1f},{15,0,.1f},{7,0,.5f},{5,0,.9f},{3,0,1.4f},{2,0,2.f}},output);
  assert(output.ranges.at(180)==2.f);
  const float expected[]={10,7,5,3,2};
  for(int i=0;i<5;++i)assert(output.per_slice_ranges.at(i).at(180)==expected[i]);
  assert(std::isinf(output.ranges.at(90)));
  std::cout<<"PASS: same-angle nearest across five layers; empty bucket remains unknown/no-return\n";
}
