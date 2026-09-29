#include <cassert>
#include "astribot_s1_robot_geometry/passage_sweep.hpp"
using namespace astribot_s1_robot_geometry;
int main() {
  Polygon2 arm{{-.4,-.368},{.4,-.368},{.4,.513},{-.4,.513}};
  auto center=passageSweep(arm,{-1.6,.15,0.},{-1.6,-.0725,0.},.155);
  // A remote obstacle in the passage cannot block the entrance translation.
  assert(boxDistances(center,{{0,0,0,.16,.51,.29,.64}})[0]>0.);
  // Actual asymmetric side sweep and margin remain protected.
  assert(boxDistances(center,{{0,0,0,-1.7,.72,-1.5,.74}})[0]<=0.);
  auto transit=passageSweep(arm,{-.6,-.0725,0.},{.6,-.0725,0.},.155);
  assert(boxDistances(transit,{{0,0,0,-.1,-.05,.1,.05}})[0]<=0.);
  assert(boxDistances(transit,{{0,0,0,-.6,.625,.6,.725}})[0]>0.);
  Polygon2 long_arm{{-.9,-.1},{.9,-.1},{.9,.1},{-.9,.1}};
  auto turn=passageSweep(long_arm,{0,0,0},{0,0,1.57},.01);
  assert(boxDistances(turn,{{0,0,0,.6,.6,.65,.65}})[0]<=0.);
  std::vector<int> map(10000,0);
  assert(passageGridClear(center,{-5,-5,0},.1,100,100,map));
  map[50*100+34]=-1; // unknown beneath the filled envelope
  assert(!passageGridClear(center,{-5,-5,0},.1,100,100,map));
  assert(!passageGridClear(center,{-1,-1,0},.1,100,100,map));
}
