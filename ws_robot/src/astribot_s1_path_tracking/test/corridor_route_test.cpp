#include <iostream>
#include <limits>
#include "astribot_s1_path_tracking/corridor_route.hpp"
int main() {
  using namespace astribot_s1_path_tracking;
  int failed=0;
  auto check=[&](bool value,const char * text){if(!value){std::cerr<<text<<'\n';++failed;}};
  RoutePoint entry{.6,0.},exit{-.6,0.};
  check(pathTraversesCorridor({{1.4,0.},{0.,0.},{-1.4,0.}},entry,exit,1.3,true),"aligned transit");
  check(!pathTraversesCorridor({{1.4,0.},{.7,1.7},{-.7,1.7},{-1.4,0.}},entry,exit,1.3,true),"recorded outside-detour pattern");
  check(!pathTraversesCorridor({{1.4,1.},{-1.4,1.}},entry,exit,1.3,true),"sparse segment cannot skip wall strip");
  check(!pathTraversesCorridor({{1.4,0.},{0.,.65},{-1.4,0.}},entry,exit,1.3,true),"boundary touch rejected");
  check(!pathTraversesCorridor({{.2,0.},{-1.4,0.}},entry,exit,1.3,true),"initial entry required");
  check(pathTraversesCorridor({{.2,0.},{-1.4,0.}},entry,exit,1.3,false),"replan from inside");
  check(pathTraversesCorridor({{-.8,0.},{-1.4,0.}},entry,exit,1.3,false),"replan after exit");
  check(!pathTraversesCorridor({{1.4,0.},{0.,0.}},entry,exit,1.3,true),"inside stop is not through transit");
  check(!pathTraversesCorridor({{1.4,0.},{-.1,0.},{.3,0.},{-1.4,0.}},entry,exit,1.3,true),"no reversal inside selected transit");
  check(!pathTraversesCorridor({{1.4,0.},{std::numeric_limits<double>::quiet_NaN(),0.}},entry,exit,1.3,true),"invalid point");
  check(pathTraversesCorridor({{0.,-1.4},{0.,0.},{0.,1.4}},{0.,-.6},{0.,.6},1.3,true),"rotated corridor");
  return failed?1:0;
}
