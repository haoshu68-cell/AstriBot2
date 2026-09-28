#include "astribot_s1_path_tracking/corner_sweep.hpp"
#include <iostream>
int main() {
 using namespace astribot_s1_path_tracking;
 nav2_costmap_2d::Costmap2D map(500,500,.02,-5,-5,0);
 astribot_s1_robot_geometry::Polygon p(4);
 p[0].x=.45;p[0].y=.2;p[1].x=-.45;p[1].y=.2;
 p[2].x=-.45;p[2].y=-.2;p[3].x=.45;p[3].y=-.2;
 int failed=0;auto check=[&](bool ok,const char* why){if(!ok){++failed;std::cerr<<why<<'\n';}};
 for(double turn:{-2.356,-1.571,-.785,.785,1.571,2.356})check(cornerRotationClear(map,p,0,0,0,turn),"empty sweep");
 for(double side:{-1.,1.}) {
  unsigned int x,y;map.worldToMap(.3,side*.3,x,y);map.setCost(x,y,254);
  check(!astribot_s1_robot_geometry::collision(map,p,0,0,0),"initial footprint clear");
  check(!astribot_s1_robot_geometry::collision(map,p,0,0,side*M_PI/2),"final footprint clear");
  check(!cornerRotationClear(map,p,0,0,0,side*M_PI/2),"intermediate sweep occupied");
  map.setCost(x,y,255);check(!cornerRotationClear(map,p,0,0,0,side*M_PI/2),"unknown sweep blocked");map.setCost(x,y,0);
 }
 check(!cornerRotationClear(map,p,4.8,4.8,0,M_PI/2),"outside costmap blocked");
 check(!cornerRotationClear(map,p,0,0,NAN,0),"invalid angle blocked");
 check(!cornerRotationClear(map,{},0,0,0,M_PI/2),"missing footprint blocked");
 unsigned int mx,my;map.worldToMap(.48,0.,mx,my);map.setCost(mx,my,254);
 check(!astribot_s1_robot_geometry::collision(map,p,0,0,0),"translation sweep starts clear");
 check(!cornerCommandClear(map,p,0,0,0,.04,0.,0.),"position hold translation must include filled sweep");
 map.setCost(mx,my,0);
 check(cornerCommandClear(map,p,0,0,0,.04,.01,.6),"empty combined translation and turn");
 check(!cornerCommandClear(map,p,0,0,0,NAN,0.,0.),"nonfinite command rejected");
 return failed?1:0;
}
