#include "astribot_s1_robot_geometry/filled_collision.hpp"
#include <iostream>
#include <stdexcept>
using namespace astribot_s1_robot_geometry;
void check(bool b,const char * msg) {if(!b) throw std::runtime_error(msg);}
int main() {
  nav2_costmap_2d::Costmap2D map(200,200,.01,-1.,-1.,0);
  Polygon p;for(auto xy:std::vector<std::pair<double,double>>{{-.3,-.2},{.7,-.2},{.7,.4},{-.3,.4}}) {geometry_msgs::msg::Point q;q.x=xy.first;q.y=xy.second;p.push_back(q);}
  check(!collision(map,p,0,0,0),"empty map");
  unsigned int x,y;map.worldToMap(.15,.1,x,y);map.setCost(x,y,254);
  check(collision(map,p,0,0,0),"obstacle wholly inside polygon");map.setCost(x,y,0);
  map.worldToMap(.0,.43,x,y);map.setCost(x,y,254);
  check(!collision(map,p,0,0,0),"separated");check(collision(map,p,0,0,0,.04),"interval sampling bound");map.setCost(x,y,0);
  map.worldToMap(.1,.1,x,y);map.setCost(x,y,255);check(collision(map,p,0,0,0),"unknown interior");map.setCost(x,y,0);
  check(collision(map,p,.5,0,0),"map boundary");
  check(collision(map,Polygon{},0,0,0),"invalid empty geometry");
  std::cout<<"PASS: filled interior, unknown, sampling bound, asymmetric polygon, map boundary, invalid geometry\n";
}
