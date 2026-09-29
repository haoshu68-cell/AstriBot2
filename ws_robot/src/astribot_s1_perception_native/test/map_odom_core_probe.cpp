#include <cassert>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include "astribot_s1_perception_native/map_odom_core.hpp"
using namespace astribot_s1_perception_native::map_odom;
double number(std::istringstream &in) { std::string s; in >> s; return std::stod(s); }
Pose2D pose(std::istringstream &in) {
  const auto x=number(in), y=number(in), theta=number(in);
  return Pose2D(x,y,theta);
}
int main(int argc, char **) {
  if (argc > 1) {
    Decomposer d;
    auto r=d.update(Pose2D(3,4,0),Pose2D(1,2,0));
    assert(r.pose.x==2 && r.pose.y==2 && r.pose.theta==0);
    assert(!r.jumped && d.stats.updates==1);
    assert(!checkSourceAge(2,1,1).stale);
    assert(checkSourceAge(2.01,1,1).stale);
    assert(!checkSourceAge(0,0,0).stale);
    assert(d.checkPlanar(.2,0));
    assert(d.stats.rejected_tilt==1 && d.stats.updates==1);
    return 0;
  }
  Decomposer d;
  std::string line;
  std::cout << std::setprecision(17);
  while(std::getline(std::cin,line)) {
    try {
      std::istringstream in(line); std::string op; in >> op;
      if(op=="age") {
        const auto now=number(in), stamp=number(in), max=number(in);
        const auto r=checkSourceAge(now,stamp,max);
        std::cout << "age " << r.age_sec << ' ' << r.stale << '\n';
      } else if(op=="reset") {
        const auto jump=number(in), tilt=number(in); d=Decomposer(jump,tilt);
        std::cout << "reset\n";
      } else if(op=="tilt") {
        const auto x=number(in),y=number(in); const auto result=d.checkPlanar(x,y);
        std::cout << "tilt " << result << ' ' << d.stats.rejected_tilt << '\n';
      } else if(op=="update") {
        const auto a=pose(in), b=pose(in); auto r=d.update(a,b);
        std::cout << "update " << r.pose.x << ' ' << r.pose.y << ' ' << r.pose.theta
          << ' ' << r.jumped << ' ' << r.jump_m << ' ' << d.stats.updates
          << ' ' << d.stats.jumps << ' ' << d.stats.max_jump_m << '\n';
      } else if(op=="yaw") {
        const auto z=number(in), w=number(in); std::cout << "yaw " << yawFromQuaternion(z,w) << '\n';
      } else if(op=="quat") {
        const auto q=quaternionFromYaw(number(in)); std::cout << "quat " << q.first << ' ' << q.second << '\n';
      } else if(op=="wrap") { std::cout << "wrap " << wrapAngle(number(in)) << '\n'; }
      else throw std::invalid_argument("unknown probe command");
    } catch(const std::exception &) { std::cout << "error\n"; }
  }
}
