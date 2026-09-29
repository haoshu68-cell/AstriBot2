#include <cassert>
#include "corner_fixture.hpp"
using namespace astribot_s1_path_tracking;
bool zero(const geometry_msgs::msg::Twist &v) {return v.linear.x==0.&&v.linear.y==0.&&v.angular.z==0.;}
int main() {
  ArrivalController c;ArrivalControllerTestPeer::setup(c);ArrivalControllerTestPeer::fixedPassage(c);
  // Entrance corners would fail ordinary geometry admission. Classification
  // must happen first; no ordinary controller plan is installed yet.
  c.setPlan(path({{-1.6,.15},{-1.25,-.0725},{-.4,-.0725},{.4,-.0725},{1.09,-.0725},{1.14,-.0725},{1.19,-.0725},{1.4,-.0725}},1));
  assert(ThreePhaseControllerTestPeer::segment(c).poses.empty());
  auto tick=[&](std::string phase,double now,double x,double y,double yaw=0.,bool sample=true,bool matching=true) {
    return ArrivalControllerTestPeer::passageTick(c,phase,now,x,y,yaw,sample,matching,-.0725);
  };
  assert(zero(tick("ALIGN",1.,-1.6,.15,.1745).second));
  assert(zero(tick("ALIGN",2.,-1.6,.15,.1745,false).second)); // no new SLAM
  assert(zero(tick("ALIGN",2.1,-1.6,.15,.1745).second));
  auto align=tick("ALIGN",2.8,-1.6,.15,.1745).second;
  assert(align.angular.z<0.&&align.linear.x==0.&&align.linear.y==0.);
  assert(zero(tick("CENTER",3.,-1.6,.15).second));
  assert(zero(tick("CENTER",3.3,-1.6,.14).second)); // still moving in SLAM
  assert(zero(tick("CENTER",3.7,-1.6,.13).second));
  tick("CENTER",4.,-1.6,.13);tick("CENTER",4.4,-1.6,.13);
  auto center=tick("CENTER",4.8,-1.6,.13).second;
  assert(center.linear.y<0.&&center.linear.x==0.&&center.angular.z==0.);
  assert(zero(tick("HOLD",5.,-1.6,.1).second));
  assert(ArrivalControllerTestPeer::passagePhase(c)=="CENTER");
  assert(tick("CENTER",5.1,-1.6,.1).second.linear.y<0.);
  assert(zero(tick("TRANSIT",6.,-1.6,-.0725).second));
  assert(zero(tick("TRANSIT",6.1,-1.6,-.0725).second));
  auto moving=tick("TRANSIT",6.8,-1.6,-.0725).second;
  assert(moving.linear.x>0.&&moving.linear.y==0.&&moving.angular.z==0.);
  assert(ThreePhaseControllerTestPeer::segment(c).poses.empty());
  assert(zero(tick("TRANSIT",6.9,-1.,-.0725,0.,true,false).second));
  assert(ArrivalControllerTestPeer::passagePhase(c)=="TRANSIT");
  assert(tick("TRANSIT",7.,.7,-.0725).first); // base past exit, still owned
  assert(zero(tick("HOLD",7.1,.7,-.0725).second));
  assert(ArrivalControllerTestPeer::passagePhase(c)=="TRANSIT");
  assert(tick("TRANSIT",7.2,.7,-.0725).second.linear.x>0.);
  assert(ThreePhaseControllerTestPeer::segment(c).poses.empty());
  c.setPlan(path({{.7,-.0725},{1.09,-.0725},{1.14,-.0725},{1.19,-.0725},{1.4,-.0725}},2));
  assert(ArrivalControllerTestPeer::passagePhase(c)=="TRANSIT");
  assert(ThreePhaseControllerTestPeer::segment(c).poses.empty());
  assert(zero(tick("NORMAL",8.,1.1,-.0725).second));
  assert(zero(tick("NORMAL",8.1,1.1,-.0725).second));
  auto released=tick("NORMAL",8.8,1.1,-.0725);
  assert(!released.first&&ArrivalControllerTestPeer::passagePhase(c).empty());
  const auto remainder=ThreePhaseControllerTestPeer::segment(c);
  assert(remainder.poses.front().pose.position.x>=1.1-1e-9);
  assert(remainder.poses[1].pose.position.x>remainder.poses.front().pose.position.x);
  assert(std::abs(remainder.poses.front().pose.position.y+.0725)<1e-9);
  // Once passage ownership ends, a replacement must invalidate the old
  // terminal refinement exactly as it does in ordinary Arrival navigation.
  ArrivalControllerTestPeer::refine(c);
  ThreePhaseControllerTestPeer::observed(c,1.1,-.0725);
  c.setPlan(path({{1.1,.01},{1.4,-.0725}},3));
  tick("NORMAL",9.,1.1,-.0725);
  assert(!ArrivalControllerTestPeer::refining(c));
  // Normal routes still get the original corner validation.
  ArrivalController normal;ArrivalControllerTestPeer::setup(normal);ArrivalControllerTestPeer::fixedPassage(normal);
  normal.setPlan(path({{0.,0.},{.01,0.},{.01,.01},{1.,.01}},1));
  bool rejected=false;
  try {ArrivalControllerTestPeer::passageTick(normal,"NORMAL",1.,0.,0.);} catch(const nav2_core::PlannerException &) {rejected=true;}
  assert(rejected);
  // A blocked initial turn in a recognized passage follows the path with the
  // measured body heading, including negative and lateral chassis commands.
  for(bool sideways:{false,true}) {
    ArrivalController translation;ArrivalControllerTestPeer::setup(translation);
    ArrivalControllerTestPeer::fixedPassage(translation);
    auto route=sideways?path({{0.,0.},{0.,.5},{0.,1.4},{0.,1.8}},1):
      path({{0.,0.},{-.5,0.},{-1.4,0.},{-1.8,0.}},1);
    for(auto &p:route.poses){p.pose.orientation.z=0.;p.pose.orientation.w=1.;}
    translation.setPlan(route);
    auto run=[&](const char *phase,double at){return ArrivalControllerTestPeer::passageTick(translation,phase,at,0.,0.);};
    assert(zero(run("TRANSLATE",1.).second));
    assert(zero(run("TRANSLATE",1.1).second));
    const auto command=run("TRANSLATE",1.8).second;
    assert(command.angular.z==0.);
    assert(sideways?(command.linear.y>0.&&command.linear.x==0.):(command.linear.x<0.&&command.linear.y==0.));
    assert(zero(run("HOLD",1.9).second));
    assert(ArrivalControllerTestPeer::passagePhase(translation)=="TRANSLATE");
    assert(!zero(run("TRANSLATE",2.).second));
  }
  // Lifecycle stop ends the owned maneuver; the next execution reacquires it.
  tick("ALIGN",10.,-1.6,.15,.1745);tick("ALIGN",10.1,-1.6,.15,.1745);
  tick("ALIGN",10.8,-1.6,.15,.1745);
  assert(ArrivalControllerTestPeer::passagePhase(c)=="ALIGN");
  c.deactivate();
  assert(ArrivalControllerTestPeer::passagePhase(c).empty());
  std::cout<<"passage ownership, SLAM transfer, pause, exit and normal admission passed\n";
}
