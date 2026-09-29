#include "astribot_s1_path_tracking/path_quality.hpp"
#include "astribot_s1_path_tracking/align_math.hpp"
#include <cassert>
#include <iostream>

using namespace astribot_s1_path_tracking;

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.w=1.;return p;
}

nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses)
{
  nav_msgs::msg::Path p;p.header.frame_id="map";p.header.stamp.sec=17;p.poses=poses;return p;
}

CornerGeometry issue(const nav_msgs::msg::Path & p)
{
  std::vector<PlanarPoint> points;
  for(const auto & v:p.poses) {points.push_back({v.pose.position.x,v.pose.position.y});}
  return inspectCornerGeometry(points,.15,.4363323129985824,2.6179938779914944).kind;
}

int main()
{
  for(double sign:{-1.,1.}) {
    auto incoming=path({pose(0.,0.),pose(1.15,-sign*.00475116),pose(1.2,0.)});
    auto outgoing=path({pose(1.195,-sign*.025),pose(1.195,sign*.025),
                       pose(1.195,sign*.5),pose(1.2,sign*1.2)});
    outgoing.poses.front().pose.orientation.z=std::sin(sign*M_PI/4.);
    outgoing.poses.front().pose.orientation.w=std::cos(sign*M_PI/4.);
    auto joined=incoming;joined.poses.insert(joined.poses.end(),outgoing.poses.begin(),outgoing.poses.end());
    assert(issue(joined)==CornerGeometry::Turnaround);
    const auto end=outgoing.poses.back();const auto interior=outgoing.poses[1];
    const auto travel_heading=outgoing.poses.front().pose.orientation;
    anchorGridPathStart(outgoing,incoming.poses.back(),.05);
    assert(outgoing.poses.front().pose.position==incoming.poses.back().pose.position);
    assert(outgoing.poses.front().pose.orientation==travel_heading);
    assert(outgoing.poses.front().header==outgoing.header);
    assert(outgoing.poses.back()==end && outgoing.poses[1]==interior);
    joined=incoming;joined.poses.insert(joined.poses.end(),outgoing.poses.begin(),outgoing.poses.end());
    assert(issue(joined)==CornerGeometry::SupportedOrSmooth);
  }
  {
    auto p=path({pose(.01,0.)});auto start=pose(0.,0.);
    start.pose.orientation.z=std::sin(.5);start.pose.orientation.w=std::cos(.5);
    const auto goal=p.poses.back();anchorGridPathStart(p,start,.05);
    assert(p.poses.size()==2 && p.poses.front().pose.position==start.pose.position && p.poses.back()==goal);
    assert(p.poses.front().pose.orientation==goal.pose.orientation);
  }
  {
    auto p=path({pose(0.,0.),pose(1.,0.),pose(0.,0.)});
    anchorGridPathStart(p,pose(0.,0.),.05);
    assert(issue(p)==CornerGeometry::Turnaround);
  }
  for(int kind=0;kind<5;++kind) {
    auto p=path({pose(0.,0.),pose(1.,0.)});const auto original=p;
    auto start=pose(kind==0?.2:0.,0.);double resolution=.05;
    if(kind==1) {start.header.frame_id="odom";}
    if(kind==2) {start.pose.position.x=std::numeric_limits<double>::quiet_NaN();}
    if(kind==3) {resolution=0.;}
    if(kind==4) {resolution=std::numeric_limits<double>::infinity();}
    bool rejected=false;
    try {anchorGridPathStart(p,start,resolution);}catch(const std::invalid_argument &){rejected=true;}
    assert(rejected && p==original);
  }
  std::cout<<"exact start: left/right grid seams, endpoints, heading-only, real reversal and invalid inputs passed\n";
}
