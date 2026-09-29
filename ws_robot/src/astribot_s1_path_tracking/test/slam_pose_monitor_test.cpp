#include "corner_fixture.hpp"
#include <cassert>
using namespace astribot_s1_path_tracking;
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  using Peer=ThreePhaseControllerTestPeer;
  ThreePhaseController c;Peer::init(c);
  bool missing=false;try {Peer::latest(c);}catch(const nav2_core::PlannerException &){missing=true;}
  assert(missing);
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.orientation.w=1.;
  p.header.stamp.sec=1000;Peer::slam(c,p);Peer::received(c,10.);
  assert(!Peer::stopped(c,10.,5.));
  p.header.stamp.sec=1001;Peer::slam(c,p);Peer::received(c,10.7);
  assert(Peer::stopped(c,10.7,5.)); // only SLAM determines stopped, even if odom says moving
  auto old=p;old.header.stamp.sec=999;old.pose.position.x=10.;Peer::slam(c,old);
  assert(Peer::latest(c).first.pose.position.x==0.);
  old.header.stamp=p.header.stamp;Peer::slam(c,old);
  assert(Peer::latest(c).first.pose.position.x==0.);
  c.setPlan(path({{0,0},{1,0},{1,1}},100));Peer::active(c,0);Peer::monitor(c);
  p.header.stamp.nanosec=100000000;p.pose.position.x=2.;Peer::slam(c,p);
  bool jump=false;try {Peer::monitor(c);}catch(const nav2_core::PlannerException &e) {
    jump=std::string(e.what()).find("LOCALIZATION_DISCONTINUITY")!=std::string::npos;
  }
  assert(jump);
  p.header.stamp.sec=1002;p.header.frame_id="odom";Peer::slam(c,p);
  bool invalid=false;try {Peer::latest(c);}catch(const nav2_core::PlannerException &){invalid=true;}
  assert(invalid); // no TF or odom fallback
  ThreePhaseController normal;Peer::init(normal);normal.setPlan(path({{0,0},{1,0},{1,1}},100));
  Peer::beginCornerStop(normal,20.);
  p.header.frame_id="map";p.header.stamp.sec=2000;p.header.stamp.nanosec=0;p.pose.position.x=1.;
  Peer::slam(normal,p);Peer::received(normal,20.);Peer::advanceCorner(normal);
  assert(normal.phase()==Phase::kCornerApproach);
  p.header.stamp.sec=2001;Peer::slam(normal,p);Peer::received(normal,20.7);Peer::advanceCorner(normal);
  assert(normal.phase()==Phase::kAlignCorner); // normal SLAM stop confirms and continues to turning
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame",std::string("map")),
      rclcpp::Parameter("robot_base_frame",std::string("base_link")),
      rclcpp::Parameter("track_unknown_space",false)}));
  assert(map->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS);
  map->getCostmap()->resizeMap(100,100,.05,-2.5,-2.5);
  auto footprint=std::make_shared<geometry_msgs::msg::Polygon>();
  for(auto xy:std::vector<std::pair<float,float>>{{-.1f,-.1f},{.1f,-.1f},{.1f,.1f},{-.1f,.1f}}) {
    geometry_msgs::msg::Point32 point;point.x=xy.first;point.y=xy.second;footprint->points.push_back(point);
  }
  map->setRobotFootprintPolygon(footprint);Peer::costmap(normal,map);
  p.header.stamp.sec=2002;p.pose.orientation.z=std::sin(M_PI/4.);p.pose.orientation.w=std::cos(M_PI/4.);
  Peer::slam(normal,p);Peer::received(normal,30.);
  geometry_msgs::msg::Twist odom;odom.linear.x=2.;odom.angular.z=2.;
  normal.computeVelocityCommands(p,odom,nullptr);
  assert(normal.phase()==Phase::kAlignCorner); // fresh stationary window after yaw change
  p.header.stamp.sec=2003;Peer::slam(normal,p);Peer::received(normal,30.7);
  normal.computeVelocityCommands(p,odom,nullptr);
  assert(normal.phase()==Phase::kFollow); // actual outer gate ignores contradictory odom twist
  map->on_cleanup(rclcpp_lifecycle::State());
  std::cout<<"SLAM missing/ordering/future/stop/jump/frame checks passed\n";
  rclcpp::shutdown();
}
