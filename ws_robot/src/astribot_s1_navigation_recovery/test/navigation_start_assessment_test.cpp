#include "astribot_s1_navigation_recovery/navigation_start_assessment.hpp"
#include <rcl/time.h>
#include <chrono>
#include <iostream>
#include <thread>

namespace {
using namespace astribot_s1_navigation_recovery;
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Maps=astribot_slam_msgs::msg::HeightSliceMaps;
using Service=NavigationStartAssessment::Service;
using Steady=std::chrono::steady_clock;
void check(bool value,const char *reason) {if(!value)throw std::runtime_error(reason);}
geometry_msgs::msg::Polygon rectangle(double x,double y) {
  geometry_msgs::msg::Polygon polygon;
  for(auto xy:std::vector<std::pair<double,double>>{{-x,-y},{x,-y},{x,y},{-x,y}}) {
    geometry_msgs::msg::Point32 p;p.x=xy.first;p.y=xy.second;polygon.points.push_back(p);
  }
  return polygon;
}
void clockAt(const rclcpp::Clock::SharedPtr &clock,double seconds) {
  check(rcl_enable_ros_time_override(clock->get_clock_handle())==RCL_RET_OK,"enable fixture clock");
  check(rcl_set_ros_time_override(clock->get_clock_handle(),std::llround(seconds*1e9))==RCL_RET_OK,"advance fixture clock");
}
struct Fixture {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  rclcpp::Node::SharedPtr peer;
  rclcpp::executors::SingleThreadedExecutor executor;
  EnvelopeGuard guard;LayeredCollisionReader reader;
  std::unique_ptr<NavigationStartAssessment> assessment;
  rclcpp::Publisher<Envelope>::SharedPtr envelopes;
  rclcpp::Publisher<Maps>::SharedPtr height_maps;
  rclcpp::Client<Service>::SharedPtr client;
  Envelope envelope;Maps maps;double now{100.};
  Fixture() {
    map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("global_frame",std::string("map")),
      rclcpp::Parameter("robot_base_frame",std::string("base_link")),rclcpp::Parameter("track_unknown_space",false),
      rclcpp::Parameter("footprint_padding",0.),rclcpp::Parameter("transform_tolerance",0.),rclcpp::Parameter("use_sim_time",true)}));
    check(map->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS,"configure assessment map");
    map->getCostmap()->resizeMap(100,100,.05,-2.5,-2.5);
    // Deliberately stale installation: diagnosis must use the envelope snapshot.
    map->setRobotFootprintPolygon(std::make_shared<geometry_msgs::msg::Polygon>(rectangle(.1,.1)));
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("start_assessment_fixture",rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
      rclcpp::Parameter("use_sim_time",true),rclcpp::Parameter("navigation_geometry_mode",std::string("fixed_v2"))}));
    time();guard.configure(node,map,"planner");reader.configure(node,map);
    assessment=std::make_unique<NavigationStartAssessment>(node,map,guard,reader);
    peer=std::make_shared<rclcpp::Node>("start_assessment_peer");
    envelopes=peer->create_publisher<Envelope>("/navigation/envelope_v2",10);
    height_maps=peer->create_publisher<Maps>("/height_maps/snapshot",rclcpp::QoS(1).reliable().transient_local());
    client=peer->create_client<Service>("/navigation/assess_start");
    executor.add_node(node->get_node_base_interface());executor.add_node(peer);
    envelope.header.frame_id="base_link";envelope.mode=Envelope::FIXED_POSTURE;
    envelope.coordinator_session_id="assessment-session";envelope.epoch=7;envelope.clock_epoch=1;
    envelope.hold_id="hold";envelope.request_id="request";envelope.attachment_revision="attachment";envelope.model_revision="model";
    envelope.navigation_allowed=false;envelope.limits.transport_ready=false;envelope.reason="WAITING_FOR:controller,planner";
    envelope.installed_footprint=rectangle(.6,.3);envelope.installed_geometry_hash="installed-version";envelope.limits.height_m=1.4;
    maps.header.frame_id="map";maps.map_revision="height-map-1";maps.profile_revision=std::string(64,'a');
    maps.ground_reference="world_horizontal_ground";maps.evidence_kind="gazebo_collision_geometry";
    maps.height_edges={.05,.5,1.5};maps.layer_names={"base","arm"};
    nav_msgs::msg::OccupancyGrid grid;grid.header=maps.header;grid.info.width=grid.info.height=100;grid.info.resolution=.05;
    grid.info.origin.position.x=grid.info.origin.position.y=-2.5;grid.info.origin.orientation.w=1.;grid.data.assign(10000,0);
    maps.grids={grid,grid};envelope.height_profile_revision=maps.profile_revision;envelope.ground_in_base_m=-.1;
    for(size_t i=0;i<2;++i) {
      astribot_navigation_msgs::msg::EnvelopeSlice slice;slice.z_min_m=maps.height_edges[i]-.1;slice.z_max_m=maps.height_edges[i+1]-.1;
      slice.footprint=i?rectangle(.6,.08):rectangle(.2,.2);envelope.height_slices.push_back(slice);
    }
    envelope.height_geometry_hash=astribot_s1_robot_geometry::layeredGeometryHash(
      astribot_s1_robot_geometry::envelopePolygonPoints(envelope.installed_footprint),envelope.height_slices,"base_link",0.,maps.profile_revision,-.1);
    const auto deadline=Steady::now()+std::chrono::seconds(5);
    while((envelopes->get_subscription_count()<2||height_maps->get_subscription_count()<1)&&Steady::now()<deadline) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    check(envelopes->get_subscription_count()>=2&&height_maps->get_subscription_count()>=1,"assessment input discovery");
  }
  ~Fixture() {assessment.reset();reader.cleanup();guard.cleanup();map->on_cleanup(rclcpp_lifecycle::State());}
  void time() {clockAt(node->get_clock(),now);clockAt(map->get_clock(),now);}
  void actualPose() {
    geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="map";tf.child_frame_id="base_link";tf.header.stamp=node->now();tf.transform.rotation.w=1.;
    check(map->getTfBuffer()->setTransform(tf,"assessment_fixture",false),"set actual pose");
  }
  void publish(bool send_envelope=true,bool send_map=true) {
    time();
    if(send_envelope) {envelope.header.stamp=node->now();envelope.limits.stamp=node->now();envelope.valid_until=node->now()+rclcpp::Duration::from_seconds(.3);}
    if(send_map) {maps.header.stamp=node->now();for(auto &g:maps.grids)g.header.stamp=maps.header.stamp;}
    for(int i=0;i<12;++i) {
      if(send_envelope)envelopes->publish(envelope);
      if(send_map)height_maps->publish(maps);
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  void planar(double x,double y,unsigned char value) {
    unsigned mx,my;check(map->getCostmap()->worldToMap(x,y,mx,my),"planar fixture cell");map->getCostmap()->setCost(mx,my,value);
  }
  void layered(double x,double y,int8_t value) {
    unsigned mx,my;check(map->getCostmap()->worldToMap(x,y,mx,my),"layered fixture cell");
    maps.grids[0].data.at(size_t(my)*100+mx)=value;maps.map_revision+="x";
  }
  Service::Response::SharedPtr assess() {
    check(client->wait_for_service(std::chrono::seconds(2)),"assessment service discovery");
    auto request=std::make_shared<Service::Request>();request->execution_id="assessment-execution";
    request->goal.header.frame_id="map";request->goal.pose.position.y=2.;request->goal.pose.orientation.w=1.;
    auto future=client->async_send_request(request);const auto deadline=Steady::now()+std::chrono::seconds(2);
    while(future.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready&&Steady::now()<deadline) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(future.wait_for(std::chrono::milliseconds(0))==std::future_status::ready,"assessment response timeout");return future.get();
  }
};
void run() {
  using Response=Service::Response;Fixture f;
  f.actualPose();check(f.assess()->state==Response::UNAVAILABLE,"missing envelope is unavailable");
  f.planar(.4,.25,254);f.layered(.4,.25,100);f.publish();
  check(!f.guard.ready(),"fixture must have no motion permission");
  auto result=f.assess();check(result->state==Response::RECOVERY_REQUIRED,"waiting ACK still permits diagnosis with snapshot footprint");
  check(result->height_map_revision==f.maps.map_revision&&result->geometry_hash==f.envelope.height_geometry_hash&&result->envelope_epoch==7&&result->costmap_revision.size()==64,"same snapshot evidence");
  f.planar(.4,.25,0);f.planar(.4,.5,254);f.now+=.05;f.publish();f.actualPose();
  check(f.assess()->state==Response::READY,"current pose clear: possible later rotation collision is not a trapped start");
  f.layered(.1,.1,100);f.now+=.05;f.publish();f.actualPose();
  check(f.assess()->state==Response::BLOCKED,"actual same-layer intersection is blocked");
  f.layered(.1,.1,0);f.now+=.35;f.publish(false,true);f.actualPose();
  result=f.assess();check(result->state==Response::READY,"latest geometry remains available after source age");
  f.now+=.05;f.publish();f.actualPose();check(f.assess()->state==Response::READY,"fresh input recovers diagnosis");
  f.now+=1.6;f.publish(true,false);f.actualPose();
  result=f.assess();check(result->state==Response::READY,"latest map remains available after source age");
  f.now+=.05;f.publish();f.actualPose();check(f.assess()->state==Response::READY,"fresh map recovers diagnosis");
  f.maps.profile_revision=std::string(64,'b');f.maps.map_revision+="x";f.now+=.05;f.publish();f.actualPose();
  result=f.assess();check(result->state==Response::UNAVAILABLE&&result->reason.find("PROFILE_MISMATCH")!=std::string::npos,"different height profile unavailable");
  f.maps.profile_revision=f.envelope.height_profile_revision;f.maps.map_revision+="x";f.now+=1.;f.publish();
  result=f.assess();check(result->state==Response::READY,"latest TF remains usable independent of local time");
  std::cout<<"assessment: pre-ACK diagnosis, one-snapshot footprint, current yaw, entity collision and input freshness passed\n";
}
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {run();rclcpp::shutdown();return 0;}
  catch(const std::exception &error) {std::cerr<<error.what()<<'\n';rclcpp::shutdown();return 1;}
}
