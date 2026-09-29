#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <action_msgs/msg/goal_status_array.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include "astribot_s1_path_tracking/route_commit.hpp"
#include "astribot_s1_robot_geometry/layered_collision.hpp"
using namespace std::chrono_literals;
using BTStatus=BT::NodeStatus;
using Clock=std::chrono::steady_clock;
using Service=astribot_navigation_msgs::srv::ResolveRoute;
using Snapshot=astribot_s1_robot_geometry::LayeredCollisionSnapshot;
struct Probe {bool obstruct=false,committed=false,finish=false,complete=false;int halts=0,align_ticks=0,normal_polls=0;};
class Navigation : public BT::StatefulActionNode {
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts(){return {};}
  BTStatus onStart() override {return onRunning();}
  BTStatus onRunning() override {
    if(config().blackboard->get<Probe*>("probe")->obstruct)
      throw astribot_s1_path_tracking::ObstructionDeadline("TEMPORARILY_BLOCKED: obstruction deadline");
    return BTStatus::RUNNING;
  }
  void onHalted() override {++config().blackboard->get<Probe*>("probe")->halts;}
};
class Alignment : public BT::StatefulActionNode {
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts(){return {};}
  BTStatus onStart() override{return onRunning();}
  BTStatus onRunning() override {
    auto*p=config().blackboard->get<Probe*>("probe");assert(p->committed&&p->halts==1);++p->align_ticks;
    return p->complete?BTStatus::SUCCESS:BTStatus::RUNNING;
  }
  void onHalted() override {}
};
geometry_msgs::msg::Polygon square() {
  geometry_msgs::msg::Polygon p;
  for(auto xy:std::vector<std::pair<float,float>>{{-.1f,-.1f},{.1f,-.1f},{.1f,.1f},{-.1f,.1f}}) {
    geometry_msgs::msg::Point32 v;v.x=xy.first;v.y=xy.second;p.points.push_back(v);
  }
  return p;
}
void run(const char*library,double distance,bool occupied,bool cancel=false,bool missing_maps=false) {
  Probe probe;
  auto node=std::make_shared<rclcpp::Node>("workstation_bt_test",
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("WorkstationAlign.settle_time",.08)}));
  auto source=std::make_shared<rclcpp::Node>("workstation_source");
  auto pose_pub=source->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose",rclcpp::SensorDataQoS());
  auto maps_pub=source->create_publisher<Snapshot::Maps>("/height_maps/snapshot",rclcpp::QoS(1).reliable().transient_local());
  auto envelope_pub=source->create_publisher<Snapshot::Envelope>("/navigation/envelope_v2",10);
  auto status_pub=source->create_publisher<action_msgs::msg::GoalStatusArray>("/follow_path/_action/status",rclcpp::QoS(1).reliable().transient_local());
  auto service=source->create_service<Service>("/navigation_policy/resolve_route",
    [&probe](Service::Request::SharedPtr req,Service::Response::SharedPtr res) {
      assert(req->session_id=="station-test");assert(req->goal==req->reference_path.poses.back());
      if(req->mode==Service::Request::WORKSTATION_ALIGN) {
        assert(probe.halts==1);probe.committed=true;res->disposition=res->ALIGNMENT_COMMITTED;
      } else if(req->mode==Service::Request::FINISH) {
        probe.finish=true;res->disposition=res->FINISHED;
      } else {
        ++probe.normal_polls;res->disposition=res->BLOCKED;res->reason_code=res->OBSTRUCTION_DEADLINE;
      }
    });
  BT::BehaviorTreeFactory factory;factory.registerFromPlugin(library);
  factory.registerNodeType<Navigation>("Navigation");factory.registerNodeType<Alignment>("Alignment");
  auto bb=BT::Blackboard::create();bb->set("node",node);bb->set("probe",&probe);bb->set("session",std::string("station-test"));
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.position.x=distance;goal.pose.orientation.w=1.;
  nav_msgs::msg::Path path;path.header=goal.header;path.poses={goal,goal};path.poses.front().pose.position.x=0.;
  bb->set("goal",goal);bb->set("path",path);
  auto tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<WorkstationApproach session='{session}' goal='{goal}' path='{path}' alignment_path='{alignment_path}'>"
    "<Navigation/><Alignment/></WorkstationApproach></BehaviorTree></root>",bb);
  Snapshot::Maps maps;maps.header.frame_id="map";maps.map_revision="observed";maps.profile_revision=std::string(64,'a');
  maps.ground_reference="world";maps.evidence_kind="gazebo_collision_geometry";maps.height_edges={.05,1.5};maps.layer_names={"body"};
  nav_msgs::msg::OccupancyGrid grid;grid.header=maps.header;grid.info.width=grid.info.height=60;grid.info.resolution=.05;
  grid.info.origin.position.x=grid.info.origin.position.y=-1.5;grid.info.origin.orientation.w=1.;grid.data.assign(3600,0);
  if(occupied)grid.data[30*60+36]=100;
  maps.grids={grid};
  Snapshot::Envelope envelope;envelope.header.frame_id="base";envelope.epoch=1;envelope.installed_geometry_hash="body";
  envelope.height_profile_revision=maps.profile_revision;envelope.limits.height_m=1.4;envelope.installed_footprint=square();
  astribot_navigation_msgs::msg::EnvelopeSlice slice;slice.z_min_m=.05;slice.z_max_m=1.5;slice.footprint=square();
  envelope.height_slices={slice};envelope.height_geometry_hash=astribot_s1_robot_geometry::layeredGeometryHash(
    astribot_s1_robot_geometry::envelopePolygonPoints(envelope.installed_footprint),envelope.height_slices,"base",0.,envelope.height_profile_revision,0.);
  auto started=Clock::now();BTStatus result=BTStatus::RUNNING;bool outside=false;
  while(Clock::now()-started<5s) {
    auto stamp=source->now();
    geometry_msgs::msg::PoseWithCovarianceStamped pose;pose.header.frame_id="map";pose.header.stamp=stamp;pose.pose.pose.orientation.w=1.;pose_pub->publish(pose);
    maps.header.stamp=stamp;if(!missing_maps)maps_pub->publish(maps);envelope.header.stamp=stamp;envelope_pub->publish(envelope);
    action_msgs::msg::GoalStatusArray statuses;action_msgs::msg::GoalStatus s;
    s.status=probe.halts?action_msgs::msg::GoalStatus::STATUS_CANCELED:action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    statuses.status_list={s};status_pub->publish(statuses);
    rclcpp::spin_some(source);
    if(Clock::now()-started>250ms)probe.obstruct=true;
    try {result=tree.tickRoot();} catch(const astribot_s1_path_tracking::ObstructionDeadline&) {outside=true;break;}
    if(probe.align_ticks>2) {
      auto aligned=bb->get<nav_msgs::msg::Path>("alignment_path");assert(aligned.poses.back()==goal);assert(aligned.poses.front().pose.position.x==0.);
      if(cancel) {
        std::thread responder([&]{auto until=Clock::now()+500ms;while(Clock::now()<until){rclcpp::spin_some(source);std::this_thread::sleep_for(2ms);}});
        tree.haltTree();responder.join();assert(probe.finish);return;
      }
      probe.complete=true;
    }
    if(result!=BTStatus::RUNNING)break;
    std::this_thread::sleep_for(10ms);
  }
  if(distance>.5){assert(outside&&!probe.committed&&probe.align_ticks==0);tree.haltTree();}
  else if(occupied){assert(outside&&!probe.committed&&probe.align_ticks==0);tree.haltTree();}
  else if(missing_maps){assert(result==BTStatus::FAILURE&&!outside&&!probe.committed&&probe.align_ticks==0);}
  else {assert(result==BTStatus::SUCCESS&&probe.finish&&probe.normal_polls>0&&probe.align_ticks>0);}
}
int main(int argc,char**argv) {
  assert(argc==2);rclcpp::init(argc,argv);
  run(argv[1],.49,false);run(argv[1],.50,false);run(argv[1],.51,false);
  run(argv[1],.49,true);run(argv[1],.49,false,true);run(argv[1],.49,false,false,true);
  rclcpp::shutdown();std::cout<<"6 workstation BT boundary, handover, obstruction handoff, missing input and cancel cases passed\n";
}
