// The real dynamically loaded BT plugin and ROS services are exercised here.
// Only planner/assessment answers and the Departure motion child are doubles.
#include <cassert>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "action_msgs/msg/goal_status_array.hpp"
#include "astribot_navigation_msgs/srv/assess_navigation_start.hpp"
#include "astribot_navigation_msgs/srv/plan_candidate.hpp"
#include "astribot_navigation_msgs/srv/plan_start_recovery.hpp"
#include "astribot_navigation_msgs/srv/resolve_route.hpp"
#include "astribot_s1_path_tracking/route_commit.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

using Plan=astribot_navigation_msgs::srv::PlanCandidate;
using Recovery=astribot_navigation_msgs::srv::PlanStartRecovery;
using Resolve=astribot_navigation_msgs::srv::ResolveRoute;
using Assess=astribot_navigation_msgs::srv::AssessNavigationStart;
using Status=BT::NodeStatus;
using Clock=std::chrono::steady_clock;
struct Probe {
  geometry_msgs::msg::PoseStamped goal,actual;
  std::string execution="connection_fixture";
  std::deque<int> plans{0}; // 0: success, 1: typed start obstruction, 2: other failure.
  std::deque<uint8_t> assessments{Assess::Response::READY};
  std::vector<geometry_msgs::msg::PoseStamped> planning_starts;
  std::vector<double> recovery_headings;
  unsigned departures=0,halted=0,assessed=0;
  bool finish=true,recovery_success=true,one_point=false,publish_slam=true;
  bool publish_status=false,old_follow_active=false,policy=false;
  unsigned clear_after_departures=2,normal_starts=0,normal_halts=0,blocked_episodes=1;
  std::vector<uint8_t> route_modes;
  nav_msgs::msg::Path rejected_reference;
  int last_mode=-1;unsigned mode_requests=0;bool recovery_mode_confirmed=false;
};
class Departure : public BT::StatefulActionNode {
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),BT::InputPort<std::string>("controller_id"),
      BT::InputPort<std::string>("goal_checker_id")};
  }
  Status onStart() override {
    p_=config().blackboard->get<Probe*>("probe");++p_->departures;
    nav_msgs::msg::Path path;std::string controller;
    assert(getInput("path",path)&&path.poses.size()>1);
    assert(getInput("controller_id",controller)&&controller=="Departure");
    endpoint_=path.poses.back().pose.position.x;
    return Status::RUNNING;
  }
  Status onRunning() override {
    if(!p_->finish)return Status::RUNNING;
    // Actual arrival deliberately differs from the planned endpoint.
    p_->actual.pose.position.x+=.123;
    assert(std::abs(p_->actual.pose.position.x-endpoint_)>.01);
    return Status::SUCCESS;
  }
  void onHalted() override {++p_->halted;}
private:
  Probe *p_=nullptr;double endpoint_=0.;
};
class NormalNavigation : public BT::StatefulActionNode {
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts(){return {BT::InputPort<nav_msgs::msg::Path>("path")};}
  Status onStart() override {p_=config().blackboard->get<Probe*>("probe");++p_->normal_starts;return Status::RUNNING;}
  Status onRunning() override {
    nav_msgs::msg::Path path;assert(getInput("path",path));
    if(p_->normal_starts<=p_->blocked_episodes) {
      p_->rejected_reference=path;
      throw astribot_s1_path_tracking::ObstructionDeadline("TEMPORARILY_BLOCKED: obstruction deadline");
    }
    assert(!p_->planning_starts.empty()&&path.poses.front()==p_->actual);
    return Status::SUCCESS;
  }
  void onHalted() override {++p_->normal_halts;}
private:
  Probe *p_=nullptr;
};
struct Fixture {
  Probe p;
  rclcpp::Node::SharedPtr owner,server;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Service<Plan>::SharedPtr plan;
  rclcpp::Service<Recovery>::SharedPtr recovery;
  rclcpp::Service<Assess>::SharedPtr assess;
  rclcpp::Service<Resolve>::SharedPtr resolve;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_pub;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board;
  BT::Tree tree;
  Status status=Status::IDLE;
  Fixture(const char *library,bool policy=false,bool wrapper=false) {
    p.policy=policy;p.publish_status=policy;
    p.goal.header.frame_id=p.actual.header.frame_id="map";
    p.goal.pose.orientation.w=p.actual.pose.orientation.w=1.;p.goal.pose.position.x=2.;
    owner=std::make_shared<rclcpp::Node>("connection_bt_fixture_owner");
    server=std::make_shared<rclcpp::Node>("connection_bt_fixture_services");executor.add_node(server);
    plan=server->create_service<Plan>("/path_tracking/plan_candidate",
      [this](const Plan::Request::SharedPtr req,Plan::Response::SharedPtr res) {
        assert(req->mode==Plan::Request::INITIAL&&req->goal==p.goal);
        assert(req->candidate_path.poses.empty()&&req->reference_path.poses.empty());
        p.planning_starts.push_back(p.actual);assert(!p.plans.empty());
        const int answer=p.plans.front();p.plans.pop_front();
        res->evaluated_start=p.actual;res->evaluated_start.header.stamp=server->now();
        res->geometry_valid=answer==0;res->failure_code=answer==1?Plan::Response::START_CONNECTION_BLOCKED:Plan::Response::NONE;
        res->required_start_heading=.7+.1*p.planning_starts.size();
        res->reason=answer==0?"READY":(answer==1?"START_CONNECTION_BLOCKED":"OTHER_PLAN_REJECTION");
        if(answer==0){res->path.header.frame_id="map";res->path.poses={p.actual,p.goal};}
      });
    recovery=server->create_service<Recovery>("/navigation/plan_start_recovery",
      [this](const Recovery::Request::SharedPtr req,Recovery::Response::SharedPtr res) {
        assert(req->execution_id==p.execution&&req->header.frame_id=="map");
        assert(req->use_policy_obstacles==p.policy);
        if(!p.route_modes.empty())assert(p.recovery_mode_confirmed);
        p.recovery_headings.push_back(req->required_heading);res->success=p.recovery_success;
        res->reason=res->success?"CLEAR_DEPARTURE":"NO_SAFE_EXIT";
        if(res->success) {
          res->path.header.frame_id="map";res->path.poses.push_back(p.actual);
          if(!p.one_point&&(!p.policy||p.departures<p.clear_after_departures)){auto end=p.actual;end.pose.position.x+=.2;res->path.poses.push_back(end);}
        }
      });
    assess=server->create_service<Assess>("/navigation/assess_start",
      [this](const Assess::Request::SharedPtr req,Assess::Response::SharedPtr res) {
        assert(req->execution_id==p.execution&&req->goal==p.goal);assert(!p.assessments.empty());
        ++p.assessed;res->state=p.assessments.front();p.assessments.pop_front();
        res->reason="RECHECK_"+std::to_string(res->state);res->evaluated_start=p.actual;
        res->evaluated_start.header.stamp=server->now();res->header=res->evaluated_start.header;
      });
    resolve=server->create_service<Resolve>("/navigation_policy/resolve_route",
      [this](const Resolve::Request::SharedPtr req,Resolve::Response::SharedPtr res) {
        assert(req->session_id==p.execution&&req->goal==p.goal);
        assert(req->reference_path==p.rejected_reference);
        p.route_modes.push_back(req->mode);
        if(p.last_mode!=req->mode){p.last_mode=req->mode;p.mode_requests=0;}
        if(++p.mode_requests<=2){res->disposition=Resolve::Response::KEEP;res->reason="ROUTE_EVALUATION_PENDING";return;}
        if(req->mode==Resolve::Request::RECOVER){res->disposition=Resolve::Response::RECOVERY_COMMITTED;p.recovery_mode_confirmed=true;}
        else if(req->mode==Resolve::Request::RESUME){res->disposition=Resolve::Response::RESUMED;p.recovery_mode_confirmed=false;}
        else {assert(req->mode==Resolve::Request::FINISH);res->disposition=Resolve::Response::FINISHED;}
      });
    slam=server->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose",rclcpp::SensorDataQoS());
    status_pub=server->create_publisher<action_msgs::msg::GoalStatusArray>("/follow_path/_action/status",rclcpp::QoS(1).reliable().transient_local());
    factory.registerFromPlugin(library);factory.registerNodeType<Departure>("FollowPath");
    factory.registerNodeType<NormalNavigation>("NormalNavigation");
    board=BT::Blackboard::create();board->set("node",owner);board->set("probe",&p);
    board->set("goal",p.goal);board->set("session",p.execution);board->set("policy",policy);board->set("heading",.8);
    nav_msgs::msg::Path stale;stale.poses={p.actual,p.goal};board->set("path",stale);
    std::string xml=R"(<root main_tree_to_execute="Main"><BehaviorTree ID="Main">)";
    if(wrapper)xml+=R"(<RecoverPolicyObstruction session="{session}" goal="{goal}" path="{path}" required_heading="{heading}"><NormalNavigation path="{path}"/>)";
    xml+=R"(<ComputePathWithRecovery session="{session}" goal="{goal}" path="{path}" departure_path="{departure}" policy_recovery="{policy}" required_heading="{heading}">
        <FollowPath path="{departure}" controller_id="Departure" goal_checker_id="precise_goal_checker"/>
      </ComputePathWithRecovery>)";
    if(wrapper)xml+="</RecoverPolicyObstruction>";
    xml+="</BehaviorTree></root>";tree=factory.createTreeFromText(xml,board);
    const auto deadline=Clock::now()+std::chrono::seconds(3);
    while(slam->get_subscription_count()!=(wrapper?2u:1u)||status_pub->get_subscription_count()!=1) {
      assert(Clock::now()<deadline);executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  ~Fixture(){tree.haltTree();executor.remove_node(server);}
  void tick() {
    assert(status==Status::IDLE||status==Status::RUNNING);
    if(p.publish_slam) {geometry_msgs::msg::PoseWithCovarianceStamped msg;msg.header=p.actual.header;msg.header.stamp=server->now();msg.pose.pose=p.actual.pose;slam->publish(msg);}
    if(p.publish_status) {action_msgs::msg::GoalStatusArray msg;action_msgs::msg::GoalStatus s;s.goal_info.goal_id.uuid[0]=1;s.status=p.old_follow_active?2:4;msg.status_list.push_back(s);status_pub->publish(msg);}
    executor.spin_some();status=tree.tickRoot();executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  template<class Predicate>void until(Predicate done) {
    const auto deadline=Clock::now()+std::chrono::seconds(5);
    while(!done()){assert(Clock::now()<deadline);tick();}
  }
  void duration(double seconds) {
    const auto deadline=Clock::now()+std::chrono::duration<double>(seconds);
    while(Clock::now()<deadline)tick();
  }
  void terminal(){until([this]{return status==Status::SUCCESS||status==Status::FAILURE;});}
};
int main(int argc,char **argv) {
  assert(argc==2||argc==3);const bool policy_only=argc==3;const char *domain=std::getenv("ROS_DOMAIN_ID");
  assert(domain&&std::string(domain)=="71");rclcpp::init(argc,argv);
  if(!policy_only) {
  {
    Fixture f(argv[1]);f.terminal();assert(f.status==Status::SUCCESS);
    assert(f.p.departures==0&&f.p.recovery_headings.empty()&&f.p.assessed==0);
    // A long ordinary navigation must not consume the recovery-only budget.
    std::this_thread::sleep_for(std::chrono::seconds(121));
    f.tree.haltTree();f.status=Status::IDLE;f.p.plans={0};f.terminal();
    assert(f.status==Status::SUCCESS&&f.p.recovery_headings.empty());
    f.tree.haltTree();f.status=Status::IDLE;f.p.plans={1,0};f.terminal();
    assert(f.status==Status::SUCCESS&&f.p.departures==1&&f.p.assessed==1);
  }
  {
    Fixture f(argv[1]);f.p.plans={1,0};f.terminal();
    assert(f.status==Status::SUCCESS&&f.p.departures==1&&f.p.assessed==1);
    assert(f.p.planning_starts.size()==2&&f.p.planning_starts[0].pose.position.x==0.);
    assert(std::abs(f.p.planning_starts[1].pose.position.x-.123)<1e-10);
    assert(f.board->get<nav_msgs::msg::Path>("path").poses.back()==f.p.goal);
  }
  {
    Fixture f(argv[1]);f.p.plans={1,0};f.p.publish_status=true;f.p.old_follow_active=true;
    f.until([&]{return f.p.plans.size()==1;});f.duration(.75);
    assert(f.p.recovery_headings.empty());assert(f.board->get<nav_msgs::msg::Path>("path").poses.empty());
    f.p.old_follow_active=false;const auto at=Clock::now();f.terminal();
    assert(f.status==Status::SUCCESS&&Clock::now()-at>=std::chrono::milliseconds(600));
  }
  {
    Fixture f(argv[1]);f.p.plans={1,0};f.p.publish_slam=false;
    f.until([&]{return f.p.plans.size()==1;});f.duration(.75);assert(f.p.recovery_headings.empty());
    f.p.publish_slam=true;f.terminal();assert(f.status==Status::SUCCESS);
  }
  {
    Fixture f(argv[1]);f.p.plans={2};f.terminal();
    assert(f.status==Status::FAILURE&&f.p.recovery_headings.empty()&&f.p.departures==0);
  }
  {
    Fixture f(argv[1]);f.p.plans={1};f.p.recovery_success=false;f.terminal();
    assert(f.status==Status::FAILURE&&f.p.recovery_headings.size()==1&&f.p.departures==0);
  }
  {
    Fixture f(argv[1]);f.p.plans={1};f.p.finish=false;
    f.until([&]{return f.p.departures==1;});f.tree.haltTree();
    assert(f.p.halted==1&&f.p.assessed==0&&f.p.planning_starts.size()==1);
  }
  {
    Fixture f(argv[1]);f.p.plans={1,0};f.p.assessments={Assess::Response::RECOVERY_REQUIRED,Assess::Response::READY};f.terminal();
    assert(f.status==Status::SUCCESS&&f.p.departures==2&&f.p.recovery_headings.size()==2);
    assert(f.p.recovery_headings[0]==f.p.recovery_headings[1]);
    assert(std::abs(f.p.planning_starts.back().pose.position.x-.246)<1e-10);
  }
  {
    Fixture f(argv[1]);f.p.plans={1,1,0};f.p.assessments={Assess::Response::READY,Assess::Response::READY};f.terminal();
    assert(f.status==Status::SUCCESS&&f.p.recovery_headings.size()==2);
    assert(f.p.recovery_headings[0]!=f.p.recovery_headings[1]);
  }
  {
    Fixture f(argv[1]);f.p.plans={1,1};f.p.one_point=true;f.terminal();
    assert(f.status==Status::FAILURE&&f.p.departures==0&&f.p.recovery_headings.size()==2);
    assert(f.p.assessed==1&&f.p.planning_starts.size()==2);
  }
  for(const auto state:{Assess::Response::BLOCKED,Assess::Response::UNAVAILABLE}) {
    Fixture f(argv[1]);f.p.plans={1};f.p.assessments={state};f.terminal();
    assert(f.status==Status::FAILURE&&f.p.departures==1&&f.p.planning_starts.size()==1);
  }
  }
  {
    Fixture f(argv[1],true);f.p.old_follow_active=true;f.duration(.8);
    assert(f.p.planning_starts.empty()&&f.p.recovery_headings.empty());
    f.p.old_follow_active=false;f.p.publish_slam=false;f.duration(.8);
    assert(f.p.planning_starts.empty()&&f.p.recovery_headings.empty());
    f.p.publish_slam=true;f.terminal();
    assert(f.status==Status::SUCCESS&&f.p.departures==2&&f.p.assessed==1);
    assert(f.p.recovery_headings.size()==3&&f.p.planning_starts.size()==1);
    assert(std::abs(f.p.planning_starts[0].pose.position.x-.246)<1e-10);
  }
  {
    Fixture f(argv[1],true);f.p.clear_after_departures=10;f.terminal();
    assert(f.status==Status::SUCCESS&&f.p.departures==10&&f.p.recovery_headings.size()==11);
  }
  {
    Fixture f(argv[1],true);f.p.clear_after_departures=11;f.terminal();
    assert(f.status==Status::FAILURE&&f.p.departures==10&&f.p.planning_starts.empty());
  }
  {
    Fixture f(argv[1],true);f.p.recovery_success=false;f.terminal();
    assert(f.status==Status::FAILURE&&f.p.departures==0&&f.p.planning_starts.empty());
  }
  {
    Fixture f(argv[1],true,true);f.terminal();
    assert(f.status==Status::SUCCESS&&f.p.normal_starts==2&&f.p.normal_halts==1);
    assert(f.p.departures==2&&f.p.planning_starts.size()==1);
    assert(f.p.route_modes.front()==Resolve::Request::RECOVER&&f.p.route_modes.back()==Resolve::Request::RESUME);
  }
  {
    Fixture f(argv[1],true,true);f.p.actual.pose.position.x=1.;f.p.actual.pose.position.y=.4;
    nav_msgs::msg::Path corner;corner.header.frame_id="map";
    for(const auto xy:std::vector<std::pair<double,double>>{{0.,0.},{1.,0.},{1.,1.},{2.,1.},{2.,0.}}) {
      auto pose=f.p.actual;pose.pose.position.x=xy.first;pose.pose.position.y=xy.second;corner.poses.push_back(pose);
    }
    f.board->set("path",corner);f.p.clear_after_departures=1;f.terminal();
    assert(f.status==Status::SUCCESS&&std::abs(f.p.recovery_headings.front()-1.5707963267948966)<1e-10);
    assert(std::abs(f.p.planning_starts.front().pose.position.x-1.123)<1e-10);
  }
  {
    Fixture f(argv[1],true,true);f.p.blocked_episodes=2;f.p.plans={0,0};
    f.p.assessments={Assess::Response::READY,Assess::Response::READY};f.p.clear_after_departures=1;
    f.until([&]{return f.p.normal_starts==2;});f.p.clear_after_departures=2;f.p.old_follow_active=true;
    f.duration(.75);
    assert(f.p.departures==1&&f.p.planning_starts.size()==1&&f.p.recovery_headings.size()==2);
    f.p.old_follow_active=false;f.terminal();
    assert(f.status==Status::SUCCESS&&f.p.normal_starts==3&&f.p.departures==2&&f.p.recovery_headings.size()==4);
    assert(f.p.planning_starts.size()==2&&std::abs(f.p.planning_starts.back().pose.position.x-.246)<1e-10);
  }
  {
    Fixture f(argv[1],true,true);f.p.finish=false;f.until([&]{return f.p.departures==1;});
    std::atomic<bool> run{true};std::thread services([&]{while(run){f.executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    f.tree.haltTree();run=false;services.join();
    assert(f.p.halted==1&&f.p.planning_starts.empty()&&f.p.assessed==0);
    assert(f.p.route_modes.back()==Resolve::Request::FINISH);
  }
  rclcpp::shutdown();return 0;
}
