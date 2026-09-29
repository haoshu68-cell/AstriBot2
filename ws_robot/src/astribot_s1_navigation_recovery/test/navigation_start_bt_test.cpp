// Run only in a test domain owned by the caller. Loads the actual Ensure BT
// plugin; only assessment answers, envelope guard and motion actions are doubles.
#include <cassert>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "astribot_navigation_msgs/srv/assess_navigation_start.hpp"
#include "astribot_navigation_msgs/msg/navigation_envelope_v2.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using Assessment=astribot_navigation_msgs::srv::AssessNavigationStart;
using Answer=Assessment::Response;
using Status=BT::NodeStatus;
using Clock=std::chrono::steady_clock;

struct Probe {
  bool envelope=true, planned=true, stopped=false, plan_failed=false, follow_failed=false;
  bool single_point=false, safe_path=false, target_done=false, defer=false, repeat_answer=false;
  int stopped_steps=0;
  std::string execution="execution_a";
  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::PoseStamped current_pose;
  std::deque<uint8_t> answers;
  std::deque<std::pair<double,double>> positions;
  std::deque<double> headings;
  std::vector<geometry_msgs::msg::PoseStamped> assessed_starts;
  std::map<std::string,std::vector<geometry_msgs::msg::PoseStamped>> planning_starts;
  std::vector<std::string> requests;
  std::map<std::string,int> starts,halts;
};

class MotionAction : public BT::StatefulActionNode {
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("start"),
      BT::BidirectionalPort<nav_msgs::msg::Path>("path"),BT::InputPort<std::string>("planner_id"),
      BT::InputPort<std::string>("controller_id"),BT::InputPort<std::string>("goal_checker_id")};
  }
  Status onStart() override {
    probe_=config().blackboard->get<Probe*>("probe");planning_=bool(getInput("planner_id",id_));
    if(!planning_)assert(getInput("controller_id",id_));
    key_=std::string(planning_?"plan:":"follow:")+id_;++probe_->starts[key_];
    if(planning_) {
      geometry_msgs::msg::PoseStamped goal;assert(getInput("goal",goal));assert(goal==probe_->goal);
      // Nav2 resolves the live robot pose when its optional start input is absent.
      // Keep this XML contract explicit: neither assessment nor departure end
      // may become a cached start override for the next planning request.
      geometry_msgs::msg::PoseStamped start;assert(!getInput("start",start));
      probe_->planning_starts[id_].push_back(probe_->current_pose);
    } else {
      std::string checker;assert(getInput("goal_checker_id",checker));assert(checker=="precise_goal_checker");
    }
    return result();
  }
  Status onRunning() override {return result();}
  void onHalted() override {++probe_->halts[key_];}
private:
  Status result() {
    if(planning_) {
      if(id_=="Departure") {
        if(probe_->plan_failed)return Status::FAILURE;
        if(!probe_->planned)return Status::RUNNING;
      } else assert(id_=="GridBased");
      nav_msgs::msg::Path path;path.header.frame_id="map";
      path.poses.resize(id_=="Departure"&&probe_->single_point?1:2,probe_->goal);
      setOutput("path",path);return Status::SUCCESS;
    }
    nav_msgs::msg::Path path;assert(getInput("path",path));
    if(id_=="Departure") {
      assert(path.poses.size()==(probe_->single_point?1u:2u));
      if(probe_->follow_failed)return Status::FAILURE;
      return probe_->stopped||probe_->starts[key_]<=probe_->stopped_steps?Status::SUCCESS:Status::RUNNING;
    }
    assert(id_=="FollowPath");return probe_->target_done?Status::SUCCESS:Status::RUNNING;
  }
  Probe* probe_=nullptr;
  std::string id_,key_;
  bool planning_=false;
};

class SafePath : public BT::ConditionNode {
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("session"),BT::BidirectionalPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
  }
  Status tick() override {
    return config().blackboard->get<Probe*>("probe")->safe_path?Status::SUCCESS:Status::FAILURE;
  }
};

class MotionGuard : public BT::DecoratorNode {
public:
  using BT::DecoratorNode::DecoratorNode;
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<std::string>("reason"),BT::OutputPort<std::string>("session")};
  }
  Status tick() override {
    setStatus(Status::RUNNING);auto p=config().blackboard->get<Probe*>("probe");
    if(!p->envelope) {resetChild();return Status::FAILURE;}
    const auto result=child_node_->executeTick();if(result!=Status::RUNNING)resetChild();return result;
  }
};

class PolicySession : public BT::DecoratorNode {
public:
  using BT::DecoratorNode::DecoratorNode;
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),BT::OutputPort<std::string>("session")};
  }
  Status tick() override {
    setStatus(Status::RUNNING);setOutput("session",config().blackboard->get<Probe*>("probe")->execution);
    const auto result=child_node_->executeTick();if(result!=Status::RUNNING)resetChild();return result;
  }
};

struct Fixture {
  Probe probe;
  rclcpp::Node::SharedPtr node,server;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Service<Assessment>::SharedPtr service;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr alarms;
  rclcpp::Publisher<astribot_navigation_msgs::msg::NavigationEnvelopeV2>::SharedPtr envelopes;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> received;
  std::vector<std::shared_ptr<rmw_request_id_t>> deferred;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board;
  BT::Tree tree;
  Status status=Status::IDLE;

  Fixture(const char* library,const char* xml,std::initializer_list<uint8_t> answers,bool fixed=false) {
    probe.answers=answers;probe.goal.header.frame_id="map";probe.goal.pose.orientation.w=1.;
    probe.goal.pose.position.x=2.;
    node=std::make_shared<rclcpp::Node>("navigation_start_bt_test_client",rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("navigation_geometry_mode",std::string(fixed?"fixed_v2":"legacy")),
      rclcpp::Parameter("robot_base_frame",std::string("base_link"))}));
    server=std::make_shared<rclcpp::Node>("navigation_start_bt_test_assessment");executor.add_node(server);
    service=server->create_service<Assessment>("/navigation/assess_start",
      [this](std::shared_ptr<rmw_request_id_t> header,Assessment::Request::SharedPtr request) {
        assert(request->goal==probe.goal);probe.requests.push_back(request->execution_id);
        if(probe.defer) {deferred.push_back(header);return;}
        assert(!probe.answers.empty());Answer response=answer(probe.answers.front());
        if(!probe.repeat_answer||probe.answers.size()>1)probe.answers.pop_front();
        service->send_response(*header,response);
      });
    alarms=server->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/navigation/start_alarm",10,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr array) {
        received.insert(received.end(),array->status.begin(),array->status.end());
      });
    factory.registerFromPlugin(library);
    factory.registerNodeType<MotionAction>("ComputePathToPose");factory.registerNodeType<MotionAction>("FollowPath");
    factory.registerNodeType<SafePath>("KeepSafePath");
    factory.registerNodeType<MotionGuard>("RequireNavigationEnvelope");factory.registerNodeType<PolicySession>("PolicyExecution");
    board=BT::Blackboard::create();board->set("node",node);board->set("probe",&probe);board->set("goal",probe.goal);
    tree=factory.createTreeFromFile(xml,board);
    if(fixed)envelopes=server->create_publisher<astribot_navigation_msgs::msg::NavigationEnvelopeV2>("/navigation/envelope_v2",10);
  }
  ~Fixture() {tree.haltTree();executor.remove_node(server);}
  Answer answer(uint8_t state) {
    Answer result;result.state=state;result.reason="FIXTURE_REASON_"+std::to_string(state);
    result.header.stamp=server->now();result.header.frame_id="map";result.evaluated_start.header=result.header;result.evaluated_start.pose.orientation.w=1.;
    if(!probe.positions.empty()) {
      result.evaluated_start.pose.position.x=probe.positions.front().first;
      result.evaluated_start.pose.position.y=probe.positions.front().second;probe.positions.pop_front();
    }
    if(!probe.headings.empty()) {
      const double yaw=probe.headings.front();probe.headings.pop_front();
      result.evaluated_start.pose.orientation.z=std::sin(yaw*.5);
      result.evaluated_start.pose.orientation.w=std::cos(yaw*.5);
    }
    probe.current_pose=result.evaluated_start;
    probe.assessed_starts.push_back(result.evaluated_start);
    result.height_map_revision="height_1";result.costmap_revision="map_1";
    result.geometry_hash="geometry_1";result.envelope_epoch=7;return result;
  }
  void tick() {
    assert(status==Status::IDLE||status==Status::RUNNING);
    if(envelopes) {
      astribot_navigation_msgs::msg::NavigationEnvelopeV2 e;e.header.stamp=server->now();e.header.frame_id="base_link";
      e.valid_until=server->now()+rclcpp::Duration::from_seconds(.3);e.mode=e.FIXED_POSTURE;
      e.coordinator_session_id="fixture";e.installed_geometry_hash="geometry_1";e.epoch=7;
      e.navigation_allowed=e.limits.transport_ready=probe.envelope;e.limits.stamp=e.header.stamp;
      e.reason=probe.envelope?"READY_FIXED":"WAITING_FOR:controller";envelopes->publish(e);
    }
    executor.spin_some();status=tree.tickRoot();executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  template<class Predicate> void until(Predicate done) {
    const auto deadline=Clock::now()+std::chrono::seconds(4);
    while(!done()) {assert(Clock::now()<deadline);tick();}
  }
  void terminal() {until([this]{return status==Status::SUCCESS||status==Status::FAILURE;});}
  void halt() {tree.haltTree();status=Status::IDLE;}
  bool hasStage(const std::string &stage) const {
    for(const auto &s:received)for(const auto &v:s.values)if(v.key=="stage"&&v.value==stage)return true;
    return false;
  }
  void diagnostic(const std::string& stage,const std::string& reason) {
    const auto deadline=Clock::now()+std::chrono::seconds(1);
    for(;;) {
      executor.spin_some();
      for(const auto& s:received)if(s.level==diagnostic_msgs::msg::DiagnosticStatus::ERROR&&s.message.find(reason)!=std::string::npos) {
        bool matched=false;for(const auto& v:s.values)if(v.key=="stage"&&v.value==stage)matched=true;
        assert(matched);return;
      }
      assert(Clock::now()<deadline);std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};

int main(int argc,char** argv) {
  assert(argc==3);rclcpp::init(argc,argv);
  {
    Fixture f(argv[1],argv[2],{Answer::READY});f.probe.target_done=true;f.terminal();
    assert(f.status==Status::SUCCESS&&f.probe.requests.size()==1);
    assert(f.probe.starts["plan:Departure"]==0&&f.probe.starts["follow:Departure"]==0);
    assert(f.probe.starts["plan:GridBased"]==1);
    f.probe.execution="after_success";f.probe.answers.push_back(Answer::READY);f.status=Status::IDLE;
    f.terminal();assert(f.status==Status::SUCCESS&&f.probe.requests.size()==2);
  }
  for(bool single:{false,true}) {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED,Answer::READY});f.probe.single_point=single;
    f.until([&]{return f.probe.starts["follow:Departure"]==1;});
    for(int i=0;i<5;++i)f.tick();
    assert(f.probe.requests.size()==1&&f.probe.starts["plan:GridBased"]==0);
    f.probe.stopped=true;f.until([&]{return f.probe.starts["follow:FollowPath"]==1;});
    assert(f.probe.requests.size()==2&&f.probe.starts["plan:Departure"]==1);
    f.probe.safe_path=true;f.tick();assert(f.probe.starts["plan:GridBased"]==1);
    f.probe.safe_path=false;f.tick();assert(f.probe.starts["plan:GridBased"]==2);
    assert(f.probe.requests.size()==2&&f.probe.starts["follow:Departure"]==1);
    f.halt();assert(f.probe.halts["follow:FollowPath"]==1);
    f.probe.execution="execution_b";f.probe.goal.pose.position.x=3.;f.board->set("goal",f.probe.goal);
    f.probe.answers.push_back(Answer::READY);f.probe.target_done=true;f.terminal();
    assert(f.status==Status::SUCCESS&&f.probe.requests.back()=="execution_b");
  }
  // Every short step must finish its stop acknowledgement before another check.
  {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED,Answer::RECOVERY_REQUIRED,Answer::READY});
    f.probe.positions={{0.,0.},{-.2,0.},{-.4,0.}};
    f.until([&]{return f.probe.starts["follow:Departure"]==1;});
    for(int i=0;i<5;++i)f.tick();
    assert(f.probe.requests.size()==1&&f.probe.starts["plan:GridBased"]==0);
    f.probe.stopped_steps=1;f.until([&]{return f.probe.starts["follow:Departure"]==2;});
    for(int i=0;i<5;++i)f.tick();
    assert(f.probe.requests.size()==2&&f.probe.starts["plan:GridBased"]==0);
    f.probe.stopped_steps=2;f.probe.target_done=true;f.terminal();
    assert(f.status==Status::SUCCESS&&f.probe.requests.size()==3&&f.probe.starts["plan:GridBased"]==1);
  }
  // Repeated recovery re-assesses translated/rotated actual positions. The
  // action doubles return paths ending at goal=(2,0), deliberately different
  // from every measured stop, so that endpoint reuse cannot pass this contract.
  {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED,Answer::RECOVERY_REQUIRED,Answer::READY});
    f.probe.positions={{.25,.15},{.24,.04},{.22,-.08}};
    f.probe.headings={1.57,1.55,1.53};
    f.until([&]{return f.probe.starts["follow:Departure"]==1;});
    assert(f.probe.planning_starts["Departure"].at(0)==f.probe.assessed_starts.at(0));
    assert(f.probe.starts["plan:GridBased"]==0);
    f.probe.stopped_steps=1;
    f.until([&]{return f.probe.starts["follow:Departure"]==2;});
    assert(f.probe.planning_starts["Departure"].at(1)==f.probe.assessed_starts.at(1));
    assert(f.probe.starts["plan:GridBased"]==0);
    f.probe.stopped_steps=2;f.probe.target_done=true;f.terminal();
    assert(f.status==Status::SUCCESS&&f.probe.requests.size()==3);
    const auto &normal=f.probe.planning_starts["GridBased"].at(0);
    assert(normal==f.probe.assessed_starts.at(2));
    assert(normal.pose.position.x==.22&&normal.pose.position.y==-.08);
    assert(normal.pose.orientation.z==std::sin(1.53*.5));
    assert(normal!=f.probe.assessed_starts.at(0)&&normal!=f.probe.goal);
  }
  for(uint8_t state:{Answer::BLOCKED,uint8_t(42)}) {
    Fixture f(argv[1],argv[2],{state});f.terminal();assert(f.status==Status::FAILURE);
    assert(f.probe.starts["plan:Departure"]==0&&f.probe.starts["plan:GridBased"]==0);
    f.diagnostic("CHECK","FIXTURE_REASON_"+std::to_string(state));
  }
  {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED,Answer::BLOCKED});f.probe.stopped=true;f.terminal();
    assert(f.status==Status::FAILURE&&f.probe.requests.size()==2);
    assert(f.probe.starts["plan:Departure"]==1&&f.probe.starts["plan:GridBased"]==0);
    f.diagnostic("RECHECK","ASSESSMENT_REJECTED:FIXTURE_REASON_2");
  }
  // Missing input is a bounded wait, not a trapped-start decision or permission.
  {
    Fixture f(argv[1],argv[2],{Answer::UNAVAILABLE,Answer::READY});
    f.until([&]{return f.hasStage("WAIT_INPUT");});
    assert(f.status==Status::RUNNING&&f.probe.starts["plan:Departure"]==0&&f.probe.starts["plan:GridBased"]==0);
    f.probe.target_done=true;f.terminal();
    assert(f.status==Status::SUCCESS&&f.probe.requests.size()==2&&f.probe.starts["plan:GridBased"]==1);
  }
  {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED,Answer::UNAVAILABLE,Answer::READY});f.probe.stopped=true;
    f.probe.positions={{0.,0.},{-.2,0.},{-.2,0.}};
    f.until([&]{return f.hasStage("WAIT_INPUT");});
    assert(f.probe.starts["follow:Departure"]==1&&f.probe.starts["plan:GridBased"]==0);
    f.probe.target_done=true;f.terminal();
    assert(f.status==Status::SUCCESS&&f.probe.requests.size()==3);
  }
  {
    Fixture f(argv[1],argv[2],{Answer::UNAVAILABLE});f.probe.repeat_answer=true;
    const auto start=Clock::now();f.terminal();
    assert(f.status==Status::FAILURE&&Clock::now()-start>=std::chrono::seconds(2));
    assert(f.probe.requests.size()>1&&f.probe.starts["plan:Departure"]==0&&f.probe.starts["plan:GridBased"]==0);
    f.diagnostic("WAIT_INPUT","START_INPUT_TIMEOUT");
  }
  // No displacement and a return to a prior location both terminate recovery.
  for(bool returned:{false,true}) {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED,Answer::RECOVERY_REQUIRED});
    f.probe.positions={{0.,0.},{returned?-.2:0.,0.}};
    if(returned) {f.probe.answers.push_back(Answer::RECOVERY_REQUIRED);f.probe.positions.push_back({0.,0.});}
    f.probe.stopped=true;f.terminal();
    assert(f.status==Status::FAILURE&&f.probe.starts["plan:Departure"]==(returned?2:1)&&f.probe.starts["plan:GridBased"]==0);
    f.diagnostic("RECHECK","START_RECOVERY_NO_PROGRESS");
  }
  // The actual plugin consumes fixed_v2 permissions. PolicySession must not
  // prevent assessment while ACKs are pending; neither motion nor normal plan runs.
  for(uint8_t first:{Answer::READY,Answer::RECOVERY_REQUIRED}) {
    Fixture f(argv[1],argv[2],{first,Answer::READY},true);f.probe.envelope=false;
    f.until([&]{return f.hasStage("WAIT_PERMISSION");});
    for(int i=0;i<20;++i)f.tick();
    assert(f.status==Status::RUNNING&&f.probe.requests.size()==1&&f.probe.starts["plan:Departure"]==0&&f.probe.starts["plan:GridBased"]==0);
    f.probe.envelope=true;f.probe.target_done=true;f.terminal();
    assert(f.status==Status::SUCCESS&&f.probe.requests.size()==2&&f.probe.starts["plan:Departure"]==0&&f.probe.starts["plan:GridBased"]==1);
  }
  {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED},true);f.probe.envelope=false;
    f.terminal();assert(f.status==Status::FAILURE&&f.probe.requests.size()==1);
    assert(f.probe.starts["plan:Departure"]==0&&f.probe.starts["plan:GridBased"]==0);
    f.diagnostic("WAIT_PERMISSION","START_PERMISSION_TIMEOUT:");
  }
  // A committed short step is not interrupted by another envelope admission.
  // Only after its measured stop do refreshed inputs gate normal planning.
  {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED,Answer::READY,Answer::READY},true);
    f.until([&]{return f.probe.starts["follow:Departure"]==1;});
    f.probe.envelope=false;
    for(int i=0;i<20;++i)f.tick();
    assert(f.status==Status::RUNNING&&f.probe.halts["follow:Departure"]==0);
    assert(f.probe.requests.size()==1&&f.probe.starts["plan:GridBased"]==0);
    f.probe.stopped=true;
    f.until([&]{return f.hasStage("WAIT_PERMISSION");});
    assert(f.probe.requests.size()==2&&f.probe.starts["plan:GridBased"]==0);
    f.probe.envelope=true;f.probe.target_done=true;f.terminal();
    assert(f.status==Status::SUCCESS&&f.probe.requests.size()==3&&f.probe.starts["plan:GridBased"]==1);
  }
  for(bool planner:{false,true}) {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED});
    f.probe.plan_failed=planner;f.probe.follow_failed=!planner;f.terminal();
    assert(f.status==Status::FAILURE&&f.probe.requests.size()==1&&f.probe.starts["plan:GridBased"]==0);
    f.diagnostic("RECOVER","DEPARTURE_CHILD_FAILURE");
  }
  {
    Fixture f(argv[1],argv[2],{});f.probe.defer=true;const auto start=Clock::now();f.terminal();
    assert(f.status==Status::FAILURE&&Clock::now()-start>=std::chrono::seconds(2));
    assert(f.probe.starts["plan:Departure"]==0);f.diagnostic("CHECK","ASSESSMENT_TIMEOUT");
  }
  {
    Fixture f(argv[1],argv[2],{});f.probe.defer=true;
    f.until([&]{return !f.deferred.empty();});f.halt();
    auto late=f.answer(Answer::READY);f.service->send_response(*f.deferred.front(),late);
    f.probe.execution="after_cancel";f.probe.defer=false;f.probe.answers.push_back(Answer::BLOCKED);f.terminal();
    assert(f.status==Status::FAILURE&&f.probe.requests.size()==2);
    assert(f.probe.starts["plan:GridBased"]==0);f.diagnostic("CHECK","FIXTURE_REASON_2");
  }
  for(bool changed:{false,true}) {
    Fixture f(argv[1],argv[2],{Answer::RECOVERY_REQUIRED});
    f.until([&]{return f.probe.starts["follow:Departure"]==1;});
    if(changed) {f.probe.execution="unexpected_new_execution";f.terminal();assert(f.status==Status::FAILURE);}
    else f.halt();
    assert(f.probe.halts["follow:Departure"]==1&&f.probe.starts["plan:GridBased"]==0);
    if(changed)f.diagnostic("RECOVER","NAVIGATION_START_EXECUTION_CHANGED");
  }
  rclcpp::shutdown();
  std::cout<<"EnsureNavigationStart BT: multistep stop/recheck, input and permission waits, no-progress, cancellation and task changes passed; motion/stop remain fixtures\n";
}
