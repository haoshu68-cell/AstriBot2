#include "astribot_s1_transport_native/fixed_station_navigation.hpp"
#include "astribot_s1_transport_native/slam_pose_stop_window.hpp"
#include <astribot_navigation_msgs/msg/envelope_apply_status.hpp>
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace astribot::transport {
namespace {
constexpr int64_t MS=1000000;
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Ack=astribot_navigation_msgs::msg::EnvelopeApplyStatus;
using SlamPose=geometry_msgs::msg::PoseWithCovarianceStamped;
using Command=geometry_msgs::msg::Twist;
using Nav=nav2_msgs::action::NavigateToPose;
using Handle=rclcpp_action::ClientGoalHandle<Nav>;
const std::array<std::string,5> CONSUMERS={"global_costmap","local_costmap","planner","controller","policy"};
int64_t stamp(const builtin_interfaces::msg::Time& t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
int64_t wall(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Time {int64_t ros,steady;};
template<class T>struct Received {T value;Time at;};
bool pose_valid(const geometry_msgs::msg::Pose& p) {
  const auto& q=p.orientation;
  return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&std::isfinite(p.position.z)&&
    std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z)&&std::isfinite(q.w)&&
    std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<=.001;
}
double yaw(const geometry_msgs::msg::Quaternion& q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
double command_max(const Command& v){return std::max({std::abs(v.linear.x),std::abs(v.linear.y),std::abs(v.angular.z)});}
std::string uuid(const rclcpp_action::GoalUUID& id) {
  std::ostringstream out;out<<std::hex<<std::setfill('0');for(auto b:id)out<<std::setw(2)<<unsigned(b);return out.str();
}
void require(bool okay,const char* why){if(!okay)throw std::invalid_argument(why);}
} // namespace

struct FixedStationNavigation::Impl {
  struct Sample {Time at;int64_t command_at;std::optional<double> command;};
  rclcpp::Node& node;
  std::string workstation_bt;
  Status result;
  Request request;
  rclcpp::Client<Fixed>::SharedPtr fixed;
  rclcpp_action::Client<Nav>::SharedPtr nav;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::optional<rclcpp::Client<Fixed>::FutureAndRequestId> fixed_future;
  std::shared_future<Handle::SharedPtr> goal_future;
  std::shared_future<Handle::WrappedResult> result_future;
  std::shared_future<rclcpp_action::Client<Nav>::CancelResponse::SharedPtr> cancel_future;
  Handle::SharedPtr handle;
  std::optional<Received<Envelope>> envelope;
  std::map<std::string,Received<Ack>> acks;
  std::optional<Received<SlamPose>> slam_pose;
  std::optional<std::pair<Command,Time>> command;
  std::deque<Sample> samples;
  SlamPoseStopWindow stop_window;
  std::optional<Time> previous;
  int64_t phase_started=0,stop_after=0,cancel_started=0,nav_started=0;
  bool nav_sent=false,permission_bound=false,cancel_sent=false,stopping=false,command_observed=false;

  Impl(rclcpp::Node& n,std::string endpoint):node(n) {
    require(endpoint=="/navigate_to_pose","UNIFIED_NAVIGATION_ENDPOINT_REQUIRED");
    if(!node.has_parameter("workstation_navigation_bt"))
      node.declare_parameter<std::string>("workstation_navigation_bt",
        ament_index_cpp::get_package_share_directory("astribot_s1_navigation")+
        "/behavior_trees/navigate_to_workstation.xml");
    workstation_bt=node.get_parameter("workstation_navigation_bt").as_string();
    result.endpoint=std::move(endpoint);
    fixed=node.create_client<Fixed>("/navigation/set_fixed_envelope");
    nav=rclcpp_action::create_client<Nav>(&node,result.endpoint);
    subscriptions.push_back(node.create_subscription<Envelope>("/navigation/envelope_v2",10,[this](Envelope::ConstSharedPtr v){receive(*v);}));
    subscriptions.push_back(node.create_subscription<Ack>("/navigation/envelope_applied",20,[this](Ack::ConstSharedPtr v){receive(*v);}));
    subscriptions.push_back(node.create_subscription<SlamPose>("/slam/pose",rclcpp::SensorDataQoS(),[this](SlamPose::ConstSharedPtr v){receive(*v);}));
    subscriptions.push_back(node.create_subscription<Command>("/cmd_vel",rclcpp::SensorDataQoS(),[this](Command::ConstSharedPtr v){receive(*v);}));
  }
  Time now()const{return {node.now().nanoseconds(),wall()};}
  bool active()const{return result.phase!=Phase::IDLE&&result.phase!=Phase::SUCCEEDED&&result.phase!=Phase::FAILED;}
  void fail(const std::string& reason) {
    if(!active()||stopping)return;
    stopping=true;result.reason=reason;result.phase=Phase::CANCELING;cancel_started=wall();
    stop_after=wall();samples.clear();stop_window.clear();result.measured_stopped=false;
  }
  bool clock(Time at) {
    if(previous&&(at.steady<previous->steady)){fail("NAVIGATION_CLOCK_ROLLBACK");return false;}
    previous=at;return true;
  }
  bool binding(const Envelope& e)const {
    const auto& g=request.reference_geometry;
    return e.request_id==request.fixed.request_id&&e.hold_id==request.fixed.hold_id&&
      e.epoch==result.envelope_epoch&&e.clock_epoch==g.clock_epoch&&e.model_revision==g.model_revision&&
      e.attachment_revision==g.attachment_revision&&e.mode==Envelope::FIXED_POSTURE&&
      !e.coordinator_session_id.empty()&&!e.installed_geometry_hash.empty()&&
      e.reference_state_sequence==request.fixed.geometry_sequence&&e.header.frame_id==g.header.frame_id&&
      e.limits.payload_mass_kg==request.fixed.limits.payload_mass_kg;
  }
  void receive(const Envelope& v) {
    // After the initial handoff, Nav2 owns admission for recovery and normal
    // motion. This task helper owns only its goal, feedback and stop lifecycle.
    if(!active()||stopping||permission_bound)return;
    const auto at=now();if(!clock(at))return;
    if(v.request_id!=request.fixed.request_id)return;
    const auto source=stamp(v.header.stamp);
    if(source<=0||v.header.stamp.nanosec>=1000000000u||v.valid_until.nanosec>=1000000000u){fail("NAVIGATION_ENVELOPE_INVALID");return;}
    if(envelope&&source<stamp(envelope->value.header.stamp))return;
    if(envelope&&source==stamp(envelope->value.header.stamp)) {
      envelope->value=v;
    } else envelope=Received<Envelope>{v,at};
  }
  void receive(const Ack& v) {
    if(!active()||stopping||permission_bound)return;
    const auto at=now();if(!clock(at))return;
    if(std::find(CONSUMERS.begin(),CONSUMERS.end(),v.consumer_id)==CONSUMERS.end())return;
    if(!envelope||v.coordinator_session_id!=envelope->value.coordinator_session_id||v.envelope_epoch!=envelope->value.epoch||v.installed_geometry_hash!=envelope->value.installed_geometry_hash)return;
    const auto source=stamp(v.header.stamp);
    if(source<=0||v.header.stamp.nanosec>=1000000000u){fail("NAVIGATION_ACK_INVALID");return;}
    auto old=acks.find(v.consumer_id);
    if(old!=acks.end()&&source<stamp(old->second.value.header.stamp))return;
    if(old!=acks.end()&&source==stamp(old->second.value.header.stamp)) {
      if(v!=old->second.value)fail("NAVIGATION_ACK_CONFLICT:"+v.consumer_id);
      return;
    }
    acks[v.consumer_id]={v,at};
  }
  void receive(const Command& v) {
    if(!active())return;
    command_observed=true;
    const auto at=now();if(!clock(at))return;
    if(!std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.angular.z)){fail("NAVIGATION_COMMAND_INVALID");return;}
    if(!stopping&&nav_sent&&command&&command_max(command->first)>1e-6&&at.steady-command->second.steady>=300*MS)
      fail("NAVIGATION_COMMAND_STALE");
    // Preserve a real late zero for cleanup even when its preceding nonzero expired.
    command=std::make_pair(v,at);
  }
  void receive(const SlamPose& v) {
    if(!active())return;
    const auto at=now();if(!clock(at))return;
    const auto source=stamp(v.header.stamp);
    if(slam_pose&&source<=stamp(slam_pose->value.header.stamp))return;
    if(v.header.frame_id!="map"||!pose_valid(v.pose.pose)||source<=0||v.header.stamp.nanosec>=1000000000u){fail("NAVIGATION_SLAM_POSE_INVALID");return;}
    slam_pose=Received<SlamPose>{v,at};
    stop_window.observe(v.pose.pose,at.steady);
    samples.push_back({at,command?command->second.steady:0,
      command?std::optional<double>(command_max(command->first)):std::optional<double>{}});
    while(samples.size()>2&&samples[1].at.steady<=at.steady-600*MS)samples.pop_front();
  }
  bool stopped(Time /*at*/)const {
    // Stop duration is measured only by the local monotonic clock.
    if(!stop_window.stopped()||samples.front().at.steady<=stop_after||
      !slam_pose)return false;
    if(command&&command_max(command->first)>1e-6)return false;
    // A failed/canceled goal may terminate before producing any velocity command.
    // Its confirmed terminal plus this post-terminal SLAM window proves stop;
    // an observed nonzero still needs a real zero, and success keeps its zero gate.
    const bool silent_failure=stopping&&result.nav_terminal&&!command_observed;
    if(nav_sent&&(!result.nav_terminal||(!silent_failure&&(!command||command->second.steady<nav_started))))return false;
    int64_t last=samples.front().at.steady;
    for(auto it=samples.begin();it!=samples.end();++it) {
      const auto& s=*it;
      if(it!=samples.begin()&&s.at.steady<=last)return false;
      if(nav_sent&&!silent_failure&&(!s.command||s.command_at<nav_started))return false;
      if(s.command&&(s.at.steady<s.command_at||*s.command>1e-6))return false;
      last=s.at.steady;
    }
    return true;
  }
  bool ready(Time /*at*/) {
    if(!envelope){result.reason="WAITING_FOR_NAVIGATION_ENVELOPE";return false;}
    if(!binding(envelope->value)){result.reason="WAITING_FOR_MATCHING_NAVIGATION_ENVELOPE";return false;}
    if(!envelope->value.navigation_allowed||!envelope->value.limits.transport_ready){result.reason="WAITING_FOR_NAVIGATION_PERMISSION";return false;}
    for(const auto& consumer:CONSUMERS) {
      auto found=acks.find(consumer);if(found==acks.end()){result.reason="WAITING_FOR_NAVIGATION_ACK:"+consumer;return false;}
      const auto& a=found->second;
      if(!a.value.applied||a.value.coordinator_session_id!=envelope->value.coordinator_session_id||
        a.value.envelope_epoch!=result.envelope_epoch||a.value.installed_geometry_hash!=envelope->value.installed_geometry_hash){result.reason="WAITING_FOR_VALID_NAVIGATION_ACK:"+consumer;return false;}
    }
    return true;
  }
  void start(Request value) {
    require(result.phase==Phase::IDLE,"NAVIGATION_HELPER_SINGLE_LEG_ONLY");
    require(!workstation_bt.empty(),"WORKSTATION_NAVIGATION_BT_REQUIRED");
    require(!value.task_id.empty()&&!value.context_id.empty()&&!value.lease_id.empty()&&!value.resource_epoch.empty(),"NAVIGATION_OWNER_BINDING_REQUIRED");
    const auto& g=value.reference_geometry;
    require(g.complete&&g.attachment_state_confirmed&&g.attachment_ids.size()==1&&!g.attachment_ids.front().empty()&&
      !g.source_id.empty()&&!g.model_revision.empty()&&!g.attachment_revision.empty()&&g.sequence==value.fixed.geometry_sequence&&
      !value.fixed.request_id.empty()&&!value.fixed.hold_id.empty(),"NAVIGATION_PICK_BINDING_REQUIRED");
    require(value.navigation_target.header.frame_id=="map"&&!value.robot_base_frame.empty()&&pose_valid(value.navigation_target.pose)&&
      std::isfinite(value.position_tolerance_m)&&value.position_tolerance_m>0&&
      std::isfinite(value.yaw_tolerance_rad)&&value.yaw_tolerance_rad>0,"NAVIGATION_STATION_TARGET_INVALID");
    require(fixed->service_is_ready()&&nav->action_server_is_ready(),"NAVIGATION_ENDPOINT_UNAVAILABLE");
    request=std::move(value);const auto at=now();phase_started=at.steady;stop_after=at.steady;previous=at;
    result.phase=Phase::WAIT_FIXED;result.reason="WAITING_FOR_FIXED_ENVELOPE";result.fixed_request_terminal=false;
    try{fixed_future.emplace(fixed->async_send_request(std::make_shared<Fixed::Request>(request.fixed)));}
    catch(const std::exception& e){fail(std::string("FIXED_SEND_UNCONFIRMED:")+e.what());}
  }
  void poll(Time at) {
    using namespace std::chrono_literals;
    if(fixed_future&&fixed_future->wait_for(0s)==std::future_status::ready) {
      auto answer=fixed_future->get();fixed_future.reset();result.fixed_request_terminal=true;
      result.envelope_epoch=answer->epoch;
      if(!stopping) {
        if(!answer->accepted||answer->epoch==0)fail("FIXED_ENVELOPE_REJECTED:"+answer->reason);
        else {result.phase=Phase::WAIT_ACK;result.reason="WAITING_FOR_NAVIGATION_INTENT_STOP";phase_started=at.steady;}
      }
    }
    if(nav_sent&&!handle&&goal_future.valid()&&goal_future.wait_for(0s)==std::future_status::ready) {
      handle=goal_future.get();goal_future={};
      if(!handle){result.nav_terminal=true;fail("NAVIGATION_GOAL_REJECTED");}
      else {result.nav_goal_uuid=uuid(handle->get_goal_id());result_future=nav->async_get_result(handle);}
    }
    if(result_future.valid()&&result_future.wait_for(0s)==std::future_status::ready) {
      const auto answer=result_future.get();result_future={};
      if(!handle||answer.goal_id!=handle->get_goal_id()){fail("NAVIGATION_RESULT_UUID_MISMATCH");return;}
      const bool terminal=answer.code==rclcpp_action::ResultCode::SUCCEEDED||answer.code==rclcpp_action::ResultCode::ABORTED||answer.code==rclcpp_action::ResultCode::CANCELED;
      result.nav_terminal=terminal;
      if(!terminal){fail("NAVIGATION_RESULT_UNKNOWN");return;}
      if(!stopping) {
        if(answer.code!=rclcpp_action::ResultCode::SUCCEEDED)fail("NAVIGATION_NOT_SUCCEEDED");
        else if(!permission_bound)fail("NAVIGATION_SUCCEEDED_WITHOUT_PERMISSION");
        else {result.phase=Phase::SETTLING;result.reason="WAITING_FOR_ARRIVAL_STOP";phase_started=at.steady;stop_after=at.steady;samples.clear();stop_window.clear();}
      } else {stop_after=at.steady;samples.clear();stop_window.clear();}
    }
    if(cancel_future.valid()&&cancel_future.wait_for(0s)==std::future_status::ready) {
      (void)cancel_future.get();cancel_future={}; // ACK cannot prove terminal or physical stop.
    }
  }
  void tick() {
    if(!active())return;
    const auto at=now();clock(at);
    try {
      poll(at);
      if(stopping) {
        if(handle&&!result.nav_terminal&&!cancel_sent) {cancel_sent=true;cancel_future=nav->async_cancel_goal(handle);}
        result.measured_stopped=stopped(at);
        result.cleanup_complete=result.fixed_request_terminal&&result.nav_terminal&&result.measured_stopped;
        if(result.cleanup_complete)result.phase=Phase::FAILED;
        else if(at.steady-cancel_started>=10000*MS)result.phase=Phase::UNRESOLVED;
        return;
      }
      if(result.phase==Phase::WAIT_FIXED&&at.steady-phase_started>=3000*MS){fail("FIXED_ENVELOPE_REPLY_TIMEOUT");return;}
      if(nav_sent&&!handle&&!result.nav_terminal&&at.steady-nav_started>=5000*MS) {
        fail("NAVIGATION_ACCEPTANCE_TIMEOUT");return;
      }
      if(result.phase==Phase::WAIT_ACK) {
        if(at.steady-phase_started>=15000*MS){fail("FIVE_ACK_OR_STOP_TIMEOUT:"+result.reason);return;}
        if(!nav_sent) {
          if(!stopped(at)){result.reason="WAITING_FOR_NAVIGATION_INTENT_STOP";return;}
          // Sending the intention starts the BT's input/start diagnosis; it is
          // not a motion permit. The BT owns the first motion admission.
          Nav::Goal goal;goal.pose=request.navigation_target;goal.pose.header.stamp=node.now();
          goal.behavior_tree=workstation_bt;
          nav_sent=true;nav_started=at.steady;result.nav_terminal=false;goal_future=nav->async_send_goal(goal);
        }
        if(!ready(at))return;
        result.envelope_session=envelope->value.coordinator_session_id;result.geometry_hash=envelope->value.installed_geometry_hash;
        permission_bound=true;result.phase=Phase::NAVIGATING;result.reason="NAVIGATING";phase_started=at.steady;
        return;
      }
      if(result.phase==Phase::NAVIGATING||result.phase==Phase::SETTLING) {
        if(!slam_pose){fail("NAVIGATION_MOTION_OBSERVATION_UNAVAILABLE");return;}
        if(command&&command_max(command->first)>1e-6&&at.steady-command->second.steady>=300*MS){fail("NAVIGATION_COMMAND_STALE");return;}
        if(result.phase==Phase::NAVIGATING) {
          if(at.steady-nav_started>=180000*MS)fail("NAVIGATION_RESULT_TIMEOUT");
          return;
        }
        result.measured_stopped=stopped(at);
        if(!result.measured_stopped){if(at.steady-phase_started>=15000*MS)fail("NAVIGATION_ARRIVAL_STOP_TIMEOUT");return;}
        const auto& actual=slam_pose->value.pose.pose;const auto& target=request.navigation_target.pose;
        result.position_error_m=std::hypot(actual.position.x-target.position.x,actual.position.y-target.position.y);
        result.yaw_error_rad=std::abs(std::remainder(yaw(actual.orientation)-yaw(target.orientation),2*std::acos(-1.)));
        if(result.position_error_m>request.position_tolerance_m||result.yaw_error_rad>request.yaw_tolerance_rad){fail("NAVIGATION_ARRIVAL_OUT_OF_TOLERANCE");return;}
        result.phase=Phase::SUCCEEDED;result.reason="NAVIGATION_ARRIVED_AND_STOPPED";result.cleanup_complete=true;
      }
    } catch(const std::exception& e) {
      if(stopping){result.cleanup_reason=e.what();result.phase=Phase::UNRESOLVED;}
      else fail(std::string("NAVIGATION_ASYNC_ERROR:")+e.what());
    }
  }
};
FixedStationNavigation::FixedStationNavigation(rclcpp::Node& node,std::string endpoint)
  :impl_(std::make_unique<Impl>(node,std::move(endpoint))){}
FixedStationNavigation::~FixedStationNavigation()=default;
void FixedStationNavigation::start(Request request){impl_->start(std::move(request));}
void FixedStationNavigation::tick(){impl_->tick();}
void FixedStationNavigation::cancel(const std::string& reason){impl_->fail(reason);}
const FixedStationNavigation::Status& FixedStationNavigation::status()const{return impl_->result;}
} // namespace astribot::transport
