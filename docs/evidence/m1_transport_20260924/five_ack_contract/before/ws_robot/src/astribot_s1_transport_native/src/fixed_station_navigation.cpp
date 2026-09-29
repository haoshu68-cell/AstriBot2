#include "astribot_s1_transport_native/fixed_station_navigation.hpp"
#include <astribot_navigation_msgs/msg/envelope_apply_status.hpp>
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
using Odom=nav_msgs::msg::Odometry;
using Command=geometry_msgs::msg::Twist;
using Nav=nav2_msgs::action::NavigateToPose;
using Handle=rclcpp_action::ClientGoalHandle<Nav>;
const std::array<std::string,6> CONSUMERS={"global_costmap","local_costmap","planner","controller","policy","protection"};
int64_t stamp(const builtin_interfaces::msg::Time& t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
int64_t wall(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Time {int64_t ros,steady;};
template<class T>struct Received {T value;Time at;int64_t until;};
bool fresh(int64_t source,int64_t until,Time at,Time now) {
  return source>0&&source<=at.ros&&at.ros<=now.ros&&now.ros<until&&at.steady<=now.steady&&now.steady-at.steady<until-at.ros;
}
template<class T>bool fresh(const Received<T>& value,Time now){return fresh(stamp(value.value.header.stamp),value.until,value.at,now);}
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
  struct Sample {int64_t source;Time at;int64_t command_at;double x,y,angle,v,w,command;};
  rclcpp::Node& node;
  Status result;
  Request request;
  rclcpp::Client<Fixed>::SharedPtr fixed;
  rclcpp_action::Client<Nav>::SharedPtr nav;
  tf2_ros::Buffer tf;
  tf2_ros::TransformListener listener;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::optional<rclcpp::Client<Fixed>::FutureAndRequestId> fixed_future;
  std::shared_future<Handle::SharedPtr> goal_future;
  std::shared_future<Handle::WrappedResult> result_future;
  std::shared_future<rclcpp_action::Client<Nav>::CancelResponse::SharedPtr> cancel_future;
  Handle::SharedPtr handle;
  std::optional<Received<Envelope>> envelope;
  std::map<std::string,Received<Ack>> acks;
  std::optional<Received<Odom>> odom;
  std::optional<std::pair<Command,Time>> command;
  std::deque<Sample> samples;
  std::optional<Time> previous;
  int64_t phase_started=0,stop_after=0,cancel_started=0;
  bool nav_sent=false,cancel_sent=false,stopping=false;
  std::string odom_child;

  Impl(rclcpp::Node& n,std::string endpoint):node(n),tf(n.get_clock()),listener(tf,&n,false) {
    require(endpoint=="/navigate_to_pose","UNIFIED_NAVIGATION_ENDPOINT_REQUIRED");
    result.endpoint=std::move(endpoint);
    fixed=node.create_client<Fixed>("/navigation/set_fixed_envelope");
    nav=rclcpp_action::create_client<Nav>(&node,result.endpoint);
    subscriptions.push_back(node.create_subscription<Envelope>("/navigation/envelope_v2",10,[this](Envelope::ConstSharedPtr v){receive(*v);}));
    subscriptions.push_back(node.create_subscription<Ack>("/navigation/envelope_applied",20,[this](Ack::ConstSharedPtr v){receive(*v);}));
    subscriptions.push_back(node.create_subscription<Odom>("/odom",rclcpp::SensorDataQoS(),[this](Odom::ConstSharedPtr v){receive(*v);}));
    subscriptions.push_back(node.create_subscription<Command>("/cmd_vel",rclcpp::SensorDataQoS(),[this](Command::ConstSharedPtr v){receive(*v);}));
  }
  Time now()const{return {node.now().nanoseconds(),wall()};}
  bool active()const{return result.phase!=Phase::IDLE&&result.phase!=Phase::SUCCEEDED&&result.phase!=Phase::FAILED;}
  void fail(const std::string& reason) {
    if(!active()||stopping)return;
    stopping=true;result.reason=reason;result.phase=Phase::CANCELING;cancel_started=wall();
    stop_after=node.now().nanoseconds();samples.clear();result.measured_stopped=false;
  }
  bool clock(Time at) {
    if(previous&&(at.ros<previous->ros||at.steady<previous->steady)){fail("NAVIGATION_CLOCK_ROLLBACK");return false;}
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
    if(!active()||stopping)return;
    const auto at=now();if(!clock(at))return;
    if(v.request_id!=request.fixed.request_id){if(nav_sent)fail("NAVIGATION_ENVELOPE_REPLACED");return;}
    if(nav_sent&&envelope&&!fresh(*envelope,at)){fail("NAVIGATION_ENVELOPE_EXPIRED");return;}
    const auto source=stamp(v.header.stamp),until=std::min(stamp(v.valid_until),source+300*MS);
    if(source<=0||source>at.ros||v.header.stamp.nanosec>=1000000000u||v.valid_until.nanosec>=1000000000u){fail("NAVIGATION_ENVELOPE_INVALID");return;}
    if(envelope&&source<stamp(envelope->value.header.stamp)){if(nav_sent)fail("NAVIGATION_ENVELOPE_ROLLBACK");return;}
    if(envelope&&source==stamp(envelope->value.header.stamp)) {
      envelope->value=v;envelope->until=std::min(envelope->until,until);
    } else envelope=Received<Envelope>{v,at,until};
    if(nav_sent&&(!binding(v)||v.coordinator_session_id!=result.envelope_session||v.installed_geometry_hash!=result.geometry_hash||
      !v.navigation_allowed||!v.limits.transport_ready||!fresh(*envelope,at)))fail("NAVIGATION_ENVELOPE_LOST");
  }
  void receive(const Ack& v) {
    if(!active()||stopping)return;
    const auto at=now();if(!clock(at))return;
    if(std::find(CONSUMERS.begin(),CONSUMERS.end(),v.consumer_id)==CONSUMERS.end())return;
    if(!envelope||v.coordinator_session_id!=envelope->value.coordinator_session_id||v.envelope_epoch!=envelope->value.epoch||v.installed_geometry_hash!=envelope->value.installed_geometry_hash)return;
    const auto source=stamp(v.header.stamp);
    if(source<=0||source>at.ros||v.header.stamp.nanosec>=1000000000u){fail("NAVIGATION_ACK_INVALID");return;}
    auto old=acks.find(v.consumer_id);
    if(nav_sent&&old!=acks.end()&&!fresh(old->second,at)){fail("NAVIGATION_ACK_EXPIRED:"+v.consumer_id);return;}
    if(old!=acks.end()&&source<stamp(old->second.value.header.stamp))return;
    if(nav_sent&&!v.applied){fail("NAVIGATION_ACK_REVOKED:"+v.consumer_id);return;}
    if(old!=acks.end()&&source==stamp(old->second.value.header.stamp)) {
      if(v!=old->second.value)fail("NAVIGATION_ACK_CONFLICT:"+v.consumer_id);
      return;
    }
    acks[v.consumer_id]={v,at,source+500*MS};
  }
  void receive(const Command& v) {
    if(!active())return;
    const auto at=now();if(!clock(at))return;
    if(!std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.angular.z)){fail("NAVIGATION_COMMAND_INVALID");return;}
    if(!stopping&&nav_sent&&command&&at.steady-command->second.steady>=300*MS){fail("NAVIGATION_COMMAND_STALE");return;}
    command=std::make_pair(v,at);
  }
  void receive(const Odom& v) {
    if(!active())return;
    const auto at=now();if(!clock(at))return;
    const auto& velocity=v.twist.twist;const auto source=stamp(v.header.stamp);
    if(v.header.frame_id!="odom"||v.child_frame_id!=request.robot_base_frame||!pose_valid(v.pose.pose)||!std::isfinite(velocity.linear.x)||
      !std::isfinite(velocity.linear.y)||!std::isfinite(velocity.angular.z)||source<=0||source>at.ros||v.header.stamp.nanosec>=1000000000u){fail("NAVIGATION_ODOM_INVALID");return;}
    if(!odom_child.empty()&&odom_child!=v.child_frame_id){fail("NAVIGATION_ODOM_FRAME_CHANGED");return;}
    if(!stopping&&nav_sent&&odom&&!fresh(*odom,at)){fail("NAVIGATION_ODOM_STALE");return;}
    if(odom&&source<=stamp(odom->value.header.stamp)) {
      if(source==stamp(odom->value.header.stamp)&&v!=odom->value)fail("NAVIGATION_ODOM_CONFLICT");
      else if(source<stamp(odom->value.header.stamp))fail("NAVIGATION_ODOM_ROLLBACK");
      return;
    }
    odom_child=v.child_frame_id;odom=Received<Odom>{v,at,source+300*MS};
    if(!command)return;
    const auto& p=v.pose.pose;
    samples.push_back({source,at,command->second.steady,p.position.x,p.position.y,yaw(p.orientation),
      std::hypot(velocity.linear.x,velocity.linear.y),std::abs(velocity.angular.z),command_max(command->first)});
    while(!samples.empty()&&samples.front().source<source-700*MS-10)samples.pop_front();
  }
  bool stopped(Time at)const {
    // Mainline tightens the source-only M2 window to require the same minimum
    // receipt-steady span. A burst of queued source samples cannot prove a stop.
    if(samples.size()<12||samples.front().source<=stop_after||samples.back().source-samples.front().source<600*MS||
      samples.back().at.steady-samples.front().at.steady<600*MS||
      !odom||!fresh(*odom,at)||!command||at.steady-command->second.steady>=300*MS||command_max(command->first)>1e-6)return false;
    const auto& first=samples.front();double angle=0,mean=0;int64_t last=first.source;double last_angle=first.angle;
    for(auto it=samples.begin();it!=samples.end();++it) {
      const auto& s=*it;
      if(it!=samples.begin()&&(s.source<=last||s.source-last>120*MS))return false;
      if(s.at.steady<s.command_at||s.at.steady-s.command_at>=300*MS||s.v>.01||s.w>.02||s.command>1e-6||std::hypot(s.x-first.x,s.y-first.y)>.005)return false;
      if(it!=samples.begin())angle+=std::remainder(s.angle-last_angle,2*std::acos(-1.));
      if(std::abs(angle)>.01)return false;
      mean+=double(s.source-first.source)*1e-9;last=s.source;last_angle=s.angle;
    }
    mean/=double(samples.size());double variance=0,x=0,y=0;
    for(const auto& s:samples) {
      const double d=double(s.source-first.source)*1e-9-mean;
      variance+=d*d;x+=d*(s.x-first.x);y+=d*(s.y-first.y);
    }
    return std::hypot(x/variance,y/variance)<=.01;
  }
  bool ready(Time at)const {
    if(!envelope||!fresh(*envelope,at)||!binding(envelope->value)||!envelope->value.navigation_allowed||!envelope->value.limits.transport_ready)return false;
    for(const auto& consumer:CONSUMERS) {
      auto found=acks.find(consumer);if(found==acks.end())return false;
      const auto& a=found->second;
      if(!a.value.applied||!fresh(a,at)||a.value.coordinator_session_id!=envelope->value.coordinator_session_id||
        a.value.envelope_epoch!=result.envelope_epoch||a.value.installed_geometry_hash!=envelope->value.installed_geometry_hash)return false;
    }
    return true;
  }
  void start(Request value) {
    require(result.phase==Phase::IDLE,"NAVIGATION_HELPER_SINGLE_LEG_ONLY");
    require(!value.task_id.empty()&&!value.context_id.empty()&&!value.lease_id.empty()&&!value.resource_epoch.empty(),"NAVIGATION_OWNER_BINDING_REQUIRED");
    const auto& g=value.reference_geometry;
    require(g.complete&&g.attachment_state_confirmed&&g.attachment_ids.size()==1&&!g.attachment_ids.front().empty()&&
      !g.source_id.empty()&&!g.model_revision.empty()&&!g.attachment_revision.empty()&&g.sequence==value.fixed.geometry_sequence&&
      !value.fixed.request_id.empty()&&!value.fixed.hold_id.empty(),"NAVIGATION_PICK_BINDING_REQUIRED");
    require(value.navigation_target.header.frame_id=="map"&&!value.robot_base_frame.empty()&&pose_valid(value.navigation_target.pose)&&
      std::isfinite(value.position_tolerance_m)&&value.position_tolerance_m>0&&
      std::isfinite(value.yaw_tolerance_rad)&&value.yaw_tolerance_rad>0,"NAVIGATION_STATION_TARGET_INVALID");
    require(fixed->service_is_ready()&&nav->action_server_is_ready(),"NAVIGATION_ENDPOINT_UNAVAILABLE");
    request=std::move(value);const auto at=now();phase_started=at.steady;stop_after=at.ros;previous=at;
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
        else {result.phase=Phase::WAIT_ACK;result.reason="WAITING_FOR_SIX_ACK_AND_STOP";phase_started=at.steady;}
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
        else {result.phase=Phase::SETTLING;result.reason="WAITING_FOR_ARRIVAL_STOP";phase_started=at.steady;stop_after=at.ros;samples.clear();}
      } else {stop_after=at.ros;samples.clear();}
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
      if(result.phase==Phase::WAIT_ACK) {
        if(at.steady-phase_started>=15000*MS){fail("SIX_ACK_OR_STOP_TIMEOUT");return;}
        if(!ready(at)||!stopped(at))return;
        result.envelope_session=envelope->value.coordinator_session_id;result.geometry_hash=envelope->value.installed_geometry_hash;
        Nav::Goal goal;goal.pose=request.navigation_target;goal.pose.header.stamp=node.now();
        result.phase=Phase::NAVIGATING;result.reason="NAVIGATING";phase_started=at.steady;
        nav_sent=true;result.nav_terminal=false;goal_future=nav->async_send_goal(goal);return;
      }
      if(result.phase==Phase::NAVIGATING||result.phase==Phase::SETTLING) {
        if(!ready(at)){fail("NAVIGATION_ENVELOPE_OR_ACK_LOST");return;}
        if(!odom||!fresh(*odom,at)||!command||at.steady-command->second.steady>=300*MS){fail("NAVIGATION_MOTION_OBSERVATION_STALE");return;}
        if(result.phase==Phase::NAVIGATING) {
          if(!handle&&at.steady-phase_started>=5000*MS)fail("NAVIGATION_ACCEPTANCE_TIMEOUT");
          else if(at.steady-phase_started>=180000*MS)fail("NAVIGATION_RESULT_TIMEOUT");
          return;
        }
        result.measured_stopped=stopped(at);
        if(!result.measured_stopped){if(at.steady-phase_started>=15000*MS)fail("NAVIGATION_ARRIVAL_STOP_TIMEOUT");return;}
        const tf2::TimePoint capture{std::chrono::nanoseconds(stamp(odom->value.header.stamp))};
        if(!tf.canTransform("map","odom",capture)) {
          result.reason="WAITING_FOR_ARRIVAL_CAPTURE_TF";
          if(at.steady-phase_started>=15000*MS)fail("NAVIGATION_ARRIVAL_TF_UNAVAILABLE");
          return;
        }
        const auto transform=tf.lookupTransform("map","odom",capture);
        geometry_msgs::msg::PoseStamped local,global;local.header=odom->value.header;local.pose=odom->value.pose.pose;
        tf2::doTransform(local,global,transform);
        if(!pose_valid(global.pose)){fail("NAVIGATION_ARRIVAL_TF_INVALID");return;}
        const auto& actual=global.pose;const auto& target=request.navigation_target.pose;
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
