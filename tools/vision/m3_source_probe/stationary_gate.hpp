#pragma once
// Probe-private observation gate. It owns no ROS node, lease or control client.
#include <astribot_navigation_msgs/msg/arm_hold_status.hpp>
#include <astribot_navigation_msgs/msg/envelope_apply_status.hpp>
#include <astribot_navigation_msgs/msg/navigation_envelope_v2.hpp>
#include <astribot_navigation_msgs/msg/navigation_execution_status.hpp>
#include <astribot_navigation_msgs/msg/robot_geometry_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <utility>

namespace m3_probe {
using Hold=astribot_navigation_msgs::msg::ArmHoldStatus;
using Ack=astribot_navigation_msgs::msg::EnvelopeApplyStatus;
using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
using Geometry=astribot_navigation_msgs::msg::RobotGeometryState;
using Navigation=astribot_navigation_msgs::msg::NavigationExecutionStatus;
using Odom=nav_msgs::msg::Odometry;
using Command=geometry_msgs::msg::Twist;
constexpr int64_t ms=1000000;
inline int64_t stamp(const builtin_interfaces::msg::Time& t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
struct Time {int64_t ros{},steady{};};
template<class T>struct Received {T value;Time at;};
inline bool fresh(int64_t source,int64_t until,Time received,Time now) {
  return source>0&&source<=received.ros&&received.ros<=now.ros&&now.ros<until&&
    received.steady<=now.steady&&now.steady-received.steady<until-received.ros;
}
inline bool valid_stamp(const builtin_interfaces::msg::Time& value) {return value.sec>=0&&value.nanosec<1000000000u;}
inline bool pose_valid(const geometry_msgs::msg::Pose& p) {
  const auto& q=p.orientation;
  return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&std::isfinite(p.position.z)&&
    std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z)&&std::isfinite(q.w)&&
    std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<=.001;
}
inline double command_max(const Command& v) {
  return std::max({std::abs(v.linear.x),std::abs(v.linear.y),std::abs(v.angular.z)});
}
struct StopSample {
  int64_t source{};Time received,command_received;
  double x{},y{},yaw{},vx{},vy{},wz{},command{};
};
struct StopMetrics {
  bool passed{};size_t samples{};int64_t span{};
  double drift{},rotation{},speed{},angular_speed{},command{},fitted_speed{};
  std::string reason;
};
// Exact M2 measured_stop thresholds; 10 ns is only its tail-selection slack.
inline StopMetrics measured_stop(const std::deque<StopSample>& rows,int64_t after,Time now) {
  StopMetrics result;auto fail=[&](const char* why){result.reason=why;return result;};
  if(rows.empty())return fail("STOP_WINDOW_INCOMPLETE");
  auto first=std::find_if(rows.begin(),rows.end(),[&](const auto& s){return s.source>=rows.back().source-700*ms-10;});
  result.samples=size_t(std::distance(first,rows.end()));
  if(result.samples<12)return fail("STOP_WINDOW_INCOMPLETE");
  const auto& start=*first;const auto& last=rows.back();result.span=last.source-start.source;
  if(result.span<600*ms||start.source<=after)return fail("STOP_WINDOW_INCOMPLETE");
  if(!fresh(last.source,last.source+300*ms,last.received,now))return fail("ODOM_STALE");
  double angle=0,mean=0;int64_t previous=start.source;double previous_yaw=start.yaw;
  for(auto it=first;it!=rows.end();++it) {
    const auto& s=*it;
    for(double v:{s.x,s.y,s.yaw,s.vx,s.vy,s.wz,s.command})if(!std::isfinite(v))return fail("STOP_SAMPLE_INVALID");
    if(it!=first&&(s.source<=previous||s.source-previous>120*ms))return fail("ODOM_SEQUENCE_GAP");
    if(s.received.steady<s.command_received.steady||s.received.steady-s.command_received.steady>=300*ms)return fail("COMMAND_STALE_AT_ODOM");
    if(it!=first)angle+=std::remainder(s.yaw-previous_yaw,2*std::acos(-1.));
    previous=s.source;previous_yaw=s.yaw;
    result.rotation=std::max(result.rotation,std::abs(angle));
    result.drift=std::max(result.drift,std::hypot(s.x-start.x,s.y-start.y));
    result.speed=std::max(result.speed,std::hypot(s.vx,s.vy));
    result.angular_speed=std::max(result.angular_speed,std::abs(s.wz));result.command=std::max(result.command,s.command);
    mean+=double(s.source-start.source)*1e-9;
  }
  mean/=double(result.samples);double variance=0,x=0,y=0;
  for(auto it=first;it!=rows.end();++it) {
    const double delta=double(it->source-start.source)*1e-9-mean;
    variance+=delta*delta;x+=delta*(it->x-start.x);y+=delta*(it->y-start.y);
  }
  result.fitted_speed=std::hypot(x/variance,y/variance);
  result.passed=result.speed<=.01&&result.angular_speed<=.02&&result.command<=1e-6&&
    result.drift<=.005&&result.rotation<=.01&&result.fitted_speed<=.01;
  if(!result.passed)result.reason="BASE_NOT_STATIONARY";
  return result;
}

class StationaryGate {
public:
  inline static const std::array<std::string,6> consumers={"global_costmap","local_costmap","planner","controller","policy","protection"};
  StationaryGate(std::string owner,std::string hold,std::string coordinator)
    :owner_(std::move(owner)),hold_id_(std::move(hold)),coordinator_(std::move(coordinator)){}
  const std::string& fault()const{return fault_;}
  bool armed()const{return reference_.has_value();}
  const StopMetrics& metrics()const{return metrics_;}
  const std::optional<Received<Hold>>& hold()const{return hold_;}
  const std::optional<Received<Odom>>& odom()const{return odom_;}
  const std::optional<Received<Command>>& command()const{return command_;}
  const std::optional<Received<Geometry>>& geometry()const{return geometry_;}
  const std::optional<Received<Envelope>>& envelope()const{return envelope_;}
  int64_t envelope_deadline()const{return envelope_deadline_;}
  const std::map<std::string,Received<Ack>>& acks()const{return acks_;}
  const std::map<std::string,Navigation>& navigation()const{return navigation_;}
  const std::optional<Odom>& reference()const{return reference_;}
  int64_t armed_ros()const{return armed_ros_;}

  void receive(const Hold& value,Time at) {
    if(!clock(at))return;
    if(armed()&&!fresh(stamp(hold_->value.header.stamp),stamp(hold_->value.header.stamp)+int64_t(std::nearbyint(hold_->value.lease_s*1e9)),hold_->at,at)) {fail("TYPED_HOLD_EXPIRED");return;}
    if(!valid_stamp(value.header.stamp)||!std::isfinite(value.lease_s)||value.lease_s<0||value.lease_s>.5){fail("HOLD_MESSAGE_INVALID");return;}
    if(!remember(hold_,value,at,"HOLD_SOURCE_CONFLICT"))return;
    if(armed()&&(!value.hold_confirmed||value.owner_id!=owner_||value.hold_id!=hold_id_||value.attachment_revision!=binding_->attachment_revision))fail("OWNER_TYPED_HOLD_CHANGED");
  }
  void receive(const Ack& value,Time at) {
    if(!clock(at))return;
    if(value.coordinator_session_id!=coordinator_||std::find(consumers.begin(),consumers.end(),value.consumer_id)==consumers.end())return;
    if(binding_&&(value.envelope_epoch!=binding_->epoch||value.installed_geometry_hash!=binding_->installed_geometry_hash))return;
    if(!valid_stamp(value.header.stamp)||stamp(value.header.stamp)<=0||stamp(value.header.stamp)>at.ros){fail("ACK_SOURCE_INVALID");return;}
    if(armed()&&!value.applied)fail("CURRENT_ACK_REVOKED");
    auto it=acks_.find(value.consumer_id);
    if(armed()&&it!=acks_.end()&&!fresh(stamp(it->second.value.header.stamp),stamp(it->second.value.header.stamp)+500*ms,it->second.at,at)) {fail("ACK_INVALID_OR_EXPIRED:"+value.consumer_id);return;}
    if(it!=acks_.end()&&value.envelope_epoch==it->second.value.envelope_epoch&&value.installed_geometry_hash==it->second.value.installed_geometry_hash) {
      if(value.header.stamp==it->second.value.header.stamp){if(value!=it->second.value)fail("ACK_SOURCE_CONFLICT");return;}
      if(stamp(value.header.stamp)<stamp(it->second.value.header.stamp)) {
        if(armed())fail("ACK_SOURCE_ROLLBACK");
        return;
      }
    }
    acks_[value.consumer_id]={value,at};
  }
  void receive(const Command& value,Time at) {
    if(!clock(at))return;
    if(armed()&&at.steady-command_->at.steady>=300*ms){fail("COMMAND_STALE");return;}
    if(!std::isfinite(value.linear.x)||!std::isfinite(value.linear.y)||!std::isfinite(value.angular.z)){fail("COMMAND_INVALID");return;}
    command_=Received<Command>{value,at}; // Unstamped genuine zero heartbeats may repeat.
    if(armed()&&command_max(value)>1e-6)fail("NONZERO_CHASSIS_COMMAND");
  }
  void receive(const Odom& value,Time at) {
    if(!clock(at))return;
    if(armed()&&!fresh(stamp(odom_->value.header.stamp),stamp(odom_->value.header.stamp)+300*ms,odom_->at,at)){fail("ODOM_STALE");return;}
    const auto& v=value.twist.twist;const auto& p=value.pose.pose;
    if(!valid_stamp(value.header.stamp)||value.header.frame_id!="odom"||value.child_frame_id.empty()||!pose_valid(p)||
      !std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.angular.z)){fail("ODOM_INVALID");return;}
    if(!armed()&&odom_&&value.child_frame_id!=odom_->value.child_frame_id){settle_after_.reset();samples_.clear();}
    if(!remember(odom_,value,at,"ODOM_SOURCE_CONFLICT"))return;
    if(armed()) {
      if(value.child_frame_id!=reference_->child_frame_id){fail("ODOM_FRAME_CHANGED");return;}
      const auto& r=reference_->pose.pose;const auto& q=p.orientation;const auto& b=r.orientation;
      const double dot=std::abs(q.x*b.x+q.y*b.y+q.z*b.z+q.w*b.w);
      if(std::hypot(std::hypot(p.position.x-r.position.x,p.position.y-r.position.y),p.position.z-r.position.z)>.02||
        2*std::acos(std::min(1.,dot))>.02){fail("BASE_REFERENCE_MOVED");return;}
      if(std::hypot(v.linear.x,v.linear.y)>.01||std::abs(v.angular.z)>.02){fail("BASE_NOT_STATIONARY");return;}
    }
    if(!command_)return;
    const auto& q=p.orientation;
    samples_.push_back({stamp(value.header.stamp),at,command_->at,p.position.x,p.position.y,
      std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),v.linear.x,v.linear.y,v.angular.z,command_max(command_->value)});
    while(!samples_.empty()&&samples_.front().source<samples_.back().source-700*ms-10)samples_.pop_front();
    if(armed()){metrics_=measured_stop(samples_,*settle_after_,at);if(!metrics_.passed)fail(metrics_.reason);}
  }
  void receive(const Navigation& value,Time at) {
    if(!clock(at))return;
    if(!valid_stamp(value.stamp)||stamp(value.stamp)>at.ros||stamp(value.stamp)<=0||value.task_id.empty()||value.sequence==0){fail("NAVIGATION_EVENT_INVALID");return;}
    const bool active=value.state=="ACCEPTED"||value.state=="EXECUTING"||value.state=="CANCELING";
    const bool terminal=value.state=="SUCCEEDED"||value.state=="CANCELED"||value.state=="PREEMPTED"||value.state=="FAILED";
    if(!active&&!terminal){fail("NAVIGATION_EVENT_INVALID");return;}
    auto found=navigation_.find(value.task_id);
    if(found!=navigation_.end()&&value.sequence<=found->second.sequence) {
      if(value.sequence==found->second.sequence&&value!=found->second)fail("NAVIGATION_EVENT_CONFLICT");
      return;
    }
    navigation_[value.task_id]=value;
    if(armed()&&active)fail("NAVIGATION_TASK_ACTIVE");
  }
  // Called from the existing callbacks so transient context loss is not hidden
  // by a later healthy message before the main loop polls.
  void receive(const Envelope& value,Time at) {
    if(!clock(at))return;
    if(armed()&&!fresh(stamp(envelope_->value.header.stamp),envelope_deadline_,envelope_->at,at)) {
      fail("OWNER_FIXED_ENVELOPE_EXPIRED");return;
    }
    if(!valid_stamp(value.header.stamp)||!valid_stamp(value.valid_until)){fail("ENVELOPE_SOURCE_INVALID");return;}
    const auto source=stamp(value.header.stamp);
    if(source<=0||source>at.ros){fail("STATIONARY_SOURCE_INVALID");return;}
    const auto deadline=std::min(stamp(value.valid_until),source+300*ms);
    if(envelope_&&source<stamp(envelope_->value.header.stamp)) {
      if(armed())fail("STATIONARY_SOURCE_ROLLBACK");
      return;
    }
    if(envelope_&&source==stamp(envelope_->value.header.stamp)) {
      // The coordinator can publish several ACK-driven state transitions at
      // one simulation stamp. Preserve the original receipt and expiry.
      envelope_->value=value;envelope_deadline_=std::min(envelope_deadline_,deadline);
    } else {envelope_=Received<Envelope>{value,at};envelope_deadline_=deadline;}
    if(armed()&&(!same_envelope(value,*binding_)||value.mode!=Envelope::FIXED_POSTURE||
      !value.navigation_allowed||!value.limits.transport_ready||
      !fresh(stamp(value.header.stamp),std::min(stamp(value.valid_until),stamp(value.header.stamp)+300*ms),at,at)))fail("OWNER_FIXED_ENVELOPE_CHANGED");
  }
  void receive(const Geometry& value,Time at) {
    if(!clock(at))return;
    if(armed()&&!fresh(stamp(geometry_->value.header.stamp),std::min(stamp(geometry_->value.valid_until),stamp(geometry_->value.header.stamp)+300*ms),geometry_->at,at)) {
      fail("OWNER_GEOMETRY_EXPIRED");return;
    }
    if(!valid_stamp(value.header.stamp)||!valid_stamp(value.valid_until)){fail("GEOMETRY_SOURCE_INVALID");return;}
    if(!remember(geometry_,value,at,"GEOMETRY_SOURCE_CONFLICT"))return;
    if(armed()&&(!value.complete||!value.attachment_state_confirmed||!value.attachment_ids.empty()||
      value.source_id!=geometry_source_||value.clock_epoch!=binding_->clock_epoch||value.model_revision!=binding_->model_revision||
      value.attachment_revision!=binding_->attachment_revision||
      !fresh(stamp(value.header.stamp),std::min(stamp(value.valid_until),stamp(value.header.stamp)+300*ms),at,at)))fail("OWNER_GEOMETRY_CHANGED");
  }
  std::string check(const Envelope& envelope,const Geometry& geometry,Time now) {
    if(!clock(now))return fault_;
    if(!fault_.empty())return fault_;
    auto why=prerequisites(envelope,geometry,now);
    if(!why.empty()) {
      if(armed())fail(why);else{settle_after_.reset();samples_.clear();}
      return why;
    }
    if(!settle_after_){settle_after_=now.ros;samples_.clear();}
    metrics_=measured_stop(samples_,*settle_after_,now);
    if(!metrics_.passed&&armed())fail(metrics_.reason);
    return metrics_.reason;
  }
  std::string arm(const Envelope& envelope,const Geometry& geometry,Time now) {
    const auto why=check(envelope,geometry,now);if(!why.empty())return why;
    reference_=odom_->value;binding_=envelope;geometry_source_=geometry.source_id;armed_ros_=now.ros;
    return {};
  }
private:
  void fail(const std::string& why){if(fault_.empty())fault_=why;}
  bool clock(Time at) {
    if(previous_&&(at.ros<previous_->ros||at.steady<previous_->steady)){fail("STATIONARY_CLOCK_ROLLBACK");return false;}
    previous_=at;return fault_.empty();
  }
  template<class T>bool remember(std::optional<Received<T>>& last,const T& value,Time at,const char* conflict) {
    const auto source=stamp(value.header.stamp);
    if(source<=0||source>at.ros){fail("STATIONARY_SOURCE_INVALID");return false;}
    if(last) {
      const auto previous=stamp(last->value.header.stamp);
      if(source==previous){if(value!=last->value)fail(conflict);return false;}
      if(source<previous){if(armed())fail("STATIONARY_SOURCE_ROLLBACK");return false;}
    }
    last=Received<T>{value,at};return true;
  }
  static bool same_envelope(const Envelope& a,const Envelope& b) {
    return a.coordinator_session_id==b.coordinator_session_id&&a.epoch==b.epoch&&a.clock_epoch==b.clock_epoch&&
      a.hold_id==b.hold_id&&a.model_revision==b.model_revision&&a.attachment_revision==b.attachment_revision&&
      a.installed_geometry_hash==b.installed_geometry_hash&&a.request_id==b.request_id;
  }
  std::string prerequisites(const Envelope& e,const Geometry& g,Time now)const {
    if(!g.complete||!g.attachment_state_confirmed||!g.attachment_ids.empty()||g.source_id.empty()||g.model_revision.empty()||g.attachment_revision.empty())return "OWNER_GEOMETRY_NOT_READY";
    if(!fresh(stamp(g.header.stamp),std::min(stamp(g.valid_until),stamp(g.header.stamp)+300*ms),geometry_->at,now))return "OWNER_GEOMETRY_EXPIRED";
    if(e.coordinator_session_id!=coordinator_||e.hold_id!=hold_id_||e.epoch==0||e.installed_geometry_hash.empty()||
      e.mode!=Envelope::FIXED_POSTURE||!e.navigation_allowed||!e.limits.transport_ready||
      e.clock_epoch!=g.clock_epoch||e.model_revision!=g.model_revision||e.attachment_revision!=g.attachment_revision||
      !valid_stamp(e.header.stamp)||!valid_stamp(e.valid_until)||stamp(e.header.stamp)>now.ros||now.ros>=stamp(e.valid_until))return "FIXED_ENVELOPE_NOT_READY";
    if(!fresh(stamp(e.header.stamp),envelope_deadline_,envelope_->at,now))return "OWNER_FIXED_ENVELOPE_EXPIRED";
    if(binding_&&!same_envelope(e,*binding_))return "OWNER_FIXED_ENVELOPE_CHANGED";
    if(!hold_)return "TYPED_HOLD_MISSING";
    const auto& h=hold_->value;
    if(!h.hold_confirmed||h.owner_id!=owner_||h.hold_id!=hold_id_||h.attachment_revision!=g.attachment_revision)return "TYPED_HOLD_MISMATCH";
    const auto until=stamp(h.header.stamp)+int64_t(std::nearbyint(h.lease_s*1e9));
    if(!fresh(stamp(h.header.stamp),until,hold_->at,now))return "TYPED_HOLD_EXPIRED";
    for(const auto& consumer:consumers) {
      auto found=acks_.find(consumer);if(found==acks_.end())return "ACK_MISSING:"+consumer;
      const auto& a=found->second.value;
      if(!a.applied||a.coordinator_session_id!=e.coordinator_session_id||a.envelope_epoch!=e.epoch||
        a.installed_geometry_hash!=e.installed_geometry_hash||!fresh(stamp(a.header.stamp),stamp(a.header.stamp)+500*ms,found->second.at,now))return "ACK_INVALID_OR_EXPIRED:"+consumer;
    }
    for(const auto& task:navigation_)if(task.second.state=="ACCEPTED"||task.second.state=="EXECUTING"||task.second.state=="CANCELING")return "NAVIGATION_TASK_ACTIVE";
    if(!odom_||!fresh(stamp(odom_->value.header.stamp),stamp(odom_->value.header.stamp)+300*ms,odom_->at,now))return "ODOM_STALE";
    if(!command_||command_->at.steady>now.steady||now.steady-command_->at.steady>=300*ms)return "COMMAND_STALE";
    if(command_max(command_->value)>1e-6)return "NONZERO_CHASSIS_COMMAND";
    return {};
  }
  std::string owner_,hold_id_,coordinator_,geometry_source_,fault_;
  std::optional<Time> previous_;
  std::optional<int64_t> settle_after_;
  std::optional<Received<Hold>> hold_;
  std::optional<Received<Odom>> odom_;
  std::optional<Received<Command>> command_;
  std::optional<Received<Geometry>> geometry_;
  std::optional<Received<Envelope>> envelope_;
  int64_t envelope_deadline_{};
  std::map<std::string,Received<Ack>> acks_;
  std::map<std::string,Navigation> navigation_;
  std::optional<Odom> reference_;
  std::optional<Envelope> binding_;
  int64_t armed_ros_{};
  std::deque<StopSample> samples_;
  StopMetrics metrics_;
};
} // namespace m3_probe
