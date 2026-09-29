#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <limits>
#include <random>
#include <optional>
#include "astribot_s1_perception_components/vision_validation.hpp"
#include <memory>
#include <string>

#include "astribot_perception_msgs/msg/camera_health.hpp"
#include "astribot_perception_msgs/msg/camera_session_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_health_detail {

struct StreamState {
  bool seen{false};
  std::chrono::steady_clock::time_point received;
  rclcpp::Time stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
  rclcpp::Time previous_stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
  std::string frame_id;
  uint32_t width{0};
  uint32_t height{0};
  double last_interval_sec{0.0};
  double max_interval_sec{0.0};
  uint32_t drops{0};
};

struct MatchedSnapshot {
  StreamState color, depth, info;
  rclcpp::Time capture{static_cast<int64_t>(0), RCL_ROS_TIME};
  rclcpp::Time newest{static_cast<int64_t>(0), RCL_ROS_TIME};
  double last_interval_sec{0.0};
};

double seconds_between(const rclcpp::Time & newer, const rclcpp::Time & older)
{
  return static_cast<double>((newer - older).nanoseconds()) * 1e-9;
}

builtin_interfaces::msg::Time time_message(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time message;
  const int64_t nanoseconds = time.nanoseconds();
  message.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  message.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return message;
}

}  // namespace camera_health_detail
using namespace camera_health_detail;

class CameraHealthNode final : public rclcpp::Node {
public:
  explicit CameraHealthNode(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
  : Node("camera_health", options)
  {
    camera_id_ = declare_parameter<std::string>("camera_id", "camera");
    source_epoch_ = declare_parameter<std::string>("source_epoch", "sim");
    source_epoch_ += ":" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
      ":" + std::to_string(std::random_device{}());
    source_epoch_base_=source_epoch_;
    expected_frame_ = declare_parameter<std::string>("frame_id", "");
    color_topic_ = declare_parameter<std::string>("color_topic", "");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "");
    info_topic_ = declare_parameter<std::string>("info_topic", "");
    health_topic_ = declare_parameter<std::string>(
      "health_topic", "/perception/camera_health");
    expected_rate_hz_ = declare_parameter<double>("expected_rate_hz", 10.0);
    max_age_sec_ = declare_parameter<double>("max_age_sec", 0.25);
    max_sync_skew_sec_ = declare_parameter<double>("max_sync_skew_sec", 0.03);
    rcl_interfaces::msg::ParameterDescriptor calibration_descriptor;
    calibration_descriptor.read_only=true;
    calibration_descriptor.description="Calibration revision is pinned until restart.";
    calibration_revision_ = declare_parameter<int64_t>("calibration_revision", 0, calibration_descriptor);
    const double timer_period = declare_parameter<double>("timer_period_sec", 0.05);

    if (color_topic_.empty() || depth_topic_.empty() || info_topic_.empty()) {
      throw std::invalid_argument("color_topic, depth_topic and info_topic are required");
    }
    if (camera_id_.empty() || !std::isfinite(expected_rate_hz_) || !(expected_rate_hz_ > 0.0) ||
        !std::isfinite(max_age_sec_) || !(max_age_sec_ > 0.0) ||
        !std::isfinite(max_sync_skew_sec_) || !(max_sync_skew_sec_ >= 0.0) || calibration_revision_ < 0 ||
        !std::isfinite(timer_period) || timer_period<.001 || timer_period>1.0) {
      throw std::invalid_argument("invalid camera health thresholds");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      color_topic_, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { update_image(color_, *msg); });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { update_image(depth_, *msg); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic_, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { update_info(*msg); });

    health_pub_ = create_publisher<astribot_perception_msgs::msg::CameraHealth>(
      health_topic_, rclcpp::QoS(10).reliable().transient_local());
    activation_topic_=declare_parameter<std::string>("activation_topic", "");
    if(!activation_topic_.empty()) {
      activation_sub_=create_subscription<astribot_perception_msgs::msg::CameraSessionState>(
        activation_topic_,rclcpp::QoS(1).reliable().transient_local(),
        [this](astribot_perception_msgs::msg::CameraSessionState::ConstSharedPtr state){
          if(state->camera_id!=camera_id_)return;
          const auto received=std::chrono::steady_clock::now();
          if(activation_ && state->session_token==activation_->session_token &&
             astribot::vision::ns(state->header.stamp)>0 &&
             astribot::vision::ns(state->header.stamp)<astribot::vision::ns(activation_->header.stamp))return;
          const bool context_changed=!activation_ || state->session_token!=activation_->session_token ||
            state->active!=activation_->active || state->owner_id!=activation_->owner_id ||
            state->execution_id!=activation_->execution_id || state->activated_at!=activation_->activated_at;
          activation_=state;activation_received_=received;
          if(context_changed){
            color_=StreamState{};depth_=StreamState{};info_=StreamState{};clear_matches();matched_max_interval_sec_=0.;
            break_continuity(true);
          }
          publish_health();
        });
    }
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(timer_period)),
      [this]() { publish_health(); });
  }

private:
  void clear_pending()
  {
    color_history_.clear();depth_history_.clear();info_history_.clear();
  }

  void clear_matches()
  {
    clear_pending();
    matched_.reset();
  }

  void break_continuity(bool new_context=false)
  {
    // One generation per continuous fault episode. A clock/session identity
    // change is a new context even while unhealthy. Never append to an old ID.
    if(!continuity_broken_ || new_context) {
      source_epoch_=source_epoch_base_+":g"+std::to_string(++generation_);
      if(activation_) source_epoch_+=":session:"+activation_->session_token+
        (activation_->active?":active":":inactive");
      clear_pending();
      recovery_capture_after_=matched_?matched_->capture:rclcpp::Time(int64_t(0),RCL_ROS_TIME);
    }
    continuity_broken_=true;
    recovery_required_=true;
    recovery_after_match_=matched_sequence_;
  }

  void publish_result(astribot_perception_msgs::msg::CameraHealth message)
  {
    if(message.valid && recovery_required_ &&
       (!matched_ || matched_sequence_<=recovery_after_match_)) {
      message.valid=false;message.state="RECOVERING";
      message.reason_code="CAMERA_RECOVERY_REQUIRES_NEW_FRAME";
    }
    if(message.valid) {
      continuity_broken_=false;recovery_required_=false;
    } else {
      break_continuity();
    }
    // Retain capture/expiry of the observed triple for fault diagnostics; only
    // its authorization and epoch change. Old observations fail equality gates.
    message.source_epoch=source_epoch_;
    health_pub_->publish(message);
  }

  void check_stream_geometry()
  {
    if(color_.seen && depth_.seen && info_.seen &&
       (color_.frame_id!=depth_.frame_id || color_.frame_id!=info_.frame_id ||
        (!expected_frame_.empty() && color_.frame_id!=expected_frame_) ||
        color_.width!=depth_.width || color_.height!=depth_.height ||
        color_.width!=info_.width || color_.height!=info_.height)) break_continuity();
  }

  void match_pending()
  {
    if(!(color_.seen && depth_.seen && info_.seen) || calibration_changed_ ||
       color_history_.empty() || depth_history_.empty() || info_history_.empty())return;
    const auto &color=color_history_.back();
    if(matched_ && color.stamp<matched_->color.stamp)return;
    const auto nearest=[&](const auto &history) {
      return std::min_element(history.begin(),history.end(),[&](const auto &a,const auto &b) {
        return std::abs(seconds_between(a.stamp,color.stamp))<std::abs(seconds_between(b.stamp,color.stamp));
      });
    };
    const auto depth=nearest(depth_history_),info=nearest(info_history_);
    if(color.frame_id!=depth->frame_id || color.frame_id!=info->frame_id ||
       (!expected_frame_.empty() && color.frame_id!=expected_frame_) ||
       color.width!=depth->width || color.height!=depth->height ||
       color.width!=info->width || color.height!=info->height)return;
    const auto capture=std::min(color.stamp,depth->stamp);
    const auto newest=std::max(color.stamp,depth->stamp);
    if(matched_ && color.stamp==matched_->color.stamp && depth->stamp==matched_->depth.stamp && info->stamp==matched_->info.stamp)return;
    const double interval=matched_?seconds_between(color.stamp,matched_->color.stamp):0.;
    matched_=MatchedSnapshot{color,*depth,*info,capture,newest,interval};
    matched_max_interval_sec_=std::max(matched_max_interval_sec_,interval);++matched_sequence_;
  }

  void remember(const StreamState & state,std::deque<StreamState> & history)
  {
    // Metadata only: image payloads are never retained by this monitor.
    constexpr size_t max_pending_samples=32;
    history.push_back(state);
    while(history.size()>max_pending_samples) history.pop_front();
    match_pending();
  }

  void update_image(StreamState & state, const sensor_msgs::msg::Image & msg)
  {
    const auto incoming=astribot::vision::ns(msg.header.stamp);
    if(state.seen && incoming>0 && incoming<=state.stamp.nanoseconds()){++state.drops;return;}
    const bool color=&state==&color_;
    size_t bytes=0;
    if(color) {
      if(msg.encoding=="rgb8" || msg.encoding=="bgr8") bytes=3;
      else if(msg.encoding=="rgba8" || msg.encoding=="bgra8") bytes=4;
      else if(msg.encoding=="mono8") bytes=1;
    } else {
      if(msg.encoding=="32FC1") bytes=4;
      else if(msg.encoding=="16UC1") bytes=2;
    }
    if(!bytes || !astribot::vision::image_layout(msg,bytes) ||
       astribot::vision::ns(msg.header.stamp)==0 || msg.header.frame_id.empty()) {
      break_continuity();state.seen=false;clear_matches();return;
    }
    const rclcpp::Time stamp(msg.header.stamp);
    if (state.seen && stamp <= state.stamp) {
      ++state.drops;
      return;
    }
    if (state.seen) {
      const double interval = seconds_between(stamp, state.stamp);
      if (interval > 0.0 && std::isfinite(interval)) {
        state.last_interval_sec = interval;
        state.max_interval_sec = std::max(state.max_interval_sec, interval);
      }
    }
    state.received=std::chrono::steady_clock::now();
    state.previous_stamp = state.stamp;
    state.stamp = stamp;
    state.frame_id = msg.header.frame_id;
    state.width = msg.width;
    state.height = msg.height;
    state.seen = true;
    check_stream_geometry();
    remember(state,color?color_history_:depth_history_);
  }

  void update_info(const sensor_msgs::msg::CameraInfo & msg)
  {
    const auto incoming=astribot::vision::ns(msg.header.stamp);
    if(info_.seen && incoming>0 && incoming<=info_.stamp.nanoseconds()){++info_.drops;return;}
    if(incoming==0 || msg.header.frame_id.empty() ||
       !msg.width || !msg.height || msg.width>8192 || msg.height>8192 ||
       !std::all_of(msg.k.begin(),msg.k.end(),[](double v){return std::isfinite(v);}) ||
       msg.k[0]<=0 || msg.k[4]<=0 || msg.k[8]!=1 ||
       !std::all_of(msg.d.begin(),msg.d.end(),[](double v){return std::isfinite(v);})) {
      break_continuity();info_.seen=false;clear_matches();return;
    }
    const rclcpp::Time stamp(msg.header.stamp);
    if (info_.seen && stamp <= info_.stamp) {
      ++info_.drops;
      return;
    }
    if (info_.seen) {
      const double interval = seconds_between(stamp, info_.stamp);
      if (interval > 0.0 && std::isfinite(interval)) {
        info_.last_interval_sec = interval;
        info_.max_interval_sec = std::max(info_.max_interval_sec, interval);
      }
    }
    if(baseline_info_ && !astribot::vision::same_calibration(*baseline_info_,msg)) {
      break_continuity();calibration_changed_=true;info_.seen=false;clear_matches();return;
    }
    if(!baseline_info_) baseline_info_=msg;
    info_.received=std::chrono::steady_clock::now();
    info_.stamp = stamp;
    info_.frame_id = msg.header.frame_id;
    info_.width = msg.width;
    info_.height = msg.height;
    info_.seen = true;
    check_stream_geometry();
    remember(info_,info_history_);
  }

  void publish_health()
  {
    using Health = astribot_perception_msgs::msg::CameraHealth;
    Health message;
    const rclcpp::Time now = get_clock()->now();
    message.header.stamp = time_message(now);
    message.header.frame_id = expected_frame_;
    message.camera_id = camera_id_;
    message.source_epoch = source_epoch_;
    message.sequence = ++sequence_;
    message.calibration_revision = static_cast<uint64_t>(calibration_revision_);
    message.valid = false;
    message.state = "NO_DATA";
    message.reason_code = "CAMERA_DATA_UNAVAILABLE";

    if(!activation_topic_.empty()) {
      const auto ns=[](const builtin_interfaces::msg::Time& t)->int64_t{return int64_t(t.sec)*1000000000LL+t.nanosec;};
      const int64_t t=now.nanoseconds();
      const bool current=activation_ && activation_->active && !activation_->owner_id.empty() &&
        !activation_->execution_id.empty() && !activation_->session_token.empty() &&
        activation_->header.stamp.sec>=0 && activation_->header.stamp.nanosec<1000000000u &&
        activation_->activated_at.sec>=0 && activation_->activated_at.nanosec<1000000000u &&
        ns(activation_->activated_at)>0 &&
        activation_->valid_until.sec>=0 && activation_->valid_until.nanosec<1000000000u;
      if(!current){
        if(!was_session_unavailable_) RCLCPP_WARN(get_logger(),
          "CAMERA_SESSION_UNAVAILABLE camera=%s active=%d status_delta_ns=%ld lease_remaining_ns=%ld wall_status_age=%.6f",
          camera_id_.c_str(),activation_ && activation_->active,
          activation_?t-ns(activation_->header.stamp):0,
          activation_?ns(activation_->valid_until)-t:0,
          activation_?std::chrono::duration<double>(std::chrono::steady_clock::now()-activation_received_).count():0.);
        was_session_unavailable_=true;message.state="INACTIVE";message.reason_code="CAMERA_SESSION_UNAVAILABLE";
        publish_result(message);return;
      }
      if(was_session_unavailable_) {
        // Even complete images received while the lease was unavailable must
        // not become executable on a later heartbeat alone.
        break_continuity();clear_pending();
        recovery_capture_after_=matched_?matched_->capture:rclcpp::Time(int64_t(0),RCL_ROS_TIME);
      }
      was_session_unavailable_=false;
    }
    if(calibration_changed_) {
      message.state="CALIBRATION_CHANGED";message.reason_code="CALIBRATION_CHANGED_WITHOUT_REVISION";
      publish_result(message);return;
    }
    if (!(color_.seen && depth_.seen && info_.seen)) {
      publish_result(message);
      return;
    }

    // Freshness and synchronization describe one complete observed triple, not
    // independent latest samples from callbacks that may still be in flight.
    const auto & snapshot_color=matched_?matched_->color:color_;
    const auto & snapshot_depth=matched_?matched_->depth:depth_;
    const auto & snapshot_info=matched_?matched_->info:info_;
    const rclcpp::Time capture = std::min(snapshot_color.stamp, snapshot_depth.stamp);
    if(!activation_topic_.empty() && capture<rclcpp::Time(activation_->activated_at)) {
      message.state="SESSION_FRAME_OLD";message.reason_code="CAMERA_FRAME_BEFORE_SESSION";
      publish_result(message);return;
    }
    const rclcpp::Time newest = std::max(snapshot_color.stamp, snapshot_depth.stamp);
    const double skew = seconds_between(newest, capture);
    const double age = seconds_between(now, capture);
    message.capture_stamp = time_message(capture);
    auto expires=capture+rclcpp::Duration::from_seconds(max_age_sec_);
    if(!activation_topic_.empty()) expires=std::min(expires,rclcpp::Time(activation_->valid_until));
    message.valid_until = time_message(expires);
    message.frame_id = snapshot_color.frame_id;
    message.header.frame_id = snapshot_color.frame_id;
    message.age_sec = age;
    message.sync_skew_sec = skew;
    const double interval=std::max({color_.last_interval_sec,depth_.last_interval_sec,info_.last_interval_sec,
      matched_?matched_->last_interval_sec:0.0});
    message.frequency_hz = interval > 0.0 ? 1.0 / interval : 0.0;
    message.max_interval_sec = std::max({color_.max_interval_sec,depth_.max_interval_sec,info_.max_interval_sec,matched_max_interval_sec_});
    message.drop_count = color_.drops + depth_.drops + info_.drops;

    const bool frame_ok = color_.frame_id == depth_.frame_id &&
      color_.frame_id == info_.frame_id &&
      (expected_frame_.empty() || color_.frame_id == expected_frame_);
    const bool resolution_ok = color_.width == depth_.width &&
      color_.height == depth_.height &&
      info_.width == color_.width && info_.height == color_.height;

    if(calibration_revision_==0) {
      message.state="UNCALIBRATED";message.reason_code="CALIBRATION_REVISION_MISSING";
    } else if (!frame_ok) {
      message.state = "FRAME_MISMATCH";
      message.reason_code = "CAMERA_FRAME_MISMATCH";
    } else if (!resolution_ok) {
      message.state = "RESOLUTION_MISMATCH";
      message.reason_code = "CAMERA_RESOLUTION_MISMATCH";
    } else if (!matched_) {
      message.state = "SYNC_FAILED";
      message.reason_code = "CAMERA_SYNC_SKEW";
    } else {
      message.state = "OK";
      message.reason_code = "CAMERA_READY";
      message.valid = true;
    }
    publish_result(message);
  }

  std::string activation_topic_;
  bool was_session_unavailable_{true};
  astribot_perception_msgs::msg::CameraSessionState::ConstSharedPtr activation_;
  std::chrono::steady_clock::time_point activation_received_;
  rclcpp::Subscription<astribot_perception_msgs::msg::CameraSessionState>::SharedPtr activation_sub_;
  std::string camera_id_, source_epoch_, source_epoch_base_, expected_frame_;
  std::string color_topic_, depth_topic_, info_topic_, health_topic_;
  double expected_rate_hz_{10.0};
  double max_age_sec_{0.25};
  double max_sync_skew_sec_{0.03};
  int64_t calibration_revision_{0};
  uint64_t sequence_{0},generation_{0},matched_sequence_{0},recovery_after_match_{0};
  bool continuity_broken_{true},recovery_required_{false};
  rclcpp::Time recovery_capture_after_{static_cast<int64_t>(0),RCL_ROS_TIME};
  StreamState color_, depth_, info_;
  std::deque<StreamState> color_history_,depth_history_,info_history_;
  std::optional<MatchedSnapshot> matched_;
  double matched_max_interval_sec_{0.0};
  std::optional<sensor_msgs::msg::CameraInfo> baseline_info_;bool calibration_changed_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<astribot_perception_msgs::msg::CameraHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CameraHealthNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "camera_health: %s\n", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
