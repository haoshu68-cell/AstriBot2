#include "astribot_s1_autonomy/session_archive.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <astribot_slam_msgs/msg/keyframe_pose_array.hpp>
#include <future>
#include "astribot_navigation_zones/client.hpp"

namespace astribot_s1_autonomy {
class MappingSession : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
  using Get = rcl_interfaces::srv::GetParameters;
  using Set = rcl_interfaces::srv::SetParameters;
  using Trigger = std_srvs::srv::Trigger;
  std::string state_{"IDLE"}, detail_, end_reason_;
  const std::string session_id_{"slam_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count())};
  fs::path directory_;
  bool require_zones_{false};astribot_navigation_zones::Client zones_;nlohmann::json archived_zones_;
  Clock::time_point deadline_, odom_at_{}, still_since_{}, next_status_{};
  bool pending_{false}, finish_sent_{false}, final_seen_{false};
  size_t samples_{0}, updates_{0};
  uint64_t generation_{0};
  int64_t request_id_{0}, last_odom_stamp_{0};
  double timeout_{120.0}, settle_sec_{0.5}, odom_timeout_{0.3};
  std::future<std::string> inspection_;
  rclcpp::Client<Get>::SharedPtr get_;
  rclcpp::Client<Set>::SharedPtr set_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<astribot_slam_msgs::msg::KeyframePoseArray>::SharedPtr final_;
  rclcpp::Service<Trigger>::SharedPtr finalize_, completed_, canceled_, retry_;
  rclcpp::TimerBase::SharedPtr timer_;
  void status(const std::string & state, const std::string & detail) {
    state_ = state; detail_ = detail;
    RCLCPP_INFO(get_logger(), "mapping_session state=%s detail=%s directory=%s",
      state.c_str(), detail.c_str(), directory_.c_str());
    publish();
  }
  void publish() {
    std_msgs::msg::String message;
    message.data = nlohmann::json({{"state", state_}, {"session_id",session_id_}, {"detail", detail_},
      {"directory", directory_.string()}, {"finish_sent", finish_sent_}, {"exploration_outcome", end_reason_}}).dump();
    status_->publish(message);
  }
  bool stopped() const {
    return odom_->get_publisher_count() == 1 && samples_ >= 5 && Clock::now() - odom_at_ <= std::chrono::duration<double>(odom_timeout_) &&
      Clock::now() - still_since_ >= std::chrono::duration<double>(settle_sec_);
  }
  void begin(Trigger::Response & response, bool retry, const std::string & reason = "MANUAL") {
    if (retry && state_ != "FAILED") {
      response.success = false; response.message = "Retry is only available after FAILED"; return;
    }
    if (state_ != "IDLE" && state_ != "FAILED") {
      response.success = true; response.message = state_ + ": " + directory_.string(); return;
    }
    if (state_ == "FAILED" && !retry) {
      response.success = false; response.message = "FAILED: inspect status, then use ~/retry"; return;
    }
    if (!retry) end_reason_ = reason;
    ++generation_; pending_ = false;
    deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_));
    samples_ = 0;
    // A timed-out finish may already be executing: retries only wait/validate, never resend it.
    status(finish_sent_ ? "FINALIZING" : "WAIT_STOP", "Finalization accepted; awaiting evidence");
    response.success = true; response.message = "Accepted; completion is reported on /mapping_session/status";
  }
  void tick() {
    if (Clock::now() >= next_status_) {publish(); next_status_ = Clock::now() + std::chrono::seconds(1);}
    if (state_ == "IDLE" || state_ == "SAVED" || state_ == "FAILED") return;
    if (inspection_.valid()) {
      if (inspection_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
      const auto error = inspection_.get();
      if (error.empty()) {status("SAVED", "Map, keyframes and manifest committed"); return;}
      detail_ = error;
    }
    if (Clock::now() > deadline_) {
      if (pending_) {
        if (state_ == "READ_CONFIG") get_->remove_pending_request(request_id_);
        else set_->remove_pending_request(request_id_);
      }
      pending_ = false; ++generation_;
      status("FAILED", "Timed out; " + detail_); return;
    }
    if (state_ == "WAIT_STOP" && stopped() && get_->service_is_ready()) {
      status("READ_CONFIG", "Checking Voxel save configuration"); pending_ = true;
      auto request = std::make_shared<Get::Request>();
      request->names = {"General.save_path", "General.mapname", "General.is_save_map", "finish"};
      const auto generation = generation_;
      auto result = get_->async_send_request(request, [this, generation](rclcpp::Client<Get>::SharedFuture future) {
        if (generation != generation_) return;
        pending_ = false;
        try {
          const auto v = future.get()->values;
          if (v.size() != 4 || v[0].type != 4 || v[1].type != 4 || v[2].type != 2 ||
            v[2].integer_value != 1 || !std::regex_match(v[1].string_value, std::regex("[A-Za-z0-9_-]+")))
            throw std::runtime_error("SLAM must start with save_map:=1 and unique map_name");
          if (v[3].type != 1 || v[3].bool_value) throw std::runtime_error("SLAM already finalizing or finish unavailable");
          directory_ = fs::weakly_canonical(fs::path(v[0].string_value) / v[1].string_value);
          status("WAIT_FINISH", "Save configuration checked; waiting for fresh standstill");
        } catch (const std::exception & e) {status("FAILED", e.what());}
      }); request_id_ = result.request_id;
    }
    if (state_ == "WAIT_FINISH" && stopped() && set_->service_is_ready()) {
      if(require_zones_){auto z=zones_.get();if(!z||z->context!="mapping:"+session_id_){detail_="Waiting for current navigation zones snapshot";return;}
        archived_zones_={{"schema_version",1},{"frame","map"},{"context_id",z->context},{"revision",z->revision},{"regions",astribot_navigation_zones::encodeRegions(z->regions)}};}
      status("FINALIZING", "Submitting finish; this ends the current SLAM session");
      finish_sent_ = true; pending_ = true;
      auto request = std::make_shared<Set::Request>();
      rcl_interfaces::msg::Parameter p; p.name = "finish"; p.value.type = 1; p.value.bool_value = true;
      request->parameters.push_back(p);
      const auto generation = generation_;
      auto result = set_->async_send_request(request, [this, generation](rclcpp::Client<Set>::SharedFuture future) {
        if (generation != generation_) return;
        pending_ = false;
        try {
          const auto results = future.get()->results;
          if (results.size() != 1 || !results[0].successful) {
            finish_sent_ = false; status("FAILED", "SLAM rejected finish");
          }
        } catch (const std::exception & e) {status("FAILED", e.what());}
      }); request_id_ = result.request_id;
    }
    if (state_ == "FINALIZING" && final_seen_ && !pending_) {
      const auto dir = directory_; const auto updates = updates_; const auto reason = end_reason_;
      const auto zones=archived_zones_;
      inspection_ = std::async(std::launch::async, [dir, updates, reason,zones] {
        try {auto manifest = inspectSession(dir); manifest["exploration_outcome"] = reason;if(!zones.is_null())manifest["navigation_zones"]=zones;
          commitSession(dir, manifest, updates); return std::string();}
        catch (const std::exception & e) {return std::string(e.what());}
      });
    }
  }
public:
  explicit MappingSession(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("mapping_session", options) {
    require_zones_=declare_parameter("require_navigation_zones",false);if(require_zones_)zones_.init(*this);
    timeout_ = declare_parameter("save_timeout_sec", 120.0);
    settle_sec_ = declare_parameter("settle_sec", 0.5);
    odom_timeout_ = declare_parameter("odom_timeout_sec", 0.3);
    for (const auto v : {timeout_, settle_sec_, odom_timeout_})
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Timing parameters must be positive and finite");
    const auto slam = declare_parameter("slam_node", std::string("/voxelslam"));
    get_ = create_client<Get>(slam + "/get_parameters"); set_ = create_client<Set>(slam + "/set_parameters");
    status_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).transient_local());
    odom_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic", std::string("/odom")),
      rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & v = msg->twist.twist; const auto received = Clock::now();
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (stamp <= last_odom_stamp_) {samples_ = 0; last_odom_stamp_ = stamp; return;}
        last_odom_stamp_ = stamp;
        const auto age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
        const double speed = std::hypot(v.linear.x, v.linear.y);
        if (!std::isfinite(speed) || !std::isfinite(v.angular.z) || speed > .01 || std::abs(v.angular.z) > .02 ||
            age < 0 || age > odom_timeout_) {samples_ = 0; odom_at_ = received; return;}
        if (!samples_ || received - odom_at_ > std::chrono::duration<double>(odom_timeout_)) {
          samples_ = 0; still_since_ = received;
        }
        odom_at_ = received; ++samples_;
      });
    final_ = create_subscription<astribot_slam_msgs::msg::KeyframePoseArray>(
      "/voxel_slam/keyframe_pose_array", 10, [this](astribot_slam_msgs::msg::KeyframePoseArray::ConstSharedPtr msg) {
        if (!finish_sent_ || !msg->is_final || msg->save_dir.empty() || msg->updates.empty()) return;
        try {
        if (fs::path(msg->save_dir).lexically_normal() != directory_.lexically_normal() &&
            fs::weakly_canonical(msg->save_dir) != directory_) return;
        if (msg->map_name != directory_.filename().string()) return;
        final_seen_ = true; updates_ = msg->updates.size();
        } catch (const fs::filesystem_error & e) {RCLCPP_WARN(get_logger(), "Invalid final path: %s", e.what());}
      });
    finalize_ = create_service<Trigger>("~/finalize", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {begin(*r, false);});
    completed_ = create_service<Trigger>("~/finalize_completed", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {begin(*r, false, "COMPLETED");});
    canceled_ = create_service<Trigger>("~/finalize_canceled", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {begin(*r, false, "CANCELED_PARTIAL");});
    retry_ = create_service<Trigger>("~/retry", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {begin(*r, true);});
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      try {tick();} catch (const std::exception & e) {++generation_; status("FAILED", e.what());}
    });
  }
};
}
#ifndef ASTRIBOT_MAPPING_SESSION_NO_MAIN
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astribot_s1_autonomy::MappingSession>());
  rclcpp::shutdown(); return 0;
}
#endif
