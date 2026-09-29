#include <chrono>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <astribot_perception_msgs/msg/camera_session_state.hpp>
#include <astribot_perception_msgs/srv/set_camera_session.hpp>
#include "astribot_s1_autonomy/camera_session.hpp"

// This gate owns perception subscriptions only. A session is not motion authority.
class WristCameraSession final : public rclcpp::Node {
  using State = astribot_perception_msgs::msg::CameraSessionState;
  using Service = astribot_perception_msgs::srv::SetCameraSession;
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  static constexpr int64_t max_age_ns = 250000000;
  static int64_t wall() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  template<class Message> struct Stream {
    struct Pending {typename Message::ConstSharedPtr message;int64_t received;};
    std::deque<Pending> pending;
    typename rclcpp::Publisher<Message>::SharedPtr publisher;
    typename rclcpp::Subscription<Message>::SharedPtr subscription;
    uint64_t received=0, forwarded=0, deferred=0, rejected=0, overflow=0;
    void reset() {subscription.reset();pending.clear();}
  };

public:
  explicit WristCameraSession(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
  : Node("wrist_camera_session", options),
    lease_(std::to_string(wall())+":"+std::to_string(std::random_device{}())) {
    camera_=declare_parameter<std::string>("camera_id", "left_wrist_rgbd");
    if(camera_!="left_wrist_rgbd" && camera_!="right_wrist_rgbd")
      throw std::invalid_argument("session gate is reserved for wrist cameras");
    raw_=declare_parameter<std::string>("raw_prefix", "/camera/raw/"+camera_);
    active_=declare_parameter<std::string>("active_prefix", "/manipulation/camera/"+camera_+"/active");
    queue_depth_=declare_parameter<int>("queue_depth", 4);
    if(queue_depth_<1 || queue_depth_>4) throw std::invalid_argument("queue_depth must be 1..4");
    auto qos=rclcpp::SensorDataQoS().keep_last(queue_depth_);
    color_.publisher=create_publisher<Image>(active_+"/image", qos);
    depth_.publisher=create_publisher<Image>(active_+"/depth_image", qos);
    info_.publisher=create_publisher<Info>(active_+"/camera_info", qos);
    state_pub_=create_publisher<State>("/perception/camera_session/"+camera_,
      rclcpp::QoS(1).reliable().transient_local());
    service_=create_service<Service>("/perception/camera_session/"+camera_+"/set",
      [this](const std::shared_ptr<Service::Request> req,std::shared_ptr<Service::Response> res) {
        if(req->header.stamp.sec<0 || req->header.stamp.nanosec>=1000000000u) {
          res->accepted=false;res->reason_code="INVALID_STAMP";res->state=state();return;
        }
        if(req->camera_id!=camera_) {
          res->accepted=false;res->reason_code="CAMERA_MISMATCH";res->state=state();return;
        }
        res->accepted=lease_.request(req->operation,req->owner_id,req->execution_id,
          req->request_id,req->session_token,req->lease_sec,
          rclcpp::Time(req->header.stamp).nanoseconds(),now().nanoseconds(),wall());
        update();res->state=state();res->reason_code=lease_.reason();state_pub_->publish(res->state);
      });
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this] {
      update();
      const auto current=wall();
      if(current-last_status_>=50000000) {state_pub_->publish(state());last_status_=current;}
      if(current-last_metrics_>=5000000000LL) {
        metrics("rgb",color_);metrics("depth",depth_);metrics("info",info_);last_metrics_=current;
      }
    });
  }

private:
  State state() {
    State s;s.header.stamp=now();s.camera_id=camera_;s.owner_id=lease_.owner();
    s.execution_id=lease_.execution();s.session_token=lease_.token();s.active=lease_.active();
    s.activated_at=rclcpp::Time(lease_.activated());s.valid_until=rclcpp::Time(lease_.deadline());
    s.reason_code=lease_.reason();return s;
  }
  template<class Message> void metrics(const char * stream,const Stream<Message> & s) {
    if(!s.received) return;
    RCLCPP_INFO(get_logger(),"CAMERA_SESSION_STREAM camera=%s stream=%s received=%lu forwarded=%lu clock_deferred=%lu rejected=%lu overflow=%lu pending=%zu",
      camera_.c_str(),stream,s.received,s.forwarded,s.deferred,s.rejected,s.overflow,s.pending.size());
  }
  template<class Message> void flush(Stream<Message> & stream) {
    const auto ros=now().nanoseconds(),steady=wall();
    lease_.tick(ros,steady);
    if(!lease_.active()) {stream.pending.clear();return;}
    for(auto it=stream.pending.begin();it!=stream.pending.end();) {
      const auto stamp=rclcpp::Time(it->message->header.stamp).nanoseconds();
      if(stamp<lease_.activated() || ros-stamp>max_age_ns || steady-it->received>max_age_ns) {
        ++stream.rejected;it=stream.pending.erase(it);
      } else if(stamp>ros) {
        ++it; // Never forward a future capture. Wait for its matching clock tick.
      } else {
        stream.publisher->publish(*it->message);++stream.forwarded;it=stream.pending.erase(it);
      }
    }
  }
  template<class Message> void enqueue(Stream<Message> & stream,typename Message::ConstSharedPtr msg) {
    ++stream.received;
    const auto ros=now().nanoseconds(),steady=wall();lease_.tick(ros,steady);
    const auto & t=msg->header.stamp;
    if(!lease_.active() || t.sec<0 || t.nanosec>=1000000000u) {++stream.rejected;return;}
    const auto stamp=rclcpp::Time(t).nanoseconds();
    if(stamp<lease_.activated() || ros-stamp>max_age_ns || stamp-ros>max_age_ns) {
      ++stream.rejected;return;
    }
    if(stamp>ros) ++stream.deferred;
    stream.pending.push_back({std::move(msg),steady});
    while(stream.pending.size()>static_cast<size_t>(queue_depth_)) {
      stream.pending.pop_front();++stream.overflow;
    }
    flush(stream);
  }
  void update() {
    lease_.tick(now().nanoseconds(),wall());
    if(!lease_.active()) {color_.reset();depth_.reset();info_.reset();return;}
    if(buffer_token_!=lease_.token()) {
      color_.pending.clear();depth_.pending.clear();info_.pending.clear();buffer_token_=lease_.token();
    }
    if(!color_.subscription) {
      auto qos=rclcpp::SensorDataQoS().keep_last(queue_depth_);
      color_.subscription=create_subscription<Image>(raw_+"/image",qos,
        [this](Image::ConstSharedPtr msg) {enqueue(color_,msg);});
      depth_.subscription=create_subscription<Image>(raw_+"/depth_image",qos,
        [this](Image::ConstSharedPtr msg) {enqueue(depth_,msg);});
      info_.subscription=create_subscription<Info>(raw_+"/camera_info",qos,
        [this](Info::ConstSharedPtr msg) {enqueue(info_,msg);});
    }
    flush(color_);flush(depth_);flush(info_);
  }
  astribot::vision::CameraSession lease_;
  std::string camera_,raw_,active_,buffer_token_;
  int queue_depth_{4};int64_t last_status_=0,last_metrics_=0;
  Stream<Image> color_,depth_;Stream<Info> info_;
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  rclcpp::Service<Service>::SharedPtr service_;rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<WristCameraSession>());}
  catch(const std::exception & error) {
    fprintf(stderr,"wrist camera session: %s\n",error.what());rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
