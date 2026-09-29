#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "astribot_s1_perception_native/map_odom_core.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/executors/static_single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/buffer_core.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace mo=astribot_s1_perception_native::map_odom;
namespace {
struct ExitRequested { int code; };
// Python Duration and Timer first multiply by 1e9, truncate, then check int64.
// Keep conversion at the original consumption site, especially TF timeout.
int64_t nanoseconds(double seconds) {
  const double ns=seconds*1e9;
  if(!std::isfinite(ns)||ns>=9223372036854775808.0||ns< -9223372036854775808.0)
    throw std::invalid_argument("duration cannot be represented as int64 nanoseconds");
  return static_cast<int64_t>(ns);
}
class MapOdomTfNode final : public rclcpp::Node {
 public:
  MapOdomTfNode() : Node("map_odom_tf",rclcpp::NodeOptions().use_clock_thread(false)) {
    slam_world_=declare_parameter("slam_world_frame",std::string("map"));
    slam_base_=declare_parameter("slam_base_frame",std::string("aft_mapped"));
    map_frame_=declare_parameter("map_frame",std::string("map"));
    odom_frame_=declare_parameter("odom_frame",std::string("odom"));
    base_frame_=declare_parameter("base_frame",std::string("astribot_torso_base"));
    const auto rate=declare_parameter("publish_rate",20.0);
    tf_timeout_=declare_parameter("tf_timeout_sec",.2);
    const auto jump=declare_parameter("jump_report_m",.30);
    max_tilt_=declare_parameter("max_tilt_rad",.10);
    source_timeout_=declare_parameter("source_timeout_sec",60.0);
    const auto report=declare_parameter("report_period_sec",10.0);
    max_source_age_=declare_parameter("max_source_age_sec",1.0);
    if(slam_world_!=map_frame_) throw mo::DecompositionError("SLAM must directly use map_frame");
    if(!(rate>0)) throw mo::DecompositionError("publish_rate must be positive");
    decomposer_=std::make_unique<mo::Decomposer>(jump,max_tilt_);
    // Python Buffer() has no node clock and does not clear on ROS clock jumps.
    // Listener stays in this executor: source/clock callbacks cannot run while
    // the legacy synchronous lookup timeout is waiting.
    rclcpp::SubscriptionOptions options;
    listener_=std::make_unique<tf2_ros::TransformListener>(buffer_,this,false,
      tf2_ros::DynamicListenerQoS(),tf2_ros::StaticListenerQoS(),options,options);
    broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    start_sec_=nowSeconds();
    tick_=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_nanoseconds(nanoseconds(1.0/rate)),
                               [this]{ tick(); });
    watchdog_=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_nanoseconds(2000000000LL),
                                   [this]{ watchdog(); });
    if(report>0)
      report_=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_nanoseconds(nanoseconds(report)),
                                   [this]{ reportStats(); });
    RCLCPP_INFO(get_logger(),"map_odom_tf 启动: SLAM %s→%s; odom %s→%s; 输出 %s→%s (%.0fHz). "
      "前提: 两个base必须表示同一物理点。",slam_world_.c_str(),slam_base_.c_str(),
      odom_frame_.c_str(),base_frame_.c_str(),map_frame_.c_str(),odom_frame_.c_str(),rate);
  }
 private:
  double nowSeconds() { return static_cast<double>(get_clock()->now().nanoseconds())*1e-9; }
  geometry_msgs::msg::TransformStamped latest(const std::string &target,const std::string &source) {
    const int64_t timeout=nanoseconds(tf_timeout_);
    const rclcpp::Time start=system_clock_.now();
    // Avoid tf2_ros::Buffer's dedicated-thread prerequisite and ROS-clock reset
    // policy. This is the same SYSTEM_TIME/20ms polling as Humble's Python Buffer.
    if(timeout!=0) {
      const auto deadline=start+rclcpp::Duration::from_nanoseconds(timeout);
      while(system_clock_.now()<deadline &&
            !buffer_.canTransform(target,source,tf2::TimePointZero) &&
            system_clock_.now()+rclcpp::Duration::from_nanoseconds(3000000000LL)>=start) {
        if(!rclcpp::ok()) throw ExitRequested{0};
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    if(!rclcpp::ok()) throw ExitRequested{0};
    return buffer_.lookupTransform(target,source,tf2::TimePointZero);
  }
  std::optional<mo::Pose2D> missing(const std::string &target,const std::string &source,
                                   size_t kind,const std::exception &error) {
    const auto count=++miss_[kind];
    if(count<=3||count%100==0)
      RCLCPP_WARN(get_logger(),"取不到 %s→%s（第 %lu 次）: %s; 检查%s TF发布方。",
        target.c_str(),source.c_str(),static_cast<unsigned long>(count),error.what(),kind==0?"SLAM":"里程计");
    return std::nullopt;
  }
  std::optional<mo::Pose2D> lookup(const std::string &target,const std::string &source,size_t kind) {
    geometry_msgs::msg::TransformStamped tf;
    try { tf=latest(target,source); }
    catch(const tf2::LookupException &e) { return missing(target,source,kind,e); }
    catch(const tf2::ConnectivityException &e) { return missing(target,source,kind,e); }
    catch(const tf2::ExtrapolationException &e) { return missing(target,source,kind,e); }
    // Use the original two floating conversions; integer-age arithmetic would
    // change exact tolerance decisions at large epoch timestamps.
    const double stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9;
    const auto age=mo::checkSourceAge(nowSeconds(),stamp,max_source_age_);
    if(age.stale) {
      const auto count=++stale_[kind];
      if(count<=3||count%100==0)
        RCLCPP_DEBUG(get_logger(),"（第 %lu 次源时间诊断）%s→%s age=%.9gs limit=%.9gs reason=%s",
          static_cast<unsigned long>(count),target.c_str(),source.c_str(),age.age_sec,max_source_age_,
          age.reason==mo::AgeReason::Unstamped?"unstamped":age.reason==mo::AgeReason::Future?"future":"stale");
    }
    const auto &t=tf.transform.translation; const auto &q=tf.transform.rotation;
    height_[kind]=t.z;  // Legacy height is intentionally not finite-validated.
    if(decomposer_->checkPlanar(q.x,q.y)) {
      const auto count=decomposer_->stats.rejected_tilt;
      if(count<=3||count%100==0)
        RCLCPP_WARN(get_logger(),"%s→%s 四元数 x=%.4f y=%.4f 超过平面容差 %.9g; "
          "按2D分解，倾角告警不拒绝输出。",target.c_str(),source.c_str(),q.x,q.y,max_tilt_);
    }
    try { return mo::Pose2D(t.x,t.y,mo::yawFromQuaternion(q.z,q.w)); }
    catch(const mo::DecompositionError &error) {
      RCLCPP_ERROR(get_logger(),"%s→%s 的数值非法: %s",target.c_str(),source.c_str(),error.what());
      return std::nullopt;
    }
  }
  void tick() {
    const auto slam=lookup(slam_world_,slam_base_,0);
    if(!slam) return;
    const auto odom=lookup(odom_frame_,base_frame_,1);
    if(!odom) return;
    const auto result=decomposer_->update(*slam,*odom);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp=now(); tf.header.frame_id=map_frame_; tf.child_frame_id=odom_frame_;
    tf.transform.translation.x=result.pose.x;tf.transform.translation.y=result.pose.y;
    tf.transform.translation.z=height_[0]-height_[1];
    const auto q=mo::quaternionFromYaw(result.pose.theta);
    tf.transform.rotation.z=q.first;tf.transform.rotation.w=q.second;
    if(!rclcpp::ok()) throw ExitRequested{0};
    broadcaster_->sendTransform(tf);
    if(result.jumped)
      RCLCPP_INFO(get_logger(),"map→odom 跳变 %.3fm（累计 %lu 次，最大 %.3fm）; "
        "SLAM全局修正保留在map→odom，odom→base保持连续。",result.jump_m,
        static_cast<unsigned long>(decomposer_->stats.jumps),decomposer_->stats.max_jump_m);
  }
  void watchdog() {
    if(decomposer_->stats.updates>0||nowSeconds()-start_sec_<source_timeout_) return;
    RCLCPP_ERROR(get_logger(),"%.9gs 内一次都没算出 map→odom，退出。缺失次数: SLAM %lu odom %lu; "
      "陈旧次数: SLAM %lu odom %lu; 检查对应发布方。",source_timeout_,
      static_cast<unsigned long>(miss_[0]),static_cast<unsigned long>(miss_[1]),
      static_cast<unsigned long>(stale_[0]),static_cast<unsigned long>(stale_[1]));
    throw ExitRequested{1};
  }
  void reportStats() {
    const auto &s=decomposer_->stats;
    std::string pose="(尚无)";
    if(decomposer_->last()) {
      const auto &p=*decomposer_->last(); char text[160];
      std::snprintf(text,sizeof(text),"(%.3f, %.3f, %.3f)",p.x,p.y,p.theta);pose=text;
    }
    RCLCPP_INFO(get_logger(),"map→odom=%s 更新=%lu 跳变=%lu(最大 %.3fm) 倾角超限=%lu "
      "缺失 slam=%lu odom=%lu 陈旧 slam=%lu odom=%lu",pose.c_str(),
      static_cast<unsigned long>(s.updates),static_cast<unsigned long>(s.jumps),s.max_jump_m,
      static_cast<unsigned long>(s.rejected_tilt),static_cast<unsigned long>(miss_[0]),
      static_cast<unsigned long>(miss_[1]),static_cast<unsigned long>(stale_[0]),static_cast<unsigned long>(stale_[1]));
  }
  std::string slam_world_,slam_base_,map_frame_,odom_frame_,base_frame_;
  double tf_timeout_,source_timeout_,max_source_age_,max_tilt_,start_sec_;
  std::unique_ptr<mo::Decomposer> decomposer_;
  tf2::BufferCore buffer_;
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::array<double,2> height_{{0,0}};
  std::array<uint64_t,2> miss_{{0,0}},stale_{{0,0}};
  rclcpp::TimerBase::SharedPtr tick_,watchdog_,report_;
};
}
int main(int argc,char **argv) {
  int code=0;
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<MapOdomTfNode>();
    // Static executor visits the collected ready set in one cycle. Like rclpy,
    // it keeps source/clock/watchdog callbacks live even for a zero-period timer.
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);executor.spin();executor.remove_node(node);
  } catch(const mo::DecompositionError &error) {
    RCLCPP_ERROR(rclcpp::get_logger("map_odom_tf"),"参数或分解被拒绝: %s",error.what());code=2;
  } catch(const ExitRequested &exit) { code=exit.code; }
  catch(const std::exception &error) {
    if(rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("map_odom_tf"),"%s",error.what());code=1;
    }
  }
  if(rclcpp::ok()) rclcpp::shutdown();
  return code;
}
