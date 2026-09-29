#include "astribot_sensor_sync/sync_core.hpp"
#include "astribot_sensor_sync/msg/trigger_edge.hpp"
#include "astribot_sensor_sync/msg/sensor_timing.hpp"
#include "astribot_sensor_sync/msg/sync_status.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
using namespace std::chrono_literals;
namespace ss=astribot::sync;
using Timing=astribot_sensor_sync::msg::SensorTiming;
using Edge=astribot_sensor_sync::msg::TriggerEdge;
using Status=astribot_sensor_sync::msg::SyncStatus;
class SyncMonitor final:public rclcpp::Node {
 using Steady=std::chrono::steady_clock;
 struct Pending {Timing message;Steady::time_point received;};
 struct State {Timing message{rosidl_runtime_cpp::MessageInitialization::ALL};Steady::time_point received;bool seen{};};
 public:
 SyncMonitor():Node("sensor_sync_monitor") {
   ss::Config c;
   c.sources=declare_parameter<std::vector<std::string>>("sources",std::vector<std::string>{"head_rgbd/color","head_rgbd/depth","head_stereo/left","head_stereo/right","torso_rgbd/color","torso_rgbd/depth","left_wrist_rgbd/color","left_wrist_rgbd/depth","right_wrist_rgbd/color","right_wrist_rgbd/depth","lidar_front","lidar_back","imu"});
   const auto time_only=declare_parameter<std::vector<std::string>>("clock_only_sources",std::vector<std::string>{"lidar_front","lidar_back","imu"});
   c.clock_only_sources={time_only.begin(),time_only.end()};
   c.max_skew_ns=declare_parameter<int64_t>("max_trigger_skew_ns",2000000);
   c.max_uncertainty_ns=declare_parameter<int64_t>("max_clock_uncertainty_ns",1000000);
   max_age_=c.max_age_ns=declare_parameter<int64_t>("max_sample_age_ns",250000000);
   max_receive_age_=declare_parameter<int64_t>("max_receive_age_ns",250000000);
   reorder_ns_=declare_parameter<int64_t>("reorder_wait_ns",50000000);
   c.reference_clock_epoch=declare_parameter<std::string>("reference_clock_epoch","UNCONFIGURED");
   c.allow_simulated=declare_parameter<bool>("allow_simulated",false);
   if(max_receive_age_<=0 || reorder_ns_<0 || reorder_ns_>max_receive_age_)throw std::invalid_argument("invalid receipt/reorder budget");
   monitor_=std::make_unique<ss::Monitor>(c);
   for(const auto&s:c.sources)states_.emplace(s,State{});
   publisher_=create_publisher<Status>("sensor_sync/status",rclcpp::QoS(64).reliable());
   edges_=create_subscription<Edge>("sensor_sync/trigger_edges",rclcpp::QoS(256).reliable(),[this](Edge::ConstSharedPtr e){
     if(!monitor_->trigger({e->group,e->trigger_epoch,e->clock_epoch,e->sequence,e->stamp_ns,e->simulated}))fault_="TRIGGER_INVALID_OR_CONFLICT";
     process_pending();
   });
   timing_=create_subscription<Timing>("sensor_sync/frame_timing",rclcpp::QoS(128).reliable(),[this](Timing::ConstSharedPtr f){
     const auto received=Steady::now();
     if(!states_.count(f->source_id)){publish(*f,{false,false,"UNKNOWN_SOURCE"});return;}
     auto result=evaluate(*f);
     if(result.reason=="TRIGGER_UNKNOWN") {
       if(pending_.size()>=64){publish(pending_.front().message,{false,false,"REORDER_OVERFLOW"});pending_.pop_front();}
       pending_.push_back({*f,received});return;
     }
     finish(*f,result,received);
   });
   timer_=create_wall_timer(20ms,[this]{
     const auto current=now().nanoseconds();
     if(last_now_>0 && current<last_now_)fault_="CLOCK_ROLLBACK";
     last_now_=current;process_pending();
     for(const auto& [source,state]:states_) {
       if(!state.seen){Timing f;f.source_id=source;publish(f,{false,false,fault_.empty()?"MISSING_SOURCE":fault_});continue;}
       std::string reason=fault_;
       if(reason.empty()&&std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now()-state.received).count()>max_receive_age_)reason="SOURCE_TIMEOUT";
       if(reason.empty()&&state.message.capture_stamp_ns>current)reason="FUTURE_SAMPLE";
       if(reason.empty()&&current-state.message.capture_stamp_ns>max_age_)reason="STALE_SAMPLE";
       if(!reason.empty())publish(state.message,{false,false,reason});
     }
   });
 }
 private:
 ss::Result evaluate(const Timing& f) {
   if(!fault_.empty())return {false,false,fault_};
   return monitor_->frame({f.source_id,f.source_epoch,f.group,f.trigger_epoch,f.clock_epoch,f.frame_sequence,f.trigger_sequence,f.capture_stamp_ns,f.clock_uncertainty_ns,f.hardware_associated,f.simulated,f.clock_locked},now().nanoseconds());
 }
 void publish(const Timing& f,const ss::Result&r) {
   Status s;s.source_id=f.source_id;s.source_epoch=f.source_epoch;s.clock_epoch=f.clock_epoch;s.frame_sequence=f.frame_sequence;s.trigger_sequence=f.trigger_sequence;s.capture_stamp_ns=f.capture_stamp_ns;s.evaluated_stamp_ns=now().nanoseconds();s.trigger_skew_ns=r.skew_ns;s.dropped_frames=r.dropped;s.valid=r.valid;s.hardware_trigger_verified=r.hardware;s.simulated=f.simulated;s.reason=r.reason;publisher_->publish(s);
 }
 void finish(const Timing& f,const ss::Result& r,Steady::time_point received) {
   auto it=states_.find(f.source_id);
   if(it!=states_.end())it->second={f,received,true};
   publish(f,r);
 }
 void process_pending() {
   for(auto it=pending_.begin();it!=pending_.end();) {
     const auto age=std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now()-it->received).count();
     // Do not refresh receipt age on a late edge. Expiry wins before validator state advances.
     auto result=age>max_receive_age_?ss::Result{false,false,"SOURCE_TIMEOUT"}:evaluate(it->message);
     if(result.reason=="TRIGGER_UNKNOWN" && age<=reorder_ns_){++it;continue;}
     if(result.reason=="TRIGGER_UNKNOWN")result.reason="TRIGGER_MISSING";
     finish(it->message,result,it->received);it=pending_.erase(it);
   }
 }
 std::unique_ptr<ss::Monitor>monitor_;std::deque<Pending>pending_;std::map<std::string,State>states_;
 std::string fault_;int64_t max_age_{},max_receive_age_{},reorder_ns_{},last_now_{};
 rclcpp::Publisher<Status>::SharedPtr publisher_;
 rclcpp::Subscription<Edge>::SharedPtr edges_;rclcpp::Subscription<Timing>::SharedPtr timing_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<SyncMonitor>());}catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("sensor_sync_monitor"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
