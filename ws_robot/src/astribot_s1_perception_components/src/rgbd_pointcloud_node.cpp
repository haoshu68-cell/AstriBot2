// Bounded receive/pair -> latest work slot -> CPU/CUDA -> versioned commit.
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <cmath>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <astribot_perception_msgs/msg/camera_session_state.hpp>
#include <astribot_perception_msgs/msg/projection_health.hpp>
#include "astribot_s1_perception_components/depth_projection.hpp"
#include "astribot_s1_perception_components/projection_process.hpp"
#include "astribot_s1_perception_components/latency_metrics.hpp"

namespace astribot_s1_autonomy {
class RgbdPointcloudNode final:public rclcpp::Node {
  using Steady=std::chrono::steady_clock;
  using Image=sensor_msgs::msg::Image;using Info=sensor_msgs::msg::CameraInfo;
  using Session=astribot_perception_msgs::msg::CameraSessionState;
  using Health=astribot_perception_msgs::msg::ProjectionHealth;
  template<class T>using Cache=std::map<int64_t,std::pair<T,Steady::time_point>>;
  struct Job {Image::ConstSharedPtr image;Info::ConstSharedPtr info;Steady::time_point received;uint64_t generation;Steady::time_point paired;};
public:
  struct Statistics {
    uint64_t received=0,paired=0,replaced=0,processed=0,expired=0,invalidated=0,errors=0,evicted=0;
    uint64_t rejected_future=0,rejected_old=0,rejected_order=0;
    size_t cache_peak=0,pending_peak=0;
    double max_queue_ms=0,max_compute_ms=0,max_total_ms=0;
    std::array<astribot::vision::LatencySummary,5> latency{};
  };
  explicit RgbdPointcloudNode(const rclcpp::NodeOptions& options,
      std::unique_ptr<astribot::vision::DepthProjector> injected={})
  :Node("rgbd_pointcloud",options) {
    depth_topic_=declare_parameter<std::string>("depth_topic","~/depth_image");
    info_topic_=declare_parameter<std::string>("camera_info_topic","~/camera_info");
    output_topic_=declare_parameter<std::string>("output_topic","~/points");
    const auto decimation=declare_parameter<int64_t>("decimation",2);
    if(decimation<1 || decimation>64)throw std::invalid_argument("invalid RGB-D decimation");
    config_={0,0,0,0,declare_parameter("min_depth",.2),declare_parameter("max_depth",5.),static_cast<int>(decimation)};
    max_age_=declare_parameter("max_pair_age_sec",.25);
    backend_=declare_parameter<std::string>("projection_backend","cpu");
    camera_id_=declare_parameter<std::string>("camera_id",get_name());
    const auto health_topic=declare_parameter<std::string>("processing_health_topic","~/processing_health");
    boot_=std::to_string(::getpid())+"-"+std::to_string(Steady::now().time_since_epoch().count());
    if(!std::isfinite(max_age_)||max_age_<=0||max_age_>.5||config_.decimation<1||config_.decimation>64||
       !std::isfinite(config_.min_depth)||!std::isfinite(config_.max_depth)||config_.min_depth<0||config_.max_depth<=config_.min_depth)
      throw std::invalid_argument("invalid RGB-D budget");
    if(injected)projector_=std::move(injected);
    else if(backend_=="cuda") {
      astribot::vision::ProjectionProcessOptions p;
      p.executable=astribot::vision::projection_worker_executable();p.arguments={"--backend","cuda"};
      p.startup_timeout_sec=declare_parameter("worker_startup_timeout_sec",3.);
      p.request_timeout_sec=declare_parameter("worker_request_timeout_sec",.20);
      p.restart_backoff_sec=declare_parameter("worker_restart_backoff_sec",.1);
      const auto restarts=declare_parameter<int64_t>("worker_max_restarts",3);
      if(restarts<0||restarts>10)throw std::invalid_argument("invalid worker restart budget");
      p.max_restarts=static_cast<uint32_t>(restarts);
      projector_=std::make_unique<astribot::vision::ProjectionProcess>(std::move(p));
    }else projector_=astribot::vision::make_depth_projector(backend_);
    process_=dynamic_cast<astribot::vision::ProjectionProcess*>(projector_.get());
    if(declare_parameter("allow_cpu_fallback",false))cpu_fallback_=astribot::vision::make_depth_projector("cpu");
    const auto activation=declare_parameter<std::string>("activation_topic","");session_required_=!activation.empty();
    if(session_required_)session_sub_=create_subscription<Session>(activation,rclcpp::QoS(1).reliable().transient_local(),
      [this](Session::ConstSharedPtr s){
        std::lock_guard<std::mutex> lock(mutex_);clock_locked();
        if(!valid_stamp(s->header.stamp)||!valid_stamp(s->activated_at)||!valid_stamp(s->valid_until)) {
          ++stats_.errors;invalidate_locked();session_.reset();return;
        }
        const auto stamp=rclcpp::Time(s->header.stamp).nanoseconds();
        if(stamp<=0)return;
        if(session_&&session_->session_token==s->session_token&&
           stamp<rclcpp::Time(session_->header.stamp).nanoseconds())return;
        if(!s->active){invalidate_locked();session_=s;return;}
        if(rclcpp::Time(s->activated_at).nanoseconds()>stamp) {
          invalidate_locked();session_.reset();return;
        }
        if(!session_||session_->session_token!=s->session_token||session_->owner_id!=s->owner_id||session_->execution_id!=s->execution_id||session_->activated_at!=s->activated_at)
          invalidate_locked();
        session_=s;session_received_=Steady::now();
      });
    auto qos=rclcpp::SensorDataQoS().keep_last(4);
    points_pub_=create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_,rclcpp::SensorDataQoS().keep_last(1));
    health_pub_=create_publisher<Health>(health_topic,rclcpp::QoS(1).reliable().transient_local());
    health_timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{publish_health();});
    info_sub_=create_subscription<Info>(info_topic_,qos,[this](Info::ConstSharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex_);++stats_.received;
      if(!valid_stamp(msg->header.stamp)){++stats_.errors;processing_fault_locked("INVALID_INFO_STAMP");return;}
      const auto stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
      if(stamp<=0 || (context_&&stamp<rclcpp::Time(context_->header.stamp).nanoseconds()))return;
      if(context_&&(context_->header.frame_id!=msg->header.frame_id||context_->k!=msg->k||context_->width!=msg->width||context_->height!=msg->height)) {
        // Raw depth can arrive before its new calibration; retain only that exact pair.
        auto found=depths_.find(stamp);
        std::optional<std::pair<Image::ConstSharedPtr,Steady::time_point>> matching;
        if(found!=depths_.end()&&found->second.first->header.frame_id==msg->header.frame_id&&
           found->second.first->width==msg->width&&found->second.first->height==msg->height)matching=found->second;
        invalidate_locked();if(matching)depths_[stamp]=*matching;
      }
      context_=msg;infos_[stamp]={std::move(msg),Steady::now()};trim(infos_);pair_locked(stamp);
    });
    depth_sub_=create_subscription<Image>(depth_topic_,qos,[this](Image::ConstSharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex_);++stats_.received;
      if(!valid_stamp(msg->header.stamp)){++stats_.errors;processing_fault_locked("INVALID_DEPTH_STAMP");return;}
      const auto stamp=rclcpp::Time(msg->header.stamp).nanoseconds();if(!fresh_locked(stamp))return;
      if(msg->data.size()>33554432||uint64_t(msg->width)*msg->height>4194304){++stats_.errors;processing_fault_locked("INVALID_DEPTH_SIZE");return;}
      depths_[stamp]={std::move(msg),Steady::now()};trim(depths_);pair_locked(stamp);
    });
    diagnostics_=create_wall_timer(std::chrono::seconds(5),[this]{auto s=statistics();
      RCLCPP_INFO(get_logger(),"RGBD_WORKER paired=%lu replaced=%lu processed=%lu expired=%lu invalidated=%lu errors=%lu cache_peak=%zu pending_peak=%zu max_queue_ms=%.3f max_compute_ms=%.3f max_total_ms=%.3f",
        s.paired,s.replaced,s.processed,s.expired,s.invalidated,s.errors,s.cache_peak,s.pending_peak,s.max_queue_ms,s.max_compute_ms,s.max_total_ms);
      RCLCPP_INFO(get_logger(),"RGBD_INPUT received=%lu future=%lu old=%lu out_of_order=%lu evicted=%lu",
        s.received,s.rejected_future,s.rejected_old,s.rejected_order,s.evicted);
      const char* stages[]={"pair_wait","dispatch_wait","compute","publish","receive_to_publish"};
      for(size_t i=0;i<s.latency.size();++i){const auto& v=s.latency[i];
        RCLCPP_INFO(get_logger(),"RGBD_LATENCY stage=%s samples=%lu p50_upper_ms=%.3f p95_upper_ms=%.3f p99_upper_ms=%.3f max_ms=%.3f overflow=%lu bin_ms=0.050",
          stages[i],v.samples,v.p50_upper_ms,v.p95_upper_ms,v.p99_upper_ms,v.max_ms,v.overflow_samples);}
    });
    worker_=std::thread([this]{run();});
  }
  ~RgbdPointcloudNode()override {
    {std::lock_guard<std::mutex>lock(mutex_);stopping_=true;pending_.reset();}
    projector_->cancel();
    ready_.notify_one();if(worker_.joinable())worker_.join();
  }
  Statistics statistics(){std::lock_guard<std::mutex>lock(mutex_);auto result=stats_;
    for(size_t i=0;i<latency_.size();++i)result.latency[i]=latency_[i].summary();
    return result;}
private:
  static bool valid_stamp(const builtin_interfaces::msg::Time& t) {
    return t.sec>=0 && t.nanosec<1000000000u;
  }
  static double age(Steady::time_point t){return std::chrono::duration<double>(Steady::now()-t).count();}
  void invalidate_locked(){++generation_;pending_.reset();infos_.clear();depths_.clear();}
  void processing_fault_locked(const std::string& reason) {
    if(!processing_fault_) {
      invalidate_locked();recovery_after_=last_output_;processing_fault_=true;
    }
    failure_reason_=reason;
  }
  int64_t clock_locked(){
    const auto current=now().nanoseconds();
    last_clock_=current;return current;
  }
  bool fresh_locked(int64_t stamp){clock_locked();
    if(stamp<=0){++stats_.rejected_old;return false;}
    if(stamp<=last_scheduled_){++stats_.rejected_order;return false;}
    if(stamp<=recovery_after_){++stats_.rejected_old;return false;}return true;}
  template<class T>void trim(Cache<T>& cache){while(cache.size()>4){cache.erase(cache.begin());++stats_.evicted;}stats_.cache_peak=std::max(stats_.cache_peak,infos_.size()+depths_.size());}
  bool valid_locked(const Job& j){
    const auto stamp=rclcpp::Time(j.image->header.stamp).nanoseconds();
    if(j.generation!=generation_){++stats_.invalidated;return false;}
    if(stamp<=last_output_){++stats_.expired;return false;}
    if(session_required_&&(!session_||!session_->active||session_->session_token.empty()||
       stamp<rclcpp::Time(session_->activated_at).nanoseconds())){++stats_.invalidated;return false;}
    return true;
  }
  void publish_health() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto current=clock_locked();
    bool valid=last_output_>0&&output_generation_==generation_&&!processing_fault_&&!stopping_;
    if(session_required_)valid=valid&&session_&&session_->active&&!session_->session_token.empty();
    const auto status=process_?process_->status():astribot::vision::ProjectionProcessStatus{};
    if(process_&&!using_fallback_)valid=valid&&(status.state=="READY"||status.state=="RUNNING");
    if(!valid&&last_output_>0&&output_generation_==generation_&&!processing_fault_)
      processing_fault_locked("OUTPUT_UNAVAILABLE");
    Health h;h.header.stamp=rclcpp::Time(current);h.header.frame_id=output_frame_;
    h.camera_id=camera_id_;h.backend=using_fallback_?"cpu":backend_;h.processing_epoch=boot_+":"+std::to_string(generation_);
    h.worker_epoch=status.worker_epoch;h.worker_pid=status.pid;h.restarts=status.restarts;
    h.state=process_?status.state:(valid?"READY":"INVALID");
    h.reason_code=valid?"OK":(processing_fault_?failure_reason_:"NO_CURRENT_OUTPUT");
    if(!valid&&process_&&!using_fallback_&&(status.state=="BACKOFF"||status.state=="QUARANTINED"))h.reason_code=status.reason;
    if(using_fallback_){h.state=valid?"READY_CPU_FALLBACK":"CPU_FALLBACK_WAIT";if(valid)h.reason_code="OK_CPU_FALLBACK";}
    h.valid=valid;h.capture_stamp=rclcpp::Time(last_output_);
    h.epoch_first_capture_stamp=rclcpp::Time(epoch_first_capture_);h.published_stamp=rclcpp::Time(published_stamp_);
    if(last_output_>0) {
      auto until=last_output_+static_cast<int64_t>(max_age_*1.e9);
      if(session_required_&&session_)until=std::min(until,rclcpp::Time(session_->valid_until).nanoseconds());
      h.valid_until=rclcpp::Time(until);
    }
    h.sequence=++health_sequence_;h.processed=stats_.processed;h.discarded=stats_.expired+stats_.invalidated+stats_.replaced;
    h.errors=stats_.errors;h.pending_depth=pending_?1:0;health_pub_->publish(std::move(h));
  }
  void pair_locked(int64_t stamp){
    if(infos_.empty()||depths_.empty())return;
    auto d=std::prev(depths_.end());
    auto i=std::min_element(infos_.begin(),infos_.end(),[&](const auto &a,const auto &b){
      return std::abs(a.first-d->first)<std::abs(b.first-d->first);});
    stamp=d->first;
    if(stamp<=last_scheduled_)return;
    if(i->second.first->header.frame_id!=d->second.first->header.frame_id ||
       i->second.first->width!=d->second.first->width || i->second.first->height!=d->second.first->height)return;
    Job job{d->second.first,i->second.first,std::min(i->second.second,d->second.second),generation_,Steady::now()};
    depths_.erase(depths_.begin(),std::next(d));
    if(!valid_locked(job))return;
    latency_[0].add_ms(std::chrono::duration<double,std::milli>(job.paired-job.received).count());
    if(pending_)++stats_.replaced;
    pending_=std::move(job);last_scheduled_=stamp;++stats_.paired;stats_.pending_peak=1;ready_.notify_one();
  }
  sensor_msgs::msg::PointCloud2 compute(const Job& j){
    const auto& image=*j.image;const auto& info=*j.info;
    const bool floating=image.encoding=="32FC1",integral=image.encoding=="16UC1"||image.encoding=="mono16";
    if((!floating&&!integral)||image.width!=info.width||image.height!=info.height||image.header.frame_id.empty()||
       image.header.frame_id!=info.header.frame_id||!std::all_of(info.k.begin(),info.k.end(),[](double v){return std::isfinite(v);}))
      throw std::invalid_argument("RGB-D layout/frame/intrinsics mismatch");
    auto c=config_;c.fx=info.k[0];c.fy=info.k[4];c.cx=info.k[2];c.cy=info.k[5];
    astribot::vision::DepthView v{image.data.data(),image.data.size(),image.width,image.height,image.step,floating,bool(image.is_bigendian)};
    const auto n=astribot::vision::output_points(v,c);
    sensor_msgs::msg::PointCloud2 out;out.header=image.header;out.height=1;out.is_dense=false;
    sensor_msgs::PointCloud2Modifier modifier(out);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(n);
    auto* output=reinterpret_cast<float*>(out.data.data());
    if(using_fallback_)cpu_fallback_->project(v,c,output);
    else if(process_)process_->project_with_context(v,c,output,{j.generation,rclcpp::Time(image.header.stamp).nanoseconds()});
    else projector_->project(v,c,output);
    return out;
  }
  void run(){
    for(;;){
      Job job;
      {std::unique_lock<std::mutex>lock(mutex_);ready_.wait(lock,[this]{return stopping_||pending_.has_value();});if(stopping_)return;
        job=std::move(*pending_);pending_.reset();if(!valid_locked(job))continue;stats_.max_queue_ms=std::max(stats_.max_queue_ms,age(job.received)*1000);
        latency_[1].add_ms(age(job.paired)*1000);}
      const auto begin=Steady::now();
      try {
        auto cloud=compute(job);
        const double compute_ms=age(begin)*1000;
        std::lock_guard<std::mutex>lock(mutex_);stats_.max_compute_ms=std::max(stats_.max_compute_ms,compute_ms);latency_[2].add_ms(compute_ms);
        if(stopping_||!valid_locked(job))continue;
        // Linearize publication with epoch/session invalidation. Pixel work never holds this lock.
        const auto publish_started=Steady::now();points_pub_->publish(std::move(cloud));latency_[3].add_ms(age(publish_started)*1000);
        if(last_output_==0||output_generation_!=generation_)epoch_first_capture_=rclcpp::Time(job.image->header.stamp).nanoseconds();
        last_output_=rclcpp::Time(job.image->header.stamp).nanoseconds();published_stamp_=now().nanoseconds();
        ++stats_.processed;latency_[4].add_ms(age(job.received)*1000);
        output_generation_=generation_;output_received_=job.received;output_frame_=job.image->header.frame_id;
        processing_fault_=false;failure_reason_.clear();
        stats_.max_total_ms=std::max(stats_.max_total_ms,age(job.received)*1000);
      }catch(const std::exception& e){std::lock_guard<std::mutex>lock(mutex_);++stats_.errors;
        processing_fault_locked(e.what());
        if(cpu_fallback_&&process_&&!using_fallback_&&!stopping_) {
          const auto status=process_->status();
          // Never overlap an unconfirmed old GPU worker or retry the old frame.
          if(status.pid<0&&(status.state=="BACKOFF"||status.state=="QUARANTINED")) {
            // The backend transition is a distinct boundary even if the
            // health timer already latched an earlier output-stale fault.
            invalidate_locked();recovery_after_=last_output_;
            using_fallback_=true;RCLCPP_WARN(get_logger(),"RGBD_EXPLICIT_CPU_FALLBACK: %s; waiting for a new pair",e.what());
          }
        }
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"RGBD_PROCESSING_FAILED: %s",e.what());}
    }
  }
  std::string depth_topic_,info_topic_,output_topic_;double max_age_{};
  astribot::vision::ProjectionConfig config_{};std::unique_ptr<astribot::vision::DepthProjector>projector_;
  astribot::vision::ProjectionProcess* process_{};
  std::unique_ptr<astribot::vision::DepthProjector> cpu_fallback_;bool using_fallback_=false;
  std::string backend_,camera_id_,boot_,output_frame_,failure_reason_;
  uint64_t health_sequence_=0,output_generation_=0;int64_t recovery_after_=0;bool processing_fault_=false;
  int64_t epoch_first_capture_=0,published_stamp_=0;
  Steady::time_point output_received_;
  std::mutex mutex_;std::condition_variable ready_;std::thread worker_;bool stopping_=false,session_required_=false;
  uint64_t generation_=0;int64_t last_clock_=0,last_output_=0,last_scheduled_=0;Statistics stats_;
  std::array<astribot::vision::LatencyHistogram,5> latency_{};
  Cache<Info::ConstSharedPtr>infos_;Cache<Image::ConstSharedPtr>depths_;Info::ConstSharedPtr context_;
  std::optional<Job>pending_;Session::ConstSharedPtr session_;Steady::time_point session_received_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;rclcpp::Subscription<Info>::SharedPtr info_sub_;
  rclcpp::Subscription<Session>::SharedPtr session_sub_;rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_;
  rclcpp::Publisher<Health>::SharedPtr health_pub_;rclcpp::TimerBase::SharedPtr health_timer_;
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<astribot_s1_autonomy::RgbdPointcloudNode>(rclcpp::NodeOptions{}));rclcpp::shutdown();return 0;}
