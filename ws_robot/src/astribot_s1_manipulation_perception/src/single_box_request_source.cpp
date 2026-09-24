#include <astribot_s1_manipulation_perception/single_box_request_source.hpp>
#include "single_box_source_detail.hpp"
#include <algorithm>
#include <map>
#include <mutex>

namespace astribot::perception_planning {
struct SingleBoxRequestSource::Impl {
  using Steady=std::chrono::steady_clock;
  using Image=sensor_msgs::msg::Image;
  using Cloud=sensor_msgs::msg::PointCloud2;
  using Info=sensor_msgs::msg::CameraInfo;
  using Camera=astribot_perception_msgs::msg::CameraHealth;
  using Projection=astribot_perception_msgs::msg::ProjectionHealth;
  template<class T>struct Entry {std::shared_ptr<const T> value;SensorReceipt received;};
  template<class T>using Cache=std::map<int64_t,Entry<T>>;
  rclcpp::Node::SharedPtr node;tf2_ros::Buffer& tf;SingleBoxTopics topics;
  std::mutex mutex;Cache<Image> colors;Cache<Cloud> clouds;Cache<Info> infos;
  Camera::ConstSharedPtr camera;Projection::ConstSharedPtr projection;
  SensorReceipt camera_received,camera_capture_received,projection_received,projection_capture_received;
  std::string info_revision,error;uint64_t generation{},owner_clock_epoch{};
  int64_t last_clock{},source_floor{};bool calibration_fault{},clock_fault{};
  int64_t color_highwater{},cloud_highwater{},info_highwater{};
  rclcpp::Subscription<Image>::SharedPtr color_sub;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub;
  rclcpp::Subscription<Info>::SharedPtr info_sub;
  rclcpp::Subscription<Camera>::SharedPtr camera_sub;
  rclcpp::Subscription<Projection>::SharedPtr projection_sub;

  Impl(rclcpp::Node::SharedPtr n,tf2_ros::Buffer& buffer,SingleBoxTopics t):node(std::move(n)),tf(buffer),topics(std::move(t)) {
    using detail::require;
    require(!topics.camera_id.empty()&&!topics.color.empty()&&!topics.info.empty()&&!topics.points.empty()&&
      !topics.camera_health.empty()&&!topics.projection_health.empty(),"SENSOR_TOPICS_REQUIRED");
    const auto sensor_qos=rclcpp::SensorDataQoS().keep_last(4);
    color_sub=node->create_subscription<Image>(topics.color,sensor_qos,[this](Image::ConstSharedPtr p){
      std::lock_guard<std::mutex> lock(mutex);const auto now=clock();
      if(p->data.size()>16777216){fault("RGB_LAYOUT_LIMIT");return;}
      put(colors,std::move(p),now,color_highwater);
    });
    cloud_sub=node->create_subscription<Cloud>(topics.points,sensor_qos,[this](Cloud::ConstSharedPtr p){
      std::lock_guard<std::mutex> lock(mutex);const auto now=clock();
      if(p->data.size()>67108864){fault("CLOUD_LAYOUT_LIMIT");return;}
      put(clouds,std::move(p),now,cloud_highwater);
    });
    info_sub=node->create_subscription<Info>(topics.info,sensor_qos,[this](Info::ConstSharedPtr p){
      std::lock_guard<std::mutex> lock(mutex);const auto now=clock();
      if(p->d.size()>14){fault("CAMERA_INFO_LAYOUT_LIMIT");return;}
      if(!admissible(p->header.stamp,now))return;
      if(detail::stamp(p->header.stamp)<=info_highwater)return;
      const auto revision=camera_info_revision(*p);
      if(!info_revision.empty()&&revision!=info_revision) {
        calibration_fault=true;fault("CAMERA_INFO_CHANGED_WITHOUT_REVISION");return;
      }
      info_revision=revision;put(infos,std::move(p),now,info_highwater);
    });
    camera_sub=node->create_subscription<Camera>(topics.camera_health,rclcpp::QoS(10).reliable(),[this](Camera::ConstSharedPtr p){
      std::lock_guard<std::mutex> lock(mutex);const auto now=clock();
      if(p->camera_id!=topics.camera_id)return;
      if(!valid_stamp(p->header.stamp)||!valid_stamp(p->capture_stamp)||!valid_stamp(p->valid_until)) {
        camera.reset();fault("INVALID_CAMERA_HEALTH_STAMP");return;
      }
      const auto time=detail::stamp(p->header.stamp);
      if(p->valid&&time>now&&(!camera||(p->source_epoch==camera->source_epoch&&
         p->calibration_revision==camera->calibration_revision&&p->frame_id==camera->frame_id)))return;
      if(camera&&time<detail::stamp(camera->header.stamp))return;
      const bool changed=!camera||p->source_epoch!=camera->source_epoch||
        p->calibration_revision!=camera->calibration_revision||p->frame_id!=camera->frame_id;
      if(changed) {
        clear_frames();info_revision.clear();calibration_fault=false;
        color_highwater=cloud_highwater=info_highwater=0;
        source_floor=std::max(time,detail::stamp(p->capture_stamp));
      } else if(!p->valid)clear_frames();
      if(changed||p->capture_stamp!=camera->capture_stamp)camera_capture_received={now,Steady::now()};
      camera=std::move(p);camera_received={now,Steady::now()};
    });
    projection_sub=node->create_subscription<Projection>(topics.projection_health,rclcpp::QoS(1).reliable().transient_local(),[this](Projection::ConstSharedPtr p){
      std::lock_guard<std::mutex> lock(mutex);const auto now=clock();
      if(p->camera_id!=topics.camera_id)return;
      if(!valid_stamp(p->header.stamp)||!valid_stamp(p->capture_stamp)||!valid_stamp(p->epoch_first_capture_stamp)||
         !valid_stamp(p->valid_until)) {projection.reset();fault("INVALID_PROJECTION_STAMP");return;}
      const auto time=detail::stamp(p->header.stamp);
      if(p->valid&&time>now&&(!projection||(p->processing_epoch==projection->processing_epoch&&
         p->header.frame_id==projection->header.frame_id)))return;
      if(projection&&(time<detail::stamp(projection->header.stamp)||
         (p->processing_epoch==projection->processing_epoch&&p->sequence<=projection->sequence)))return;
      const bool changed=!projection||p->processing_epoch!=projection->processing_epoch||
        p->header.frame_id!=projection->header.frame_id;
      if(changed||!p->valid)clear_frames();
      if(changed||p->capture_stamp!=projection->capture_stamp)projection_capture_received={now,Steady::now()};
      projection=std::move(p);projection_received={now,Steady::now()};
    });
  }
  static bool valid_stamp(const builtin_interfaces::msg::Time& t) {return t.sec>=0&&t.nanosec<1000000000u;}
  void clear_frames() {colors.clear();clouds.clear();infos.clear();++generation;}
  void fault(const char* reason) {clear_frames();error=reason;RCLCPP_WARN(node->get_logger(),"RGBD_REQUEST_SOURCE: %s",reason);}
  int64_t clock() {
    const auto now=node->now().nanoseconds();
    if(last_clock&&now<last_clock) {
      clear_frames();camera.reset();projection.reset();info_revision.clear();calibration_fault=false;
      color_highwater=cloud_highwater=info_highwater=0;
      clock_fault=true;source_floor=now;error="CLOCK_ROLLBACK";
    }
    last_clock=now;return now;
  }
  bool admissible(const builtin_interfaces::msg::Time& s,int64_t now) {
    if(!valid_stamp(s)){fault("INVALID_SENSOR_STAMP");return false;}
    const auto stamp=detail::stamp(s);
    if(!camera||!camera->valid||stamp<source_floor||stamp<=0||stamp>now||now-stamp>250000000LL) {
      error="SENSOR_SOURCE_OR_CAPTURE_NOT_READY";return false;
    }
    return true;
  }
  template<class T>void put(Cache<T>& cache,std::shared_ptr<const T> p,int64_t now,int64_t& highwater) {
    if(!admissible(p->header.stamp,now)||calibration_fault)return;
    const auto stamp=detail::stamp(p->header.stamp);
    if(stamp<=highwater)return;
    highwater=stamp;
    cache.emplace(stamp,Entry<T>{std::move(p),{now,Steady::now()}});
    while(cache.size()>4)cache.erase(cache.begin());
  }
  RgbdFrame health_frame() const {
    detail::require(camera&&projection,"SENSOR_HEALTH_NOT_READY");
    RgbdFrame f;f.camera=*camera;f.projection=*projection;
    f.camera_received=camera_received;f.camera_capture_received=camera_capture_received;
    f.projection_received=projection_received;f.projection_capture_received=projection_capture_received;
    return f;
  }
  Context bind(Context c,int64_t now) {
    detail::require(!calibration_fault,"CAMERA_INFO_CHANGED_WITHOUT_REVISION");
    detail::require(!infos.empty()&&!info_revision.empty(),"CAMERA_INFO_NOT_READY");
    detail::require(!clock_fault||c.clock_epoch!=owner_clock_epoch,"CLOCK_EPOCH_NOT_ADVANCED");
    clock_fault=false;owner_clock_epoch=c.clock_epoch;
    auto frame=health_frame();
    detail::require(detail::fresh(infos.rbegin()->second.value->header.stamp,infos.rbegin()->second.received,now,250000000LL),
      "CAMERA_INFO_STALE");
    detail::require(c.camera_id.empty()||c.camera_id==topics.camera_id,"CAMERA_SELECTION_MISMATCH");
    c.camera_id=topics.camera_id;c.optical_frame=camera->frame_id;c.source_epoch=camera->source_epoch;
    c.processing_epoch=projection->processing_epoch;c.camera_info_revision=info_revision;
    detail::health(frame,c,now);return c;
  }
};

SingleBoxRequestSource::SingleBoxRequestSource(rclcpp::Node::SharedPtr node,tf2_ros::Buffer& tf,SingleBoxTopics topics)
  :impl_(std::make_unique<Impl>(std::move(node),tf,std::move(topics))) {}
SingleBoxRequestSource::~SingleBoxRequestSource()=default;
Context SingleBoxRequestSource::bind_context(Context context) {
  std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->bind(std::move(context),impl_->clock());
}
Request SingleBoxRequestSource::capture(Request task,const SingleBoxFixture& fixture,const std::function<Context()>& current) {
  detail::require(bool(current),"CURRENT_CONTEXT_PROVIDER_REQUIRED");
  RgbdFrame frame;uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);const auto now=impl_->clock();
    task.context=impl_->bind(std::move(task.context),now);frame=impl_->health_frame();generation=impl_->generation;
    for(auto it=impl_->colors.rbegin();it!=impl_->colors.rend();++it) {
      const auto points=impl_->clouds.find(it->first);
      const auto info=impl_->infos.find(it->first);
      if(points==impl_->clouds.end()||info==impl_->infos.end())continue;
      frame.color=it->second.value;frame.points=points->second.value;frame.info=info->second.value;
      frame.color_received=it->second.received;frame.points_received=points->second.received;frame.info_received=info->second.received;
      break;
    }
    if(!frame.color)throw std::runtime_error("RGBD_EXACT_FRAME_NOT_READY:"+impl_->error);
  }
  // Use BufferCore's immediate lookup, not the timeout overload that assumes a
  // dedicated TF thread even for a zero timeout. Missing data fails explicitly.
  const tf2::TimePoint capture_time{std::chrono::nanoseconds(detail::stamp(frame.color->header.stamp))};
  const auto tf=impl_->tf.lookupTransform("astribot_torso_base",frame.color->header.frame_id,capture_time);
  const auto station_tf=impl_->tf.lookupTransform(fixture.station_frame,frame.color->header.frame_id,capture_time);
  auto result=single_box_request(std::move(task),fixture,frame,tf,station_tf,impl_->node->now().nanoseconds());
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);impl_->clock();
    detail::require(generation==impl_->generation,"SENSOR_CONTEXT_CHANGED_DURING_CAPTURE");
  }
  const auto now=impl_->node->now().nanoseconds();
  detail::require(detail::fresh(frame.color->header.stamp,frame.color_received,now,250000000LL)&&
    detail::fresh(frame.points->header.stamp,frame.points_received,now,250000000LL)&&
    detail::fresh(frame.info->header.stamp,frame.info_received,now,250000000LL),"RGBD_EXPIRED_DURING_CAPTURE");
  check_context(result,current(),impl_->node->now().nanoseconds(),true);
  return result;
}
} // namespace astribot::perception_planning
