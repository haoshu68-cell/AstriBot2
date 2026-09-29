#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <optional>
#include <memory>
#include <vector>
#include <opencv2/calib3d.hpp>
#include "astribot_s1_perception_components/vision_validation.hpp"
#include "astribot_perception_msgs/msg/object_pose_observation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"

// Single-threaded ROS executor; history is bounded and keyed by capture time.
class RgbdObjectPoseNode final : public rclcpp::Node {
  using Detection = astribot_perception_msgs::msg::Detection2D;
  using Health = astribot_perception_msgs::msg::CameraHealth;
  using Observation = astribot_perception_msgs::msg::ObjectPoseObservation;
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  using Steady=std::chrono::steady_clock;
  template<class T> struct Sample {std::shared_ptr<const T> value; Steady::time_point received;};
public:
  explicit RgbdObjectPoseNode(const rclcpp::NodeOptions & options=rclcpp::NodeOptions()) : Node("rgbd_object_pose",options) {
    age_ = declare_parameter("max_detection_age_sec", .30);
    depth_age_ = declare_parameter("max_depth_age_sec", .20);
    health_age_ = declare_parameter("max_health_age_sec", .50);
    skew_ = declare_parameter("max_sync_skew_sec", .03);
    confidence_ = declare_parameter("min_confidence", .25);
    max_depth_ = declare_parameter("max_depth_m", 5.0);
    validity_ = declare_parameter("output_validity_sec", .30);
    samples_ = declare_parameter("min_samples", 12);
    const int capacity = declare_parameter("history_size", 12);
    if (!declare_parameter("require_camera_health", true) || capacity<1 || capacity>30 || samples_<1 ||
        !std::isfinite(age_) || age_<=0 || !std::isfinite(depth_age_) || depth_age_<=0 ||
        !std::isfinite(health_age_) || health_age_<=0 || !std::isfinite(skew_) || skew_<0 ||
        !std::isfinite(max_depth_) || max_depth_<=.05 || !std::isfinite(validity_) || validity_<=0 ||
        !std::isfinite(confidence_) || confidence_<0 || confidence_>1) throw std::invalid_argument("invalid vision thresholds");
    capacity_ = size_t(capacity);
    const auto output = declare_parameter<std::string>("output_topic", "/perception/object_pose");
    pub_ = create_publisher<Observation>(output, rclcpp::QoS(1));
    status_ = create_publisher<std_msgs::msg::String>(output+"/status", rclcpp::QoS(1));
    depth_sub_ = create_subscription<Image>(declare_parameter<std::string>("depth_topic","/camera/depth/image_raw"),
      rclcpp::SensorDataQoS().keep_last(3), [this](Image::ConstSharedPtr p) {
        check_clock();
        if ((p->encoding!="32FC1" && p->encoding!="16UC1") ||
            !astribot::vision::image_layout(*p,p->encoding=="32FC1" ? 4:2)) { reject("DEPTH_LAYOUT_INVALID"); return; }
        push(depths_,p);
      });
    info_sub_ = create_subscription<Info>(declare_parameter<std::string>("camera_info_topic","/camera/color/camera_info"),
      rclcpp::SensorDataQoS(), [this](Info::ConstSharedPtr p) {
        check_clock();
        if(!astribot::vision::fresh(p->header.stamp,now().nanoseconds(),depth_age_) ||
           p->header.frame_id.empty() || !p->width || !p->height || p->width>8192 || p->height>8192 ||
           !std::all_of(p->k.begin(),p->k.end(),[](double v){return std::isfinite(v);}) ||
           p->k[0]<=0 || p->k[4]<=0 || p->k[8]!=1 ||
           !std::all_of(p->d.begin(),p->d.end(),[](double v){return std::isfinite(v);})) {
          reject("INTRINSICS_INVALID");return;
        }
        if(!infos_.empty() && astribot::vision::ns(p->header.stamp)<=astribot::vision::ns(infos_.back().value->header.stamp)) return;
        if(baseline_info_ && !astribot::vision::same_calibration(*baseline_info_,*p)) {
          calibration_changed_=true;infos_.clear();reject("CALIBRATION_CHANGED_WITHOUT_REVISION");return;
        }
        if(!baseline_info_) baseline_info_=*p;
        push(infos_,p);
      });
    health_sub_ = create_subscription<Health>(declare_parameter<std::string>("health_topic","/perception/camera_health/head_rgbd"),
      rclcpp::QoS(1).transient_local(), [this](Health::ConstSharedPtr p) {
        check_clock();
        if (health_ && (health_->source_epoch!=p->source_epoch || health_->calibration_revision!=p->calibration_revision)) {
          depths_.clear(); infos_.clear(); baseline_info_.reset(); calibration_changed_=false;
        }
        health_=p; health_received_=Steady::now();
      });
    det_sub_ = create_subscription<Detection>(declare_parameter<std::string>("detection_topic","/perception/detections"),
      rclcpp::QoS(3), [this](Detection::ConstSharedPtr d) { observe(*d); });
  }
private:
  void check_clock() {
    const auto t=now().nanoseconds();
    if(t<last_time_) { depths_.clear(); infos_.clear(); health_.reset(); baseline_info_.reset(); calibration_changed_=false; }
    last_time_=t;
  }
  template<class T> void push(std::deque<Sample<T>> & queue, std::shared_ptr<const T> p) {
    using astribot::vision::ns;
    if(ns(p->header.stamp)==0) return;
    if(!queue.empty() && ns(p->header.stamp)<=ns(queue.back().value->header.stamp)) return;
    queue.push_back({p,Steady::now()}); if(queue.size()>capacity_) queue.pop_front();
  }
  template<class T> const Sample<T> * match(const std::deque<Sample<T>> & queue,int64_t stamp) {
    const Sample<T> * best=nullptr; int64_t distance=INT64_MAX;
    for(const auto & sample:queue) {
      if(std::chrono::duration<double>(Steady::now()-sample.received).count()>depth_age_) continue;
      const auto & p=sample.value;
      const auto delta=std::abs(astribot::vision::ns(p->header.stamp)-stamp);
      if(delta<distance && double(delta)*1e-9<=skew_) { best=&sample; distance=delta; }
    }
    return best;
  }
  void reject(const std::string & reason) { std_msgs::msg::String s; s.data=reason; status_->publish(s); }
  void observe(const Detection & d) {
    using namespace astribot::vision;
    check_clock(); const int64_t t=now().nanoseconds();
    if(calibration_changed_) { reject("CALIBRATION_CHANGED_WITHOUT_REVISION");return; }
    if(std::chrono::duration<double>(Steady::now()-health_received_).count()>health_age_) { reject("CAMERA_HEALTH_RECEIVE_TIMEOUT");return; }
    const auto error=detection_error(d,health_.get(),t,age_,health_age_,confidence_);
    if(!error.empty()) { reject(error); return; }
    const auto depth_sample=match(depths_,ns(d.header.stamp));
    const auto info_sample=match(infos_,ns(d.header.stamp));
    if(!depth_sample || !info_sample) { reject("RGBD_FRAME_NOT_FOUND"); return; }
    const auto & depth=depth_sample->value;
    const auto & info=info_sample->value;
    if(!fresh(depth->header.stamp,t,depth_age_) || !fresh(info->header.stamp,t,depth_age_) ||
       double(std::abs(ns(depth->header.stamp)-ns(info->header.stamp)))*1e-9>skew_) { reject("RGBD_STALE"); return; }
    if(depth->header.frame_id!=d.header.frame_id || info->header.frame_id!=d.header.frame_id ||
       info->width!=depth->width || info->height!=depth->height) { reject("RGBD_FRAME_OR_SIZE_MISMATCH"); return; }
    if(int64_t(d.x_offset)+d.width>depth->width || int64_t(d.y_offset)+d.height>depth->height) { reject("BOX_OUTSIDE_IMAGE"); return; }
    if(!std::all_of(info->k.begin(),info->k.end(),[](double v){return std::isfinite(v);}) ||
       info->k[0]<=0 || info->k[4]<=0 || info->k[8]!=1 ||
       !std::all_of(info->d.begin(),info->d.end(),[](double v){return std::isfinite(v);})) { reject("INTRINSICS_INVALID"); return; }
    // Accept pinhole or OpenCV plumb-bob/rational distortion; no silent fisheye fallback.
    const bool distorted=std::any_of(info->d.begin(),info->d.end(),[](double v){return v!=0;});
    if(distorted && ((info->distortion_model!="plumb_bob" && info->distortion_model!="rational_polynomial") ||
       (info->d.size()!=4 && info->d.size()!=5 && info->d.size()!=8))) { reject("DISTORTION_UNSUPPORTED"); return; }
    const bool masked=!d.mask.data.empty();
    const bool full=masked && d.mask.width==depth->width && d.mask.height==depth->height;
    if(masked && !full && (d.mask.width!=uint32_t(d.width) || d.mask.height!=uint32_t(d.height))) { reject("MASK_SIZE_MISMATCH"); return; }
    std::vector<cv::Point2d> pixels; std::vector<double> zs;
    for(int y=d.y_offset;y<d.y_offset+d.height;++y) for(int x=d.x_offset;x<d.x_offset+d.width;++x) {
      if(masked && d.mask.data[size_t(full?y:y-d.y_offset)*d.mask.step+(full?x:x-d.x_offset)]==0) continue;
      double z;
      if(depth->encoding=="32FC1") { float v; std::memcpy(&v,depth->data.data()+size_t(y)*depth->step+size_t(x)*4,4); z=v; }
      else { uint16_t v; std::memcpy(&v,depth->data.data()+size_t(y)*depth->step+size_t(x)*2,2); z=v*.001; }
      if(std::isfinite(z) && z>.05 && z<=max_depth_) { pixels.emplace_back(x,y); zs.push_back(z); }
    }
    if(zs.size()<size_t(samples_)) { reject("DEPTH_SAMPLES_INSUFFICIENT"); return; }
    std::vector<cv::Point2d> rays;
    try {
      cv::Mat k(3,3,CV_64F,const_cast<double *>(info->k.data()));
      cv::undistortPoints(pixels,rays,k,info->d);
    } catch(const cv::Exception &) { reject("PROJECTION_FAILED"); return; }
    if(rays.size()!=zs.size() || !std::all_of(rays.begin(),rays.end(),[](const auto & r){return std::isfinite(r.x) && std::isfinite(r.y);})) {
      reject("PROJECTION_NONFINITE");return;
    }
    // Visible support centroid. It is NOT the CAD origin, object orientation or a grasp.
    double x=0,y=0,z=0;
    for(size_t i=0;i<zs.size();++i) { x+=rays[i].x*zs[i]; y+=rays[i].y*zs[i]; z+=zs[i]; }
    const double n=double(zs.size()); x/=n; y/=n; z/=n;
    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {reject("POSITION_NONFINITE");return;}
    Observation o; o.header=d.header; o.object_id=d.object_id; o.source_camera_id=d.source_camera_id;
    o.source_epoch=d.source_epoch; o.source_model=d.model_name; o.model_revision=d.model_revision;
    o.calibration_revision=d.calibration_revision; o.position_valid=true; o.orientation_valid=false;
    o.reason_code=masked?"VISIBLE_SURFACE_CENTROID_MASK":"VISIBLE_SURFACE_CENTROID_BBOX";
    o.quality=d.confidence*std::min(1.,n/100.);
    const auto deadline=std::min({ns(d.header.stamp)+int64_t(validity_*1e9),
      ns(depth->header.stamp)+int64_t(depth_age_*1e9),ns(health_->valid_until)});
    if(now().nanoseconds()>=deadline) { reject("OBSERVATION_EXPIRED_DURING_PROJECTION"); return; }
    o.valid_until=rclcpp::Time(deadline,RCL_ROS_TIME);
    o.pose.pose.position.x=x; o.pose.pose.position.y=y; o.pose.pose.position.z=z;
    o.pose.pose.orientation.w=1.;
    for(size_t i=0;i<zs.size();++i) {
      const double delta[3]={rays[i].x*zs[i]-x,rays[i].y*zs[i]-y,zs[i]-z};
      for(size_t a=0;a<3;++a) for(size_t b=0;b<3;++b) o.pose.covariance[a*6+b]+=delta[a]*delta[b]/n;
    }
    for(size_t a=0;a<3;++a) o.pose.covariance[a*7]+=1e-4; // conservative, uncalibrated spread floor
    o.pose.covariance[21]=o.pose.covariance[28]=o.pose.covariance[35]=1e6; // orientation unknown
    if(!std::all_of(o.pose.covariance.begin(),o.pose.covariance.end(),[](double v){return std::isfinite(v);})) {
      reject("COVARIANCE_NONFINITE");return;
    }
    // Recheck at the publication boundary: projection and covariance can be expensive.
    const auto completed=Steady::now(); const auto completed_ros=now().nanoseconds();
    if(completed_ros<t || completed_ros>=deadline ||
       std::chrono::duration<double>(completed-depth_sample->received).count()>depth_age_ ||
       std::chrono::duration<double>(completed-info_sample->received).count()>depth_age_ ||
       std::chrono::duration<double>(completed-health_received_).count()>health_age_) {
      reject("OBSERVATION_EXPIRED_DURING_PROJECTION");return;
    }
    pub_->publish(o); reject("POSITION_ONLY_OBSERVATION");
  }
  double age_,depth_age_,health_age_,skew_,confidence_,max_depth_,validity_; int samples_;
  size_t capacity_; int64_t last_time_{0};
  std::deque<Sample<Image>> depths_; std::deque<Sample<Info>> infos_;
  Steady::time_point health_received_; std::optional<Info> baseline_info_; bool calibration_changed_{false};
  Health::ConstSharedPtr health_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<Info>::SharedPtr info_sub_;
  rclcpp::Subscription<Health>::SharedPtr health_sub_;
  rclcpp::Subscription<Detection>::SharedPtr det_sub_;
  rclcpp::Publisher<Observation>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<RgbdObjectPoseNode>()); }
  catch(const std::exception & e) { fprintf(stderr,"rgbd_object_pose: %s\n",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
