#include <chrono>
#include <mutex>
#include <optional>
#include "astribot_s1_perception_components/yolo_dnn.hpp"
#include "astribot_s1_perception_components/vision_validation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// The receiver keeps a single latest frame. Inference runs in a separate callback
// group so health updates are not blocked by DNN. This node never commands motion.
class YoloDetectorNode final : public rclcpp::Node {
  using Image = sensor_msgs::msg::Image;
  using Health = astribot_perception_msgs::msg::CameraHealth;
  using Detection = astribot_perception_msgs::msg::Detection2D;
  using Steady = std::chrono::steady_clock;
  struct Pending { Image::ConstSharedPtr image; Health context; Steady::time_point received; uint64_t generation; };
public:
  explicit YoloDetectorNode(const rclcpp::NodeOptions & options=rclcpp::NodeOptions()) : Node("yolo_detector",options) {
    camera_=declare_parameter<std::string>("camera_id","head_rgbd");
    revision_=declare_parameter<std::string>("model_revision","");
    layout_=declare_parameter<std::string>("model_layout","yolov5");
    max_age_=declare_parameter("max_image_age_sec",.20);
    health_age_=declare_parameter("max_health_age_sec",.50);
    confidence_=declare_parameter("min_confidence",.25);
    iou_=declare_parameter("nms_iou",.45);
    maximum_=declare_parameter("max_detections",100);
    const int size=declare_parameter("input_size",640);
    const int threads=declare_parameter("cpu_threads",2);
    if(camera_.empty() || revision_.empty() || !std::isfinite(max_age_) || max_age_<=0 ||
       !std::isfinite(health_age_) || health_age_<=0 || !std::isfinite(confidence_) || confidence_<0 || confidence_>1 ||
       !std::isfinite(iou_) || iou_<=0 || iou_>1 || maximum_<1 || maximum_>1000 || threads<1 || threads>16)
      throw std::invalid_argument("invalid YOLO metadata or inference budget");
    labels_=astribot::vision::load_labels(declare_parameter<std::string>("labels_path",""));
    cv::setNumThreads(threads);
    model_=std::make_unique<astribot::vision::YoloDnn>(declare_parameter<std::string>("model_path",""),int(labels_.size()),layout_,size);
    // Warm-up validates graph/output compatibility before subscriptions are started.
    model_->infer(cv::Mat(size,size,CV_8UC3,cv::Scalar(114,114,114)),confidence_,iou_,maximum_);
    const auto topic=declare_parameter<std::string>("output_topic","/perception/detections/head_rgbd");
    output_=create_publisher<Detection>(topic,rclcpp::QoS(10));
    status_=create_publisher<std_msgs::msg::String>(topic+"/status",rclcpp::QoS(1));
    receivers_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    inference_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts; opts.callback_group=receivers_;
    health_sub_=create_subscription<Health>(declare_parameter<std::string>("health_topic","/perception/camera_health/head_rgbd"),
      rclcpp::QoS(1).transient_local(),[this](Health::ConstSharedPtr h) {
        std::lock_guard<std::mutex> lock(mutex_); check_clock();
        if(!health_ || h->source_epoch!=health_->source_epoch || h->calibration_revision!=health_->calibration_revision || !h->valid) {
          ++generation_; pending_.reset(); last_stamp_=0;
        }
        health_=h; health_received_=Steady::now();
      },opts);
    image_sub_=create_subscription<Image>(declare_parameter<std::string>("color_topic","/camera/color/image_raw"),
      rclcpp::SensorDataQoS().keep_last(1),[this](Image::ConstSharedPtr p) {
        using namespace astribot::vision;
        std::lock_guard<std::mutex> lock(mutex_); check_clock();
        if((p->encoding!="rgb8" && p->encoding!="bgr8") || !image_layout(*p,3)) { report("IMAGE_LAYOUT_INVALID");return; }
        if(ns(p->header.stamp)<=last_stamp_) { report("IMAGE_REPLAYED");return; }
        if(!fresh(p->header.stamp,now().nanoseconds(),max_age_) || !health_valid(*p)) { report("IMAGE_OR_HEALTH_INVALID");return; }
        last_stamp_=ns(p->header.stamp);
        pending_=Pending{p,*health_,Steady::now(),generation_};
      },opts);
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this](){ infer(); },inference_);
  }
private:
  void report(const std::string & reason) { std_msgs::msg::String m;m.data=reason;status_->publish(m); }
  void check_clock() {
    const auto t=now().nanoseconds();
    if(t<last_now_) { ++generation_;pending_.reset();health_.reset();last_stamp_=0; }
    last_now_=t;
  }
  bool health_valid(const Image & p) const {
    using namespace astribot::vision;
    const auto t=now().nanoseconds();
    return health_ && health_->valid && health_->camera_id==camera_ && !health_->source_epoch.empty() &&
      health_->calibration_revision>0 && health_->frame_id==p.header.frame_id && health_->header.frame_id==p.header.frame_id &&
      fresh(health_->header.stamp,t,health_age_) && fresh(health_->capture_stamp,t,health_age_) && ns(health_->valid_until)>t &&
      std::chrono::duration<double>(Steady::now()-health_received_).count()<=health_age_;
  }
  void infer() {
    std::optional<Pending> work;
    { std::lock_guard<std::mutex> lock(mutex_);check_clock();work=std::move(pending_);pending_.reset(); }
    if(!work) return;
    const auto & p=*work->image;
    try {
      cv::Mat rgb(p.height,p.width,CV_8UC3,const_cast<uint8_t *>(p.data.data()),p.step),bgr;
      if(p.encoding=="rgb8") cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR); else bgr=rgb;
      const auto boxes=model_->infer(bgr,confidence_,iou_,maximum_);
      std::lock_guard<std::mutex> lock(mutex_);check_clock();
      if(work->generation!=generation_ || !health_valid(p) ||
         !astribot::vision::fresh(p.header.stamp,now().nanoseconds(),max_age_) ||
         std::chrono::duration<double>(Steady::now()-work->received).count()>max_age_) { report("RESULT_EXPIRED_OR_CONTEXT_CHANGED");return; }
      size_t index=0;
      for(const auto & b:boxes) {
        Detection d;d.header=p.header;d.source_camera_id=camera_;d.source_epoch=work->context.source_epoch;
        d.calibration_revision=work->context.calibration_revision;d.model_name="yolo_dnn_"+layout_;d.model_revision=revision_;
        d.detection_id=camera_+":"+d.source_epoch+":"+std::to_string(astribot::vision::ns(p.header.stamp))+":"+std::to_string(index++);
        // Per-frame hypothesis ID; persistent world identity belongs to a tracker/world model.
        d.object_id=d.detection_id;d.class_id=b.class_id;d.class_name=labels_.at(size_t(b.class_id));d.confidence=b.score;
        d.x_offset=b.box.x;d.y_offset=b.box.y;d.width=b.box.width;d.height=b.box.height;
        output_->publish(d);
      }
      report(boxes.empty()?"NO_DETECTION":"DETECTIONS_PUBLISHED");
    } catch(const std::exception & e) { report(std::string("INFERENCE_FAILED: ")+e.what()); }
  }
  std::string camera_,revision_,layout_;double max_age_,health_age_,confidence_,iou_;int maximum_;
  std::vector<std::string> labels_;std::unique_ptr<astribot::vision::YoloDnn> model_;
  std::mutex mutex_;std::optional<Pending> pending_;Health::ConstSharedPtr health_;
  Steady::time_point health_received_;uint64_t generation_{0};int64_t last_now_{0},last_stamp_{0};
  rclcpp::CallbackGroup::SharedPtr receivers_,inference_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;rclcpp::Subscription<Health>::SharedPtr health_sub_;
  rclcpp::Publisher<Detection>::SharedPtr output_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<YoloDetectorNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node);executor.spin();
  } catch(const std::exception & e) {fprintf(stderr,"yolo_detector: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
