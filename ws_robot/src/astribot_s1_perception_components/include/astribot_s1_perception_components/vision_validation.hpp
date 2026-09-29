#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include "astribot_perception_msgs/msg/camera_health.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "astribot_perception_msgs/msg/detection2_d.hpp"

namespace astribot::vision {
inline bool same_calibration(const sensor_msgs::msg::CameraInfo & a,const sensor_msgs::msg::CameraInfo & b) {
  return a.header.frame_id==b.header.frame_id && a.width==b.width && a.height==b.height &&
    a.k==b.k && a.d==b.d && a.r==b.r && a.p==b.p && a.distortion_model==b.distortion_model &&
    a.binning_x==b.binning_x && a.binning_y==b.binning_y && a.roi==b.roi;
}
inline int64_t ns(const builtin_interfaces::msg::Time & t) {
  if (t.sec < 0 || t.nanosec >= 1000000000u) return 0;
  return int64_t(t.sec)*1000000000LL + t.nanosec;
}
inline bool fresh(const builtin_interfaces::msg::Time & t, int64_t now, double limit) {
  const auto n = ns(t);
  return n > 0 && now >= n && double(now-n)*1e-9 <= limit;
}
inline bool image_layout(const sensor_msgs::msg::Image & image, size_t bytes) {
  return image.width > 0 && image.height > 0 && image.width <= 8192 && image.height <= 8192 &&
    size_t(image.step) >= size_t(image.width)*bytes &&
    uint64_t(image.step)*image.height <= image.data.size() && !image.is_bigendian;
}
inline std::string detection_error(const astribot_perception_msgs::msg::Detection2D & d,
  const astribot_perception_msgs::msg::CameraHealth * h, int64_t now, double age=.30,
  double health_age=.50, double confidence=.25)
{
  if (!fresh(d.header.stamp,now,age)) return "DETECTION_STALE";
  if (d.detection_id.empty() || d.object_id.empty() || d.source_camera_id.empty() ||
      d.source_epoch.empty() || d.model_name.empty() || d.model_revision.empty() ||
      d.header.frame_id.empty()) return "DETECTION_METADATA_MISSING";
  if (!std::isfinite(d.confidence) || d.confidence < confidence || d.confidence > 1.)
    return "CONFIDENCE_INVALID";
  if (d.x_offset < 0 || d.y_offset < 0 || d.width <= 0 || d.height <= 0 ||
      int64_t(d.x_offset)+d.width > 8192 || int64_t(d.y_offset)+d.height > 8192)
    return "DETECTION_BOX_INVALID";
  if (!h || !h->valid || !fresh(h->header.stamp,now,health_age) ||
      !fresh(h->capture_stamp,now,health_age) || ns(h->valid_until) <= now)
    return "CAMERA_HEALTH_INVALID";
  if (h->camera_id != d.source_camera_id || h->source_epoch != d.source_epoch)
    return "CAMERA_SOURCE_MISMATCH";
  if (h->frame_id.empty() || h->frame_id != d.header.frame_id ||
      h->header.frame_id != d.header.frame_id) return "CAMERA_FRAME_MISMATCH";
  if (!d.calibration_revision || h->calibration_revision != d.calibration_revision)
    return "CALIBRATION_REVISION_MISMATCH";
  const auto & m = d.mask;
  const bool supplied = !m.data.empty() || m.width || m.height || !m.encoding.empty();
  if (supplied && ((m.encoding != "mono8" && m.encoding != "8UC1") || !image_layout(m,1)))
    return "MASK_LAYOUT_INVALID";
  return {};
}
}  // namespace astribot::vision
