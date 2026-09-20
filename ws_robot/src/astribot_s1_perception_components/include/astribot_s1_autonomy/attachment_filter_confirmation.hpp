#ifndef ASTRIBOT_S1_AUTONOMY__ATTACHMENT_FILTER_CONFIRMATION_HPP_
#define ASTRIBOT_S1_AUTONOMY__ATTACHMENT_FILTER_CONFIRMATION_HPP_

#include <cmath>
#include <limits>
#include <string>

namespace astribot_s1_autonomy
{
// A configuration heartbeat, never a new sensor observation. Its lease remains
// bounded by the acquisition time of the last successfully filtered cloud.
class AttachmentFilterConfirmation
{
public:
  void invalidate() {revision_.clear();}
  void applied(const std::string & revision, double cloud_source)
  {revision_ = revision; cloud_source_ = cloud_source;}

  bool current(const std::string & snapshot_revision, double snapshot_source,
    bool snapshot_valid, double changed_source, double source_now, double wall_now,
    double max_cloud_age)
  {
    if (!std::isfinite(source_now) || !std::isfinite(wall_now)) {
      invalidate(); return false;
    }
    if (source_now < last_clock_) {invalidate();}
    if (source_now != last_clock_) {last_progress_wall_ = wall_now;}
    last_clock_ = source_now;
    const double cloud_age = source_now - cloud_source_;
    const double snapshot_age = source_now - snapshot_source;
    if (!snapshot_valid || revision_.empty() || revision_ != snapshot_revision ||
      !std::isfinite(cloud_age) || !std::isfinite(snapshot_age) ||
      !std::isfinite(changed_source) || !std::isfinite(max_cloud_age) || max_cloud_age <= 0. ||
      cloud_source_ < changed_source || cloud_age < 0. || cloud_age > max_cloud_age ||
      snapshot_age < 0. || snapshot_age > .5 ||
      wall_now < last_progress_wall_ || wall_now - last_progress_wall_ > .5)
    {
      invalidate(); return false;
    }
    return true;
  }

private:
  std::string revision_;
  double cloud_source_{0.};
  double last_clock_{-std::numeric_limits<double>::infinity()};
  double last_progress_wall_{0.};
};
}  // namespace astribot_s1_autonomy
#endif
