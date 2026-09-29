#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace astribot_s1_perception_native::map_odom {
class DecompositionError : public std::invalid_argument {
 public: using std::invalid_argument::invalid_argument;
};
inline double wrapAngle(double theta) {
  if (std::isinf(theta)) throw std::domain_error("infinite angle");
  return std::atan2(std::sin(theta),std::cos(theta));
}
inline double yawFromQuaternion(double z,double w) { return wrapAngle(2.0*std::atan2(z,w)); }
inline std::pair<double,double> quaternionFromYaw(double theta) {
  if (std::isinf(theta)) throw std::domain_error("infinite angle");
  const double half=0.5*theta;
  return {std::sin(half),std::cos(half)};
}
struct Pose2D {
  double x,y,theta;
  Pose2D(double x_=0,double y_=0,double theta_=0) : x(x_),y(y_),theta(theta_) {
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(theta))
      throw DecompositionError("Pose2D x/y/theta must be finite");
  }
  Pose2D inverse() const {
    const double c=std::cos(theta),s=std::sin(theta);
    return Pose2D(-(c*x+s*y),-(-s*x+c*y),wrapAngle(-theta));
  }
  Pose2D compose(const Pose2D &other) const {
    const double c=std::cos(theta),s=std::sin(theta);
    return Pose2D(x+c*other.x-s*other.y,y+s*other.x+c*other.y,
                  wrapAngle(theta+other.theta));
  }
  double distanceTo(const Pose2D &other) const { return std::hypot(x-other.x,y-other.y); }
};
enum class AgeReason { Fresh, Unstamped, Future, Stale };
struct SourceAge { double age_sec; bool stale; AgeReason reason; };
inline SourceAge checkSourceAge(double now,double stamp,double max_age) {
  if (!std::isfinite(now)||!std::isfinite(stamp))
    throw DecompositionError("source age requires finite now and stamp");
  // Preserve the explicit escape hatch, including zero/future stamps. NaN
  // max_age is intentionally not tightened here: that is legacy policy.
  if(max_age<=0) return {0,false,AgeReason::Fresh};
  if(stamp<=0) return {std::numeric_limits<double>::infinity(),true,AgeReason::Unstamped};
  const double age=now-stamp;
  if(age<-.05) return {age,true,AgeReason::Future};
  if(age>max_age) return {age,true,AgeReason::Stale};
  return {age,false,AgeReason::Fresh};
}
struct Stats { uint64_t updates{0},jumps{0}; double max_jump_m{0}; uint64_t rejected_tilt{0}; };
struct Update { Pose2D pose; bool jumped; double jump_m; };
class Decomposer {
 public:
  explicit Decomposer(double jump_report=.30,double max_tilt=.10)
  : jump_report_(jump_report),max_tilt_(max_tilt) {
    if(!(jump_report>0)) throw DecompositionError("jump_report_m must be positive");
  }
  Update update(const Pose2D &map_base,const Pose2D &odom_base) {
    auto result=map_base.compose(odom_base.inverse());
    bool jumped=false; double jump=0;
    if(last_) {
      jump=last_->distanceTo(result);
      jumped=jump>jump_report_;
      if(jumped) { ++stats.jumps; stats.max_jump_m=std::max(stats.max_jump_m,jump); }
    }
    last_=result; ++stats.updates;
    return {result,jumped,jump};
  }
  bool checkPlanar(double qx,double qy) {
    if(std::hypot(qx,qy)>max_tilt_) { ++stats.rejected_tilt; return true; }
    return false;
  }
  const std::optional<Pose2D> &last() const { return last_; }
  Stats stats;
 private:
  double jump_report_,max_tilt_;
  std::optional<Pose2D> last_;
};
}  // namespace astribot_s1_perception_native::map_odom
