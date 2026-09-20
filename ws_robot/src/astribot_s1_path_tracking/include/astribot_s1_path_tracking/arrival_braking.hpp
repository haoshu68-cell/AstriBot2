#ifndef ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_BRAKING_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__ARRIVAL_BRAKING_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace astribot_s1_path_tracking
{
inline double referenceStopDistance(double speed, double acceleration, double jerk)
{
  speed=std::abs(speed);
  return speed<=acceleration*acceleration/jerk ? speed*std::sqrt(speed/jerk) :
    0.5*speed*(speed/acceleration+acceleration/jerk);
}

inline double referenceSpeedForDistance(double distance, double maximum, double acceleration, double jerk)
{
  if (referenceStopDistance(maximum,acceleration,jerk)<=distance) {return maximum;}
  double low=0.,high=maximum;
  for (int i=0;i<40;++i) {
    const double middle=(low+high)*.5;
    if (referenceStopDistance(middle,acceleration,jerk)<=distance) {low=middle;}
    else {high=middle;}
  }
  return low;
}

class SettledOffsetCurve
{
public:
  void configure(std::vector<double> speeds,std::vector<double> offsets)
  {
    if (speeds.size()<2 || speeds.size()!=offsets.size() || speeds[0]!=0 || offsets[0]!=0) {
      throw std::invalid_argument("Stop curve needs equal vectors beginning at zero");
    }
    for (size_t i=0;i<speeds.size();++i) {
      if (!std::isfinite(speeds[i]) || !std::isfinite(offsets[i]) || offsets[i]<0 ||
        (i>0 && speeds[i]<=speeds[i-1])) {throw std::invalid_argument("Invalid stop curve");}
    }
    speeds_=std::move(speeds);offsets_=std::move(offsets);
  }
  double distance(double speed) const
  {
    speed=std::abs(speed);
    if (speed>speeds_.back()) {throw std::out_of_range("STOP_MODEL_SPEED_OUTSIDE_CALIBRATION");}
    for (size_t i=1;i<speeds_.size();++i) {
      if (speed<=speeds_[i]) {
        const double ratio=(speed-speeds_[i-1])/(speeds_[i]-speeds_[i-1]);
        return offsets_[i-1]+ratio*(offsets_[i]-offsets_[i-1]);
      }
    }
    return offsets_.back();
  }
private:
  std::vector<double> speeds_{0.,1.},offsets_{0.,0.};
};

// Reuse the localization stop window before issuing another correction.
template<std::size_t N>
class ArrivalCoast
{
public:
  void reset() {*this = ArrivalCoast();}
  bool active() const {return coasting_;}
  std::array<double, N> apply(const std::array<double, N> & request, bool stopped)
  {
    double requested_sq = 0.0, previous_sq = 0.0;
    double previous_dot = 0.0;
    for (std::size_t i = 0; i < N; ++i) {
      requested_sq += request[i]*request[i];
      previous_sq += previous_[i]*previous_[i];
      previous_dot += request[i]*previous_[i];
    }
    const bool zero = requested_sq <= 1e-18;
    // Coast only after this axis was commanded to move. Drift caused by another
    // axis must remain correctable, rather than waiting for that drift to stop.
    if (!coasting_ && ((zero && previous_sq > 1e-18) || previous_dot < -1e-12))
    {
      coasting_ = true;
    }
    if (coasting_) {
      if (stopped) {coasting_ = false; previous_ = {};}
      // Re-evaluate the current pose next tick, after the stop has been confirmed.
      return {};
    }
    previous_ = request;
    return request;
  }
private:
  std::array<double, N> previous_{};
  bool coasting_{false};
};

inline double applyArrivalYawCoast(ArrivalCoast<1> & coast,double request,
  bool stopped,bool coupled_translation) {
  // During corridor translation, heading disturbance continues even with a
  // zero yaw command. Waiting for a yaw-only stopped window can latch COAST
  // until XY has stopped, when an in-place correction is forbidden.
  if(coupled_translation) {coast.reset();return request;}
  return coast.apply({request},stopped)[0];
}

inline std::array<double, 2> brakingTranslation(
  double x, double y, double vx, double vy, double gain,
  double minimum, double maximum, double horizon)
{
  const double px = x - horizon*vx, py = y - horizon*vy;
  const double remaining = std::hypot(px, py);
  // Brake before the measured motion consumes the remaining error; do not reverse early.
  if (x*px + y*py <= 0.0 || remaining <= 1e-12) {return {0.0, 0.0};}
  const double speed = std::clamp(gain*remaining, minimum, maximum);
  return {speed*px/remaining, speed*py/remaining};
}

inline double brakingRotation(
  double error, double measured, double gain, double minimum, double maximum, double horizon)
{
  const double remaining = error - horizon*measured;
  if (error*remaining <= 0.0) {return 0.0;}
  return std::copysign(std::clamp(gain*std::abs(remaining), minimum, maximum), remaining);
}
}  // namespace astribot_s1_path_tracking
#endif
