#include "astribot_s1_chassis_effort_drive_native/wheel_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace astribot::chassis_effort {

double clamp(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(upper, value));
}

std::array<double, 4> inverse_kinematics(
    const double vx, const double vy, const double wz,
    const double wheel_radius, const std::array<double, 4>& coeff_vx,
    const std::array<double, 4>& coeff_vy,
    const std::array<double, 4>& coeff_wz, const double global_sign) {
  std::array<double, 4> targets{};
  if (wheel_radius <= 1e-6) {
    return targets;
  }
  for (std::size_t i = 0; i < targets.size(); ++i) {
    targets[i] = global_sign / wheel_radius *
                 (coeff_vx[i] * vx + coeff_vy[i] * vy + coeff_wz[i] * wz);
  }
  return targets;
}

void WheelLoop::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
}

TauError WheelLoop::update(const double target, const double measured,
                           const double kp, const double ki, const double kd,
                           const double tau_c, const double tau_v,
                           const double deadband, const double tau_max,
                           const double dt) {
  const double error = target - measured;
  // A stopped wheel must not unwind the preceding motion's opposing bias
  // before responding. Preserve integral braking while it is still moving.
  if (ki > 0.0 && dt > 0.0 && deadband >= 0.0 &&
      std::abs(target) > deadband && std::abs(measured) <= deadband &&
      integral_ * target < 0.0) {
    integral_ = 0.0;
  }
  double tau_ff = 0.0;
  if (std::abs(target) > deadband) {
    const double sign = target > 0.0 ? 1.0 : (target < 0.0 ? -1.0 : 0.0);
    tau_ff = tau_c * sign + tau_v * measured;
  }
  const double derivative = dt > 1e-9 ? (error - prev_error_) / dt : 0.0;
  const double tau_pid = kp * error + ki * integral_ + kd * derivative;
  const double tau_unclamped = tau_pid + tau_ff;
  const double tau = clamp(tau_unclamped, -tau_max, tau_max);
  if (tau == tau_unclamped) {
    integral_ += error * dt;
  }
  prev_error_ = error;
  return {tau, error};
}

}  // namespace astribot::chassis_effort
