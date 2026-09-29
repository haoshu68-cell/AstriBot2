#pragma once

#include <array>

namespace astribot::chassis_effort {

using TauError = std::array<double, 2>;

double clamp(double value, double lower, double upper);

std::array<double, 4> inverse_kinematics(
    double vx, double vy, double wz, double wheel_radius,
    const std::array<double, 4>& coeff_vx,
    const std::array<double, 4>& coeff_vy,
    const std::array<double, 4>& coeff_wz,
    double global_sign);

class WheelLoop {
public:
  void reset();
  TauError update(double target, double measured, double kp, double ki,
                  double kd, double tau_c, double tau_v, double deadband,
                  double tau_max, double dt);
  double integral() const { return integral_; }
  double prev_error() const { return prev_error_; }

private:
  double integral_{0.0};
  double prev_error_{0.0};
};

}  // namespace astribot::chassis_effort
