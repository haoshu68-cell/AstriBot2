#include <cassert>
#include <cmath>

#include "astribot_s1_chassis_effort_drive_native/wheel_math.hpp"

int main() {
  using astribot::chassis_effort::WheelLoop;
  using astribot::chassis_effort::inverse_kinematics;
  WheelLoop loop;
  const auto first = loop.update(1.0, 0.0, 0.4, 0.1, 0.0,
                                 0.1, 1.0, 0.05, 15.0, 0.01);
  assert(std::abs(first[1] - 1.0) < 1e-12);
  assert(loop.integral() > 0.0);
  const auto saturated = loop.update(100.0, 0.0, 0.4, 0.1, 0.0,
                                     0.1, 1.0, 0.05, 0.01, 0.01);
  assert(std::abs(saturated[0] - 0.01) < 1e-12);
  const double saved = loop.integral();
  loop.reset();
  assert(loop.integral() == 0.0);
  assert(loop.prev_error() == 0.0);
  assert(saved > 0.0);
  const auto wheels = inverse_kinematics(1.0, 0.0, 0.0, 0.5,
                                         {1.0, -1.0, 1.0, -1.0},
                                         {0.0, 0.0, 0.0, 0.0},
                                         {0.0, 0.0, 0.0, 0.0}, 1.0);
  assert(wheels[0] == 2.0);
  assert(wheels[1] == -2.0);
  return 0;
}
