#include <cassert>
#include <cmath>

#include "astribot_s1_navigation_policy_native/cmd_vel_body_to_world_core.hpp"

int main() {
  using namespace astribot::navigation;
  BodyToWorldConfig config;
  config.enable_body_to_world = true;
  config.enable_posture_monitor = false;
  CmdVelBodyToWorldCore core(config);
  const double half_turn = 0.5 * 3.14159265358979323846;
  const auto odom = core.odom(0.0, 0.0, std::sin(half_turn / 2.0),
                              std::cos(half_turn / 2.0), 0.134, 1.0, 1.0,
                              1.0);
  assert(!odom.publish_zero);
  const auto converted = core.command(1.0, 0.0, 0.0, 0.0, 0.0, 0.2,
                                      1.0, 1.0);
  assert(std::abs(converted.linear_x) < 1e-12);
  assert(std::abs(converted.linear_y - 1.0) < 1e-12);
  assert(std::abs(converted.angular_z - 0.2) < 1e-12);
  auto runtime_config = config;
  runtime_config.enable_body_to_world = false;
  core.update_config(runtime_config);
  const auto body = core.command(1.0, 0.0, 0.0, 0.0, 0.0, 0.2, 1.1, 1.1);
  assert(body.linear_x == 1.0 && body.linear_y == 0.0);
  core.update_config(config);
  // No new odometry: the previous yaw and source age must survive reconfiguration.
  const auto retained = core.command(1.0, 0.0, 0.0, 0.0, 0.0, 0.2, 1.2, 1.2);
  assert(std::abs(retained.linear_x) < 1e-12);
  assert(std::abs(retained.linear_y - 1.0) < 1e-12);
  assert(core.watchdog(1.6, 1.6));
  assert(!core.watchdog(1.7, 1.7));

  const auto invalid = core.odom(0.0, 0.0, 0.0, 0.0, 0.134, 1.7, 1.7, 1.7);
  assert(invalid.publish_zero);
  const auto bad_command = core.command(NAN, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        1.7, 1.7);
  assert(bad_command.linear_x == 0.0 && bad_command.linear_y == 0.0 &&
         bad_command.angular_z == 0.0);

  BodyToWorldConfig posture_config;
  posture_config.enable_posture_monitor = true;
  CmdVelBodyToWorldCore posture(posture_config);
  for (int index = 0; index < 20; ++index) {
    const auto sample = posture.odom(0.0, 0.0, 0.0, 1.0, 0.134,
                                    2.0 + index * 0.01,
                                    2.0 + index * 0.01,
                                    2.0 + index * 0.01);
    assert(!sample.tripped);
  }
  const auto tripped = posture.odom(0.0, 0.0, 0.0, 1.0, 0.3, 2.3, 2.3, 2.3);
  assert(tripped.tripped && tripped.publish_zero);
  const auto held = posture.command(0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    2.3, 2.3);
  assert(held.linear_x == 0.0 && held.linear_y == 0.0);
  return 0;
}
