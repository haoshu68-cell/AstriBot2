#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "astribot_trajectory_bridge_native/chassis_math.hpp"
#include "astribot_trajectory_bridge_native/arm_math.hpp"

using astribot_trajectory_bridge_native::Vec3;
using astribot_trajectory_bridge_native::error_magnitude;
using astribot_trajectory_bridge_native::leash_error;
using astribot_trajectory_bridge_native::integrate_step_dt;
using astribot_trajectory_bridge_native::local_pose_displacement;
using astribot_trajectory_bridge_native::pose_error;
using astribot_trajectory_bridge_native::pose_jump_distance;
using astribot_trajectory_bridge_native::odom_drift;
using astribot_trajectory_bridge_native::measure_tick_dt;
using astribot_trajectory_bridge_native::PoseFrameIntegrator;
using astribot_trajectory_bridge_native::SdkPoseHistory;
using astribot_trajectory_bridge_native::ChassisOdomSource;
using astribot_trajectory_bridge_native::to_local_velocity;
using astribot_trajectory_bridge_native::wrap_angle;
using astribot_trajectory_bridge_native::interpolate_trajectory;
using astribot_trajectory_bridge_native::clamp_cmd;
using astribot_trajectory_bridge_native::cmd_to_rad;
using astribot_trajectory_bridge_native::rad_to_cmd;
using astribot_trajectory_bridge_native::opening_fraction_to_cmd;
using astribot_trajectory_bridge_native::cmd_to_opening_fraction;

namespace {
void near(double actual, double expected, double tolerance = 1e-12) {
  if (!(std::abs(actual - expected) <= tolerance)) {
    std::fprintf(stderr, "actual=%.17g expected=%.17g tolerance=%.17g\n", actual, expected, tolerance);
  }
  assert(std::abs(actual - expected) <= tolerance);
}
}

int main() {
  constexpr double pi = 3.141592653589793238462643383279502884;
  near(wrap_angle(3.0 * pi), pi, 1e-12);
  const auto body = to_local_velocity({1.0, -2.0, 0.3}, "body", 1.2);
  near(body[0], 1.0);
  near(body[1], -2.0);
  const auto world = to_local_velocity({1.0, 0.0, 0.3}, "world", pi / 2.0);
  near(world[0], 0.0, 1e-12);
  near(world[1], -1.0, 1e-12);
  const auto integrated = integrate_step_dt({1.0, 2.0, 3.0}, {0.5, -1.0, 2.0}, 0.2);
  near(integrated[0], 1.1);
  near(integrated[1], 1.8);
  near(integrated[2], 3.4);
  const auto displacement = local_pose_displacement({0.0, 0.0, 0.0}, {1.0, 0.0, 0.2});
  near(displacement[0], 0.996664442325924, 1e-12);
  near(displacement[1], -0.1, 1e-12);
  const auto error = pose_error({1.0, -2.0, 3.2}, {0.5, -1.0, -3.0});
  near(error[0], 0.5);
  near(error[1], -1.0);
  near(error[2], -0.08318530717958623, 1e-12);
  const auto magnitude = error_magnitude(error);
  near(magnitude[0], std::hypot(0.5, -1.0));
  near(magnitude[1], std::abs(error[2]));
  const auto leash = leash_error({1.0, -2.0, 3.2}, {0.5, -1.0, -3.0});
  near(leash[0], std::hypot(0.5, -1.0));
  near(leash[1], 6.2);
  near(pose_jump_distance({1.0, 2.0, 0.0}, {0.0, 0.0, 2.0}), std::hypot(1.0, 2.0));
  near(odom_drift({3.0, 4.0}, {0.0, 0.0}), 5.0);
  const auto first_tick = measure_tick_dt(1.0, std::nullopt, 0.004, 0.04);
  near(first_tick.dt, 0.004);
  assert(!first_tick.clamped);
  assert(!first_tick.has_raw);
  const auto stalled_tick = measure_tick_dt(0.99, 1.0, 0.004, 0.04);
  near(stalled_tick.dt, 0.004);
  assert(stalled_tick.clamped);
  assert(stalled_tick.reason == "时钟未前进（raw=-0.010000s），退回标称步长");
  const auto long_tick = measure_tick_dt(1.2, 1.0, 0.004, 0.04);
  near(long_tick.dt, 0.04);
  assert(long_tick.clamped);
  assert(long_tick.reason == "实测步长 0.2000s 超过上限 0.0400s，已钳位。"
                            "未钳位的话本拍会积出一次位置阶跃");
  bool threw = false;
  try {
    (void)integrate_step_dt({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw);

  PoseFrameIntegrator integrator({0.0, 0.0, 0.0}, 1.0, {10.0, 20.0, 30.0});
  assert(!integrator.observe({1.0, 0.0, 0.0}, 1.0));
  assert(integrator.observe({1.0, 0.0, 0.2}, 2.0));
  near(integrator.anchor()[0], 10.996664442325924, 1e-12);
  near(integrator.anchor()[1], 19.9, 1e-12);
  near(integrator.anchor()[2], 30.2, 1e-12);
  integrator.reanchor_axis(0, 11.0);
  integrator.commit_preview({0.1, 0.0, 0.2}, 0.01);
  near(integrator.integral()[0], 0.001, 1e-12);

  SdkPoseHistory history(1.0);
  history.append(0.0, {0.0, 0.0, 0.0});
  history.append(1.0, {1.0, 2.0, 3.0});
  const auto interpolated = history.at(0.5);
  near(interpolated[0], 0.5);
  near(interpolated[1], 1.0);
  near(interpolated[2], 1.5);

  ChassisOdomSource odom(0.3, "body");
  const auto odom_first = odom.sample({0.0, 0.0, 3.0 * pi}, {1.0, -2.0, 0.4});
  near(odom_first.theta, pi, 1e-12);
  assert(!odom_first.jumped);
  const auto odom_second = odom.sample({0.1, 0.0, 3.0 * pi}, {1.0, -2.0, 0.4});
  near(odom_second.jump_m, 0.1);
  const auto odom_third = odom.sample({1.0, 0.0, 3.0 * pi}, {1.0, -2.0, 0.4});
  assert(odom_third.jumped);
  const auto odom_stats = odom.stats();
  assert(odom_stats.samples == 3U);
  assert(odom_stats.jumps == 1U);
  near(odom_stats.travelled_m, 0.1);
  near(odom.jump_ratio(), 1.0 / 3.0);

  ChassisOdomSource world_odom(0.3, "world");
  const auto world_sample = world_odom.sample({0.0, 0.0, pi / 2.0},
                                               {1.0, 0.0, 0.0});
  near(world_sample.vx_body, 0.0, 1e-12);
  near(world_sample.vy_body, -1.0, 1e-12);

  const auto linear = interpolate_trajectory({0.0, 1.0}, {{0.0, 1.0}, {1.0, 3.0}},
                                             {}, 0.25, false);
  near(linear[0], 0.25);
  near(linear[1], 1.5);
  const auto cubic = interpolate_trajectory({0.0, 1.0}, {{0.0}, {1.0}},
                                            {{0.0}, {0.0}}, 0.5, true);
  near(cubic[0], 0.5);
  near(clamp_cmd(-20.0), 0.0);
  near(clamp_cmd(150.0), 100.0);
  near(cmd_to_rad(100.0, true), 0.93);
  near(rad_to_cmd(0.93, true), 100.0);
  near(opening_fraction_to_cmd(1.0), 0.0);
  near(cmd_to_opening_fraction(0.0), 1.0);
  return 0;
}
