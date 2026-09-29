#include <cassert>
#include <cmath>

#include "astribot_s1_navigation_policy_native/navigation_math.hpp"

int main() {
  using namespace astribot::navigation;
  const auto pose = body_pose({1.0, 0.0, 0.5}, 0.2);
  assert(std::abs(pose[2] - 0.1) < 1e-12);
  assert(std::abs(stopping_horizon({3.0, 4.0, 0.0}, 0.1, 1.0, 2.0, 0.0) - 5.1) < 1e-12);
  const auto world = body_to_world_xy(1.0, 0.0, 1.5707963267948966);
  assert(std::abs(world[0]) < 1e-12);
  assert(std::abs(world[1] - 1.0) < 1e-12);
  const auto axes = footprint_axes(3.0, 4.0, 0.0);
  assert(std::abs(axes[2] - 3.0) < 1e-12);
  assert(std::abs(sampling_margin({3.0, 4.0, 0.0}, 0.3, 0.2, 0.1) - 0.25) < 1e-12);
  CommandRestriction restriction;
  const auto limited = restriction.apply({1.0, 0.0, 0.0}, 0.5, 1.0, false,
                                         0.1, 1.0, 2.0, false);
  // The reference caps recovery integration at 50 ms, even for a 100 ms tick.
  assert(std::abs(limited[0] - 0.05) < 1e-12);
  assert(restriction.recovering());
  restriction.set_state({0.5, 0.0, 0.0}, false);
  const auto clear = restriction.apply({0.2, 0.0, 0.0}, 1.0, 1.0, false,
                                       0.1, 1.0, 2.0, false);
  assert(std::abs(clear[0] - 0.2) < 1e-12);
  assert(!restriction.recovering());
  const auto stopped = restriction.apply({0.2, 0.0, 0.0}, 1.0, 1.0, true,
                                          0.1, 1.0, 2.0, false);
  assert(stopped == std::vector<double>({0.0, 0.0, 0.0}));
  assert(scan_usable({0.1, 0.2, INFINITY}, 0.05, 1.0, -1.0, 0.1, 0.9));
  assert(!scan_usable({0.1, NAN, 2.0}, 0.05, 1.0, -1.0, 0.1, 0.9));
  const auto cells = occupied_cells({{{-0.1, -0.1}, {0.1, 0.1}, {0.1, 0.1}}}, 0.2);
  assert(cells.size() == 2U);
  assert(cells[0][0] == -1 && cells[0][1] == -1);
  assert(cells[1][0] == 0 && cells[1][1] == 0);
  assert(!angular_box_free({{{1.0, -0.05}, {1.0, 0.05},
                           {1.1, 0.05}, {1.1, -0.05}}},
                          {2.0, 2.0, 2.0, 2.0}, 0.1, 3.0, -1.0,
                          0.1, 0.2));
  assert(angular_box_free({{{1.0, -0.05}, {1.0, 0.05},
                           {1.1, 0.05}, {1.1, -0.05}}},
                          {2.0, 2.0, 2.0, 2.0}, 0.1, 3.0, -0.15,
                          0.1, 0.2));
  const auto posture = posture_out_of_bounds(0.8, 0.0, 0.0, 0.6, 0.1, 0.2);
  assert(posture.first);
  assert(posture.second == "高度偏差 |0.8000 - 0.6000| = 0.2000 超过 0.1000");
  const std::vector<std::array<double, 3>> steady(20, {0.6, 0.0, 0.0});
  assert(is_degenerate_attitude_source(steady, 1e-9, 20U));
  assert(!is_degenerate_attitude_source(steady, 1e-9, 21U));
  assert(describe_monitor_state(false, false, false) ==
         "姿态监控已由配置显式禁用。");
  assert(evaluate_posture(true, steady, 0.6, 0.0, 0.0, 0.6, 0.1, 0.2, 20U).first ==
         "pass");
  const auto coverage = scan_coverage(
      {0.5, 0.6, INFINITY, NAN, 0.7, 0.8}, 0.2, 1.0, -1.0, 0.1, 0.2);
  assert(coverage.size() == 8U);
  assert(std::abs(coverage[0] - std::cos(-0.7)) < 1e-12);
  assert(std::abs(coverage[1] - std::sin(-0.7)) < 1e-12);
  assert(std::abs(coverage[3] - 0.15) < 1e-12);
  const auto rotate_directions = movement_directions(0.0, 0.0, 0.03);
  assert(rotate_directions.size() == 16U);
  const auto translate_directions = movement_directions(1.0, 0.0, 0.0);
  assert(translate_directions.size() == 3U);
  assert(coverage_allows_motion(
      {-1.0, 0.0, 3.14159265358979323846,
       1.0, 0.0, 3.14159265358979323846}, 1.0, 0.0, 0.0));
  assert(!coverage_allows_motion({1.0, 0.0, 0.1}, 1.0, 0.0, 0.0));
  ControlTime sim_time(true, 0.5);
  const auto first_time = sim_time.advance(10.0, 100.0);
  assert(first_time.now == 10.0 && first_time.dt == 0.0 && first_time.running);
  (void)first_time;
  const auto stalled_time = sim_time.advance(10.0, 100.6);
  assert(!stalled_time.running && stalled_time.stop_commands);
  (void)stalled_time;
  const auto rewind_time = sim_time.advance(9.0, 100.7);
  assert(rewind_time.reset);
  (void)rewind_time;
  assert(sim_time.accepts(8.9));
  assert(sim_time.accepts(1000.0));
  assert(sim_time.fresh(1000.0, 1.0, .3, 9.0, 100.7));
  assert(sim_time.fresh(1.0, 1.0, .3, 9.0, 100.7));
  assert(sim_time.command_fresh(1000.0, 100.6, .3, 9.0, 100.7));
  assert(!sim_time.command_fresh(1000.0, 100.0, .3, 9.0, 100.7));
  assert(!sim_time.accepts(NAN));
  const auto variants = lateral_variants(
      {0.0, 0.0, 0.5, 0.1, 1.0, 0.0, 1.5, -0.1}, 0.2);
  assert(variants.size() == 8U * 8U);
  assert(variants[0] == 0.0 && variants[1] == 0.0);
  YieldPolicy yield(0.7, 0.2, 0.18, 0.72, 0.04, 2.0, 0.3);
  const auto yield_hold = yield.select(true, false, false, 0.0, true, 1.0);
  assert(yield_hold.motion == "HOLD" && yield_hold.reason == "IMMEDIATE_RISK");
  const auto yield_clear = yield.select(false, false, false, 0.0, true, 1.5);
  assert(yield_clear.reason == "CLEAR_CONFIRMATION");
  ExecutionContext execution;
  execution.task("goal", "EXECUTING", 1);
  assert(std::get<0>(execution.version()) == "goal");
  assert(execution.map("map-a"));
  assert(!execution.map("map-a"));
  assert(!execution.localization({0.0, 0.0, 0.0}, 0.2, 0.15));
  assert(execution.localization({0.5, 0.0, 0.0}, 0.2, 0.15));
  execution.path();
  assert(std::get<1>(execution.version()) == 1);
  PlanningSessionState planning("session", 100000000, 500000000, 2);
  planning.activate("goal", 1, 2, 3, 4, 5, 1000000000);
  const auto planning_request = planning.request(
      "goal", 1, 2, 3, 4, 5, 7, 1000000000);
  assert(std::get<0>(planning_request) == 0);
  assert(std::get<1>(planning_request) == "session:1");
  const auto same_request = planning.request(
      "goal", 1, 2, 3, 4, 5, 8, 1050000000);
  assert(std::get<0>(same_request) == 1);
  assert(planning.response_current("session:1", "goal", 1, 2, 3, 4, 5,
                                  1050000000));
  planning.retire("session:1");
  assert(planning.failure_reason(1100000000, 5) == 0);
  planning.activate("goal", 2, 2, 3, 4, 5, 1200000000);
  assert(!planning.clear_blockage(1100000000, 5));
  const auto cleared = costmap_clearing_ranges({INFINITY, 1.0, 2.0}, 3.0, 1.0);
  assert(std::abs(cleared[0] - 2.9999) < 1e-12);
  assert(std::abs(cleared[1] - 1.0) < 1e-12);
  const auto clearance = clearance_many_rect(
      {0.0, 2.0}, {0.0, 0.0}, {0.0, 0.0},
      {-0.5, -0.5, 1.5, -0.5}, {0.5, 0.5, 2.5, 0.5},
      0.3, 0.2, 0.1);
  // Golden values from the Python rectangle bound for overlapping boxes.
  assert(std::abs(clearance[0] + 0.4605551275473989) < 1e-12);
  assert(std::abs(clearance[1] + 0.4605551275473989) < 1e-12);
  const auto sweep = motion_clearance_rect(
      {1.0, 0.0, 0.0}, {0.0, 0.0}, {0.5, 0.5},
      {2.0, -0.5, 0.4, -0.5}, {2.5, 0.5, 0.9, 0.5},
      0.3, 0.2, 0.1, 0.0, {0.0, 0.0, 0.0});
  assert(sweep.size() == 2U);
  assert(sweep[0] > 0.0);
  assert(sweep[1] < 0.0);
  assert(motion_clearance_rect(
             {1.0, 0.0, 0.0}, {}, {}, {}, {},
             0.3, 0.2, 0.1, 0.0, {0.0, 0.0, 0.0})
             .empty());
  const auto forecast = path_position_batch(
      {0.0, 0.0, 0.0, 0.0, 1.0, 0.0}, {-0.1, 0.5, 2.0},
      {9.0, 8.0, 0.7});
  assert(forecast.size() == 9U);
  assert(std::abs(forecast[0] + 0.1) < 1e-12);
  assert(std::abs(forecast[1]) < 1e-12);
  assert(std::abs(forecast[2]) < 1e-12);
  assert(std::abs(forecast[3] - 0.5) < 1e-12);
  assert(std::abs(forecast[4]) < 1e-12);
  assert(std::abs(forecast[5]) < 1e-12);
  assert(std::abs(forecast[6] - 1.0) < 1e-12);
  assert(std::abs(forecast[7]) < 1e-12);
  assert(std::abs(forecast[8]) < 1e-12);
  const auto remaining = remaining_path(
      {0.0, 0.0, 1.0, 0.0, 1.0, 1.0}, 0.2, 0.3);
  assert(remaining.size() == 8U);
  assert(std::abs(remaining[0] - 0.2) < 1e-12);
  assert(std::abs(remaining[1] - 0.3) < 1e-12);
  assert(std::abs(remaining[2] - 0.2) < 1e-12);
  assert(std::abs(remaining[3]) < 1e-12);
  assert(std::abs(remaining[4] - 1.0) < 1e-12);
  assert(std::abs(remaining[5]) < 1e-12);
  assert(std::abs(remaining[6] - 1.0) < 1e-12);
  assert(std::abs(remaining[7] - 1.0) < 1e-12);
  return 0;
}
