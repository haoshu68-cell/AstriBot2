#include "astribot_s1_path_tracking/align_math.hpp"

#include <cassert>
#include <cmath>

#ifdef NDEBUG
#error "corner_turn_test requires active assertions in Release builds"
#endif

using astribot_s1_path_tracking::detectStandardCorners;
using astribot_s1_path_tracking::projectPathProgress;

int main()
{
  const double pi = std::acos(-1.0);
  const auto straight = detectStandardCorners(
    {{0., 0.}, {1., 0.}, {2., 0.}}, .25, .61, 2.75);
  assert(straight.empty());

  const auto right = detectStandardCorners(
    {{0., 0.}, {1., 0.}, {1., 1.}, {1., 2.}}, .25, .61, 2.75);
  assert(right.size() == 1U);
  assert(std::abs(right.front().turn_angle - pi / 2.) < 1e-9);
  assert(std::abs(right.front().arc_length - 1.) < 1e-9);
  const std::vector<astribot_s1_path_tracking::PlanarPoint> right_path{
    {0., 0.}, {1., 0.}, {1., 1.}, {1., 2.}};

  const auto acute = detectStandardCorners(
    {{0., 0.}, {1., 0.}, {1.5, .8660254038}, {2.5, .8660254038}}, .25, .61, 2.75);
  assert(acute.size() == 2U);
  assert(acute.front().turn_angle > .9 && acute.front().turn_angle < 1.3);

  // An obtuse standard turn is handled the same way as a right/acute turn;
  // only the near-U-turn recovery range is excluded.
  const auto obtuse = detectStandardCorners(
    {{0., 0.}, {1., 0.}, {0.5, .8660254038}, {0., 1.7320508076}}, .25, .61, 2.75);
  assert(obtuse.size() == 1U);
  assert(std::abs(obtuse.front().turn_angle - 2.0943951024) < 1e-8);

  // The planner may add a short bevel immediately after a waypoint.  The
  // effective-window detector must still retain the standard corner.
  const auto beveled = detectStandardCorners(
    {{0., 0.}, {1., 0.}, {1.0, .02}, {.98, .25}, {1.0, .50}}, .25, .61, 2.75);
  assert(beveled.size() == 1U);
  assert(beveled.front().index == 1U || beveled.front().index == 2U);

  // A smooth quarter circle has no single segment turn above the threshold.
  const auto smooth = detectStandardCorners(
    {{0., 0.}, {.3827, .0761}, {.7071, .2929}, {.9239, .6173}, {1., 1.}},
    .25, .61, 2.75);
  assert(smooth.empty());

  // A U-turn is a recovery maneuver, not a standard corner.
  const auto u_turn = detectStandardCorners(
    {{0., 0.}, {1., 0.}, {.1, 0.}, {-1., 0.}}, .25, .61, 2.75);
  assert(u_turn.empty());

  for (const double angle : {-135., -90., -45., 45., 90., 135.}) {
    std::vector<astribot_s1_path_tracking::PlanarPoint> sampled;
    for (int i = 0; i <= 20; ++i) {sampled.push_back({i * .05, 0.});}
    for (int i = 1; i <= 20; ++i) {
      sampled.push_back({1. + i * .05 * std::cos(angle * pi / 180.),
        i * .05 * std::sin(angle * pi / 180.)});
    }
    const auto detected = detectStandardCorners(sampled, .25, .61, 2.75);
    assert(detected.size() == 1U);
    assert(std::hypot(detected.front().position.x - 1., detected.front().position.y) < 1e-9);
  }

  // High curvature remains a smooth arc; accumulated window heading alone
  // must not turn it into a succession of stop-and-rotate vertices.
  std::vector<astribot_s1_path_tracking::PlanarPoint> tight_arc;
  for (int i = 0; i <= 36; ++i) {
    const double angle = i * pi / 36.;
    tight_arc.push_back({.4 * std::sin(angle), .4 * (1. - std::cos(angle))});
  }
  assert(detectStandardCorners(tight_arc, .25, .61, 2.75).empty());

  const auto duplicates = detectStandardCorners(
    {{0., 0.}, {1., 0.}, {1., 0.}, {1., 0.}, {1., 1.}}, .25, .61, 2.75);
  assert(duplicates.size() == 1U);
  const auto two_corners = detectStandardCorners(
    {{0., 0.}, {1., 0.}, {1., .6}, {2., .6}}, .25, .61, 2.75);
  assert(two_corners.size() == 2U);

  double progress = 0.;
  double lateral = 0.;
  assert(projectPathProgress(right_path, {1.0, .2}, progress, lateral));
  assert(std::abs(progress - 1.2) < 1e-9);
  assert(std::abs(lateral - 0.0) < 1e-9);
  assert(projectPathProgress(right_path, {.7, .2}, progress, lateral));
  assert(std::abs(progress - .7) < 1e-9);
  assert(std::abs(lateral - .2) < 1e-9);
  return 0;
}
