// Copyright 2026 Astribot

#include "astribot_s1_path_tracking/align_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace astribot_s1_path_tracking
{

const char * toString(Phase p)
{
  switch (p) {
    case Phase::kAlignStart:
      return "ALIGN_START";
    case Phase::kCornerApproach:
      return "CORNER_APPROACH";
    case Phase::kAlignCorner:
      return "ALIGN_CORNER";
    case Phase::kFollow:
      return "FOLLOW";
    case Phase::kAlignGoal:
      return "ALIGN_GOAL";
    case Phase::kDone:
      return "DONE";
  }
  return "UNKNOWN";
}

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double shortestAngularDiff(double from, double to)
{
  return normalizeAngle(to - from);
}

bool pathStartHeading(
  const std::vector<PlanarPoint> & path, double lookahead_m, double & heading)
{
  if (path.size() < 2U || !(lookahead_m > 0.0)) {
    return false;
  }
  const PlanarPoint & p0 = path.front();
  double acc = 0.0;
  std::size_t idx = 0U;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    acc += std::hypot(path[i].x - path[i - 1U].x, path[i].y - path[i - 1U].y);
    idx = i;
    if (acc >= lookahead_m) {
      break;
    }
  }
  const double dx = path[idx].x - p0.x;
  const double dy = path[idx].y - p0.y;
  if (std::hypot(dx, dy) < 1e-9) {
    return false;
  }
  heading = std::atan2(dy, dx);
  return true;
}

CornerGeometryIssue inspectCornerGeometry(const std::vector<PlanarPoint> & path,
  double min_segment_m, double min_turn_rad, double max_turn_rad)
{
  std::vector<PlanarPoint> unique;
  std::vector<std::size_t> indices;
  std::vector<double> arc;
  for (std::size_t i=0;i<path.size();++i) {
    const double length=unique.empty()?0.:std::hypot(path[i].x-unique.back().x,path[i].y-unique.back().y);
    if (!unique.empty() && length<1e-9) {continue;}
    unique.push_back(path[i]);indices.push_back(i);arc.push_back(arc.empty()?0.:arc.back()+length);
  }
  if (unique.size()<3) {return {};}
  double previous_arc=-1., grouped_turn=0.;
  CornerGeometryIssue result;
  for (std::size_t i=1;i+1<unique.size();++i) {
    const auto & a=unique[i-1];const auto & b=unique[i];const auto & c=unique[i+1];
    const double turn=shortestAngularDiff(std::atan2(b.y-a.y,b.x-a.x),std::atan2(c.y-b.y,c.x-b.x));
    if (std::abs(turn)<min_turn_rad) {continue;}
    if (std::abs(turn)>max_turn_rad) {return {CornerGeometry::Turnaround,indices[i]};}
    if (arc[i]+1e-9<min_segment_m) {return {CornerGeometry::ShortApproach,indices[i]};}
    if (previous_arc>=0. && arc[i]-previous_arc+1e-9<min_segment_m) {
      // A same-direction bevel may represent one vertex; a short opposite
      // turn represents two different required directions and must not vanish.
      if (grouped_turn*turn<0.) {return {CornerGeometry::DenseDogleg,indices[i]};}
      grouped_turn+=turn;
      if (std::abs(grouped_turn)>max_turn_rad) {return {CornerGeometry::Turnaround,indices[i]};}
    } else {grouped_turn=turn;}
    previous_arc=arc[i];
    if (arc.back()-arc[i]+1e-9<min_segment_m) {
      result={CornerGeometry::TerminalHandoff,indices[i]};
    }
  }
  return result;
}

std::vector<PathCorner> detectStandardCorners(
  const std::vector<PlanarPoint> & path, double min_segment_m,
  double min_turn_rad, double max_turn_rad)
{
  std::vector<PathCorner> result;
  if (path.size() < 3U || !(min_segment_m > 0.0) || !(min_turn_rad > 0.0) ||
    !(max_turn_rad > min_turn_rad) || max_turn_rad >= M_PI)
  {
    return result;
  }

  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i = 1U; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1U].x;
    const double dy = path[i].y - path[i - 1U].y;
    if (!std::isfinite(dx) || !std::isfinite(dy)) {
      return {};
    }
    arc[i] = arc[i - 1U] + std::hypot(dx, dy);
  }

  for (std::size_t i = 1U; i + 1U < path.size(); ++i) {
    // A standard vertex needs a local heading discontinuity. Window heading
    // alone also grows on smooth tight arcs and at samples beside one vertex.
    std::size_t local_before = i;
    std::size_t local_after = i;
    while (local_before > 0U && arc[i] - arc[local_before] < 1e-9) {--local_before;}
    while (local_after + 1U < path.size() && arc[local_after] - arc[i] < 1e-9) {
      ++local_after;
    }
    if (arc[i] - arc[local_before] < 1e-9 || arc[local_after] - arc[i] < 1e-9) {
      continue;
    }
    const double local_in = std::atan2(
      path[i].y - path[local_before].y, path[i].x - path[local_before].x);
    const double local_out = std::atan2(
      path[local_after].y - path[i].y, path[local_after].x - path[i].x);
    if (std::fabs(shortestAngularDiff(local_in, local_out)) < min_turn_rad) {
      continue;
    }
    // Planners commonly insert a short bevel/sample at a waypoint.  Comparing
    // only adjacent samples would then miss a real 90-degree turn.  Estimate
    // each side over at least min_segment_m of accumulated arc length instead.
    std::size_t before = i;
    double in_len = 0.0;
    while (before > 0U && in_len < min_segment_m) {
      --before;
      in_len += arc[before + 1U] - arc[before];
    }
    std::size_t after = i;
    double out_len = 0.0;
    while (after + 1U < path.size() && out_len < min_segment_m) {
      ++after;
      out_len += arc[after] - arc[after - 1U];
    }
    if (in_len < min_segment_m || out_len < min_segment_m || before == i || after == i) {
      continue;
    }
    const double in_dx = path[i].x - path[before].x;
    const double in_dy = path[i].y - path[before].y;
    const double out_dx = path[after].x - path[i].x;
    const double out_dy = path[after].y - path[i].y;
    if (std::hypot(in_dx, in_dy) < 1e-9 || std::hypot(out_dx, out_dy) < 1e-9) {
      continue;
    }
    const double in_heading = std::atan2(in_dy, in_dx);
    const double out_heading = std::atan2(out_dy, out_dx);
    const double turn = std::fabs(shortestAngularDiff(in_heading, out_heading));
    if (!std::isfinite(turn) || turn < min_turn_rad || turn > max_turn_rad) {
      continue;
    }
    PathCorner corner;
    corner.position = path[i];
    corner.incoming_heading = in_heading;
    corner.outgoing_heading = out_heading;
    corner.turn_angle = turn;
    corner.arc_length = arc[i];
    corner.index = i;

    // A discretized sharp vertex may occupy two adjacent samples. Keep the
    // sharper one so the controller stops once instead of oscillating between
    // nearly coincident corner targets.
    if (!result.empty() && corner.arc_length - result.back().arc_length < min_segment_m) {
      if (corner.turn_angle > result.back().turn_angle) {
        result.back() = corner;
      }
      continue;
    }
    result.push_back(corner);
  }
  return result;
}

bool projectPathProgress(
  const std::vector<PlanarPoint> & path, const PlanarPoint & point,
  double & progress_m, double & lateral_m)
{
  if (path.size() < 2U || !std::isfinite(point.x) || !std::isfinite(point.y)) {
    return false;
  }
  double best = std::numeric_limits<double>::infinity();
  double best_progress = 0.0;
  double best_lateral = 0.0;
  double arc = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1U].x;
    const double dy = path[i].y - path[i - 1U].y;
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length)) {
      return false;
    }
    if (length < 1e-9) {
      continue;
    }
    const double f = std::clamp(
      ((point.x - path[i - 1U].x) * dx + (point.y - path[i - 1U].y) * dy) /
      (length * length), 0.0, 1.0);
    const double px = path[i - 1U].x + f * dx;
    const double py = path[i - 1U].y + f * dy;
    const double ex = point.x - px;
    const double ey = point.y - py;
    const double distance = std::hypot(ex, ey);
    if (distance < best) {
      best = distance;
      best_progress = arc + f * length;
      best_lateral = (dx * ey - dy * ex) / length;
    }
    arc += length;
  }
  if (!std::isfinite(best)) {
    return false;
  }
  progress_m = best_progress;
  lateral_m = best_lateral;
  return true;
}

bool isSameGoal(const PlanarPoint & prev_end, const PlanarPoint & cur_end, double eps_m)
{
  if (!(eps_m > 0.0)) {
    return false;
  }
  return std::hypot(cur_end.x - prev_end.x, cur_end.y - prev_end.y) <= eps_m;
}

bool needsStartAlign(double heading_error_rad, double min_angle_rad)
{
  return std::fabs(heading_error_rad) > min_angle_rad;
}

double alignAngularVelocity(
  double error_rad, double kp, double max_vel, double floor_vel, double tol_rad)
{
  if (std::fabs(error_rad) <= tol_rad) {
    return 0.0;
  }
  double v = kp * error_rad;
  const double mag = std::fabs(v);
  const double sign = (error_rad >= 0.0) ? 1.0 : -1.0;
  double out = std::max(mag, std::fabs(floor_vel));
  out = std::min(out, std::fabs(max_vel));
  return sign * out;
}

double coastAngle(double wz, double lag_s, double decel_rad_s2)
{
  if (!(lag_s >= 0.0) || !(decel_rad_s2 > 0.0)) {
    return 0.0;
  }
  const double w = std::fabs(wz);
  if (!(w > 0.0)) {              // NaN 也走这一支
    return 0.0;
  }
  return w * lag_s + w * w / (2.0 * decel_rad_s2);
}

double predictedHeadingError(double error_rad, double wz, double lag_s, double decel_rad_s2)
{
  const double coast = coastAngle(wz, lag_s, decel_rad_s2);
  if (!(coast > 0.0)) {
    return error_rad;
  }
  const double dir = (wz > 0.0) ? 1.0 : -1.0;      // wz==0 已被上面挡住
  return error_rad - dir * coast;
}

double alignAngularVelocityWithInertia(
  double error_rad, double wz, double kp, double max_vel, double floor_vel, double tol_rad,
  double lag_s, double decel_rad_s2)
{
  const double e_pred = predictedHeadingError(error_rad, wz, lag_s, decel_rad_s2);
  if (std::fabs(e_pred) <= tol_rad) {
    return 0.0;                  // 剩下的角度正好被惯性吃掉：现在就松手
  }
  if (e_pred * error_rad <= 0.0) {
    return 0.0;
  }
  return alignAngularVelocity(e_pred, kp, max_vel, floor_vel, tol_rad);
}

bool headingSettled(double error_rad, double wz, double tol_rad, double settled_wz)
{
  if (!(std::fabs(error_rad) <= tol_rad)) {
    return false;                // NaN 走这一支：判"没到位"，保守方向
  }
  if (!(settled_wz > 0.0)) {
    return true;
  }
  return std::fabs(wz) <= settled_wz;
}

Phase advancePhase(
  Phase current,
  double start_error_rad,
  double goal_error_rad,
  double dist_to_goal_m,
  double xy_tol_m,
  double align_tol_rad,
  double start_min_rad,
  bool align_goal_enabled)
{
  return advancePhase(
    current, start_error_rad, goal_error_rad, dist_to_goal_m, xy_tol_m, align_tol_rad,
    start_min_rad, align_goal_enabled, 0.0, std::numeric_limits<double>::infinity());
}

Phase advancePhase(
  Phase current,
  double start_error_rad,
  double goal_error_rad,
  double dist_to_goal_m,
  double xy_tol_m,
  double align_tol_rad,
  double start_min_rad,
  bool align_goal_enabled,
  double wz,
  double settled_wz)
{
  switch (current) {
    case Phase::kAlignStart:
      if (!needsStartAlign(start_error_rad, start_min_rad) ||
        std::fabs(start_error_rad) <= align_tol_rad)
      {
        return Phase::kFollow;
      }
      return Phase::kAlignStart;

    case Phase::kCornerApproach:
    case Phase::kAlignCorner:
      // Corner phases are driven by ThreePhaseController's explicit corner
      // state machine; the generic goal phase function must never collapse
      // them into FOLLOW or DONE.
      return current;

    case Phase::kFollow:
      if (dist_to_goal_m > xy_tol_m) {
        return Phase::kFollow;
      }
      if (!align_goal_enabled) {
        return Phase::kDone;
      }
      return headingSettled(goal_error_rad, wz, align_tol_rad, settled_wz) ?
             Phase::kDone : Phase::kAlignGoal;

    case Phase::kAlignGoal:
      if (!align_goal_enabled) {
        return Phase::kDone;
      }
      return headingSettled(goal_error_rad, wz, align_tol_rad, settled_wz) ?
             Phase::kDone : Phase::kAlignGoal;

    case Phase::kDone:
      if (dist_to_goal_m > xy_tol_m) {
        return Phase::kFollow;
      }
      if (align_goal_enabled &&
        !headingSettled(goal_error_rad, wz, align_tol_rad, settled_wz))
      {
        return Phase::kAlignGoal;
      }
      return Phase::kDone;
  }
  return Phase::kDone;
}

double approachSpeedCap(
  double dist_to_goal_m, double approach_dist_m, double v_min, double nominal_speed)
{
  if (!(approach_dist_m > 0.0) || !(v_min >= 0.0) || !(nominal_speed > 0.0)) {
    return nominal_speed;
  }
  if (!(dist_to_goal_m < approach_dist_m)) {
    return nominal_speed;
  }
  const double d = std::max(0.0, dist_to_goal_m);
  const double linear = nominal_speed * (d / approach_dist_m);
  return std::min(std::max(linear, v_min), nominal_speed);
}

bool shouldRestartPhaseTimer(Phase before, Phase requested, bool is_new_goal)
{
  if (before != requested) {
    return true;                 // 相位变了，计时器天然要从头算
  }
  return is_new_goal;
}

bool isFreshFollowAttempt(bool has_prev_tick, double idle_gap_sec, double gap_threshold_sec)
{
  if (!has_prev_tick) {
    return true;                 // 从没 tick 过 = 这是第一次下发，当然是新尝试
  }
  if (!(gap_threshold_sec > 0.0)) {
    return false;
  }
  return idle_gap_sec > gap_threshold_sec;
}

}  // namespace astribot_s1_path_tracking
