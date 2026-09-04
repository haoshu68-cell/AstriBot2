// Copyright 2026 Astribot

#include "astribot_s1_path_tracking/align_math.hpp"

#include <algorithm>
#include <cmath>

namespace astribot_s1_path_tracking
{

const char * toString(Phase p)
{
  switch (p) {
    case Phase::kAlignStart:
      return "ALIGN_START";
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
  // atan2(sin, cos) 天然落在 (-pi, pi]，比手写循环减 2pi 稳（不会因大角度死循环）。
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
  // 路径整体退化成一个点（所有顶点重合）时无方向可言 —— 明确失败，
  // 不返回 atan2(0,0)=0 冒充「朝 +x」。
  if (std::hypot(dx, dy) < 1e-9) {
    return false;
  }
  heading = std::atan2(dy, dx);
  return true;
}

bool isSameGoal(const PlanarPoint & prev_end, const PlanarPoint & cur_end, double eps_m)
{
  if (!(eps_m > 0.0)) {
    // 阈值非法时保守判「不同目标」：宁可多做一次起步对齐，
    // 也不要把新目标误当成旧目标而跳过对齐。
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
  // 先压下限再压上限：floor 只保证「能动」，绝不允许因此超过 max。
  double out = std::max(mag, std::fabs(floor_vel));
  out = std::min(out, std::fabs(max_vel));
  return sign * out;
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
  switch (current) {
    case Phase::kAlignStart:
      // 已经对上（或本来就不值得转）就进跟踪段。
      if (!needsStartAlign(start_error_rad, start_min_rad) ||
        std::fabs(start_error_rad) <= align_tol_rad)
      {
        return Phase::kFollow;
      }
      return Phase::kAlignStart;

    case Phase::kFollow:
      if (dist_to_goal_m > xy_tol_m) {
        return Phase::kFollow;
      }
      // 位置到了。探索场景刻意不对齐终点姿态 —— 直接结束。
      if (!align_goal_enabled) {
        return Phase::kDone;
      }
      return (std::fabs(goal_error_rad) <= align_tol_rad) ? Phase::kDone : Phase::kAlignGoal;

    case Phase::kAlignGoal:
      // 防御：本相位在 align_goal_enabled=false 时不该出现；
      // 万一出现（例如运行期改参数），立即收敛到 kDone 而不是继续转。
      if (!align_goal_enabled) {
        return Phase::kDone;
      }
      return (std::fabs(goal_error_rad) <= align_tol_rad) ? Phase::kDone : Phase::kAlignGoal;

    case Phase::kDone:
      // 🔴 kDone **不是吸收态**。这里曾经写成 `return Phase::kDone;`，后果是
      //    机器人到了路径末端附近就永久停车、整段目标跑到超时。
      //
      //    实测证据（远端目标 5.88,-5.86 那一腿）：
      //      t+45.5s  ALIGN_GOAL -> DONE : 本段完成
      //      随后 135s 内 /cmd_vel 共 2589 帧，**非零线速度 0 帧**、max‖v‖=0.0000、
      //      净位移 0.048m；日志里每 ~11s 一轮
      //        Failed to make progress -> Aborting handle -> 清 costmap + spin 恢复
      //        -> Received a goal -> setPlan(same_goal=true) 保持相位 -> 又是零速
      //      共 12 轮，直到调用方 180s 主动取消。日志里 **从未**出现
      //      "Reached the goal!" —— 即 nav2 的 GoalChecker 一次都没判到位。
      //
      //    机制：本控制器的 dist_to_goal 是量到 plan_.poses.back()，只在进入
      //    kDone 的**那一拍**成立。之后 1Hz 重规划换了 135 次路径、恢复行为还把
      //    机器人原地转了（|wz| 到 1.5，那是 behavior_server 不是本层），
      //    机器人早已滑出容差；而 GoalChecker 每拍都在量、每拍都说没到。
      //    两边判据相同、参照点相同，唯一的差别就是**本层锁存了、它没有**。
      //
      //    结论：到位的裁判权在 GoalChecker，本层的 kDone 只是一个意见。
      //    意见的前提（dist <= xy_tol）不再成立时必须撤回，回到 kFollow 继续开。
      //
      //    ⚠️ 退出阈值只能用 xy_tol_m 本身，**不许**为了防抖把它放宽。
      //       放宽到 exit_tol > xy_tol 会造出死区 (xy_tol, exit_tol]：在那一段里
      //       本层认为"还算到了"故停车，GoalChecker 认为"没到"故不结束 ——
      //       正是上面那个死锁原样复现。用同一个阈值时死区为空集。
      //       边界上的抖动是"贴着容差反复轻推"，那是期望行为（推到 GoalChecker
      //       认账为止），且窄通道层的停滞检测仍在兜底；与
      //       yaw 闸门那种"两侧都在开车"的自激不是一回事。
      if (dist_to_goal_m > xy_tol_m) {
        return Phase::kFollow;
      }
      return Phase::kDone;
  }
  return Phase::kDone;
}

bool shouldRestartPhaseTimer(Phase before, Phase requested, bool is_new_goal)
{
  if (before != requested) {
    return true;                 // 相位变了，计时器天然要从头算
  }
  // 相位没变：只有"这是一个新目标"才重置。
  return is_new_goal;
}

}  // namespace astribot_s1_path_tracking
