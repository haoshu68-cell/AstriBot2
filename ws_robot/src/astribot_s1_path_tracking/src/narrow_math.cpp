// Copyright 2026 Astribot
//
// narrow_math 的实现。设计依据全部在头文件，这里只放实现细节注释。

#include "astribot_s1_path_tracking/narrow_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace astribot_s1_path_tracking
{

const char * toString(NarrowVerdict v)
{
  switch (v) {
    case NarrowVerdict::kNone:
      return "NONE";
    case NarrowVerdict::kNarrow:
      return "NARROW";
    case NarrowVerdict::kCenterLethal:
      return "CENTER_LETHAL";
    case NarrowVerdict::kPhysicallyBlocked:
      return "PHYSICALLY_BLOCKED";
    case NarrowVerdict::kNoPath:
      return "NO_PATH";
  }
  return "UNKNOWN";
}

NarrowVerdict evaluateNarrowTrigger(
  double center_cost,
  double footprint_cost,
  double favorable_cost,
  bool have_path,
  const NarrowTriggerConfig & cfg)
{
  // ① 🔴 红线最先判：**转到最有利朝向后仍然**压到真障碍(254)。
  //    253 可以贴（膨胀语义），254 不可以（真障碍）。混淆一次就是安全事故，
  //    所以这个判断放在最前面，不受任何其它条件影响。
  //
  //    用 favorable_cost 而不是 footprint_cost：正八边形顶点比边中点多伸出
  //    0.034m，顶点朝墙时压 254、边朝墙时过得去。用当前朝向判会让本层
  //    在自己的目标域(0.772~0.840m 通道)上把自己否掉 —— 实测已发生：
  //    中心代价 0 -> 218 -> 229 -> 致命，车被一路推进膨胀带深处。
  //    完整推导见头文件。
  if (favorable_cost >= NarrowCostValues::kLethal) {
    return NarrowVerdict::kPhysicallyBlocked;
  }

  // ② 中心格已致命 ⇒ 机器人中心在此则足迹必然碰撞 ⇒ 物理放不进去。
  //    这是 clearance < 0.388 那一档（占本地图自由域 31.26%），
  //    正确动作是把机器人挪出来（协调器 ESCAPE），不是在这儿贴边硬挤。
  //    本层如实上报、拒绝接管 —— 静默接管会让机器人在放不进去的地方蹭。
  if (center_cost >= cfg.center_lethal_threshold) {
    return NarrowVerdict::kCenterLethal;
  }

  // ③ 本层以全局路径为核心约束，无路径不接管。
  if (!have_path) {
    return NarrowVerdict::kNoPath;
  }

  // ④ 足迹没碰致命带 ⇒ MPPI 有正常梯度，不需要接管。
  if (footprint_cost < cfg.footprint_lethal_threshold) {
    return NarrowVerdict::kNone;
  }

  // ⑤ 中心可站 + 足迹碰致命带 ⇒ 正是 0.388 ≤ clearance < 0.42 那一档。
  return NarrowVerdict::kNarrow;
}

double favorableYawError(double yaw, double corridor_heading, double period_rad)
{
  if (!(period_rad > 0.0)) {
    // 非法周期按「不做朝向修正」处理，而不是除零。
    return 0.0;
  }
  // 车体朝向相对通道方向的角度。正八边形每 period 复现相同横向包络，
  // 所以有利朝向是一个同余类，误差要归一到最近的 period 整数倍。
  const double d = normalizeAngle(yaw - corridor_heading);
  const double k = std::round(d / period_rad);
  double err = d - (k * period_rad);
  // 数值上钳到 [-period/2, period/2]，防止 round 边界处溢出半格。
  const double half = period_rad * 0.5;
  err = std::max(-half, std::min(half, err));
  return err;
}

LateralScanResult scanLateral(
  const PlanarPoint & robot,
  double yaw,
  double corridor_heading,
  double half_width_m,
  double step_m,
  const FootprintCostFn & cost_fn)
{
  LateralScanResult out;
  if (!cost_fn || !(half_width_m > 0.0) || !(step_m > 0.0)) {
    return out;                        // valid 保持 false
  }

  // 横向单位向量 = 通道方向逆时针 90°。
  const double lx = -std::sin(corridor_heading);
  const double ly = std::cos(corridor_heading);

  const int n = static_cast<int>(std::floor(half_width_m / step_m));
  double best_cost = 0.0;
  double best_off = 0.0;
  bool have_best = false;
  double min_cost = 0.0;
  double max_cost = 0.0;
  bool first = true;

  for (int i = -n; i <= n; ++i) {
    const double off = static_cast<double>(i) * step_m;
    const double x = robot.x + (lx * off);
    const double y = robot.y + (ly * off);
    const double c = cost_fn(x, y, yaw);

    // 真障碍与未知一律不可用。未知不等于可通行 ——
    // 本项目已在别处踩过「无数据当成安全」这一类错误。
    if (c >= NarrowCostValues::kLethal) {
      continue;
    }
    ++out.usable_count;

    if (first) {
      min_cost = c;
      max_cost = c;
      first = false;
    } else {
      min_cost = std::min(min_cost, c);
      max_cost = std::max(max_cost, c);
    }

    // 同代价时优先取**横向偏移小**的：无谓的横移在窄通道里是纯风险。
    if (!have_best || c < best_cost ||
      (std::fabs(c - best_cost) < 1e-9 && std::fabs(off) < std::fabs(best_off)))
    {
      best_cost = c;
      best_off = off;
      have_best = true;
    }
  }

  if (!have_best) {
    return out;                        // 两侧全被真障碍堵住
  }
  out.valid = true;
  out.best_offset_m = best_off;
  out.best_cost = best_cost;
  // 饱和判定：所有可用样本代价相同 ⇒「最低代价中线」无意义。
  // 此时调用方应放弃横向寻优、纯沿路径走，而不是把噪声当梯度追 ——
  // 实测本项目就是在饱和区把 [253] 当成了有梯度的量。
  out.saturated = (max_cost - min_cost) < 1e-9;
  if (out.saturated) {
    out.best_offset_m = 0.0;           // 饱和 ⇒ 不横移
  }
  return out;
}

bool yawGateHolding(double yaw_error, double yaw_gate_rad)
{
  return std::fabs(yaw_error) > yaw_gate_rad;
}

NarrowCommand narrowVelocity(
  double yaw,
  double corridor_heading,
  double lateral_offset_m,
  double yaw_error,
  const NarrowLimits & lim)
{
  NarrowCommand cmd;

  // 朝向修正：误差为正表示车体相对有利朝向偏正，wz 取其相反数。
  const double wz_raw = -lim.kp_yaw * yaw_error;
  cmd.wz = std::max(-lim.wz_max, std::min(lim.wz_max, wz_raw));

  // ⚠️ 朝向门：这是本层最关键的一条。
  // 在 0.776~0.84 的通道里，朝向不对就是过不去；带着错的朝向往前走
  // 等于朝卡死里走。所以朝向误差超门限时**只转不走**。
  //
  // 判据走 yawGateHolding()：卡住判据也要问同一个问题（这一拍是不是在
  // 按指令转向），两处必须是同一个函数，不能各写一遍。
  if (yawGateHolding(yaw_error, lim.yaw_gate_rad)) {
    cmd.holding_for_yaw = true;
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    return cmd;
  }

  // 世界系速度 = 沿通道前进 + 横向修正，再转到车体系。
  const double v_lat_raw = lim.kp_lateral * lateral_offset_m;
  const double v_lat = std::max(-lim.v_lateral_max, std::min(lim.v_lateral_max, v_lat_raw));

  const double world_vx = (lim.v_along * std::cos(corridor_heading)) +
    (v_lat * -std::sin(corridor_heading));
  const double world_vy = (lim.v_along * std::sin(corridor_heading)) +
    (v_lat * std::cos(corridor_heading));

  // world → body（绕 z 转 -yaw）
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  cmd.vx = (world_vx * c) + (world_vy * s);
  cmd.vy = (-world_vx * s) + (world_vy * c);
  return cmd;
}

bool corridorHeadingFromPath(
  const std::vector<PlanarPoint> & path,
  const PlanarPoint & robot,
  double lookahead_m,
  double & heading)
{
  if (path.size() < 2U || !(lookahead_m > 0.0)) {
    return false;
  }
  // 最近顶点
  std::size_t near = 0U;
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0U; i < path.size(); ++i) {
    const double d = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
    if (d < best) {
      best = d;
      near = i;
    }
  }
  // 从最近顶点起累积弧长到 lookahead_m
  double acc = 0.0;
  std::size_t j = near;
  for (std::size_t i = near; i + 1U < path.size(); ++i) {
    acc += std::hypot(path[i + 1U].x - path[i].x, path[i + 1U].y - path[i].y);
    j = i + 1U;
    if (acc >= lookahead_m) {
      break;
    }
  }
  if (j == near) {
    return false;                      // 最近点已是终点，没有前向可言
  }
  const double dx = path[j].x - path[near].x;
  const double dy = path[j].y - path[near].y;
  if (std::hypot(dx, dy) < 1e-9) {
    return false;                      // 路径退化成一点
  }
  heading = std::atan2(dy, dx);
  return true;
}

bool narrowCleared(int consecutive_clear_ticks, int need)
{
  if (need <= 0) {
    // need<=0 是配置错误。按「永不判脱离」处理而不是「立刻脱离」：
    // 后者会让接管一拍就退出，等于本层从未生效且不报错。
    return false;
  }
  return consecutive_clear_ticks >= need;
}

void updateNarrowFailureCount(bool cleared_through, int & count)
{
  if (cleared_through) {
    count = 0;      // 穿过去了 ⇒ 之前的失败不再连续
    return;
  }
  if (count < 0) {
    count = 0;      // 防御：负值没有意义
  }
  ++count;
}

bool shouldGiveUpNarrow(int consecutive_failures, int max_attempts, bool already_gave_up)
{
  if (already_gave_up) {
    return false;   // 已经放弃过一次，不重复
  }
  if (max_attempts < 1) {
    // <1 是配置错误。按「立刻放弃」处理而不是「永不放弃」：
    // 永不放弃会让「不能无限循环脱困」这条红线彻底失守。
    return true;
  }
  return consecutive_failures >= max_attempts;
}

bool pathArcProgress(
  const std::vector<PlanarPoint> & path, const PlanarPoint & robot, double & progress_m)
{
  if (path.size() < 2U) {
    return false;
  }
  // 找最近的线段，进度 = 路径起点到该段上投影点的累计弧长。
  double best_d_sq = std::numeric_limits<double>::max();
  double best_arc = 0.0;
  double arc = 0.0;
  for (std::size_t i = 0U; i + 1U < path.size(); ++i) {
    const double ax = path[i].x;
    const double ay = path[i].y;
    const double vx = path[i + 1U].x - ax;
    const double vy = path[i + 1U].y - ay;
    const double seg = std::hypot(vx, vy);
    const double vv = (vx * vx) + (vy * vy);
    double t = 0.0;
    if (vv > 0.0) {
      t = std::max(0.0, std::min(1.0, (((robot.x - ax) * vx) + ((robot.y - ay) * vy)) / vv));
    }
    const double px = ax + (t * vx);
    const double py = ay + (t * vy);
    const double d_sq = ((robot.x - px) * (robot.x - px)) + ((robot.y - py) * (robot.y - py));
    if (d_sq < best_d_sq) {
      best_d_sq = d_sq;
      best_arc = arc + (t * seg);
    }
    arc += seg;
  }
  progress_m = best_arc;
  return true;
}

bool pathRemainingArc(
  const std::vector<PlanarPoint> & path, const PlanarPoint & robot, double & remaining_m)
{
  if (path.size() < 2U) {
    return false;
  }
  // 先找最近线段与段内投影参数，再把「该段剩余部分 + 其后所有段」加起来。
  // 与 pathArcProgress 的区别只在参考点：那边从起点累计，这边到终点累计。
  // 换路径时前者的原点会跳，后者不会 —— 这正是改用它的原因。
  double best_d_sq = std::numeric_limits<double>::max();
  std::size_t best_i = 0U;
  double best_t = 0.0;
  for (std::size_t i = 0U; i + 1U < path.size(); ++i) {
    const double ax = path[i].x;
    const double ay = path[i].y;
    const double vx = path[i + 1U].x - ax;
    const double vy = path[i + 1U].y - ay;
    const double vv = (vx * vx) + (vy * vy);
    double t = 0.0;
    if (vv > 0.0) {
      t = std::max(0.0, std::min(1.0, (((robot.x - ax) * vx) + ((robot.y - ay) * vy)) / vv));
    }
    const double px = ax + (t * vx);
    const double py = ay + (t * vy);
    const double d_sq = ((robot.x - px) * (robot.x - px)) + ((robot.y - py) * (robot.y - py));
    if (d_sq < best_d_sq) {
      best_d_sq = d_sq;
      best_i = i;
      best_t = t;
    }
  }
  // 最近段上「投影点 → 段末」那一截
  const double sx = path[best_i + 1U].x - path[best_i].x;
  const double sy = path[best_i + 1U].y - path[best_i].y;
  double rem = (1.0 - best_t) * std::hypot(sx, sy);
  // 其后所有整段
  for (std::size_t i = best_i + 1U; i + 1U < path.size(); ++i) {
    rem += std::hypot(path[i + 1U].x - path[i].x, path[i + 1U].y - path[i].y);
  }
  remaining_m = rem;
  return true;
}

bool pathWasReplaced(
  const std::vector<PlanarPoint> & path, PlanarPoint & last_end, std::size_t & last_size)
{
  if (path.empty()) {
    return false;                       // 空路径不算"换了"，交给上游的退化判据
  }
  const PlanarPoint & end = path.back();
  const bool first_time = (last_size == 0U);
  // 终点挪动超过 0.05m 就认为换了目标/换了路径。
  // 阈值取一个栅格量级：小于它的差异是重规划的顶点抖动，不该重开窗口
  // （否则每拍都重开，等于把卡住判据关掉）。
  const double moved = std::hypot(end.x - last_end.x, end.y - last_end.y);
  const bool changed = first_time || (moved > 0.05);
  last_end = end;
  last_size = path.size();
  return changed && !first_time;
}

bool narrowStalled(
  double remaining_m,
  double now_sec,
  double min_gain_m,
  double stall_timeout_sec,
  NarrowProgressState & state)
{
  if (!state.initialized) {
    state.initialized = true;
    state.best_remaining_m = remaining_m;
    state.last_gain_sec = now_sec;
    return false;
  }
  // 有进展 = 剩余弧长**减少**了至少 min_gain_m。注意方向：这里是 <，
  // 上一版用的是「累计进度 >」，改量的时候方向必须跟着翻，否则判据恒真/恒假。
  if (remaining_m < state.best_remaining_m - min_gain_m) {
    state.best_remaining_m = remaining_m;
    state.last_gain_sec = now_sec;
    return false;
  }
  if (stall_timeout_sec <= 0.0) {
    // <=0 是配置错误。按「立刻判卡住」处理而不是「永不判」：
    // 永不判会让「不能无限循环脱困」这条红线失守。
    return true;
  }
  return (now_sec - state.last_gain_sec) > stall_timeout_sec;
}

}  // namespace astribot_s1_path_tracking
