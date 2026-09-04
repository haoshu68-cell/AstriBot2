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
  bool saturated_stalled,
  bool already_engaged,
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

  // ④ 足迹没碰致命带 ⇒ **通常**说明 MPPI 有正常梯度，不需要接管。
  //
  //    🔴 例外（2026-09-03 实测加上）：足迹恒 253 而 254 永不出现的那一类通道里，
  //    「没碰 254」并不意味着 MPPI 有梯度 —— 多边形最外侧格全在膨胀致命带里，
  //    代价场是平的，方向信息只能来自路径。这一类通道用 254/253 两个硬判据
  //    在原理上都测不到，只能靠**实测无进展**。详见 NarrowTriggerConfig
  //    里 saturation_threshold 上方那段实测数据与量级比较。
  if (footprint_cost < cfg.footprint_lethal_threshold) {
    const bool saturated = footprint_cost >= cfg.saturation_threshold;
    if (!saturated) {
      // 真的脱离饱和带了 ⇒ 代价场重新有梯度，本层没有存在理由。
      return NarrowVerdict::kNone;
    }
    // 🔴 例外五（入口判据 ≠ 驾驶判据，2026-09-03 实测加上）：
    //    已接管期间**不许再问 saturated_stalled**。
    //
    //    saturated_stalled 是「6s 窗口内推进 < narrow_saturation_min_move」，
    //    每拍重算。本层 v_along = 0.10m/s、min_move = 0.15m：
    //        0.15 / 0.10 = 1.5s
    //    ⇒ 本层一开始走，1.5 秒后就用**自己的进展**把自己的授权吊销，
    //      这一拍立刻掉回 kNone、方向盘交回 MPPI，MPPI 把车推回坡脚，
    //      6s 后卡住判据重新成立、再接管 —— 自激，且无法靠调参消除
    //      （min_move 调大就等于要求本层比它自己更慢）。
    //
    //    实测（51.3s 一次接管、1024 帧 /cmd_vel）：
    //      · 中位 |wz| = 0.0287，而 kp_yaw*err = 1.5*0.44 = 0.66 早已饱和到
    //        wz_max = 0.20 ⇒ 本层只要在下令就必然是 0.20；
    //      · |wz| 落在 0.19~0.21 的帧仅 12.1%；
    //      · **105 帧 |vx| > 0.10、32 帧 |wz| > 0.21，超过本层硬上限 ⇒
    //        这些帧不可能是本层发的**（这是判"谁在开车"的硬证据，
    //        比任何日志里的"已接管"字样都可靠）；
    //      · 累计转角 3.728rad vs 净转角 1.311rad ⇒ 65% 的转动在来回抵消，
    //        于是 nav2 的 PoseProgressChecker 如实报 Failed to make progress
    //        （净位姿几乎不变，转角也够不到 required_movement_angle=0.10）。
    //
    //    这是同一个结构性错误的第三次：例外三对齐了阈值、例外四对齐了变量，
    //    这一次是**把"要不要开始"的条件当成了"要不要继续"的条件**。
    //    接管一旦成立，每拍要问的是「我还在饱和带里吗」（上面那个 saturated），
    //    退出只由最短驻留 + 连续 N 拍脱离决定。
    if (already_engaged) {
      return NarrowVerdict::kNarrow;
    }
    if (!saturated_stalled) {
      return NarrowVerdict::kNone;
    }
    // 落到这里：代价场饱和 + 内层在 FOLLOW 相位下实测没推进 ⇒ 沿路径接管。
    return NarrowVerdict::kNarrow;
  }

  // ⑤ 中心可站 + 足迹碰致命带 ⇒ 正是 0.388 ≤ clearance < 0.42 那一档。
  return NarrowVerdict::kNarrow;
}

bool updateSaturationStall(
  bool saturated,
  bool following,
  double x,
  double y,
  double now_sec,
  const NarrowTriggerConfig & cfg,
  NarrowStallState & state)
{
  // 不在饱和带、或不在 FOLLOW 相位 ⇒ 窗口关闭并清空。
  // 必须是「清空」而不是「暂停」：暂停会把「走一段-停一段」累计成假卡住。
  if (!saturated || !following) {
    state = NarrowStallState{};
    return false;
  }

  if (!state.has_anchor) {
    state.has_anchor = true;
    state.anchor_x = x;
    state.anchor_y = y;
    state.anchor_sec = now_sec;
    state.stalled = false;
    return false;
  }

  const double moved = std::hypot(x - state.anchor_x, y - state.anchor_y);
  if (moved >= cfg.saturation_min_move_m) {
    // 推进过 ⇒ 以当前位姿重开窗口。内层还在工作，本层不插手。
    state.anchor_x = x;
    state.anchor_y = y;
    state.anchor_sec = now_sec;
    state.stalled = false;
    return false;
  }

  // 时钟回跳（sim time 重置 / bag 循环）不能算成"卡了很久"。
  if (now_sec < state.anchor_sec) {
    state.anchor_sec = now_sec;
    state.stalled = false;
    return false;
  }

  state.stalled = (now_sec - state.anchor_sec) >= cfg.saturation_stall_sec;
  return state.stalled;
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

double saturationAlignBoundSec(double favorable_period_rad, double wz_max)
{
  // 到最近有利朝向的误差不会超过半个周期；以 wz_max 转过去就要这么久。
  return (favorable_period_rad / 2.0) / std::max(wz_max, 1e-6);
}

bool saturationDwellHolding(
  bool engaged, bool via_saturation, double engaged_sec, double min_dwell_sec)
{
  if (!engaged || !via_saturation) {
    return false;   // 未接管 / 不是这条入口进来的 ⇒ 与本规则无关
  }
  if (engaged_sec < 0.0) {
    return false;   // 时钟回跳：宁可放行，也不要因为负数把驻留拖成无限
  }
  return engaged_sec < min_dwell_sec;
}

bool narrowShouldDisengage(int clear_hits, int need, bool dwell_holding)
{
  // 驻留优先：入口问的是「有没有推进」，出口在给足推进时间之前不许只问代价。
  if (dwell_holding) {
    return false;
  }
  return narrowCleared(clear_hits, need);
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

namespace
{
/// 沿路径从 path[start] 起向前走 arc_m 弧长，输出落点。
///
/// @return **是否走满了 arc_m**。走到路径终点还没走满就返回 false ——
///         这一位必须区分，否则"路径不够长"会被当成"该处很开阔"（静默假阴性）。
bool advanceAlongPath(
  const std::vector<PlanarPoint> & path,
  std::size_t start,
  double arc_m,
  PlanarPoint & out)
{
  if (start >= path.size()) {
    return false;
  }
  out = path[start];
  double remain = arc_m;
  for (std::size_t i = start; i + 1U < path.size(); ++i) {
    const double seg = std::hypot(path[i + 1U].x - out.x, path[i + 1U].y - out.y);
    if (seg >= remain) {
      const double t = (seg > 1e-12) ? (remain / seg) : 0.0;
      out.x += (path[i + 1U].x - out.x) * t;
      out.y += (path[i + 1U].y - out.y) * t;
      return true;
    }
    remain -= seg;
    out = path[i + 1U];
  }
  return false;      // 走到终点仍没走满
}

/// 路径上距 robot 最近的顶点下标。
std::size_t nearestIndex(const std::vector<PlanarPoint> & path, const PlanarPoint & robot)
{
  std::size_t near = 0U;
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0U; i < path.size(); ++i) {
    const double d = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
    if (d < best) {
      best = d;
      near = i;
    }
  }
  return near;
}
}  // namespace

StrategyPreview previewNarrowStrategy(
  const std::vector<PlanarPoint> & path,
  const PlanarPoint & robot,
  double robot_yaw,
  const PrealignConfig & cfg,
  const FootprintCostFn & cost_default,
  const FootprintCostFn & cost_narrow)
{
  StrategyPreview r;

  // 参数非法一律拒绝，不静默回落到"看起来正常"的默认值。
  if (path.size() < 2U || !cost_default ||
    !(cfg.preview_m > 0.0) || !(cfg.sample_step_m > 0.0) ||
    cfg.sample_step_m > cfg.preview_m ||
    !(cfg.tangent_lookahead_m > 0.0) || !(cfg.favorable_period_rad > 0.0))
  {
    r.strategy = NarrowStrategy::kInvalidConfig;
    return r;
  }

  const std::size_t near = nearestIndex(path, robot);
  bool blocked = false;

  for (double s = cfg.sample_step_m; s <= cfg.preview_m + 1e-9; s += cfg.sample_step_m) {
    PlanarPoint p;
    if (!advanceAlongPath(path, near, s, p)) {
      break;                        // 路径到头，前视结束
    }
    ++r.samples;

    // 该采样点处的通道方向 = 从 p 再往前 tangent_lookahead_m 的方向。
    // 用「同一条路径上再往前一段」而不是相邻两点：相邻点间距可能只有几毫米，
    // 方向会被噪声主导（与 corridorHeadingFromPath 同一理由）。
    PlanarPoint q;
    if (!advanceAlongPath(path, near, s + cfg.tangent_lookahead_m, q)) {
      // 前视够、但切向前视不够 ⇒ 这一点估不出方向。跳过并计数，
      // 不拿一个凑出来的方向去算有利朝向。
      ++r.skipped_no_tangent;
      continue;
    }
    const double dx = q.x - p.x;
    const double dy = q.y - p.y;
    if (std::hypot(dx, dy) < 1e-9) {
      ++r.skipped_no_tangent;
      continue;
    }
    const double corridor_heading = std::atan2(dy, dx);

    // ---- ① 保持当前朝向走到 p 会不会压致命带 ----
    if (cost_default(p.x, p.y, robot_yaw) < cfg.footprint_lethal_threshold) {
      continue;                     // 这一点照现在的姿态就过得去
    }
    const double yaw_error =
      favorableYawError(robot_yaw, corridor_heading, cfg.favorable_period_rad);
    const double favorable_yaw = robot_yaw - yaw_error;

    // ---- ② 默认足迹转到有利朝向后过不过得去 ----
    if (cost_default(p.x, p.y, favorable_yaw) < cfg.footprint_lethal_threshold) {
      r.strategy = NarrowStrategy::kAlignOnly;
      r.target_yaw = favorable_yaw;
      r.corridor_heading = corridor_heading;
      r.at_distance_m = s;
      return r;
    }

    // ---- ③ 换成小足迹、同样转到有利朝向 ----
    if (cost_narrow && cost_narrow(p.x, p.y, favorable_yaw) < cfg.footprint_lethal_threshold) {
      r.strategy = NarrowStrategy::kAlignThenShrink;
      r.target_yaw = favorable_yaw;
      r.corridor_heading = corridor_heading;
      r.at_distance_m = s;
      return r;
    }

    // 两种足迹转正都过不去 ⇒ 它挡在前面，后面的点无意义。
    blocked = true;
    break;
  }

  if (r.samples == 0U) {
    r.strategy = NarrowStrategy::kPathTooShort;
  } else {
    r.strategy = blocked ? NarrowStrategy::kBlocked : NarrowStrategy::kNone;
  }
  return r;
}

PrealignPreview previewFavorableAlignment(
  const std::vector<PlanarPoint> & path,
  const PlanarPoint & robot,
  double robot_yaw,
  const PrealignConfig & cfg,
  const FootprintCostFn & cost_fn)
{
  // 只有一份实现：本函数是 previewNarrowStrategy 在「没有小足迹」下的投影。
  // 复制一份判据必然与被测的那份漂开，所以这里只做枚举映射。
  const StrategyPreview s =
    previewNarrowStrategy(path, robot, robot_yaw, cfg, cost_fn, FootprintCostFn{});
  PrealignPreview r;
  r.target_yaw = s.target_yaw;
  r.at_distance_m = s.at_distance_m;
  r.samples = s.samples;
  r.skipped_no_tangent = s.skipped_no_tangent;
  switch (s.strategy) {
    case NarrowStrategy::kAlignOnly:
      r.verdict = PrealignVerdict::kNeeded;
      break;
    case NarrowStrategy::kBlocked:
      r.verdict = PrealignVerdict::kBlockedEvenFavorable;
      break;
    case NarrowStrategy::kPathTooShort:
      r.verdict = PrealignVerdict::kPathTooShort;
      break;
    case NarrowStrategy::kInvalidConfig:
      r.verdict = PrealignVerdict::kInvalidConfig;
      break;
    case NarrowStrategy::kAlignThenShrink:
      // 传了空的 cost_narrow，这个分支不可能出现。真出现就是实现被改坏了，
      // 按「不需要预对齐」处理并不安全，所以映射成"前方堵死"这一保守侧。
      r.verdict = PrealignVerdict::kBlockedEvenFavorable;
      break;
    case NarrowStrategy::kNone:
    default:
      r.verdict = PrealignVerdict::kClearAhead;
      break;
  }
  return r;
}

double lateralHalfExtent(const std::vector<PlanarPoint> & footprint, double delta_yaw)
{
  if (footprint.size() < 3U) {
    return 0.0;      // 退化多边形算不出包络。返回 0 会让调用方的"更窄"断言失败，
                     // 这是刻意的：宁可拒绝切换，也不要拿一个凑出来的数放行。
  }
  const double sd = std::sin(delta_yaw);
  const double cd = std::cos(delta_yaw);
  double worst = 0.0;
  for (const auto & v : footprint) {
    worst = std::max(worst, std::fabs(v.x * sd + v.y * cd));
  }
  return worst;
}

bool periodPreservesLateralExtent(
  const std::vector<PlanarPoint> & footprint, double period_rad, double tol_m)
{
  if (footprint.size() < 3U || !(period_rad > 0.0) || !(tol_m > 0.0)) {
    return false;                       // 参数非法一律不放行（fail-safe）
  }
  // 在一个周期内密集取样，逐点比较 d 与 d+period 的侧向半宽。
  // 步长取 1 度：远细于任何朝向闸门，也远细于代价地图能表达的角度分辨率。
  const int kSamples = 360;
  for (int i = 0; i < kSamples; ++i) {
    const double d = (2.0 * M_PI * static_cast<double>(i)) / static_cast<double>(kSamples);
    const double a = lateralHalfExtent(footprint, d);
    const double b = lateralHalfExtent(footprint, d + period_rad);
    if (std::fabs(a - b) > tol_m) {
      return false;
    }
  }
  return true;
}

bool sweepClearForRotation(
  const PlanarPoint & at,
  double from_yaw,
  double to_yaw,
  double step_rad,
  double lethal_threshold,
  const FootprintCostFn & cost_fn,
  double & worst_cost)
{
  worst_cost = 0.0;
  if (!(step_rad > 0.0) || !cost_fn) {
    return false;      // 参数非法 ⇒ 不放行旋转（失败偏安全侧）
  }
  // 取最近方向旋转，|d| <= pi，与 favorableYawError 的同余类语义一致。
  const double d = normalizeAngle(to_yaw - from_yaw);
  const int n = std::max(1, static_cast<int>(std::ceil(std::fabs(d) / step_rad)));
  bool clear = true;
  for (int i = 0; i <= n; ++i) {
    const double yaw = from_yaw + d * (static_cast<double>(i) / static_cast<double>(n));
    const double c = cost_fn(at.x, at.y, yaw);
    worst_cost = std::max(worst_cost, c);
    if (c >= lethal_threshold) {
      clear = false;
      break;           // 已经否决，但 worst_cost 已填好供日志用
    }
  }
  return clear;
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
