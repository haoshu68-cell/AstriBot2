// Copyright 2026 Astribot.
//
// 未知区域禁行校验的纯算法核心。
//
// 和本包其它算法件一样，刻意不依赖 ROS 类型（只用 GridMap + 简单 POD），
// 便于脱离仿真做单元测试——「路径穿未知区」这种事一旦漏检，现场表现是机器人
// 一头撞进没扫过的区域，事后很难复盘，所以必须能离线把每条规则都钉死。
//
// ======================== 两层校验 ========================
// 校验1（目标点）：目标格必须已知且空闲，且以它为心的 goal_clearance_radius
//                  邻域内不能有未知格或占据格。
//                  只查单格是不够的——机器人有 0.438m 外接半径，目标格本身空闲
//                  但紧邻未知区时，机器人一停过去半个身子就在未知区里。
//
// 校验2（路径）：把 Nav2 规划出来的路径**逐段插值采样**，每个采样点所在格
//                都必须已知且空闲。
//
//                !!! 这里有一个极易写漏的点 !!!
//                只检查路径的顶点是不完全的：Nav2 返回的 Path 顶点间距可能
//                远大于一个栅格（Smac 的路径点可能几十厘米一个），
//                一小块未知区正好夹在两个合法顶点中间时，只查顶点会完全放过它。
//                所以必须按 <= resolution/2 的步长在段内采样。
// =========================================================
#ifndef ASTRIBOT_S1_AUTONOMY__PATH_VALIDATOR_HPP_
#define ASTRIBOT_S1_AUTONOMY__PATH_VALIDATOR_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "astribot_s1_autonomy/frontier_search.hpp"   // 复用 GridMap

namespace astribot_s1_autonomy
{

/// 平面上一个点（世界坐标，m）。刻意不用 geometry_msgs，保持算法层无 ROS 依赖。
struct PlanarPoint
{
  double x{0.0};
  double y{0.0};
};

/// 校验参数，全部来自 YAML。
struct PathValidatorParams
{
  /// 栅格三态判定阈值，与 FrontierSearch 保持同一套口径。
  int occupied_threshold{65};
  int free_threshold{25};
  /// 目标点【占据】净空半径(m)：该半径内不允许有占据格。
  /// 这是碰撞约束。⚠️ 别按足迹外接半径(本机 0.438)取：代价地图里 >=253 的格
  /// 已经含了足迹半径，再叠一个外接半径等于重复计一次，2026-08 实测会把
  /// 99% 的贴墙前沿误否决。正确取值是控制器的 xy_goal_tolerance(本项目 0.25)。
  /// 这里的默认值只是历史遗留，实际由 yaml 覆盖成 0.25。
  double goal_clearance_radius{0.42};
  /// 目标点【未知】净空半径(m)：该半径内不允许有未知格。
  ///
  /// 必须远小于 goal_clearance_radius，且刻意是独立的一个参数。
  /// 前沿点的定义就是「空闲格且邻域含未知格」——如果拿机器人外接半径去要求
  /// 「周围一个未知格都没有」，所有前沿点都必然不合法，探索在构造上无法进行。
  /// 这里只要求目标点踏实位于已知区内留一点余量；
  /// 「路径不穿未知区」由 validatePath 保证，「目标格本身已知空闲」由单格检查保证。
  double goal_unknown_clearance_radius{0.10};
  /// 路径段内采样步长(m)。<=0 表示自动取 map.resolution/2。
  double path_sample_step{0.0};
  /// 规划终点与请求目标的最大允许偏差(m)。
  /// 防止规划器把不可达目标「尽力靠近」后返回一条半截路径也被当成成功。
  double path_endpoint_tolerance{0.5};
  /// 单条路径最多允许检查的采样点数，防御性上限（禁止无界循环）。
  std::size_t max_samples{200000U};
};

/// 校验结论。失败时带上人类可读原因和出问题的位置，便于日志追溯。
struct ValidationResult
{
  bool valid{false};
  std::string reason;
  /// 首个非法采样点的世界坐标（valid==false 且原因与位置相关时有效）。
  PlanarPoint first_bad_point;
  /// 首个非法点位于路径的第几段（顶点下标），-1 表示与路径无关。
  int bad_segment_index{-1};
  /// 实际检查过的采样点数，便于确认采样密度是否符合预期。
  std::size_t samples_checked{0U};
};

/// 未知区域禁行校验器。无状态，可反复调用。
class PathValidator
{
public:
  PathValidator() = default;

  bool configure(const PathValidatorParams & params, std::string & error);
  const PathValidatorParams & params() const {return params_;}

  /// 单格是否「已知且空闲」。未知(-1)、占据、越界都返回 false。
  bool isKnownFree(const GridMap & map, double wx, double wy) const;

  /// 校验1：目标点合法性（自身已知空闲 + 净空半径内无未知/占据格）。
  ValidationResult validateGoal(const GridMap & map, double gx, double gy) const;

  /// 校验2：路径合法性。path 为世界坐标顶点序列（通常来自 nav_msgs/Path）。
  /// requested_goal 用于校验规划终点没有被截断。
  ValidationResult validatePath(
    const GridMap & map,
    const std::vector<PlanarPoint> & path,
    const PlanarPoint & requested_goal) const;

private:
  /// 计算实际使用的采样步长。
  double effectiveStep(const GridMap & map) const;

  PathValidatorParams params_;
  bool configured_{false};
};

// ============ 跟踪期的纯几何辅助（同样无 ROS 依赖，可离线测） ============
//
// 这三个函数是「未跟踪到位不换路径」判据的算法核心。
//
// 为什么需要它们：原先跟踪期是**无条件**周期重规划（replan_period_sec=1.0），
// 实测 36 个目标共下发 393 次 FollowPath —— 平均每个目标换 10.9 条路径、
// 节拍约 1.5s。也就是说没有任何一条路径被跟踪到位过，每条都在 1.5s 后
// 被下一条抢占。正确判据是「只有当前路径已经不能用了才换」，而
// 「能不能用」需要两个量：机器人是否已偏离这条路径，剩余段是否还可通行。
//
// 注意都用**顶点**距离而不是「点到线段」的垂距：Nav2 全局路径的顶点间距是
// 栅格量级（0.05m），顶点距离与垂距的差别远小于判据阈值（0.6m），
// 而顶点法没有线段退化（零长段）这一类边界情况。

/// 路径上距 robot 最近的顶点下标。路径为空时返回 0（调用方必须自己先判空）。
std::size_t nearestPathIndex(const std::vector<PlanarPoint> & path, const PlanarPoint & robot);

/// robot 到路径最近顶点的距离(m)。
///
/// 路径为空时返回 **-1.0**，不是 0.0：返回 0 会让「根本没有路径」被读成
/// 「完美贴合路径」，从而永远不触发重规划 —— 这正是「无数据当成安全」那一类
/// 错误（本项目已在探针脚本上踩过一次：没收到 scan 被当成前方无障碍）。
double pathDeviation(const std::vector<PlanarPoint> & path, const PlanarPoint & robot);

/// 剩余段：从距 robot 最近的顶点一直到路径终点。路径为空返回空 vector。
///
/// 只校验剩余段而不是整条路径：已经走过的那一段是否仍可通行与「还能不能继续
/// 跟踪」无关，拿整条路径去校验会因为身后新出现的障碍（比如刚被观测到的墙）
/// 而反复误判成需要重规划。
std::vector<PlanarPoint> remainingPath(
  const std::vector<PlanarPoint> & path, const PlanarPoint & robot);

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__PATH_VALIDATOR_HPP_
