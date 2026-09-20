// Copyright 2026 Astribot.
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
  /// 使用膨胀代价地图时避免重复叠加足迹半径；当前探索 YAML 覆盖为 0.25 m。
  double goal_clearance_radius{0.42};
  /// 目标点【未知】净空半径(m)：该半径内不允许有未知格。
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


/// 路径上距 robot 最近的顶点下标。路径为空时返回 0（调用方必须自己先判空）。
std::size_t nearestPathIndex(const std::vector<PlanarPoint> & path, const PlanarPoint & robot);

/// robot 到路径最近顶点的距离(m)。空路径返回 -1.0，不能视为零偏差。
double pathDeviation(const std::vector<PlanarPoint> & path, const PlanarPoint & robot);

/// 从距 robot 最近的顶点截取剩余路径；空路径返回空 vector。
/// 只校验剩余段，避免身后障碍触发无关的重规划。
std::vector<PlanarPoint> remainingPath(
  const std::vector<PlanarPoint> & path, const PlanarPoint & robot);

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__PATH_VALIDATOR_HPP_
