// Copyright 2026 Astribot.
#ifndef ASTRIBOT_S1_AUTONOMY__COSTMAP_ADAPTER_HPP_
#define ASTRIBOT_S1_AUTONOMY__COSTMAP_ADAPTER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "astribot_s1_autonomy/frontier_search.hpp"   // 复用 GridMap

namespace astribot_s1_autonomy
{

/// nav2 代价值常量。刻意在本包内复述一份而不 include nav2_costmap_2d，
/// 是为了让这一层保持「无 ROS / 无 Nav2 依赖」从而可离线单测；
/// 数值来源：nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp。
struct Nav2CostValues
{
  static constexpr uint8_t kNoInformation = 255U;
  static constexpr uint8_t kLethalObstacle = 254U;
  static constexpr uint8_t kInscribedInflatedObstacle = 253U;
  static constexpr uint8_t kFreeSpace = 0U;
};

/// 转换参数。全部来自 YAML，便于现场调阈值而不用重编译。
struct CostmapAdapterParams
{
  /// 视为「未知」的代价值。nav2 固定用 255，做成参数只为极端情况下可绕开。
  int unknown_cost{Nav2CostValues::kNoInformation};
  /// 视为「致命障碍」的下界（含）。默认 253 = INSCRIBED_INFLATED_OBSTACLE，
  /// 语义正好是「机器人中心落在此处，足迹必然碰撞」，与校验职责精确对应。
  ///
  /// 不要为了「更安全」把它调低：调低等于把可通行的膨胀带判成障碍，
  /// 会让贴墙路径全被否决，探索重新锁死。真要更保守应该去调
  /// inflation_layer 的 inflation_radius，让 costmap 自己把致命区扩大。
  int lethal_cost_threshold{Nav2CostValues::kInscribedInflatedObstacle};
};

/// 转换失败时填充原因，便于日志追溯。成功返回 true。
bool costmapToGridMap(
  unsigned int size_x,
  unsigned int size_y,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<uint8_t> & data,
  const CostmapAdapterParams & params,
  GridMap & out,
  std::string & error);

/// 参数自检。非法组合在节点启动时就拦住，不要等到运行期。
bool validateCostmapAdapterParams(const CostmapAdapterParams & params, std::string & error);

/// 转换结果的分布统计，只用于日志/诊断（探索发不出目标时第一手排查依据）。
struct CostmapGridStats
{
  std::size_t unknown{0U};
  std::size_t lethal{0U};
  std::size_t free{0U};
  std::size_t total{0U};
  double unknownRatio() const
  {
    return total == 0U ? 0.0 : static_cast<double>(unknown) / static_cast<double>(total);
  }
  double lethalRatio() const
  {
    return total == 0U ? 0.0 : static_cast<double>(lethal) / static_cast<double>(total);
  }
};

/// 统计一张已转换的 GridMap 的三态分布。
CostmapGridStats summarizeGrid(const GridMap & grid);

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__COSTMAP_ADAPTER_HPP_
