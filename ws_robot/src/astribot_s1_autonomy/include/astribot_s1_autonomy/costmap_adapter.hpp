// Copyright 2026 Astribot.
//
// nav2 代价地图（nav2_msgs/Costmap 的原始 0~255 字节栅格）→ 本包 GridMap 的转换。
//
// 和本包其它算法件一样刻意不依赖 ROS 类型：入参是尺寸/分辨率/原点 + 一段
// uint8 数据，节点层负责从消息里把这几样拆出来。这样这层能离线单测。
//
// ==================== 为什么必须有这一层 ====================
// 实测踩坑（631/631 次候选否决，探索下发恒为 0）：
// 「路径校验」和「路径规划」查的必须是**同一张栅格图**，否则两层判据会互相锁死。
//
// 之前的实现是：Nav2 的 planner_server 在 global_costmap 上规划，而本包的
// PathValidator 在原始 /map（slam_toolbox 输出）上校验。两张图的自由区并不相等：
//
//   · global_costmap = static_layer(/map) + obstacle_layer + inflation_layer，
//     其中 obstacle_layer 配了 clearing:True / raytrace_max_range:6.0，
//     会沿**每一条**扫描射线（含无回波的 inf 射线）把栅格刷成 FREE_SPACE，
//     **覆盖掉 NO_INFORMATION**。
//   · /map 由 slam_toolbox(Karto) 产出，而 Karto **丢弃 inf 读数**，
//     那些方向永远不碾出自由空间，始终是 -1 未知。
//
// 而 /scan_from_cloud 有约 66% 的束是无回波（实测 723 束仅 245 束检出障碍），
// 于是 costmap 自由区 ⊋ /map 自由区，差集正好是那 66% 的方向。
// 结果：规划器在 allow_unknown:false 下合法穿过的栅格，校验器一律判「穿越未知」。
// 实测非法点最近出现在机器人正前方 0.05m 处 —— 任何路径都活不过前三个采样点。
//
// 所以校验层改用规划器所用的那张 global_costmap，矛盾在构造上消失。
// 前沿搜索仍然用 /map：前沿的定义依赖「未知」这个状态，而 costmap 被清障刷过之后
// 未知区已经不完整，拿它找前沿会漏掉大片真正没探索过的区域。
// ==========================================================
//
// ==================== 膨胀梯度被刻意丢弃 ====================
// nav2 代价值的语义（nav2_costmap_2d/cost_values.hpp）：
//     255 NO_INFORMATION            未知
//     254 LETHAL_OBSTACLE           障碍本体
//     253 INSCRIBED_INFLATED_OBSTACLE  机器人中心落此处 → 足迹必然碰撞
//     1~252 膨胀梯度                 越靠近障碍越大，但**可通行**
//     0   FREE_SPACE
//
// 本层把 1~252 全部压成「空闲」，只保留「未知 / 致命 / 空闲」三态。这不是偷懒：
// 第二层校验的职责是拦住【穿越未知区】和【硬碰撞】，而不是替规划器重新评价
// 「贴着墙走划不划算」——膨胀梯度是代价偏好，不是可通行性。
// 若保留梯度，任何贴墙路径都会被判非法（实测 local_costmap 有 27.9% 的栅格
// 带非零代价，目标点本身代价 99 也很常见），等于把探索又锁死一次，
// 只是换了个理由。
// ==========================================================
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
///
/// 入参说明：
///   size_x/size_y  代价地图尺寸（栅格数）
///   resolution     分辨率 (m/cell)，必须 > 0
///   origin_x/y     栅格 (0,0) 的世界坐标 (m)
///   data           row-major 的原始代价字节，长度必须 == size_x*size_y
///   out            输出 GridMap，data 为本包三态口径：
///                    -1  未知
///                     0  已知空闲（含膨胀梯度，见文件头说明）
///                   100  致命障碍
///
/// 输出的 0/100 是「口径无关」的极值：无论 PathValidator 的
/// free_threshold / occupied_threshold 配成多少（合法区间 0<free<occupied<100），
/// 0 必然被判空闲、100 必然被判占据。这样这一层就不需要知道校验阈值，
/// 两处参数也不会漂移。
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
