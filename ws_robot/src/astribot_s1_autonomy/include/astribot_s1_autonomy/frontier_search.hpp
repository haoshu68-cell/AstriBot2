// Copyright 2026 Astribot.
//
// 「边界遍历 + 前沿点自适应采样」的纯算法核心。
//
// 同样刻意不依赖 ROS 类型：输入是一份自带尺寸/分辨率/原点的栅格快照，
// 输出是候选目标点列表。节点层负责把 nav_msgs::msg::OccupancyGrid 拷成
// GridMap、把 TF 里的机器人位姿传进来、把结果转成 PoseStamped 和 Marker。
//
// ============================ 算法总览 ============================
//   栅格预处理  : 去小斑块 → 膨胀障碍物 → 计算可达域(从机器人做自由空间 BFS)
//   前沿提取    : 自由格 && 邻域含未知格 && 不在膨胀障碍内  ⇒ 前沿格
//   边界遍历    : 对前沿格做 8 邻域连通域聚类，得到多块前沿区域
//   前沿块过滤  : 面积过小 / 距离过近 / 不可达(被障碍物包围) ⇒ 丢弃
//   自适应采样  : 采样数 ∝ 前沿块面积，按空间跨度均匀取点
//   候选校验    : 落在障碍/膨胀区内、离机器人过近、净空不足、不可达 ⇒ 丢弃
//   代价评估    : cost = w_dist*距离 + w_visit*历史访问惩罚 - w_gain*未知增益
//   选最优      : cost 最小者；朝向取「由目标点指向前沿块质心」，让雷达朝未知区看
// =================================================================
#ifndef ASTRIBOT_S1_AUTONOMY__FRONTIER_SEARCH_HPP_
#define ASTRIBOT_S1_AUTONOMY__FRONTIER_SEARCH_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace astribot_s1_autonomy
{

/// 与 ROS 解耦的占据栅格快照。data 沿用 ROS 的约定：
/// -1 未知，0 空闲，100 占据（中间值按阈值判定）。
struct GridMap
{
  unsigned int width{0U};
  unsigned int height{0U};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;

  bool empty() const {return width == 0U || height == 0U || data.empty();}
  /// data 长度是否和声明的尺寸一致。节点层必须校验，防止越界读。
  bool consistent() const
  {
    return !empty() &&
           data.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }
  std::size_t index(unsigned int mx, unsigned int my) const
  {
    return (static_cast<std::size_t>(my) * static_cast<std::size_t>(width)) +
           static_cast<std::size_t>(mx);
  }
  /// 栅格中心的世界坐标。
  double worldX(unsigned int mx) const
  {
    return origin_x + ((static_cast<double>(mx) + 0.5) * resolution);
  }
  double worldY(unsigned int my) const
  {
    return origin_y + ((static_cast<double>(my) + 0.5) * resolution);
  }
  /// 世界坐标 → 栅格下标。返回 false 表示落在地图外。
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
};

/// 一块前沿连通域。
struct FrontierCluster
{
  /// 组成该前沿块的栅格线性下标（BFS 顺序，空间上连续）。
  std::vector<std::size_t> cells;
  /// 质心（世界坐标，m）。
  double centroid_x{0.0};
  double centroid_y{0.0};
  /// 前沿格数量，等价于前沿「长度/面积」的度量。
  std::size_t size{0U};
  /// 未知增益：该前沿块邻域窗口内的未知格数量，代表「走过去能新探明多少」。
  std::size_t unknown_gain{0U};
  /// 质心到机器人的直线距离(m)。
  double distance_to_robot{0.0};
  /// 是否通过了过滤（false 的块依然会发 Marker，用不同颜色标出，便于调参）。
  bool accepted{false};
  /// 被过滤掉的原因，便于日志和可视化标注。
  std::string reject_reason;
};

/// 一个候选目标点。
struct GoalCandidate
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::size_t cluster_index{0U};
  /// 代价分解，全部留着是为了能在日志/Marker 里解释「为什么选它」。
  double cost{0.0};
  double distance{0.0};
  double gain_normalized{0.0};
  double visit_penalty{0.0};
  bool valid{false};
  std::string reject_reason;
};

/// 历史访问记录，用于抑制反复往同一个地方跑。
struct VisitRecord
{
  double x{0.0};
  double y{0.0};
  /// 该位置被选为目标的次数，次数越多惩罚越大。
  unsigned int count{1U};
};

/// 全部来自 YAML，禁止硬编码。
struct FrontierSearchParams
{
  // ---- 栅格判定阈值 ----
  /// data 值 >= 该阈值判为占据。
  int occupied_threshold{65};
  /// data 值 <= 该阈值且非负判为空闲。
  int free_threshold{25};

  // ---- 预处理 ----
  /// 障碍物膨胀半径(m)。至少应覆盖机器人半径，否则会采到贴墙走不进去的目标。
  double obstacle_inflation_radius{0.0};
  /// 小于该格数的孤立占据斑块视为噪声，预处理阶段抹掉。
  int min_obstacle_cluster_cells{0};

  // ---- 前沿提取/聚类 ----
  /// 判定「邻域含未知格」时是否用 8 邻域（false 则用 4 邻域）。
  bool use_eight_connectivity{true};
  /// 小于该格数的前沿块直接丢弃。
  int min_frontier_cells{0};
  /// 计算未知增益时的邻域窗口半径(m)。
  double gain_window_radius{0.0};

  // ---- 采样 ----
  /// 采样数 = clamp(ceil(前沿格数 * adaptive_sample_gain), min, max)。
  double adaptive_sample_gain{0.0};
  int min_samples_per_cluster{1};
  int max_samples_per_cluster{1};
  /// 候选点必须保证的净空半径(m)：以候选点为心、该半径内不能有膨胀后的障碍。
  double required_clearance_radius{0.0};
  /// 目标点离机器人过近则丢弃(m)。
  double min_goal_distance{0.0};
  /// 目标点离机器人过远则丢弃(m)；<=0 表示不限制。
  double max_goal_distance{0.0};

  // ---- 代价权重 ----
  double weight_distance{1.0};
  double weight_gain{1.0};
  double weight_visit_penalty{1.0};
  /// 历史访问惩罚的作用半径(m)：候选点落在该半径内才会吃到惩罚。
  double visit_penalty_radius{0.0};

  /// 采样随机数种子。固定种子让复现调试变得可能。
  unsigned int random_seed{0U};
};

/// 前沿搜索器。无状态（历史记录由节点层持有并传入），可反复调用。
class FrontierSearch
{
public:
  struct Result
  {
    std::vector<FrontierCluster> clusters;
    std::vector<GoalCandidate> candidates;
    /// 最优候选在 candidates 里的下标；-1 表示本次没有任何有效候选。
    int best_candidate_index{-1};
    /// 原始前沿格总数（聚类/过滤之前），用于判断「是否真的探索完了」。
    std::size_t raw_frontier_cell_count{0U};
    /// 通过过滤的前沿块数量。
    std::size_t accepted_cluster_count{0U};
    /// 人类可读的结论，直接可打日志。
    std::string summary;
  };

  FrontierSearch() = default;

  bool configure(const FrontierSearchParams & params, std::string & error);
  const FrontierSearchParams & params() const {return params_;}

  /// 主入口。map 必须 consistent()，否则直接返回空结果（不抛异常、不崩溃）。
  void search(
    const GridMap & map,
    double robot_x,
    double robot_y,
    const std::vector<VisitRecord> & history,
    Result & out);

private:
  /// 预处理：去小斑块 + 膨胀，产出 inflated_occupied_ 掩码。
  void buildObstacleMask(const GridMap & map);
  /// 从机器人所在格出发做自由空间 BFS，产出 reachable_ 掩码。
  /// 机器人位置本身若落在障碍/未知格上，会就近寻找一个自由格作为种子。
  bool buildReachableMask(const GridMap & map, double robot_x, double robot_y);
  /// 提取前沿格，写入 is_frontier_。
  std::size_t extractFrontierCells(const GridMap & map);
  /// 对前沿格做连通域聚类。
  void clusterFrontiers(const GridMap & map, double robot_x, double robot_y, Result & out);
  /// 统计某点邻域窗口内的未知格数量。
  std::size_t countUnknownAround(const GridMap & map, unsigned int mx, unsigned int my) const;
  /// 候选点净空校验。
  bool hasClearance(const GridMap & map, unsigned int mx, unsigned int my) const;
  /// 计算历史访问惩罚。
  double visitPenaltyAt(double x, double y, const std::vector<VisitRecord> & history) const;

  FrontierSearchParams params_;
  bool configured_{false};

  // 内部工作缓冲，按地图尺寸复用，避免每次 search 都重新分配大块内存。
  std::vector<uint8_t> inflated_occupied_;
  std::vector<uint8_t> reachable_;
  std::vector<uint8_t> is_frontier_;
  std::vector<uint8_t> visited_;
  std::vector<std::size_t> bfs_queue_;
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__FRONTIER_SEARCH_HPP_
