// Copyright 2026 Astribot.
//
// 决策模块节点：边界遍历 + 前沿点自适应采样 的自主探索。
//
// 明确的职责边界（对应「禁止探索模块直接发布 Twist」）：
//   本节点**只输出 geometry_msgs/PoseStamped 目标位姿**，不含任何 Nav2 客户端、
//   不发 Twist、不做路径跟踪。谁去执行、怎么执行，由外部订阅者决定。
//
// 线程模型：地图回调只做校验和快照拷贝，规划循环跑在独立工作线程上，
// 避免大地图的 BFS/膨胀阻塞订阅回调。
#ifndef ASTRIBOT_S1_AUTONOMY__FRONTIER_EXPLORER_NODE_HPP_
#define ASTRIBOT_S1_AUTONOMY__FRONTIER_EXPLORER_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "astribot_s1_autonomy/frontier_search.hpp"

namespace astribot_s1_autonomy
{

class FrontierExplorerNode : public rclcpp::Node
{
public:
  explicit FrontierExplorerNode(const rclcpp::NodeOptions & options);
  ~FrontierExplorerNode() override;

  FrontierExplorerNode(const FrontierExplorerNode &) = delete;
  FrontierExplorerNode & operator=(const FrontierExplorerNode &) = delete;
  FrontierExplorerNode(FrontierExplorerNode &&) = delete;
  FrontierExplorerNode & operator=(FrontierExplorerNode &&) = delete;

private:
  void declareParameters();
  bool loadParameters(std::string & error);

  // ---------------- 数据流 ----------------
  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  /// 规划线程主循环：按 planning_period_sec_ 周期跑一次 planOnce()。
  void plannerLoop();
  /// 跑一轮探索规划。
  void planOnce();

  /// 从 TF 取机器人在地图坐标系下的位置。失败返回 false。
  bool lookupRobotPose(double & x, double & y);

  /// 带「约束放宽」的搜索：连续失败时逐级放宽约束，避免一直采不到点而死循环。
  /// 返回实际使用的放宽等级（0 = 未放宽）。
  int searchWithRelaxation(
    const GridMap & map, double robot_x, double robot_y, FrontierSearch::Result & out);

  /// 判定新目标是否与上一次目标「实质相同」（用于震荡检测）。
  bool isSameAsLastGoal(double x, double y) const;

  // ---------------- 输出 ----------------
  void publishGoal(const GoalCandidate & goal, const rclcpp::Time & stamp);
  void publishStatus(const std::string & state, const std::string & detail);
  void publishMarkers(
    const GridMap & map, const FrontierSearch::Result & result, const rclcpp::Time & stamp);

  // ---------------- 成员 ----------------
  // 参数
  std::string map_topic_;
  std::string goal_topic_;
  std::string status_topic_;
  std::string complete_topic_;
  std::string marker_topic_;
  std::string map_frame_;
  std::string robot_base_frame_;
  double tf_timeout_sec_{0.0};
  double planning_period_sec_{0.0};
  double map_timeout_sec_{0.0};
  double goal_same_tolerance_{0.0};
  /// 机器人「算作有推进」的最小位移(m)。目标不变但机器人在动 = 正常；
  /// 目标不变且机器人不动 = 卡住，才触发回退。
  double robot_progress_tolerance_{0.0};
  int max_same_goal_count_{0};
  int max_relaxation_level_{0};
  double relaxation_scale_{0.0};
  int max_consecutive_failures_{0};
  std::size_t visit_history_limit_{0U};
  bool publish_markers_{true};

  FrontierSearchParams search_params_;
  FrontierSearch search_;
  std::mutex config_mutex_;

  // ROS 句柄
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 地图快照
  std::shared_ptr<GridMap> latest_map_;
  rclcpp::Time latest_map_stamp_;
  std::mutex map_mutex_;

  // 规划线程
  std::atomic<bool> running_{false};
  std::thread planner_thread_;
  std::condition_variable planner_cv_;
  std::mutex planner_mutex_;

  // 状态
  std::vector<VisitRecord> visit_history_;
  bool has_last_goal_{false};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  /// 上一轮规划时机器人的位置，用于判断这一轮是否有推进。
  double last_plan_robot_x_{0.0};
  double last_plan_robot_y_{0.0};
  int same_goal_count_{0};
  int consecutive_failure_count_{0};
  bool exploration_complete_{false};
  bool map_ready_warned_{false};
};

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__FRONTIER_EXPLORER_NODE_HPP_
