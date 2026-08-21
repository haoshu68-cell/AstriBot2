// Copyright 2026 Astribot
//
// 自碰撞对生成工具。
//
// 用途
// ----
// SRDF 的 disable_collisions 列表如果只写"父子相邻对"，会漏掉两类：
//   · 结构上必然重叠的对（例如两条臂的基座通过一个无碰撞体的 link 挂在
//     torso_link_4 上，几何上是嵌套的，但它们不是父子关系）
//   · 在整个可达空间里永远碰不到的对（保留它们只是白白增加碰撞检测开销）
// 官方做法是用 MoveIt Setup Assistant 的 GUI 生成，但那不可脚本化、不可复现。
// 本工具用**同一套** moveit collision 检测做随机采样统计，输出可直接粘进
// SRDF 的片段，让碰撞关闭列表变成可复现的实测结果而不是手填。
//
// 判定规则（与 Setup Assistant 一致的语义）
// --------------------------------------
//   ALWAYS  在所有采样构型下都碰 -> reason="Default"，必须关掉，否则无法规划
//   NEVER   在所有采样构型下都不碰 -> reason="Never"，关掉纯粹是省开销
//   其余    真实可能碰撞的对 -> **不能关**，必须留给运行时检测
//
// 用法
// ----
//   ros2 run astribot_s1_manipulation self_collision_pair_generator [samples]
// 需要 robot_description / robot_description_semantic 两个参数可用
// （即 move_group 或 robot_state_publisher 已在运行）。

#include <algorithm>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

namespace
{

struct PairStats
{
  std::size_t collision_count{0};
  std::size_t sample_count{0};
};

using LinkPair = std::pair<std::string, std::string>;

/// 规范化 link 对的顺序，保证 (a,b) 与 (b,a) 统计到同一条目。
LinkPair makeKey(const std::string & a, const std::string & b)
{
  return (a < b) ? LinkPair{a, b} : LinkPair{b, a};
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("self_collision_pair_generator");
  const rclcpp::Logger logger = node->get_logger();

  // 采样数：默认 10000。给太少会把"偶尔才碰"的对误判成 NEVER 而关掉，
  // 那是**危险**的（运行时真碰了也检测不到）。所以宁可多采。
  std::size_t samples = 10000U;
  if (argc > 1) {
    const long parsed = std::strtol(argv[1], nullptr, 10);
    if (parsed > 0) {
      samples = static_cast<std::size_t>(parsed);
    } else {
      RCLCPP_WARN(logger, "invalid sample count '%s', using default %zu", argv[1], samples);
    }
  }

  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  const moveit::core::RobotModelPtr model = loader.getModel();
  if (!model) {
    RCLCPP_ERROR(
      logger,
      "failed to load robot model from 'robot_description'; "
      "is move_group or robot_state_publisher running?");
    rclcpp::shutdown();
    return 1;
  }

  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  if (!scene) {
    RCLCPP_ERROR(logger, "failed to create planning scene");
    rclcpp::shutdown();
    return 1;
  }

  // 只统计有碰撞几何的 link：没有 collision 的 link 永远不可能碰撞，
  // 把它们列进 SRDF 只是噪音（本机器人有 3 个这样的 link：两个 tool_link
  // 和 torso_end_effector）。
  std::vector<std::string> links;
  for (const moveit::core::LinkModel * link : model->getLinkModelsWithCollisionGeometry()) {
    if (link != nullptr) {
      links.push_back(link->getName());
    }
  }
  RCLCPP_INFO(
    logger, "links with collision geometry: %zu, sampling %zu random states",
    links.size(), samples);

  std::map<LinkPair, PairStats> stats;

  moveit::core::RobotState state(model);
  // 固定种子：同样的输入必须给出同样的输出，否则"可复现"就是空话。
  random_numbers::RandomNumberGenerator rng(20260820U);

  for (std::size_t s = 0; s < samples; ++s) {
    // 第一个样本用默认(home)构型：Setup Assistant 的 "Default" 判定就基于它，
    // 结构上必然重叠的对在这个构型下一定会碰。
    if (s == 0U) {
      state.setToDefaultValues();
    } else {
      // 逐组随机，覆盖全身活动自由度。
      // 只对"链"组随机：非链组（如 dual_arm）是若干链的并集，
      // 对它调 setToRandomPositions 会把同一批关节重复随机，没有额外收益。
      for (const moveit::core::JointModelGroup * jmg : model->getJointModelGroups()) {
        if (jmg != nullptr && jmg->isChain()) {
          state.setToRandomPositions(jmg, rng);
        }
      }
    }
    state.update();

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 200U;
    request.max_contacts_per_pair = 1U;
    collision_detection::CollisionResult result;
    // 注意这里**不传** AllowedCollisionMatrix：本工具的目的就是统计
    // "如果什么都不关，哪些对会碰"，传了 ACM 就把已关掉的对过滤掉了。
    scene->getCollisionEnv()->checkSelfCollision(request, result, state);

    std::vector<LinkPair> collided_now;
    collided_now.reserve(result.contacts.size());
    for (const auto & entry : result.contacts) {
      collided_now.push_back(makeKey(entry.first.first, entry.first.second));
    }
    std::sort(collided_now.begin(), collided_now.end());
    collided_now.erase(
      std::unique(collided_now.begin(), collided_now.end()), collided_now.end());

    // 所有对的 sample_count 都要 +1，否则"从没碰过"的对统计不到分母。
    for (std::size_t i = 0; i < links.size(); ++i) {
      for (std::size_t j = i + 1U; j < links.size(); ++j) {
        PairStats & entry = stats[makeKey(links[i], links[j])];
        ++entry.sample_count;
      }
    }
    for (const LinkPair & key : collided_now) {
      auto it = stats.find(key);
      if (it != stats.end()) {
        ++it->second.collision_count;
      }
    }
  }

  // ---- 输出 ----
  std::vector<LinkPair> always_pairs;
  std::vector<LinkPair> never_pairs;
  std::vector<std::pair<LinkPair, double>> sometimes_pairs;

  for (const auto & entry : stats) {
    if (entry.second.sample_count == 0U) {
      continue;
    }
    const double ratio = static_cast<double>(entry.second.collision_count) /
      static_cast<double>(entry.second.sample_count);
    if (entry.second.collision_count == entry.second.sample_count) {
      always_pairs.push_back(entry.first);
    } else if (entry.second.collision_count == 0U) {
      never_pairs.push_back(entry.first);
    } else {
      sometimes_pairs.emplace_back(entry.first, ratio);
    }
  }

  RCLCPP_INFO(logger, "==================== 统计结果 ====================");
  RCLCPP_INFO(
    logger, "总对数 %zu | 总是碰撞 %zu | 从不碰撞 %zu | 有时碰撞 %zu",
    stats.size(), always_pairs.size(), never_pairs.size(), sometimes_pairs.size());
  RCLCPP_INFO(logger, "以下片段可直接粘进 SRDF 的 <robot> 内 (stdout, 无日志前缀)");

  // 用 printf 到 stdout 而不是 RCLCPP：日志带时间戳前缀，没法直接复制粘贴。
  // 这是工具的输出产物，不是日志。
  std::printf("  <!-- ALWAYS colliding (%zu pairs): 结构上必然重叠，必须关掉 -->\n",
    always_pairs.size());
  for (const LinkPair & p : always_pairs) {
    std::printf(
      "  <disable_collisions link1=\"%s\" link2=\"%s\" reason=\"Default\"/>\n",
      p.first.c_str(), p.second.c_str());
  }
  std::printf("\n  <!-- NEVER colliding (%zu pairs): 关掉纯为省开销 -->\n",
    never_pairs.size());
  for (const LinkPair & p : never_pairs) {
    std::printf(
      "  <disable_collisions link1=\"%s\" link2=\"%s\" reason=\"Never\"/>\n",
      p.first.c_str(), p.second.c_str());
  }

  // "有时碰撞"的对**绝对不能**关掉 —— 它们代表真实的碰撞风险，
  // 必须留给运行时检测。这里只列出来供人工核对。
  std::printf(
    "\n  <!-- 下面 %zu 对「有时碰撞」，绝对不要关闭，必须留给运行时检测： -->\n",
    sometimes_pairs.size());
  std::sort(
    sometimes_pairs.begin(), sometimes_pairs.end(),
    [](const auto & a, const auto & b) {return a.second > b.second;});
  for (const auto & entry : sometimes_pairs) {
    std::printf(
      "  <!-- %s <-> %s : 碰撞率 %.2f%% -->\n",
      entry.first.first.c_str(), entry.first.second.c_str(), 100.0 * entry.second);
  }
  std::fflush(stdout);

  rclcpp::shutdown();
  return 0;
}
