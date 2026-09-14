// Copyright 2026 Astribot

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
  random_numbers::RandomNumberGenerator rng(20260820U);

  for (std::size_t s = 0; s < samples; ++s) {
    if (s == 0U) {
      state.setToDefaultValues();
    } else {
      for (const moveit::core::JointModelGroup * jmg : model->getJointModelGroups()) {
        if (jmg != nullptr && jmg->isChain()) {
          state.setToRandomPositions(jmg, rng);
        }
      }
    }
    state.update();

    collision_detection::CollisionRequest request;
    request.contacts = true;
    const std::size_t pair_count = links.size() * (links.size() - 1U) / 2U;
    request.max_contacts = pair_count;
    request.max_contacts_per_pair = 1U;
    collision_detection::CollisionResult result;
    scene->getCollisionEnv()->checkSelfCollision(request, result, state);

    std::vector<LinkPair> collided_now;
    collided_now.reserve(result.contacts.size());
    for (const auto & entry : result.contacts) {
      collided_now.push_back(makeKey(entry.first.first, entry.first.second));
    }
    std::sort(collided_now.begin(), collided_now.end());
    collided_now.erase(
      std::unique(collided_now.begin(), collided_now.end()), collided_now.end());

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
