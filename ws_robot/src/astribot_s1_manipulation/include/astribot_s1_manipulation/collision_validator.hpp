// Copyright 2026 Astribot
//
// 碰撞校验：自碰撞（双臂互撞、臂撞躯干/轮子/头）+ 环境碰撞。
//
// 为什么单独封一层而不直接调 PlanningScene
// ----------------------------------------
// 1. 任务要求把"自碰撞"和"环境碰撞"分开返回不同错误码：自碰撞是构型本身的问题
//    （必须重规划），环境碰撞可能只是场景里多了个临时物体（上层可能选择先清场景）。
//    PlanningScene 的 checkCollision 把两者混在一起报，需要分别调两个接口。
// 2. 闭链规划里每个 waypoint 都要校验一次，需要"发现第一个碰撞就返回"的快路径
//    和"收集全部接触对"的诊断路径两种模式。
// 3. 接触对名字必须能拿出来打日志 —— 否则现场只知道"碰了"，不知道碰哪儿，
//    没法判断是 SRDF 该关的对没关，还是真的构型不对。

#ifndef ASTRIBOT_S1_MANIPULATION__COLLISION_VALIDATOR_HPP_
#define ASTRIBOT_S1_MANIPULATION__COLLISION_VALIDATOR_HPP_

#include <string>
#include <utility>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include "astribot_s1_manipulation/error_codes.hpp"

namespace astribot_s1_manipulation
{

struct CollisionParams
{
  /// 自碰撞检测开关。任务明确禁止跳过碰撞校验，这里默认 true；
  /// 留出开关只为在纯运动学调试时临时关闭，正式运行不应关。
  bool check_self_collision{true};

  /// 环境碰撞检测开关（与 PlanningScene 里的 world 物体检测）。
  bool check_environment_collision{true};

  /// 是否收集全部接触对。false 时发现第一对就返回（快，用于逐点校验）；
  /// true 时收集所有接触对（慢，用于失败后的诊断日志）。
  bool collect_all_contacts{false};

  /// collect_all_contacts=true 时最多收集多少对，防止病态构型下
  /// 接触对爆炸把日志刷爆。
  std::size_t max_contacts{16};
};

struct CollisionReport
{
  bool collision{false};
  bool self_collision{false};
  bool environment_collision{false};
  /// 接触的 link 对（自碰撞时两边都是机器人 link；环境碰撞时一边是物体 id）。
  std::vector<std::pair<std::string, std::string>> contacts;
  std::string reason;

  /// 映射成错误码：自碰撞优先（更严重，且必然要重规划）。
  PlanErrorCode toErrorCode() const noexcept
  {
    if (self_collision) {
      return PlanErrorCode::kSelfCollision;
    }
    if (environment_collision) {
      return PlanErrorCode::kEnvironmentCollision;
    }
    return PlanErrorCode::kSuccess;
  }
};

/// 碰撞校验器。持有 PlanningScene 的**常量**引用计数指针：
/// 校验期间不修改场景，多线程只读安全。
class CollisionValidator
{
public:
  CollisionValidator() = default;

  /// @param scene 必须非空且其 RobotModel 有效。
  bool configure(
    const planning_scene::PlanningSceneConstPtr & scene,
    const CollisionParams & params,
    std::string & error);

  bool isConfigured() const noexcept
  {
    return configured_;
  }

  /// 校验单个构型。
  /// @param group_name 只关心该组相关的碰撞时传组名；传空串则检查整机。
  ///        注意：即使限定了组，MoveIt 仍会把该组的 link 与**所有**其他 link
  ///        比对（包括躯干和轮子），所以"臂-底盘碰撞"是被覆盖到的。
  CollisionReport check(
    const moveit::core::RobotState & state,
    const std::string & group_name = std::string()) const;

  /// 校验一串构型，返回第一个碰撞点的报告。
  /// @param first_bad_index 输出第一个碰撞点下标；全部无碰撞时不写。
  CollisionReport checkStates(
    const std::vector<moveit::core::RobotState> & states,
    const std::string & group_name,
    std::size_t * first_bad_index = nullptr) const;

private:
  planning_scene::PlanningSceneConstPtr scene_;
  CollisionParams params_;
  bool configured_{false};
};

}  // namespace astribot_s1_manipulation

#endif  // ASTRIBOT_S1_MANIPULATION__COLLISION_VALIDATOR_HPP_
