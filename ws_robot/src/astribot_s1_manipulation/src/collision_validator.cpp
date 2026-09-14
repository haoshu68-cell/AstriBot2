// Copyright 2026 Astribot

#include "astribot_s1_manipulation/collision_validator.hpp"

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_common.h>
#include <rclcpp/logging.hpp>

namespace astribot_s1_manipulation
{

namespace
{
constexpr const char * kLoggerName = "astribot_s1_manipulation.collision";

/// 把 MoveIt 的接触结果摊平成 (link1, link2) 列表。
void appendContacts(
  const collision_detection::CollisionResult & result,
  std::size_t max_contacts,
  std::vector<std::pair<std::string, std::string>> & out)
{
  for (const auto & entry : result.contacts) {
    if (out.size() >= max_contacts) {
      return;
    }
    out.emplace_back(entry.first.first, entry.first.second);
  }
}

std::string describeContacts(
  const std::vector<std::pair<std::string, std::string>> & contacts)
{
  std::ostringstream oss;
  bool first = true;
  for (const auto & c : contacts) {
    if (!first) {
      oss << ", ";
    }
    oss << c.first << "<->" << c.second;
    first = false;
  }
  return oss.str();
}
}  // namespace

bool CollisionValidator::configure(
  const planning_scene::PlanningSceneConstPtr & scene,
  const CollisionParams & params,
  std::string & error)
{
  error.clear();

  if (!scene) {
    error = "planning scene pointer is null";
    return false;
  }
  if (!scene->getRobotModel()) {
    error = "planning scene has no robot model";
    return false;
  }
  if (params.max_contacts == 0U) {
    error = "max_contacts must be >= 1";
    return false;
  }
  if (!params.check_self_collision && !params.check_environment_collision) {
    error = "both self and environment collision checks are disabled; "
      "this would skip collision validation entirely";
    return false;
  }

  scene_ = scene;
  params_ = params;
  configured_ = true;
  return true;
}

CollisionReport CollisionValidator::check(
  const moveit::core::RobotState & state,
  const std::string & group_name) const
{
  CollisionReport report;

  if (!configured_) {
    report.reason = "CollisionValidator not configured";
    report.collision = true;
    return report;
  }
  if (!scene_) {
    report.reason = "planning scene became null";
    report.collision = true;
    return report;
  }

  moveit::core::RobotState local_state(state);
  local_state.update();

  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = params_.collect_all_contacts ? params_.max_contacts : 1U;
  request.max_contacts_per_pair = 1U;
  request.distance = false;
  if (!group_name.empty()) {
    request.group_name = group_name;
  }

  if (params_.check_self_collision) {
    collision_detection::CollisionResult result;
    scene_->checkSelfCollision(request, result, local_state, scene_->getAllowedCollisionMatrix());
    if (result.collision) {
      report.collision = true;
      report.self_collision = true;
      appendContacts(result, params_.max_contacts, report.contacts);
      report.reason = "self collision: " + describeContacts(report.contacts);
      if (!params_.collect_all_contacts) {
        return report;
      }
    }
  }

  if (params_.check_environment_collision) {
    collision_detection::CollisionResult result;
    scene_->checkCollision(request, result, local_state, scene_->getAllowedCollisionMatrix());
    if (result.collision) {
      std::vector<std::pair<std::string, std::string>> all;
      appendContacts(result, params_.max_contacts, all);

      const moveit::core::RobotModelConstPtr & model = scene_->getRobotModel();
      bool has_world_contact = false;
      for (const auto & c : all) {
        const bool a_is_link = model->hasLinkModel(c.first);
        const bool b_is_link = model->hasLinkModel(c.second);
        if (!a_is_link || !b_is_link) {
          has_world_contact = true;
          if (report.contacts.size() < params_.max_contacts) {
            report.contacts.push_back(c);
          }
        }
      }
      if (has_world_contact) {
        report.collision = true;
        report.environment_collision = true;
        const std::string desc = describeContacts(report.contacts);
        report.reason = report.reason.empty() ?
          ("environment collision: " + desc) :
          (report.reason + " | environment collision: " + desc);
        if (!params_.collect_all_contacts) {
          return report;
        }
      } else if (!report.self_collision) {
        report.collision = true;
        report.self_collision = true;
        report.contacts = all;
        report.reason = "self collision (detected via full check): " + describeContacts(all);
        if (!params_.collect_all_contacts) {
          return report;
        }
      }
    }
  }

  if (!report.collision) {
    report.reason = "collision free";
  }
  return report;
}

CollisionReport CollisionValidator::checkStates(
  const std::vector<moveit::core::RobotState> & states,
  const std::string & group_name,
  std::size_t * first_bad_index) const
{
  CollisionReport report;
  if (states.empty()) {
    report.reason = "no states to check";
    return report;
  }

  for (std::size_t i = 0; i < states.size(); ++i) {
    report = check(states[i], group_name);
    if (report.collision) {
      if (first_bad_index != nullptr) {
        *first_bad_index = i;
      }
      RCLCPP_DEBUG(
        rclcpp::get_logger(kLoggerName),
        "collision at waypoint %zu/%zu: %s", i, states.size(), report.reason.c_str());
      return report;
    }
  }

  report.reason = "all " + std::to_string(states.size()) + " waypoints collision free";
  return report;
}

}  // namespace astribot_s1_manipulation
