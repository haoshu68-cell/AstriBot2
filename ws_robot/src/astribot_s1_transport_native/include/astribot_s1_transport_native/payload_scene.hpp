#pragma once
#include <astribot_s1_payload_state/ledger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
namespace astribot::transport {
// The caller registers three positive nominal BOX dimensions and requires an
// identity primitive pose. World geometry is nominal initially and retains the
// observed conservative dimensions after detach; neither changes nominal size.
bool payload_registered_box_size_matches(const shape_msgs::msg::SolidPrimitive &box,
 const std::vector<double> &nominal);
// A previous, still-valid ledger revision may lag the physical/scene readback.
// It authorizes no progress; source changes or a newer unexpected revision fail.
void require_payload_observation_bound(const astribot::payload::Observation &expected,
 const astribot::payload::Observation &latest);
bool payload_revision_ready(const astribot::payload::State &state,
 const astribot::payload::Observation &expected,const std::string &ledger_epoch);
moveit_msgs::msg::PlanningScene payload_scene_diff(const moveit_msgs::msg::PlanningScene &before,
 const astribot::payload::Observation &physical,const std::string &object,bool attach,
 const geometry_msgs::msg::Pose &base_object,const std::string &base,const std::set<std::string> &known_links);
void validate_payload_scene(const moveit_msgs::msg::PlanningScene &before,
 const moveit_msgs::msg::PlanningScene &after,const astribot::payload::Observation &physical,
 const std::string &object,bool attach,const geometry_msgs::msg::Pose &base_object,const std::string &base,const std::set<std::string> &known_links);
}
