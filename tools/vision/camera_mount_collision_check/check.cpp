// Offline model verification only: no ROS node, subscriptions or controller calls.
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <nlohmann/json.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

using nlohmann::json;
std::string read(const char* path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error(std::string("Cannot read ") + path);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
bool camera(const std::string& link) { return link.find("_camera_link") != std::string::npos; }

int main(int argc, char** argv) {
  if (argc != 4) { std::cerr << "Usage: check robot.urdf robot.srdf result.json\n"; return 2; }
  auto urdf = urdf::parseURDF(read(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read(argv[2]))) return 2;
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  planning_scene::PlanningScene scene(model);
  const std::vector<std::string> cameras = {"head_rgbd", "head_stereo_left", "head_stereo_right",
    "torso_rgbd", "left_wrist_rgbd", "right_wrist_rgbd"};
  for (const auto& id : cameras) {
    auto* link = model->getLinkModel(id + "_camera_link");
    if (!link || link->getShapes().empty()) throw std::runtime_error("Camera collision shape missing: " + id);
  }
  json report = {{"scope", "offline MoveIt FCL geometry; no trajectory execution"},
                 {"states", json::array()}, {"environment_checks", json::array()},
                 {"parent_coverage_checks", json::array()}};
  bool safe = true;
  for (const auto& pose : {"zero", "home", "ready_left", "ready_right", "ready_both",
                           "transport_compact_right", "transport_compact_both",
                           "torso_yaw_min", "torso_yaw_mid", "torso_yaw_max"}) {
    for (double opening : {0.0, 0.465, 0.93}) {
      moveit::core::RobotState state(model);
      state.setToDefaultValues();
      if (std::string(pose) != "zero") {
        state.setToDefaultValues(model->getJointModelGroup("torso"), "home");
        for (const auto& side : {"left", "right"}) {
          const bool ready = std::string(pose) == "ready_both" || std::string(pose) == std::string("ready_") + side;
          state.setToDefaultValues(model->getJointModelGroup(std::string("arm_") + side), ready ? "ready" : "home");
        }
      }
      // READY_RIGHT in TransportTask requests transport_compact, not SRDF ready.
      if (std::string(pose).find("transport_compact_") == 0) {
        state.setJointGroupPositions(model->getJointModelGroup("arm_right"),
                                     std::vector<double>{0., 0., .6, 1., 0., 0., 0.});
        if (std::string(pose) == "transport_compact_both")
          state.setJointGroupPositions(model->getJointModelGroup("arm_left"),
                                       std::vector<double>{0., 0., -.6, 1., 0., 0., 0.});
      }
      if (std::string(pose).find("torso_yaw_") == 0)
        state.setVariablePosition("astribot_torso_joint_4", std::string(pose) == "torso_yaw_min" ? -1.2 :
                                  (std::string(pose) == "torso_yaw_max" ? 1.2 : 0.0));
      for (const auto& side : {"left", "right"})
        state.setVariablePosition(std::string("astribot_gripper_") + side + "_joint_L1", opening);
      state.update();
      collision_detection::CollisionRequest request;
      request.contacts = true;
      request.max_contacts = 10000;
      request.max_contacts_per_pair = 1;
      collision_detection::CollisionResult result;
      scene.checkSelfCollision(request, result, state);
      json row = {{"pose", pose}, {"gripper_rad", opening}, {"camera_contacts", json::array()},
                  {"other_contacts", json::array()}, {"within_bounds", state.satisfiesBounds()}};
      for (const auto& contact : result.contacts) {
        const auto& pair = contact.first;
        const bool involved = camera(pair.first) || camera(pair.second);
        row[involved ? "camera_contacts" : "other_contacts"].push_back({pair.first, pair.second});
        if (involved) safe = false;
      }
      report["states"].push_back(row);
    }
  }
  // Put an external box inside each sensor housing in turn. Mount adjacency
  // exclusions must never make the sensor invisible to environment collision.
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.update();
  for (const auto& id : cameras) {
    const auto name = id + "_camera_link";
    Eigen::Isometry3d position = state.getGlobalLinkTransform(name) *
      model->getLinkModel(name)->getCollisionOriginTransforms().front();
    scene.getWorldNonConst()->addToObject("probe_obstacle", std::make_shared<shapes::Box>(0.008, 0.008, 0.008), position);
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 10000;
    request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult result;
    scene.checkCollision(request, result, state);
    bool detected = false;
    for (const auto& entry : result.contacts) {
      const auto& p = entry.first;
      if ((p.first == name && p.second == "probe_obstacle") ||
          (p.second == name && p.first == "probe_obstacle")) detected = true;
    }
    report["environment_checks"].push_back({{"camera", id}, {"external_collision_detected", detected}});
    safe = safe && detected;
    scene.getWorldNonConst()->removeObject("probe_obstacle");
  }
  // The removed duplicate housing and bracket remain collision-protected by
  // the fixed torso parent. Full continuous box coverage is checked by pytest.
  for (const auto& part : {"housing", "bracket"}) {
    Eigen::Isometry3d position = state.getGlobalLinkTransform("torso_rgbd_camera_link");
    position.translate(Eigen::Vector3d(std::string(part) == "housing" ? -0.022 : -0.033668, 0, 0));
    scene.getWorldNonConst()->addToObject("covered_probe", std::make_shared<shapes::Box>(0.002, 0.002, 0.002), position);
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 10000;
    request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult result;
    scene.checkCollision(request, result, state);
    bool detected = false;
    for (const auto& entry : result.contacts) {
      const auto& p = entry.first;
      if ((p.first == "astribot_torso_link_4" && p.second == "covered_probe") ||
          (p.second == "astribot_torso_link_4" && p.first == "covered_probe")) detected = true;
    }
    report["parent_coverage_checks"].push_back({{"part", part}, {"parent_collision_detected", detected}});
    safe = safe && detected;
    scene.getWorldNonConst()->removeObject("covered_probe");
  }
  report["camera_mount_checks_passed"] = safe;
  std::ofstream(argv[3]) << report.dump(2) << '\n';
  std::cout << report.dump(2) << '\n';
  return safe ? 0 : 1;
}
