#include "astribot_s1_gazebo_bringup/height_slice_geometry.hpp"
#include "astribot_s1_gazebo_bringup/height_slice_shapes.hpp"
#include "astribot_s1_gazebo_bringup/kinematic_inventory.hpp"
#include <astribot_slam_msgs/msg/height_slice_maps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/SystemPluginInfo.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>
#include <sdf/Mesh.hh>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>

namespace astribot {
namespace sim = ignition::gazebo;
namespace c = sim::components;
namespace math = ignition::math;
using Bundle = astribot_slam_msgs::msg::HeightSliceMaps;
using json = nlohmann::json;

class HeightSliceMap final : public sim::System, public sim::ISystemConfigure,
                             public sim::ISystemPostUpdate {
  struct Collision {
    sdf::Geometry geometry;
    math::Pose3d pose;
  };
  sim::Entity world_{sim::kNullEntity};
  std::string world_name_, robot_name_, frame_, reference_, profile_revision_;
  math::Pose3d map_from_world_;
  double ground_ = 0., resolution_ = 0.;
  std::array<double, 4> bounds_{};
  std::vector<double> edges_;
  std::vector<std::string> names_;
  simulation::SliceRaster empty_grid_;
  Bundle bundle_;
  std::string signature_, last_error_;
  int64_t last_check_ = -1;
  std::chrono::steady_clock::time_point last_check_wall_{};
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Bundle>::SharedPtr snapshot_pub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> layer_pubs_;

  static std::string hash(const std::string &bytes) {
    unsigned char digest[EVP_MAX_MD_SIZE]; unsigned int size = 0;
    if (EVP_Digest(bytes.data(), bytes.size(), digest, &size, EVP_sha256(), nullptr) != 1)
      throw std::runtime_error("height map SHA256 failed");
    std::ostringstream out; out << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < size; ++i) out << std::setw(2) << unsigned(digest[i]);
    return out.str();
  }

  static builtin_interfaces::msg::Time stamp(int64_t ns) {
    if (ns < 0 || ns / 1000000000 > INT32_MAX)
      throw std::invalid_argument("height map simulation time outside ROS stamp range");
    builtin_interfaces::msg::Time value;
    value.sec = static_cast<int32_t>(ns / 1000000000);
    value.nanosec = static_cast<uint32_t>(ns % 1000000000);
    return value;
  }

  void publish(int64_t capture) {
    bundle_.header.stamp = stamp(capture);
    for (auto &grid : bundle_.grids) grid.header = bundle_.header;
    // The single bundle is the versioned consumer interface; individual grids
    // are views for existing displays and carry the same capture stamp.
    snapshot_pub_->publish(bundle_);
    for (size_t i = 0; i < layer_pubs_.size(); ++i) layer_pubs_[i]->publish(bundle_.grids[i]);
  }

  void rebuild(const std::map<sim::Entity, Collision> &collisions,
               const std::string &revision, int64_t capture) {
    std::vector<simulation::SliceRaster> rasters(names_.size(), empty_grid_);
    for (const auto &[entity, value] : collisions) {
      (void)entity;
      const auto triangles = simulation::collision_triangles(value.geometry, value.pose);
      for (size_t i = 0; i < rasters.size(); ++i)
        simulation::rasterize_mesh(triangles, ground_ + edges_[i], ground_ + edges_[i + 1], rasters[i]);
    }
    Bundle next;
    next.header.frame_id = frame_;
    next.header.stamp = stamp(capture);
    next.map_revision = revision;
    next.profile_revision = profile_revision_;
    next.ground_z = ground_;
    next.ground_reference = reference_;
    next.height_edges = edges_;
    next.layer_names = names_;
    // No sampled point cloud is used by this producer. Point-count fields are
    // zero, not triangle counts disguised as observations.
    next.point_counts.assign(names_.size(), 0);
    next.evidence_kind = "gazebo_collision_geometry";
    for (auto &raster : rasters) {
      nav_msgs::msg::OccupancyGrid grid;
      grid.header = next.header;
      grid.info.map_load_time = next.header.stamp;
      grid.info.resolution = resolution_;
      grid.info.width = raster.width;
      grid.info.height = raster.height;
      grid.info.origin.position.x = raster.origin_x;
      grid.info.origin.position.y = raster.origin_y;
      grid.info.origin.orientation.w = 1.;
      grid.data = std::move(raster.data);
      next.grids.push_back(std::move(grid));
    }
    bundle_ = std::move(next);
  }

 public:
  void Configure(const sim::Entity &entity, const std::shared_ptr<const sdf::Element> &config,
                 sim::EntityComponentManager &ecm, sim::EventManager &) override {
    world_ = entity;
    if (!ecm.Component<c::World>(world_))
      throw std::invalid_argument("HeightSliceMap must be attached to the world");
    const auto required = [&](const char *key) {
      if (!config->HasElement(key)) throw std::invalid_argument(std::string("missing height map tag: ") + key);
      const auto value = config->Get<std::string>(key);
      if (value.empty()) throw std::invalid_argument(std::string("empty height map tag: ") + key);
      return value;
    };
    world_name_ = required("world_name"); robot_name_ = required("robot_model");
    frame_ = required("frame_id"); reference_ = required("ground_reference");
    const auto *world_name = ecm.Component<c::Name>(world_);
    if (!world_name || world_name->Data() != world_name_)
      throw std::invalid_argument("height map world name mismatch");
    std::string extra;
    std::array<double, 6> transform{};
    std::istringstream transform_text(required("map_from_world"));
    if (!(transform_text >> transform[0] >> transform[1] >> transform[2] >> transform[3] >> transform[4] >> transform[5]) || (transform_text >> extra))
      throw std::invalid_argument("map_from_world requires x y z roll pitch yaw");
    map_from_world_ = math::Pose3d(transform[0], transform[1], transform[2], transform[3], transform[4], transform[5]);
    std::istringstream ground_text(required("ground_z"));
    if (!(ground_text >> ground_) || (ground_text >> extra))
      throw std::invalid_argument("ground_z requires one finite number");
    if (!map_from_world_.IsFinite() || !std::isfinite(ground_))
      throw std::invalid_argument("nonfinite height map frame or ground");
    std::istringstream limits(required("known_xy_bounds"));
    if (!(limits >> bounds_[0] >> bounds_[1] >> bounds_[2] >> bounds_[3]) || (limits >> extra))
      throw std::invalid_argument("known_xy_bounds requires xmin ymin xmax ymax");
    for (double bound : bounds_) if (!std::isfinite(bound))
      throw std::invalid_argument("nonfinite known XY bounds");
    if (bounds_[0] >= bounds_[2] || bounds_[1] >= bounds_[3])
      throw std::invalid_argument("known XY bounds must increase");
    const auto profile_file = required("profile_file");
    std::ifstream input(profile_file, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open height profile: " + profile_file);
    std::ostringstream bytes; bytes << input.rdbuf();
    if (input.bad()) throw std::runtime_error("cannot read height profile: " + profile_file);
    profile_revision_ = hash(bytes.str());
    const auto profile = YAML::Load(bytes.str())["/**"]["ros__parameters"];
    edges_ = profile["height_edges"].as<std::vector<double>>();
    names_ = profile["layer_names"].as<std::vector<std::string>>();
    resolution_ = profile["resolution"].as<double>();
    const auto budget = profile["max_grid_cells"].as<int64_t>();
    if (!std::isfinite(resolution_) || resolution_ <= 0 || budget <= 0 || edges_.size() < 2 || names_.size() + 1 != edges_.size())
      throw std::invalid_argument("invalid height profile dimensions");
    std::set<std::string> unique_names;
    for (size_t i = 0; i < edges_.size(); ++i)
      if (!std::isfinite(edges_[i]) || (i && edges_[i] <= edges_[i - 1]))
        throw std::invalid_argument("height profile edges must increase");
    for (const auto &name : names_) {
      if (name.empty() || name.front() < 'a' || name.front() > 'z' ||
          name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_") != std::string::npos || !unique_names.insert(name).second)
        throw std::invalid_argument("invalid height layer name");
    }
    empty_grid_.resolution = resolution_;
    empty_grid_.origin_x = std::floor(bounds_[0] / resolution_) * resolution_ - resolution_;
    empty_grid_.origin_y = std::floor(bounds_[1] / resolution_) * resolution_ - resolution_;
    const double width = std::ceil((bounds_[2] - empty_grid_.origin_x) / resolution_) + 1;
    const double height = std::ceil((bounds_[3] - empty_grid_.origin_y) / resolution_) + 1;
    if (!std::isfinite(width) || !std::isfinite(height) || width > UINT32_MAX || height > UINT32_MAX || width * height > double(budget))
      throw std::invalid_argument("height map exceeds grid budget");
    empty_grid_.width = static_cast<size_t>(width);
    empty_grid_.height = static_cast<size_t>(height);
    empty_grid_.data.assign(empty_grid_.width * empty_grid_.height, -1);
    // A whole cell must lie inside the declared known domain. Boundary cells
    // and an explicit outer border remain unknown; outside the grid is unknown.
    for (size_t y = 0; y < empty_grid_.height; ++y)
      for (size_t x = 0; x < empty_grid_.width; ++x) {
        const double left = empty_grid_.origin_x + x * resolution_;
        const double bottom = empty_grid_.origin_y + y * resolution_;
        if (left >= bounds_[0] && bottom >= bounds_[1] && left + resolution_ <= bounds_[2] && bottom + resolution_ <= bounds_[3])
          empty_grid_.data[y * empty_grid_.width + x] = 0;
      }
    context_ = std::make_shared<rclcpp::Context>(); context_->init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("simulation_height_slice_map", rclcpp::NodeOptions().context(context_).use_global_arguments(false));
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    snapshot_pub_ = node_->create_publisher<Bundle>("/height_maps/snapshot", qos);
    for (const auto &name : names_)
      layer_pubs_.push_back(node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/height_maps/" + name, qos));
  }

  void PostUpdate(const sim::UpdateInfo &info, const sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    const auto capture = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    const auto wall = std::chrono::steady_clock::now();
    if (capture == last_check_) return;
    if (last_check_ >= 0 && capture > last_check_ && wall - last_check_wall_ < std::chrono::milliseconds(500)) return;
    if (capture < last_check_) signature_.clear();
    last_check_ = capture;
    last_check_wall_ = wall;
    try {
      if (!ecm.HasEntity(world_)) throw std::runtime_error("height map world disappeared");
      std::map<sim::Entity, Collision> collisions;
      json entities = json::array();
      size_t checked = 0;
      ecm.Each<c::Collision>([&](const sim::Entity &entity, const auto *) {
        ++checked;
        bool robot = false, attached = false;
        sim::Entity ancestor = entity;
        json ownership = json::array();
        while (ancestor != world_) {
          if (ancestor == sim::kNullEntity) throw std::runtime_error("collision has no world ancestry");
          if (ecm.Component<c::Model>(ancestor)) {
            const auto *name = ecm.Component<c::Name>(ancestor);
            if (!name) throw std::runtime_error("collision model name missing");
            if (name->Data() == robot_name_) robot = true;
            bool payload = false;
            if (const auto *plugins = ecm.Component<c::SystemPluginInfo>(ancestor))
              for (const auto &plugin : plugins->Data().plugins())
                if (plugin.name() == "astribot::KinematicPayload") payload = true;
            const auto execution = simulation::read_execution(ancestor, ecm);
            if (payload && !execution) throw std::runtime_error("kinematic payload execution unavailable: " + name->Data());
            if (execution) {
              if (execution->capture != capture || execution->pending || execution->accepted != execution->applied || !execution->error.empty())
                throw std::runtime_error("kinematic payload transition or stale execution: " + name->Data());
              if (execution->attached && (!execution->parent || !ecm.HasEntity(execution->parent)))
                throw std::runtime_error("attached payload parent unavailable");
              attached = attached || (execution->attached && execution->parent_model == robot_name_);
              ownership.push_back({{"model", ancestor}, {"attached", execution->attached}, {"parent", execution->parent},
                                   {"epoch", execution->epoch}, {"applied", execution->applied}});
            }
          }
          const auto *parent = ecm.Component<c::ParentEntity>(ancestor);
          if (!parent) throw std::runtime_error("collision parent metadata missing");
          ancestor = parent->Data();
        }
        if (robot) return true;
        if (attached) {
          entities.push_back({{"entity", entity}, {"excluded_attached", ownership}});
          return true;
        }
        const auto *geometry = ecm.Component<c::Geometry>(entity);
        if (!geometry) throw std::runtime_error("collision geometry missing");
        const auto pose = map_from_world_ * sim::worldPose(entity, ecm);
        if (!pose.IsFinite()) throw std::runtime_error("collision pose nonfinite");
        const auto &g = geometry->Data();
        if (g.Type() != sdf::GeometryType::BOX && g.Type() != sdf::GeometryType::MESH)
          throw std::runtime_error("unsupported environment collision geometry: " + std::to_string(static_cast<int>(g.Type())));
        json record = {{"entity", entity}, {"geometry", g.ToElement()->ToString("")},
                       {"pose", {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()}},
                       {"ownership", ownership}};
        if (g.MeshShape()) record["mesh_source"] = g.MeshShape()->FilePath();
        entities.push_back(std::move(record));
        collisions.emplace(entity, Collision{g, pose});
        return true;
      });
      if (!checked || collisions.empty()) throw std::runtime_error("height map collision inventory is empty");
      std::sort(entities.begin(), entities.end(), [](const json &a, const json &b) {
        return a.at("entity").get<sim::Entity>() < b.at("entity").get<sim::Entity>();
      });
      json frame = {map_from_world_.Pos().X(), map_from_world_.Pos().Y(), map_from_world_.Pos().Z(),
                    map_from_world_.Rot().X(), map_from_world_.Rot().Y(), map_from_world_.Rot().Z(), map_from_world_.Rot().W()};
      const auto revision = hash(json{{"world", world_name_}, {"frame", frame_}, {"map_from_world", frame},
                                      {"ground", ground_}, {"reference", reference_}, {"bounds", bounds_},
                                      {"profile", profile_revision_}, {"collisions", entities}}.dump());
      if (revision != signature_) {
        rebuild(collisions, revision, capture);
        signature_ = revision;
        RCLCPP_INFO(node_->get_logger(), "Rebuilt %zu height layers from %zu environment collisions; revision=%s",
                    names_.size(), collisions.size(), revision.c_str());
      }
      last_error_.clear();
      // Reuse data only after this capture's complete collision/ownership/pose
      // traversal; no timer-only renewal of old world evidence.
      publish(capture);
    } catch (const std::exception &error) {
      signature_.clear();
      if (last_error_ != error.what()) RCLCPP_ERROR(node_->get_logger(), "Height map invalid: %s", error.what());
      last_error_ = error.what();
      if (!bundle_.grids.empty()) {
        for (auto &grid : bundle_.grids) std::fill(grid.data.begin(), grid.data.end(), int8_t(-1));
        bundle_.evidence_kind = "gazebo_collision_geometry_invalid";
        bundle_.map_revision = hash(last_error_);
        publish(capture);
      }
    }
  }

  ~HeightSliceMap() override {
    layer_pubs_.clear(); snapshot_pub_.reset(); node_.reset();
    if (context_) context_->shutdown("height slice map plugin unloaded");
  }
};
}  // namespace astribot

IGNITION_ADD_PLUGIN(astribot::HeightSliceMap, ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure, ignition::gazebo::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(astribot::HeightSliceMap, "astribot::HeightSliceMap")
