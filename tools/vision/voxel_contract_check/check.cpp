// Uses the installed Nav2 VoxelLayer, with explicit static test observations.
// No simulator, robot topics, controllers, navigation goals or motion commands.
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <nav2_costmap_2d/voxel_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

class ProbeLayer : public nav2_costmap_2d::VoxelLayer {
public:
  unsigned actualLayers() {return voxel_grid_.sizeZ();}
};

nav2_costmap_2d::Observation observation(double x, double z) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(1);
  sensor_msgs::PointCloud2Iterator<float> it(cloud, "x");
  it[0] = x; it[1] = 0.55; it[2] = z;
  geometry_msgs::msg::Point origin;
  origin.x = 0.05; origin.y = 0.55; origin.z = z;
  return {origin, cloud, 5.0, 0.0, 5.0, 0.0};
}

int main(int argc, char** argv) {
  if (argc != 4) {std::cerr << "Usage: check mppi.yaml rpp.yaml output.json\n"; return 2;}
  rclcpp::init(argc, argv);
  nlohmann::json result = {{"scope", "installed Nav2 plugin, synthetic static observations; not Gazebo acceptance"},
                            {"profiles", nlohmann::json::array()}};
  bool all = true;
  int index = 0;
  for (int input = 1; input <= 2; ++input) {
    auto source = YAML::LoadFile(argv[input]);
    for (const std::string costmap : {"local_costmap", "global_costmap"}) {
      auto c = source[costmap][costmap]["ros__parameters"]["obstacle_layer"];
      const int layers = c["z_voxels"].as<int>();
      const double dz = c["z_resolution"].as<double>();
      const double origin = c["origin_z"].as<double>();
      const double max_height = c["max_obstacle_height"] ? c["max_obstacle_height"].as<double>() : 2.0;
      rclcpp::NodeOptions options;
      options.parameter_overrides({
        {"obstacle_layer.z_voxels", layers}, {"obstacle_layer.z_resolution", dz},
        {"obstacle_layer.origin_z", origin}, {"obstacle_layer.max_obstacle_height", max_height},
        {"obstacle_layer.unknown_threshold", c["unknown_threshold"].as<int>()},
        {"obstacle_layer.mark_threshold", c["mark_threshold"].as<int>()},
        {"obstacle_layer.observation_sources", ""}, {"obstacle_layer.publish_voxel_map", false},
        {"obstacle_layer.footprint_clearing_enabled", false}});
      auto node = std::make_shared<nav2_util::LifecycleNode>("voxel_contract_"+std::to_string(index++), "", options);
      tf2_ros::Buffer tf(node->get_clock());
      nav2_costmap_2d::LayeredCostmap map("map", false, true);
      map.resizeMap(20, 20, .1, 0, 0);
      auto layer = std::make_shared<ProbeLayer>();
      map.addPlugin(layer);
      layer->initialize(&map, "obstacle_layer", &tf, node,
                        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
      nlohmann::json row = {{"source", argv[input]}, {"costmap", costmap},
        {"configured_layers", layers}, {"actual_layers", layer->actualLayers()},
        {"resolution_m", dz}, {"actual_capacity_top_m", origin+dz*layer->actualLayers()},
        {"max_obstacle_height_m", max_height}, {"cases", nlohmann::json::array()}};
      bool pass = layers > 0 && layers <= 16 && int(layer->actualLayers()) == layers &&
                  dz > 0 && origin+dz*layers > max_height;
      for (const double z : {.01, .149, .15, .151, .3, .9, 1.49, 1.59, 1.61, 1.79, 1.81, 1.99, 2.01, 2.4}) {
        layer->clearStaticObservations(true, true);
        layer->reset();
        auto obs = observation(.55, z);
        layer->addStaticObservation(obs, true, false);
        double xmin=1e9, ymin=1e9, xmax=-1e9, ymax=-1e9;
        layer->updateBounds(0, 0, 0, &xmin, &ymin, &xmax, &ymax);
        const bool marked = layer->getCost(5, 5) == nav2_costmap_2d::LETHAL_OBSTACLE;
        const bool expected = z >= origin && z <= max_height;
        bool cleared = false;
        if (marked) {
          layer->clearStaticObservations(true, true);
          auto ray = observation(1.25, z);
          layer->addStaticObservation(ray, false, true);
          layer->updateBounds(0, 0, 0, &xmin, &ymin, &xmax, &ymax);
          cleared = layer->getCost(5, 5) == nav2_costmap_2d::FREE_SPACE;
        }
        const bool ok = marked == expected && (!expected || cleared);
        pass = pass && ok;
        row["cases"].push_back({{"height_m", z}, {"expected_marked", expected},
                               {"marked", marked}, {"ray_cleared", cleared}, {"pass", ok}});
      }
      row["pass"] = pass;
      all = all && pass;
      result["profiles"].push_back(row);
    }
  }
  result["pass"] = all;
  std::ofstream(argv[3]) << result.dump(2) << '\n';
  std::cout << "Voxel contracts " << (all ? "PASS" : "FAIL") << "; report=" << argv[3] << '\n';
  rclcpp::shutdown();
  return all ? 0 : 1;
}
