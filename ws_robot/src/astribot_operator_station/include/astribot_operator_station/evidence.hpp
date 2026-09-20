#pragma once
#include <filesystem>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
namespace astribot_operator_station {
using Json = nlohmann::json;
// Ordering uses the recorder's receive sequence, not source ROS time (which can rewind).
std::vector<Json> read_events(const std::filesystem::path & path);
Json parameters_at(const std::vector<Json> & events, uint64_t sequence);
Json parameter_value(int type, const Json & value);
}
