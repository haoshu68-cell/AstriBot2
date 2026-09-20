#include "astribot_operator_station/evidence.hpp"
#include <fstream>
#include <stdexcept>
namespace astribot_operator_station {
std::vector<Json> read_events(const std::filesystem::path & path) {
  if (std::filesystem::file_size(path) > 64 * 1024 * 1024) {
    throw std::runtime_error("Event file exceeds 64 MiB; export a smaller interval");
  }
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot open events file");
  std::vector<Json> result;
  std::string line;
  uint64_t last = 0;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    auto e = Json::parse(line);
    auto seq = e.at("sequence").get<uint64_t>();
    if (seq <= last) throw std::runtime_error("Non-monotonic event sequence");
    last = seq;
    result.push_back(std::move(e));
  }
  return result;
}
Json parameters_at(const std::vector<Json> & events, uint64_t sequence) {
  Json state = Json::object();
  for (const auto & e : events) {
    if (e.at("sequence").get<uint64_t>() > sequence) break;
    const auto kind = e.value("kind", "");
    if (kind == "parameter_gap" || kind == "node_instance") {
      const auto node = e.at("node").get<std::string>();
      state[node] = {{"quality", "unknown"}, {"reason", e.value("reason", "identity_changed")},
        {"node_instance_id",e.value("node_instance_id","")}};
    } else if (kind == "parameter_snapshot" || kind == "parameter_event") {
      const auto node = e.at("node").get<std::string>();
      if (!state[node].is_object()) state[node] = Json::object();
      const auto instance=e.value("node_instance_id","");
      if(state[node].value("node_instance_id","")!=instance)state[node]=Json::object();
      if (kind == "parameter_snapshot") state[node] = Json::object();
      state[node]["node_instance_id"]=instance;
      state[node]["quality"] = kind == "parameter_snapshot" ? "observed" : "event_only";
      // Neither a service read nor a parameter event proves controller-cycle application.
      state[node]["effective_confirmed"] = false;
      state[node]["sequence"] = e.at("sequence");
      for (auto it = e.at("values").begin(); it != e.at("values").end(); ++it)
        state[node]["values"][it.key()] = it.value();
    }
  }
  // A later readback can reveal a missed change. Earlier replay must then show uncertainty.
  for(const auto & e:events) {
    if(e.value("kind","")!="parameter_gap" || !e.contains("uncertain_after_sequence"))continue;
    if(sequence>e.at("uncertain_after_sequence").get<uint64_t>() && sequence<e.at("sequence").get<uint64_t>()) {
      state[e.at("node").get<std::string>()]={{"quality","unknown"},
        {"reason",e.value("reason","gap")},{"discovered_at_sequence",e.at("sequence")}};
    }
  }
  return state;
}
Json parameter_value(int type, const Json & value) {
  return {{"type", type}, {"value", value}, {"available", type != 0}};
}
}
