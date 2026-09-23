#pragma once
#include <astribot_payload_msgs/msg/attachment_observation.hpp>
#include <astribot_payload_msgs/msg/attachment_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <nlohmann/json.hpp>
#include <set>
#include <optional>
#include <functional>

namespace astribot::payload {
using Observation = astribot_payload_msgs::msg::AttachmentObservation;
using State = astribot_payload_msgs::msg::AttachmentState;
using Objects = std::vector<moveit_msgs::msg::AttachedCollisionObject>;
constexpr int64_t kLease = 500000000;
int64_t ns(const builtin_interfaces::msg::Time &stamp);
builtin_interfaces::msg::Time stamp(int64_t value);
std::string digest(const std::string &text);
// Reject unsupported/incomplete geometry before it can become a load envelope.
nlohmann::json canonical(const Objects &objects, const std::set<std::string> &allowed_links);
// Humble does not retain attachment weight; only zero readback means unavailable.
// Source mass/digest stay intact. Pose-only roundtrip tolerance is 1e-12.
bool scene_matches(const Objects &physical, const Objects &readback,
                   const std::set<std::string> &allowed_links);
struct Config {
  std::string environment, session, source, ledger_epoch;
  std::set<std::string> allowed_links;
};
struct Receipt { int64_t ros_at, steady_at; };
struct Ticket { uint64_t generation; int64_t ros_at, steady_at; };
using Commit = std::function<void(const nlohmann::json &)>;
class Ledger {
public:
  Ledger(Config config, Commit commit);
  bool observe(const Observation &value, int64_t ros_now, int64_t steady_now, std::optional<Receipt> receipt=std::nullopt);
  void source_fault(const std::string &reason, int64_t ros_now, int64_t steady_now);
  std::optional<Ticket> request(int64_t ros_now, int64_t steady_now);
  bool reconcile(const Ticket &ticket, const moveit_msgs::msg::PlanningScene &scene,
                 int64_t ros_now, int64_t steady_now);
  State state(int64_t ros_now, int64_t steady_now);
private:
  void invalidate(const std::string &reason);
  bool clock(int64_t ros_now, int64_t steady_now);
  void persist(const nlohmann::json &record);
  bool fresh(int64_t ros_now, int64_t steady_now) const;
  Config config_; Commit commit_; State state_;
  uint64_t generation_=0;
  int64_t last_ros_=-1, last_steady_=-1, received_=-1, scene_ros_=-1, scene_steady_=-1;
  int64_t observed_wall_deadline_=-1, recovery_after_=-1;
  bool have_=false, clock_latched_=false, storage_failed_=false, source_rejected_=false;
  uint64_t clock_floor_=0;
  std::set<std::string> retired_;
};
// Persistent history is audit only, never read to authorize movement at startup.
class Journal {
public:
  explicit Journal(const std::string &path);
  ~Journal();
  Journal(const Journal &)=delete;
  Journal &operator=(const Journal &)=delete;
  void append(const nlohmann::json &record);
private: int fd_=-1; bool failed_=false;
};
} // namespace astribot::payload
