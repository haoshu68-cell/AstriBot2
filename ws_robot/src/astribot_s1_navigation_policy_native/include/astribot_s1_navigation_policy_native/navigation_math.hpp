#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>
#include <tuple>

namespace astribot::navigation {

double stopping_horizon(const std::vector<double>& command,
                        double reaction_time,
                        double linear_brake,
                        double angular_brake,
                        double linear_stop_delay);

std::vector<double> body_pose(const std::vector<double>& command, double t);
std::vector<double> body_to_world_xy(double vx, double vy, double yaw);

double sampling_margin(const std::vector<double>& command,
                       double half_length,
                       double half_width,
                       double step);

std::vector<double> footprint_axes(double length, double width, double yaw);
bool scan_usable(const std::vector<double>& ranges, double range_min,
                 double range_max, double angle_min, double angle_increment,
                 double minimum_fraction);
std::vector<std::array<std::int64_t, 2>> occupied_cells(
    const std::vector<std::array<double, 2>>& points, double resolution);
bool angular_box_free(const std::vector<std::array<double, 2>>& corners,
                      const std::vector<double>& ranges, double range_min,
                      double range_max, double angle_min,
                      double angle_increment, double resolution);
std::pair<bool, std::string> posture_out_of_bounds(
    double z, double roll, double pitch, double normal_height,
    double max_height_deviation, double max_tilt_rad);
bool is_degenerate_attitude_source(
    const std::vector<std::array<double, 3>>& samples, double eps,
    std::size_t min_samples);
std::string describe_monitor_state(bool enabled, bool degenerate, bool tripped);
std::pair<std::string, std::string> evaluate_posture(
    bool enabled, const std::vector<std::array<double, 3>>& samples, double z,
    double roll, double pitch, double normal_height,
    double max_height_deviation, double max_tilt_rad, std::size_t min_samples);
std::vector<double> lateral_variants(const std::vector<double>& route_xy,
                                     double maximum);
std::vector<double> scan_coverage(const std::vector<double>& ranges,
                                  double range_min, double range_max,
                                  double angle_min, double angle_increment,
                                  double body_yaw);
std::vector<double> movement_directions(double vx, double vy, double wz);
bool coverage_allows_motion(const std::vector<double>& cones_xy_half,
                            double vx, double vy, double wz);
struct PathAssessmentResult {
  bool blocked;
  double conflict_time_s;
  std::string status;
  double distance_m;
};
PathAssessmentResult assess_path(bool known, bool blocked, double stamp_s,
                                 double received_wall_s, double distance_m,
                                 double now_s, double wall_s,
                                 double path_risk_timeout_s,
                                 double max_speed_m_s, double clearance_margin_m,
                                 double payload_extra_margin_m,
                                 bool legacy_blocked);
bool yield_requires_stop(bool immediate, bool uncertain, bool blocked,
                         double conflict_time_s, double max_speed_m_s,
                         double reaction_time_s, double brake_deceleration_m_s2,
                         double linear_stop_delay_s);
struct SelectionResult {
  std::string motion;
  double speed;
  std::string reason;
  int episode;
};
class YieldPolicy {
public:
  YieldPolicy(double max_speed_m_s, double narrow_speed_m_s,
              double reaction_time_s, double brake_deceleration_m_s2,
              double linear_stop_delay_s, double wait_budget_s,
              double clear_hold_s);
  SelectionResult select(bool immediate, bool blocked, bool uncertain,
                        double conflict_time_s, bool valid, double now);
  // Apply current envelope limits without resetting a waiting/clear episode.
  void update_profile(double max_speed_m_s, double narrow_speed_m_s,
                      double reaction_time_s, double brake_deceleration_m_s2,
                      double linear_stop_delay_s, double wait_budget_s,
                      double clear_hold_s);
  std::vector<double> state() const;

private:
  double max_speed_m_s_;
  double narrow_speed_m_s_;
  double stop_time_s_;
  double wait_budget_s_;
  double clear_hold_s_;
  bool has_blocked_at_;
  double blocked_at_;
  bool has_clear_at_;
  double clear_at_;
  bool has_last_time_;
  double last_time_;
  bool held_;
  int episode_;
  bool in_episode_;
};

class ExecutionContext {
public:
  ExecutionContext();
  void task(const std::string& identifier, const std::string& state,
            std::int64_t sequence);
  void task_wire(const std::string& identifier, const std::string& state,
                 std::uint64_t sequence);
  bool map(const std::string& key);
  bool localization(const std::vector<double>& pose, double position_limit,
                    double angle_limit);
  void path();
  void set_version(const std::string& goal_id, std::uint64_t path_revision,
                   std::uint64_t map_epoch, std::uint64_t envelope_epoch,
                   std::uint64_t localization_epoch,
                   std::uint64_t clock_epoch);
  std::tuple<std::string, std::uint64_t, std::uint64_t, std::uint64_t,
             std::uint64_t, std::uint64_t> version() const;

private:
  std::string goal_id_;
  std::uint64_t path_revision_;
  std::uint64_t map_epoch_;
  std::uint64_t envelope_epoch_;
  std::uint64_t localization_epoch_;
  std::uint64_t clock_epoch_;
  std::uint64_t sequence_;
  bool has_sequence_;
  std::string map_key_;
  bool has_map_key_;
  bool has_transform_;
  std::array<double, 3> transform_;
};

class PlanningSessionState {
public:
  PlanningSessionState(std::string session_id, std::int64_t request_timeout_ns,
                       std::int64_t episode_timeout_ns,
                       std::int64_t max_requests_per_goal);
  void activate(const std::string& goal_id, std::int64_t path_revision,
                std::int64_t map_epoch, std::int64_t envelope_epoch,
                std::int64_t localization_epoch, std::int64_t clock_epoch,
                std::int64_t now_ns);
  void finish(const std::string& goal_id);
  bool clear_blockage(std::int64_t now_ns, std::int64_t clock_epoch);
  std::tuple<int, std::string, std::int64_t, std::int64_t, std::int64_t> request(
      const std::string& goal_id, std::int64_t path_revision,
      std::int64_t map_epoch, std::int64_t envelope_epoch,
      std::int64_t localization_epoch, std::int64_t clock_epoch,
      std::int64_t observation_seq, std::int64_t now_ns);
  bool response_current(const std::string& request_id,
                        const std::string& goal_id,
                        std::int64_t path_revision, std::int64_t map_epoch,
                        std::int64_t envelope_epoch,
                        std::int64_t localization_epoch,
                        std::int64_t clock_epoch, std::int64_t now_ns);
  int failure_reason(std::int64_t now_ns, std::int64_t clock_epoch);
  void retire(const std::string& request_id);

private:
  bool same_version(const std::string& goal_id, std::int64_t path_revision,
                    std::int64_t map_epoch, std::int64_t envelope_epoch,
                    std::int64_t localization_epoch,
                    std::int64_t clock_epoch) const;
  bool advance(std::int64_t now_ns, std::int64_t clock_epoch);
  std::string session_id_;
  std::int64_t request_timeout_ns_;
  std::int64_t episode_timeout_ns_;
  std::int64_t max_requests_per_goal_;
  bool has_version_;
  std::string goal_id_;
  std::int64_t path_revision_;
  std::int64_t map_epoch_;
  std::int64_t envelope_epoch_;
  std::int64_t localization_epoch_;
  std::int64_t clock_epoch_;
  bool has_pending_;
  std::string pending_id_;
  std::int64_t pending_issued_ns_;
  std::int64_t pending_issued_epoch_;
  std::int64_t pending_valid_until_ns_;
  bool has_last_time_;
  std::int64_t last_time_ns_;
  std::int64_t last_time_epoch_;
  bool has_blocked_at_;
  std::int64_t blocked_at_ns_;
  bool clock_fault_;
  std::int64_t serial_;
  std::int64_t episode_;
  std::int64_t attempts_;
};

struct TimeStepResult {
  double now;
  double dt;
  double wall_dt;
  bool running;
  bool reset;
  bool stop_commands;
};

class ControlTime {
public:
  ControlTime(bool simulated, double stall_timeout_s);
  TimeStepResult advance(double ros, double wall);
  bool accepts(double capture) const;
  bool fresh(double capture, double received_wall, double ttl, double ros,
             double wall) const;
  bool command_fresh(double received_ros, double received_wall, double ttl,
                     double ros, double wall) const;
  std::vector<double> state() const;
  bool simulated() const { return simulated_; }

private:
  bool simulated_;
  double stall_timeout_s_;
  bool has_last_;
  double last_ros_;
  double last_wall_;
  bool has_progress_;
  double progress_wall_;
  bool stalled_;
  double capture_floor_;
  double command_floor_;
};
std::vector<double> costmap_clearing_ranges(const std::vector<double>& ranges,
                                            double range_max,
                                            double max_marking_range);
std::vector<double> clearance_many_rect(const std::vector<double>& x,
                                        const std::vector<double>& y,
                                        const std::vector<double>& yaw,
                                        const std::vector<double>& lower,
                                        const std::vector<double>& upper,
                                        double half_length, double half_width,
                                        double margin);
std::vector<double> motion_clearance_rect(
    const std::vector<double>& command, const std::vector<double>& begin,
    const std::vector<double>& end, const std::vector<double>& lower,
    const std::vector<double>& upper, double half_length, double half_width,
    double clearance_margin, double payload_extra_margin,
    const std::vector<double>& origin);
std::vector<double> path_position_batch(
    const std::vector<double>& path_xy, const std::vector<double>& distances,
    const std::vector<double>& fallback);
std::vector<double> remaining_path(
    const std::vector<double>& path_xy, double robot_x, double robot_y);
std::vector<double> path_samples(
    const std::vector<double>& route_xy, double reach, double half_length,
    double half_width, double clearance_margin);

class CommandRestriction {
public:
  CommandRestriction();
  std::vector<double> apply(const std::vector<double>& command,
                            double cap, double angular_cap, bool stop,
                            double dt, double max_acceleration,
                            double max_angular_acceleration,
                            bool allow_zero_dt);
  void reset();
  void set_state(const std::vector<double>& output, bool recovering);
  std::vector<double> output() const;
  bool recovering() const { return recovering_; }

private:
  std::vector<double> output_;
  bool recovering_;
};

}  // namespace astribot::navigation
