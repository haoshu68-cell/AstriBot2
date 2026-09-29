#pragma once

// Immutable SI-unit values shared by the native policy cores. No ROS or JSON transport.
#include "astribot_s1_navigation_policy_native/policy_numeric.hpp"
#include "astribot_s1_navigation_policy_native/policy_integer.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace astribot::navigation::policy {
enum class ErrorCode { INVALID_INPUT, CLOCK_MISMATCH, STALE_OBSERVATION, FUTURE_OBSERVATION,
  VERSION_MISMATCH, CAPABILITY_UNAVAILABLE, UNSAFE_DECISION };
const char* to_string(ErrorCode code);
struct ContractError : std::invalid_argument {
  const ErrorCode code;
  const std::string field;
  ContractError(ErrorCode code, std::string field);
};
void require(bool condition, const std::string& field, ErrorCode code=ErrorCode::INVALID_INPUT);
void finite(double value, const std::string& field, std::optional<double> minimum=std::nullopt);
void finite(const PixelScalar& value, const std::string& field, std::optional<double> minimum=std::nullopt);
void label(const std::string& value, const std::string& field);

struct Stamp {
  const std::int64_t ns;
  const std::string clock;
  const std::int64_t epoch;
  Stamp(std::int64_t ns, std::string clock, std::int64_t epoch);
  std::int64_t since(const Stamp& other) const;
  bool operator==(const Stamp& other) const;
  bool operator!=(const Stamp& other) const { return !(*this==other); }
};
struct Vec3 {
  const double x, y, z;
  Vec3(double x, double y, double z);
  bool operator==(const Vec3& other) const;
};
struct Covariance3 {
  const std::array<double,9> values;
  explicit Covariance3(std::array<double,9> values);
  bool operator==(const Covariance3& other) const { return values==other.values; }
};
struct CameraCalibration {
  const std::string camera_id, optical_frame;
  const Integer calibration_epoch;
  const Integer image_width_px, image_height_px;
  const std::array<double,9> intrinsic_matrix;
  const std::string distortion_model;
  const std::vector<double> distortion_coefficients;
  const std::optional<double> depth_unit_m;
  CameraCalibration(std::string camera_id, std::string optical_frame, Integer calibration_epoch,
    Integer image_width_px, Integer image_height_px, std::array<double,9> intrinsic_matrix,
    std::string distortion_model, std::vector<double> distortion_coefficients,
    std::optional<double> depth_unit_m=std::nullopt);
  bool operator==(const CameraCalibration& other) const;
};
struct ImageBox {
  const std::string camera_id;
  const Integer image_width_px, image_height_px;
  const PixelScalar xmin_px, ymin_px, xmax_px, ymax_px;
  ImageBox(std::string camera_id, Integer image_width_px, Integer image_height_px,
    PixelScalar xmin_px, PixelScalar ymin_px, PixelScalar xmax_px, PixelScalar ymax_px);
};
struct BearingCone {
  const Vec3 direction;
  const double half_angle_rad;
  BearingCone(Vec3 direction, double half_angle_rad);
};
struct MetricBox {
  const Vec3 center_m, size_m;
  const Covariance3 position_covariance_m2;
  const std::optional<Vec3> velocity_m_s;
  const std::optional<Covariance3> velocity_covariance_m2_s2;
  MetricBox(Vec3 center_m, Vec3 size_m, Covariance3 position_covariance_m2,
    std::optional<Vec3> velocity_m_s=std::nullopt,
    std::optional<Covariance3> velocity_covariance_m2_s2=std::nullopt);
};
using Geometry=std::variant<ImageBox,BearingCone,MetricBox>;
struct Observation {
  const std::string sensor_id, measurement_id;
  const std::optional<std::string> source_track_id;
  const Stamp capture_stamp, received_at, valid_until;
  const std::string frame_id;
  const Integer calibration_epoch;
  const Geometry geometry;
  const double geometry_quality;
  const std::vector<std::pair<std::string,double>> class_probabilities;
  const std::vector<std::string> provenance;
  const bool velocity_observable, spatial_occupancy;
  Observation(std::string sensor_id, std::string measurement_id, std::optional<std::string> source_track_id,
    Stamp capture_stamp, Stamp received_at, Stamp valid_until, std::string frame_id,
    Integer calibration_epoch, Geometry geometry, double geometry_quality,
    std::vector<std::pair<std::string,double>> class_probabilities, std::vector<std::string> provenance,
    bool velocity_observable=true, bool spatial_occupancy=false);
  void check_fresh(const Stamp& now, std::int64_t max_age_ns, std::int64_t future_tolerance_ns=0) const;
};
enum class Health { VALID, DEGRADED, UNAVAILABLE, STALE };
const char* to_string(Health health);
struct SensorHealth {
  const std::string sensor_id;
  const Health health;
  const Stamp stamp, valid_until;
  const std::string frame_id;
  const std::vector<BearingCone> coverage;
  const bool depth_available;
  const Integer calibration_epoch;
  const std::string reason;
  SensorHealth(std::string sensor_id, Health health, Stamp stamp, Stamp valid_until,
    std::string frame_id, std::vector<BearingCone> coverage, bool depth_available,
    Integer calibration_epoch, std::string reason);
};
struct Version {
  // Capture signedness before converting to the full wire domain. Validation is
  // deferred to Version so goal/field error ordering stays deterministic.
  struct Counter {
    const std::uint64_t value;
    const bool valid_integer;
    template<class T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
    Counter(T input) : value(static_cast<std::uint64_t>(input)),
      valid_integer([input] {
        if constexpr (std::is_same_v<T, bool>) return false;
        else if constexpr (std::is_signed_v<T>) return input >= 0;
        else return true;
      }()) {}
  };
  const std::string goal_id;
  const std::uint64_t path_revision, map_epoch, envelope_epoch, localization_epoch, clock_epoch;
  Version(std::string goal_id, Counter path_revision, Counter map_epoch,
    Counter envelope_epoch, Counter localization_epoch=0, Counter clock_epoch=0);
  bool operator==(const Version& other) const;
};
enum class Motion { CONTINUE, SLOW, HOLD, STOP, FOLLOW_COMMITTED_PATH, RETREAT };
enum class Planning { NONE, LOCAL, GLOBAL };
enum class Trigger { NONE, NEW_GOAL, PATH_RISK };
struct MotionLimits {
  const double linear_speed_m_s, angular_speed_rad_s, linear_accel_m_s2, angular_accel_rad_s2;
  MotionLimits(double linear_speed_m_s, double angular_speed_rad_s,
    double linear_accel_m_s2, double angular_accel_rad_s2);
};
struct Decision {
  const std::string decision_id, episode_id;
  const Version version;
  const Stamp issued_at, valid_until;
  const Motion motion;
  const Planning planning;
  const Trigger trigger;
  const MotionLimits limits;
  const std::string reason;
  const std::optional<std::string> request_id;
  const std::optional<std::uint64_t> committed_path_revision;
  Decision(std::string decision_id, std::string episode_id, Version version, Stamp issued_at,
    Stamp valid_until, Motion motion, Planning planning, Trigger trigger, MotionLimits limits,
    std::string reason, std::optional<std::string> request_id=std::nullopt,
    std::optional<Version::Counter> committed_path_revision=std::nullopt);
};
struct ExecutionContext {
  const Version version;
  const Stamp now;
  const bool required_inputs_valid, motion_enabled;
  const std::set<Planning> allowed_planning;
  const bool retreat_enabled;
  const MotionLimits baseline_limits;
  ExecutionContext(Version version, Stamp now, bool required_inputs_valid, bool motion_enabled,
    std::set<Planning> allowed_planning, bool retreat_enabled, MotionLimits baseline_limits);
};
void check_executable(const Decision& decision, const ExecutionContext& context);
struct Prediction {
  const std::int64_t offset_ns;
  const MetricBox geometry;
  Prediction(std::int64_t offset_ns, MetricBox geometry);
};
struct PredictionModel {
  const Vec3 velocity;
  const double variance_m2_s2;
  const std::vector<std::pair<std::int64_t,double>> steps;
  PredictionModel(Vec3 velocity, double variance_m2_s2,
    std::vector<std::pair<std::int64_t,double>> steps);
};
struct TrackedObstacle {
  const std::string fused_track_id, frame_id;
  const Stamp stamp;
  const MetricBox geometry;
  const std::vector<Prediction> predictions;
  const std::vector<std::string> provenance;
  const std::optional<PredictionModel> prediction_model;
  TrackedObstacle(std::string fused_track_id, std::string frame_id, Stamp stamp, MetricBox geometry,
    std::vector<Prediction> predictions, std::vector<std::string> provenance,
    std::optional<PredictionModel> prediction_model=std::nullopt);
};
struct WorldSnapshot {
  const Version version;
  const Stamp stamp;
  const std::string frame_id;
  const std::vector<TrackedObstacle> tracks;
  const std::vector<Observation> unassociated;
  const std::vector<SensorHealth> sensors;
  const std::int64_t observation_seq;
  WorldSnapshot(Version version, Stamp stamp, std::string frame_id,
    std::vector<TrackedObstacle> tracks, std::vector<Observation> unassociated,
    std::vector<SensorHealth> sensors, std::int64_t observation_seq);
};
}  // namespace astribot::navigation::policy
