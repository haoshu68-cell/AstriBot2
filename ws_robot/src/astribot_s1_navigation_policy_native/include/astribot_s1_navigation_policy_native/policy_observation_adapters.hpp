#pragma once
#include "astribot_s1_navigation_policy_native/policy_contracts.hpp"
#include <functional>
#include <map>
#include <memory>
#include "astribot_s1_navigation_policy_native/policy_integer_json.hpp"

namespace astribot::navigation::policy {
using AdapterJson=IntegerJson;
struct AdapterError : std::invalid_argument {
  const std::string kind;
  AdapterError(std::string kind,std::string reason);
};
// Python int(value) coercion preserves arbitrary integer precision.
// Exposed so observer raw-metadata transactions retain the same conversion.
Integer adapter_integer(const AdapterJson& value);
std::int64_t adapter_int64(const AdapterJson& value);
double adapter_float(const AdapterJson& value);
struct AdapterProfile { double sensor_timeout_s=.5;std::string tracking_frame="odom"; };
struct AdapterOptions {
  Integer max_packet_bytes=1048576,max_observations=128,max_points=32768;
  std::optional<std::string> sensor_id;
  Integer calibration_epoch=0;
  double position_variance_m2=.0025;
};
struct AdapterQuaternion { double x=0.,y=0.,z=0.,w=1.; };
struct AdapterTransform {
  std::array<double,3> translation{0.,0.,0.};
  AdapterQuaternion rotation;
};
using LookupTransform=std::function<AdapterTransform(const std::string& target_frame,
  const std::string& source_frame,std::int64_t capture_ns)>;
using AdapterNow=std::function<Stamp()>;
using AdapterSteadyNow=std::function<std::int64_t()>;
struct VisionPacket { std::string data; };
struct CloudField {
  std::string name;
  std::uint32_t offset=0;
  std::uint8_t datatype=0;  // sensor_msgs/PointField 1..8, unchanged on transport.
  std::uint32_t count=0;
};
struct PointCloudPacket {
  std::int32_t sec=0;
  std::uint32_t nanosec=0,width=0,height=0,point_step=0,row_step=0;
  std::string frame_id;
  std::vector<CloudField> fields;  // ROS field order is significant.
  bool is_bigendian=false,is_dense=false;
  std::vector<std::uint8_t> data; // Deliberately mutable: Humble swaps xyz in place.
};
enum class AdapterMessageKind { VISION_STRING, POINT_CLOUD2 };
struct PacketMetadata {
  const std::int64_t stamp_ns;
  const Integer calibration_epoch;
  const std::string sensor_id,frame_id;
  // Preserve the transport value without early validation; the observer resolves
  // evidence after ingest/health record, in the original transaction order.
  const AdapterJson resolved_measurement_ids;
};
class ObservationAdapter {
 public:
  virtual ~ObservationAdapter()=default;
  virtual AdapterMessageKind message_kind() const noexcept=0;
  virtual std::vector<Observation> normalize(const VisionPacket& packet)=0;
  virtual std::vector<Observation> normalize(PointCloudPacket& packet)=0;
  virtual const AdapterOptions& options() const noexcept=0;
  virtual void set_raw_options(AdapterJson options)=0;
  virtual const std::optional<AdapterJson>& raw_options() const noexcept=0;
  virtual void set_calibration_epoch(Integer epoch)=0;
  virtual const std::optional<AdapterJson>& last_packet() const noexcept=0;
  virtual PacketMetadata last_metadata() const=0;
};
class VisionAdapter : public ObservationAdapter {
 public:
  VisionAdapter(AdapterProfile profile,LookupTransform tf,AdapterNow now,
    AdapterSteadyNow steady_now={},AdapterOptions options={});
  AdapterMessageKind message_kind() const noexcept override;
  std::vector<Observation> normalize(const VisionPacket& packet) override;
  std::vector<Observation> normalize(PointCloudPacket& packet) override;
  const AdapterOptions& options() const noexcept override { return options_; }
  // Configuration transport is retained until the source's original read point.
  // Calling options() still returns the typed defaults/options supplied at creation.
  void set_raw_options(AdapterJson options) override;
  const std::optional<AdapterJson>& raw_options() const noexcept override { return raw_options_; }
  void set_calibration_epoch(Integer epoch) override;
  const std::optional<AdapterJson>& last_packet() const noexcept override { return last_packet_; }
  PacketMetadata last_metadata() const override;
 protected:
  std::vector<Observation> normalize_parsed(AdapterJson data);
  void check_packet_budget(std::size_t packet_characters) const;
  AdapterJson option(const std::string& name,AdapterJson fallback) const;
  AdapterProfile profile_;
  LookupTransform tf_;
  AdapterNow now_;
  AdapterSteadyNow steady_now_;
  AdapterOptions options_;
  std::optional<AdapterJson> raw_options_;
  std::optional<AdapterJson> last_packet_;
};
class PointCloudBoxAdapter final : public VisionAdapter {
 public:
  using VisionAdapter::VisionAdapter;
  using VisionAdapter::normalize;
  AdapterMessageKind message_kind() const noexcept override;
  std::vector<Observation> normalize(PointCloudPacket& packet) override;
};
// Explicit native extension point; factories return C++ implementations. No
// Python import, embedding or module:Class fallback exists at this boundary.
using NativeAdapterFactory=std::function<std::unique_ptr<ObservationAdapter>(AdapterProfile,
  LookupTransform,AdapterNow,AdapterSteadyNow,AdapterOptions)>;
struct NativeAdapterRegistration { AdapterMessageKind message_kind;NativeAdapterFactory factory; };
class NativeAdapterRegistry {
 public:
  void register_adapter(std::string name,NativeAdapterRegistration registration);
  AdapterMessageKind message_kind(const std::string& name) const;
  std::unique_ptr<ObservationAdapter> make(const std::string& name,AdapterProfile profile,
    LookupTransform tf,AdapterNow now,AdapterSteadyNow steady_now={},AdapterOptions options={}) const;
 private:
  std::map<std::string,NativeAdapterRegistration> extensions_;
};
AdapterMessageKind adapter_message_kind(const std::string& name);
std::unique_ptr<ObservationAdapter> make_observation_adapter(const std::string& name,AdapterProfile profile,
  LookupTransform tf,AdapterNow now,AdapterSteadyNow steady_now={},AdapterOptions options={});
Vec3 rotate_observation_vector(const Vec3& vector,const AdapterQuaternion& quaternion);
}  // namespace astribot::navigation::policy
