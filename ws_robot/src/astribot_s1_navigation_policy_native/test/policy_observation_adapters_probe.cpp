#include "astribot_s1_navigation_policy_native/policy_observation_adapters.hpp"
#include <iostream>
#include <iterator>
using namespace astribot::navigation::policy;
using J=AdapterJson;
namespace {
J canonical(J value){if(value.is_number_float()&&!std::isfinite(value.get<double>())){const double v=value.get<double>();return J{{"nonfinite",std::isnan(v)?"nan":(v>0?"inf":"-inf")}};}
  if(value.is_array()||value.is_object())for(auto& child:value)child=canonical(std::move(child));
  return value;}
Stamp stamp(const J& j){return Stamp(j.at("ns"),j.at("clock"),j.at("epoch"));}
J encode(const Stamp& s){return J{{"ns",s.ns},{"clock",s.clock},{"epoch",s.epoch}};}
J encode(const Vec3& v){return J{{"x",v.x},{"y",v.y},{"z",v.z}};}
J encode(const Covariance3& c){return J{{"values",c.values}};}
J encode(const MetricBox& b){return J{{"center_m",encode(b.center_m)},{"size_m",encode(b.size_m)},{"position_covariance_m2",encode(b.position_covariance_m2)},
  {"velocity_m_s",b.velocity_m_s?encode(*b.velocity_m_s):J(nullptr)},{"velocity_covariance_m2_s2",b.velocity_covariance_m2_s2?encode(*b.velocity_covariance_m2_s2):J(nullptr)}};}
J encode(const BearingCone& b){return J{{"direction",encode(b.direction)},{"half_angle_rad",b.half_angle_rad}};}
J encode(const PixelScalar& p){return p.is_integer()?integer_json(p.integer()):J(p.floating());}
J encode(const ImageBox& b){return J{{"camera_id",b.camera_id},{"image_width_px",b.image_width_px},{"image_height_px",b.image_height_px},{"xmin_px",encode(b.xmin_px)},{"ymin_px",encode(b.ymin_px)},{"xmax_px",encode(b.xmax_px)},{"ymax_px",encode(b.ymax_px)}};}
J encode(const Observation& o){return J{{"sensor_id",o.sensor_id},{"measurement_id",o.measurement_id},{"source_track_id",o.source_track_id?J(*o.source_track_id):J(nullptr)},
  {"capture_stamp",encode(o.capture_stamp)},{"received_at",encode(o.received_at)},{"valid_until",encode(o.valid_until)},{"frame_id",o.frame_id},{"calibration_epoch",o.calibration_epoch},
  {"geometry",std::visit([](const auto& g){return encode(g);},o.geometry)},{"geometry_quality",o.geometry_quality},{"class_probabilities",o.class_probabilities},
  {"provenance",o.provenance},{"velocity_observable",o.velocity_observable},{"spatial_occupancy",o.spatial_occupancy}};}
AdapterOptions options(const J& j){AdapterOptions o;
#define OPT(name) if(j.contains(#name))o.name=j.at(#name).get<decltype(o.name)>()
  OPT(max_packet_bytes);OPT(max_observations);OPT(max_points);OPT(calibration_epoch);OPT(position_variance_m2);
#undef OPT
  if(j.contains("sensor_id")&&!j["sensor_id"].is_null())o.sensor_id=j["sensor_id"].get<std::string>();
  return o;}
PointCloudPacket cloud(const J& j){PointCloudPacket p;p.sec=j.at("sec");p.nanosec=j.at("nanosec");p.frame_id=j.at("frame_id");p.width=j.at("width");p.height=j.at("height");p.point_step=j.at("point_step");p.row_step=j.at("row_step");p.is_bigendian=j.at("is_bigendian");p.is_dense=j.at("is_dense");p.data=j.at("data").get<std::vector<std::uint8_t>>();
  for(const auto& f:j.at("fields"))p.fields.push_back(CloudField{f.at("name"),f.at("offset"),f.at("datatype"),f.at("count")});
  return p;}
}
int main(){try{
  const auto input=parse_integer_json(std::string(std::istreambuf_iterator<char>(std::cin),{}));J calls=J::array();std::size_t now_count=0,steady_count=0;
  const auto times=input.value("now",J::array({J{{"ns",100},{"clock","ros"},{"epoch",4}},J{{"ns",200},{"clock","other"},{"epoch",9}}}));
  const auto steadies=input.value("steady",J::array({1000,1001,1002,1003,1004}));
  AdapterNow now=[&](){calls.push_back(J::array({"now",now_count}));const auto& value=times[std::min(now_count++,times.size()-1)];if(value.contains("error"))throw AdapterError("RuntimeError",value.at("error"));return stamp(value);};
  AdapterSteadyNow steady=[&](){calls.push_back(J::array({"steady",steady_count}));return steadies[std::min(steady_count++,steadies.size()-1)].get<std::int64_t>();};
  LookupTransform tf=[&](const std::string& target,const std::string& source,std::int64_t ns){calls.push_back(J::array({"tf",target,source,ns}));const auto value=input.value("transform",J{{"translation",J::array({0.,0.,0.})},{"rotation",J::array({0.,0.,0.,1.})}});
    if(value.contains("error"))throw AdapterError("RuntimeError",value.at("error"));
    const auto q=value.at("rotation").get<std::array<double,4>>();return AdapterTransform{value.at("translation").get<std::array<double,3>>(),AdapterQuaternion{q[0],q[1],q[2],q[3]}};};
  if(input.value("use_default_steady",false))steady={};
  auto adapter=make_observation_adapter(input.value("adapter",std::string("vision_json")),AdapterProfile{input.value("timeout",.5),input.value("tracking_frame",std::string("odom"))},tf,now,steady,options(input.value("options",J::object())));
  if(input.contains("raw_options"))adapter->set_raw_options(input.at("raw_options"));
  J output=J::array();std::map<std::string,std::shared_ptr<PointCloudPacket>> packets;
  for(const auto& op:input.at("operations")){J response;std::shared_ptr<PointCloudPacket> packet;
    try{J result=nullptr;
      if(op.value("action",std::string{})=="integer_dump")result=dump_integer_json(integer_json(Integer("0x"+op.at("hex").get<std::string>())));
      else if(op.value("action",std::string{})=="json")result=canonical(parse_integer_json(op.at("data")));
      else if(op.value("action",std::string{})=="metadata"){const auto m=adapter->last_metadata();result=J{{"stamp_ns",m.stamp_ns},{"calibration_epoch",m.calibration_epoch},{"sensor_id",m.sensor_id},{"frame_id",m.frame_id},{"resolved_measurement_ids",m.resolved_measurement_ids}};}
      else if(op.value("action",std::string{})=="registry"){
        NativeAdapterRegistry registry;NativeAdapterFactory factory=[](AdapterProfile p,LookupTransform tf,AdapterNow n,AdapterSteadyNow s,AdapterOptions o){return make_observation_adapter("vision_json",std::move(p),std::move(tf),std::move(n),std::move(s),std::move(o));};
        registry.register_adapter("custom",{AdapterMessageKind::VISION_STRING,factory});
        auto custom=registry.make("custom",AdapterProfile{},tf,now,steady,AdapterOptions{});
        bool duplicate=false,python=false,mismatch=false;
        try{registry.register_adapter("vision_json",{AdapterMessageKind::VISION_STRING,factory});}catch(const AdapterError&){duplicate=true;}
        try{(void)make_observation_adapter("arbitrary.module:Class",AdapterProfile{},tf,now,steady,AdapterOptions{});}catch(const AdapterError&){python=true;}
        registry.register_adapter("wrong",{AdapterMessageKind::POINT_CLOUD2,factory});
        try{(void)registry.make("wrong",AdapterProfile{},tf,now,steady,AdapterOptions{});}catch(const AdapterError&){mismatch=true;}
        result=J{{"custom_kind",registry.message_kind("custom")==AdapterMessageKind::VISION_STRING},{"custom_factory",custom->message_kind()==AdapterMessageKind::VISION_STRING},{"duplicate_rejected",duplicate},{"python_plugin_rejected",python},{"mismatched_kind_rejected",mismatch}};
      }
      else if(op.value("action",std::string{})=="calibration")adapter->set_calibration_epoch(adapter_integer(op.at("value")));
      else{std::vector<Observation> observations;
        if(adapter->message_kind()==AdapterMessageKind::VISION_STRING){const auto data=op.contains("data")?op["data"].get<std::string>():dump_integer_json(op.at("packet"));observations=adapter->normalize(VisionPacket{data});}
        else {const auto id=op.value("packet_id",std::string{});if(op.value("reuse",false))packet=packets.at(id);
          else{packet=std::make_shared<PointCloudPacket>(cloud(op.at("packet")));if(!id.empty())packets[id]=packet;}observations=adapter->normalize(*packet);}
        result=J::array();for(const auto& o:observations)result.push_back(encode(o));}
      response=J{{"ok",true},{"result",result}};
    }catch(const ContractError& e){response=J{{"ok",false},{"type","ContractError"},{"code",to_string(e.code)},{"field",e.field}};}
    catch(const IntegerJsonError& e){response=J{{"ok",false},{"type",e.kind}};}
    catch(const AdapterError& e){response=J{{"ok",false},{"type",e.kind}};}
    catch(const std::exception& e){response=J{{"ok",false},{"type","RuntimeError"},{"unexpected",e.what()}};}
    response["last_packet"]=adapter->last_packet()?canonical(*adapter->last_packet()):J(nullptr);response["calls"]=calls;if(packet)response["data"]=packet->data;output.push_back(response);
  }
  std::cout<<dump_integer_json(output)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
