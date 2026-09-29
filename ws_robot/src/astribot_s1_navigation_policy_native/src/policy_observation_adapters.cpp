#include "astribot_s1_navigation_policy_native/policy_observation_adapters.hpp"
#include "astribot_s1_robot_geometry/geometry_kernels.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <climits>
#include <cstdlib>
#include <limits>
#include <set>

namespace astribot::navigation::policy {
namespace {
using J=AdapterJson;
[[noreturn]] void fail(const char* kind,const std::string& message){throw AdapterError(kind,message);}
const J& field(const J& value,const std::string& key){
  if(!value.is_object())fail("TypeError","object field access requires an object");
  const auto it=value.find(key);if(it==value.end())fail("KeyError",key);return *it;
}
J get(const J& value,const std::string& key,J fallback=nullptr){
  if(!value.is_object())fail("AttributeError","packet has no get method");
  const auto it=value.find(key);return it==value.end()?fallback:*it;
}
std::vector<std::string> characters(const std::string& value){
  std::vector<std::string> result;
  for(std::size_t at=0;at<value.size();){const auto start=at++;while(at<value.size()&&(static_cast<unsigned char>(value[at])&0xc0)==0x80)++at;result.push_back(value.substr(start,at-start));}
  return result;
}
std::size_t character_count(const std::string& text){std::size_t count=0;for(unsigned char c:text)if((c&0xc0)!=0x80)++count;return count;}
std::size_t length(const J& j){if(j.is_array()||j.is_object())return j.size();if(j.is_string())return character_count(j.get_ref<const std::string&>());fail("TypeError","object has no len");}
std::vector<J> iterable(const J& j){std::vector<J> out;if(j.is_array()){for(const auto& v:j)out.push_back(v);}
  else if(j.is_object()){for(auto it=j.begin();it!=j.end();++it)out.push_back(it.key());}
  else if(j.is_string()){for(const auto& c:characters(j.get_ref<const std::string&>()))out.emplace_back(c);}
  else fail("TypeError","object is not iterable");
  return out;
}
std::string string(const J& j,const std::string& reason){if(!j.is_string())fail("TypeError",reason);return j.get<std::string>();}
std::string trim(std::string s){const auto first=s.find_first_not_of(" \t\n\r\f\v"),last=s.find_last_not_of(" \t\n\r\f\v");return first==std::string::npos?"":s.substr(first,last-first+1);}
std::string numeric_string(const std::string& raw){
  // CPython 3.10 Unicode decimal zero code points (each begins ten Nd digits).
  static constexpr std::uint32_t zeros[]={0x30,0x660,0x6f0,0x7c0,0x966,0x9e6,0xa66,0xae6,0xb66,0xbe6,0xc66,0xce6,0xd66,0xde6,0xe50,0xed0,0xf20,0x1040,0x1090,0x17e0,0x1810,0x1946,0x19d0,0x1a80,0x1a90,0x1b50,0x1bb0,0x1c40,0x1c50,0xa620,0xa8d0,0xa900,0xa9d0,0xa9f0,0xaa50,0xabf0,0xff10,0x104a0,0x10d30,0x11066,0x110f0,0x11136,0x111d0,0x112f0,0x11450,0x114d0,0x11650,0x116c0,0x11730,0x118e0,0x11950,0x11c50,0x11d50,0x11da0,0x16a60,0x16b50,0x1d7ce,0x1d7d8,0x1d7e2,0x1d7ec,0x1d7f6,0x1e140,0x1e2f0,0x1e950,0x1fbf0};
  std::string result;
  for(std::size_t i=0;i<raw.size();){const auto c=static_cast<unsigned char>(raw[i++]);std::uint32_t cp=c;unsigned rest=0;
    if(c>=0xf0){cp=c&7;rest=3;}else if(c>=0xe0){cp=c&15;rest=2;}else if(c>=0xc0){cp=c&31;rest=1;}
    while(rest--&&i<raw.size())cp=(cp<<6)|(static_cast<unsigned char>(raw[i++])&63);
    if(cp<128){result+=static_cast<char>(cp);continue;}
    const bool space=cp==0x85||cp==0xa0||cp==0x1680||(cp>=0x2000&&cp<=0x200a)||cp==0x2028||cp==0x2029||cp==0x202f||cp==0x205f||cp==0x3000;
    if(space){result+=' ';continue;}bool digit=false;for(auto zero:zeros)if(cp>=zero&&cp<zero+10){result+=static_cast<char>('0'+cp-zero);digit=true;break;}
    if(!digit)fail("ValueError","numeric conversion");
  }
  return trim(result);
}
std::string remove_digit_underscores(const std::string& s){std::string result;
  for(std::size_t i=0;i<s.size();++i){if(s[i]=='_'){if(i==0||i+1==s.size()||s[i-1]<'0'||s[i-1]>'9'||s[i+1]<'0'||s[i+1]>'9')fail("ValueError","numeric conversion");}else result+=s[i];}
  return result;
}
double as_float(const J& j){
  if(j.is_binary()&&json_is_integer(j)){const auto token=json_integer(j).str();const double value=std::strtod(token.c_str(),nullptr);if(!std::isfinite(value))fail("OverflowError","integer too large to convert to float");return value;}
  if(j.is_number())return j.get<double>();
  if(j.is_boolean())return j.get<bool>()?1.:0.;
  if(!j.is_string())fail("TypeError","float conversion");
  const auto s=remove_digit_underscores(numeric_string(j.get<std::string>()));std::size_t i=0;if(i<s.size()&&(s[i]=='+'||s[i]=='-'))++i;
  auto special=s.substr(i);for(auto& c:special)if(c>='A'&&c<='Z')c=static_cast<char>(c-'A'+'a');
  if(special!="inf"&&special!="infinity"&&special!="nan"){
    std::size_t digits=0;while(i<s.size()&&s[i]>='0'&&s[i]<='9'){++i;++digits;}
    if(i<s.size()&&s[i]=='.'){++i;while(i<s.size()&&s[i]>='0'&&s[i]<='9'){++i;++digits;}}
    if(!digits)fail("ValueError","float conversion");
    if(i<s.size()&&(s[i]=='e'||s[i]=='E')){++i;if(i<s.size()&&(s[i]=='+'||s[i]=='-'))++i;const auto first=i;while(i<s.size()&&s[i]>='0'&&s[i]<='9')++i;if(i==first)fail("ValueError","float conversion");}
    if(i!=s.size())fail("ValueError","float conversion");
  }
  return std::strtod(s.c_str(),nullptr);
}
Integer as_int(const J& j){
  if(j.is_boolean())return j.get<bool>()?1:0;
  if(json_is_integer(j))return json_integer(j);
  if(j.is_number_float()){const double value=j.get<double>();if(std::isnan(value))fail("ValueError","cannot convert NaN to integer");
    if(!std::isfinite(value))fail("OverflowError","cannot convert infinity to integer");
    return Integer(value);}
  if(!j.is_string())fail("TypeError","integer conversion");
  const auto digits=remove_digit_underscores(numeric_string(j.get<std::string>()));
  try{return decimal_integer(digits,python_integer_digit_limit());}catch(const IntegerJsonError& e){fail(e.kind.c_str(),e.what());}
}
std::int64_t as_int64(const J& j){const Integer value=as_int(j);if(value<INT64_MIN||value>INT64_MAX)fail("OverflowError","native timestamp integer range");return value.convert_to<std::int64_t>();}
double contract_number(const J& j,const std::string& f){require(j.is_number()||json_is_integer(j),f);if(j.is_binary()){const auto text=json_integer(j).str();return std::strtod(text.c_str(),nullptr);}return j.get<double>();}
Vec3 vector(const J& j){const auto a=iterable(j);if(a.size()!=3)fail("TypeError","Vec3 needs three values");
  std::array<double,3> values{};for(int i=0;i<3;++i){const auto name=std::string(1,"xyz"[i]);values[i]=contract_number(a[i],name);finite(values[i],name);}return Vec3(values[0],values[1],values[2]);}
Integer pixel_dimension(const J& j,const char* f){require(json_is_integer(j),f);const Integer n=json_integer(j);require(n>0,f);return n;}
PixelScalar pixel_scalar(const J& j,const char* f){require(j.is_number()||json_is_integer(j),f);return json_is_integer(j)?PixelScalar(json_integer(j)):PixelScalar(j.get<double>());}
std::string contract_label(const J& j,const std::string& f){require(j.is_string(),f);const auto value=j.get<std::string>();label(value,f);return value;}
bool truth(const J& j){if(j.is_null())return false;if(j.is_boolean())return j.get<bool>();if(json_is_integer(j))return json_integer(j)!=0;if(j.is_number())return j.get<double>()!=0.;if(j.is_string())return !j.get_ref<const std::string&>().empty();if(j.is_array()||j.is_object())return !j.empty();return false;}
bool python_equal(const J& a,const J& b){
  const auto numeric=[](const J& v){if(v.is_boolean())return PixelScalar(Integer(v.get<bool>()?1:0));if(json_is_integer(v))return PixelScalar(json_integer(v));return PixelScalar(v.get<double>());};
  if((a.is_number()||a.is_boolean()||json_is_integer(a))&&(b.is_number()||b.is_boolean()||json_is_integer(b)))return numeric(a)==numeric(b);
  if(a.type()!=b.type())return false;
  if(a.is_object()){if(a.size()!=b.size())return false;for(auto it=a.begin();it!=a.end();++it){const auto found=b.find(it.key());if(found==b.end()||!python_equal(it.value(),*found))return false;}return true;}
  if(a.is_array()){if(a.size()!=b.size())return false;for(std::size_t i=0;i<a.size();++i)if(!python_equal(a[i],b[i]))return false;return true;}
  return a==b;
}
std::string python_repr(const J& value){
  if(value.is_null())return "None";
  if(value.is_boolean())return value.get<bool>()?"True":"False";
  if(value.is_number_float()){const auto number=value.get<double>();if(std::isnan(number))return "nan";if(std::isinf(number))return number<0?"-inf":"inf";return astribot_s1_robot_geometry::pythonFloat(number);}
  if(json_is_integer(value))return dump_integer_json(value);
  if(value.is_string()){const auto& s=value.get_ref<const std::string&>();const char quote=s.find('\'')!=std::string::npos&&s.find('"')==std::string::npos?'"':'\'';std::string out(1,quote);
    for(std::size_t i=0;i<s.size();++i){if(json_strings::surrogate_at(s,i)){out+="\\u"+json_strings::hex(json_strings::surrogate(s,i));i+=2;continue;}const auto c=static_cast<unsigned char>(s[i]);if(c==quote||c=='\\'){out+='\\';out+=static_cast<char>(c);}else if(c=='\n')out+="\\n";else if(c=='\r')out+="\\r";else if(c=='\t')out+="\\t";else if(c<32||c==127){const char* digits="0123456789abcdef";out+="\\x";out+=digits[c/16];out+=digits[c%16];}else out+=static_cast<char>(c);}return out+quote;}
  const bool object=value.is_object();std::string out=object?"{":"[";bool first=true;for(auto it=value.begin();it!=value.end();++it){if(!first)out+=", ";first=false;if(object)out+=python_repr(J(it.key()))+": ";out+=python_repr(it.value());}return out+(object?"}":"]");
}
std::int64_t add_ns(std::int64_t ns,double timeout){const double value=timeout*1e9;if(!std::isfinite(value)||value>=std::ldexp(1.,63)||value<-std::ldexp(1.,63))fail("OverflowError","timeout range");
  const auto delta=static_cast<std::int64_t>(value);if((delta>0&&ns>INT64_MAX-delta)||(delta<0&&ns<INT64_MIN-delta))fail("OverflowError","stamp range");return ns+delta;}
J parse_python_json(const std::string& raw){try{return parse_integer_json(raw);}catch(const IntegerJsonError& e){fail(e.kind.c_str(),e.what());}}
std::string python_dump(const J& value){
  if(json_is_integer(value)){try{return dump_integer_json(value);}catch(const IntegerJsonError& e){fail(e.kind.c_str(),e.what());}}
  if(value.is_number_float()){const double v=value.get<double>();if(std::isnan(v))return "NaN";if(std::isinf(v))return v<0?"-Infinity":"Infinity";return astribot_s1_robot_geometry::pythonFloat(v);}
  if(value.is_array()){std::string out="[";for(std::size_t i=0;i<value.size();++i){if(i)out+=", ";out+=python_dump(value[i]);}return out+"]";}
  if(value.is_object()){std::string out="{";bool first=true;for(auto it=value.begin();it!=value.end();++it){if(!first)out+=", ";first=false;out+=json_strings::dump(it.key(),true)+": "+python_dump(it.value());}return out+"}";}
  return value.is_string()?json_strings::dump(value.get<std::string>(),true):value.dump(-1,' ',true);
}
struct DecodedField { std::string name;std::size_t offset,size;std::uint8_t type; };
std::size_t type_size(std::uint8_t type){switch(type){case 1:case 2:return 1;case 3:case 4:return 2;case 5:case 6:case 7:return 4;case 8:return 8;default:fail("KeyError","PointField datatype");}}
double read_value(const std::uint8_t* p,std::uint8_t type){switch(type){
#define READ_CASE(code,T) case code:{T value;std::memcpy(&value,p,sizeof value);return static_cast<double>(value);}
READ_CASE(1,std::int8_t) READ_CASE(2,std::uint8_t) READ_CASE(3,std::int16_t) READ_CASE(4,std::uint16_t) READ_CASE(5,std::int32_t) READ_CASE(6,std::uint32_t) READ_CASE(7,float) READ_CASE(8,double)
#undef READ_CASE
  default:fail("KeyError","PointField datatype");}}
std::vector<std::array<double,3>> read_cloud(PointCloudPacket& packet){
  std::vector<DecodedField> fields;std::set<std::string> names;std::map<std::string,std::uint32_t> arrays;std::uint64_t minimum_size=0;
  const auto member=[](const std::string& name,const std::string& base,std::uint32_t count){
    const auto prefix=base+"_";if(name.compare(0,prefix.size(),prefix)!=0)return false;
    const auto suffix=name.substr(prefix.size());if(suffix.empty()||(suffix.size()>1&&suffix[0]=='0')||suffix.size()>10)return false;
    std::uint64_t index=0;for(char c:suffix){if(c<'0'||c>'9')return false;index=index*10+(c-'0');}return index<count;};
  for(std::size_t i=0;i<packet.fields.size();++i){const auto& f=packet.fields[i];const auto size=type_size(f.datatype);if(f.count==0)fail("AssertionError","Can't process fields with count = 0.");
    const std::string name=f.name.empty()?"unnamed_field_"+std::to_string(i):f.name;
    const std::uint64_t end=static_cast<std::uint64_t>(f.offset)+static_cast<std::uint64_t>(size)*f.count;
    minimum_size=std::max(minimum_size,end);
    // Validate numpy's expanded names without allocating count entries. Only
    // scalar x/y/z can be selected; count>1 creates suffixed component names.
    bool duplicate=false;
    if(f.count==1){duplicate=names.count(name);for(const auto& entry:arrays)duplicate=duplicate||member(name,entry.first,entry.second);names.insert(name);fields.push_back({name,f.offset,size,f.datatype});}
    else {duplicate=arrays.count(name);for(const auto& entry:names)duplicate=duplicate||member(entry,name,f.count);arrays.emplace(name,f.count);}
    if(duplicate)fail("AssertionError","Duplicate field names are not allowed!");
  }
  if(minimum_size>packet.point_step)fail("ValueError","point_step smaller than field extent");
  const std::uint64_t count=static_cast<std::uint64_t>(packet.width)*packet.height;
  if(count&&packet.point_step>packet.data.size()/count)fail("TypeError","buffer is too small for requested array");
  std::array<DecodedField,3> selected;for(int k=0;k<3;++k){const auto it=std::find_if(fields.begin(),fields.end(),[&](const auto& f){return f.name==std::string(1,"xyz"[k]);});
    if(it==fields.end())fail("AssertionError","Requests field is not in the fields of the PointCloud!");
    selected[k]=*it;}
  const std::uint16_t endian=1;const bool host_big=*reinterpret_cast<const std::uint8_t*>(&endian)==0;
  if(host_big!=packet.is_bigendian)for(std::uint64_t row=0;row<count;++row)for(const auto& f:selected){auto begin=packet.data.begin()+row*packet.point_step+f.offset;std::reverse(begin,begin+f.size);}
  std::vector<std::array<double,3>> points;points.reserve(count);
  for(std::uint64_t row=0;row<count;++row){std::array<double,3> point{};bool nan=false;for(int k=0;k<3;++k){const auto& f=selected[k];point[k]=read_value(packet.data.data()+row*packet.point_step+f.offset,f.type);nan=nan||std::isnan(point[k]);}
    if(packet.is_dense||!nan)points.push_back(point);}
  return points;
}
}
AdapterError::AdapterError(std::string type,std::string message):std::invalid_argument(std::move(message)),kind(std::move(type)){}
Integer adapter_integer(const AdapterJson& value){return as_int(value);}
std::int64_t adapter_int64(const AdapterJson& value){return as_int64(value);}
double adapter_float(const AdapterJson& value){return as_float(value);}
Vec3 rotate_observation_vector(const Vec3& v,const AdapterQuaternion& q){
  const double tx=2*(q.y*v.z-q.z*v.y),ty=2*(q.z*v.x-q.x*v.z),tz=2*(q.x*v.y-q.y*v.x);
  return Vec3(v.x+q.w*tx+q.y*tz-q.z*ty,v.y+q.w*ty+q.z*tx-q.x*tz,v.z+q.w*tz+q.x*ty-q.y*tx);
}
VisionAdapter::VisionAdapter(AdapterProfile profile,LookupTransform tf,AdapterNow now,AdapterSteadyNow steady,AdapterOptions options)
  :profile_(std::move(profile)),tf_(std::move(tf)),now_(std::move(now)),steady_now_(std::move(steady)),options_(std::move(options)){
  if(!tf_||!now_)fail("ValueError","transform and capture clock callbacks required");
  if(!steady_now_)steady_now_=[](){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();};
}
AdapterMessageKind VisionAdapter::message_kind() const noexcept{return AdapterMessageKind::VISION_STRING;}
void VisionAdapter::set_raw_options(J options){raw_options_=truth(options)?std::move(options):J::object();}
void VisionAdapter::set_calibration_epoch(Integer epoch){
  if(raw_options_){if(!raw_options_->is_object())fail("TypeError","options item assignment");(*raw_options_)["calibration_epoch"]=epoch;}
  options_.calibration_epoch=epoch;
}
J VisionAdapter::option(const std::string& name,J fallback) const{
  if(raw_options_)return get(*raw_options_,name,std::move(fallback));
  if(name=="max_packet_bytes")return options_.max_packet_bytes;
  if(name=="max_observations")return options_.max_observations;
  if(name=="max_points")return options_.max_points;
  if(name=="calibration_epoch")return options_.calibration_epoch;
  if(name=="position_variance_m2")return options_.position_variance_m2;
  if(name=="sensor_id"&&options_.sensor_id)return *options_.sensor_id;
  return fallback;
}
void VisionAdapter::check_packet_budget(std::size_t count) const{const auto limit=as_int(option("max_packet_bytes",1048576));if(limit<0||count>limit)fail("ValueError","packet over byte budget");}
std::vector<Observation> VisionAdapter::normalize(PointCloudPacket&){fail("TypeError","vision adapter requires a String packet");}
std::vector<Observation> VisionAdapter::normalize(const VisionPacket& packet){
  check_packet_budget(character_count(packet.data));return normalize_parsed(parse_python_json(packet.data));
}
std::vector<Observation> VisionAdapter::normalize_parsed(J data){
  const auto count=length(get(data,"observations",J::array()));const auto limit=as_int(option("max_observations",128));if(limit<0||count>limit)fail("ValueError","observation budget exceeded");
  const Stamp first=now_();
  const auto expected=option("sensor_id",nullptr);if(truth(expected)&&!python_equal(get(data,"sensor_id"),expected))fail("ValueError","sensor identity mismatch");
  last_packet_=data;
  const auto schema=get(data,"schema_version");if(!((schema.is_number()&&schema.get<double>()==1.)||(schema.is_boolean()&&schema.get<bool>())))fail("ValueError","vision schema_version");
  (void)now_();const Stamp capture(as_int64(field(data,"stamp_ns")),first.clock,first.epoch);
  std::vector<Observation> result;
  for(const auto& item:iterable(field(data,"observations"))){J frame=field(data,"frame_id");const auto kind=field(item,"kind");
    std::optional<Geometry> geometry;
    if(kind=="metric_box"){
      const auto transform=tf_(profile_.tracking_frame,string(frame,"frame_id"),capture.ns);
      const auto rotated=rotate_observation_vector(vector(field(item,"center_m")),transform.rotation);
      const std::array<double,3> center{rotated.x+transform.translation[0],rotated.y+transform.translation[1],rotated.z+transform.translation[2]};
      const auto size=vector(field(item,"size_m"));if(std::min({size.x,size.y,size.z})<=0)fail("ValueError","positive metric dimensions required");
      const auto a=rotate_observation_vector(Vec3(size.x,0,0),transform.rotation),b=rotate_observation_vector(Vec3(0,size.y,0),transform.rotation),c=rotate_observation_vector(Vec3(0,0,size.z),transform.rotation);
      const Vec3 extent(std::abs(a.x)+std::abs(b.x)+std::abs(c.x),std::abs(a.y)+std::abs(b.y)+std::abs(c.y),std::abs(a.z)+std::abs(b.z)+std::abs(c.z));
      const double variance=as_float(field(item,"position_variance_m2"));std::optional<Vec3> velocity;std::optional<Covariance3> velocity_covariance;
      if(item.contains("velocity_m_s")||item.contains("velocity_variance_m2_s2")){velocity.emplace(rotate_observation_vector(vector(field(item,"velocity_m_s")),transform.rotation));const double v=as_float(field(item,"velocity_variance_m2_s2"));velocity_covariance.emplace(std::array<double,9>{v,0.,0.,0.,v,0.,0.,0.,v});}
      const Vec3 center_value(center[0],center[1],center[2]);const Covariance3 covariance({variance,0.,0.,0.,variance,0.,0.,0.,variance});
      geometry.emplace(MetricBox(center_value,extent,covariance,velocity,velocity_covariance));frame=profile_.tracking_frame;
    }else if(kind=="image_box"){
      const auto& camera=field(data,"sensor_id");auto args=iterable(field(item,"image_size_px"));const auto bounds=iterable(field(item,"box_xyxy_px"));args.insert(args.end(),bounds.begin(),bounds.end());
      if(args.size()!=6)fail("TypeError","ImageBox argument count");
      const auto camera_id=contract_label(camera,"camera_id");const auto width=pixel_dimension(args[0],"image.width"),height=pixel_dimension(args[1],"image.height");
      std::vector<PixelScalar> pixels;pixels.reserve(4);const std::array<const char*,4> fields{"xmin_px","ymin_px","xmax_px","ymax_px"};
      for(int i=0;i<4;++i){pixels.push_back(pixel_scalar(args[i+2],fields[i]));finite(pixels.back(),fields[i],0.);}
      geometry.emplace(ImageBox(camera_id,width,height,pixels[0],pixels[1],pixels[2],pixels[3]));
    }else if(kind=="bearing_cone"){
      const auto direction=vector(field(item,"direction"));const auto& angle=field(item,"half_angle_rad");
      require(std::abs(euclidean_norm(direction.x,direction.y,direction.z)-1.)<=1e-6,"bearing.unit_direction");
      geometry.emplace(BearingCone(direction,contract_number(angle,"bearing.half_angle")));
    }else fail("ValueError","unsupported vision geometry");
    const auto& sensor=field(data,"sensor_id");const auto& measurement=field(item,"measurement_id");const auto track=get(item,"track_id");
    const Stamp received(steady_now_(),"steady",0);const Stamp until(add_ns(capture.ns,profile_.sensor_timeout_s),first.clock,first.epoch);
    const auto calibration=as_int(field(data,"calibration_epoch"));const double quality=as_float(field(item,"geometry_quality"));
    const auto classes=get(item,"classes",J::object());if(!classes.is_object())fail("AttributeError","classes has no items method");
    std::vector<std::pair<std::string,double>> probabilities;for(auto it=classes.begin();it!=classes.end();++it)probabilities.emplace_back(it.key(),as_float(it.value()));
    const auto origins=iterable(field(item,"provenance"));
    // Dataclass validation begins after all argument expressions have completed.
    const auto sensor_id=contract_label(sensor,"sensor_id"),measurement_id=contract_label(measurement,"measurement_id"),frame_id=contract_label(frame,"frame_id");
    std::optional<std::string> source;if(!track.is_null())source=contract_label(track,"source_track_id");
    std::vector<std::string> provenance;bool untyped=false;for(const auto& p:origins){untyped=untyped||!p.is_string();provenance.push_back(p.is_string()?p.get<std::string>():"native-placeholder");}
    if(untyped){
      // Validate the prefix before diagnosing a value which cannot enter the
      // typed provenance vector. Duplicate detection belongs after every label.
      (void)Observation(sensor_id,measurement_id,source,capture,received,until,frame_id,calibration,*geometry,quality,probabilities,{"native-placeholder"},truth(track),false);
      for(const auto& p:origins)(void)contract_label(p,"provenance.item");
    }
    result.emplace_back(sensor_id,measurement_id,source,capture,received,until,frame_id,calibration,*geometry,quality,probabilities,provenance,truth(track),false);
  }
  return result;
}
PacketMetadata VisionAdapter::last_metadata() const{
  if(!last_packet_)fail("ValueError","no parsed packet");
  const auto& d=*last_packet_;
  return PacketMetadata{as_int64(field(d,"stamp_ns")),as_int(field(d,"calibration_epoch")),string(field(d,"sensor_id"),"sensor_id"),get(d,"frame_id").is_string()?get(d,"frame_id").get<std::string>():std::string{},get(d,"resolved_measurement_ids",J::array())};
}
AdapterMessageKind PointCloudBoxAdapter::message_kind() const noexcept{return AdapterMessageKind::POINT_CLOUD2;}
std::vector<Observation> PointCloudBoxAdapter::normalize(PointCloudPacket& packet){
  const auto limit=as_int(option("max_points",32768));
  const std::uint64_t count=static_cast<std::uint64_t>(packet.width)*packet.height;
  if(count==0||limit<=0||count>limit)fail("ValueError","cloud empty or over point budget");
  const auto points=read_cloud(packet);if(points.empty())fail("ValueError","no finite depth");
  auto lo=points.front(),hi=points.front();for(const auto& p:points)for(int i=0;i<3;++i){if(p[i]<lo[i])lo[i]=p[i];if(p[i]>hi[i])hi[i]=p[i];}
  const std::int64_t ns=static_cast<std::int64_t>(packet.sec)*1000000000LL+packet.nanosec;
  J sensor;if(raw_options_)sensor=field(*raw_options_,"sensor_id");else {if(!options_.sensor_id)fail("KeyError","sensor_id");sensor=*options_.sensor_id;}
  const auto calibration=as_int(option("calibration_epoch",0));
  std::array<double,3> center{},size{};for(int i=0;i<3;++i){center[i]=(lo[i]+hi[i])/2;size[i]=std::max(.02,hi[i]-lo[i]);}
  const double variance=as_float(option("position_variance_m2",.0025));
  const auto sensor_text=sensor.is_string()?sensor.get<std::string>():python_repr(sensor);
  J data{{"schema_version",1},{"sensor_id",sensor},{"stamp_ns",ns},{"frame_id",packet.frame_id},{"calibration_epoch",calibration},
    {"observations",J::array({J{{"kind","metric_box"},{"measurement_id",std::to_string(ns)},{"center_m",center},{"size_m",size},
      {"position_variance_m2",variance},{"geometry_quality",1.},{"provenance",J::array({sensor_text+":"+std::to_string(ns)})}}})}};
  check_packet_budget(python_dump(data).size());return normalize_parsed(std::move(data));
}
AdapterMessageKind adapter_message_kind(const std::string& name){if(name=="vision_json")return AdapterMessageKind::VISION_STRING;if(name=="pointcloud_boxes")return AdapterMessageKind::POINT_CLOUD2;fail("ValueError","unsupported native observation adapter: "+name);}
std::unique_ptr<ObservationAdapter> make_observation_adapter(const std::string& name,AdapterProfile profile,LookupTransform tf,AdapterNow now,AdapterSteadyNow steady,AdapterOptions options){
  const auto kind=adapter_message_kind(name);if(kind==AdapterMessageKind::VISION_STRING)return std::make_unique<VisionAdapter>(std::move(profile),std::move(tf),std::move(now),std::move(steady),std::move(options));
  return std::make_unique<PointCloudBoxAdapter>(std::move(profile),std::move(tf),std::move(now),std::move(steady),std::move(options));
}
void NativeAdapterRegistry::register_adapter(std::string name,NativeAdapterRegistration registration){label(name,"adapter.name");if(name=="vision_json"||name=="pointcloud_boxes"||extensions_.count(name)||!registration.factory)fail("ValueError","duplicate or invalid native adapter registration");extensions_.emplace(std::move(name),std::move(registration));}
AdapterMessageKind NativeAdapterRegistry::message_kind(const std::string& name) const{const auto it=extensions_.find(name);return it==extensions_.end()?adapter_message_kind(name):it->second.message_kind;}
std::unique_ptr<ObservationAdapter> NativeAdapterRegistry::make(const std::string& name,AdapterProfile profile,LookupTransform tf,AdapterNow now,AdapterSteadyNow steady,AdapterOptions options) const{
  const auto it=extensions_.find(name);if(it==extensions_.end())return make_observation_adapter(name,std::move(profile),std::move(tf),std::move(now),std::move(steady),std::move(options));
  auto adapter=it->second.factory(std::move(profile),std::move(tf),std::move(now),std::move(steady),std::move(options));if(!adapter||adapter->message_kind()!=it->second.message_kind)fail("ValueError","native adapter factory message kind mismatch");return adapter;
}
}  // namespace astribot::navigation::policy
