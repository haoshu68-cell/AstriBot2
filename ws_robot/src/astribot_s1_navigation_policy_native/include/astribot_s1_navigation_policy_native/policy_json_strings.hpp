#pragma once
#include <nlohmann/json.hpp>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace astribot::navigation::policy {
namespace json_strings {
inline bool hex4(const std::string& text,std::size_t at,std::uint32_t& result) {
  if(at+4>text.size())return false;
  result=0;
  for(std::size_t i=at;i<at+4;++i){const char c=text[i];unsigned digit;
    if(c>='0'&&c<='9')digit=c-'0';else if(c>='a'&&c<='f')digit=c-'a'+10;
    else if(c>='A'&&c<='F')digit=c-'A'+10;else return false;
    result=(result<<4)|digit;
  }
  return true;
}
inline std::string hex(std::uint32_t cp) {
  constexpr char digits[]="0123456789abcdef";std::string out(4,'0');
  for(int i=3;i>=0;--i){out[i]=digits[cp&15];cp>>=4;}return out;
}
struct Prepared {std::string text;bool changed=false;};
inline Prepared prepare(const std::string& raw) {
  // NUL is escaped injectively: original NUL -> NUL NUL, lone surrogate ->
  // NUL S hhhh. Genuine marker-looking input therefore cannot become a tag.
  if(raw.find("\\u")==std::string::npos)return {raw,false};
  Prepared out;out.text.reserve(raw.size());
  for(std::size_t i=0;i<raw.size();) {
    if(raw[i]!='"'){out.text+=raw[i++];continue;}
    out.text+=raw[i++];
    while(i<raw.size()) {
      const char c=raw[i++];
      if(c=='"'){out.text+=c;break;}
      if(c!='\\'||i==raw.size()){out.text+=c;continue;}
      std::uint32_t cp=0,low=0;
      if(raw[i]=='u'&&hex4(raw,i+1,cp)) {
        if(cp>=0xd800&&cp<=0xdbff&&i+6<raw.size()&&raw[i+5]=='\\'&&raw[i+6]=='u'&&hex4(raw,i+7,low)&&low>=0xdc00&&low<=0xdfff) {
          out.text+=raw.substr(i-1,12);i+=11;continue;
        }
        if(cp==0){out.text+="\\u0000\\u0000";out.changed=true;}
        else if(cp>=0xd800&&cp<=0xdfff){out.text+="\\u0000S"+hex(cp);out.changed=true;}
        else out.text+=raw.substr(i-1,6);
        i+=5;continue;
      }
      out.text+='\\';out.text+=raw[i++];
    }
  }
  return out;
}
inline std::string restore(const std::string& encoded) {
  std::string out;out.reserve(encoded.size());
  for(std::size_t i=0;i<encoded.size();) {
    if(encoded[i]!='\0'){out+=encoded[i++];continue;}
    if(i+1<encoded.size()&&encoded[i+1]=='\0'){out+='\0';i+=2;continue;}
    std::uint32_t cp=0;
    if(i+1>=encoded.size()||encoded[i+1]!='S'||!hex4(encoded,i+2,cp)||cp<0xd800||cp>0xdfff)
      throw std::logic_error("invalid internal JSON string escape");
    out+=static_cast<char>(0xe0|(cp>>12));out+=static_cast<char>(0x80|((cp>>6)&63));out+=static_cast<char>(0x80|(cp&63));i+=6;
  }
  return out;
}
template<class Json> Json restore(Json value) {
  if(value.is_string())return Json(restore(value.template get<std::string>()));
  if(value.is_array()){for(auto& child:value)child=restore(std::move(child));return value;}
  if(value.is_object()){Json result=Json::object();for(auto it=value.begin();it!=value.end();++it)result[restore(it.key())]=restore(std::move(it.value()));return result;}
  return value;
}
inline bool surrogate_at(const std::string& text,std::size_t i) {
  return i+2<text.size()&&static_cast<unsigned char>(text[i])==0xed&&
    static_cast<unsigned char>(text[i+1])>=0xa0&&static_cast<unsigned char>(text[i+1])<=0xbf&&
    (static_cast<unsigned char>(text[i+2])&0xc0)==0x80;
}
inline std::uint32_t surrogate(const std::string& text,std::size_t i) {
  return 0xd000|((static_cast<unsigned char>(text[i+1])&63)<<6)|(static_cast<unsigned char>(text[i+2])&63);
}
inline std::string dump(const std::string& text,bool ensure_ascii) {
  std::string out="\"";std::size_t start=0;
  for(std::size_t i=0;i<text.size();++i)if(surrogate_at(text,i)) {
    const auto segment=nlohmann::ordered_json(text.substr(start,i-start)).dump(-1,' ',ensure_ascii);
    out+=segment.substr(1,segment.size()-2);out+="\\u"+hex(surrogate(text,i));i+=2;start=i+1;
  }
  if(start==0)return nlohmann::ordered_json(text).dump(-1,' ',ensure_ascii);
  const auto segment=nlohmann::ordered_json(text.substr(start)).dump(-1,' ',ensure_ascii);
  out+=segment.substr(1,segment.size()-2);return out+'"';
}
inline bool valid_utf8(const std::string& text) {
  for(std::size_t i=0;i<text.size();) {
    const auto lead=static_cast<unsigned char>(text[i++]);if(lead<128)continue;
    unsigned count;std::uint32_t cp,minimum;
    if(lead>=0xc2&&lead<=0xdf){count=1;cp=lead&31;minimum=0x80;}
    else if(lead>=0xe0&&lead<=0xef){count=2;cp=lead&15;minimum=0x800;}
    else if(lead>=0xf0&&lead<=0xf4){count=3;cp=lead&7;minimum=0x10000;}
    else return false;
    if(i+count>text.size())return false;
    while(count--){const auto c=static_cast<unsigned char>(text[i++]);if((c&0xc0)!=0x80)return false;cp=(cp<<6)|(c&63);}
    if(cp<minimum||cp>0x10ffff||(cp>=0xd800&&cp<=0xdfff))return false;
  }
  return true;
}
inline void require_utf8(const std::string& text) {
  if(!valid_utf8(text))throw std::invalid_argument("UnicodeEncodeError: string cannot be encoded as UTF-8");
}
inline std::string health_wire_string(const std::string& text) {
  // Humble's Python message converter first encodes the complete string, then
  // calls rosidl_runtime_c__String__assign (strlen). Keep that exact order.
  require_utf8(text);return text.substr(0,text.find('\0'));
}
inline void require_tf_string(const std::string& text) {
  require_utf8(text);if(text.find('\0')!=std::string::npos)throw std::invalid_argument("ValueError: embedded null character");
}
}  // namespace json_strings
}  // namespace astribot::navigation::policy
