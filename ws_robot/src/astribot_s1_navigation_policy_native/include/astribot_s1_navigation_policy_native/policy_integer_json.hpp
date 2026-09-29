#pragma once
// Lossless Python-compatible integer tokens at the policy JSON boundary. Binary
// is an internal scalar category: a user JSON string/object cannot construct it.
#include "astribot_s1_navigation_policy_native/policy_integer.hpp"
#include "astribot_s1_navigation_policy_native/policy_json_strings.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace astribot::navigation::policy {
using IntegerJson=nlohmann::ordered_json;
struct IntegerJsonError : std::invalid_argument {
  const std::string kind;
  IntegerJsonError(std::string kind,std::string message)
    :std::invalid_argument(std::move(message)),kind(std::move(kind)){}
};
inline constexpr std::uint64_t integer_json_subtype=0x435050494e54ULL;
inline constexpr std::size_t python_decimal_digit_limit=4300;
inline std::size_t python_integer_digit_limit() {
  // CPython reads this once during startup. Main can call this before ROS init
  // so an invalid environment fails before a node or executor is constructed.
  static const std::size_t configured=[](){
    const char* text=std::getenv("PYTHONINTMAXSTRDIGITS");if(!text||!*text)return python_decimal_digit_limit;
    errno=0;char* end=nullptr;const long long value=std::strtoll(text,&end,10);
    if(end==text||*end!='\0'||errno==ERANGE||value<0||value>INT_MAX||(value!=0&&value<640))
      throw IntegerJsonError("ValueError","PYTHONINTMAXSTRDIGITS must be 0 or an integer between 640 and INT_MAX");
    return static_cast<std::size_t>(value);
  }();
  return configured;
}
inline Integer decimal_integer(const std::string& text,std::size_t max_digits=0) {
  std::size_t at=0;bool negative=false;
  if(!text.empty()&&(text[0]=='-'||text[0]=='+')){negative=text[0]=='-';++at;}
  if(at==text.size())throw IntegerJsonError("ValueError","invalid decimal integer");
  if(max_digits&&text.size()-at>max_digits)throw IntegerJsonError("ValueError","integer decimal digit limit");
  Integer value=0;for(;at<text.size();++at){const char digit=text[at];if(digit<'0'||digit>'9')throw IntegerJsonError("ValueError","invalid decimal integer");value*=10;value+=digit-'0';}
  return negative?-value:value;
}
template<class Json> bool json_is_integer(const Json& value) {
  return value.is_number_integer()||(value.is_binary()&&value.get_binary().has_subtype()&&value.get_binary().subtype()==integer_json_subtype);
}
template<class Json> Integer json_integer(const Json& value) {
  if(value.is_number_unsigned())return Integer(value.template get<std::uint64_t>());
  if(value.is_number_integer())return Integer(value.template get<std::int64_t>());
  if(value.is_binary()&&json_is_integer(value)){const auto& bytes=value.get_binary();return decimal_integer(std::string(bytes.begin(),bytes.end()));}
  throw IntegerJsonError("TypeError","exact integer required");
}
template<class Json> void assign_integer_json(Json& result,const Integer& value) {
  if(value>=std::numeric_limits<std::int64_t>::min()&&value<=std::numeric_limits<std::int64_t>::max())result=value.template convert_to<std::int64_t>();
  else if(value>=0&&value<=std::numeric_limits<std::uint64_t>::max())result=value.template convert_to<std::uint64_t>();
  else {const auto decimal=value.str();result=Json::binary(std::vector<std::uint8_t>(decimal.begin(),decimal.end()),integer_json_subtype);}
}
inline IntegerJson integer_json(const Integer& value){IntegerJson result;assign_integer_json(result,value);return result;}
inline IntegerJson parse_integer_json(const std::string& input,std::size_t max_digits=python_integer_digit_limit()) {
  using J=IntegerJson;
  const auto prepared=json_strings::prepare(input);const auto& raw=prepared.text;
  if(raw.compare(0,3,"\xef\xbb\xbf")==0)throw IntegerJsonError("JSONDecodeError","unexpected UTF-8 BOM");
  // Decode original string literals only to reserve an injective marker key.
  // Errors are deferred to the real parser, preserving lexical error ordering.
  std::set<std::string> strings;std::vector<std::pair<std::size_t,std::size_t>> quoted;
  for(std::size_t i=0;i<raw.size();++i)if(raw[i]=='"'){
    const auto first=i++;for(;i<raw.size();++i){if(raw[i]=='\\'){++i;continue;}if(raw[i]=='"')break;}
    const auto end=std::min(i+1,raw.size());quoted.emplace_back(first,end);
    try{strings.insert(J::parse(raw.substr(first,end-first)).template get<std::string>());}catch(const J::exception&){}
  }
  std::string marker="__integer_json__";while(strings.count(marker))marker+='_';
  struct Replacement {bool integer;std::string token;double floating;};
  std::vector<Replacement> replacements;std::string rewritten;std::size_t quoted_index=0;
  const auto substitute=[&](bool integer,const std::string& token,double floating){const auto index=replacements.size();replacements.push_back({integer,token,floating});return "{"+J(marker).dump()+":"+std::to_string(index)+"}";};
  for(std::size_t i=0;i<raw.size();){
    if(quoted_index<quoted.size()&&i==quoted[quoted_index].first){const auto end=quoted[quoted_index++].second;rewritten+=raw.substr(i,end-i);i=end;continue;}
    bool replaced=false;const std::array<std::string,3> specials{"NaN","Infinity","-Infinity"};
    for(std::size_t k=0;k<specials.size();++k)if(raw.compare(i,specials[k].size(),specials[k])==0){
      const double value=k==0?std::numeric_limits<double>::quiet_NaN():(k==1?std::numeric_limits<double>::infinity():-std::numeric_limits<double>::infinity());
      rewritten+=substitute(false,{},value);i+=specials[k].size();replaced=true;break;
    }
    if(!replaced&&(raw[i]=='-'||(raw[i]>='0'&&raw[i]<='9'))){
      std::size_t end=i;if(raw[end]=='-')++end;const auto first_digit=end;bool valid=end<raw.size(),floating=false;
      if(valid&&raw[end]=='0')++end;
      else {while(end<raw.size()&&raw[end]>='0'&&raw[end]<='9')++end;valid=valid&&end>first_digit;}
      if(end<raw.size()&&raw[end]=='.'){floating=true;const auto first=++end;while(end<raw.size()&&raw[end]>='0'&&raw[end]<='9')++end;valid=valid&&end>first;}
      if(end<raw.size()&&(raw[end]=='e'||raw[end]=='E')){floating=true;++end;if(end<raw.size()&&(raw[end]=='+'||raw[end]=='-'))++end;const auto first=end;while(end<raw.size()&&raw[end]>='0'&&raw[end]<='9')++end;valid=valid&&end>first;}
      if(valid){const auto token=raw.substr(i,end-i);
        if(!floating&&end-first_digit>=19){rewritten+=substitute(true,token,0.);i=end;replaced=true;}
        else if(floating){const double value=std::strtod(token.c_str(),nullptr);if(std::isinf(value)){rewritten+=substitute(false,{},value);i=end;replaced=true;}}
        if(!replaced){rewritten+=token;i=end;replaced=true;}
      }
    }
    if(!replaced)rewritten+=raw[i++];
  }
  try {
    auto result=J::parse(rewritten,[&](int,J::parse_event_t event,J& value){
      if(event==J::parse_event_t::object_end&&value.is_object()&&value.size()==1&&value.contains(marker)){
        const auto index=value.at(marker).template get<std::size_t>();const auto& item=replacements.at(index);
        value=item.integer?integer_json(decimal_integer(item.token,max_digits)):J(item.floating);
      }
      return true;
    });
    return prepared.changed?json_strings::restore(std::move(result)):std::move(result);
  }catch(const J::exception&){throw IntegerJsonError("JSONDecodeError","invalid JSON packet");}
}
template<class Json> std::string dump_integer_json(const Json& value,int indent=-1,bool ensure_ascii=false) {
  std::function<std::string(const Json&,int)> dump=[&](const Json& v,int level)->std::string{
    if(json_is_integer(v)){const auto result=json_integer(v).str();const auto digits=result.size()-(!result.empty()&&result[0]=='-');const auto limit=python_integer_digit_limit();if(limit&&digits>limit)throw IntegerJsonError("ValueError","integer decimal digit limit");return result;}
    if(v.is_number_float()){const double number=v.template get<double>();if(std::isnan(number))return "NaN";if(std::isinf(number))return number<0?"-Infinity":"Infinity";}
    if(v.is_string())return json_strings::dump(v.template get<std::string>(),ensure_ascii);
    if(!v.is_array()&&!v.is_object())return v.dump(-1,' ',ensure_ascii);
    const bool object=v.is_object();std::string out=object?"{":"[";bool first=true;
    for(auto it=v.begin();it!=v.end();++it){if(!first)out+=",";first=false;if(indent>=0)out+="\n"+std::string(static_cast<std::size_t>((level+1)*indent),' ');
      if(object){out+=json_strings::dump(it.key(),ensure_ascii)+":";if(indent>=0)out+=' ';}out+=dump(it.value(),level+1);}
    if(!first&&indent>=0)out+="\n"+std::string(static_cast<std::size_t>(level*indent),' ');
    return out+(object?"}":"]");
  };
  return dump(value,0);
}
}  // namespace astribot::navigation::policy
namespace nlohmann {
template<> struct adl_serializer<astribot::navigation::policy::Integer> {
  template<class Json> static void to_json(Json& json,const astribot::navigation::policy::Integer& value){astribot::navigation::policy::assign_integer_json(json,value);}
  template<class Json> static void from_json(const Json& json,astribot::navigation::policy::Integer& value){value=astribot::navigation::policy::json_integer(json);}
};
}  // namespace nlohmann
