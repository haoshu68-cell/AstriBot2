#include "astribot_operator_station/evidence.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/logger.h>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <iomanip>
#include <unistd.h>

namespace station = astribot_operator_station;
using Json = station::Json;
using Trigger = std_srvs::srv::Trigger;
using Get = rcl_interfaces::srv::GetParameters;
using Clock = std::chrono::steady_clock;
namespace {
std::string gid_text(const uint8_t * data, size_t size) {
  std::ostringstream out;
  for(size_t i=0;i<size;++i)out<<std::hex<<std::setfill('0')<<std::setw(2)<<static_cast<unsigned>(data[i]);
  return out.str();
}
Json finite_value(double x) {
  if (std::isfinite(x)) return x;
  return Json{{"non_finite", std::isnan(x) ? "nan" : (x > 0 ? "+inf" : "-inf")}};
}
Json value(const rcl_interfaces::msg::ParameterValue & p) {
  Json v;
  switch (p.type) {
    case 1: v=p.bool_value; break;
    case 2: v=p.integer_value; break;
    case 3: v=finite_value(p.double_value); break;
    case 4: v=p.string_value; break;
    case 5: v=p.byte_array_value; break;
    case 6: v=p.bool_array_value; break;
    case 7: v=p.integer_array_value; break;
    case 8: v=Json::array();for(auto x:p.double_array_value)v.push_back(finite_value(x)); break;
    case 9: v=p.string_array_value; break;
    default: v=nullptr;
  }
  return station::parameter_value(p.type,v);
}
}
class Recorder : public rclcpp::Node {
  struct Target {
    std::vector<std::string> names;
    rclcpp::Client<Get>::SharedPtr client;
    std::string instance;
    std::string identity_quality{"unavailable"};
    Json observed=Json::object();
    uint64_t request_token{0}, last_known_sequence{0};
    int64_t read_begin_ns{0};
    uint64_t event_revision{0}; bool pending{false}; int64_t request_id{0}; Clock::time_point deadline;
  };
  struct TopicQuality {
    uint64_t messages{0}, bytes{0};
    Clock::time_point last{};
    double maximum_gap_s{0.0};
    size_t publishers{0};
  };
  std::map<std::string,TopicQuality> topic_quality_;
  double stale_sec_{3.0};
  std::map<std::string,Target> targets_;
  std::map<std::string,rclcpp::GenericSubscription::SharedPtr> subs_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr events_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::shared_ptr<spdlog::logger> log_;
  std::ofstream events_;
  std::filesystem::path directory_;
  std::vector<std::string> topics_;
  uint64_t sequence_{0}, generation_{0}, messages_{0};
  bool active_{false}; std::string fault_;
  Clock::time_point next_snapshot_;
  void event(Json e) {
    if (!active_) return;
    e["schema_version"]=2;
    e["sequence"]=++sequence_;
    e["ros_ns"]=now().nanoseconds();
    e["steady_ns"]=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
    e["utc_ns"]=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    events_ << e.dump() << '\n'; events_.flush();
    if (!events_) throw std::runtime_error("Event write failed");
  }
  void manifest(const std::string & state) {
    Json j={{"schema_version",2},{"state",state},{"session_id",directory_.filename().string()},
      {"robot_id",get_parameter("robot_id").as_string()},
      {"deployment_revision",get_parameter("deployment_revision").as_string()},
      {"parameter_semantics","observed_not_effective"},{"message_count",messages_},
      {"parameter_quality",parameter_quality()},{"topic_quality",quality_report()},{"event_count",sequence_},{"fault",fault_},{"storage","sqlite3"},
      {"limitations",{"No device effective confirmation","DDS endpoint identity is not a firmware/process boot UUID","No packet-loss proof","No pretrigger cache"}}};
    std::ofstream file(directory_/"manifest.tmp"); file << j.dump(2); file.close();
    if (!file) throw std::runtime_error("Manifest write failed");
    std::filesystem::rename(directory_/"manifest.tmp",directory_/"manifest.json");
  }
  void close(const std::string & state) {
    ++generation_;
    for (auto & item : targets_) {
      auto & t=item.second;
      if(t.pending){gap(item.first,"recording_stopped_during_snapshot");t.client->remove_pending_request(t.request_id);}
      ++t.request_token;t.pending=false;
    }
    subs_.clear();
    writer_.reset();
    if (active_) {event({{"kind","session_end"},{"state",state}});active_=false;events_.close();manifest(state);}
    log_.reset();
  }
  void fail(const std::string & reason) {
    fault_=reason; RCLCPP_ERROR(get_logger(),"Recording failed: %s",reason.c_str());
    try {close("partial");} catch (...) {active_=false; writer_.reset(); events_.close();}
  }
  void gap(const std::string & node,const std::string & reason) {
    auto & t=targets_.at(node);
    event({{"kind","parameter_gap"},{"node",node},{"node_instance_id",t.instance},
      {"reason",reason},{"uncertain_after_sequence",t.last_known_sequence}});
    t.observed=Json::object();
  }
  bool identity(const std::string & node) {
    auto & t=targets_.at(node);
    std::set<std::string> gids;
    for(const auto & info:get_publishers_info_by_topic("/parameter_events")) {
      auto fqn=info.node_namespace();
      if(fqn.empty() || fqn.back()!='/')fqn+='/';
      fqn+=info.node_name();
      if(fqn==node) {const auto & id=info.endpoint_gid();gids.insert(gid_text(id.data(),id.size()));}
    }
    auto quality=gids.empty()?"unavailable":gids.size()==1?"endpoint_observed":"ambiguous";
    auto id=gids.size()==1?*gids.begin():"";
    if(t.instance!=id || t.identity_quality!=quality) {
      if(t.pending)t.client->remove_pending_request(t.request_id);
      t.pending=false;++t.request_token;
      auto previous=t.instance;t.instance=id;t.identity_quality=quality;
      gap(node,"endpoint_identity_changed");
      event({{"kind","node_instance"},{"node",node},{"node_instance_id",id},
        {"previous_instance_id",previous},{"identity_quality",quality},
        {"identity_source","parameter_event_publisher_gid"}});
    }
    return gids.size()==1;
  }
  Json parameter_quality() const {
    Json result=Json::object();
    for(const auto & [node,t]:targets_) {
      size_t available=0;
      for(auto it=t.observed.begin();it!=t.observed.end();++it)if(it.value().value("available",false))++available;
      result[node]={{"node_instance_id",t.instance},{"identity_quality",t.identity_quality},
        {"pending",t.pending},{"requested",t.names.size()},{"available",available},
        {"effective_confirmed",false},{"last_known_sequence",t.last_known_sequence}};
    }
    return result;
  }
  Json quality_report() const {
    Json report=Json::object();
    for(const auto & name:topics_) {
      auto it=topic_quality_.find(name);
      TopicQuality q;if(it!=topic_quality_.end())q=it->second;
      Json age=q.messages?Json(std::chrono::duration<double>(Clock::now()-q.last).count()):Json(nullptr);
      std::string state=q.publishers==0?"missing_publisher":q.messages==0?"awaiting_data":
        (name=="/tf_static" || name=="/map")?"latched_received":age.get<double>()>stale_sec_?"stale":"received";
      report[name]={{"state",state},{"messages",q.messages},{"bytes",q.bytes},
        {"receive_age_s",age},{"maximum_receive_gap_s",q.maximum_gap_s},{"publishers",q.publishers},
        {"loss_count",nullptr},{"time_basis","recorder_steady_receive_time"}};
    }
    return report;
  }
  void read_next(const std::string & node, uint64_t generation, uint64_t revision,
    uint64_t token, size_t index, std::shared_ptr<Json> values) {
    auto & target=targets_.at(node);
    auto request=std::make_shared<Get::Request>();request->names={target.names.at(index)};
    auto future=target.client->async_send_request(request,
      [this,node,generation,revision,token,index,values](rclcpp::Client<Get>::SharedFuture f) {
        if(!active_ || generation!=generation_)return;
        auto & t=targets_.at(node);
        if(!t.pending || token!=t.request_token)return;
        try {
          if(!identity(node) || token!=t.request_token)return;
          if(t.event_revision!=revision) {
            t.pending=false;gap(node,"snapshot_overlapped_change");return;
          }
          auto response=f.get();
          (*values)[t.names[index]]=response->values.size()==1 ? value(response->values.front()) : station::parameter_value(0,nullptr);
          if(index+1<t.names.size()) {read_next(node,generation,revision,token,index+1,values);return;}
          t.pending=false;
          bool unobserved=false;
          for(auto it=t.observed.begin();it!=t.observed.end();++it)
            if(values->contains(it.key()) && (*values)[it.key()]!=it.value())unobserved=true;
          if(unobserved)gap(node,"readback_differs_without_event");
          event({{"kind","parameter_snapshot"},{"node",node},{"node_instance_id",t.instance},
            {"identity_quality",t.identity_quality},{"read_begin_ros_ns",t.read_begin_ns},
            {"values",*values},{"effective_confirmed",false}});
          t.observed=*values;t.last_known_sequence=sequence_;
        }catch(const std::exception & e){fail(e.what());}
      });
    target.request_id=future.request_id;
  }
  void snapshots() {
    if(!active_)throw std::runtime_error("Recording is not active");
    for(auto & [node,t]:targets_) {
      if(!identity(node)){gap(node,"endpoint_"+t.identity_quality);continue;}
      if(t.pending)continue;
      if(!t.client->service_is_ready()) {gap(node,"service_unavailable");continue;}
      t.pending=true;++t.request_token;t.deadline=Clock::now()+std::chrono::seconds(3);
      t.read_begin_ns=now().nanoseconds();
      read_next(node,generation_,t.event_revision,t.request_token,0,std::make_shared<Json>(Json::object()));
    }
  }
  void discover() {
    const auto graph=get_topic_names_and_types();
    for(const auto & name:topics_)topic_quality_[name].publishers=get_publishers_info_by_topic(name).size();
    for(const auto & topic:topics_) {
      if(subs_.count(topic) || !graph.count(topic) || graph.at(topic).size()!=1)continue;
      const auto type=graph.at(topic).front();
      rclcpp::QoS qos(50);qos.best_effort();
      if(topic=="/tf_static" || topic=="/map" || topic=="/map_nav")qos.reliable().transient_local();
      // Only explicitly listed topics are recorded; playback never republishes them.
      subs_[topic]=create_generic_subscription(topic,type,qos,[this,topic,type](std::shared_ptr<rclcpp::SerializedMessage> msg){
        if(!active_)return;
        try {
          auto & quality=topic_quality_[topic];auto received=Clock::now();
          if(quality.messages)quality.maximum_gap_s=std::max(quality.maximum_gap_s,std::chrono::duration<double>(received-quality.last).count());
          quality.last=received;++quality.messages;quality.bytes+=msg->size();
          if(type=="std_msgs/msg/String"&&(topic=="/map_manager/events"||topic=="/operator_backend/events"||topic=="/exploration_coordinator_node/command_events")) {
            if(msg->size()<=65536){
              std_msgs::msg::String decoded;rclcpp::Serialization<std_msgs::msg::String> serializer;serializer.deserialize_message(msg.get(),&decoded);
              auto payload=Json::parse(decoded.data,nullptr,false);
              if(payload.is_object())event({{"kind","operation_event"},{"topic",topic},{"payload",payload}});
              else event({{"kind","operation_event_invalid"},{"topic",topic}});
            }else event({{"kind","operation_event_invalid"},{"topic",topic},{"reason","oversized"}});
          }
          writer_->write(msg,topic,type,now());++messages_;
        }
        catch(const std::exception & e){fail(e.what());}
      });
      event({{"kind","topic_attached"},{"topic",topic},{"type",type}});
    }
  }
  std::string start() {
    if(active_)return "Already recording: "+directory_.string();
    fault_.clear();sequence_=messages_=0;++generation_;topic_quality_.clear();
    for(auto & [node,t]:targets_) {t.instance.clear();t.identity_quality="unavailable";t.observed=Json::object();t.last_known_sequence=0;}
    auto root=std::filesystem::path(get_parameter("output_root").as_string());
    std::filesystem::create_directories(root);
    if(std::filesystem::space(root).available < static_cast<uint64_t>(get_parameter("min_free_bytes").as_int()))throw std::runtime_error("Insufficient disk space");
    auto tick=std::chrono::system_clock::now().time_since_epoch().count();
    directory_=root/("incident_"+std::to_string(tick)+"_"+std::to_string(getpid()));
    if(!std::filesystem::create_directory(directory_))throw std::runtime_error("Session path collision");
    events_.open(directory_/"events.jsonl");if(!events_)throw std::runtime_error("Cannot create event file");
    active_=true;manifest("recording");
    log_=std::make_shared<spdlog::logger>("diagnostics",std::make_shared<spdlog::sinks::basic_file_sink_mt>((directory_/"session.log").string()));
    log_->flush_on(spdlog::level::info);
    writer_=std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions options;options.uri=(directory_/"bag").string();options.storage_id="sqlite3";
    options.max_bagfile_size=256*1024*1024;options.max_cache_size=8*1024*1024;
    writer_->open(options);
    event({{"kind","session_start"},{"topics",topics_}});
    Json config=Json::object();
    for (const auto & p:get_parameters({"output_root","robot_id","deployment_revision","min_free_bytes","topic_stale_sec","topics","parameters","use_sim_time"}))
      config[p.get_name()]=value(p.to_parameter_msg().value);
    event({{"kind","recorder_config"},{"values",config}});
    log_->info("Recording started {}",directory_.string());
    discover();snapshots();next_snapshot_=Clock::now()+std::chrono::seconds(10);
    return directory_.string();
  }
  void service(const std::string & name,std::function<std::string()> fn) {
    services_.push_back(create_service<Trigger>("~/"+name,[this,fn](std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> res){
      try {res->message=fn();res->success=true;}
      catch(const std::exception & e){res->success=false;res->message=e.what();if(active_)fail(e.what());}
    }));
  }
public:
  explicit Recorder(const rclcpp::NodeOptions & options=rclcpp::NodeOptions()):Node("diagnostics_recorder", options) {
    declare_parameter("output_root","/tmp/astribot_incidents");
    stale_sec_=declare_parameter("topic_stale_sec",3.0);
    if(!std::isfinite(stale_sec_) || stale_sec_<=0)throw std::runtime_error("topic_stale_sec must be positive");
    declare_parameter("robot_id","unknown");declare_parameter("deployment_revision","unknown");
    if (declare_parameter<int64_t>("min_free_bytes",1024LL*1024*1024) < 1) throw std::runtime_error("min_free_bytes must be positive");
    topics_=declare_parameter<std::vector<std::string>>("topics",{"/tf","/tf_static","/odom","/joint_states","/map","/plan","/navigation/execution_status","/navigation/policy_status","/exploration/state","/cmd_vel"});
    const auto specs=declare_parameter<std::vector<std::string>>("parameters",{"/controller_server:controller_frequency","/exploration_coordinator_node:arrival_xy_tolerance"});
    for(const auto & spec:specs) {
      auto pos=spec.find(':');if(pos==std::string::npos || pos==0 || pos+1==spec.size())throw std::runtime_error("Expected /node:parameter");
      targets_[spec.substr(0,pos)].names.push_back(spec.substr(pos+1));
    }
    for(auto & [node,t]:targets_) t.client=create_client<Get>(node+"/get_parameters");
    status_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).reliable().transient_local());
    events_sub_=create_subscription<rcl_interfaces::msg::ParameterEvent>("/parameter_events",rclcpp::ParameterEventsQoS(),[this](rcl_interfaces::msg::ParameterEvent::ConstSharedPtr msg, const rclcpp::MessageInfo & info){
      if(!active_ || !targets_.count(msg->node))return;
      try {
        auto & target=targets_.at(msg->node);
        if(!identity(msg->node)) {gap(msg->node,"ambiguous_or_missing_endpoint");return;}
        const auto & gid=info.get_rmw_message_info().publisher_gid;
        if(gid_text(gid.data,sizeof(gid.data))!=target.instance) {
          event({{"kind","parameter_event_rejected"},{"node",msg->node},{"reason","old_or_unknown_endpoint"}});return;
        }
        Json values=Json::object();const auto & names=target.names;
        auto add=[&](const auto & ps,bool deleted){for(const auto & p:ps)if(std::find(names.begin(),names.end(),p.name)!=names.end())values[p.name]=deleted?station::parameter_value(0,nullptr):value(p.value);};
        add(msg->new_parameters,false);add(msg->changed_parameters,false);add(msg->deleted_parameters,true);
        if(!values.empty()) {
          ++target.event_revision;
          event({{"kind","parameter_event"},{"node",msg->node},{"node_instance_id",target.instance},
            {"values",values},{"source_ros_ns",rclcpp::Time(msg->stamp).nanoseconds()}});
          for(auto it=values.begin();it!=values.end();++it)target.observed[it.key()]=it.value();
          target.last_known_sequence=sequence_;
        }
      }catch(const std::exception & e){fail(e.what());}
    });
    service("start",[this]{return start();});
    service("stop",[this]{if(!active_)return std::string("Already stopped");close("closed");return directory_.string();});
    service("snapshot",[this]{snapshots();return std::string("Snapshot requested; inspect events for observed/unknown results");});
    service("mark",[this]{if(!active_)throw std::runtime_error("Recording is not active");event({{"kind","operator_mark"}});return std::string("Recorded marker at sequence ")+std::to_string(sequence_);});
    timer_=create_wall_timer(std::chrono::seconds(1),[this]{
      try {
        if(active_) {
          if(std::filesystem::space(directory_).available < static_cast<uint64_t>(get_parameter("min_free_bytes").as_int()))throw std::runtime_error("Disk reserve reached");
          for(auto & [node,t]:targets_) {
            identity(node);
            if(t.pending && Clock::now()>t.deadline) {t.client->remove_pending_request(t.request_id);t.pending=false;++t.request_token;gap(node,"timeout");}
          }
          discover();
          event({{"kind","data_quality"},{"topics",quality_report()}});
          if(Clock::now()>=next_snapshot_){snapshots();next_snapshot_=Clock::now()+std::chrono::seconds(10);}
        }
        std::vector<std::string> missing;for(const auto & t:topics_)if(!subs_.count(t))missing.push_back(t);
        std_msgs::msg::String m;m.data=Json({{"recording",active_},{"directory",directory_.string()},{"messages",messages_},{"parameter_quality",parameter_quality()},{"topic_quality",quality_report()},{"missing_topics",missing},{"fault",fault_}}).dump();status_->publish(m);
      }catch(const std::exception & e){fail(e.what());}
    });
  }
  ~Recorder() override {try{close("closed");}catch(...) {}}
};

#ifndef ASTRIBOT_RECORDER_NO_MAIN
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {auto node=std::make_shared<Recorder>();rclcpp::spin(node);node.reset();}
  catch(const std::exception & e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}

#endif
