#include "astribot_operator_station/replay_policy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
// No service/action clients and no original control-topic publishers exist here.
int main(int argc,char ** argv) {
 if(argc!=5){std::cerr<<"replay_stream BAG PREFIX OFFSET_SECONDS RATE\n";return 2;}
 try {
   const std::string bag=argv[1],prefix=argv[2];double offset=std::stod(argv[3]),rate=std::stod(argv[4]);
   if(prefix.rfind("/astribot_replay_",0)!=0||prefix.find("..")!=std::string::npos||!std::isfinite(offset)||offset<0||!std::isfinite(rate)||rate<0.1||rate>4)throw std::runtime_error("Invalid replay arguments");
   rclcpp::init(0,nullptr);auto node=std::make_shared<rclcpp::Node>("read_only_replay");
   rosbag2_cpp::Reader reader;reader.open(bag);
   const auto & meta=reader.get_metadata();const int64_t start=meta.starting_time.time_since_epoch().count();
   const int64_t target=start+static_cast<int64_t>(offset*1e9);
   std::map<std::string,rclcpp::GenericPublisher::SharedPtr> publishers;
   for(const auto & topic:reader.get_all_topics_and_types())if(astribot_operator_station::replay_allowed(topic.name,topic.type))
     publishers[topic.name]=node->create_generic_publisher(prefix+topic.name,topic.type,rclcpp::QoS(100).reliable().transient_local());
   auto clock=node->create_publisher<rosgraph_msgs::msg::Clock>(prefix+"/clock",rclcpp::QoS(1));
   std::cout<<"META "<<nlohmann::json({{"start_ns",start},{"duration_sec",meta.duration.count()/1e9},{"topics",publishers.size()}}).dump()<<std::endl;
   fcntl(STDIN_FILENO,F_SETFL,fcntl(STDIN_FILENO,F_GETFL,0)|O_NONBLOCK);
   bool paused=false;std::string input;int64_t current=start,last=start;
   auto anchor=std::chrono::steady_clock::now();auto reported=anchor;int64_t anchor_ns=target;
   // Allow the separately launched RViz to subscribe before reconstruction.
   std::this_thread::sleep_for(std::chrono::seconds(2));anchor=std::chrono::steady_clock::now();
   auto commands=[&]{char bytes[256];ssize_t n=read(STDIN_FILENO,bytes,sizeof(bytes));if(n>0)input.append(bytes,n);
     for(size_t end;(end=input.find('\n'))!=std::string::npos;){auto line=input.substr(0,end);input.erase(0,end+1);
       auto j=nlohmann::json::parse(line,nullptr,false);if(j.is_discarded())continue;
       anchor_ns=current;anchor=std::chrono::steady_clock::now();
       if(j.contains("paused"))paused=j.at("paused").get<bool>();
       if(j.contains("rate")){double v=j.at("rate");if(std::isfinite(v)&&v>=0.1&&v<=4)rate=v;}
     }
   };
   while(rclcpp::ok()&&reader.has_next()) {
     auto message=reader.read_next();if(message->time_stamp<last)throw std::runtime_error("Non-monotonic bag time; split clock epochs before replay");last=message->time_stamp;
     commands();
     while(rclcpp::ok()&&message->time_stamp>=target&&(paused||anchor_ns+static_cast<int64_t>(std::chrono::duration<double>(std::chrono::steady_clock::now()-anchor).count()*rate*1e9)<message->time_stamp)){
       commands();rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(5));
     }
     current=message->time_stamp;
     rosgraph_msgs::msg::Clock time;time.clock=rclcpp::Time(current);clock->publish(time);
     auto pub=publishers.find(message->topic_name);if(pub!=publishers.end()) {rclcpp::SerializedMessage serialized(*message->serialized_data);pub->second->publish(serialized);}
     rclcpp::spin_some(node);
     // Reconstruct from the beginning on each seek, bounded to one message in memory.
     if(current<target)std::this_thread::sleep_for(std::chrono::milliseconds(1));
     if(std::chrono::steady_clock::now()-reported>std::chrono::milliseconds(100)||!reader.has_next()){std::cout<<"POSITION "<<current<<std::endl;reported=std::chrono::steady_clock::now();}
   }
   std::cout<<"EOF\n"<<std::flush;
   while(rclcpp::ok()){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(20));}
   rclcpp::shutdown();return 0;
 }catch(const std::exception & e){std::cerr<<"REPLAY_ERROR "<<e.what()<<std::endl;return 1;}
}
