#pragma once
#include "astribot_navigation_zones/snapshot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
namespace astribot_navigation_zones {
inline double steadySeconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class Client {
 SnapshotCache cache_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
 rclcpp::Clock::SharedPtr clock_;
public:
 template<class Node> void init(Node & node,rclcpp::CallbackGroup::SharedPtr group=nullptr){
 clock_=node.get_clock();rclcpp::SubscriptionOptions options;options.callback_group=group;
 sub_=node.template create_subscription<std_msgs::msg::String>("/navigation_zones/constraints",rclcpp::QoS(1).transient_local().reliable(),[this](std_msgs::msg::String::ConstSharedPtr m){try{if(m->data.size()>131072)throw std::runtime_error("oversized constraints");cache_.receive(Json::parse(m->data),clock_->now().seconds(),steadySeconds());}catch(...){cache_.invalidate();}},options);
 }
 std::shared_ptr<const Snapshot> get()const{return sub_?cache_.get(clock_->now().seconds(),steadySeconds(),sub_->get_publisher_count()):nullptr;}
 void invalidate(){cache_.invalidate();}
};
class Gate {
 mutable std::mutex mutex_;Json status_;double received_{-1e30};
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;rclcpp::Clock::SharedPtr clock_;
public:
 Client constraints;
 template<class Node> void init(Node &node,rclcpp::CallbackGroup::SharedPtr group=nullptr){
 constraints.init(node,group);clock_=node.get_clock();rclcpp::SubscriptionOptions options;options.callback_group=group;
 status_sub_=node.template create_subscription<std_msgs::msg::String>("/navigation_zones/status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex_);try{if(m->data.size()>131072)throw std::runtime_error("oversized status");status_=Json::parse(m->data);received_=steadySeconds();}catch(...){received_=-1e30;}},options);
 }
 bool ready()const{auto s=constraints.get();if(!s||!status_sub_||status_sub_->get_publisher_count()!=1)return false;std::lock_guard<std::mutex> lock(mutex_);try{return steadySeconds()-received_<2.&&std::abs(clock_->now().seconds()-status_.at("stamp").get<double>())<2.&&status_.at("ready").get<bool>()&&status_.at("token")==s->token;}catch(...){return false;}}
};
inline std_msgs::msg::String acknowledgement(const Snapshot &s,const std::string &consumer,bool applied,const std::string &reason=""){
 std_msgs::msg::String m;m.data=Json({{"token",s.token},{"consumer",consumer},{"applied",applied},{"reason",reason}}).dump();return m;
}
}
