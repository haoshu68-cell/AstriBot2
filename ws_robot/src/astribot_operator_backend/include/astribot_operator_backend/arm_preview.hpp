#pragma once
#include <rclcpp/rclcpp.hpp>
#include <astribot_transport_msgs/srv/plan_skill.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nlohmann/json.hpp>
#include <set>
#include <algorithm>
#include <cmath>

// Planning only. Execution remains owned by the transport resource/hold transaction.
class ArmPreview {
 using Json=nlohmann::json;
 using Plan=astribot_transport_msgs::srv::PlanSkill;
 using Clock=std::chrono::steady_clock;
 rclcpp::Node & node_;
 rclcpp::Client<Plan>::SharedPtr planner_;
 rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
 rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_;
 sensor_msgs::msg::JointState joints_,start_;
 Clock::time_point joint_at_{},requested_{},ready_{};
 double ttl_,timeout_,tolerance_;
 std::string id_,state_{"EMPTY"},reason_{"ARM.NO_PLAN"},context_;
 uint64_t generation_{0};bool pending_{false};int64_t request_id_{0};
 std::function<void(std::string,std::string)> completion_;
 bool fresh()const {
   const double age=(node_.now()-rclcpp::Time(joints_.header.stamp)).seconds();
   return joints_sub_->get_publisher_count()==1&&Clock::now()-joint_at_<std::chrono::seconds(2)&&age>=0&&age<2;
 }
 bool matching()const {
   for(size_t i=0;i<start_.name.size();++i){auto it=std::find(joints_.name.begin(),joints_.name.end(),start_.name[i]);
     if(it==joints_.name.end()||std::abs(joints_.position[it-joints_.name.begin()]-start_.position[i])>tolerance_)return false;}
   return true;
 }
 void clearDisplay(){display_->publish(moveit_msgs::msg::DisplayTrajectory());}
public:
 explicit ArmPreview(rclcpp::Node & node):node_(node){
   rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only=true;
   ttl_=node.declare_parameter("arm_preview_ttl_sec",30.0,fixed);
   timeout_=node.declare_parameter("arm_planning_timeout_sec",20.0,fixed);
   tolerance_=node.declare_parameter("arm_start_tolerance_rad",.025,fixed);
   if(!std::isfinite(ttl_)||ttl_<=0||ttl_>120||!std::isfinite(timeout_)||timeout_<=0||timeout_>20||!std::isfinite(tolerance_)||tolerance_<=0||tolerance_>.025)throw std::runtime_error("Invalid arm preview limits");
   planner_=node.create_client<Plan>("/transport/plan_skill");
   display_=node.create_publisher<moveit_msgs::msg::DisplayTrajectory>("/operator/arm_preview",rclcpp::QoS(1).transient_local());
   joints_sub_=node.create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr m){
     joint_at_={};if(m->header.stamp.sec<0||m->header.stamp.nanosec>=1000000000u)return;
     if(m->name.empty()||m->name.size()>256||m->name.size()!=m->position.size())return;
     std::set<std::string> names;for(size_t i=0;i<m->name.size();++i)if(m->name[i].empty()||!names.insert(m->name[i]).second||!std::isfinite(m->position[i]))return;
     joints_=*m;joint_at_=Clock::now();
   });
 }
 Json status()const{return {{"state",state_},{"plan_id",id_},{"reason_code",reason_},{"planning_available",planner_->service_is_ready()},
   {"can_execute",false},{"execution_reason","ARM.EXECUTION_ADAPTER_UNAVAILABLE"},{"payload_quality","UNKNOWN"},{"context",context_},{"ttl_sec",ttl_},{"planning_timeout_sec",timeout_},{"start_tolerance_rad",tolerance_}};}
 void invalidate(const std::string & reason){
   if(state_!="PLANNING"&&state_!="READY")return;
   ++generation_;state_="INVALIDATED";reason_=reason;clearDisplay();
   if(pending_){planner_->remove_pending_request(request_id_);pending_=false;auto cb=std::move(completion_);if(cb)cb("CANCELED",reason);}
 }
 void tick(const std::string & context){
   if(state_!="PLANNING"&&state_!="READY")return;
   if(context!=context_)invalidate("ARM.CONTEXT_CHANGED");
   else if(!fresh()||!matching())invalidate("ARM.START_STATE_CHANGED");
   else if(state_=="PLANNING"&&Clock::now()-requested_>std::chrono::duration<double>(timeout_))invalidate("ARM.PLANNING_TIMEOUT");
   else if(state_=="READY"&&Clock::now()-ready_>std::chrono::duration<double>(ttl_))invalidate("ARM.PLAN_EXPIRED");
 }
 void plan(const std::string & id,const Json & p,const std::string & context,std::function<void(std::string,std::string)> done){
   if(state_=="PLANNING")throw std::runtime_error("ARM.PLANNING_BUSY");
   if(!fresh())throw std::runtime_error("ARM.JOINT_STATE_UNAVAILABLE");
   if(!planner_->service_is_ready())throw std::runtime_error("ARM.PLANNER_UNAVAILABLE");
   auto q=std::make_shared<Plan::Request>();q->operation="named";q->group=p.at("group").get<std::string>();q->named_target=p.at("named_target").get<std::string>();
   if((q->group!="arm_left"&&q->group!="arm_right")||q->named_target!="transport_compact")throw std::runtime_error("ARM.TARGET_UNSUPPORTED");
   invalidate("ARM.PLAN_REPLACED");id_=id;context_=context;start_=joints_;state_="PLANNING";reason_="ARM.PLANNING";requested_=Clock::now();completion_=std::move(done);pending_=true;const auto generation=++generation_;
   auto f=planner_->async_send_request(q,[this,generation](rclcpp::Client<Plan>::SharedFuture f){
     if(generation!=generation_)return;
     pending_=false;auto cb=std::move(completion_);try {auto r=f.get();
     bool valid=r->success&&Clock::now()-requested_<std::chrono::duration<double>(timeout_)&&fresh()&&matching();const auto & t=r->trajectory.joint_trajectory;
     valid=valid&&!t.joint_names.empty()&&t.joint_names.size()<=32&&!t.points.empty()&&t.points.size()<=10000&&r->trajectory.multi_dof_joint_trajectory.points.empty();
     std::set<std::string> names;for(const auto & name:t.joint_names)valid=valid&&names.insert(name).second&&std::find(start_.name.begin(),start_.name.end(),name)!=start_.name.end();
     int64_t previous=-1;for(const auto & point:t.points){auto time=rclcpp::Duration(point.time_from_start).nanoseconds();valid=valid&&time>=0&&time>previous&&time<=300000000000LL;previous=time;
       valid=valid&&point.positions.size()==t.joint_names.size();
       for(const auto * values:{&point.positions,&point.velocities,&point.accelerations,&point.effort}){valid=valid&&(values->empty()||values->size()==t.joint_names.size());for(auto v:*values)valid=valid&&std::isfinite(v);}}
     if(valid){const auto & first=t.points.front();for(size_t i=0;i<t.joint_names.size();++i){auto it=std::find(start_.name.begin(),start_.name.end(),t.joint_names[i]);valid=valid&&std::abs(first.positions[i]-start_.position[it-start_.name.begin()])<=tolerance_;}}
     state_=valid?"READY":"FAILED";reason_=valid?"ARM.PREVIEW_READY":"ARM.INVALID_PLAN";
     if(valid){ready_=Clock::now();moveit_msgs::msg::DisplayTrajectory d;d.trajectory_start.joint_state=start_;d.trajectory.push_back(r->trajectory);display_->publish(d);}else clearDisplay();
     cb(valid?"SUCCEEDED":"FAILED",reason_);
     }catch(const std::exception &){state_="FAILED";reason_="ARM.INVALID_PLAN";clearDisplay();cb("FAILED",reason_);}
   });request_id_=f.request_id;
 }
};
