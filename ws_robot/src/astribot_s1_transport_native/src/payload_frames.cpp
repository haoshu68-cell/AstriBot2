#include "astribot_s1_transport_native/payload_frames.hpp"
#include <urdf_parser/urdf_parser.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>
#include <stdexcept>
namespace astribot::transport {
namespace {
void require(bool okay,const char *reason){if(!okay)throw std::runtime_error(reason);}
Eigen::Isometry3d fixed_chain(urdf::LinkConstSharedPtr &link,const std::string &stop) {
 Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
 while(link->name!=stop&&link->parent_joint&&link->parent_joint->type==urdf::Joint::FIXED) {
  const auto &p=link->parent_joint->parent_to_joint_origin_transform;
  Eigen::Isometry3d step=Eigen::Isometry3d::Identity();step.translation()=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
  step.linear()=Eigen::Quaterniond(p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z).toRotationMatrix();
  result=step*result;link=link->getParent();
 }
 return result;
}
}
void PayloadWorldInbox::bind(int64_t ros) {
 std::lock_guard<std::mutex> lock(mutex_);++generation_;queue_.clear();failure_.clear();(void)ros;
}
void PayloadWorldInbox::receive(const ignition::msgs::Pose_V &value,PayloadCommand::Receipt receipt,uint64_t ticket) {
 std::lock_guard<std::mutex> lock(mutex_);
 if(ticket!=generation_.load()||ticket==0)return;
 const auto &stamp=value.header().stamp();
 if(stamp.sec()<0||stamp.sec()>INT32_MAX||stamp.nsec()<0||stamp.nsec()>=1000000000){failure_="PAYLOAD_WORLD_STAMP_INVALID";return;}
 queue_.push_back({value,receipt});while(queue_.size()>256)queue_.pop_front();
}
PayloadWorldInbox::Samples PayloadWorldInbox::take() {
 std::lock_guard<std::mutex> lock(mutex_);
 if(!failure_.empty())throw std::runtime_error(failure_);
 Samples result;result.swap(queue_);return result;
}
PayloadFrames::PayloadFrames(const std::string &xml,const std::string &base,const std::string &tcp,
 std::string robot,uint64_t entity,const Eigen::Isometry3d &model_root):robot_(std::move(robot)),entity_(entity) {
 const auto model=urdf::parseURDF(xml);require(bool(model),"PAYLOAD_ROBOT_MODEL_INVALID");
 for(const auto &[name,link]:model->links_)known_links_.insert(name);
 auto root=model->getRoot();auto base_link=model->getLink(base),tcp_link=model->getLink(tcp);
 require(root&&base_link&&tcp_link&&entity_>0&&!robot_.empty(),"PAYLOAD_FRAME_IDENTITY_INVALID");
 require(model_root.matrix().allFinite()&&(model_root.linear().transpose()*model_root.linear()-Eigen::Matrix3d::Identity()).norm()<1e-9&&std::abs(model_root.linear().determinant()-1.)<1e-9,"PAYLOAD_MODEL_ROOT_TRANSFORM_INVALID");
 model_from_base_=model_root*fixed_chain(base_link,root->name);
 require(base_link->name==root->name,"PAYLOAD_BASE_NOT_FIXED_TO_MODEL_ROOT");
 parent_from_tcp_=fixed_chain(tcp_link,root->name);parent_=tcp_link->name;
 require(tcp_link->parent_joint&&tcp_link->parent_joint->type!=urdf::Joint::FIXED,"PAYLOAD_PHYSICAL_PARENT_NOT_ARTICULATED");
}
void PayloadFrames::observe(const ignition::msgs::Pose_V &value,PayloadCommand::Receipt receipt) {
 const auto &stamp=value.header().stamp();
 require(stamp.sec()>=0&&stamp.nsec()>=0&&stamp.nsec()<1000000000&&stamp.sec()<=INT32_MAX,"PAYLOAD_WORLD_STAMP_INVALID");
 const int64_t capture=stamp.sec()*1000000000+stamp.nsec();
 require(capture>0,"PAYLOAD_WORLD_STAMP_INVALID");
 const ignition::msgs::Pose *model=nullptr;
 for(const auto &pose:value.pose())if(pose.name()==robot_) {
  require(!model&&pose.id()==entity_,"PAYLOAD_WORLD_MODEL_CHANGED");model=&pose;
 }
 require(model,"PAYLOAD_WORLD_MODEL_MISSING");
 const auto &p=model->position();const auto &q=model->orientation();
 Eigen::Quaterniond rotation(q.w(),q.x(),q.y(),q.z());Eigen::Vector3d position(p.x(),p.y(),p.z());
 require(position.allFinite()&&rotation.coeffs().allFinite()&&std::abs(rotation.norm()-1.)<1e-6,"PAYLOAD_WORLD_POSE_INVALID");
 Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation()=position;pose.linear()=rotation.toRotationMatrix();
 if(!samples_.empty()&&capture<=samples_.back().capture) {
  if(capture==samples_.back().capture) {
   const auto previous=truth_.lookupTransform("physical_world","physical_robot_model",tf2::TimePoint(std::chrono::nanoseconds(capture)));
   require((tf2::transformToEigen(previous).matrix()-pose.matrix()).norm()<1e-12,"PAYLOAD_WORLD_CAPTURE_CONFLICT");
  }
  return;
 }
 auto transform=tf2::eigenToTransform(pose);transform.header.frame_id="physical_world";transform.child_frame_id="physical_robot_model";
 transform.header.stamp.sec=stamp.sec();transform.header.stamp.nanosec=stamp.nsec();
 require(truth_.setTransform(transform,"owned_gazebo_model",false),"PAYLOAD_WORLD_TRANSFORM_REJECTED");
 samples_.push_back({capture,receipt.steady});while(samples_.size()>128)samples_.pop_front();
}
std::optional<Eigen::Isometry3d> PayloadFrames::world_from_base(int64_t capture,int64_t ros,int64_t steady) const {
 (void)ros;(void)steady;
 if(samples_.empty())return {};
 const auto selected=std::min_element(samples_.begin(),samples_.end(),[capture](const auto&a,const auto&b){return std::abs(a.capture-capture)<std::abs(b.capture-capture);})->capture;
 const auto pose=truth_.lookupTransform("physical_world","physical_robot_model",tf2::TimePoint(std::chrono::nanoseconds(selected)));
 return tf2::transformToEigen(pose)*model_from_base_;
}
}
