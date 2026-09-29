#pragma once
#include "astribot_s1_transport_native/payload_command.hpp"
#include <Eigen/Geometry>
#include <ignition/msgs/pose_v.pb.h>
#include <tf2/buffer_core.h>
#include <deque>
#include <optional>
#include <set>
#include <atomic>
#include <mutex>
namespace astribot::transport {
// Synchronizes the source-epoch binding with the Ignition receive callback.
class PayloadWorldInbox {
public:
 using Samples=std::deque<std::pair<ignition::msgs::Pose_V,PayloadCommand::Receipt>>;
 uint64_t ticket() const{return generation_.load();}
 void bind(int64_t ros);
 void receive(const ignition::msgs::Pose_V &sample,PayloadCommand::Receipt receipt,uint64_t ticket);
 Samples take();
private:
 std::mutex mutex_;std::atomic<uint64_t> generation_{0};
 Samples queue_;std::string failure_;
};
// Construct for one independently bound inventory source/clock epoch. The
// owner discards this buffer on a changed epoch. Pose_V links are never world
// poses; only the registered top-level model supplies the world anchor.
class PayloadFrames {
public:
 PayloadFrames(const std::string &urdf,const std::string &base,const std::string &tcp,
   std::string robot,uint64_t robot_entity,const Eigen::Isometry3d &model_from_root);
 const std::set<std::string> &known_links() const{return known_links_;}
 const std::string &parent_link() const{return parent_;}
 const Eigen::Isometry3d &parent_from_tcp() const{return parent_from_tcp_;}
 void observe(const ignition::msgs::Pose_V &sample,PayloadCommand::Receipt receipt);
 std::optional<Eigen::Isometry3d> world_from_base(int64_t capture,int64_t ros,int64_t steady) const;
private:
 struct Sample {int64_t capture,received_steady;};
 std::string robot_,parent_;
 std::set<std::string> known_links_;
 uint64_t entity_;
 Eigen::Isometry3d model_from_base_,parent_from_tcp_;
 tf2::BufferCore truth_;
 std::deque<Sample> samples_;
};
}
