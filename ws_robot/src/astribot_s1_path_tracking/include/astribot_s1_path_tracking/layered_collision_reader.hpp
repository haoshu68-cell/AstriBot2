#pragma once
#include <chrono>
#include <mutex>
#include "astribot_s1_path_tracking/envelope_evidence.hpp"
#include "astribot_s1_path_tracking/layered_collision.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace astribot_s1_path_tracking {
// Shared immutable geometry reader; it publishes no authority or installation ACKs.
class LayeredCollisionReader {
  using Envelope=astribot_navigation_msgs::msg::NavigationEnvelopeV2;
  using Maps=astribot_slam_msgs::msg::HeightSliceMaps;
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) {
    clock_=node->get_clock();costmap_=std::move(costmap);
    envelope_sub_=node->create_subscription<Envelope>("/navigation/envelope_v2",10,[this](Envelope::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now=clock_->now();
      evidence_.accept(value,now.nanoseconds(),std::chrono::steady_clock::now());
    });
    map_sub_=node->create_subscription<Maps>("/height_maps/snapshot",rclcpp::QoS(1).reliable().transient_local(),
      [this](Maps::ConstSharedPtr value) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now=clock_->now();
        if(maps_ && maps_->header.frame_id==value->header.frame_id &&
            rclcpp::Time(value->header.stamp)<=rclcpp::Time(maps_->header.stamp))return;
        maps_=std::move(value);map_received_=std::chrono::steady_clock::now();
      });
  }
  void cleanup() {
    envelope_sub_.reset();map_sub_.reset();std::lock_guard<std::mutex> lock(mutex_);
    evidence_.clear();maps_.reset();cached_.reset();cached_envelope_.reset();costmap_.reset();clock_.reset();
  }
  std::shared_ptr<const LayeredCollisionSnapshot> snapshot(
      const geometry_msgs::msg::PoseStamped &pose) {
    std::shared_ptr<const LayeredCollisionSnapshot> snapshot;
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!clock_)throw std::runtime_error("ALIGNMENT_LAYER_READER_UNCONFIGURED");
      const auto now=clock_->now();
      
      const auto sample=evidence_.sample(now.nanoseconds());
      const auto &envelope=sample.message;
      if(!envelope)throw std::runtime_error("ALIGNMENT_ENVELOPE_MISSING");
      if(envelope->header.frame_id!=costmap_->getBaseFrameID()||envelope->mode!=Envelope::FIXED_POSTURE)
        throw std::runtime_error("ALIGNMENT_ENVELOPE_FRAME_OR_MODE_MISMATCH");
      if(!maps_)throw std::runtime_error("ALIGNMENT_HEIGHT_MAP_MISSING");
      if(!cached_||cached_->revision()!=maps_->map_revision||!cached_envelope_||
          cached_envelope_->height_geometry_hash!=envelope->height_geometry_hash||
          !EnvelopeEvidence::sameExecution(*cached_envelope_,*envelope)) {
        cached_=std::make_shared<LayeredCollisionSnapshot>(maps_,envelope,maps_->header.frame_id);
        cached_envelope_=envelope;
      }
      snapshot=cached_;
      if(pose.header.frame_id!=maps_->header.frame_id) {
        const auto tf=costmap_->getTfBuffer()->lookupTransform(maps_->header.frame_id,pose.header.frame_id,tf2::TimePointZero);
        snapshot=cached_->inFrame(pose.header.frame_id,tf.transform.translation.x,tf.transform.translation.y,tf2::getYaw(tf.transform.rotation));
      }
    } catch(const std::exception &error) {
      throw nav2_core::PlannerException(std::string("ALIGNMENT_LAYER_UNAVAILABLE: ")+error.what());
    }
    return snapshot;
  }
  void requireRotationClear(const geometry_msgs::msg::PoseStamped &pose,double target_yaw) {
    const auto checked=snapshot(pose);
    if(checked->edgeCollision(pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation),
        pose.pose.position.x,pose.pose.position.y,target_yaw))
      throw nav2_core::PlannerException("ALIGNMENT_LAYER_ROTATION_COLLISION");
  }
private:
  std::mutex mutex_;EnvelopeEvidence evidence_;
  Maps::ConstSharedPtr maps_;Envelope::ConstSharedPtr cached_envelope_;
  std::shared_ptr<const LayeredCollisionSnapshot> cached_;
  std::chrono::steady_clock::time_point map_received_;
  rclcpp::Clock::SharedPtr clock_;std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp::Subscription<Envelope>::SharedPtr envelope_sub_;rclcpp::Subscription<Maps>::SharedPtr map_sub_;
};
}
