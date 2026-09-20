#pragma once

#include <astribot_s1_autonomy/self_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Shared geometry, evaluated in-process before LIO and keyframe accumulation.
class RobotSelfFilter
{
  struct Chain { std::string name; double radius; std::vector<std::string> frames; };
  std::vector<Chain> chains_;
  astribot_s1_autonomy::SelfFilter filter_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Node::SharedPtr node_;
  std::string base_;
public:
  void configure(const rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    base_ = node->declare_parameter<std::string>("self_filter.base_frame", "astribot_torso_base");
    astribot_s1_autonomy::FootprintCylinder footprint;
    footprint.enabled = node->declare_parameter<bool>("self_filter.footprint.enabled", true);
    footprint.radius = node->declare_parameter<double>("self_filter.footprint.radius", 0.44);
    footprint.z_min = node->declare_parameter<double>("self_filter.footprint.z_min", -0.20);
    footprint.z_max = node->declare_parameter<double>("self_filter.footprint.z_max", 0.20);
    filter_.setFootprint(footprint);
    for(const auto &name: node->declare_parameter<std::vector<std::string>>("self_filter.chain_names", {}))
    {
      const std::string prefix = "self_filter.chains." + name + ".";
      Chain chain{name, node->declare_parameter<double>(prefix+"radius", 0.0),
                  node->declare_parameter<std::vector<std::string>>(prefix+"frames", {})};
      if(!std::isfinite(chain.radius) || chain.radius <= 0 || chain.frames.size() < 2)
        throw std::invalid_argument("Invalid self-filter chain: " + name);
      chains_.push_back(std::move(chain));
    }
    buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }

  bool update(const builtin_interfaces::msg::Time &stamp)
  {
    std::vector<astribot_s1_autonomy::FilterCapsule> capsules;
    try
    {
      for(const auto &chain: chains_)
      {
        std::vector<geometry_msgs::msg::Vector3> points;
        for(const auto &frame: chain.frames)
        {
          const auto tf = buffer_->lookupTransform(base_, frame, tf2::TimePointZero);
          const rclcpp::Time tf_time(tf.header.stamp);
          // Fixed links have stamp zero. Dynamic links must be from this scan's epoch.
          if(tf_time.nanoseconds() != 0 && std::abs((rclcpp::Time(stamp)-tf_time).seconds()) > 0.25)
            throw std::runtime_error("stale link TF: " + frame);
          points.push_back(tf.transform.translation);
        }
        for(size_t i=1; i<points.size(); ++i)
        {
          const auto &a=points[i-1], &b=points[i];
          capsules.push_back({float(a.x), float(a.y), float(a.z), float(b.x), float(b.y),
                              float(b.z), float(chain.radius), chain.name});
        }
      }
    }
    catch(const std::exception &exc)
    {
      filter_.setCapsules({});
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "LiDAR dropped: robot self-filter TF unavailable (%s)", exc.what());
      return false;
    }
    filter_.setCapsules(std::move(capsules));
    return true;
  }

  bool contains(const Eigen::Vector3d &point) const
  {
    astribot_s1_autonomy::SlicePoint p;
    p.x = point.x(); p.y = point.y(); p.z = point.z();
    return filter_.isSelfPoint(p);
  }
};
