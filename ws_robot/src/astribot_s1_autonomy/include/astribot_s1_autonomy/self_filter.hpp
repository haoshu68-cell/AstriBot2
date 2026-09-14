// Copyright 2026 Astribot.
#ifndef ASTRIBOT_S1_AUTONOMY__SELF_FILTER_HPP_
#define ASTRIBOT_S1_AUTONOMY__SELF_FILTER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "astribot_s1_autonomy/slice_projector.hpp"

namespace astribot_s1_autonomy
{

/// 胶囊体：以线段 p0-p1 为轴、半径 radius 的圆柱加两端半球。
/// 坐标均为 base_frame 下的米。
struct FilterCapsule
{
  float x0{0.0F};
  float y0{0.0F};
  float z0{0.0F};
  float x1{0.0F};
  float y1{0.0F};
  float z1{0.0F};
  float radius{0.0F};
  /// 仅用于 Marker 命名和日志。
  std::string name;
};

/// 底盘足迹圆柱：以 base_frame 原点为轴心的竖直圆柱，用于剔除底盘/轮子自身点。
/// 注意它跟着 base_frame 走，底盘倾斜时圆柱也跟着倾斜。
struct FootprintCylinder
{
  bool enabled{false};
  double radius{0.0};
  double z_min{0.0};
  double z_max{0.0};
};

/// 自身点云剔除器。每帧调用 setCapsules() 更新连杆位姿后再逐点判断。
class SelfFilter
{
public:
  SelfFilter() = default;

  void setFootprint(const FootprintCylinder & footprint) {footprint_ = footprint;}

  /// 用本帧 TF 解析出的胶囊体列表整体替换旧的。
  /// 传空表示本帧没有拿到任何连杆位姿（节点层应已打过 WARN），
  /// 此时只有足迹圆柱生效——宁可少剔一点，也不要拿上一帧的过期位姿去剔，
  /// 那样在机械臂快速运动时会剔错位置。
  void setCapsules(std::vector<FilterCapsule> capsules) {capsules_ = std::move(capsules);}

  const std::vector<FilterCapsule> & capsules() const {return capsules_;}
  const FootprintCylinder & footprint() const {return footprint_;}

  /// 判断某点是否属于机器人自身（应被剔除）。
  bool isSelfPoint(const SlicePoint & p) const;

private:
  FootprintCylinder footprint_;
  std::vector<FilterCapsule> capsules_;
};

/// 点到线段的最短距离的平方。抽出来单独暴露，方便单测。
/// 退化情况（线段两端重合）会自动退化成点到点距离，不会出现除零。
float pointSegmentDistanceSquared(
  float px, float py, float pz,
  float ax, float ay, float az,
  float bx, float by, float bz);

}  // namespace astribot_s1_autonomy

#endif  // ASTRIBOT_S1_AUTONOMY__SELF_FILTER_HPP_
