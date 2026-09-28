#pragma once
#include "astribot_s1_robot_geometry/robot_model.hpp"
#include "astribot_s1_payload_state/consumer.hpp"
namespace astribot_s1_robot_geometry {
inline int64_t attachmentCompletionDeadline(astribot::payload::Consumer &consumer,int64_t original,int64_t ros_now,int64_t steady_now) {
  const auto *current=consumer.current(ros_now,steady_now);
  geometryRequire(current!=nullptr,consumer.reason().c_str());
  const auto until=std::min(original,astribot::payload::ns(current->valid_until));
  return until;
}
struct AttachmentGeometry {
  std::vector<Shape> shapes;
  std::vector<std::string> ids;
  std::string revision;
  int64_t observed_at=0,valid_until=0;
};
inline AttachmentGeometry confirmedAttachments(astribot::payload::Consumer &consumer,int64_t ros_now,int64_t steady_now) {
  const auto *state=consumer.current(ros_now,steady_now);
  geometryRequire(state!=nullptr,consumer.reason().c_str());
  AttachmentGeometry out;out.revision=state->attachment_revision;
  out.observed_at=astribot::payload::ns(state->observation.observed_at);out.valid_until=astribot::payload::ns(state->valid_until);
  const auto transform=[](const geometry_msgs::msg::Pose &p) {
    Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
    geometryRequire(q.coeffs().allFinite() && std::abs(q.norm()-1.)<=1e-6,"INVALID_ATTACHMENT_ROTATION");
    Transform t=Transform::Identity();t.block<3,3>(0,0)=q.normalized().toRotationMatrix();
    t.block<3,1>(0,3)=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
    geometryRequire(t.allFinite(),"INVALID_ATTACHMENT_POSITION");return t;
  };
  for(const auto &a:state->observation.objects) {
    out.ids.push_back(a.object.id);
    for(std::size_t i=0;i<a.object.primitives.size();++i) {
      const auto &p=a.object.primitives[i];std::string kind;std::vector<double> dimensions(p.dimensions.begin(),p.dimensions.end());
      if(p.type==p.BOX)kind="box";else if(p.type==p.SPHERE)kind="sphere";
      else if(p.type==p.CYLINDER) {kind="cylinder";std::swap(dimensions[0],dimensions[1]);}
      else throw std::invalid_argument("UNSUPPORTED_ATTACHMENT_PRIMITIVE");
      out.shapes.emplace_back(a.link_name,kind,dimensions,transform(a.object.pose)*transform(a.object.primitive_poses.at(i)));
    }
  }
  return out;
}
}
