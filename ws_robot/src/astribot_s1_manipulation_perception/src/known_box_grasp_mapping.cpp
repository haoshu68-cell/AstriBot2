#include <astribot_s1_manipulation_perception/known_box_grasp_mapping.hpp>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shapes.h>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace astribot::perception_planning {
namespace {
Eigen::Isometry3d pose_transform(const geometry_msgs::msg::Pose &p) {
  Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  Eigen::Vector3d t(p.position.x,p.position.y,p.position.z);
  if(!q.coeffs().allFinite()||std::abs(q.squaredNorm()-1)>1e-6||!t.allFinite())
    throw std::runtime_error("INVALID_REGISTRATION_POSE");
  Eigen::Isometry3d out=Eigen::Isometry3d::Identity();out.linear()=q.toRotationMatrix();out.translation()=t;return out;
}
void supported(bool condition,const char *reason) {
  if(!condition)throw std::runtime_error(std::string("BOX_GRASP_UNSUPPORTED:")+reason);
}
} // namespace

GraspMapping map_known_box_grasp(const moveit_msgs::msg::CollisionObject &geometry,const Candidate &candidate,
  const Pose::Result &pose,const moveit::core::RobotModelConstPtr &model,
  const astribot_s1_manipulation::GripperCommander &gripper) {
  if(!model||!gripper.isConfigured()||gripper.jointName()!="astribot_gripper_left_joint_L1")
    throw std::runtime_error("LEFT_GRIPPER_REGISTRATION_NOT_CONFIGURED");
  supported(geometry.primitives.size()==1&&geometry.primitive_poses.size()==1&&geometry.meshes.empty()&&
    geometry.planes.empty()&&geometry.primitives.front().type==shape_msgs::msg::SolidPrimitive::BOX&&
    geometry.primitives.front().dimensions.size()==3,"SINGLE_BOX_REQUIRED");
  const auto &d=geometry.primitives.front().dimensions;
  const Eigen::Vector3d size(d[0],d[1],d[2]);supported(size.allFinite()&&(size.array()>0).all(),"BOX_DIMENSIONS");
  const double depth=candidate.gripper_depth_m;
  supported(std::isfinite(depth)&&depth>=.01-1e-7&&depth<=.04+1e-7&&
    std::abs(depth*100-std::round(depth*100))<1e-5,"MODEL_DEPTH");
  supported(std::isfinite(candidate.gripper_height_m)&&std::abs(candidate.gripper_height_m-.02)<1e-7,"MODEL_HEIGHT");
  const auto grasp_box=pose_transform(candidate.grasp_pose.pose).inverse()*
    pose_transform(pose.observation.pose.pose)*pose_transform(geometry.pose)*pose_transform(geometry.primitive_poses.front());
  // Only axis-aligned opposite faces; projected width alone would accept
  // diagonal edge grasps whose actual contact has not been established.
  const auto rotation=grasp_box.linear();
  for(int row=0;row<3;++row)supported(rotation.row(row).cwiseAbs().maxCoeff()>1-1e-6,"FACE_ALIGNMENT");
  const double width=rotation.row(1).cwiseAbs().dot(size);
  supported(std::isfinite(candidate.gripper_width_m)&&candidate.gripper_width_m>=width,
    "MODEL_OPENING_TOO_SMALL");
  supported(width<gripper.jawWidthAtAngle(gripper.openAngle()),"ACTUAL_OPENING_TOO_SMALL");
  // The registration branch does not silently re-center a network candidate.
  // Both opposite faces must remain within the configured final preload.
  double q;std::string why;
  if(gripper.graspAngleForWidth(width,q,why)!=astribot_s1_manipulation::PlanErrorCode::kSuccess)
    throw std::runtime_error("BOX_GRASP_UNSUPPORTED:CLOSURE:"+why);
  const double overlap=(width-gripper.jawWidthAtAngle(q))*.5;
  supported(std::abs(grasp_box.translation().y())<overlap,"OFF_CENTER_CONTACT");
  // Require a nonempty longitudinal intersection of the box and the model's
  // 60 mm fingers. This is not the subsequent full robot contact/sweep test.
  const double longitudinal=rotation.row(0).cwiseAbs().dot(size);
  const double vertical=rotation.row(2).cwiseAbs().dot(size);
  supported(grasp_box.translation().x()+longitudinal*.5>depth-.06&&
    grasp_box.translation().x()-longitudinal*.5<depth&&
    std::abs(grasp_box.translation().z())<(vertical+candidate.gripper_height_m)*.5,"NO_CONTACT_OVERLAP");

  moveit::core::RobotState state(model);state.setToDefaultValues();
  state.setJointPositions(model->getJointModel(gripper.jointName()),&q);state.update();
  const auto *tcp=model->getLinkModel("astribot_arm_left_tcp_link");
  const auto *base=model->getLinkModel("astribot_gripper_left_base");
  if(!tcp||!base)throw std::runtime_error("REGISTERED_GRIPPER_LINKS_MISSING");
  const Eigen::Isometry3d tcp_root=state.getGlobalLinkTransform(tcp).inverse();
  const Eigen::Isometry3d tcp_base=tcp_root*state.getGlobalLinkTransform(base);
  Eigen::Matrix3d grasp_tcp;
  grasp_tcp.row(0)=tcp_base.linear().col(2).transpose(); // approach
  grasp_tcp.row(1)=tcp_base.linear().col(0).transpose(); // jaws
  grasp_tcp.row(2)=tcp_base.linear().col(1).transpose(); // height
  Eigen::Vector3d lo[2],hi[2];
  const char *links[]={"astribot_gripper_left_Link_L11","astribot_gripper_left_Link_R11"};
  for(int side=0;side<2;++side) {
    const auto *link=model->getLinkModel(links[side]);
    if(!link||link->getShapes().empty())throw std::runtime_error("REGISTERED_PAD_GEOMETRY_MISSING");
    lo[side]=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());hi[side]=-lo[side];
    for(size_t shape=0;shape<link->getShapes().size();++shape) {
      if(link->getShapes()[shape]->type!=shapes::MESH)throw std::runtime_error("REGISTERED_PAD_MESH_REQUIRED");
      const auto *mesh=static_cast<const shapes::Mesh *>(link->getShapes()[shape].get());
      const Eigen::Isometry3d tcp_shape=tcp_root*state.getCollisionBodyTransform(link,shape);
      for(unsigned vertex=0;vertex<mesh->vertex_count;++vertex) {
        const Eigen::Vector3d p=grasp_tcp*(tcp_shape*Eigen::Vector3d(mesh->vertices[3*vertex],mesh->vertices[3*vertex+1],mesh->vertices[3*vertex+2]));
        lo[side]=lo[side].cwiseMin(p);hi[side]=hi[side].cwiseMax(p);
      }
    }
  }
  if((lo[0].y()+hi[0].y())<(lo[1].y()+hi[1].y()))throw std::runtime_error("REGISTERED_PAD_BRANCH_CHANGED");
  if(std::abs(hi[0].x()-hi[1].x())>1e-6)throw std::runtime_error("REGISTERED_DISTAL_PLANES_DIFFER");
  const Eigen::Vector3d tip_mid((hi[0].x()+hi[1].x())*.5,(lo[0].y()+hi[1].y())*.5,
    (lo[0].z()+hi[0].z()+lo[1].z()+hi[1].z())*.25);
  const Eigen::Vector3d translation=Eigen::Vector3d(depth,0,0)-tip_mid;
  const Eigen::Quaterniond orientation(grasp_tcp);
  GraspMapping out;out.physical_object_width_m=width;out.commanded_joint_angle_rad=q;
  out.grasp_from_tcp.translation.x=translation.x();out.grasp_from_tcp.translation.y=translation.y();
  out.grasp_from_tcp.translation.z=translation.z();out.grasp_from_tcp.rotation.x=orientation.x();
  out.grasp_from_tcp.rotation.y=orientation.y();out.grasp_from_tcp.rotation.z=orientation.z();out.grasp_from_tcp.rotation.w=orientation.w();
  return out;
}
}
