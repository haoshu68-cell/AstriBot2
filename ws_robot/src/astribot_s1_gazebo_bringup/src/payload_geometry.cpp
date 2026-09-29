#include "astribot_s1_gazebo_bringup/payload_geometry.hpp"
#include <astribot_s1_payload_state/kinematic_geometry.hpp>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#include <urdf_parser/urdf_parser.h>
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Mesh.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <filesystem>
namespace astribot::simulation {
namespace sim=ignition::gazebo;namespace c=sim::components;namespace math=ignition::math;
namespace {
void require(bool v,const char *why) {if(!v)throw std::invalid_argument(why);}
void require_pose_chain(sim::Entity entity,const sim::EntityComponentManager &ecm) {
 for(unsigned depth=0;depth<64;++depth) {
  require(ecm.HasEntity(entity),"PAYLOAD_POSE_METADATA_MISSING");
  if(ecm.Component<c::World>(entity))return;
  const auto *p=ecm.Component<c::Pose>(entity);const auto *parent=ecm.Component<c::ParentEntity>(entity);
  require(p && parent && p->Data().IsFinite(),"PAYLOAD_POSE_METADATA_MISSING");
  entity=parent->Data();
 }
 throw std::invalid_argument("PAYLOAD_POSE_CHAIN_INVALID");
}
geometry_msgs::msg::Pose pose(const math::Pose3d &p) {
 require(p.IsFinite(),"PAYLOAD_POSE_INVALID");geometry_msgs::msg::Pose out;
 out.position.x=p.Pos().X();out.position.y=p.Pos().Y();out.position.z=p.Pos().Z();
 auto q=p.Rot();q.Normalize();out.orientation.x=q.X();out.orientation.y=q.Y();out.orientation.z=q.Z();out.orientation.w=q.W();return out;
}
}
math::Pose3d fixed_parent_from_tcp(const std::string &xml,const std::string &parent,const std::string &tcp) {
 const auto model=urdf::parseURDF(xml);require(bool(model),"INVALID_ROBOT_URDF");
 auto link=model->getLink(tcp);math::Pose3d transform;
 for(unsigned depth=0;link && link->name!=parent && depth<64;++depth) {
  const auto joint=link->parent_joint;require(joint && joint->type==urdf::Joint::FIXED,"TCP_CHAIN_NOT_FIXED");
  const auto &p=joint->parent_to_joint_origin_transform;
  const math::Pose3d step({p.position.x,p.position.y,p.position.z},math::Quaterniond(p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z));
  require(step.IsFinite(),"TCP_TRANSFORM_INVALID");transform=step*transform;link=link->getParent();
 }
 require(link && link->name==parent,"TCP_PARENT_NOT_FOUND");return transform;
}
moveit_msgs::msg::AttachedCollisionObject observed_payload(const sim::EntityComponentManager &ecm,uint64_t model,
 const PayloadExecution &p,const std::string &id,const std::string &tcp,const math::Pose3d &parent_from_tcp) {
 require(p.attached && p.parent && ecm.HasEntity(p.parent) && ecm.HasEntity(model),"PAYLOAD_PARENT_UNAVAILABLE");
 require_pose_chain(p.parent,ecm);require_pose_chain(model,ecm);
 const auto delta=(sim::worldPose(p.parent,ecm)*p.offset).Inverse()*sim::worldPose(model,ecm);
 require(delta.IsFinite() && delta.Pos().Length()<=.005 && 2.*std::acos(std::clamp(std::abs(delta.Rot().W()),0.,1.))<=.01,"PAYLOAD_FOLLOW_ERROR");
 const auto links=ecm.ChildrenByComponents(model,c::Link());require(links.size()==1,"PAYLOAD_REQUIRES_SINGLE_RIGID_LINK");
 const auto *link_pose=ecm.Component<c::Pose>(links[0]);const auto *inertial=ecm.Component<c::Inertial>(links[0]);
 require(link_pose && inertial,"PAYLOAD_MASS_OR_POSE_UNKNOWN");
 const auto mass=inertial->Data().MassMatrix().Mass();require(std::isfinite(mass) && mass>0.,"PAYLOAD_MASS_INVALID");
 auto collisions=ecm.ChildrenByComponents(links[0],c::Collision());require(!collisions.empty() && collisions.size()<=64,"PAYLOAD_COLLISION_INCOMPLETE");
 std::sort(collisions.begin(),collisions.end());
 moveit_msgs::msg::AttachedCollisionObject out;out.link_name=tcp;out.weight=mass;out.touch_links={tcp};
 // Keep intentional pad contact in the authoritative attachment after temporary grasp ACM restoration.
 if(tcp=="astribot_arm_left_tcp_link")
   out.touch_links={tcp,"astribot_gripper_left_Link_L11","astribot_gripper_left_Link_R11"};
 else if(tcp=="astribot_arm_right_tcp_link")
   out.touch_links={tcp,"astribot_gripper_right_Link_L11","astribot_gripper_right_Link_R11"};
 out.object.id=id;out.object.header.frame_id=tcp;out.object.operation=out.object.ADD;
 out.object.pose=pose(parent_from_tcp.Inverse()*p.offset);
 for(auto entity:collisions) {
  const auto *geometry=ecm.Component<c::Geometry>(entity);const auto *origin=ecm.Component<c::Pose>(entity);
  require(geometry && origin,"PAYLOAD_COLLISION_METADATA_MISSING");
  auto local=link_pose->Data()*origin->Data();const auto &g=geometry->Data();
  shape_msgs::msg::SolidPrimitive shape;double radius=0.;
  if(g.Type()==sdf::GeometryType::BOX && g.BoxShape()) {
    const auto size=g.BoxShape()->Size();shape.type=shape.BOX;shape.dimensions={size.X(),size.Y(),size.Z()};radius=.5*size.Length();
  }else if(g.Type()==sdf::GeometryType::SPHERE && g.SphereShape()) {
    radius=g.SphereShape()->Radius();shape.type=shape.SPHERE;shape.dimensions={radius};
  }else if(g.Type()==sdf::GeometryType::CYLINDER && g.CylinderShape()) {
    const auto *v=g.CylinderShape();shape.type=shape.CYLINDER;shape.dimensions={v->Length(),v->Radius()};radius=std::hypot(v->Radius(),.5*v->Length());
  }else if(g.Type()==sdf::GeometryType::MESH && g.MeshShape()) {
    const auto *v=g.MeshShape();auto uri=v->Uri();if(uri.rfind("file://",0)==0)uri.erase(0,7);
    require(std::filesystem::path(uri).is_absolute() && std::filesystem::is_regular_file(uri) &&
      v->Submesh().empty() && !v->CenterSubmesh(),"PAYLOAD_MESH_REFERENCE_UNSUPPORTED");
    const auto scale=v->Scale();require(scale.IsFinite() && scale.X()>0 && scale.Y()>0 && scale.Z()>0,"PAYLOAD_MESH_SCALE_INVALID");
    // Gazebo's in-process mesh cache supplies the loaded vertices. Use an
    // enclosing box, never an inscribed fitted primitive or a visual-only mesh.
    const auto *mesh=ignition::common::MeshManager::Instance()->Load(uri);
    require(mesh && mesh->VertexCount()>0,"PAYLOAD_MESH_UNAVAILABLE");
    const auto lower=mesh->Min()*scale,upper=mesh->Max()*scale,size=upper-lower;
    require(lower.IsFinite() && upper.IsFinite(),"PAYLOAD_MESH_BOUNDS_INVALID");
    local=local*math::Pose3d((lower+upper)*.5,math::Quaterniond::Identity);
    shape.type=shape.BOX;shape.dimensions={size.X(),size.Y(),size.Z()};radius=.5*size.Length();
  }else throw std::invalid_argument("UNSUPPORTED_PHYSICAL_PAYLOAD_GEOMETRY");
  for(double d:shape.dimensions)require(std::isfinite(d) && d>0.,"PAYLOAD_DIMENSIONS_INVALID");
  require(local.IsFinite(),"PAYLOAD_COLLISION_POSE_INVALID");
  // Conservatively enclose bounded model-origin translation and orientation error.
  const auto margin=astribot::payload::kinematic_payload_margin(radius,local.Pos().Length());
  if(shape.type==shape.BOX)for(auto &d:shape.dimensions)d+=2.*margin;
  else if(shape.type==shape.SPHERE)shape.dimensions[0]+=margin;
  else {shape.dimensions[0]+=2.*margin;shape.dimensions[1]+=margin;}
  out.object.primitives.push_back(shape);out.object.primitive_poses.push_back(pose(local));
 }
 return out;
}
}
