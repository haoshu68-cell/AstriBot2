#include <gtest/gtest.h>
#include "astribot_s1_gazebo_bringup/payload_geometry.hpp"
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <sdf/Box.hh>
#include <sdf/Mesh.hh>
#include <fstream>
#include <unistd.h>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>
using namespace astribot::simulation;
namespace sim=ignition::gazebo;namespace c=sim::components;
TEST(PayloadGeometry,FixedTransformMustUseWholeFixedChainAndRejectMovingMount) {
 const std::string fixed="<robot name='r'><link name='p'/><link name='a'/><link name='tcp'/><joint name='j' type='fixed'><parent link='p'/><child link='a'/><origin xyz='0 0.1 0'/></joint><joint name='k' type='fixed'><parent link='a'/><child link='tcp'/><origin xyz='0 .05 0'/></joint></robot>";
 EXPECT_NEAR(fixed_parent_from_tcp(fixed,"p","tcp").Pos().Y(),.15,1e-12);
 EXPECT_THROW(fixed_parent_from_tcp(fixed,"absent","tcp"),std::invalid_argument);
 auto moving=fixed;moving.replace(moving.find("type='fixed'"),12,"type='floating'");
 EXPECT_THROW(fixed_parent_from_tcp(moving,"p","tcp"),std::invalid_argument);
}
TEST(PayloadGeometry,MassOffsetAndTrackingBoundComeFromPhysicalComponents) {
 sim::EntityComponentManager e;const auto world=e.CreateEntity();e.CreateComponent(world,c::World());const auto parent=e.CreateEntity(),model=e.CreateEntity(),link=e.CreateEntity(),collision=e.CreateEntity();
 e.CreateComponent(parent,c::ParentEntity(world));e.CreateComponent(model,c::ParentEntity(world));e.CreateComponent(parent,c::Link());e.CreateComponent(parent,c::Pose());e.CreateComponent(model,c::Model());e.CreateComponent(model,c::Pose());
 e.CreateComponent(link,c::Link());e.CreateComponent(link,c::Pose());e.CreateComponent(link,c::ParentEntity(model));
 ignition::math::Inertiald mass;ignition::math::MassMatrix3d mm(.75,{.01,.01,.01},{0,0,0});mass.SetMassMatrix(mm);e.CreateComponent(link,c::Inertial(mass));
 e.CreateComponent(collision,c::Collision());e.CreateComponent(collision,c::ParentEntity(link));e.CreateComponent(collision,c::Pose());
 sdf::Geometry g;sdf::Box box;box.SetSize({.2,.2,.2});g.SetType(sdf::GeometryType::BOX);g.SetBoxShape(box);e.CreateComponent(collision,c::Geometry(g));
 PayloadExecution p;p.attached=true;p.parent=parent;p.capture=100;p.epoch="e";p.accepted=p.applied=1;
 write_execution(model,e,p);auto actual=read_execution(model,e);ASSERT_TRUE(actual);EXPECT_EQ(actual->applied,1u);
 auto out=observed_payload(e,model,p,"box","tcp",ignition::math::Pose3d(0,.15,0,0,0,0));
 ASSERT_EQ(out.object.primitives.size(),1u);EXPECT_DOUBLE_EQ(out.weight,.75);EXPECT_NEAR(out.object.pose.position.y,-.15,1e-12);
 // Translation 5 mm plus angular 10 mrad at sqrt(3)*0.1 m radius, each side.
 EXPECT_NEAR(out.object.primitives[0].dimensions[0],.21346410161513776,1e-12);
 char path[]="/tmp/astribot_payload_mesh_XXXXXX.obj";int fd=mkstemps(path,4);ASSERT_GE(fd,0);close(fd);
 {std::ofstream mesh(path);mesh<<"v -0.1 -0.1 -0.1\nv 0.1 -0.1 -0.1\nv 0.1 0.1 -0.1\nv -0.1 0.1 -0.1\nv -0.1 -0.1 0.1\nv 0.1 -0.1 0.1\nv 0.1 0.1 0.1\nv -0.1 0.1 0.1\nf 1 2 3\nf 1 3 4\nf 5 7 6\nf 5 8 7\nf 1 5 6\nf 1 6 2\nf 2 6 7\nf 2 7 3\nf 3 7 8\nf 3 8 4\nf 4 8 5\nf 4 5 1\n";}
 sdf::Mesh mesh;mesh.SetUri(path);mesh.SetScale({1,1,1});sdf::Geometry mg;mg.SetType(sdf::GeometryType::MESH);mg.SetMeshShape(mesh);
 e.SetComponentData<c::Geometry>(collision,mg);
 try {auto enclosed=observed_payload(e,model,p,"mesh","tcp",{});
   ASSERT_EQ(enclosed.object.primitives.size(),1u);EXPECT_EQ(enclosed.object.primitives[0].type,shape_msgs::msg::SolidPrimitive::BOX);
   EXPECT_NEAR(enclosed.object.primitives[0].dimensions[0],.21346410161513776,1e-7);
 }catch(const std::exception &error) {
   const auto *loaded=ignition::common::MeshManager::Instance()->Load(path);
   ADD_FAILURE()<<error.what()<<" uri="<<mesh.Uri()<<" submesh="<<mesh.Submesh()<<" center="<<mesh.CenterSubmesh()
     <<" vertices="<<(loaded?loaded->VertexCount():0)<<" min="<<(loaded?loaded->Min():ignition::math::Vector3d())<<" max="<<(loaded?loaded->Max():ignition::math::Vector3d())
     <<" vertex0="<<(loaded && loaded->SubMeshCount()?loaded->SubMeshByIndex(0).lock()->Vertex(0):ignition::math::Vector3d());
 }
 unlink(path);
 e.SetComponentData<c::Geometry>(collision,g);
 e.RemoveComponent<c::Pose>(parent);
 EXPECT_THROW(observed_payload(e,model,p,"box","tcp",{}),std::invalid_argument);
 e.CreateComponent(parent,c::Pose());e.RemoveComponent<c::Pose>(model);
 EXPECT_THROW(observed_payload(e,model,p,"box","tcp",{}),std::invalid_argument);
 e.CreateComponent(model,c::Pose());
 e.SetComponentData<c::Pose>(model,ignition::math::Pose3d(.006,0,0,0,0,0));
 EXPECT_THROW(observed_payload(e,model,p,"box","tcp",{}),std::invalid_argument);
}
