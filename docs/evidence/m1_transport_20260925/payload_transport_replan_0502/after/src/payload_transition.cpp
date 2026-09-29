#include "astribot_s1_transport_mtc/payload_transition.hpp"
#include "astribot_s1_transport_mtc/canonical_octomap.hpp"
#include "astribot_s1_transport_mtc/canonical_scene.hpp"
#include "astribot_s1_transport_mtc/joint_margin.hpp"
#include <astribot_s1_manipulation/external_trajectory_validator.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astribot_s1_transport_mtc {
namespace {
// Joint samples and observation stamps advance during execution. Only occupancy
// data and the one authorized payload may otherwise differ from the input scene.
moveit_msgs::msg::PlanningScene fixedScene(moveit_msgs::msg::PlanningScene scene,
                                         const std::string& omitted_object) {
  scene.robot_state.joint_state=sensor_msgs::msg::JointState();
  scene.robot_state.multi_dof_joint_state=sensor_msgs::msg::MultiDOFJointState();
  auto& objects=scene.world.collision_objects;
  objects.erase(std::remove_if(objects.begin(),objects.end(),[&](const auto& object) {
    return object.id==omitted_object;
  }),objects.end());
  std::sort(objects.begin(),objects.end(),[](const auto& a,const auto& b){return a.id<b.id;});
  for(auto& object:objects)object.header.stamp=builtin_interfaces::msg::Time();
  auto& attached=scene.robot_state.attached_collision_objects;
  attached.erase(std::remove_if(attached.begin(),attached.end(),[&](const auto& object) {
    return object.object.id==omitted_object;
  }),attached.end());
  std::sort(attached.begin(),attached.end(),[](const auto& a,const auto& b){return a.object.id<b.object.id;});
  for(auto& object:attached)object.object.header.stamp=builtin_interfaces::msg::Time();
  scene.world.octomap.header.stamp=builtin_interfaces::msg::Time();
  scene.world.octomap.octomap.header.stamp=builtin_interfaces::msg::Time();
  scene.world.octomap.octomap.data.clear();
  return scene;
}

void requireGeometry(const moveit_msgs::msg::CollisionObject& object) {
  const auto pose_valid=[](const geometry_msgs::msg::Pose& pose) {
    const auto& p=pose.position;const auto& q=pose.orientation;
    return std::isfinite(p.x+p.y+p.z+q.x+q.y+q.z+q.w) &&
      std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<.001;
  };
  if(object.operation!=object.ADD || (object.primitives.empty() && object.meshes.empty()) ||
     !object.planes.empty() || object.primitives.size()!=object.primitive_poses.size() ||
     object.meshes.size()!=object.mesh_poses.size() || !pose_valid(object.pose))
    throw std::runtime_error("MTC_PAYLOAD_TARGET_GEOMETRY_INVALID");
  for(const auto& shape:object.primitives) {
    const size_t count=shape.type==shape.BOX?3:shape.type==shape.SPHERE?1:
      (shape.type==shape.CYLINDER || shape.type==shape.CONE)?2:0;
    if(!count || shape.dimensions.size()!=count ||
       std::any_of(shape.dimensions.begin(),shape.dimensions.end(),[](double v){return !std::isfinite(v)||v<=0.;}))
      throw std::runtime_error("MTC_PAYLOAD_TARGET_GEOMETRY_INVALID");
  }
  for(const auto& mesh:object.meshes) {
    if(mesh.vertices.empty() || mesh.triangles.empty())throw std::runtime_error("MTC_PAYLOAD_TARGET_GEOMETRY_INVALID");
    for(const auto& vertex:mesh.vertices)
      if(!std::isfinite(vertex.x+vertex.y+vertex.z))throw std::runtime_error("MTC_PAYLOAD_TARGET_GEOMETRY_INVALID");
    for(const auto& triangle:mesh.triangles)
      for(auto index:triangle.vertex_indices)
        if(index>=mesh.vertices.size())throw std::runtime_error("MTC_PAYLOAD_TARGET_GEOMETRY_INVALID");
  }
  for(const auto& pose:object.primitive_poses)
    if(!pose_valid(pose))throw std::runtime_error("MTC_PAYLOAD_TARGET_GEOMETRY_INVALID");
  for(const auto& pose:object.mesh_poses)
    if(!pose_valid(pose))throw std::runtime_error("MTC_PAYLOAD_TARGET_GEOMETRY_INVALID");
  if(object.subframe_names.size()!=object.subframe_poses.size())throw std::runtime_error("MTC_PAYLOAD_TARGET_GEOMETRY_INVALID");
  for(const auto& pose:object.subframe_poses)
    if(!pose_valid(pose))throw std::runtime_error("MTC_PAYLOAD_TARGET_GEOMETRY_INVALID");
}
}

PayloadTransitionResult revalidatePayloadTransition(
  const astribot_transport_msgs::srv::RevalidatePayloadTransition::Request& request,
  const std::string& cached_context, const PayloadTransitionBinding& binding,
  const std::vector<CachedStage>& stages,
  std::chrono::steady_clock::time_point cached_at,
  double velocity_scaling, double acceleration_scaling,
  const TransportReplanner& replan_transport, double time_scaling, double joint_margin) {
  if(request.context_id.empty() || request.scene.is_diff || request.scene.robot_state.is_diff)
    throw std::runtime_error("MTC_PAYLOAD_FULL_SCENE_REQUIRED");
  if(request.context_id!=cached_context || stages.empty())throw std::runtime_error("MTC_PAYLOAD_CONTEXT_UNKNOWN");
  const auto expires=cached_at+std::chrono::seconds(120);
  const auto deadline=std::min(expires,std::chrono::steady_clock::now()+std::chrono::seconds(10));
  if(std::chrono::steady_clock::now()>=expires)throw std::runtime_error("MTC_PAYLOAD_CONTEXT_EXPIRED");
  if(request.start_index!=4 || request.start_index>=stages.size())throw std::runtime_error("MTC_PAYLOAD_INDEX_INVALID");
  const bool attach=binding.operation=="PICK";
  if((!attach && binding.operation!="PLACE") || stages[3].id!=(attach?"ATTACH_CONFIRM":"DETACH_CONFIRM"))
    throw std::runtime_error("MTC_PAYLOAD_STAGE_INVALID");
  if(request.transaction_id.empty())throw std::runtime_error("MTC_PAYLOAD_TRANSACTION_REQUIRED");
  const auto& reference=*stages[request.start_index].scene;
  const auto& links=reference.getRobotModel()->getLinkModelNames();
  const std::set<std::string> known_links(links.begin(),links.end());
  // Validate every object before the authorized payload is omitted from the
  // static comparison. The same contract also bounds the planner's input.
  const auto current_scene=astribot::transport::canonicalScene(
    request.scene,reference.getPlanningFrame(),known_links);
  const auto input_scene=astribot::transport::canonicalScene(
    binding.input_scene,reference.getPlanningFrame(),known_links);
  if(!binding.transaction_id.empty()) {
    if(binding.transaction_id!=request.transaction_id)throw std::runtime_error("MTC_PAYLOAD_TRANSACTION_CHANGED");
    const auto confirmed_scene=astribot::transport::canonicalScene(
      binding.confirmed_scene,reference.getPlanningFrame(),known_links);
    if(fixedScene(confirmed_scene,"")!=fixedScene(current_scene,""))
      throw std::runtime_error("MTC_PAYLOAD_CONFIRMED_SCENE_CHANGED");
  }
  if(fixedScene(input_scene,binding.object_id)!=fixedScene(current_scene,binding.object_id))
    throw std::runtime_error("MTC_PAYLOAD_SCENE_CHANGED");
  const auto& map=current_scene.world.octomap;
  const auto& p=map.origin.position;const auto& q=map.origin.orientation;
  if(map.header.frame_id!="astribot_torso_base" || !std::isfinite(p.x+p.y+p.z+q.x+q.y+q.z+q.w) ||
     std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.001)
    throw std::runtime_error("MTC_PAYLOAD_OCTOMAP_FRAME_OR_ORIGIN_INVALID");
  const auto& octomap=map.octomap;
  (void)astribot::transport::canonical_octomap(std::string(octomap.data.begin(),octomap.data.end()),
                                            octomap.binary,octomap.resolution,octomap.id);
  const moveit_msgs::msg::CollisionObject* world_target=nullptr;
  const moveit_msgs::msg::AttachedCollisionObject* attached_target=nullptr;
  size_t world_count=0,attached_count=0;
  for(const auto& object:current_scene.world.collision_objects)
    if(object.id==binding.object_id){world_target=&object;++world_count;}
  for(const auto& object:current_scene.robot_state.attached_collision_objects)
    if(object.object.id==binding.object_id){attached_target=&object;++attached_count;}
  if(world_count!=(attach?0u:1u) || attached_count!=(attach?1u:0u))
    throw std::runtime_error("MTC_PAYLOAD_TARGET_STATE_INVALID");
  if(attach) {
    if(attached_target->object.header.frame_id!=attached_target->link_name)
      throw std::runtime_error("MTC_PAYLOAD_TARGET_FRAME_INVALID");
    const auto* predicted=stages[4].scene->getCurrentState().getAttachedBody(binding.object_id);
    if(!predicted || attached_target->link_name!=predicted->getAttachedLinkName())
      throw std::runtime_error("MTC_PAYLOAD_TARGET_LINK_CHANGED");
    requireGeometry(attached_target->object);
  } else {
    if(world_target->header.frame_id!=stages[4].scene->getPlanningFrame())
      throw std::runtime_error("MTC_PAYLOAD_TARGET_FRAME_INVALID");
    requireGeometry(*world_target);
  }

  auto updated=stages;
  bool transport_replanned=binding.transport_replanned;
  bool lift_validated=false;
  for(size_t index=request.start_index;index<stages.size();++index) {
    if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("MTC_PAYLOAD_BUDGET_EXHAUSTED");
    const auto& stage=stages[index];
    auto scene=planning_scene::PlanningScene::clone(stage.scene);
    // Keep this stage's planned temporary ACM. Only replace the payload and map.
    scene->getCurrentStateNonConst().clearAttachedBody(binding.object_id);
    scene->getWorldNonConst()->removeObject(binding.object_id);
    const bool processed=attach?scene->processAttachedCollisionObjectMsg(*attached_target):
                                scene->processCollisionObjectMsg(*world_target);
    if(!processed)throw std::runtime_error("MTC_PAYLOAD_TARGET_PROCESSING_FAILED");
    scene->processOctomapMsg(map);
    updated[index].scene=scene;
    if(stage.trajectory) {
      auto trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(*stage.trajectory,true);
      const auto* actual=scene->getCurrentState().getAttachedBody(binding.object_id);
      const auto replace_payload=[&](robot_trajectory::RobotTrajectory& path) {
        for(size_t waypoint=0;waypoint<path.getWayPointCount();++waypoint) {
          auto& state=*path.getWayPointPtr(waypoint);state.clearAttachedBody(binding.object_id);
          if(attach)state.attachBody(binding.object_id,actual->getPose(),actual->getShapes(),actual->getShapePoses(),
            actual->getTouchLinks(),actual->getAttachedLinkName(),actual->getDetachPosture(),actual->getSubframes());
          state.update();
        }
      };
      replace_payload(*trajectory);
      std::string reason;
      if(!astribot_s1_manipulation::validateExternalTrajectoryGeometry(scene,*trajectory,reason)) {
        if(!attach || index!=5 || stage.id!="TRANSPORT_POSTURE" || !lift_validated ||
           !binding.transaction_id.empty() || !replan_transport || reason.rfind("EXTERNAL_COLLISION:",0)!=0)
          throw std::runtime_error("MTC_PAYLOAD_REVALIDATION:"+stage.id+":"+reason);
        if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("MTC_PAYLOAD_BUDGET_EXHAUSTED");
        const auto start=updated[4].trajectory->getLastWayPoint();
        const auto goal=trajectory->getLastWayPoint();
        const auto* group=trajectory->getGroup();
        auto replan_scene=planning_scene::PlanningScene::clone(scene);
        replan_scene->setCurrentState(start);
        auto replacement=replan_transport(replan_scene,goal,group,deadline);
        if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("MTC_PAYLOAD_BUDGET_EXHAUSTED");
        if(!replacement || replacement->getGroup()!=group || replacement->getWayPointCount()<2)
          throw std::runtime_error("MTC_PAYLOAD_REPLAN_INVALID_TRAJECTORY");
        // Pipeline joint-goal tolerances must not change the owner's exact
        // boundaries. Validate and parameterize the bound path afterwards.
        *replacement->getWayPointPtr(0)=start;
        *replacement->getWayPointPtr(replacement->getWayPointCount()-1)=goal;
        replace_payload(*replacement);
        if(!astribot_s1_manipulation::validateExternalTrajectory(
             scene,*replacement,reason,velocity_scaling,acceleration_scaling,time_scaling))
          throw std::runtime_error("MTC_PAYLOAD_REPLAN:"+reason);
        for(size_t waypoint=0;waypoint<replacement->getWayPointCount();++waypoint) {
          const auto& state=replacement->getWayPoint(waypoint);
          for(const auto& joint:group->getVariableNames()) {
            const auto& bounds=scene->getRobotModel()->getVariableBounds(joint);
            if(bounds.position_bounded_ && !planningInterval(bounds.min_position_,bounds.max_position_,joint_margin)
                 .contains(state.getVariablePosition(joint)))
              throw std::runtime_error("MTC_PAYLOAD_REPLAN_JOINT_PLANNING_MARGIN:"+joint);
          }
          // The planner owns only the original trajectory group.
          for(size_t variable=0;variable<state.getVariableCount();++variable)
            if(std::find(group->getVariableIndexList().begin(),group->getVariableIndexList().end(),
                         static_cast<int>(variable))==group->getVariableIndexList().end() &&
               state.getVariablePositions()[variable]!=start.getVariablePositions()[variable])
              throw std::runtime_error("MTC_PAYLOAD_REPLAN_NON_GROUP_JOINT_CHANGED");
        }
        trajectory=std::move(replacement);transport_replanned=true;
      }
      if(attach && index==4 && stage.id=="LIFT")lift_validated=true;
      updated[index].trajectory=std::move(trajectory);
    }
    if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("MTC_PAYLOAD_BUDGET_EXHAUSTED");
  }
  return {std::move(updated),transport_replanned,deadline};
}
}
