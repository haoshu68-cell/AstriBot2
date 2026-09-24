#include <astribot_s1_manipulation_perception/pick_planning_client.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace astribot::perception_planning {
namespace {
int64_t ns(const builtin_interfaces::msg::Time &t) {return int64_t(t.sec)*1000000000LL+t.nanosec;}
void require(bool condition,const char *reason) {if(!condition) throw std::runtime_error(reason);}
auto fields(const Context &c) {
  return std::tie(c.object_instance,c.identity_revision,c.camera_id,c.optical_frame,c.source_epoch,
    c.processing_epoch,c.model_id,c.pose_model_revision,c.grasp_model_revision,
    c.calibration_revision,c.scene_revision,c.envelope_epoch,c.clock_epoch,c.scene_signature,c.camera_info_revision,c.station_region_revision);
}
tf2::Transform transform(const geometry_msgs::msg::Pose &p) {
  const auto &q=p.orientation;const auto &v=p.position;
  require(std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z)&&
    std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z)&&std::isfinite(q.w)&&
    std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<1e-6,"INVALID_RIGID_TRANSFORM");
  tf2::Transform out;tf2::fromMsg(p,out);return out;
}
tf2::Transform transform(const geometry_msgs::msg::Transform &t) {
  geometry_msgs::msg::Pose p;p.orientation=t.rotation;
  p.position.x=t.translation.x;p.position.y=t.translation.y;p.position.z=t.translation.z;
  return transform(p);
}
geometry_msgs::msg::Pose pose_message(const tf2::Transform &t) {
  geometry_msgs::msg::Pose p;tf2::toMsg(t,p);return p;
}
} // namespace

void check_context(const Request &r,const Context &current,int64_t now,bool admission) {
  const auto &c=r.context;
  require(fields(c)==fields(current),"CONTEXT_CHANGED");
  require(!r.task_id.empty()&&!r.context_id.empty()&&!c.object_instance.empty()&&
    !c.identity_revision.empty()&&!r.detection_id.empty()&&!c.camera_id.empty()&&
    !c.source_epoch.empty()&&!c.optical_frame.empty()&&!c.model_id.empty()&&
    !c.pose_model_revision.empty()&&!c.grasp_model_revision.empty()&&!c.scene_signature.empty()&&
    c.calibration_revision>0&&c.scene_revision>0&&c.envelope_epoch>0,"CONTEXT_MISSING");
  require(r.segmentation_source=="single_instance_fixture"&&r.visible_instances==1,
          "EXPLICIT_SINGLE_INSTANCE_REQUIRED");
  require(r.object_cloud.header.frame_id==c.optical_frame,"CLOUD_FRAME_MISMATCH");
  const auto sample=ns(r.object_cloud.header.stamp),until=ns(r.valid_until);
  require(sample>0&&now>=sample&&now<until&&until-sample<=5000000000LL,
          "SNAPSHOT_EXPIRED_OR_FUTURE");
  if(admission) require(now-sample<=500000000LL,"INPUT_EXPIRED");
  require(r.admission_deadline_steady.time_since_epoch().count()!=0&&
    r.result_deadline_steady.time_since_epoch().count()!=0,"ORIGINAL_STEADY_DEADLINE_REQUIRED");
  require(std::chrono::steady_clock::now()<r.result_deadline_steady,"ORIGINAL_DEADLINE_EXPIRED");
  if(admission) require(std::chrono::steady_clock::now()<r.admission_deadline_steady,"INPUT_STEADY_EXPIRED");
}

InferenceGoals inference_goals(const Request &r,const Context &current,int64_t now) {
  check_context(r,current,now,true);
  require(std::isfinite(r.inference_timeout_s)&&r.inference_timeout_s>0&&
    r.inference_timeout_s<=120&&r.max_candidates>0&&r.max_candidates<=128,"INVALID_INFERENCE_BUDGET");
  InferenceGoals out;
  auto fill=[&](auto &g) {
    g.header=r.object_cloud.header;g.task_id=r.task_id;g.object_id=r.context.object_instance;
    g.camera_id=r.context.camera_id;g.source_epoch=r.context.source_epoch;
    g.processing_epoch=r.context.processing_epoch;g.object_cloud=r.object_cloud;
    g.calibration_revision=r.context.calibration_revision;g.planning_scene_revision=r.context.scene_revision;
    g.envelope_epoch=r.context.envelope_epoch;g.valid_until=r.valid_until;
    g.timeout_sec=std::min(r.inference_timeout_s,double(ns(r.valid_until)-now)*1e-9);
  };
  fill(out.pose);fill(out.grasps);
  out.pose.model_id=r.context.model_id;out.grasps.arm_id="left";
  out.grasps.max_candidates=r.max_candidates;
  return out;
}

Plan::Goal planning_goal(const Request &r,const Pose::Result &estimated,const Candidate &c,
                         const Context &current,int64_t now,GraspMapping *mapping_used) {
  check_context(r,current,now);
  const auto &ctx=r.context;const auto &o=estimated.observation;
  require(estimated.success&&o.position_valid&&o.orientation_valid,"OBJECT_6D_REQUIRED");
  require(o.object_id==ctx.object_instance&&o.source_model==ctx.model_id&&
    o.source_camera_id==ctx.camera_id&&o.source_epoch==ctx.source_epoch&&
    o.header==r.object_cloud.header&&o.model_revision==ctx.pose_model_revision&&
    o.calibration_revision==ctx.calibration_revision&&o.planning_scene_revision==ctx.scene_revision&&
    o.envelope_epoch==ctx.envelope_epoch&&o.valid_until==r.valid_until,"POSE_CONTEXT_MISMATCH");
  require(c.object_id==ctx.object_instance&&c.camera_id==ctx.camera_id&&c.source_epoch==ctx.source_epoch&&
    c.header==r.object_cloud.header&&c.grasp_pose.header==c.header&&c.arm_id=="left"&&
    c.model_name=="graspnet_baseline_torchscript"&&c.model_revision==ctx.grasp_model_revision&&
    c.calibration_revision==ctx.calibration_revision&&c.planning_scene_revision==ctx.scene_revision&&
    c.envelope_epoch==ctx.envelope_epoch&&c.valid_until==r.valid_until&&
    c.geometry_valid&&!c.candidate_id.empty(),"GRASP_CONTEXT_MISMATCH");
  require(!r.grasp_registration_revision.empty()&&!r.grasp_registration_evidence.empty()&&
    !r.geometry_revision.empty()&&bool(r.map_grasp),"REGISTRATION_EVIDENCE_REQUIRED");
  require(std::isfinite(r.minimum_width_m)&&std::isfinite(r.maximum_width_m)&&r.minimum_width_m>0&&
    r.maximum_width_m>r.minimum_width_m&&std::isfinite(r.approach_m)&&r.approach_m>0&&
    std::isfinite(r.lift_m)&&r.lift_m>0&&!r.touch_links.empty()&&
    std::isfinite(r.planning_timeout_s)&&r.planning_timeout_s>0&&r.planning_timeout_s<=60,
    "INVALID_PLANNING_CONFIGURATION");
  require(std::isfinite(c.gripper_width_m)&&c.gripper_width_m>0&&
    c.gripper_width_m<=r.maximum_width_m,"GRIPPER_WIDTH_UNSUPPORTED");
  const auto mapping=r.map_grasp(c,estimated);
  require(std::isfinite(mapping.physical_object_width_m)&&mapping.physical_object_width_m>=r.minimum_width_m&&
    mapping.physical_object_width_m<=c.gripper_width_m,"GRIPPER_WIDTH_UNSUPPORTED");
  require(r.base_from_camera.header.frame_id=="astribot_torso_base"&&
    r.base_from_camera.child_frame_id==ctx.optical_frame&&
    r.base_from_camera.header.stamp==r.object_cloud.header.stamp,"CAPTURE_TF_REQUIRED");
  require(!r.scene.is_diff&&!r.scene.robot_state.is_diff,"FULL_SCENE_REQUIRED");
  require(std::none_of(r.scene.robot_state.attached_collision_objects.begin(),
    r.scene.robot_state.attached_collision_objects.end(),[&](const auto &a){return a.object.id==ctx.object_instance;}),
    "TARGET_ALREADY_ATTACHED");
  require(std::count_if(r.scene.world.collision_objects.begin(),r.scene.world.collision_objects.end(),
    [&](const auto &a){return a.id==ctx.object_instance;})==1,"WORLD_TARGET_REQUIRED");
  const auto &geometry=r.model_geometry;
  require(geometry.id==ctx.object_instance&&geometry.header.frame_id==ctx.model_id&&
    geometry.primitives.size()==geometry.primitive_poses.size()&&geometry.meshes.size()==geometry.mesh_poses.size()&&
    geometry.planes.size()==geometry.plane_poses.size()&&geometry.subframe_names.size()==geometry.subframe_poses.size()&&
    (!geometry.primitives.empty()||!geometry.meshes.empty())&&geometry.planes.empty(),"REGISTERED_CAD_GEOMETRY_REQUIRED");
  const auto base_camera=transform(r.base_from_camera.transform);
  const auto base_grasp=base_camera*transform(c.grasp_pose.pose);
  const auto base_tcp=base_grasp*transform(mapping.grasp_from_tcp);
  auto pre=base_tcp;
  pre.setOrigin(pre.getOrigin()-base_grasp.getBasis().getColumn(0)*r.approach_m);
  auto lift=base_tcp;lift.setOrigin(lift.getOrigin()+tf2::Vector3(0,0,r.lift_m));
  Plan::Goal g;g.operation="PICK";g.object_id=ctx.object_instance;
  // Distinct candidate context also distinguishes the MTC revalidation cache.
  g.context_id=r.context_id+":"+c.candidate_id;g.scene=r.scene;
  g.target.header=r.object_cloud.header;g.target.header.frame_id="astribot_torso_base";
  g.target.pose=pose_message(base_tcp);g.pre_target=g.target;g.pre_target.pose=pose_message(pre);
  g.exit_targets={g.target};g.exit_targets.front().pose=pose_message(lift);
  g.touch_links=r.touch_links;g.grasp_width_m=mapping.physical_object_width_m;
  g.timeout_s=std::min(r.planning_timeout_s,double(ns(r.valid_until)-now)*1e-9);
  auto object=geometry;object.header=g.target.header;object.pose=geometry_msgs::msg::Pose();
  object.pose.orientation.w=1;object.operation=object.ADD;
  const auto base_geometry=base_camera*transform(o.pose.pose)*transform(geometry.pose);
  for(auto *poses:{&object.primitive_poses,&object.mesh_poses,&object.subframe_poses})
    for(auto &p:*poses)p=pose_message(base_geometry*transform(p));
  for(auto &item:g.scene.world.collision_objects)if(item.id==ctx.object_instance)item=object;
  if(mapping_used)*mapping_used=mapping;
  return g;
}

void check_plan(const Plan::Goal &g,const Plan::Result &r) {
  require(r.success&&r.context_id==g.context_id,"MTC_RESULT_CONTEXT_OR_STATUS");
  static const std::vector<std::pair<std::string,std::string>> expected={
    {"PREGRASP","ARM"},{"GRASP_APPROACH","ARM"},{"GRASP_CONFIRM","GRIPPER"},
    {"ATTACH_CONFIRM","ATTACH"},{"LIFT","ARM"},{"TRANSPORT_POSTURE","ARM"}};
  require(r.stages.size()==expected.size(),"INCOMPLETE_MTC_PICK");
  for(size_t i=0;i<expected.size();++i)
    require(r.stages[i].stage_id==expected[i].first&&r.stages[i].kind==expected[i].second,
      "MTC_PICK_STAGE_ORDER");
}
} // namespace astribot::perception_planning
