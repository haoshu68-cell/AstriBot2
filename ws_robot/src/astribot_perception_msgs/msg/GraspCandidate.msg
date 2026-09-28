std_msgs/Header header
string source_epoch
string candidate_id
string object_id
string arm_id
string camera_id
# Pose of model_name's native grasp frame in header.frame_id, not robot TCP.
# A registered model-grasp-to-end-effector transform is required before planning.
geometry_msgs/PoseStamped grasp_pose
# Bounded ranking value, not a calibrated probability. See score_semantics.
float64 score
float64 raw_model_score
string score_semantics
float64 gripper_width_m
float64 gripper_height_m
float64 gripper_depth_m
bool collision_checked
bool collision_free
bool geometry_valid
string reason_code
string model_name
string model_revision
uint64 calibration_revision
uint64 planning_scene_revision
uint64 envelope_epoch
builtin_interfaces/Time valid_until
