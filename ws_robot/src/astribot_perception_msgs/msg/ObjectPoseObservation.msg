std_msgs/Header header
string object_id
geometry_msgs/PoseWithCovariance pose
string source_camera_id
string source_epoch
string source_model
string model_revision
uint64 calibration_revision
# Zero means no task planning/envelope context (e.g. a position-only detector).
uint64 planning_scene_revision
uint64 envelope_epoch
string reason_code
float64 quality
builtin_interfaces/Time valid_until
# A depth centroid never establishes object orientation. Consumers must check both flags.
bool position_valid
bool orientation_valid
