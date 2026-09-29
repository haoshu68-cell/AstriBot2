std_msgs/Header header
builtin_interfaces/Time published_at
builtin_interfaces/Time valid_until
string source_id
uint64 sequence
uint64 clock_epoch
string model_revision
string attachment_revision
sensor_msgs/JointState joints
builtin_interfaces/Time[] joint_source_stamps
float64[] joint_position_error_bounds
bool complete
bool attachment_state_confirmed
geometry_msgs/Polygon physical_footprint
# Includes model padding and bounded joint hold error, not environmental clearance.
geometry_msgs/Polygon reserved_footprint
EnvelopeSlice[] height_slices
# SHA256 of the shared height_slices.yaml file. Heights above are in header.frame_id.
string height_profile_revision
float64 ground_in_base_m
float64 height_m
string[] attachment_ids
string reason
