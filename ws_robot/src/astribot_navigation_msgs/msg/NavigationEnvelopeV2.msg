uint8 HOLD=0
uint8 FIXED_POSTURE=1
uint8 RESERVED_ARM_MOTION=2
std_msgs/Header header
builtin_interfaces/Time valid_until
string coordinator_session_id
string request_id
string hold_id
uint64 epoch
uint64 clock_epoch
uint64 reference_state_sequence
uint64 source_state_sequence
string model_revision
string attachment_revision
uint8 mode
RobotEnvelope limits
geometry_msgs/Polygon reserved_footprint
# reserved_footprint plus clearance_m; Nav2 footprint_padding must be zero.
geometry_msgs/Polygon installed_footprint
string installed_geometry_hash
# Fixed-posture height layers include clearance exactly once; empty layers retain identity.
string height_geometry_hash
EnvelopeSlice[] height_slices
string height_profile_revision
float64 ground_in_base_m
float64 clearance_m
bool navigation_allowed
string reason
