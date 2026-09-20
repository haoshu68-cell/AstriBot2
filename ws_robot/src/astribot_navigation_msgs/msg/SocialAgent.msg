# Identity is (array.source_id, array.source_epoch, track_id); never recycle an ID within an epoch.
string track_id
builtin_interfaces/Time last_observed
uint8 OBSERVED=0
uint8 OCCLUDED=1
uint8 LOST=2
uint8 tracking_state
# All positions, orientations and velocities use the array header frame at header.stamp.
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity
bool velocity_valid
float64 radius
float64 person_confidence
float64 heading_confidence
string group_id
float64 group_confidence
