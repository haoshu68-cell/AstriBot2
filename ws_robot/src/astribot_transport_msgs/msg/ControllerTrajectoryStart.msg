# One JTC first-sampling cycle, not action acceptance or feedback reception.
# BOUND identifies the immutable FJT command by its action UUID. point_before
# uses joint_names order; the executor retains the original command unchanged.
# A callback concurrent with that cycle makes the UUID binding UNKNOWN.
uint8 UNKNOWN=0
uint8 BOUND=1
uint8 binding
uint8[16] goal_id
builtin_interfaces/Time sample_stamp
string[] joint_names
trajectory_msgs/JointTrajectoryPoint point_before
