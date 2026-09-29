# Straight-passage manoeuvre, bound to both clocks, active path and starting anchor.
builtin_interfaces/Time stamp
float64 lease_s
nav_msgs/Path reference_path
geometry_msgs/PoseStamped anchor
bool centering_required
geometry_msgs/PoseStamped target
bool tracking_required
# Fixed-envelope passage ownership. NORMAL explicitly releases ownership only
# after the entire envelope clears the exit. WAIT/HOLD retain the current phase.
string phase
string corridor_id
float64 centering_tolerance_m
