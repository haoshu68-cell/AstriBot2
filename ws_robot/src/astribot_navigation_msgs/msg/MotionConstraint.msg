# A short lease restricts an existing command; it never creates motion authority.
builtin_interfaces/Time stamp
uint64 epoch
uint64 sequence
float64 lease_s
bool hold
float64 max_linear_speed
float64 max_angular_speed
uint8 planning
string reason
# When true, translation is forbidden and a matching CorridorAlignment is required.
bool alignment_required
# Mutually exclusive with alignment; rotation is forbidden while centering.
bool centering_required
# During straight-passage tracking, a matching heading request is required.
bool corridor_tracking_required
