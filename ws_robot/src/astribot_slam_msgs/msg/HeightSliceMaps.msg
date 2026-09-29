# Atomic height maps. Unknown (-1) is NOT observed free.
# evidence_kind identifies occupied-only archives or complete simulation collision geometry.
# header.stamp is the source snapshot inspection time; profile_revision is the
# SHA256 of the exact shared height_slices.yaml bytes.
# Heights are above a horizontal ground plane at ground_z in header.frame_id.
# grids[i] corresponds to [height_edges[i], height_edges[i+1]).
std_msgs/Header header
string map_revision
string profile_revision
float64 ground_z
string ground_reference
float64[] height_edges
string[] layer_names
nav_msgs/OccupancyGrid[] grids
uint64[] point_counts
uint64 below_band_points
uint64 above_band_points
uint64 nonfinite_points
string evidence_kind
