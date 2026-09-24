# Atomic occupied-endpoint snapshot. Unknown (-1) is NOT observed free.
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
