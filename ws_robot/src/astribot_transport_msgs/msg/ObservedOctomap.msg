# One actual integration, with the exact full tree copied under its write lock.
# header is the original cloud acquisition stamp/frame, not a republishing stamp.
std_msgs/Header header
builtin_interfaces/Time callback_stamp
builtin_interfaces/Time integrated_stamp
string source_id
# Epoch changes on process restart; revisions order integrations in that process.
string map_epoch
uint64 map_revision
string sensor_frame
geometry_msgs/Transform cloud_to_map
geometry_msgs/Transform sensor_to_map
octomap_msgs/Octomap octomap
# Triples of full-depth OcTree keys traversed by this frame's measured rays AND
# free in the above snapshot. No self-filter/model-clear cells. Absent keys are
# not evidence of fresh free space. This does not certify gaps between rays,
# physical calibration, sensor accuracy or coverage of an entire voxel volume.
uint16[] ray_free_keys
float64 processing_seconds
