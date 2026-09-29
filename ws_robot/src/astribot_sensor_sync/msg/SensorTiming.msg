# Sidecar from the source driver; capture stamp remains untouched in sensor messages.
# source_id identifies a STREAM, e.g. head_rgbd/color or head_rgbd/depth.
string source_id
string source_epoch
string group
string trigger_epoch
string clock_epoch
uint64 frame_sequence
uint64 trigger_sequence
int64 capture_stamp_ns
int64 clock_uncertainty_ns
bool clock_locked
# True only with device metadata/counter association, not nearest ROS timestamp.
bool hardware_associated
bool simulated
