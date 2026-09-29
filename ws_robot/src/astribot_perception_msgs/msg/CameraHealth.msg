# Runtime health for one camera stream.  A valid observation must satisfy
# timestamp, frame, calibration and freshness checks together.
std_msgs/Header header
string camera_id
string source_epoch
string frame_id
string state
string reason_code
float64 age_sec
float64 frequency_hz
float64 max_interval_sec
float64 sync_skew_sec
uint32 drop_count
uint64 sequence
uint64 calibration_revision
builtin_interfaces/Time capture_stamp
builtin_interfaces/Time valid_until
bool valid
