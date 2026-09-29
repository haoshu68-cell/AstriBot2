# Processing capability for one RGB-D projector, separate from CameraHealth.
# Consumers requiring point clouds must check both health sources and expiry.
# Parent is the unique publisher; backend never silently falls back.
std_msgs/Header header
string camera_id
string backend
string processing_epoch
uint64 worker_epoch
int64 worker_pid
string state
string reason_code
bool valid
builtin_interfaces/Time capture_stamp
builtin_interfaces/Time epoch_first_capture_stamp
builtin_interfaces/Time published_stamp
builtin_interfaces/Time valid_until
uint64 sequence
uint64 processed
uint64 discarded
uint64 errors
uint32 restarts
uint32 pending_depth
