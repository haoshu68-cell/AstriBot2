# Control lease metadata, NOT an observation or an execution permit.
std_msgs/Header header
string camera_id
string owner_id
string execution_id
string session_token
bool active
# First accepted ROS capture time in this lease generation; renewals preserve it.
builtin_interfaces/Time activated_at
builtin_interfaces/Time valid_until
string reason_code
