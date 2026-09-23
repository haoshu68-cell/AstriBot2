# Full physical/execution-environment inventory, never a PlanningScene hypothesis.
uint8 UNKNOWN=0
uint8 EMPTY=1
uint8 ATTACHED=2
uint8 TRANSITION=3
string environment
string session_id
string source_id
string source_epoch
uint64 clock_epoch
uint64 sequence
uint64 revision
builtin_interfaces/Time observed_at
builtin_interfaces/Time valid_until
bool full_inventory
uint8 status
string transaction_id
moveit_msgs/AttachedCollisionObject[] objects
