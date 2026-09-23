# Read-only reconciliation result; persisted observations do not renew leases.
astribot_payload_msgs/AttachmentObservation observation
string ledger_epoch
uint64 ledger_revision
string attachment_revision
string geometry_digest
builtin_interfaces/Time published_at
builtin_interfaces/Time valid_until
bool confirmed
string reason
