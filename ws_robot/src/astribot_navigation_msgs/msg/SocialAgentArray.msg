# stamp is the measurement/prediction epoch, never the reception time.
std_msgs/Header header
string source_id
uint64 source_epoch
uint64 sequence
uint8 PERCEPTION=0
uint8 SIMULATION_TRUTH=1
uint8 provenance
# Invalid/expired input is NOT evidence of an empty, clear environment.
bool valid
string reason
astribot_navigation_msgs/SocialAgent[] agents
