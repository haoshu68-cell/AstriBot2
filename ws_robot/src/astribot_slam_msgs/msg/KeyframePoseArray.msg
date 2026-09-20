# Full snapshot after pose-graph correction or final optimization.
KeyframePoseUpdate[] updates

# Final optimization is complete; all referenced submaps must arrive before export.
bool is_final

# Final-save destination. Empty means no export. Only meaningful when is_final.
string save_dir
string map_name
