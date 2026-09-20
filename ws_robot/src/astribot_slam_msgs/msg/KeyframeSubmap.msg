uint32 session_id
uint32 keyframe_id

# map <- keyframe IMU transform. This is not the chassis-center pose.
# Later KeyframePoseUpdate messages replace this placement without changing cloud.
geometry_msgs/Pose pose

# Points in the keyframe IMU frame, cropped using the capture-time world height ROI.
# Replayed when registering a loaded session; consumers must upsert by the two IDs.
sensor_msgs/PointCloud2 cloud
