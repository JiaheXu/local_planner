# All geometry is planar and expressed in header.frame_id (chassis).
# header.stamp is a strictly increasing request identity, echoed by MpcResult.
# Python uses local ROS time; the native coordinator uses a source stamp.
# sent_at is local ROS PC time. Sensor freshness is checked by the coordinator.
std_msgs/Header header
builtin_interfaces/Time sent_at
uint64 epoch
geometry_msgs/Point goal
geometry_msgs/Vector3 previous_velocity
geometry_msgs/Point[] obstacles
# Explicit no-obstacle pose mode; legacy requests default to XY-only.
bool plan_yaw
float64 goal_yaw
float64 previous_yaw_rate
