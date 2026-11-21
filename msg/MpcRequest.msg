# All geometry is planar and expressed in header.frame_id (chassis).
# header.stamp is the front-cloud source stamp; sent_at is local ROS PC time.
std_msgs/Header header
builtin_interfaces/Time sent_at
uint64 epoch
geometry_msgs/Point goal
geometry_msgs/Vector3 previous_velocity
geometry_msgs/Point[] obstacles
