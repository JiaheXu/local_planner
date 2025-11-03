#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class FakeInputs(Node):
    def __init__(self):
        super().__init__('fake_inputs')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)
        self.mode_pub = self.create_publisher(Int32, '/behavior_mode', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        # self.timer_odom = self.create_timer(0.2, self.publish_odom)       # 5 Hz
        self.timer_mode = self.create_timer(0.2, self.publish_mode)       # 5 Hz
        # self.timer_wp = self.create_timer(2.0, self.publish_waypoint)     # 0.5 Hz

        self.get_logger().info("FakeInputs started: publishing /odom, /way_point, and TFs.")

    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        # ---- Odometry message ----
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"        # odometry frame
        odom.child_frame_id = "base_link"    # robot body frame

        # Robot fixed at origin
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.odom_pub.publish(odom)

        # ---- TF: map -> odom (identity) ----
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.tf_broadcaster.sendTransform(t_map_odom)

        # ---- TF: odom -> base_link (identity) ----
        t_odom_bl = TransformStamped()
        t_odom_bl.header.stamp = now
        t_odom_bl.header.frame_id = "odom"
        t_odom_bl.child_frame_id = "base_link"
        t_odom_bl.transform.translation.x = 0.0
        t_odom_bl.transform.translation.y = 0.0
        t_odom_bl.transform.translation.z = 0.0
        t_odom_bl.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.tf_broadcaster.sendTransform(t_odom_bl)

        self.get_logger().info("Published /odom and TFs: map->odom, odom->base_link.")

    def publish_waypoint(self):
        # ---- Fake Waypoint ----
        wp = PointStamped()
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.header.frame_id = "map"   # global frame
        wp.point.x = 3.0
        wp.point.y = 0.0
        wp.point.z = 0.0

        self.waypoint_pub.publish(wp)
        self.get_logger().info("Published /way_point (3,0 in map frame).")

    def publish_mode(self):
        # ---- Fake Waypoint ----
        msg = Int32()
        msg.data = 2
        self.mode_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = FakeInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

