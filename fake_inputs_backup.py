#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Quaternion


class FakeInputs(Node):
    def __init__(self):
        super().__init__('fake_inputs')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)

        # Timer to publish at 1Hz
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.timer2 = self.create_timer(2, self.timer_callback2)

        self.get_logger().info("FakeInputs started (publishing /odom and /way_point at 1Hz).")

    def timer_callback(self):
        # ---- Fake Odom (0,0,0 pose, identity orientation) ----
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "odom"

        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.odom_pub.publish(odom)
        self.get_logger().info("Published fake /odom (0,0,0).")

    def timer_callback2(self):

        # ---- Fake Waypoint (3,0) ----
        wp = PointStamped()
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.header.frame_id = "map"   # or "odom", depending on your planner
        wp.point.x = 3.0
        wp.point.y = 0.0
        wp.point.z = 0.0

        self.waypoint_pub.publish(wp)

        self.get_logger().info("/way_point (3,0).")


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

