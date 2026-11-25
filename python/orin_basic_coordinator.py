"""Standard PoseStamped goals and Twist output; PC handles SDK enable."""
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Twist
from local_planner_node import LocalPlanner


class BasicPlanner(LocalPlanner):
    # The PC owns hardware admission; plain Twist planning needs navigation only.
    requires_owner_status = False

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.auto_base_enable = None  # The enable service lives on PC domain 0.

    def create_publisher(self, cls, topic, qos_profile, **kwargs):
        if cls is Twist and topic == '/cmd_vel':
            qos_profile = QoSProfile(depth=1, lifespan=Duration(seconds=.2))
        return super().create_publisher(cls, topic, qos_profile, **kwargs)

    def create_subscription(self, cls, topic, callback, *args, **kwargs):
        if cls is PoseStamped and topic == '/astribot_viz/goal_pose':
            def local_goal(message):
                # PC validates the RViz stamp; DDS goal lifespan bounds transport.
                # Do not compare the independent PC and Orin wall clocks.
                message.header.stamp = self.get_clock().now().to_msg()
                callback(message)
            return super().create_subscription(cls, topic, local_goal, *args, **kwargs)
        return super().create_subscription(cls, topic, callback, *args, **kwargs)


def main():
    rclpy.init()
    node = BasicPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.context.ok():
            node.cancel(reason='Orin basic planner shutting down')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
