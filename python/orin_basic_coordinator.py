"""Action goals and Twist output; PC handles SDK enable."""
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from local_planner_node import LocalPlanner
from goal_action_server import GoalActionServer


class BasicPlanner(LocalPlanner):
    # The PC owns hardware admission; plain Twist planning needs navigation only.
    requires_owner_status = False

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.auto_base_enable = None  # The enable service lives on PC domain 0.
        self.goal_actions = GoalActionServer(self)

    def create_publisher(self, cls, topic, qos_profile, **kwargs):
        if cls is Twist and topic == '/cmd_vel':
            qos_profile = QoSProfile(depth=1, lifespan=Duration(seconds=.2))
        return super().create_publisher(cls, topic, qos_profile, **kwargs)

    def create_subscription(self, cls, topic, callback, *args, **kwargs):
        if topic in ('/astribot_viz/goal_pose', '/astribot_viz/goal_cancel'):
            # Action admission is the sole goal/cancel path for this subclass.
            return None
        return super().create_subscription(cls, topic, callback, *args, **kwargs)


def main():
    rclpy.init()
    node = BasicPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.goal_actions.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
