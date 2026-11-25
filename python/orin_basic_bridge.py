"""Plain ROS Twist relay from Orin to the existing PC SDK owner."""
import json
import math
import signal
import time

import rclpy
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool
from auto_base_enable import AutoBaseEnable, fresh_ms

REMOTE = '/cocomatrix/local_planner'
LOCAL = '/astribot/local_planner'


class BasicBridge:
    def __init__(self, source_domain=25, output_domain=0):
        self.contexts, self.nodes, self.executors = [], [], []
        for domain, suffix in ((source_domain, 'source'), (output_domain, 'output')):
            context = Context()
            rclpy.init(args=[], context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
            node = Node('astribot_basic_' + suffix, context=context, use_global_arguments=False)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self.contexts.append(context); self.nodes.append(node); self.executors.append(executor)
        self.source, self.output = self.nodes
        self.owner, self.planner = {}, {}
        self.owner_received = self.planner_received = -math.inf
        self.command = self.output.create_publisher(Twist, '/cmd_vel',
            QoSProfile(depth=1, lifespan=Duration(seconds=.2)))
        self.owner_pub = self.source.create_publisher(String, REMOTE+'/owner_status',
            QoSProfile(depth=1, lifespan=Duration(seconds=.5)))
        self.goal_pub = self.source.create_publisher(PoseStamped, REMOTE+'/goal_pose',
            QoSProfile(depth=1, lifespan=Duration(seconds=2.)))
        self.cancel_pub = self.source.create_publisher(Empty, REMOTE+'/goal_cancel', 1)
        self.status_pub = self.output.create_publisher(String, LOCAL+'/status', 1)
        self.path_pub = self.output.create_publisher(Path, LOCAL+'/path_map', 1)
        self.source.create_subscription(Path, REMOTE+'/path_map', self.path_pub.publish, 1)
        self.source.create_subscription(String, REMOTE+'/status', self.planner_status, 1)
        self.source.create_subscription(Twist, REMOTE+'/cmd_vel', self.receive_command, 1)
        self.output.create_subscription(String, '/astribot/base/status', self.owner_status, 1)
        self.output.create_subscription(PoseStamped, '/astribot_viz/goal_pose', self.goal, 1)
        self.output.create_subscription(Empty, '/astribot_viz/goal_cancel', self.cancel, 1)
        self.enable = self.output.create_client(SetBool, '/astribot/base/enable')
        self.auto_enable = AutoBaseEnable(lambda: self.enable.call_async(SetBool.Request(data=True)),
            self.output.get_logger().info, started=time.monotonic(), enable_on_start=True)

    @staticmethod
    def decode(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {}
        except ValueError:
            return {}

    def unique(self):
        # Discovery may recover after a coordinator restart; no identity lease.
        return all(node.count_publishers(topic) == 1 for node, topic in (
            (self.source, REMOTE+'/cmd_vel'), (self.source, REMOTE+'/owner_status'),
            (self.output, '/cmd_vel'), (self.output, '/astribot/base/status')))

    def owner_status(self, message):
        self.owner, self.owner_received = self.decode(message), time.monotonic()
        if self.output.count_publishers('/astribot/base/status') == 1:
            self.owner_pub.publish(message)

    def planner_status(self, message):
        self.planner, self.planner_received = self.decode(message), time.monotonic()
        self.status_pub.publish(message)

    def receive_command(self, message):
        if not self.unique():
            return
        v, w = message.linear, message.angular
        if (not all(math.isfinite(x) for x in (v.x, v.y, v.z, w.x, w.y, w.z))
                or (v.z, w.x, w.y) != (0., 0., 0.)
                or math.hypot(v.x, v.y) > .100001 or abs(w.z) > .200001):
            return
        if any((v.x, v.y, w.z)):
            s = self.owner
            if (time.monotonic()-self.owner_received > .5 or s.get('armed') is not True
                    or s.get('mode') != 'MODEL' or s.get('selected_source') != 'auto'
                    or s.get('selected_topic') != '/cmd_vel' or s.get('stop_pending') is not False
                    or s.get('hardware_writes') is not True or s.get('feedback_source') != 'hardware'
                    or not fresh_ms(s.get('feedback_age_ms'), 50.)):
                return
        # Forward actual samples only. Loss of stream lets the owner's 200 ms
        # watchdog stop the base; never repeat the last velocity on a timer.
        self.command.publish(message)

    def goal(self, message):
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        age = (self.output.get_clock().now().nanoseconds-stamp)/1e9
        if self.output.count_publishers('/astribot_viz/goal_pose') == 1 and -.05 <= age <= 2.:
            self.goal_pub.publish(message)

    def cancel(self, message):
        self.cancel_pub.publish(message)
        self.command.publish(Twist())

    def spin_once(self):
        for executor in self.executors:
            executor.spin_once(timeout_sec=.002)
        now = time.monotonic()
        idle = (now-self.planner_received <= .5 and self.planner.get('dispatch') is True
                and self.planner.get('active') is False and self.unique()
                and self.source.count_publishers(REMOTE+'/status') == 1)
        self.auto_enable.tick(now, status=self.owner, age=now-self.owner_received,
            publishers=self.output.count_publishers('/cmd_vel'), idle=idle,
            service_ready=self.enable.service_is_ready())

    def close(self):
        for executor in self.executors: executor.shutdown()
        for node in self.nodes: node.destroy_node()
        for context in self.contexts: context.try_shutdown()


def main():
    bridge = BasicBridge()
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, stop)
    try:
        while not stopped: bridge.spin_once()
    finally:
        bridge.close()


if __name__ == '__main__':
    main()
