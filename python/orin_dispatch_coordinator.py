#!/usr/bin/env python3
"""Existing XPolicy leased command protocol around the Python MPC; no SDK."""
import json
import os
from pathlib import Path
import sys
import time
import uuid

HERE = Path(__file__).resolve().parent
sys.path.insert(0, os.environ.get('PLANNER_TRANSPORT_DIR',
    str(HERE.parents[1] / 'astribot_planner_bringup')))
from local_planner_node import LocalPlanner, PREFIX, choose_collision_source, validate_navigation
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from planner_transport import REMOTE, owner_ready


class PublisherAdapter:
    def __init__(self, publish):
        self.publish = publish


class RemotePlanner(LocalPlanner):
    def __init__(self, **kwargs):
        self.transport_session = uuid.uuid4().hex
        self.transport_sequence = 0
        self.transport_goal = None
        super().__init__(**kwargs)
        # The existing owner and enable service are on PC domain 0. The PC
        # relay maintains enable there; never create an Orin SDK writer.
        self.auto_base_enable = None

    def input_readiness(self, now):
        validate_navigation(self.nav.fields(now))
        if not self.ignore_lidar:
            choose_collision_source(self.collision_inputs, now, self.nav.stamp,
                check_publishers=self.collision_publishers, mode=self.collision_lidar)
        if len(self.get_publishers_info_by_topic(PREFIX + '/mpc_result')) != 1:
            raise ValueError('MPC backend publisher missing or duplicated')
        return True

    def create_publisher(self, cls, topic, *args, **kwargs):
        raw = super().create_publisher(cls, topic, *args, **kwargs)
        if cls is Twist and topic == '/cmd_vel':
            envelopes = super().create_publisher(String, REMOTE + '/command', 1)

            def command(message):
                raw.publish(message)
                self.transport_sequence += 1
                owner = self.owner_status
                envelopes.publish(String(data=json.dumps(dict(schema=1,
                    session=self.transport_session, sequence=self.transport_sequence,
                    goal=self.transport_goal if self.session.goal is not None else None,
                    lease=owner.get('transport_lease'), generation=owner.get('generation'),
                    command=[message.linear.x, message.linear.y, message.angular.z]))))
            return PublisherAdapter(command)
        if cls is String and topic == PREFIX + '/status':
            def status(message):
                data = json.loads(message.data)
                try:
                    now = time.monotonic()
                    data['input_ready'] = self.input_readiness(now)
                    reason = ('owner ready; waiting for an explicit goal'
                              if owner_ready(self.owner_status) and 0 <= now-self.owner_received <= .2
                              else 'waiting for desktop automatic owner')
                except ValueError as exc:
                    data['input_ready'], reason = False, str(exc)
                data.update(readiness_reason=reason, host='orin', service='manual',
                    output_topic=REMOTE+'/command', configured_collision_lidar=self.collision_lidar)
                raw.publish(String(data=json.dumps(data)))
            return PublisherAdapter(status)
        return raw

    def create_subscription(self, cls, topic, callback, *args, **kwargs):
        if cls is PoseStamped and topic == '/astribot_viz/goal_pose':
            def goal(message):
                try:
                    data = json.loads(message.data)
                    if (not isinstance(data, dict) or not isinstance(data.get('lease'), str)
                            or not isinstance(data.get('goal'), str) or len(data['goal']) != 32
                            or not 0 <= time.monotonic()-self.owner_received <= .2):
                        raise ValueError('expired goal lease')
                    pose = PoseStamped()
                    pose.header.frame_id = data['frame']
                    # PC validates original time and requires a goal-ID acknowledgment
                    # within 200 ms. Different DDS writers may arrive out of order;
                    # only the PC clock decides whether this goal may command motion.
                    pose.header.stamp = self.get_clock().now().to_msg()
                    p, q = data['position'], data['orientation']
                    if len(p) != 3 or len(q) != 4:
                        raise ValueError('invalid goal dimensions')
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, p)
                    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
                    callback(pose)
                    self.transport_goal = data['goal'] if self.session.goal is not None else None
                except (ValueError, TypeError, KeyError) as exc:
                    self.cancel(reason='remote goal rejected: ' + str(exc))
            return super().create_subscription(String, REMOTE + '/goal', goal, *args, **kwargs)
        return super().create_subscription(cls, topic, callback, *args, **kwargs)


def main():
    rclpy.init()
    node = RemotePlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.cancel(reason='Orin dispatch planner shutting down')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
