"""ARM64 dispatch integration with synthetic odometry/owner on localhost domain 213.

Run after sourcing the Orin planner environment/install, with bringup path argv[1].
Never uses the vendor domain or a hardware writer.
"""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

BRINGUP = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(BRINGUP))
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty, String
from fast_lio_display_tf import imu_from_sdk_chassis, set_pose
from planner_transport import CommandLease, REMOTE


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '213'
    log = open('/tmp/astribot-orin-planner-smoke.log', 'w')
    launch = Path(__file__).resolve().parents[1]/'launch/astribot_orin_dispatch.launch.py'
    basic = '--basic-control' in sys.argv[2:]
    process = subprocess.Popen(['ros2', 'launch', str(launch), 'basic_control:=' + str(basic).lower()],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[], domain_id=213)
    node = rclpy.create_node('synthetic_planner_inputs')
    guard, accepted, statuses, rejected = CommandLease(), [], [], []
    owner = node.create_publisher(String, REMOTE+'/owner_status', 1)
    goal = node.create_publisher(PoseStamped if basic else String,
        REMOTE+('/goal_pose' if basic else '/goal'), 1)
    cancel = node.create_publisher(Empty, REMOTE+'/goal_cancel', 1)
    odom = node.create_publisher(Odometry, '/cocomatrix/fast_lio/front/odometry', 1)
    state = dict(generation=3, armed=True, mode='MODEL', hardware_writes=True,
        feedback_source='hardware', feedback_age_ms=1, stop_pending=False,
        selected_source='auto', selected_topic='/cmd_vel')

    def command(message):
        try:
            accepted.append(guard.accept(json.loads(message.data), time.monotonic()))
        except ValueError as exc:
            rejected.append(str(exc))

    if basic:
        node.create_subscription(Twist, REMOTE+'/cmd_vel',
            lambda m: accepted.append([m.linear.x, m.linear.y, m.angular.z]), 10)
    else:
        node.create_subscription(String, REMOTE+'/command', command, 10)
    node.create_subscription(String, REMOTE+'/status', lambda m: statuses.append(json.loads(m.data)), 10)
    next_publish = 0.

    def pump(seconds):
        nonlocal next_publish
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            assert process.poll() is None, Path(log.name).read_text()
            now = time.monotonic()
            if now >= next_publish:
                stamp = node.get_clock().now().to_msg()
                pose = Odometry()
                pose.header.frame_id, pose.child_frame_id = 'map', 'body'
                pose.header.stamp = stamp
                set_pose(pose.pose.pose, np.linalg.inv(imu_from_sdk_chassis()))
                odom.publish(pose)
                owner.publish(String(data=json.dumps(state if basic else guard.issue(state, now))))
                next_publish = now+.05
            rclpy.spin_once(node, timeout_sec=.002)

    def send_goal(goal_id):
        if basic:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            # Simulate an independent PC clock; bridge already checked its age.
            pose.header.stamp.sec = 123
            pose.pose.position.x = .3
            pose.pose.orientation.w = 1.
            goal.publish(pose)
            return
        goal.publish(String(data=json.dumps(dict(lease=guard.latest['transport_lease'],
            goal=goal_id, frame='map', position=[.3, 0., 0.], orientation=[0., 0., 0., 1.]))))

    try:
        pump(3)
        assert statuses and (basic or statuses[-1]['input_ready']), statuses[-1:]
        assert statuses[-1]['ignore_lidar'] and not statuses[-1]['collision_checks_enabled']
        assert node.count_subscribers('/livox/lidar_front') == 0
        assert accepted and not any(any(v) for v in accepted)
        send_goal(guard.register_goal(time.monotonic()))
        pump(1.)
        assert any(any(v) for v in accepted), (statuses[-3:], rejected[-8:])
        guard.cancel_goal(); cancel.publish(Empty())
        accepted.clear(); pump(.3)
        assert not any(any(v) for v in accepted), 'Cancelled goal forwarded'
        if basic:
            send_goal(None); accepted.clear(); pump(.6)
            assert any(any(v) for v in accepted), 'Second basic goal after cancel was not accepted'
            cancel.publish(Empty()); accepted.clear(); pump(.3)
            assert not any(any(v) for v in accepted)
        else:
            expired = guard.register_goal(time.monotonic())
            pump(.3)
            send_goal(expired)
            accepted.clear(); pump(.5)
            assert not any(any(v) for v in accepted), 'Delayed goal resumed under fresh leases'
            assert any('goal acknowledgment expired' in r for r in rejected)
        assert node.count_publishers('/cmd_vel') == 0
        print('PASS: ARM64 MPC + ' + ('basic Twist adapter, next goal after cancel' if basic else 'leased adapter, delayed goal rejection')
              + ' + synthetic odometry without LiDAR, nonzero proposals, cancel; localhost domain 213 only')
    finally:
        node.destroy_node(); rclpy.try_shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait()
        log.close()


if __name__ == '__main__':
    main()
