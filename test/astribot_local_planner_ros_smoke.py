"""Real MPC/library + adapter, synthetic observations only, isolated domain 227."""
import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Empty
from ament_index_python.packages import get_package_prefix, get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
SHARE = Path(get_package_share_directory('local_planner'))
INSTALL = Path(get_package_prefix('local_planner'))
sys.path.insert(0, str(ROOT / 'python'))
from local_planner_core import front_mount
from local_planner_node import LocalPlanner, PREFIX, FRONT, FRONT_IMU, REAR_IMU, ODOM


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '227'
    parser = argparse.ArgumentParser()
    parser.add_argument('--planner', choices=['mpc', 'library'], default='mpc')
    backend = parser.parse_args().planner
    binary = INSTALL / 'lib/local_planner/localPlanner'
    arguments = [str(binary), '--ros-args', '--params-file', str(SHARE / 'config/astribot.yaml'),
                 '-p', 'pathFolder:=' + str(SHARE / 'paths'),
                 '-p', 'planner_backend:=' + backend]
    for src, dst in [('/scan', '/scan'), ('/way_point', '/way_point'),
                     ('/planner_cancel', '/cancel'), ('/path', '/path'), ('/free_paths', '/free_paths'),
                     ('/mpc_request', '/mpc_request'), ('/mpc_result', '/mpc_result')]:
        arguments.extend(['-r', src + ':=' + PREFIX + dst])
    log = open('/tmp/astribot-local-planner-smoke.log', 'w')
    child = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    source = rclpy.create_node('synthetic_local_planner_inputs')
    adapter = LocalPlanner(parameter_overrides=[Parameter('planner_backend', value=backend)])
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(adapter)
    odom_pub = source.create_publisher(Odometry, ODOM, qos_profile_sensor_data)
    cloud_pub = source.create_publisher(PointCloud2, FRONT, qos_profile_sensor_data)
    imu_pubs = [source.create_publisher(Imu, topic, qos_profile_sensor_data)
                for topic in (FRONT_IMU, REAR_IMU)]
    goal_pub = source.create_publisher(PoseStamped, '/astribot_viz/goal_pose', 1)
    cancel_pub = source.create_publisher(Empty, '/astribot_viz/goal_cancel', 1)
    commands = []
    lateral = [0.]
    def command(msg):
        commands.append((msg.linear.x, msg.angular.z))
        lateral.append(msg.linear.y)
    source.create_subscription(Twist, PREFIX + '/cmd_vel_shadow',
                               command, 1)
    state = dict(x=0., y=0., yaw=0., obstacle=False, cloud=True, odom=True, move=False, invalid=False)
    phase_commands = []
    last_imu, last_cloud = 0., 0.
    mount = front_mount()

    def stamp(target, ns):
        target.sec, target.nanosec = divmod(ns, 1_000_000_000)

    def drive(seconds, until=lambda: False, planner_alive=True):
        nonlocal last_imu, last_cloud
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert not planner_alive or child.poll() is None, Path(log.name).read_text()
            now = time.monotonic()
            ns = source.get_clock().now().nanoseconds + 37_000_000_000
            if now - last_imu >= .005:
                imu = Imu()
                stamp(imu.header.stamp, ns)
                for pub in imu_pubs:
                    pub.publish(imu)
                last_imu = now
            if now - last_cloud >= .1:
                if state['odom']:
                    if state['move'] and commands:
                        vx, wz = commands[-1]
                        vy = lateral[-1]
                        state['x'] += .1 * (vx * math.cos(state['yaw']) - vy * math.sin(state['yaw']))
                        state['y'] += .1 * (vx * math.sin(state['yaw']) + vy * math.cos(state['yaw']))
                        state['yaw'] += .1 * wz
                        phase_commands.append((adapter.sequence.phase, vx, wz))
                        if adapter.sequence.phase in ('rotate_to_goal', 'rotate_to_yaw'):
                            assert vy == 0., 'rotation must not translate sideways'
                    odom = Odometry()
                    odom.header.frame_id, odom.child_frame_id = 'map', 'astribot_viz_chassis'
                    odom.pose.pose.orientation.z = math.sin(state['yaw'] / 2)
                    odom.pose.pose.orientation.w = math.cos(state['yaw'] / 2)
                    odom.pose.pose.position.x = state['x']
                    odom.pose.pose.position.y = state['y']
                    stamp(odom.header.stamp, ns - 30_000_000)
                    odom_pub.publish(odom)
                if state['cloud']:
                    base = np.array([[2., 0., 0.], [2., 1., 0.], [2., -1., 0.]])
                    if state['obstacle']:
                        base = np.vstack([base, [[.45, -.1, .2], [.45, 0., .2], [.45, .1, .2]]])
                    xyz = ((base - mount[:3, 3]) @ mount[:3, :3]).astype('<f4')
                    if state['invalid']:
                        xyz[:] = 0.
                    cloud = PointCloud2()
                    cloud.header.frame_id = 'livox_frame'
                    stamp(cloud.header.stamp, ns - 100_000_000)
                    cloud.height, cloud.width, cloud.point_step = 1, len(xyz), 12
                    cloud.row_step = len(xyz) * 12
                    cloud.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                                    for i, name in enumerate(('x', 'y', 'z'))]
                    cloud.data = xyz.tobytes()
                    cloud_pub.publish(cloud)
                last_cloud = now
            executor.spin_once(timeout_sec=.001)
            if until():
                return

    def goal(distance=.2, target_yaw=0.):
        commands.clear()
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = source.get_clock().now().to_msg()
        msg.pose.position.x = state['x'] + distance
        msg.pose.position.y = state['y']
        msg.pose.orientation.z = math.sin(target_yaw / 2)
        msg.pose.orientation.w = math.cos(target_yaw / 2)
        goal_pub.publish(msg)

    try:
        drive(3.)
        assert adapter.count_publishers('/cmd_vel') == 0
        assert commands and all(v == (0., 0.) for v in commands)
        goal()
        drive(2., until=lambda: any(v[0] > 0. for v in commands))
        assert any(v[0] > 0. for v in commands), adapter.session.reason
        cancel_pub.publish(Empty())
        drive(.2)
        commands.clear()
        drive(.4)
        assert adapter.session.goal is None and commands and all(v == (0., 0.) for v in commands)
        assert lateral[-1] == 0.
        print(f'PASS: real {backend} follows clear goal; cancel clears target and output')
        state['obstacle'] = True
        drive(.3)
        goal()
        drive(1.)
        assert commands and all(v[0] == 0. for v in commands), commands
        print('PASS: front obstacle prevents forward commands')
        cancel_pub.publish(Empty())
        state['obstacle'] = False
        drive(.4)
        goal()
        drive(2., until=lambda: any(v[0] > 0. for v in commands))
        assert any(v[0] > 0. for v in commands), adapter.session.reason
        state['cloud'] = False
        drive(.5)
        assert adapter.session.goal is None
        commands.clear()
        drive(.2)
        assert commands and all(v == (0., 0.) for v in commands)
        print('PASS: lost cloud cancels target and produces zero')
        state['cloud'] = True
        drive(.5)
        goal()
        drive(2., until=lambda: any(v[0] > 0. for v in commands))
        state['odom'] = False
        drive(.5)
        assert adapter.session.goal is None
        assert commands[-1] == (0., 0.)
        print('PASS: lost odometry cancels target; no robot nodes or /cmd_vel publisher')
        state['odom'] = True
        drive(.5)
        goal()
        state['move'] = True
        drive(7., until=lambda: adapter.session.reason == 'goal pose reached within 0.03 m and 0.05 rad')
        state['move'] = False
        assert adapter.session.goal is None and .17 <= state['x'] <= .205, (state, adapter.session.reason)
        drive(.15)  # Deliver the zero command published by the arrival callback to the subscriber.
        assert commands[-1] == (0., 0.)
        print(f'PASS: 0.2 m goal reached at synthetic FAST-LIO x={state["x"]:.3f} m')
        goal()
        drive(2., until=lambda: any(v[0] > 0. for v in commands))
        state['invalid'] = True
        drive(.3)
        assert adapter.session.goal is None and commands[-1] == (0., 0.)
        print('PASS: zero-range cloud clears active goal and stops output')
        state['invalid'] = False
        state['yaw'] = -.2
        drive(.5)
        goal(target_yaw=.2)
        drive(.8)
        assert adapter.sequence.phase == 'rotate_to_goal', adapter.session.reason
        assert commands and all(vx == 0. for vx, _ in commands)
        assert any(wz > 0. for _, wz in commands)
        # Rotation must not expire merely because no library path has been requested.
        state['move'] = True
        phase_commands.clear()
        drive(10., until=lambda: adapter.session.reason == 'goal pose reached within 0.03 m and 0.05 rad')
        state['move'] = False
        assert adapter.session.goal is None, (state, adapter.session.reason)
        drive(.15)
        assert abs(state['yaw'] - .2) <= .05
        assert any(phase == 'drive' and vx > 0. for phase, vx, _ in phase_commands)
        for phase in ('rotate_to_goal', 'rotate_to_yaw'):
            values = [(vx, wz) for p, vx, wz in phase_commands if p == phase]
            assert values and all(vx == 0. for vx, _ in values), (phase, values)
        assert commands[-1] == (0., 0.)
        print(f'PASS: rotate first → real {backend} path → final yaw, no translation during rotations')
        goal(distance=0., target_yaw=state['yaw'] + .3)
        drive(.3)
        assert adapter.sequence.phase == 'rotate_to_yaw'
        cancel_pub.publish(Empty())
        drive(.2)
        assert adapter.session.goal is None and commands[-1] == (0., 0.)
        print('PASS: cancel during final rotation clears target and produces zero')
        if backend == 'mpc':
            state['yaw'] = 0.
            drive(.3)
            goal()
            drive(1., until=lambda: adapter.sequence.phase == 'drive')
            before_drift = len(lateral)
            state['y'] += .04  # Simulate drift after initial alignment.
            drive(.5)
            assert any(v < -.001 for v in lateral[before_drift:]), (lateral[-5:], adapter.session.reason)
            state['move'] = True
            drive(7., until=lambda: adapter.session.reason == 'goal pose reached within 0.03 m and 0.05 rad')
            state['move'] = False
            assert adapter.session.goal is None, adapter.session.reason
            cancel_pub.publish(Empty())
            drive(.2)
            assert lateral[-1] == 0. and commands[-1] == (0., 0.)
            assert adapter.count_publishers(PREFIX + '/path') == 0
            assert adapter.count_publishers(PREFIX + '/mpc_result') == 1
            print('PASS: C++ MPC sends lateral correction; cancel zeros both axes; no lattice output')
        state['yaw'] = 0.
        drive(.3)
        goal()
        drive(2., until=lambda: any(v[0] > 0. for v in commands))
        assert any(v[0] > 0. for v in commands), adapter.session.reason
        child.terminate()
        child.wait(timeout=5)
        drive(.7, planner_alive=False)
        assert adapter.session.goal is None and commands[-1] == (0., 0.) and lateral[-1] == 0.
        print('PASS: C++ planner process loss clears the goal and outputs zero')
    finally:
        adapter.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        if child is not None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        log.close()


if __name__ == '__main__':
    main()
