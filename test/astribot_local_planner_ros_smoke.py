"""Real MPC/library + adapter, synthetic observations only, isolated domain 227."""
import argparse
import math
import os
import signal
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
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty
from ament_index_python.packages import get_package_prefix, get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
SHARE = Path(get_package_share_directory('local_planner'))
INSTALL = Path(get_package_prefix('local_planner'))
sys.path.insert(0, str(ROOT / 'python'))
from local_planner_core import front_mount
from local_planner_node import LocalPlanner, PREFIX, FRONT, REAR, ODOM


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '227'
    parser = argparse.ArgumentParser()
    parser.add_argument('--planner', choices=['mpc', 'library'], default='mpc')
    parser.add_argument('--lidar', choices=['front', 'rear'], default='front')
    parser.add_argument('--without-collision-checks', action='store_true')
    parser.add_argument('--supervised-reverse-test', action='store_true')
    parser.add_argument('--ignore-lidar', action='store_true')
    options = parser.parse_args()
    backend, lidar = options.planner, options.lidar
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
    adapter = LocalPlanner(parameter_overrides=[Parameter('planner_backend', value=backend),
        Parameter('ignore_lidar', value=options.ignore_lidar),
        Parameter('supervised_reverse_test', value=options.supervised_reverse_test),
        Parameter('python_collision_checks_enabled', value=not options.without_collision_checks),
        Parameter('collision_lidar', value='auto' if lidar=='front' else 'rear')])
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(adapter)
    odom_pub = source.create_publisher(Odometry, ODOM, qos_profile_sensor_data)
    cloud_pub = None if options.ignore_lidar else source.create_publisher(
        PointCloud2, FRONT if lidar=='front' else REAR, qos_profile_sensor_data)
    goal_pub = source.create_publisher(PoseStamped, '/astribot_viz/goal_pose', 1)
    cancel_pub = source.create_publisher(Empty, '/astribot_viz/goal_cancel', 1)
    commands = []
    lateral = [0.]
    def command(msg):
        commands.append((msg.linear.x, msg.angular.z))
        lateral.append(msg.linear.y)
    source.create_subscription(Twist, PREFIX + '/cmd_vel_shadow',
                               command, 1)
    state = dict(x=0., y=0., yaw=0., obstacle=False, cloud=not options.ignore_lidar,
                 odom=True, move=False, invalid=False)
    phase_commands = []
    last_cloud = 0.
    mount = front_mount()

    def stamp(target, ns):
        target.sec, target.nanosec = divmod(ns, 1_000_000_000)

    def drive(seconds, until=lambda: False, planner_alive=True):
        nonlocal last_cloud
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert not planner_alive or child.poll() is None, Path(log.name).read_text()
            now = time.monotonic()
            ns = source.get_clock().now().nanoseconds + 37_000_000_000
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
        if options.ignore_lidar:
            assert not adapter.collision_checks_enabled
            for topic in (FRONT, REAR):
                assert adapter.count_publishers(topic) == adapter.count_subscribers(topic) == 0
            state['move'] = True
            goal(target_yaw=.1)
            drive(12., until=lambda: adapter.session.reason.startswith('goal pose reached'))
            drive(.2)
            assert adapter.session.goal is None and commands[-1] == (0., 0.), adapter.session.reason
            assert adapter.sequence.goal is None and adapter.sequence.phase == 'idle'
            assert not adapter.mpc_requests and not adapter.snapshots
            assert adapter.source_ids is None and adapter.owner_generation is None
            assert .17 <= state['x'] <= .23 and abs(state['yaw']-.1) <= .05
            assert any(v[0] > 0. for v in commands)
            print('PASS: MPC reaches position and yaw using odometry without any cloud publisher')
            start_x, start_yaw = state['x'], state['yaw']
            phase_commands.clear()
            goal(distance=-.2, target_yaw=start_yaw)
            drive(8., until=lambda: adapter.session.reason.startswith('goal pose reached'))
            drive(.2)
            assert adapter.session.goal is None, adapter.session.reason
            assert abs(state['x']-(start_x-.2)) <= .03
            assert any(vx < -.001 for vx, _ in commands)
            assert all(wz == 0. for _, wz in commands)
            assert all(phase != 'rotate_to_goal' for phase, _, _ in phase_commands)
            assert abs(state['yaw']-start_yaw) < 1e-6
            print('PASS: 20 cm backward MPC goal reaches target without any rotation')
            goal()
            drive(3., until=lambda: any(v[0] > 0. for v in commands))
            assert any(v[0] > 0. for v in commands), adapter.session.reason
            cancel_pub.publish(Empty())
            drive(.2)
            assert adapter.session.goal is None and commands[-1] == (0., 0.)
            assert adapter.sequence.goal is None and adapter.sequence.phase == 'idle'
            assert not adapter.mpc_requests and not adapter.snapshots
            assert adapter.source_ids is None and adapter.owner_generation is None
            goal()
            drive(3., until=lambda: any(v[0] > 0. for v in commands))
            assert any(v[0] > 0. for v in commands), adapter.session.reason
            print('PASS: new goals accepted after completion and cancel without planner restart')
            retained_goal = adapter.session.goal
            # Freeze only this synthetic C++ process, keeping DDS identity pinned.
            child.send_signal(signal.SIGSTOP)
            try:
                drive(1.3)
                assert adapter.session.goal == retained_goal and commands[-1] == (0., 0.)
            finally:
                child.send_signal(signal.SIGCONT)
            commands.clear()
            drive(2., until=lambda: any(v[0] > 0. for v in commands))
            assert adapter.session.goal == retained_goal and any(v[0] > 0. for v in commands)
            print('PASS: solver interruption expires command, retains goal and resumes fresh solving')
            state['odom'] = False
            drive(2.2)
            assert adapter.session.goal == retained_goal and commands[-1] == (0., 0.)
            assert adapter.session.reason.startswith('waiting:')
            state['odom'] = True
            commands.clear()
            drive(2., until=lambda: any(v[0] > 0. for v in commands))
            assert adapter.session.goal == retained_goal and any(v[0] > 0. for v in commands)
            drive(8., until=lambda: adapter.session.goal is None)
            assert adapter.session.reason.startswith('goal pose reached'), adapter.session.reason
            print('PASS: odometry gap stops motion, retains goal, resumes and reaches it')
            goal()
            drive(.3)
            state['odom'] = False
            drive(2.2)
            cancel_pub.publish(Empty())
            drive(.2)
            state['odom'] = True
            drive(.4)
            assert adapter.session.goal is None and commands[-1] == (0., 0.)
            print('PASS: explicit cancel while waiting prevents recovery from restarting motion')
            goal()
            drive(3., until=lambda: any(v[0] > 0. for v in commands))
            assert any(v[0] > 0. for v in commands), adapter.session.reason
            child.terminate()
            child.wait(timeout=5)
            drive(.5, planner_alive=False)
            assert adapter.session.goal is None and commands[-1] == (0., 0.)
            print('PASS: no-LiDAR MPC backend loss stops commands; shadow only')
            return
        if options.supervised_reverse_test:
            assert not adapter.dispatch and not adapter.collision_checks_enabled
            state.update(obstacle=True, move=True)
            drive(.3)
            goal(-.1)
            drive(12., until=lambda: state['x'] < -.09 and adapter.session.goal is None)
            drive(.2)  # Deliver the arrival stop through the shadow subscription.
            assert -.12 < state['x'] <= -.09, (state, adapter.session.reason)
            assert abs(state['y']) < .005 and abs(state['yaw']) < .01
            assert commands and min(v[0] for v in commands) < -.001
            assert all(abs(vx) <= .030001 and wz == 0. for vx, wz in commands)
            assert adapter.session.goal is None and commands[-1] == (0., 0.)
            assert len(adapter.points) == 0 and len(adapter.collision_inputs[lidar].points) > 0
            print('PASS: reverse 10 cm with synthetic obstacles, <=0.03 m/s, no turn, arrival stop')
            goal(-.1)
            drive(2., until=lambda: any(v[0] < -.001 for v in commands))
            cancel_pub.publish(Empty())
            drive(.2)
            assert adapter.session.goal is None and commands[-1] == (0., 0.)
            goal(-.1)
            drive(2., until=lambda: any(v[0] < -.001 for v in commands))
            state['odom'] = False
            drive(2.2)
            assert adapter.session.goal is None and commands[-1] == (0., 0.)
            print('PASS: reverse test retains operator cancel and odometry-loss stop; shadow only')
            return
        if options.without_collision_checks:
            assert adapter.collision_checks_enabled is False
            state['obstacle'] = True
            drive(.4)
            goal()
            drive(3.)
            assert adapter.session.goal is not None and commands[-1][0] > 0., adapter.session.reason
            assert len(adapter.points) == 0 and len(adapter.collision_inputs[lidar].points) > 0
            print('PASS: Python ignores synthetic obstacles for MPC and dispatch; raw cloud retained')
            cancel_pub.publish(Empty())
            drive(.2)
            assert adapter.session.goal is None and commands[-1] == (0.,0.)
            print('PASS: collision bypass retains operator cancel')
            goal()
            drive(2., until=lambda: any(v[0] > 0. for v in commands))
            assert any(v[0] > 0. for v in commands), adapter.session.reason
            state['odom'] = False
            drive(2.2)
            assert adapter.session.goal is not None and commands[-1] == (0.,0.)
            print('PASS: collision bypass retains odometry-loss stop; no live cmd_vel publisher')
            return
        goal()
        drive(2., until=lambda: any(v[0] > 0. for v in commands))
        assert any(v[0] > 0. for v in commands), adapter.session.reason
        assert adapter.selected_lidar==lidar
        if lidar=='rear':
            assert adapter.count_publishers(FRONT)==0
        assert adapter.count_subscribers('/sdk/astribot_lidar/front_imu')==0
        assert adapter.count_subscribers('/sdk/astribot_lidar/back_imu')==0
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
        print(f'PASS: {lidar} obstacle prevents forward commands')
        cancel_pub.publish(Empty())
        state['obstacle'] = False
        drive(.4)
        goal()
        drive(2., until=lambda: any(v[0] > 0. for v in commands))
        assert any(v[0] > 0. for v in commands), adapter.session.reason
        state['cloud'] = False
        drive(2.2)  # Sensor receipt expiry is 2 s; cached MPC commands expire at 1 s.
        assert (adapter.session.goal is not None) == (backend == 'mpc')
        commands.clear()
        drive(.2)
        assert commands and all(v == (0., 0.) for v in commands)
        print('PASS: lost cloud produces zero; MPC retains target, library cancels')
        state['cloud'] = True
        drive(.5)
        goal()
        drive(2., until=lambda: any(v[0] > 0. for v in commands))
        state['odom'] = False
        drive(2.2)
        assert (adapter.session.goal is not None) == (backend == 'mpc')
        assert commands[-1] == (0., 0.)
        print('PASS: lost odometry produces zero; MPC retains target, library cancels')
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
        if backend == 'mpc':
            assert adapter.sequence.phase == 'drive', adapter.session.reason
            assert any(vx > 0. for vx, _ in commands)
            assert all(wz == 0. for _, wz in commands)
        else:
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
        for phase in (('rotate_to_yaw',) if backend == 'mpc' else ('rotate_to_goal', 'rotate_to_yaw')):
            values = [(vx, wz) for p, vx, wz in phase_commands if p == phase]
            assert values and all(vx == 0. for vx, _ in values), (phase, values)
        assert commands[-1] == (0., 0.)
        print(f'PASS: real {backend} goal sequence → final yaw, no translation during rotations')
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
