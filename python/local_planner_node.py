#!/usr/bin/python3
"""Front-only (default), explicit rear/auto LiDAR + FAST-LIO MPC/library adapter."""
from collections import OrderedDict
from copy import deepcopy
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Empty, String

from local_planner_core import (FRAME, GoalSequence, PathSession, collision_free, follow,
                                front_mount, rear_cloud_mount, relative_goal, world_path, limit_goal_velocity)
from collision_feedback import choose_collision_source
from receipt_feedback import CollisionCloud, NavigationFeedback, validate_navigation, MAX_AGE_S
from planner_diagnostics import PlannerDiagnostics
from auto_base_enable import AutoBaseEnable

PREFIX = '/astribot/local_planner'
FRONT = '/sdk/astribot_lidar/front_scan'
REAR = '/sdk/astribot_lidar/back_scan'
ODOM = '/fast_lio/chassis/odometry'
LIDAR_TOPICS = {'front': (FRONT,), 'rear': (REAR,)}
# Replan independently of sensor callback cadence. Sensor freshness is checked
# on every request and dispatch; a new solve never refreshes sensor receipt age.
MPC_PERIOD_S = .1
MPC_RESULT_TIMEOUT_S = .15
MPC_COMMAND_TIMEOUT_S = 1.0


class MpcWaiting(ValueError):
    """Transient loss of usable feedback/control: stop, retain goal, retry."""


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class LocalPlanner(Node):
    requires_owner_status = True
    def __init__(self, **kwargs):
        super().__init__('astribot_local_planner', **kwargs)
        self.dispatch = self.declare_parameter('dispatch', False).value
        self.ignore_lidar = self.declare_parameter('ignore_lidar', False).value
        self.diagnostics = PlannerDiagnostics(lambda message: print(message, flush=True))
        self.supervised_reverse_test = self.declare_parameter('supervised_reverse_test', False).value
        self.collision_checks_enabled = self.declare_parameter(
            'python_collision_checks_enabled', True).value
        self.collision_lidar = self.declare_parameter('collision_lidar', 'front').value
        if self.collision_lidar not in ('front', 'rear', 'auto'):
            raise ValueError('collision_lidar must be front, rear or auto')
        self.backend = self.declare_parameter('planner_backend', 'mpc').value
        if self.backend not in ('mpc', 'library'):
            raise ValueError('planner_backend must be mpc or library')
        if self.ignore_lidar:
            if self.backend != 'mpc':
                raise ValueError('ignore_lidar requires the MPC backend')
            self.collision_checks_enabled = False
        if self.supervised_reverse_test:
            if self.backend != 'mpc':
                raise ValueError('supervised reverse test requires the MPC backend')
            self.collision_checks_enabled = False
        self.mpc_epoch = 0
        self.mpc_request_stamp = 0
        self.last_translation = (0., 0.)
        self.last_command_time = time.monotonic()
        self.reset_mpc()
        self.mount = front_mount()
        self.nav = NavigationFeedback()
        self.collision_inputs = {
            'front': CollisionCloud('front', self.mount, self.nav),
            'rear': CollisionCloud('rear', rear_cloud_mount(), self.nav)}
        self.selected_lidar = None
        self.session = PathSession()
        self.sequence = GoalSequence()
        if self.supervised_reverse_test:
            self.sequence.POSITION_TOLERANCE = .01
        self.snapshots = OrderedDict()
        self.cloud_stamp, self.cloud_received = 0, -math.inf
        self.points = np.empty((0, 2))
        self.owner_status, self.owner_received = {}, -math.inf
        self.owner_generation = None
        self.source_ids = None
        self.started = 0.
        self.output_topic = '/cmd_vel' if self.dispatch else PREFIX + '/cmd_vel_shadow'
        self.commands = self.create_publisher(Twist, self.output_topic, 1)
        self.auto_base_enable = None
        if self.dispatch:
            from std_srvs.srv import SetBool
            self.enable_client = self.create_client(SetBool, '/astribot/base/enable')
            self.auto_base_enable = AutoBaseEnable(
                lambda: self.enable_client.call_async(SetBool.Request(data=True)),
                self.get_logger().info, started=time.monotonic())
        self.scans = self.create_publisher(LaserScan, PREFIX + '/scan', 1)
        self.goals = self.create_publisher(PointStamped, PREFIX + '/way_point', 1)
        self.cancels = self.create_publisher(Empty, PREFIX + '/cancel', 1)
        self.paths = self.create_publisher(Path, PREFIX + '/path_map', 1)
        self.status = self.create_publisher(String, PREFIX + '/status', 1)
        if self.backend == 'mpc':
            from local_planner.msg import MpcRequest, MpcResult
            self.mpc_request_type = MpcRequest
            self.mpc_requests_pub = self.create_publisher(MpcRequest, PREFIX + '/mpc_request', 1)
            self.create_subscription(MpcResult, PREFIX + '/mpc_result', self.mpc_result, 1)
        self.create_subscription(Odometry, ODOM, self.odometry, qos_profile_sensor_data)
        for side, (topic,) in LIDAR_TOPICS.items():
            if self.ignore_lidar or (self.collision_lidar != 'auto' and side != self.collision_lidar):
                continue
            self.create_subscription(PointCloud2, topic,
                lambda msg, side=side: self.cloud(msg, side), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/astribot_viz/goal_pose', self.goal, 1)
        self.create_subscription(Empty, '/astribot_viz/goal_cancel', self.goal_cancel, 1)
        if self.backend == 'library':
            self.create_subscription(Path, PREFIX + '/path', self.path, 1)
        self.create_subscription(String, '/astribot/base/status', self.owner, 1)
        self.create_timer(.05, self.tick)
        if self.backend == 'mpc':
            self.create_timer(MPC_PERIOD_S, self.plan_mpc)
        enable_mode = 'maintaining automatic base enable' if self.dispatch else 'shadow only'
        self.get_logger().info(f'Local planner backend={self.backend}, output={self.output_topic}; {enable_mode}')
        if not self.collision_checks_enabled:
            self.get_logger().warning('Python collision checks DISABLED: no obstacle avoidance')
        if self.ignore_lidar:
            self.get_logger().warning('MPC TEST: LiDAR ignored; replanning at 10 Hz, no obstacle avoidance')
        if self.supervised_reverse_test:
            self.get_logger().warning('SUPERVISED REVERSE TEST: obstacles ignored; '
                                      '10 cm goals only, 0.03 m/s, 15 s deadline')

    def reset_mpc(self, *, reason='first result after goal/phase reset'):
        self.mpc_epoch += 1
        self.mpc_wait_started = time.monotonic()
        self.mpc_wait_reason = reason
        self.mpc_requests = OrderedDict()
        self.mpc_last_stamp = 0
        self.mpc_velocity = (0., 0.)
        self.mpc_reference_yaw = None
        self.mpc_received = -math.inf
        self.mpc_solve_ms = None
        self.mpc_path = np.empty((0, 2))
        self.mpc_cloud_stamp = None

    def odometry(self, msg):
        now = time.monotonic()
        self.nav.receive(msg, now=now, wall_ns=self.get_clock().now().nanoseconds)

    def plan_mpc(self):
        if self.session.goal is None or self.sequence.phase != 'drive':
            return
        try:
            now = time.monotonic()
            pose = self.ready(now)
            if self.ignore_lidar:
                self.points = np.empty((0, 2))
            else:
                self.use_collision_sample()
            self.send_mpc_request(pose, now)
        except MpcWaiting as exc:
            self.wait_mpc(str(exc))
        except ValueError as exc:
            self.cancel(reason=str(exc))

    def wait_mpc(self, reason):
        # Invalidate pre-stop proposals once, without clearing the target or
        # changing its publisher/owner pins. Explicit cancel still clears all.
        if self.mpc_received != -math.inf or self.mpc_requests:
            self.reset_mpc(reason=reason)
        self.session.reason = 'waiting: ' + reason
        self.commands.publish(Twist())
        self.last_translation = (0., 0.)
        self.last_command_time = time.monotonic()

    def send_mpc_request(self, pose, now):
        clock = self.get_clock().now()
        stamp = max(clock.nanoseconds, self.mpc_request_stamp + 1)
        self.mpc_request_stamp = stamp
        request = self.mpc_request_type()
        request.header.stamp.sec, request.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        request.header.frame_id = FRAME
        request.sent_at = clock.to_msg()
        request.epoch = self.mpc_epoch
        request.goal.x, request.goal.y = map(float, relative_goal(self.session.goal, pose))
        request.previous_velocity.x, request.previous_velocity.y = self.last_translation
        request.obstacles = [Point(x=float(x), y=float(y)) for x, y in self.points]
        self.mpc_requests[stamp] = (pose, now, self.mpc_epoch, self.last_translation)
        while len(self.mpc_requests) > 5:
            self.mpc_requests.popitem(last=False)
        self.mpc_requests_pub.publish(request)

    def owner(self, msg):
        try:
            status = json.loads(msg.data)
            self.owner_status = status if isinstance(status, dict) else {}
        except ValueError:
            self.owner_status = {}
        self.owner_received = time.monotonic()

    def collision_publishers(self, side):
        for topic in LIDAR_TOPICS[side]:
            if len(self.get_publishers_info_by_topic(topic)) != 1:
                raise ValueError(f'expected exactly one publisher: {topic}')

    def sources(self):
        topics = [ODOM, self.output_topic]
        if not getattr(self, 'ignore_lidar', False):
            if self.selected_lidar is None:
                raise ValueError('collision LiDAR has not been selected')
            topics = list(dict.fromkeys([*LIDAR_TOPICS[self.selected_lidar], *topics]))
        if self.backend == 'library':
            topics.append(PREFIX + '/path')
        else:
            topics.extend([PREFIX + '/mpc_request', PREFIX + '/mpc_result'])
        if self.dispatch and getattr(self, 'requires_owner_status', True):
            topics.append('/astribot/base/status')
        identities = []
        for topic in topics:
            endpoints = self.get_publishers_info_by_topic(topic)
            if len(endpoints) != 1:
                raise ValueError(f'expected exactly one publisher: {topic}')
            identities.append(bytes(endpoints[0].endpoint_gid))
        return tuple(identities)

    def ready(self, now):
        # Only receipt-age gaps are recoverable. Invalid geometry, replayed or
        # regressing timestamps, publisher changes and owner faults still cancel.
        if (self.backend == 'mpc' and not getattr(self, 'supervised_reverse_test', False)
                and self.nav.pose is not None and not self.nav.error
                and not self.nav.clock_fault and not self.nav.reference_error
                and now - self.nav.received > MAX_AGE_S):
            raise MpcWaiting('fresh FAST-LIO odometry')
        pose = validate_navigation(self.nav.fields(now))
        if not getattr(self, 'ignore_lidar', False):
            if self.selected_lidar is None:
                raise ValueError('collision LiDAR has not been selected')
            sample = self.collision_inputs[self.selected_lidar]
            problem = sample.problem(now, self.nav.stamp)
            if problem:
                if (self.backend == 'mpc' and not getattr(self, 'supervised_reverse_test', False)
                        and not sample.error and not sample.fault
                        and now - sample.received > MAX_AGE_S):
                    raise MpcWaiting('fresh ' + self.selected_lidar + ' cloud')
                raise ValueError(problem)
        identities = self.sources()
        if self.source_ids is not None and self.source_ids != identities:
            raise ValueError('publisher identity changed during goal')
        if self.dispatch and getattr(self, 'requires_owner_status', True):
            s = self.owner_status
            age = s.get('feedback_age_ms')
            if (not 0 <= now - self.owner_received <= .5 or s.get('armed') is not True
                    or s.get('mode') != 'MODEL' or s.get('hardware_writes') is not True
                    or s.get('feedback_source') != 'hardware' or s.get('stop_pending') is not False
                    or s.get('selected_source') != 'auto' or s.get('selected_topic') != '/cmd_vel'
                    or type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= 50
                    or type(s.get('generation')) is not int):
                raise ValueError('automatic base owner must be armed with fresh hardware feedback')
            if self.owner_generation is not None and self.owner_generation != s['generation']:
                raise ValueError('base owner generation changed')
        return pose

    @staticmethod
    def validate_reverse_test_goal(local, yaw_error):
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        if (not np.isfinite(local).all() or not math.isfinite(yaw_error)
                or not -.105 <= local[0] < 0. or np.linalg.norm(local) > .105
                or abs(local[1]) > .005 or abs(yaw_error) > .01):
            raise ValueError('reverse test requires <=10 cm behind, <=5 mm lateral, unchanged yaw')

    def goal(self, msg):
        # Replacement is an explicit new request; discard previous paths immediately.
        self.cancel(reason='new goal requested')
        try:
            p, q = msg.pose.position, msg.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            age = (self.get_clock().now().nanoseconds - stamp_ns(msg.header.stamp)) / 1e9
            if (msg.header.frame_id != 'map' or not all(math.isfinite(v) for v in values)
                    or abs(p.z) > .001 or abs(q.x) > .001 or abs(q.y) > .001
                    or abs(math.hypot(q.z, q.w) - 1.) > .001 or not -.05 <= age <= 2.):
                raise ValueError('expected a fresh finite planar RViz goal in map')
            now = time.monotonic()
            validate_navigation(self.nav.fields(now))
            if getattr(self, 'ignore_lidar', False):
                self.points = np.empty((0, 2))
            else:
                self.selected_lidar = choose_collision_source(
                    self.collision_inputs, now, self.nav.stamp, check_publishers=self.collision_publishers,
                    mode=self.collision_lidar)
                self.use_collision_sample()
            pose = self.ready(now)
            local = relative_goal((p.x, p.y), pose)
            if np.linalg.norm(local) > 3.:
                raise ValueError('local planner goals must be within 3 m')
            target_yaw = 2 * math.atan2(q.z, q.w)
            if getattr(self, 'supervised_reverse_test', False):
                self.validate_reverse_test_goal(local, target_yaw - pose[2])
                self.reverse_start_pose = tuple(pose)
                self.reverse_deadline = now + 15.
            self.source_ids = self.sources()
            self.owner_generation = (self.owner_status.get('generation')
                                     if self.dispatch and getattr(self, 'requires_owner_status', True) else None)
            self.session.begin((p.x, p.y), self.cloud_stamp)
            self.sequence.begin((p.x, p.y), target_yaw,
                reverse=getattr(self, 'supervised_reverse_test', False),
                face_goal=self.backend != 'mpc',
                concurrent_yaw=self.backend == 'mpc' and not getattr(self, 'supervised_reverse_test', False))
            self.started = now
            self.get_logger().info(f'Goal accepted: map_xy=({p.x:.3f},{p.y:.3f}) m '
                                   f'yaw={target_yaw:.3f} rad local_xy=({local[0]:.3f},{local[1]:.3f}) m; '
                                   f'reverse_test={getattr(self, "supervised_reverse_test", False)}; '
                                   f'collision_lidar={self.selected_lidar}; '
                                   f'position tolerance {self.sequence.POSITION_TOLERANCE} m')
        except ValueError as exc:
            self.cancel(reason=str(exc))
            self.get_logger().warning(str(exc))

    def goal_cancel(self, message):
        # ROS Humble callbacks do not carry a publisher GID. Record current
        # graph candidates, not an unverified claim that an operator clicked.
        try:
            publishers = [f'{ep.node_namespace.rstrip("/")}/{ep.node_name} gid={bytes(ep.endpoint_gid).hex()}'
                          for ep in self.get_publishers_info_by_topic('/astribot_viz/goal_cancel')]
        except (AttributeError, TypeError, ValueError, RuntimeError):
            publishers = []
        self.cancel(message, reason=f'cancel topic received; publishers={publishers or "unknown"}')

    def cancel(self, _msg=None, *, reason='cancel requested'):
        was_active = self.session.goal is not None
        self.reset_mpc()
        self.sequence.cancel()
        self.session.cancel(reason)
        self.snapshots.clear()
        self.source_ids, self.owner_generation = None, None
        self.selected_lidar = None
        self.commands.publish(Twist())
        self.last_translation = (0., 0.)
        self.last_command_time = time.monotonic()
        self.cancels.publish(Empty())
        empty = Path()
        empty.header.frame_id = 'map'
        self.paths.publish(empty)
        if was_active and hasattr(self, 'diagnostics'):
            self.get_logger().info(f'STOP: {reason}; goal cleared, zero command published')

    def use_collision_sample(self):
        sample = self.collision_inputs[self.selected_lidar]
        # Python-only debugging switch. All proposal and dispatch collision checks
        # consume this view; keep the original observation and timing validation.
        self.points = sample.points if self.collision_checks_enabled else np.empty((0, 2))
        self.cloud_stamp, self.cloud_received = sample.stamp, sample.received

    def cloud(self, msg, side='front'):
        if getattr(self, 'ignore_lidar', False):
            return
        now = time.monotonic()
        sample = self.collision_inputs[side]
        sample.receive(msg, now=now)
        # Other sensors cannot overwrite collision data, invalidate predictions,
        # or resurrect a cancelled goal. Source selection is pinned per goal.
        if self.selected_lidar != side:
            return
        try:
            problem = sample.problem(now, self.nav.stamp)
            if problem:
                raise ValueError(problem)
            self.use_collision_sample()
            points, stamp = self.points, self.cloud_stamp
            if self.session.goal is None or self.sequence.phase != 'drive':
                return
            pose = self.ready(now)
            if self.backend == 'mpc':
                return
            self.snapshots[stamp] = (pose, now)
            while len(self.snapshots) > 5:
                self.snapshots.popitem(last=False)
            goal = PointStamped()
            goal.header = deepcopy(msg.header)
            goal.header.frame_id = FRAME
            local = relative_goal(self.session.goal, pose)
            goal.point.x, goal.point.y = map(float, local)
            # Both messages carry the same source stamp; planner only uses matched pairs.
            self.goals.publish(goal)
            scan = LaserScan()
            scan.header = deepcopy(goal.header)
            scan.angle_min, scan.angle_increment = -math.pi, math.pi / 720
            scan.angle_max = scan.angle_min + 1439 * scan.angle_increment
            scan.range_min, scan.range_max = .01, 4.
            ranges = np.full(1440, np.inf, dtype=np.float32)
            if len(points):
                indices = np.floor((np.arctan2(points[:, 1], points[:, 0]) + math.pi)
                                   / scan.angle_increment).astype(int) % 1440
                np.minimum.at(ranges, indices, np.linalg.norm(points, axis=1))
            scan.ranges = ranges.tolist()
            self.scans.publish(scan)
        except MpcWaiting as exc:
            if self.session.goal is not None:
                self.wait_mpc(str(exc))
        except (ValueError, TypeError) as exc:
            if self.session.goal is not None:
                self.cancel(reason=str(exc))

    def path(self, msg):
        if self.backend != 'library':
            return
        stamp = stamp_ns(msg.header.stamp)
        snapshot = self.snapshots.pop(stamp, None)
        if (snapshot is None or msg.header.frame_id != FRAME or self.session.goal is None
                or self.sequence.phase != 'drive'):
            return
        pose, received = snapshot
        if time.monotonic() - received > .25:
            self.cancel(reason='planner result expired')
            return
        local = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses]).reshape(-1, 2)
        if not np.isfinite(local).all() or len(local) > 2000 or np.any(np.linalg.norm(local, axis=1) > 4.):
            self.cancel(reason='invalid planner path')
            return
        mapped = world_path(local, pose)
        if self.session.accept(stamp, mapped, received):
            display = deepcopy(msg)
            display.header.frame_id = 'map'
            for item, xy in zip(display.poses, mapped):
                item.header = deepcopy(display.header)
                item.pose.position.x, item.pose.position.y = map(float, xy)
                item.pose.orientation.w = 1.
            self.paths.publish(display)

    def mpc_result(self, result):
        stamp = stamp_ns(result.header.stamp)
        snapshot = self.mpc_requests.pop(stamp, None)
        if (snapshot is None or self.session.goal is None or self.sequence.phase != 'drive'
                or result.epoch != self.mpc_epoch or stamp <= self.mpc_last_stamp):
            return
        pose, received, epoch, previous_velocity = snapshot
        try:
            now = time.monotonic()
            self.ready(now)
            if epoch != result.epoch or not 0 <= now - received <= MPC_RESULT_TIMEOUT_S:
                # A late reply is not a new command and must not cancel a goal
                # or invalidate a newer accepted solution.
                return
            if not result.success:
                raise MpcWaiting('C++ MPC retry: ' + result.error)
            v, w = result.command.linear, result.command.angular
            velocity = (v.x, v.y)
            values = [v.x, v.y, v.z, w.x, w.y, w.z, result.solve_ms]
            local = np.array([[p.pose.position.x, p.pose.position.y] for p in result.path.poses]).reshape(-1, 2)
            if (result.header.frame_id != FRAME or result.path.header != result.header
                    or not all(math.isfinite(x) for x in values)
                    or any(x != 0. for x in (v.z, w.x, w.y, w.z))
                    or abs(v.x) + abs(v.y) > .100001 or not 0 <= result.solve_ms <= 40.
                    or len(local) != (1 if result.blocked else 21) or not np.isfinite(local).all()
                    or np.any(np.linalg.norm(local, axis=1) > .201)
                    or np.linalg.norm(local[0]) > 1e-6
                    or (result.blocked and velocity != (0., 0.))):
                raise ValueError('invalid C++ MPC result')
            for item in result.path.poses:
                p, q = item.pose.position, item.pose.orientation
                if (item.header != result.header or p.z != 0.
                        or (q.x, q.y, q.z, q.w) != (0., 0., 0., 1.)):
                    raise ValueError('invalid C++ MPC prediction pose')
            if not result.blocked:
                controls = np.diff(local, axis=0) / .1
                if (np.max(np.abs(controls[0] - velocity)) > 1e-6
                        or np.any(np.sum(np.abs(controls), axis=1) > .100001)
                        or np.max(np.abs(np.diff(np.vstack([previous_velocity, controls]), axis=0))) > .014001):
                    raise ValueError('C++ MPC command and prediction disagree')
            self.mpc_velocity = velocity
            self.mpc_reference_yaw = pose[2]
            self.mpc_received = received  # Age starts at request, never refreshed by late replies.
            self.mpc_solve_ms = result.solve_ms
            self.mpc_last_stamp = stamp
            self.mpc_path = world_path(local, pose)
            self.mpc_cloud_stamp = None  # Revalidate against latest cloud before dispatch.
            self.session.reason = 'blocked: MPC clearance' if result.blocked else 'C++ MPC tracking'
            display = deepcopy(result.path)
            display.header.frame_id = 'map'
            for item, xy in zip(display.poses, self.mpc_path):
                item.header = deepcopy(display.header)
                item.pose.position.x, item.pose.position.y = map(float, xy)
            self.paths.publish(display)
        except MpcWaiting as exc:
            self.wait_mpc(str(exc))
        except ValueError as exc:
            self.cancel(reason=str(exc))

    def mpc_drive(self, pose, now):
        if self.mpc_received == -math.inf:
            self.session.reason = 'waiting: C++ MPC result; ' + self.mpc_wait_reason
            return (0., 0.)
        if not 0 <= now - self.mpc_received <= MPC_COMMAND_TIMEOUT_S:
            raise MpcWaiting('C++ MPC command expired; replanning')
        if self.mpc_cloud_stamp != self.cloud_stamp:
            # The timer runs faster than optimization. New returns must invalidate
            # an unsafe cached prediction immediately, including the current footprint.
            local = np.vstack([np.zeros((1, 2)),
                               [relative_goal(xy, pose) for xy in self.mpc_path]])
            if any(np.any(np.linalg.norm(self.points - center, axis=1) < .55 - 1e-5)
                   for center in local):
                self.reset_mpc(reason='new cloud invalidated MPC prediction')
                self.session.reason = 'blocked: new cloud invalidated MPC prediction'
                return (0., 0.)
            self.mpc_cloud_stamp = self.cloud_stamp
        if getattr(self.sequence, 'concurrent_yaw', False):
            # The XY solution is fixed in the request's chassis frame. Keep its
            # world direction while the chassis turns between solver replies.
            angle = self.mpc_reference_yaw-pose[2]
            c, s = math.cos(angle), math.sin(angle)
            vx, vy = self.mpc_velocity
            rotated = (c*vx-s*vy, s*vx+c*vy)
            scale = min(1., .1/max(sum(map(abs, rotated)), 1e-12))
            return tuple(v*scale for v in rotated)
        return self.mpc_velocity

    def bounded_mpc_command(self, vx, vy, now):
        # Enforce physical time at dispatch, independent of cloud/reply rate.
        # Stops bypass this limiter and remain immediate.
        if vx == 0. and vy == 0.:
            return (0., 0.)
        step = .14 * max(0., min(.1, now - self.last_command_time))
        previous = np.asarray(self.last_translation)
        limited = np.clip((vx, vy), previous - step, previous + step)
        if getattr(self, 'supervised_reverse_test', False):
            speed = np.abs(limited).sum()
            if speed > .03:
                limited *= .03 / speed
        return tuple(map(float, limited))

    def tick(self):
        now = time.monotonic()
        vx, vy, wz = 0., 0., 0.
        if self.session.goal is not None:
            try:
                pose = self.ready(now)
                if getattr(self, 'supervised_reverse_test', False):
                    displacement = relative_goal(pose[:2], self.reverse_start_pose)
                    yaw_change = math.atan2(math.sin(pose[2] - self.reverse_start_pose[2]),
                                            math.cos(pose[2] - self.reverse_start_pose[2]))
                    if (now > self.reverse_deadline or not -.12 <= displacement[0] <= .01
                            or abs(displacement[1]) > .02 or abs(yaw_change) > .1):
                        raise ValueError('reverse test time/displacement/heading bound reached')
                previous_phase = self.sequence.phase
                phase, wz, arrived = self.sequence.update(pose)
                if phase != previous_phase:
                    self.reset_mpc()
                    # A path produced before/during a turn is anchored to an obsolete pose.
                    self.snapshots.clear()
                    self.session.begin(self.session.goal, self.cloud_stamp)
                    self.started = now
                    self.cancels.publish(Empty())
                    empty = Path()
                    empty.header.frame_id = 'map'
                    self.paths.publish(empty)
                if arrived:
                    self.cancel(reason=f'goal pose reached within {self.sequence.POSITION_TOLERANCE} m and 0.05 rad')
                elif phase == 'drive':
                    if self.backend == 'mpc':
                        vx, vy = self.mpc_drive(pose, now)
                        vx, vy = self.bounded_mpc_command(vx, vy, now)
                        if getattr(self.sequence, 'concurrent_yaw', False):
                            # Apply after acceleration limiting so a cached fast
                            # command cannot defeat the distance-dependent cap.
                            vx, vy = limit_goal_velocity((vx, vy), pose, self.session.goal,
                                                         self.sequence.POSITION_TOLERANCE)
                        # Limiting can change the prediction's direction during a
                        # reversal. Check the actual command through owner expiry.
                        if not collision_free(vx, wz, self.points, vy=vy, clearance=.55, horizon=.2):
                            vx, vy, wz = 0., 0., 0.
                            # While waiting, preserve the current epoch/request and
                            # its deadline. Repeated checks must not starve replies.
                            if self.mpc_received != -math.inf:
                                self.reset_mpc(reason='dispatched MPC clearance')
                            self.session.reason = 'blocked: dispatched MPC clearance'
                    elif not self.session.valid(now):
                        if now - self.started > .5:
                            raise ValueError('local path stale/missing')
                    else:
                        vx, wz, _ = follow(self.session.goal, pose, self.session.path)
                else:
                    self.session.reason = phase
                # The same collision gate covers both in-place rotations and path following.
                if self.session.goal is not None and not collision_free(vx, wz, self.points, vy=vy):
                    vx, vy, wz = 0., 0., 0.
                    if self.mpc_received != -math.inf:
                        self.reset_mpc(reason=f'{self.selected_lidar} collision envelope')
                    self.session.reason = f'blocked: {self.selected_lidar} collision envelope'
            except MpcWaiting as exc:
                self.wait_mpc(str(exc))
                vx, vy, wz = 0., 0., 0.
            except ValueError as exc:
                self.cancel(reason=str(exc))
                vx, vy, wz = 0., 0., 0.
        if vx != 0. or vy != 0. or wz != 0.:
            try:
                send_time = time.monotonic()
                self.ready(send_time)
                if self.sequence.phase == 'drive':
                    if self.backend == 'mpc' and not 0 <= send_time - self.mpc_received <= MPC_COMMAND_TIMEOUT_S:
                        raise MpcWaiting('C++ MPC command expired before dispatch; replanning')
                    if self.backend == 'library' and not self.session.valid(send_time):
                        raise ValueError('local path expired before dispatch')
            except MpcWaiting as exc:
                self.wait_mpc(str(exc))
                vx, vy, wz = 0., 0., 0.
            except ValueError as exc:
                self.cancel(reason=str(exc))
                vx, vy, wz = 0., 0., 0.
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.commands.publish(msg)
        self.last_translation = (vx, vy)
        self.last_command_time = time.monotonic()
        auto_enable = getattr(self, 'auto_base_enable', None)
        if auto_enable is not None and not auto_enable.done:
            auto_enable.tick(time.monotonic(), status=self.owner_status,
                age=time.monotonic()-self.owner_received,
                publishers=len(self.get_publishers_info_by_topic('/cmd_vel')),
                idle=self.session.goal is None,
                service_ready=self.enable_client.service_is_ready())
        status = dict(
            active=self.session.goal is not None, phase=self.sequence.phase, reason=self.session.reason,
            dispatch=self.dispatch, owner_status_required=getattr(self, 'requires_owner_status', True),
            output_topic=self.output_topic, planner_backend=self.backend,
            collision_checks_enabled=self.collision_checks_enabled,
            ignore_lidar=getattr(self, 'ignore_lidar', False),
            supervised_reverse_test=getattr(self, 'supervised_reverse_test', False),
            sensor_freshness_policy='independent_receipt_age_2s',
            mpc_solve_ms=self.mpc_solve_ms,
            collision_lidar=self.selected_lidar,
            collision_topic=LIDAR_TOPICS[self.selected_lidar][0] if self.selected_lidar else None,
            collision_input_errors={} if getattr(self, 'ignore_lidar', False) else {
                side:sample.problem(now,self.nav.stamp) for side,sample in self.collision_inputs.items()},
            command=[vx, vy, wz], owner_stop_confirmed=self.owner_status.get('stop_confirmed'))
        if self.session.goal is not None and getattr(self.nav, 'pose', None) is not None:
            pose = self.nav.pose
            status['goal_distance_m'] = math.hypot(self.session.goal[0]-pose[0], self.session.goal[1]-pose[1])
            error = self.sequence.target_yaw-pose[2]
            status['goal_yaw_error_rad'] = math.atan2(math.sin(error), math.cos(error))
            status['concurrent_yaw'] = self.sequence.concurrent_yaw
        self.status.publish(String(data=json.dumps(status)))
        if hasattr(self, 'diagnostics'):
            # Observability only: sensor receipt and owner feedback are independent
            # of the commanded velocity; never label a sent command as movement.
            side = self.selected_lidar or ('rear' if self.collision_lidar == 'rear' else 'front')
            sample = self.collision_inputs[side]
            self.diagnostics.emit(now, status=status, owner=self.owner_status,
                nav=self.nav.fields(now), goal=self.session.goal,
                lidar_age_ms=None if self.ignore_lidar else (now-sample.received)*1000,
                owner_age_ms=(now-self.owner_received)*1000)


def main():
    rclpy.init()
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel(reason='planner shutting down')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
