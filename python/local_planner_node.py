#!/usr/bin/python3
"""Front LiDAR / FAST-LIO / RViz adapter with MPC or path-library driving."""
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
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import Empty, String

from local_planner_core import (FRAME, GoalSequence, PathSession, cloud_xyz, collision_free, follow,
                                front_mount, obstacle_points, relative_goal, world_path)
from navigation_feedback import NavigationFeedback, validate_navigation

PREFIX = '/astribot/local_planner'
FRONT = '/sdk/astribot_lidar/front_scan'
FRONT_IMU = '/sdk/astribot_lidar/front_imu'
REAR_IMU = '/sdk/astribot_lidar/back_imu'
ODOM = '/fast_lio/chassis/odometry'


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class LocalPlanner(Node):
    def __init__(self, **kwargs):
        super().__init__('astribot_local_planner', **kwargs)
        self.dispatch = self.declare_parameter('dispatch', False).value
        self.backend = self.declare_parameter('planner_backend', 'mpc').value
        if self.backend not in ('mpc', 'library'):
            raise ValueError('planner_backend must be mpc or library')
        self.mpc_epoch = 0
        self.last_translation = (0., 0.)
        self.last_command_time = time.monotonic()
        self.reset_mpc()
        self.mount = front_mount()
        self.nav, self.front_clock = NavigationFeedback(), NavigationFeedback()
        self.session = PathSession()
        self.sequence = GoalSequence()
        self.snapshots = OrderedDict()
        self.cloud_stamp, self.cloud_received = 0, -math.inf
        self.cloud_fault, self.cloud_error = False, 'waiting for front cloud'
        self.points = np.empty((0, 2))
        self.owner_status, self.owner_received = {}, -math.inf
        self.owner_generation = None
        self.source_ids = None
        self.started = 0.
        self.output_topic = '/cmd_vel' if self.dispatch else PREFIX + '/cmd_vel_shadow'
        self.commands = self.create_publisher(Twist, self.output_topic, 1)
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
        for topic, clock in [(REAR_IMU, self.nav), (FRONT_IMU, self.front_clock)]:
            self.create_subscription(Imu, topic,
                lambda msg, clock=clock: clock.receive_reference(msg, now=time.monotonic()),
                qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM, self.odometry, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, FRONT, self.cloud, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/astribot_viz/goal_pose', self.goal, 1)
        self.create_subscription(Empty, '/astribot_viz/goal_cancel', self.cancel, 1)
        if self.backend == 'library':
            self.create_subscription(Path, PREFIX + '/path', self.path, 1)
        self.create_subscription(String, '/astribot/base/status', self.owner, 1)
        self.create_timer(.05, self.tick)
        self.get_logger().info(f'Local planner backend={self.backend}, output={self.output_topic}; '
                               'no automatic base enable')

    def reset_mpc(self):
        self.mpc_epoch += 1
        self.mpc_requests = OrderedDict()
        self.mpc_last_stamp = 0
        self.mpc_velocity = (0., 0.)
        self.mpc_received = -math.inf
        self.mpc_solve_ms = None
        self.mpc_path = np.empty((0, 2))
        self.mpc_cloud_stamp = None

    def odometry(self, msg):
        self.nav.receive(msg, now=time.monotonic(), wall_ns=self.get_clock().now().nanoseconds)

    def owner(self, msg):
        try:
            status = json.loads(msg.data)
            self.owner_status = status if isinstance(status, dict) else {}
        except ValueError:
            self.owner_status = {}
        self.owner_received = time.monotonic()

    def sources(self):
        topics = [FRONT, FRONT_IMU, REAR_IMU, ODOM, self.output_topic]
        if self.backend == 'library':
            topics.append(PREFIX + '/path')
        else:
            topics.extend([PREFIX + '/mpc_request', PREFIX + '/mpc_result'])
        if self.dispatch:
            topics.append('/astribot/base/status')
        identities = []
        for topic in topics:
            endpoints = self.get_publishers_info_by_topic(topic)
            if len(endpoints) != 1:
                raise ValueError(f'expected exactly one publisher: {topic}')
            identities.append(bytes(endpoints[0].endpoint_gid))
        return tuple(identities)

    def ready(self, now):
        pose = validate_navigation(self.nav.fields(now))
        if self.cloud_fault or self.cloud_error or not 0 <= now - self.cloud_received <= .25:
            raise ValueError(self.cloud_error or 'front cloud stale or clock fault')
        problem = self.front_clock.reference_problem(now)
        if problem:
            raise ValueError('front ' + problem)
        if not -.1e9 <= self.front_clock.reference_stamp - self.cloud_stamp <= .3e9:
            raise ValueError('front cloud source time is stale/future')
        if abs(self.nav.reference_stamp - self.front_clock.reference_stamp) > .1e9:
            raise ValueError('front/rear sensor clocks disagree')
        if abs(self.nav.stamp - self.cloud_stamp) > .2e9:
            raise ValueError('front cloud and FAST-LIO pose are not time-aligned')
        identities = self.sources()
        if self.source_ids is not None and self.source_ids != identities:
            raise ValueError('publisher identity changed during goal')
        if self.dispatch:
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
            pose = self.ready(now)
            local = relative_goal((p.x, p.y), pose)
            if np.linalg.norm(local) > 3.:
                raise ValueError('local planner goals must be within 3 m')
            self.source_ids = self.sources()
            self.owner_generation = self.owner_status.get('generation') if self.dispatch else None
            self.session.begin((p.x, p.y), self.cloud_stamp)
            self.sequence.begin((p.x, p.y), 2 * math.atan2(q.z, q.w))
            self.started = now
            self.get_logger().info('Goal accepted: turn toward goal, follow path, align target yaw; '
                                   'tolerances 0.03 m / 0.05 rad')
        except ValueError as exc:
            self.cancel(reason=str(exc))
            self.get_logger().warning(str(exc))

    def cancel(self, _msg=None, *, reason='operator cancelled'):
        self.reset_mpc()
        self.sequence.cancel()
        self.session.cancel(reason)
        self.snapshots.clear()
        self.source_ids, self.owner_generation = None, None
        self.commands.publish(Twist())
        self.last_translation = (0., 0.)
        self.last_command_time = time.monotonic()
        self.cancels.publish(Empty())
        empty = Path()
        empty.header.frame_id = 'map'
        self.paths.publish(empty)

    def cloud(self, msg):
        now = time.monotonic()
        try:
            if self.cloud_fault:
                raise ValueError('front clock fault latched; restart after checking timing')
            stamp = stamp_ns(msg.header.stamp)
            if stamp <= self.cloud_stamp:
                self.cloud_fault = True
                raise ValueError('front cloud timestamp did not advance')
            if msg.header.frame_id != 'livox_frame':
                raise ValueError('unqualified front cloud frame; expected livox_frame')
            problem = self.front_clock.reference_problem(now)
            if problem:
                raise ValueError(problem)
            if not -.1e9 <= self.front_clock.reference_stamp - stamp <= .3e9:
                raise ValueError('front cloud timestamp stale/future relative to front IMU')
            points = obstacle_points(cloud_xyz(msg), self.mount)
            self.points, self.cloud_stamp, self.cloud_received = points, stamp, now
            self.cloud_error = ''
            if self.session.goal is None or self.sequence.phase != 'drive':
                return
            pose = self.ready(now)
            if self.backend == 'mpc':
                request = self.mpc_request_type()
                request.header = deepcopy(msg.header)
                request.header.frame_id = FRAME
                request.sent_at = self.get_clock().now().to_msg()
                request.epoch = self.mpc_epoch
                request.goal.x, request.goal.y = map(float, relative_goal(self.session.goal, pose))
                request.previous_velocity.x, request.previous_velocity.y = self.last_translation
                request.obstacles = [Point(x=float(x), y=float(y)) for x, y in points]
                self.mpc_requests[stamp] = (pose, now, self.mpc_epoch, self.last_translation)
                while len(self.mpc_requests) > 5:
                    self.mpc_requests.popitem(last=False)
                self.mpc_requests_pub.publish(request)
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
        except (ValueError, TypeError) as exc:
            self.cloud_error = str(exc)
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
            if epoch != result.epoch or not 0 <= now - received <= .15:
                raise ValueError('C++ MPC result expired or goal epoch changed')
            if not result.success:
                raise ValueError('C++ MPC failed: ' + result.error)
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
        except ValueError as exc:
            self.cancel(reason=str(exc))

    def mpc_drive(self, pose, now):
        if self.mpc_received == -math.inf:
            if now - self.started > .5:
                raise ValueError('C++ MPC result missing')
            return (0., 0.)
        if not 0 <= now - self.mpc_received <= .15:
            raise ValueError('C++ MPC command expired')
        if self.mpc_cloud_stamp != self.cloud_stamp:
            # The timer runs faster than optimization. New returns must invalidate
            # an unsafe cached prediction immediately, including the current footprint.
            local = np.vstack([np.zeros((1, 2)),
                               [relative_goal(xy, pose) for xy in self.mpc_path]])
            if any(np.any(np.linalg.norm(self.points - center, axis=1) < .55 - 1e-5)
                   for center in local):
                self.reset_mpc()
                self.session.reason = 'blocked: new cloud invalidated MPC prediction'
                return (0., 0.)
            self.mpc_cloud_stamp = self.cloud_stamp
        return self.mpc_velocity

    def bounded_mpc_command(self, vx, vy, now):
        # Enforce physical time at dispatch, independent of cloud/reply rate.
        # Stops bypass this limiter and remain immediate.
        if vx == 0. and vy == 0.:
            return (0., 0.)
        step = .14 * max(0., min(.1, now - self.last_command_time))
        previous = np.asarray(self.last_translation)
        limited = np.clip((vx, vy), previous - step, previous + step)
        return tuple(map(float, limited))

    def tick(self):
        now = time.monotonic()
        vx, vy, wz = 0., 0., 0.
        if self.session.goal is not None:
            try:
                pose = self.ready(now)
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
                    self.cancel(reason='goal pose reached within 0.03 m and 0.05 rad')
                elif phase == 'drive':
                    if self.backend == 'mpc':
                        vx, vy = self.mpc_drive(pose, now)
                        vx, vy = self.bounded_mpc_command(vx, vy, now)
                        # Limiting can change the prediction's direction during a
                        # reversal. Check the actual command through owner expiry.
                        if not collision_free(vx, 0., self.points, vy=vy, clearance=.55, horizon=.2):
                            vx, vy = 0., 0.
                            self.reset_mpc()
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
                    self.reset_mpc()
                    self.session.reason = 'blocked: front collision envelope'
            except ValueError as exc:
                self.cancel(reason=str(exc))
                vx, vy, wz = 0., 0., 0.
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.commands.publish(msg)
        self.last_translation = (vx, vy)
        self.last_command_time = time.monotonic()
        self.status.publish(String(data=json.dumps(dict(
            active=self.session.goal is not None, phase=self.sequence.phase, reason=self.session.reason,
            dispatch=self.dispatch, output_topic=self.output_topic, planner_backend=self.backend,
            mpc_solve_ms=self.mpc_solve_ms,
            command=[vx, vy, wz], owner_stop_confirmed=self.owner_status.get('stop_confirmed')))))


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
