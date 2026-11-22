"""Independent stamped front/rear collision observations, with no ROS ownership."""
import math
import numpy as np
from local_planner_core import cloud_xyz, obstacle_points

MAX_CLOUD_AGE_S = 2.
MAX_CLOUD_AGE_NS = 2_000_000_000


class CollisionCloud:
    def __init__(self, side, mount, clock):
        self.side, self.mount, self.clock = side, mount, clock
        self.stamp, self.received = 0, -math.inf
        self.fault, self.error = False, f'waiting for {side} cloud'
        self.points = np.empty((0, 2))

    def receive(self, message, *, now):
        try:
            if self.fault:
                raise ValueError('cloud clock fault latched; restart after checking timing')
            stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
            if stamp <= self.stamp:
                self.fault = True
                raise ValueError('cloud timestamp did not advance')
            if message.header.frame_id != 'livox_frame':
                raise ValueError('unqualified cloud frame; expected livox_frame')
            problem = self.clock.reference_problem(now)
            if problem:
                raise ValueError(problem)
            if not -.1e9 <= self.clock.reference_stamp-stamp <= MAX_CLOUD_AGE_NS:
                raise ValueError('cloud timestamp stale/future relative to odometry')
            points = obstacle_points(cloud_xyz(message), self.mount)
            self.points, self.stamp, self.received = points, stamp, now
            self.error = ''
        except (ValueError, TypeError) as exc:
            self.error = f'{self.side}: {exc}'

    def problem(self, now, odom_stamp):
        if self.error:
            return self.error
        if self.fault or not 0 <= now-self.received <= MAX_CLOUD_AGE_S:
            return f'{self.side} cloud stale or clock fault'
        problem = self.clock.reference_problem(now)
        if problem:
            return f'{self.side}: {problem}'
        if not -.1e9 <= self.clock.reference_stamp-self.stamp <= MAX_CLOUD_AGE_NS:
            return f'{self.side} cloud source time is stale/future'
        if abs(odom_stamp-self.stamp) > MAX_CLOUD_AGE_NS:
            return f'{self.side} cloud and FAST-LIO pose are not time-aligned'
        return None


def choose_collision_source(streams, now, odom_stamp, *, check_publishers=None, mode='auto'):
    """Choose only at a goal boundary; publisher ambiguity always fails closed."""
    if mode not in ('rear', 'auto'):
        raise ValueError('collision_lidar must be rear or auto')
    errors = []
    for side in (('rear',) if mode == 'rear' else ('front', 'rear')):
        problem = streams[side].problem(now, odom_stamp)
        if problem:
            errors.append(problem)
            continue
        if check_publishers is not None:
            check_publishers(side)
        return side
    raise ValueError('No usable collision LiDAR: ' + '; '.join(errors))
