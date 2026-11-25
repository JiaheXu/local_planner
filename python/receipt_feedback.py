"""PC testing: each validated sensor stream expires 2 s after local receipt.

Header clocks are never compared to each other or to the PC clock. Source stamps
remain unchanged and must advance for sensor replay checks.
"""
import math
from navigation_feedback import NavigationFeedback as ReferenceNavigation
from navigation_feedback import validate_navigation as validate_reference
from collision_feedback import CollisionCloud as ReferenceCloud
from local_planner_core import cloud_xyz, obstacle_points

MAX_AGE_S = 2.


class NavigationFeedback(ReferenceNavigation):
    def receive_reference(self, message, *, now):
        if self.clock_fault:
            return
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        if not math.isfinite(now) or (self.reference_received is not None and now < self.reference_received):
            self.clock_fault = True
            self.reference_error = 'odometry monotonic clock regressed/invalid'
            return
        if stamp <= 0 or (self.reference_stamp is not None and stamp <= self.reference_stamp):
            self.clock_fault = self.reference_stamp is not None and stamp < self.reference_stamp
            self.reference_error = 'non-advancing odometry timestamp'
            return
        self.reference_stamp, self.reference_received = stamp, now
        self.reference_error, self.reference_ready = None, True

    def reference_problem(self, now):
        if self.clock_fault or self.reference_error:
            return self.reference_error or 'odometry clock fault latched'
        if not self.reference_ready:
            return 'waiting for odometry'
        if not 0 <= now-self.reference_received <= MAX_AGE_S:
            return 'odometry not received within the last 2 s'
        return None

    def fields(self, now):
        result = super().fields(now)
        result['navigation_receipt_only'] = True
        return result


def validate_navigation(status):
    if status.get('navigation_receipt_only') is not True:
        return validate_reference(status)
    pose, age = status.get('position_navigation'), status.get('navigation_age_ms')
    if (pose is None or len(pose) != 3 or not all(math.isfinite(v) for v in pose)
            or type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= MAX_AGE_S*1000):
        raise ValueError('Missing/invalid/stale FAST-LIO navigation pose: '
                         + str(status.get('navigation_error') or 'requires receipt within 2 s'))
    return tuple(pose)


class CollisionCloud(ReferenceCloud):
    def receive(self, message, *, now):
        try:
            if self.fault:
                raise ValueError('cloud clock fault latched; restart after checking timing')
            stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
            if stamp <= self.stamp:
                self.fault = True
                raise ValueError('cloud timestamp did not advance')
            if not math.isfinite(now) or now < self.received:
                self.fault = True
                raise ValueError('cloud monotonic clock regressed/invalid')
            if message.header.frame_id != 'livox_frame':
                raise ValueError('unqualified cloud frame; expected livox_frame')
            points = obstacle_points(cloud_xyz(message), self.mount)
            self.points, self.stamp, self.received = points, stamp, now
            self.error = ''
        except (ValueError, TypeError) as exc:
            self.error = f'{self.side}: {exc}'

    def problem(self, now, odom_stamp):
        if self.error:
            return self.error
        if self.fault or not 0 <= now-self.received <= MAX_AGE_S:
            return f'{self.side} cloud not received within the last 2 s or clock fault'
        return None
