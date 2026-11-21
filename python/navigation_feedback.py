"""Planner-owned ROS observation validation; no robot SDK or XPolicy imports."""
import math

NAVIGATION_FRAME = "map"
NAVIGATION_CHILD = "astribot_viz_chassis"


def validate_navigation(status):
    p, age = status.get('position_navigation'), status.get('navigation_age_ms')
    if (p is None or len(p) != 3 or not all(math.isfinite(x) for x in p)
            or type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= 250.):
        raise ValueError('Missing/invalid/stale FAST-LIO navigation pose: '
                         + str(status.get('navigation_error') or 'requires fresh chassis odometry'))
    return tuple(p)


class NavigationFeedback:
    """Stamped map-to-chassis poses, independent of SDK joint coordinates."""
    def __init__(self):
        self.pose, self.received, self.stamp = None, 0., 0
        self.error = None
        self.reference_stamp = self.reference_received = None
        self.reference_ready = False
        self.reference_error = None
        self.clock_fault = False
        self.pc_minus_source_ms = None

    def receive_reference(self, message, *, now):
        """Use live rear-IMU progress, never infer freshness from PC wall time."""
        if self.clock_fault:
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp <= 0 or (self.reference_stamp is not None and stamp <= self.reference_stamp):
            self.reference_error = 'non-advancing IMU clock reference'
            return
        if self.reference_stamp is not None:
            progress = (stamp-self.reference_stamp)/1e9
            elapsed = now-self.reference_received
            if elapsed < 0 or abs(progress-elapsed) > .1:
                self.clock_fault = True
                self.reference_error = (f'IMU clock/reference delivery discontinuity: source advanced '
                                        f'{progress:.6f} s over {elapsed:.6f} s of receipt time; '
                                        'start a new check after resolving timing')
                return
            self.reference_ready = True
        self.reference_stamp, self.reference_received = stamp, now
        self.reference_error = None

    def reference_problem(self, now):
        if self.reference_error:
            return self.reference_error
        if not self.reference_ready:
            return 'waiting for advancing IMU clock reference'
        if not 0 <= now-self.reference_received <= .100001:
            return 'stale IMU clock reference'
        return None

    def receive(self, message, *, now, wall_ns):
        try:
            if message.header.frame_id != NAVIGATION_FRAME or message.child_frame_id != NAVIGATION_CHILD:
                raise ValueError('expected map -> astribot_viz_chassis')
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if stamp <= self.stamp:
                raise ValueError('non-advancing odometry stamp')
            self.pc_minus_source_ms = (wall_ns-stamp)/1e6
            problem = self.reference_problem(now)
            if problem:
                raise ValueError(problem)
            if not -.1e9 <= self.reference_stamp-stamp <= .3e9:
                raise ValueError('odometry source stamp is stale/future relative to live IMU')
            p, q = message.pose.pose.position, message.pose.pose.orientation
            if not all(math.isfinite(x) for x in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                raise ValueError('non-finite odometry pose')
            norm = math.hypot(q.x, q.y, q.z, q.w)
            if abs(norm - 1.) > .01:
                raise ValueError('invalid odometry quaternion')
            x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
            yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
            self.pose, self.received, self.stamp = (p.x, p.y, yaw), now, stamp
            self.error = None
        except (ValueError, AttributeError, TypeError) as exc:
            self.error = str(exc)

    def fields(self, now):
        problem = self.reference_problem(now) or self.error
        if (not problem and self.pose is not None
                and not -.1e9 <= self.reference_stamp-self.stamp <= .3e9):
            problem = 'odometry source stamp is stale/future relative to live IMU'
        return dict(position_navigation=self.pose if problem is None else None,
                    navigation_age_ms=(now-self.received)*1000 if self.pose is not None else None,
                    navigation_pc_minus_source_ms=self.pc_minus_source_ms,
                    navigation_reference_age_ms=(now-self.reference_received)*1000
                    if self.reference_received is not None else None,
                    navigation_error=problem)


