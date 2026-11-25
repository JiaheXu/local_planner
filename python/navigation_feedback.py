"""Planner-owned ROS observation validation; no robot SDK or XPolicy imports."""
import math

NAVIGATION_FRAME = "map"
NAVIGATION_CHILD = "astribot_viz_chassis"
ODOMETRY_MAX_AGE_S = 1.
ODOMETRY_MAX_DELIVERY_DELAY_S = 1.


def validate_navigation(status):
    p, age = status.get('position_navigation'), status.get('navigation_age_ms')
    if (p is None or len(p) != 3 or not all(math.isfinite(x) for x in p)
            or type(age) not in (int, float) or not math.isfinite(age)
            or not 0 <= age < ODOMETRY_MAX_AGE_S * 1000.):
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
        self._anchor_stamp = self._anchor_received = None
        self._stable_since = None
        self._qualified = False

    def receive_reference(self, message, *, now):
        """Compare source progress to a fixed baseline, not adjacent callbacks.

        Queue delay blocks readiness; catching up must not look like a clock jump.
        After qualification, never rebase a delayed stream into freshness.
        """
        if self.clock_fault:
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if not math.isfinite(now) or (self.reference_received is not None and now < self.reference_received):
            self.clock_fault = True
            self.reference_error = 'odometry reference monotonic clock regressed/invalid'
            self.reference_ready = False
            return
        if stamp <= 0 or (self.reference_stamp is not None and stamp <= self.reference_stamp):
            self.reference_error = 'non-advancing odometry clock reference'
            self.reference_ready, self._stable_since = False, None
            if self.reference_stamp is not None and stamp < self.reference_stamp:
                self.clock_fault = True
            return
        if self._anchor_stamp is None:
            self._anchor_stamp, self._anchor_received = stamp, now
        if self.reference_received is not None and now-self.reference_received >= ODOMETRY_MAX_AGE_S:
            # A fresh returning sample cannot certify stable delivery on its own.
            # Keep the baseline so a delayed stream cannot rebase itself to ready.
            self.reference_ready, self._stable_since = False, None
        progress = (stamp-self._anchor_stamp)/1e9
        elapsed = now-self._anchor_received
        skew = progress-elapsed
        self.reference_stamp, self.reference_received = stamp, now
        if skew > .25:
            if self._qualified:
                self.clock_fault = True
                self.reference_ready = False
                self.reference_error = (f'odometry clock jumped ahead of established reference by {skew:.6f} s; '
                                        'restart after resolving timing')
                return
            # Discard an unqualified startup backlog, then observe a full new
            # settling period. No goal can run before initial qualification.
            self._anchor_stamp, self._anchor_received = stamp, now
            self._stable_since = None
        elif skew < -ODOMETRY_MAX_DELIVERY_DELAY_S:
            self.reference_ready, self._stable_since = False, None
            self.reference_error = f'odometry delivery delayed {(-skew):.6f} s relative to established reference'
            return
        if self._stable_since is None:
            self._stable_since = now
        self.reference_ready = now-self._stable_since >= .2-1e-9
        if self.reference_ready:
            self._qualified = True
            self.reference_error = None
        else:
            self.reference_error = 'waiting for 0.2 s of stable advancing odometry reference'

    def reference_problem(self, now):
        if self.reference_error:
            return self.reference_error
        if not self.reference_ready:
            return 'waiting for advancing odometry clock reference'
        if not 0 <= now-self.reference_received < ODOMETRY_MAX_AGE_S:
            return 'stale odometry clock reference'
        return None

    def receive(self, message, *, now, wall_ns):
        try:
            if message.header.frame_id != NAVIGATION_FRAME or message.child_frame_id != NAVIGATION_CHILD:
                raise ValueError('expected map -> astribot_viz_chassis')
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            p, q = message.pose.pose.position, message.pose.pose.orientation
            if not all(math.isfinite(x) for x in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                raise ValueError('non-finite odometry pose')
            norm = math.hypot(q.x, q.y, q.z, q.w)
            if abs(norm - 1.) > .01:
                raise ValueError('invalid odometry quaternion')
            x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
            yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
            self.receive_reference(message, now=now)
            problem = self.reference_problem(now)
            if problem:
                raise ValueError(problem)
            self.pc_minus_source_ms = (wall_ns-stamp)/1e6
            self.pose, self.received, self.stamp = (p.x, p.y, yaw), now, stamp
            self.error = None
        except (ValueError, AttributeError, TypeError) as exc:
            self.error = str(exc)

    def fields(self, now):
        problem = self.reference_problem(now) or self.error
        return dict(position_navigation=self.pose if problem is None else None,
                    navigation_age_ms=(now-self.received)*1000 if self.pose is not None else None,
                    navigation_pc_minus_source_ms=self.pc_minus_source_ms,
                    navigation_reference_age_ms=(now-self.reference_received)*1000
                    if self.reference_received is not None else None,
                    navigation_error=problem)


