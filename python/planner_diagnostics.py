"""Read-only terminal diagnostics; never changes control or publishes commands."""
import math


def number(value, unit='', digits=0):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return 'n/a'
    return f'{value:.{digits}f}{unit}'


def vector(value):
    if value is None:
        return 'n/a'
    return '[' + ','.join(number(v, digits=3) for v in value) + ']'


class PlannerDiagnostics:
    def __init__(self, write):
        self.write = write
        self.last_time = -math.inf
        self.last_state = None

    def emit(self, now, *, status, owner, nav, goal, lidar_age_ms, owner_age_ms):
        key = (status.get('active'), status.get('phase'), status.get('reason'),
               owner.get('armed'), owner.get('mode'), owner.get('selected_source'),
               nav.get('navigation_error'))
        if key == self.last_state and now-self.last_time < 1.:
            return
        self.last_time, self.last_state = now, key
        phase = status.get('phase', 'idle')
        reason = status.get('reason') or phase
        if status.get('active'):
            description = phase if reason == phase else f'{phase}; {reason}'
            if status.get('goal_distance_m') is not None:
                description += (f"; distance={number(status['goal_distance_m'], ' m', 3)}"
                                f"; yaw_error={number(status.get('goal_yaw_error_rad'), ' rad', 3)}")
        elif nav.get('navigation_error'):
            description = f"idle; {nav['navigation_error']}"
        elif (status.get('dispatch', True) and status.get('owner_status_required', True)
              and owner.get('armed') is not True):
            description = f"idle; base disabled ({owner.get('reason', 'waiting for owner')})"
        else:
            # The stop event/status topic retains history. It does not prevent
            # a new goal, so do not present it as a permanent idle failure.
            description = 'idle; waiting for next goal'
        # Keep even multiline errors on a single terminal line.
        description = ' '.join(description.split())
        self.write(f"[local+planner]: {{cmd_vel}} {vector(status.get('command'))}, "
                   f"status{{{description}}}")
