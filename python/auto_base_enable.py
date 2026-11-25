"""Maintain automatic control through owner services, never a velocity writer."""
import math


def fresh_ms(value, limit):
    return type(value) in (int, float) and math.isfinite(value) and 0 <= value <= limit


class AutoBaseEnable:
    def __init__(self, request, write, *, started, enable_on_start=False):
        self.request, self.write = request, write
        self.next_attempt = started
        self.done = False
        self.pending = None
        self.sent = None
        self.generation = None
        self.minimum_generation = -1
        self.last_message = None
        self.enable_on_start = enable_on_start
        self.initial_generation = None

    def report(self, reason):
        if reason != self.last_message:
            self.write(f'Automatic base enable: {reason}')
            self.last_message = reason

    def finish(self, reason):
        self.done = True
        self.report(reason)

    def tick(self, now, *, status, age, publishers, idle, service_ready):
        if self.done:
            return
        if not idle:
            return
        if self.pending is not None:
            if self.pending.done():
                try:
                    result = self.pending.result()
                    if result.success or result.message == 'already_enabled':
                        self.enable_on_start = False
                        self.minimum_generation = self.generation + 1
                        self.report('enabled /cmd_vel; waiting for new goal')
                    elif result.message in ('fresh_zero_command_required', 'stop_pending',
                                            'robot_not_ready', 'exactly_one_publisher_required'):
                        self.report('waiting: ' + result.message)
                    else:
                        self.finish('refused: ' + result.message)
                except Exception as exc:
                    self.finish('service failed: ' + str(exc))
                self.pending = None
                self.next_attempt = now + 1.
            elif now - self.sent > 2.:
                self.finish('enable service response timed out')
            return
        if not status or not 0 <= age <= .5:
            return
        generation = status.get('generation')
        if type(generation) is not int or generation < self.minimum_generation:
            return
        if self.enable_on_start:
            if self.initial_generation is None:
                self.initial_generation = generation
            elif generation != self.initial_generation:
                self.enable_on_start = False
        if (status.get('mode') == 'FAULT' or status.get('selected_source') != 'auto'
                or status.get('selected_topic') != '/cmd_vel'):
            self.finish('owner fault or different command source; explicit selection required')
            return
        if status.get('armed') is True and status.get('mode') == 'MODEL':
            self.enable_on_start = False
            self.report('armed /cmd_vel')
            return
        if publishers > 1:
            self.report('waiting: multiple /cmd_vel publishers')
            return
        initial_enable = self.enable_on_start and status.get('reason') in (
            'startup_stop', 'not_enabled', 'source_switch', 'disabled')
        if not initial_enable and status.get('reason') in ('startup_stop', 'not_enabled', 'source_switch'):
            # Start Astribot already owns initial selection/enable. Do not race it.
            return
        if not initial_enable and status.get('reason') not in ('publisher_conflict_or_missing', 'command_timeout'):
            self.finish('owner stopped for ' + str(status.get('reason')) + '; explicit enable required')
            return
        generation = status.get('generation')
        if type(generation) is not int or generation < self.minimum_generation or now < self.next_attempt:
            return
        if (publishers != 1 or not service_ready or status.get('mode') != 'HOLD'
                or status.get('armed') is not False or status.get('stop_pending') is not False
                or status.get('stop_confirmed') is not True or type(generation) is not int
                or status.get('hardware_writes') is not True
                or status.get('feedback_source') != 'hardware'
                or not fresh_ms(status.get('feedback_age_ms'), 50.)
                or not fresh_ms(status.get('command_age_ms'), 200.)
                or status.get('requested_ros_velocity') != [0., 0., 0.]):
            return
        self.generation = generation
        self.sent = now
        try:
            self.pending = self.request()
        except Exception as exc:
            self.finish('service failed: ' + str(exc))
