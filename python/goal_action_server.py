"""ROS action admission and lifecycle for the existing local planner.

All callbacks run on the planner's single-threaded executor. Execute awaits an
rclpy Future; it must never block the sensor, planner, or cancellation callbacks.
"""
from copy import deepcopy
from dataclasses import dataclass, field
import math
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from local_planner_interfaces.action import NavigateLocalGoal

ACTION_NAME = '/cocomatrix/local_planner/navigate'
READY_TIMEOUT_S = 5.0


@dataclass
class _Goal:
    handle: object
    accepted_at: float
    future: Future = field(default_factory=Future)
    started: bool = False
    cancel_pending: bool = False


class GoalActionServer:
    def __init__(self, node):
        self.node = node
        self.active = None
        self.goals = {}
        self.closed = False
        self.server = ActionServer(
            node, NavigateLocalGoal, ACTION_NAME,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)
        self.timer = node.create_timer(.05, self.tick)

    def goal_callback(self, request):
        pose = request.pose
        p, q = pose.pose.position, pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        valid = (not self.closed and pose.header.frame_id == 'map'
                 and all(math.isfinite(value) for value in values)
                 and abs(p.z) <= .001 and abs(q.x) <= .001 and abs(q.y) <= .001
                 and abs(math.hypot(q.z, q.w) - 1.) <= .001)
        return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def handle_accepted_callback(self, handle):
        previous = self.active
        if previous is not None:
            self._finish(previous, 'aborted', 'superseded by a new action goal')
        # Clear planner state and publish zero before the replacement may start.
        self.node.cancel(reason='new action goal accepted')
        goal = _Goal(handle, time.monotonic())
        self.goals[id(handle)] = goal
        self.active = goal
        handle.execute()

    async def execute_callback(self, handle):
        goal = self.goals[id(handle)]
        try:
            return await goal.future
        finally:
            self.goals.pop(id(handle), None)

    def cancel_callback(self, handle):
        goal = self.goals.get(id(handle))
        if goal is None or goal is not self.active or goal.future.done():
            return CancelResponse.REJECT
        goal.cancel_pending = True
        self.node.cancel(reason='action cancellation requested')
        # Humble sets CANCELING only after this callback returns ACCEPT.
        return CancelResponse.ACCEPT

    def _finish(self, goal, state, reason):
        if goal.future.done():
            return
        result = NavigateLocalGoal.Result()
        result.success = state == 'succeeded'
        result.message = reason
        # Terminal transitions publish ROS status; shutdown may already have
        # invalidated the context. Still release our suspended execute task.
        if self.node.context.ok():
            if state == 'succeeded':
                goal.handle.succeed()
            elif state == 'canceled':
                goal.handle.canceled()
            else:
                goal.handle.abort()
        goal.future.set_result(result)
        if self.active is goal:
            self.active = None

    def _ready(self, now):
        if not self.node.ignore_lidar:
            # goal() normally selects this source, but ready() needs it first.
            from collision_feedback import choose_collision_source
            self.node.selected_lidar = choose_collision_source(
                self.node.collision_inputs, now, self.node.nav.stamp,
                check_publishers=self.node.collision_publishers,
                mode=self.node.collision_lidar)
            self.node.use_collision_sample()
        return self.node.ready(now)

    def _feedback(self, goal, state):
        feedback = NavigateLocalGoal.Feedback()
        feedback.state = state
        feedback.distance_remaining = math.nan
        feedback.yaw_error = math.nan
        pose = self.node.nav.pose
        if pose is not None and all(math.isfinite(value) for value in pose):
            target = goal.handle.request.pose.pose
            feedback.distance_remaining = math.hypot(
                target.position.x - pose[0], target.position.y - pose[1])
            yaw_error = 2 * math.atan2(target.orientation.z, target.orientation.w) - pose[2]
            feedback.yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        goal.handle.publish_feedback(feedback)

    def tick(self):
        goal = self.active
        if goal is None or self.closed:
            return
        if goal.cancel_pending or goal.handle.is_cancel_requested:
            if goal.handle.is_cancel_requested:
                if not goal.cancel_pending:
                    self.node.cancel(reason='action cancellation requested')
                self._finish(goal, 'canceled', 'action cancellation requested')
            return
        if not goal.started:
            now = time.monotonic()
            try:
                self._ready(now)
            except ValueError as exc:
                reason = 'waiting for navigation: ' + str(exc)
                if now - goal.accepted_at >= READY_TIMEOUT_S:
                    reason = 'navigation readiness timeout: ' + str(exc)
                    self.node.cancel(reason=reason)
                    self._finish(goal, 'aborted', reason)
                else:
                    self._feedback(goal, reason)
                return
            if now - goal.accepted_at >= READY_TIMEOUT_S:
                reason = 'navigation readiness timeout: 5 s after acceptance'
                self.node.cancel(reason=reason)
                self._finish(goal, 'aborted', reason)
                return
            message = deepcopy(goal.handle.request.pose)
            # PC validates the RViz stamp. Only Orin's own clock is comparable
            # with the planner clock; action admission bounds the local wait.
            message.header.stamp = self.node.get_clock().now().to_msg()
            goal.started = True
            self.node.goal(message)
        if self.node.session.goal is None:
            reason = self.node.session.reason or 'planner cleared goal'
            state = 'succeeded' if reason.startswith('goal pose reached within ') else 'aborted'
            self._finish(goal, state, reason)
            return
        self._feedback(goal, 'planning: ' + self.node.session.reason)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.timer.cancel()
        if self.node.context.ok():
            self.node.cancel(reason='Orin basic planner shutting down')
        if self.active is not None:
            self._finish(self.active, 'aborted', 'Orin basic planner shutting down')
        self.server.destroy()
        self.node.destroy_timer(self.timer)
