"""Explicit MPC test mode: no cloud dependency, other control gates retained."""
from pathlib import Path
from types import SimpleNamespace as NS, MethodType
from collections import OrderedDict
import subprocess

import numpy as np
import pytest

from local_planner_node import LocalPlanner, ODOM, PREFIX
from local_planner_core import PathSession, GoalSequence
from receipt_feedback import NavigationFeedback
from test_navigation_feedback import odometry

ROOT = Path(__file__).resolve().parents[1]


def state():
    node = NS(ignore_lidar=True, backend='mpc', dispatch=False, selected_lidar=None,
              nav=NavigationFeedback(), source_ids=None, output_topic='shadow',
              collision_inputs={}, session=PathSession(), sequence=GoalSequence())
    node.get_publishers_info_by_topic = lambda topic: [NS(endpoint_gid=[1])]
    node.sources = MethodType(LocalPlanner.sources, node)
    node.ready = MethodType(LocalPlanner.ready, node)
    return node


def test_readiness_without_any_cloud_retains_odometry_and_owner_checks():
    node = state()
    with pytest.raises(ValueError, match='FAST-LIO'):
        node.ready(10.)
    node.nav.receive(odometry(1_000_000_000), now=10., wall_ns=1_000_000_000)
    assert node.ready(10.) == pytest.approx((1., 2., np.pi/2))
    node.dispatch = True
    node.owner_status, node.owner_received, node.owner_generation = {}, 10., None
    with pytest.raises(ValueError, match='base owner'):
        node.ready(10.)


def test_ignore_lidar_does_not_require_cloud_publishers():
    node = state()
    seen = []
    def endpoints(topic):
        seen.append(topic)
        return [NS(endpoint_gid=[1])]
    node.get_publishers_info_by_topic = endpoints
    node.sources()
    assert seen == [ODOM, 'shadow', PREFIX+'/mpc_request', PREFIX+'/mpc_result']


def test_odometry_only_updates_observation_not_solver_cadence(monkeypatch):
    import local_planner_node as module
    from builtin_interfaces.msg import Time
    from local_planner.msg import MpcRequest
    from std_msgs.msg import Header
    def message(stamp):
        msg = odometry(stamp)
        msg.header = Header(frame_id='map', stamp=Time(sec=stamp // 10**9, nanosec=stamp % 10**9))
        return msg
    node = state()
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000, to_msg=lambda: Time(sec=1)))
    calls, cancelled = [], []
    node.mpc_request_type = MpcRequest
    node.mpc_epoch, node.last_translation, node.mpc_requests = 7, (0., 0.), OrderedDict()
    node.mpc_requests_pub = NS(publish=calls.append)
    node.send_mpc_request = MethodType(LocalPlanner.send_mpc_request, node)
    node.cancel = lambda *, reason: cancelled.append(reason)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    LocalPlanner.odometry(node, message(1_000_000_000))
    assert not calls
    node.session.begin((2., 2.), 1)
    node.sequence.begin((2., 2.), 0.)
    node.sequence.phase = 'drive'
    LocalPlanner.odometry(node, message(1_100_000_000))
    assert calls == []
    assert node.nav.stamp == 1_100_000_000
    LocalPlanner.odometry(node, message(1_100_000_000))
    assert calls == [] and node.nav.error


@pytest.fixture
def timer_node(monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    context = Context()
    rclpy.init(context=context, domain_id=228)
    node = LocalPlanner(context=context, parameter_overrides=[Parameter('ignore_lidar', value=True)])
    monkeypatch.setattr(node, 'get_publishers_info_by_topic', lambda topic: [NS(endpoint_gid=[1])])
    node.nav.receive(odometry(1_000_000_000), now=10., wall_ns=1_000_000_000)
    node.session.begin((2., 2.), 1)
    node.sequence.begin((2., 2.), np.pi/2, face_goal=False)
    requests, commands = [], []
    monkeypatch.setattr(node.mpc_requests_pub, 'publish', requests.append)
    monkeypatch.setattr(node.commands, 'publish', commands.append)
    try:
        yield node, requests, commands
    finally:
        node.destroy_node()
        context.shutdown()


def test_timer_replans_same_valid_observation_with_unique_ids(timer_node, monkeypatch):
    import local_planner_node as module
    node, requests, _ = timer_node
    for now in (10., 10.1, 10.2):
        monkeypatch.setattr(module.time, 'monotonic', lambda: now)
        node.plan_mpc()
    assert len(requests) == 3
    stamps = [module.stamp_ns(request.header.stamp) for request in requests]
    assert stamps == sorted(set(stamps))
    assert all(request.obstacles == [] for request in requests)
    assert all((request.goal.x, request.goal.y) == pytest.approx((0., -1.)) for request in requests)
    node.cancel()
    node.plan_mpc()
    assert len(requests) == 3


def test_stale_odometry_waits_then_replans_same_goal(timer_node, monkeypatch):
    import local_planner_node as module
    node, requests, commands = timer_node
    monkeypatch.setattr(module.time, 'monotonic', lambda: 12.1)
    node.tick()
    assert node.session.goal == (2., 2.)
    assert commands[-1].linear.x == commands[-1].linear.y == commands[-1].angular.z == 0.
    node.plan_mpc()
    assert not requests
    node.nav.receive(odometry(2_000_000_000), now=12.2, wall_ns=2_000_000_000)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 12.2)
    node.plan_mpc()
    assert len(requests) == 1 and node.session.goal == (2., 2.)


def test_invalid_pose_still_cancels_goal(timer_node, monkeypatch):
    import local_planner_node as module
    node, _, commands = timer_node
    msg = odometry(2_000_000_000)
    msg.pose.pose.position.x = float('nan')
    node.nav.receive(msg, now=10.1, wall_ns=2_000_000_000)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.1)
    node.tick()
    assert node.session.goal is None
    assert commands[-1].linear.x == commands[-1].linear.y == commands[-1].angular.z == 0.


def test_default_mode_still_requires_collision_source():
    node = state()
    node.ignore_lidar = False
    node.nav.receive(odometry(1_000_000_000), now=10., wall_ns=1_000_000_000)
    with pytest.raises(ValueError, match='collision LiDAR'):
        node.ready(10.)


def test_ignore_lidar_constructor_omits_cloud_subscriptions():
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    context = Context()
    rclpy.init(context=context, domain_id=228)
    node = None
    try:
        node = LocalPlanner(context=context, parameter_overrides=[Parameter('ignore_lidar', value=True)])
        topics = {subscription.topic_name for subscription in node.subscriptions}
        assert not any(topic.endswith(('/front_scan', '/back_scan')) for topic in topics)
        assert ODOM in topics and not node.collision_checks_enabled
        assert not node.dispatch
        assert node.auto_base_enable is None
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_ignore_lidar_launcher_is_explicit_and_python_mpc_only():
    script = str(ROOT/'scripts/run_astribot.sh')
    result = subprocess.run(['bash', script, '--dry-run', '--ignore-lidar'], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'ignore_lidar:=true' in result.stdout and 'planner_backend:=mpc' in result.stdout
    for option in ('--cpp', '--planner=library'):
        result = subprocess.run(['bash', script, '--dry-run', '--ignore-lidar', option], capture_output=True)
        assert result.returncode == 2
