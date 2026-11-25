"""Orin leased dispatch retains the existing PC owner and needs no cloud."""
import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
ROBOT = Path('/home/ubuntu/xpolicylabdagger/real_world/robots/astribot_s1')


def test_relay_diagnostics_preserve_lease_acceptance_and_replay_rejection():
    if not ROBOT.exists():
        pytest.skip('PC transport source checkout is not available')
    from orin_dispatch_bridge import DiagnosedLease
    lease = DiagnosedLease()
    state = lease.issue(dict(generation=2), 10.)
    envelope = dict(schema=1, session='test', sequence=1, generation=2,
        command=[0., 0., 0.], goal=None, lease=state['transport_lease'])
    assert lease.accept(envelope, 10.01) == (0., 0., 0.)
    assert lease.last_forwarded == 10.01 and lease.last_rejection is None
    with pytest.raises(ValueError, match='replayed'):
        lease.accept(envelope, 10.02)
    assert lease.last_forwarded == 10.01 and 'replayed' in lease.last_rejection


def load_adapter():
    sys.path.insert(0, str(ROBOT))
    spec = importlib.util.spec_from_file_location('orin_dispatch_coordinator',
                                                 ROOT/'python/orin_dispatch_coordinator.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_dispatch_uses_leased_goal_and_command_with_no_cloud(monkeypatch):
    if not ROBOT.exists():
        pytest.skip('PC transport source checkout is not available')
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    module = load_adapter()
    context = Context()
    rclpy.init(context=context, domain_id=228)
    node = None
    try:
        node = module.RemotePlanner(context=context, parameter_overrides=[
            Parameter('dispatch', value=True), Parameter('ignore_lidar', value=True)])
        subscriptions = {s.topic_name: s for s in node.subscriptions}
        publishers = {p.topic_name: p for p in node.publishers}
        assert '/cocomatrix/local_planner/goal' in subscriptions
        assert '/astribot_viz/goal_pose' not in subscriptions
        assert '/cocomatrix/local_planner/command' in publishers
        assert not any('scan' in topic for topic in subscriptions)
        assert node.auto_base_enable is None  # The PC relay owns service access.
        node.nav.fields = lambda now: dict(position_navigation=(0., 0., 0.), navigation_age_ms=0.)
        node.get_publishers_info_by_topic = lambda topic: [object()]
        # No cloud input is needed to report readiness in ignore-LiDAR mode.
        assert node.input_readiness(10.) is True
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
