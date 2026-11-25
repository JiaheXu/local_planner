"""Offline startup/restart enable policy; no robot services are contacted."""
import sys
from pathlib import Path
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from auto_base_enable import AutoBaseEnable


def owner(**changes):
    return dict(dict(mode='HOLD', armed=False, selected_source='auto',
        selected_topic='/cmd_vel', reason='publisher_conflict_or_missing',
        generation=7, stop_pending=False, stop_confirmed=True,
        hardware_writes=True, feedback_source='hardware', feedback_age_ms=3.,
        command_age_ms=20., requested_ros_velocity=[0., 0., 0.]), **changes)


def policy(**options):
    calls, messages = [], []
    def request():
        future = Future()
        calls.append(future)
        return future
    return AutoBaseEnable(request, messages.append, started=10., **options), calls, messages


def step(p, *, now=10.1, status=None, age=.02, publishers=1, idle=True):
    p.tick(now, status=owner() if status is None else status, age=age,
           publishers=publishers, idle=idle, service_ready=True)


def test_recovers_later_hold_without_repeating_while_armed():
    p, calls, _ = policy()
    step(p)
    assert len(calls) == 1
    step(p, now=10.15)
    assert len(calls) == 1
    calls[0].set_result(SimpleNamespace(success=True, message='enabled'))
    step(p, now=10.2)
    assert not p.done
    step(p, now=11., status=owner(armed=True, mode='MODEL', generation=8))
    assert len(calls) == 1
    step(p, now=11.3, status=owner(generation=9))
    assert len(calls) == 2


@pytest.mark.parametrize('changes', [dict(stop_pending=True), dict(stop_confirmed=False),
    dict(feedback_age_ms=51.), dict(command_age_ms=201.),
    dict(requested_ros_velocity=[.01, 0., 0.]), dict(hardware_writes=False),
    dict(feedback_source='mock')])
def test_waits_for_confirmed_stop_fresh_hardware_and_zero_command(changes):
    p, calls, _ = policy()
    step(p, status=owner(**changes))
    assert not calls
    step(p, now=10.2)
    assert len(calls) == 1


def test_stale_status_and_missing_publisher_cannot_enable():
    p, calls, _ = policy()
    step(p, age=.6)
    step(p, publishers=0)
    assert not calls


@pytest.mark.parametrize('changes', [dict(mode='FAULT'), dict(selected_source='joystick'),
    dict(reason='disabled')])
def test_fault_manual_disable_or_other_source_does_not_rearm(changes):
    p, calls, _ = policy()
    step(p, status=owner(**changes))
    step(p, now=10.2)
    assert p.done and not calls


def test_duplicate_publisher_blocks_until_conflict_clears():
    p, calls, _ = policy()
    step(p, publishers=2)
    step(p)
    assert not p.done and len(calls) == 1


def test_initial_desktop_enable_is_not_raced_or_repeated():
    p, calls, _ = policy()
    for reason in ('startup_stop', 'not_enabled', 'source_switch'):
        step(p, status=owner(reason=reason))
    step(p, status=owner(armed=True, mode='MODEL', reason='command_accepted'))
    assert not p.done and not calls
    step(p, now=11., status=owner(reason='command_timeout', generation=8))
    assert len(calls) == 1


def test_transient_refusals_retry_with_one_second_backoff():
    p, calls, messages = policy()
    step(p)
    calls[0].set_result(SimpleNamespace(success=False, message='fresh_zero_command_required'))
    step(p, now=10.2)
    step(p, now=10.3)
    assert len(calls) == 1
    step(p, now=11.2)
    assert len(calls) == 2
    calls[1].set_result(SimpleNamespace(success=False, message='robot_not_ready'))
    step(p, now=11.3)
    step(p, now=11.4)
    assert not p.done and len(calls) == 2
    assert 'robot_not_ready' in messages[-1]


def test_keeps_waiting_and_never_enables_during_active_goal():
    p, calls, _ = policy()
    step(p, now=70.1)
    assert not p.done and len(calls) == 1
    p, calls, _ = policy()
    step(p, idle=False)
    assert not p.done and not calls
    step(p, now=11.)
    assert len(calls) == 1


def test_stale_pre_enable_status_cannot_trigger_another_request():
    p, calls, _ = policy()
    step(p)
    calls[0].set_result(SimpleNamespace(success=True, message='enabled'))
    step(p, now=10.2)
    step(p, now=12.)  # Old generation 7 still says HOLD.
    assert len(calls) == 1


def test_uncertain_service_timeout_does_not_send_overlapping_requests():
    p, calls, _ = policy()
    step(p)
    step(p, now=12.2)
    step(p, now=14.)
    assert p.done and len(calls) == 1


@pytest.mark.parametrize('reason', ['disabled', 'startup_stop', 'not_enabled', 'source_switch'])
def test_basic_launch_enables_initial_stopped_owner_once(reason):
    p, calls, _ = policy(enable_on_start=True)
    step(p, status=owner(reason=reason, command_age_ms=None))
    assert not calls
    step(p, status=owner(reason=reason))
    assert len(calls) == 1
    calls[0].set_result(SimpleNamespace(success=True, message='enabled'))
    step(p, now=10.2)
    step(p, now=10.3, status=owner(reason=reason))
    assert not p.done  # Pre-enable status may still be in flight.
    step(p, now=10.4, status=owner(mode='MODEL', armed=True, generation=8))
    step(p, now=12., status=owner(reason='disabled', generation=9))
    assert p.done and len(calls) == 1


def test_basic_launch_does_not_override_new_disable_while_waiting():
    p, calls, _ = policy(enable_on_start=True)
    step(p, status=owner(reason='disabled', command_age_ms=None))
    step(p, status=owner(reason='disabled', generation=8))
    assert p.done and not calls


def test_basic_launch_consumes_initial_permission_if_already_armed():
    p, calls, _ = policy(enable_on_start=True)
    step(p, status=owner(mode='MODEL', armed=True))
    step(p, status=owner(reason='disabled', generation=8))
    assert p.done and not calls
