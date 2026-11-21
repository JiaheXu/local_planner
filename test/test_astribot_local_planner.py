"""Offline local-planner geometry and control boundary tests."""
from pathlib import Path
import sys
import math

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))


def test_front_mount_uses_model_translation_and_minus_45_degree_yaw():
    from local_planner_core import front_mount
    mount = front_mount()
    np.testing.assert_allclose(mount[:3, 3], [.17393, .16893, .170397])
    np.testing.assert_allclose(mount[:3, 0], [2**-.5, -2**-.5, 0], atol=1e-7)


def test_cloud_transform_filters_floor_and_retains_low_obstacle():
    from local_planner_core import obstacle_points
    mount = np.eye(4)
    mount[:3, 3] = [.2, .1, .2]
    points = np.array([[1., 0., -.2], [1., 0., -.1], [1., 0., 2.], [np.nan, 0., 0.]])
    result = obstacle_points(points, mount)
    np.testing.assert_allclose(result, [[1.2, .1]])
    with pytest.raises(ValueError):
        obstacle_points(np.zeros((0, 3)), mount)


@pytest.mark.parametrize('points', [np.zeros((10, 3)),
    np.array([[0., 0., 0.], [float('nan'), 1., 1.]])])
def test_zero_range_placeholders_cannot_be_treated_as_clear_space(points):
    from local_planner_core import obstacle_points, front_mount
    with pytest.raises(ValueError, match='usable'):
        obstacle_points(points, front_mount())


def test_relative_goal_and_path_following_at_nonzero_map_pose():
    from local_planner_core import relative_goal, world_path, follow
    pose = (3., -2., math.pi / 2)
    np.testing.assert_allclose(relative_goal((3., -1.), pose), [1., 0.], atol=1e-12)
    path = world_path(np.array([[0., 0.], [.2, 0.], [.5, 0.]]), pose)
    np.testing.assert_allclose(path[-1], [3., -1.5])
    vx, wz, arrived = follow((3., -1.), pose, path)
    assert vx == .05 and abs(wz) < 1e-10 and not arrived


def test_arrival_uses_actual_goal_not_short_local_path_endpoint():
    from local_planner_core import follow
    path = np.array([[0., 0.], [.1, 0.]])
    assert follow((.2, 0.), (0., 0., 0.), path)[0] == .05
    assert follow((.2, 0.), (.169, 0., 0.), path)[2] is False
    assert follow((.2, 0.), (.171, 0., 0.), path) == (0., 0., True)


def test_turning_slows_translation_and_caps_yaw():
    from local_planner_core import follow
    vx, wz, arrived = follow((0., 1.), (0., 0., 0.), np.array([[0., 0.], [0., .3]]))
    assert vx == 0. and wz == .1 and not arrived


@pytest.mark.parametrize('obstacle', [[.45, 0.], [.2, .2], [.1, -.3]])
def test_swept_command_stops_before_footprint_hits_obstacle(obstacle):
    from local_planner_core import collision_free
    assert not collision_free(.05, 0., np.array([obstacle]))


def test_distant_obstacle_and_clear_side_are_not_false_stop():
    from local_planner_core import collision_free
    assert collision_free(.05, 0., np.array([[2., 0.], [.2, .6]]))


def test_guard_cancel_stale_path_and_goal_epoch():
    from local_planner_core import PathSession
    session = PathSession()
    session.begin((.2, 0.), 100)
    assert not session.accept(99, np.array([[0., 0.], [.1, 0.]]), 10.)
    assert session.accept(101, np.array([[0., 0.], [.1, 0.]]), 10.)
    assert session.valid(10.1)
    assert not session.valid(10.4)
    session.cancel('operator cancel')
    assert not session.accept(102, np.array([[0., 0.], [.1, 0.]]), 10.5)
    assert session.goal is None


def test_cloud_parser_honors_row_padding_and_rejects_bad_schema():
    from sensor_msgs.msg import PointCloud2, PointField
    from local_planner_core import cloud_xyz
    cloud = PointCloud2()
    cloud.width, cloud.height, cloud.point_step, cloud.row_step = 1, 2, 12, 16
    cloud.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                    for i, name in enumerate(('x', 'y', 'z'))]
    cloud.data = np.array([1, 2, 3, 999, 4, 5, 6, 999], dtype='<f4').tobytes()
    np.testing.assert_allclose(cloud_xyz(cloud), [[1, 2, 3], [4, 5, 6]])
    cloud.fields[0].datatype = 8
    with pytest.raises(ValueError):
        cloud_xyz(cloud)


def test_older_inflight_path_cannot_replace_newer_path():
    from local_planner_core import PathSession
    session = PathSession()
    session.begin((1., 0.), 100)
    new = np.array([[0., 0.], [.3, .1]])
    assert session.accept(103, new, 10.)
    assert not session.accept(102, np.array([[0., 0.], [.3, -.1]]), 10.1)
    np.testing.assert_array_equal(session.path, new)


@pytest.mark.parametrize('changes', [dict(armed=False), dict(mode='FAULT'),
    dict(selected_source='joystick'), dict(generation=4), dict(feedback_age_ms=float('nan'))])
def test_dispatch_readiness_rejects_disabled_fault_changed_source_or_generation(changes):
    from types import SimpleNamespace
    from local_planner_node import LocalPlanner
    status = dict(armed=True, mode='MODEL', hardware_writes=True, feedback_source='hardware',
                  stop_pending=False, selected_source='auto', selected_topic='/cmd_vel',
                  feedback_age_ms=1., generation=3)
    state = SimpleNamespace(
        nav=SimpleNamespace(fields=lambda now: dict(position_navigation=[0., 0., 0.],
                            navigation_age_ms=10.), reference_stamp=100, stamp=100),
        front_clock=SimpleNamespace(reference_problem=lambda now: None, reference_stamp=100),
        cloud_fault=False, cloud_error='', cloud_received=10., cloud_stamp=100,
        sources=lambda: ('one',), source_ids=('one',), dispatch=True,
        owner_status=status, owner_received=10., owner_generation=3)
    assert LocalPlanner.ready(state, 10.) == (0., 0., 0.)
    status.update(changes)
    with pytest.raises(ValueError):
        LocalPlanner.ready(state, 10.)


def test_goal_sequence_faces_goal_drives_then_aligns_final_yaw():
    from local_planner_core import GoalSequence
    sequence = GoalSequence()
    sequence.begin((0., 1.), -.5)
    phase, wz, done = sequence.update((0., 0., 0.))
    assert (phase, wz, done) == ('rotate_to_goal', .1, False)
    assert sequence.update((0., 0., math.pi / 2)) == ('drive', 0., False)
    assert sequence.update((0., .98, math.pi / 2)) == ('rotate_to_yaw', -.1, False)
    assert sequence.update((0., .98, -.48)) == ('done', 0., True)


def test_rotation_goal_behind_and_shortest_yaw_wrap():
    from local_planner_core import GoalSequence
    sequence = GoalSequence()
    sequence.begin((-1., 0.), -math.pi + .02)
    phase, wz, done = sequence.update((0., 0., 0.))
    assert phase == 'rotate_to_goal' and abs(wz) == .1 and not done
    assert sequence.update((0., 0., math.pi)) == ('drive', 0., False)
    # Across ±pi, final yaw error is 0.04 rad, not almost a full revolution.
    assert sequence.update((-.98, 0., math.pi - .02)) == ('done', 0., True)


def test_rotation_only_goal_and_position_drift_restart_approach():
    from local_planner_core import GoalSequence
    sequence = GoalSequence()
    sequence.begin((1., 2.), .5)
    assert sequence.update((1., 2., 0.)) == ('rotate_to_yaw', .1, False)
    phase, wz, done = sequence.update((.9, 2., .5))
    assert phase == 'rotate_to_goal' and not done
    assert sequence.update((.9, 2., 0.)) == ('drive', 0., False)


@pytest.mark.parametrize('yaw', [float('nan'), float('inf')])
def test_sequence_rejects_invalid_goal_yaw(yaw):
    from local_planner_core import GoalSequence
    with pytest.raises(ValueError):
        GoalSequence().begin((0., 0.), yaw)


def test_cancelling_rotation_clears_target():
    from local_planner_core import GoalSequence
    sequence = GoalSequence()
    sequence.begin((0., 1.), 0.)
    sequence.cancel()
    assert sequence.update((0., 0., 0.)) == ('idle', 0., False)


def test_rotation_collision_check_covers_chassis_circle():
    from local_planner_core import collision_free
    assert not collision_free(0., .1, np.array([[0., -.35]]))
    assert collision_free(0., .1, np.array([[0., -.6]]))


@pytest.mark.parametrize('goal', [(0., 0.), (0., 1.)])
def test_cloud_pose_skew_stops_both_rotation_stages(monkeypatch, goal):
    from types import SimpleNamespace, MethodType
    import local_planner_node as module
    from local_planner_core import GoalSequence, PathSession
    commands = []
    transport = SimpleNamespace(publish=lambda msg: None)
    state = SimpleNamespace(
        nav=SimpleNamespace(fields=lambda now: dict(position_navigation=[0., 0., 0.],
            navigation_age_ms=10.), reference_stamp=1_250_000_000, stamp=1_210_000_000),
        front_clock=SimpleNamespace(reference_problem=lambda now: None, reference_stamp=1_250_000_000),
        cloud_fault=False, cloud_error='', cloud_received=10., cloud_stamp=1_000_000_000,
        sources=lambda: ('one',), source_ids=('one',), dispatch=False,
        session=PathSession(), sequence=GoalSequence(), snapshots={}, started=10.,
        commands=SimpleNamespace(publish=commands.append), cancels=transport, paths=transport,
        status=transport, points=np.empty((0, 2)), owner_status={}, owner_generation=None,
        output_topic='shadow', backend='library', mpc_epoch=0, mpc_solve_ms=None)
    state.session.begin(goal, state.cloud_stamp)
    state.sequence.begin(goal, .5)
    state.sequence.update((0., 0., 0.))
    assert state.sequence.phase in ('rotate_to_goal', 'rotate_to_yaw')
    state.ready = MethodType(module.LocalPlanner.ready, state)
    state.cancel = MethodType(module.LocalPlanner.cancel, state)
    state.reset_mpc = MethodType(module.LocalPlanner.reset_mpc, state)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    module.LocalPlanner.tick(state)
    assert state.session.goal is None
    assert commands and all(msg.linear.x == 0. and msg.angular.z == 0. for msg in commands)
    assert 'time-aligned' in state.session.reason
