import math
from types import SimpleNamespace as NS

import pytest

from navigation_feedback import NavigationFeedback, validate_navigation


def odometry(stamp):
    sec, ns = divmod(stamp, 10**9)
    return NS(header=NS(frame_id='map', stamp=NS(sec=sec, nanosec=ns)),
              child_frame_id='astribot_viz_chassis',
              pose=NS(pose=NS(position=NS(x=1., y=2., z=0.),
                              orientation=NS(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5)))))


def sample(nav, source_ms, receipt_ms):
    stamp = 100_000_000_000 + source_ms * 10**6
    nav.receive(odometry(stamp), now=receipt_ms/1000, wall_ns=stamp-37*10**9)


def prime():
    nav = NavigationFeedback()
    for ms in (0, 50, 100, 150):
        sample(nav, ms, ms)
        assert nav.fields(ms/1000)['position_navigation'] is None
    sample(nav, 200, 200)
    return nav


def test_odometry_alone_qualifies_despite_37_second_offset_then_expires():
    nav = prime()
    assert validate_navigation(nav.fields(.45)) == pytest.approx((1., 2., math.pi/2))
    assert nav.fields(.2)['navigation_pc_minus_source_ms'] == -37000.
    with pytest.raises(ValueError, match='stale odometry'):
        validate_navigation(nav.fields(.450001))


def test_missing_odometry_has_no_age():
    assert NavigationFeedback().fields(180650.)['navigation_age_ms'] is None


@pytest.mark.parametrize('defect', ['frame', 'child', 'quaternion', 'position', 'orientation'])
def test_invalid_sample_cannot_advance_reference(defect):
    nav = prime()
    msg = odometry(100_250_000_000)
    if defect == 'frame': msg.header.frame_id = 'odom'
    elif defect == 'child': msg.child_frame_id = 'other'
    elif defect == 'quaternion': msg.pose.pose.orientation.w = 2.
    elif defect == 'position': msg.pose.pose.position.z = math.nan
    else: msg.pose.pose.orientation.x = math.inf
    nav.receive(msg, now=.25, wall_ns=100_250_000_000)
    assert nav.stamp == nav.reference_stamp == 100_200_000_000
    with pytest.raises(ValueError): validate_navigation(nav.fields(.25))


@pytest.mark.parametrize('source,receipt', [(199,220), (201,199), (201,math.nan), (700,220)])
def test_qualified_clock_fault_latches(source, receipt):
    nav = prime()
    sample(nav,source,receipt)
    sample(nav,750,750)
    assert nav.clock_fault
    with pytest.raises(ValueError): validate_navigation(nav.fields(.75))


def test_duplicate_and_sparse_delivery_require_full_recovery():
    nav = prime()
    sample(nav,200,220)
    assert not nav.clock_fault and nav.reference_received == .2
    with pytest.raises(ValueError, match='non-advancing'): validate_navigation(nav.fields(.22))
    for ms in (250, 350, 449):
        sample(nav,ms,ms)
        assert nav.fields(ms/1000)['position_navigation'] is None
    sample(nav,450,450)
    assert validate_navigation(nav.fields(.45))
    sample(nav,751,751)
    assert nav.fields(.751)['position_navigation'] is None
    sample(nav,851,851)
    sample(nav,951,951)
    assert validate_navigation(nav.fields(.951))


def test_initial_backlog_requires_fresh_settling_period():
    nav = NavigationFeedback()
    sample(nav,0,0)
    sample(nav,500,5)
    sample(nav,600,105)
    assert nav.fields(.105)['position_navigation'] is None
    sample(nav,700,205)
    assert validate_navigation(nav.fields(.205))


def test_delayed_replay_never_rebases_and_catchup_recovers():
    nav = prime()
    for ms in range(500,1001,50):
        sample(nav,ms-280,ms)
        with pytest.raises(ValueError, match='delayed'): validate_navigation(nav.fields(ms/1000))
    for ms in (1050,1150,1249):
        sample(nav,ms,ms)
        assert nav.fields(ms/1000)['position_navigation'] is None
    sample(nav,1250,1250)
    assert validate_navigation(nav.fields(1.25))


def test_timing_recovery_does_not_resume_cancelled_goal(monkeypatch):
    from types import SimpleNamespace as NS, MethodType
    import local_planner_node as module
    from local_planner_core import GoalSequence,PathSession
    feedback = prime()
    sample(feedback, 210, 510)
    commands=[];transport=NS(publish=lambda msg:None)
    state=NS(nav=feedback,collision_inputs={},selected_lidar='rear',collision_checks_enabled=True,
        session=PathSession(),sequence=GoalSequence(),snapshots={},source_ids=None,
        owner_generation=None,owner_status={},dispatch=False,backend='mpc',mpc_epoch=1,
        mpc_solve_ms=None,commands=NS(publish=commands.append),cancels=transport,paths=transport,
        status=transport,output_topic='shadow')
    state.session.begin((2.,2.),100_000_000_000);state.sequence.begin((2.,2.),0.)
    for method in ('ready','cancel','reset_mpc'):
        setattr(state,method,MethodType(getattr(module.LocalPlanner,method),state))
    monkeypatch.setattr(module.time,'monotonic',lambda:.51)
    module.LocalPlanner.tick(state)
    assert state.session.goal is None and 'odometry delivery delayed' in state.session.reason
    for ms in (550,650,750):
        sample(feedback, ms, ms)
    assert validate_navigation(feedback.fields(.75))
    monkeypatch.setattr(module.time,'monotonic',lambda:.75)
    module.LocalPlanner.tick(state)
    assert state.session.goal is None
    assert commands and all((m.linear.x,m.linear.y,m.angular.z)==(0.,0.,0.) for m in commands)
