import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from planner_diagnostics import PlannerDiagnostics


def sample():
    return dict(status=dict(active=True, phase='drive', reason='C++ MPC tracking',
                            command=[-.03,0.,0.], collision_checks_enabled=False, mpc_solve_ms=1.),
                owner=dict(mode='MODEL', armed=True, selected_source='auto',
                           measured_velocity_sdk=[0.,0.,0.], mapped_sdk_velocity=[-.03,0.,0.],
                           feedback_age_ms=2., reason='command_accepted'),
                nav=dict(position_navigation=[0.,0.,0.], navigation_age_ms=10., navigation_error=None),
                goal=[-.1,0.], lidar_age_ms=20., owner_age_ms=30.)


def test_reports_one_compact_command_and_status_line():
    lines=[]
    PlannerDiagnostics(lines.append).emit(10., **sample())
    assert lines == ['[local+planner]: {cmd_vel} [-0.030,0.000,0.000], '
                     'status{drive; C++ MPC tracking}']


def test_throttles_steady_state_but_reports_transition_immediately():
    lines=[]; report=PlannerDiagnostics(lines.append); data=sample()
    report.emit(10., **data)
    count=len(lines)
    report.emit(10.1, **data)
    assert len(lines)==count
    data['status'].update(active=False,phase='idle',reason='C++ MPC command expired')
    report.emit(10.2, **data)
    assert len(lines)>count and 'idle; waiting for next goal' in lines[-1]
    count=len(lines)
    report.emit(11.3, **data)
    assert len(lines)>count


def test_missing_inputs_are_readable_and_do_not_claim_ready():
    lines=[]
    PlannerDiagnostics(lines.append).emit(10., status={'phase':'idle','reason':'idle'},
        owner={}, nav={'navigation_error':'waiting for odometry'}, goal=None,
        lidar_age_ms=None,owner_age_ms=None)
    output='\n'.join(lines)
    assert lines == ['[local+planner]: {cmd_vel} n/a, status{idle; waiting for odometry}']


def test_rearmed_owner_does_not_display_old_owner_failure_as_current():
    lines=[]
    data=sample()
    data['status'].update(active=False, phase='idle',
        reason='automatic base owner must be armed with fresh hardware feedback')
    PlannerDiagnostics(lines.append).emit(10., **data)
    assert 'status{idle; waiting for next goal}' in lines[0]


def test_completed_cancelled_or_failed_goals_do_not_look_permanently_stopped():
    for reason in ('operator cancelled', 'goal pose reached within 0.03 m and 0.05 rad',
                   'C++ MPC result missing: first result after goal/phase reset'):
        lines=[]
        data=sample()
        data['status'].update(active=False, phase='idle', reason=reason, command=[0.,0.,0.])
        PlannerDiagnostics(lines.append).emit(10., **data)
        assert lines == ['[local+planner]: {cmd_vel} [0.000,0.000,0.000], '
                         'status{idle; waiting for next goal}']
        assert data['status']['reason'] == reason  # History remains on the status topic.


def test_disabled_owner_shows_current_reason():
    lines=[]
    data=sample()
    data['status'].update(active=False, phase='idle', reason='idle')
    data['owner'].update(armed=False, mode='HOLD', reason='publisher_conflict_or_missing')
    PlannerDiagnostics(lines.append).emit(10., **data)
    assert 'status{idle; base disabled (publisher_conflict_or_missing)}' in lines[0]
