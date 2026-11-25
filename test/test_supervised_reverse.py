from types import SimpleNamespace
import pytest
from local_planner_core import GoalSequence
from local_planner_node import LocalPlanner


def test_reverse_goal_keeps_heading_and_uses_drive():
    sequence = GoalSequence()
    sequence.begin((-.1, 0.), 0., reverse=True)
    assert sequence.update((0., 0., 0.)) == ('drive', 0., False)
    assert sequence.update((-.1, 0., 0.)) == ('done', 0., True)


def test_normal_goal_behind_still_rotates():
    sequence = GoalSequence(); sequence.begin((-.1, 0.), 0.)
    assert sequence.update((0., 0., 0.))[0] == 'rotate_to_goal'


def test_test_mode_caps_translation():
    state = SimpleNamespace(supervised_reverse_test=True, last_translation=(.03, 0.), last_command_time=10.)
    x, y = LocalPlanner.bounded_mpc_command(state, .1, .1, 10.1)
    assert abs(x)+abs(y) <= .030001


@pytest.mark.parametrize('local,yaw,accepted', [((-.1,0.),0.,True),
    ((-.2,0.),0.,False), ((.1,0.),0.,False), ((-.1,.03),0.,False), ((-.1,0.),1.,False)])
def test_test_goal_is_short_straight_reverse_only(local,yaw,accepted):
    if accepted: LocalPlanner.validate_reverse_test_goal(local,yaw)
    else:
        with pytest.raises(ValueError): LocalPlanner.validate_reverse_test_goal(local,yaw)
