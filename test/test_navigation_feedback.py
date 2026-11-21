import math
import pytest


def navigation_message():
    from types import SimpleNamespace as NS
    return NS(header=NS(frame_id='map', stamp=NS(sec=100, nanosec=0)),
              child_frame_id='astribot_viz_chassis',
              pose=NS(pose=NS(position=NS(x=1., y=2., z=0.),
                             orientation=NS(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5)))))


def test_navigation_uses_live_sensor_clock_despite_pc_offset():
    from navigation_feedback import NavigationFeedback, validate_navigation
    feedback = NavigationFeedback()
    reference = navigation_message()
    feedback.receive_reference(reference, now=9.995)
    reference.header.stamp.nanosec = 5_000_000
    feedback.receive_reference(reference, now=10.)
    feedback.receive(navigation_message(), now=10., wall_ns=63_000_000_000)
    assert validate_navigation(feedback.fields(10.01)) == pytest.approx((1., 2., math.pi/2))
    assert feedback.fields(10.01)['navigation_pc_minus_source_ms'] == -37000.
    with pytest.raises(ValueError, match='reference'):
        validate_navigation(feedback.fields(10.101))


def test_navigation_never_reports_uptime_as_pose_age():
    from navigation_feedback import NavigationFeedback
    feedback = NavigationFeedback()
    assert feedback.fields(180650.)['navigation_age_ms'] is None


def test_navigation_requires_advancing_reference_and_rejects_clock_jump():
    from navigation_feedback import NavigationFeedback, validate_navigation
    feedback = NavigationFeedback()
    reference = navigation_message()
    feedback.receive_reference(reference, now=10.)
    feedback.receive(navigation_message(), now=10., wall_ns=100_000_000_000)
    with pytest.raises(ValueError, match='reference'):
        validate_navigation(feedback.fields(10.))
    reference.header.stamp.nanosec = 5_000_000
    feedback.receive_reference(reference, now=10.005)
    reference.header.stamp.sec += 1
    feedback.receive_reference(reference, now=10.01)
    feedback.receive(navigation_message(), now=10.01, wall_ns=100_000_000_000)
    with pytest.raises(ValueError, match='clock'):
        validate_navigation(feedback.fields(10.01))


def test_navigation_decoder_preserves_world_pose_and_checks_freshness():
    from navigation_feedback import NavigationFeedback, validate_navigation
    feedback = NavigationFeedback()
    reference = navigation_message()
    feedback.receive_reference(reference, now=9.995)
    reference.header.stamp.nanosec = 5_000_000
    feedback.receive_reference(reference, now=10.)
    feedback.receive(navigation_message(), now=10., wall_ns=100_000_000_000)
    assert validate_navigation(feedback.fields(10.1)) == pytest.approx((1., 2., math.pi/2))
    with pytest.raises(ValueError, match='stale'):
        validate_navigation(feedback.fields(10.251))
    # Repeated source stamps must not renew freshness.
    feedback.receive(navigation_message(), now=10.01, wall_ns=100_000_000_000)
    with pytest.raises(ValueError, match='non-advancing'):
        validate_navigation(feedback.fields(10.01))


@pytest.mark.parametrize('invalid', ['frame', 'child', 'old', 'future', 'nan', 'quaternion'])
def test_invalid_navigation_messages_do_not_replace_valid_measurements(invalid):
    from navigation_feedback import NavigationFeedback, validate_navigation
    message = navigation_message()
    if invalid == 'frame': message.header.frame_id = 'odom'
    if invalid == 'child': message.child_frame_id = 'body'
    if invalid == 'old': message.header.stamp.sec = 99
    if invalid == 'future': message.header.stamp.sec = 101
    if invalid == 'nan': message.pose.pose.position.x = float('nan')
    if invalid == 'quaternion': message.pose.pose.orientation.w = 0.
    feedback = NavigationFeedback()
    reference = navigation_message()
    feedback.receive_reference(reference, now=9.995)
    reference.header.stamp.nanosec = 5_000_000
    feedback.receive_reference(reference, now=10.)
    feedback.receive(message, now=10., wall_ns=100_000_000_000)
    with pytest.raises(ValueError, match='navigation'):
        validate_navigation(feedback.fields(10.))


