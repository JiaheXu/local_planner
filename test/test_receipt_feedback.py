import math
import numpy as np
import pytest
from test_navigation_feedback import odometry
from test_collision_feedback import cloud
from receipt_feedback import NavigationFeedback, CollisionCloud, validate_navigation


def sensors():
    nav = NavigationFeedback()
    return nav, CollisionCloud('front', np.eye(4), nav)


@pytest.mark.parametrize('odom_stamp,cloud_stamp', [(47_000_000_000, 47_800_000_000),
    (1_000_000_000, 100_000_000_000), (100_000_000_000, 1_000_000_000)])
def test_receipt_age_ignores_pc_offset_and_pairwise_skew(odom_stamp, cloud_stamp):
    nav, scan = sensors()
    nav.receive(odometry(odom_stamp), now=5., wall_ns=10_000_000_000)
    scan.receive(cloud(stamp=cloud_stamp), now=5.)
    assert validate_navigation(nav.fields(5.))
    assert scan.problem(5., nav.stamp) is None


@pytest.mark.parametrize('elapsed,valid', [(0., True), (1.5, True), (2., True),
    (2.000001, False), (-.001, False)])
def test_independent_receipt_timeout_boundaries(elapsed, valid):
    nav, scan = sensors()
    nav.receive(odometry(47_000_000_000), now=5., wall_ns=10_000_000_000)
    scan.receive(cloud(stamp=47_800_000_000), now=5.)
    if valid:
        assert validate_navigation(nav.fields(5.+elapsed))
        assert scan.problem(5.+elapsed, nav.stamp) is None
    else:
        with pytest.raises(ValueError): validate_navigation(nav.fields(5.+elapsed))
        assert scan.problem(5.+elapsed, nav.stamp)


def test_one_stream_cannot_refresh_the_other():
    nav, scan = sensors()
    nav.receive(odometry(47_000_000_000), now=5., wall_ns=10_000_000_000)
    scan.receive(cloud(stamp=47_800_000_000), now=5.)
    nav.receive(odometry(49_000_000_000), now=7.1, wall_ns=12_000_000_000)
    assert validate_navigation(nav.fields(7.1))
    assert scan.problem(7.1, nav.stamp)


def test_bad_frame_and_nonfinite_pose_do_not_refresh_valid_data():
    nav, scan = sensors()
    msg = odometry(47_000_000_000); msg.pose.pose.position.x = math.nan
    nav.receive(msg, now=5., wall_ns=10_000_000_000)
    scan.receive(cloud(stamp=47_800_000_000, frame='wrong'), now=5.)
    with pytest.raises(ValueError): validate_navigation(nav.fields(5.))
    assert scan.problem(5., 0)


def test_per_stream_replay_and_regression_still_rejected():
    nav, scan = sensors()
    nav.receive(odometry(47_000_000_000), now=5., wall_ns=10_000_000_000)
    scan.receive(cloud(stamp=47_800_000_000), now=5.)
    nav.receive(odometry(46_000_000_000), now=5.1, wall_ns=10_000_000_000)
    scan.receive(cloud(stamp=47_800_000_000), now=5.1)
    with pytest.raises(ValueError): validate_navigation(nav.fields(5.1))
    assert scan.problem(5.1, 0)
