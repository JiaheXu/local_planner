"""Full-cloud inputs, pose MPC admission and blocked-goal behavior."""
import math
from types import SimpleNamespace as NS
import numpy as np
import pytest
from test_astribot_local_mpc import boundary


def test_all_filtered_returns_survive_projection_even_at_same_bearing():
    from local_planner_core import obstacle_points
    xyz=np.array([[.8,0.,.1],[1.,0.,.6],[1.2,0.,1.6],
                  [1.,0.,0.],[1.,0.,2.],[.1,0.,.5],[np.nan,0.,1.]])
    np.testing.assert_allclose(obstacle_points(xyz,np.eye(4)),[[.8,0.],[1.,0.],[1.2,0.]])


def test_obstacle_pose_request_keeps_every_point_and_yaw(boundary):
    from local_planner_node import LocalPlanner
    from local_planner.msg import MpcRequest
    from builtin_interfaces.msg import Time
    state,_,_=boundary
    sent=[];state.ignore_lidar=False
    state.sequence.begin((1.,0.),.8,concurrent_yaw=True)
    state.points=np.column_stack((np.linspace(.8,2.,100),np.ones(100)))
    state.last_yaw_rate=.03;state.mpc_request_stamp=0;state.mpc_request_type=MpcRequest
    state.get_clock=lambda:NS(now=lambda:NS(nanoseconds=2_000_000_000,to_msg=lambda:Time(sec=2)))
    state.mpc_requests_pub=NS(publish=sent.append)
    LocalPlanner.send_mpc_request(state,(0.,0.,0.),10.)
    assert sent[0].plan_yaw and sent[0].goal_yaw==pytest.approx(.8)
    assert sent[0].previous_yaw_rate==.03
    np.testing.assert_allclose([[p.x,p.y] for p in sent[0].obstacles],state.points)


@pytest.mark.parametrize('previous_yaw',[0.,.1,-.1])
def test_blocked_pose_result_stops_and_keeps_goal(boundary,previous_yaw):
    from local_planner_node import LocalPlanner
    state,result,commands=boundary
    state.ignore_lidar=False;state.last_yaw_rate=previous_yaw
    state.sequence.begin((1.,0.),1.,concurrent_yaw=True)
    state.mpc_requests[1000000000]=((0.,0.,0.),10.,1,(0.,0.,0.))
    result.blocked=True;result.command.linear.x=0.;result.path.poses=result.path.poses[:1]
    state.mpc_result(result)
    assert state.session.goal==(1.,0.)
    LocalPlanner.tick(state)
    assert state.session.goal==(1.,0.)
    assert (commands[-1].linear.x,commands[-1].linear.y,commands[-1].angular.z)==(0.,0.,0.)


@pytest.mark.parametrize('previous_yaw',[0.,.1,-.1])
def test_obstacle_pose_waiting_does_not_use_proportional_yaw(boundary,previous_yaw):
    from local_planner_node import LocalPlanner
    state,_,commands=boundary
    state.ignore_lidar=False;state.last_yaw_rate=previous_yaw
    state.sequence.begin((1.,0.),1.,concurrent_yaw=True)
    LocalPlanner.tick(state)
    assert commands[-1].angular.z==0.


def test_collision_check_catches_obstacle_between_prediction_samples():
    from local_planner_core import path_collision_free
    path=np.array([[0.,0.],[.2,0.]])
    assert not path_collision_free(path,np.array([[.1,.549]]),clearance=.55)
    assert path_collision_free(path,np.array([[.1,.56]]),clearance=.55)
    assert not path_collision_free(path,np.array([[np.nan,0.]]),clearance=.55)


def test_projected_cloud_preserves_stamp_and_all_xyz_points():
    from local_planner_core import projected_obstacle_cloud,cloud_xyz,FRAME
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    header=Header(stamp=Time(sec=12,nanosec=34),frame_id='livox_frame')
    result=projected_obstacle_cloud(np.array([[1.,2.],[3.,4.]]),header)
    assert result.header.stamp==header.stamp and result.header.frame_id==FRAME
    np.testing.assert_allclose(cloud_xyz(result),[[1.,2.,0.],[3.,4.,0.]])
