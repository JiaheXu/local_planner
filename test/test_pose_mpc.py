"""Pose trajectory validation and terminal velocity envelopes, no ROS nodes."""
import math
import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


def trajectory():
    path=Path();p=np.zeros(3)
    for i in range(21):
        item=PoseStamped();item.pose.position.x=float(p[0]);item.pose.position.y=float(p[1])
        item.pose.orientation.z=math.sin(p[2]/2);item.pose.orientation.w=math.cos(p[2]/2)
        path.poses.append(item)
        p+=np.array([math.cos(p[2])*.01,math.sin(p[2])*.01,.02])*.1
    command=Twist();command.linear.x=.01;command.angular.z=.02
    return path,command


def test_pose_prediction_validates_rotating_body_velocities():
    from pose_mpc import validate_pose_prediction
    path,command=trajectory()
    yaw=validate_pose_prediction(path,command,(0.,0.,0.))
    assert yaw[-1]==pytest.approx(.04)


@pytest.mark.parametrize('bad',['yaw_command','quaternion','angular_jump','xy_command','nan'])
def test_bad_pose_prediction_rejected(bad):
    from pose_mpc import validate_pose_prediction
    path,command=trajectory()
    if bad=='yaw_command':command.angular.z=.03
    elif bad=='quaternion':path.poses[2].pose.orientation.w=.5
    elif bad=='angular_jump':
        path.poses[2].pose.orientation.z=math.sin(.1)
        path.poses[2].pose.orientation.w=math.cos(.1)
    elif bad=='xy_command':command.linear.x=.02
    else:path.poses[2].pose.orientation.z=float('nan')
    with pytest.raises(ValueError):validate_pose_prediction(path,command,(0.,0.,0.))


def test_angular_cap_tapers_with_position_and_heading_without_stalling_rotation():
    from pose_mpc import limit_goal_yaw_rate
    far=limit_goal_yaw_rate(.1,(0.,0.,0.),(1.,0.),1.)
    near=limit_goal_yaw_rate(.1,(0.,0.,0.),(.05,0.),1.)
    at_goal=limit_goal_yaw_rate(.1,(0.,0.,0.),(0.,0.),1.)
    small_error=limit_goal_yaw_rate(.1,(0.,0.,0.),(1.,0.),.06)
    assert far==.1 and 0.<at_goal<near<far and small_error==pytest.approx(.048)
    assert limit_goal_yaw_rate(.1,(0.,0.,math.pi-.01),(0.,0.),-math.pi+.01)==0.
