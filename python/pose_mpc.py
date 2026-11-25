"""No-obstacle pose MPC boundary checks and terminal angular envelope."""
import math
import numpy as np


def limit_goal_yaw_rate(rate, pose, goal, target_yaw):
    if not all(math.isfinite(v) for v in (rate, *pose, *goal, target_yaw)):
        raise ValueError('nonfinite terminal yaw input')
    error=math.atan2(math.sin(target_yaw-pose[2]), math.cos(target_yaw-pose[2]))
    if abs(error)<=.05:
        return 0.
    distance=math.hypot(goal[0]-pose[0],goal[1]-pose[1])
    cap=min(.1,.03+.07*min(distance/.3,1.),.8*abs(error))
    return max(-cap,min(cap,rate))


def validate_pose_prediction(path, command, previous):
    if len(path.poses)!=21 or len(previous)!=3:
        raise ValueError('invalid pose MPC horizon')
    xyz=np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in path.poses])
    quat=np.array([[p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]
                   for p in path.poses])
    if (not np.isfinite(xyz).all() or not np.isfinite(quat).all()
            or np.any(xyz[:,2]!=0.) or np.any(quat[:,:2]!=0.)
            or np.max(np.abs(np.linalg.norm(quat,axis=1)-1.))>1e-6):
        raise ValueError('invalid pose MPC quaternion/position')
    yaw=np.unwrap(2.*np.arctan2(quat[:,2],quat[:,3]))
    if abs(yaw[0])>1e-6 or np.linalg.norm(xyz[0])>1e-6:
        raise ValueError('pose MPC must start at request origin')
    delta=np.diff(xyz[:,:2],axis=0)/.1
    c,s=np.cos(yaw[:-1]),np.sin(yaw[:-1])
    controls=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1],np.diff(yaw)/.1))
    first=np.array([command.linear.x,command.linear.y,command.angular.z])
    changes=np.diff(np.vstack([previous,controls]),axis=0)
    if (not np.isfinite(first).all() or not np.isfinite(previous).all()
            or np.max(np.abs(first-controls[0]))>1e-6
            or np.max(np.sum(np.abs(controls[:,:2]),axis=1))>.100001
            or np.max(np.abs(controls[:,2]))>.100001
            or np.max(np.abs(changes[:,:2]))>.014001
            or np.max(np.abs(changes[:,2]))>.020001):
        raise ValueError('pose MPC command/prediction bounds disagree')
    return yaw
