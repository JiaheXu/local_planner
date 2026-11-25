"""Astribot geometry and bounded local following; no ROS or SDK ownership."""
import math
from pathlib import Path

import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory

FRAME = 'astribot_viz_chassis'


def front_mount():
    path = Path(get_package_share_directory('local_planner')) / 'config/astribot_geometry.yaml'
    pose = yaml.safe_load(path.read_text())['base_from_front_lidar']
    xyz, quat = np.asarray(pose['xyz'], float), np.asarray(pose['xyzw'], float)
    if (xyz.shape != (3,) or quat.shape != (4,) or not np.isfinite(xyz).all()
            or not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1.) > 1e-5):
        raise ValueError('Invalid front mount: expected metres and unit xyzw quaternion')
    x, y, z, w = quat / np.linalg.norm(quat)
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = xyz
    return result


def rear_cloud_mount():
    """Driver-transformed rear payload, not the physical rear sensor mount."""
    path = Path(get_package_share_directory('local_planner')) / 'config/astribot_geometry.yaml'
    coordinates = yaml.safe_load(path.read_text()).get('rear_cloud_coordinates')
    if coordinates != 'front_lidar':
        raise ValueError('Unqualified rear cloud coordinates; expected front_lidar contract')
    return front_mount()


def cloud_xyz(cloud):
    """Read explicit XYZ float32 fields, including organized rows and byte order."""
    if (not 0 < cloud.width * cloud.height <= 500000 or cloud.point_step < 12
            or cloud.row_step < cloud.width * cloud.point_step
            or len(cloud.data) != cloud.height * cloud.row_step):
        raise ValueError('invalid/empty PointCloud2 layout')
    fields = {f.name: f for f in cloud.fields}
    columns = []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if (field is None or field.datatype != 7 or field.count != 1
                or not 0 <= field.offset <= cloud.point_step - 4):
            raise ValueError('expected scalar FLOAT32 XYZ fields')
        array = np.ndarray((cloud.height, cloud.width),
                           dtype='>f4' if cloud.is_bigendian else '<f4', buffer=cloud.data,
                           offset=field.offset, strides=(cloud.row_step, cloud.point_step))
        columns.append(array.reshape(-1))
    return np.column_stack(columns)


def obstacle_points(xyz, mount):
    xyz = np.asarray(xyz)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz):
        raise ValueError('empty/invalid cloud')
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    # Livox invalid returns may be finite zeros, not NaNs. They are not free-space evidence.
    xyz = xyz[np.linalg.norm(xyz, axis=1) >= .05]
    if not len(xyz):
        raise ValueError('cloud contains no usable sensor returns')
    base = xyz @ mount[:3, :3].T + mount[:3, 3]
    distance = np.hypot(base[:, 0], base[:, 1])
    # Nominal chassis-only self filter. Extended arms require separate qualification.
    keep = (base[:, 2] >= .03) & (base[:, 2] <= 1.8) & (distance >= .30) & (distance <= 4.)
    return base[keep, :2]


def relative_goal(goal, pose):
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy])


def world_path(path, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(path) @ np.array([[c, s], [-s, c]]) + np.asarray(pose[:2])


def follow(goal, pose, path):
    error = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
    if error <= .03:
        return 0., 0., True
    if len(path) < 2 or not np.isfinite(path).all():
        return 0., 0., False
    # Path is anchored to its acquisition pose; never subtract map pose from a local path.
    nearest = int(np.argmin(np.linalg.norm(path - np.asarray(pose[:2]), axis=1)))
    target = path[-1]
    for point in path[nearest:]:
        if math.hypot(point[0] - pose[0], point[1] - pose[1]) >= .15:
            target = point
            break
    local = relative_goal(target, pose)
    angle = math.atan2(local[1], local[0])
    yaw = max(-.1, min(.1, angle))
    speed = .05 if abs(angle) <= .35 else 0.
    return speed, yaw, False


def collision_free(vx, wz, points, *, vy=0., clearance=.40, horizon=2.):
    """Check the commanded sweep; default is 2 s at a 0.40 m chassis margin."""
    if (not all(math.isfinite(x) for x in (vx, vy, wz, clearance, horizon))
            or clearance <= 0 or horizon <= 0 or not np.isfinite(points).all()):
        return False
    for t in np.linspace(0., horizon, 21):
        center = ([vx * t, vy * t] if abs(wz) < 1e-8 else
                  [(vx * math.sin(wz * t) + vy * (math.cos(wz * t) - 1)) / wz,
                   (vx * (1 - math.cos(wz * t)) + vy * math.sin(wz * t)) / wz])
        if len(points) and np.any(np.linalg.norm(points - center, axis=1) <= clearance):
            return False
    return True


def limit_goal_velocity(velocity, pose, goal, tolerance=.03):
    """Cap final XY speed at 0.6/s times distance (m), zero inside tolerance."""
    if (len(velocity) != 2 or len(pose) != 3 or len(goal) != 2
            or not all(math.isfinite(v) for v in (*velocity, *pose, *goal, tolerance))
            or tolerance <= 0.):
        raise ValueError('invalid terminal velocity/goal')
    distance = math.hypot(goal[0]-pose[0], goal[1]-pose[1])
    speed = math.hypot(*velocity)
    if distance <= tolerance or speed == 0.:
        return (0., 0.)
    # Preserve direction and the optimizer's L1 speed envelope after rotation.
    scale = min(1., min(.1, .6*distance)/speed, .1/sum(map(abs, velocity)))
    return tuple(float(v)*scale for v in velocity)


class GoalSequence:
    """Translate to the goal, optionally face it first, then align map-frame yaw."""
    POSITION_TOLERANCE = .03
    YAW_TOLERANCE = .05

    def __init__(self):
        self.cancel()

    def cancel(self):
        self.goal, self.target_yaw, self.phase = None, None, 'idle'
        self.reverse = False
        self.face_goal = True
        self.concurrent_yaw = False

    def begin(self, goal, target_yaw, *, reverse=False, face_goal=True, concurrent_yaw=False):
        if len(goal) != 2 or not all(math.isfinite(v) for v in (*goal, target_yaw)):
            raise ValueError('invalid goal position or yaw')
        self.goal, self.target_yaw = tuple(goal), target_yaw
        self.reverse = reverse
        self.face_goal = face_goal
        self.concurrent_yaw = concurrent_yaw
        self.phase = 'rotate_to_goal' if face_goal and not concurrent_yaw else 'drive'

    def update(self, pose):
        if self.goal is None:
            return 'idle', 0., False
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError('invalid navigation pose')
        dx, dy = self.goal[0] - pose[0], self.goal[1] - pose[1]
        near = math.hypot(dx, dy) <= self.POSITION_TOLERANCE
        if self.concurrent_yaw:
            error = math.atan2(math.sin(self.target_yaw-pose[2]), math.cos(self.target_yaw-pose[2]))
            aligned = abs(error) <= self.YAW_TOLERANCE
            self.phase = 'done' if near and aligned else 'drive'
            return self.phase, 0. if aligned else max(-.1, min(.1, error)), near and aligned
        if self.phase == 'rotate_to_yaw' and not near:
            self.phase = 'rotate_to_goal' if self.face_goal else 'drive'
            return self.phase, 0., False
        if near:
            self.phase = 'rotate_to_yaw'
        if self.phase == 'drive':
            return 'drive', 0., False
        desired = self.target_yaw if near or self.reverse else math.atan2(dy, dx)
        error = math.atan2(math.sin(desired - pose[2]), math.cos(desired - pose[2]))
        if abs(error) <= self.YAW_TOLERANCE:
            self.phase = 'done' if near else 'drive'
            return self.phase, 0., near
        return self.phase, max(-.1, min(.1, error)), False


class PathSession:
    def __init__(self):
        self.goal = None
        self.path = np.empty((0, 2))
        self.epoch = 0
        self.last_stamp = 0
        self.received = -math.inf
        self.reason = 'idle'

    def begin(self, goal, source_stamp):
        self.cancel('waiting for a new path')
        self.goal, self.epoch = tuple(goal), source_stamp
        self.last_stamp = source_stamp

    def cancel(self, reason):
        self.goal = None
        self.path = np.empty((0, 2))
        self.received, self.reason = -math.inf, reason

    def accept(self, stamp, path, now):
        if self.goal is None or stamp <= self.last_stamp or not np.isfinite(path).all():
            return False
        self.last_stamp = stamp
        self.path, self.received = np.asarray(path), now
        self.reason = 'following' if len(path) >= 2 else 'blocked: no local path'
        return True

    def valid(self, now):
        return self.goal is not None and 0 <= now - self.received <= .3
