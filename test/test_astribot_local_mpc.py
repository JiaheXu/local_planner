"""C++ MPC executable and ROS adapter boundary, without hardware."""
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace, MethodType
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))


def test_real_cpp_optimizer_regressions():
    from ament_index_python.packages import get_package_prefix
    binary = Path(get_package_prefix('local_planner')) / 'lib/local_planner/planar_mpc_test'
    result = subprocess.run([str(binary)], text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'PASS: C++ MPC' in result.stdout


@pytest.fixture
def boundary(monkeypatch):
    import local_planner_node as module
    from local_planner_core import PathSession, GoalSequence, FRAME
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped, Twist
    transport = SimpleNamespace(publish=lambda msg: None)
    commands = []
    state = SimpleNamespace(backend='mpc', mpc_epoch=1, mpc_last_stamp=0,
        selected_lidar='front', collision_checks_enabled=True, collision_inputs={}, nav=SimpleNamespace(stamp=1_000_000_000),
        mpc_received=-float('inf'), mpc_wait_started=10., mpc_wait_reason='first result',
        mpc_velocity=(0.,0.), mpc_solve_ms=None,
        mpc_requests={1000000000: ((0.,0.,0.),10.,1,(0.,0.))},
        session=PathSession(), sequence=GoalSequence(), snapshots={}, started=10.,
        commands=SimpleNamespace(publish=commands.append), cancels=transport, paths=transport,
        status=transport, points=np.empty((0,2)), owner_status={}, owner_generation=None,
        source_ids=None, dispatch=False, output_topic='shadow', cloud_stamp=1000000000,
        ready=lambda now: (0.,0.,0.), last_translation=(0.,0.), last_command_time=10.)
    state.session.begin((1.,0.), 1)
    state.sequence.begin((1.,0.),0.)
    state.sequence.update((0.,0.,0.))
    for name in ('reset_mpc','cancel','wait_mpc','mpc_drive','mpc_result','bounded_mpc_command'):
        setattr(state,name,MethodType(getattr(module.LocalPlanner,name),state))
    path=RosPath()
    path.header.frame_id=FRAME
    path.header.stamp.sec=1
    positions=np.concatenate([[0.],np.cumsum(np.minimum(.014*np.arange(1,21),.1))*.1])
    for x in positions:
        pose=PoseStamped()
        pose.header=path.header
        pose.pose.position.x=float(x)
        pose.pose.orientation.w=1.
        path.poses.append(pose)
    result=SimpleNamespace(header=path.header,epoch=1,success=True,blocked=False,error='',
                           solve_ms=1.,command=Twist(),path=path)
    result.command.linear.x=.014
    monkeypatch.setattr(module.time,'monotonic',lambda:10.05)
    return state,result,commands


def test_cpp_result_passes_current_gates(boundary):
    state,result,_=boundary
    state.mpc_result(result)
    assert state.mpc_drive((0.,0.,0.),10.05)==(.014,0.)


def test_pose_mpc_result_drives_yaw_and_cancel_clears_it(boundary):
    import math
    import local_planner_node as module
    state,result,commands=boundary
    state.ignore_lidar=True
    state.sequence.begin((1.,0.),1.,concurrent_yaw=True)
    state.mpc_requests[1000000000]=((0.,0.,0.),10.,1,(0.,0.,0.))
    state.last_yaw_rate=0.
    state.mpc_yaw_rate=0.
    state.mpc_pose_control=False
    state.mpc_reference_yaw=None
    result.command.linear.x=.01;result.command.angular.z=.02
    x=y=yaw=0.
    for item in result.path.poses:
        item.pose.position.x=x;item.pose.position.y=y
        item.pose.orientation.z=math.sin(yaw/2);item.pose.orientation.w=math.cos(yaw/2)
        x+=.001*math.cos(yaw);y+=.001*math.sin(yaw);yaw+=.002
    state.mpc_result(result)
    assert state.session.goal is not None
    assert state.mpc_yaw_rate==.02
    module.LocalPlanner.tick(state)
    assert commands[-1].linear.x>0. and commands[-1].angular.z==pytest.approx(.01)
    state.cancel()
    assert state.mpc_yaw_rate==0. and state.last_yaw_rate==0.
    assert commands[-1].angular.z==0.


def test_pose_mode_never_turns_without_fresh_solver_result(boundary):
    import local_planner_node as module
    state,result,commands=boundary
    state.ignore_lidar=True
    state.sequence.begin((1.,0.),1.,concurrent_yaw=True)
    module.LocalPlanner.tick(state)
    assert commands[-1].angular.z==0.


def test_pose_request_carries_shortest_yaw_and_previous_angular_command(boundary):
    import math
    import local_planner_node as module
    from local_planner.msg import MpcRequest
    state,_,_=boundary
    sent=[]
    state.ignore_lidar=True
    state.sequence.begin((1.,0.),-math.pi+.1,concurrent_yaw=True)
    state.last_yaw_rate=.03
    state.mpc_request_stamp=0;state.mpc_request_type=MpcRequest
    state.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(
        nanoseconds=2_000_000_000,to_msg=lambda:__import__('builtin_interfaces.msg',fromlist=['Time']).Time(sec=2)))
    state.mpc_requests_pub=SimpleNamespace(publish=sent.append)
    module.LocalPlanner.send_mpc_request(state,(0.,0.,math.pi-.1),10.)
    assert sent[0].plan_yaw and sent[0].goal_yaw==pytest.approx(.2)
    assert sent[0].previous_yaw_rate==.03
    assert state.mpc_requests[2_000_000_000][3]==(0.,0.,.03)


@pytest.mark.parametrize('pose,deadline', [((0.,0.,0.),10.), ((-.121,0.,0.),25.),
    ((.011,0.,0.),25.), ((0.,.021,0.),25.), ((0.,0.,.101),25.)])
def test_reverse_test_bounds_cancel_before_command(boundary, pose, deadline):
    import local_planner_node as module
    state, result, commands = boundary
    state.mpc_result(result)
    state.supervised_reverse_test = True
    state.reverse_start_pose = (0.,0.,0.)
    state.reverse_deadline = deadline
    state.ready = lambda now: pose
    module.LocalPlanner.tick(state)
    assert state.session.goal is None
    assert 'bound reached' in state.session.reason
    assert commands and all((m.linear.x,m.linear.y,m.angular.z)==(0.,0.,0.) for m in commands)


@pytest.mark.parametrize('enabled', [True, False])
def test_collision_switch_controls_obstacles_for_planning(boundary, enabled):
    import local_planner_node as module
    state, _, _ = boundary
    sample = SimpleNamespace(points=np.array([[.35,0.]]), stamp=123, received=10.)
    state.collision_inputs['front'] = sample
    state.collision_checks_enabled = enabled
    module.LocalPlanner.use_collision_sample(state)
    assert len(state.points) == (1 if enabled else 0)
    assert len(sample.points) == 1  # Retain raw geometry for diagnostics.
    assert state.cloud_stamp == 123 and state.cloud_received == 10.


@pytest.mark.parametrize('turning', [False, True], ids=['drive', 'turn'])
def test_configured_front_obstacle_stops_commands(boundary, turning):
    """The shipped configuration must retain front obstacles through dispatch."""
    import yaml
    import local_planner_node as module
    from collision_feedback import CollisionCloud
    from local_planner_core import front_mount
    from sensor_msgs.msg import PointCloud2, PointField

    state, result, commands = boundary
    parameters = yaml.safe_load((ROOT / 'config/astribot.yaml').read_text())['/**']['ros__parameters']
    state.collision_checks_enabled = parameters['python_collision_checks_enabled']
    state.selected_lidar = parameters['collision_lidar']
    mount = front_mount()
    # A real FLOAT32 sensor return representing an obstacle 0.35 m ahead.
    sensor = ((np.array([[.35, 0., .2]]) - mount[:3, 3]) @ mount[:3, :3]).astype('<f4')
    cloud = PointCloud2()
    cloud.header.frame_id = 'livox_frame'
    cloud.header.stamp.sec = 1
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 1, 12, 12
    cloud.fields = [PointField(name=name, offset=i * 4, datatype=7, count=1)
                    for i, name in enumerate(('x', 'y', 'z'))]
    cloud.data = sensor.tobytes()
    clock = SimpleNamespace(reference_stamp=1_000_000_000, reference_problem=lambda now: None)
    sample = CollisionCloud('front', mount, clock)
    sample.receive(cloud, now=10.)
    assert sample.problem(10.05, 1_000_000_000) is None
    state.collision_inputs = {'front': sample}
    module.LocalPlanner.use_collision_sample(state)
    if turning:
        state.sequence.begin((0., 1.), 0.)
    else:
        state.mpc_result(result)
    module.LocalPlanner.tick(state)
    assert commands[-1].linear.x == 0.
    assert commands[-1].linear.y == 0.
    assert commands[-1].angular.z == 0.
    assert state.session.reason.startswith('blocked:')


def test_cloud_cadence_jitter_does_not_cancel_drive(boundary, monkeypatch):
    """148 ms cloud gap plus request/result work can cross the 150 ms timer."""
    import local_planner_node as module
    state, result, commands = boundary
    state.mpc_result(result)  # First request at 10.0, accepted at 10.05.
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.18)
    module.LocalPlanner.tick(state)
    assert state.session.goal is not None
    assert commands[-1].linear.x > 0.
    # A fresh reply for the next cloud replaces the cached command normally.
    result.header.stamp.nanosec = 148_000_000
    state.mpc_requests[1_148_000_000] = ((0.,0.,0.),10.148,1,(0.,0.))
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.19)
    state.mpc_result(result)
    assert state.session.goal is not None
    assert state.mpc_received == 10.148


def test_cached_command_boundary_keeps_request_age(boundary):
    state, result, _ = boundary
    state.mpc_result(result)
    assert state.mpc_drive((0.,0.,0.),10.701)==(.014,0.)
    assert state.mpc_drive((0.,0.,0.),11.)==(.014,0.)
    with pytest.raises(ValueError, match='expired'):
        state.mpc_drive((0.,0.,0.),11.000001)


def test_reply_still_must_arrive_within_150_ms(boundary, monkeypatch):
    import local_planner_node as module
    state, result, commands = boundary
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.151)
    state.mpc_result(result)
    assert state.session.goal == (1., 0.)
    assert state.mpc_received == -float('inf')
    assert state.mpc_velocity == (0., 0.)


def test_expiry_during_tick_cannot_publish_nonzero(boundary,monkeypatch):
    import local_planner_node as module
    state,result,commands=boundary
    state.mpc_result(result)
    calls=[]
    def clock():
        calls.append(None)
        return 10.99 if len(calls)==1 else 11.01
    monkeypatch.setattr(module.time,'monotonic',clock)
    module.LocalPlanner.tick(state)
    assert state.session.goal == (1., 0.)
    assert 'expired before dispatch' in state.session.reason
    assert commands and all((m.linear.x,m.linear.y,m.angular.z)==(0.,0.,0.) for m in commands)


@pytest.mark.parametrize('failure', ['speed','nonfinite','deadline','frame','owner'])
def test_bad_cpp_results_stop_and_clear_target(boundary,failure):
    state,result,commands=boundary
    if failure=='solver': result.success=False
    elif failure=='late': state.mpc_requests[1000000000]=((0.,0.,0.),9.,1,(0.,0.))
    elif failure=='speed': result.command.linear.x=.11
    elif failure=='nonfinite': result.command.linear.y=float('nan')
    elif failure=='deadline': result.solve_ms=41.
    elif failure=='frame': result.header.frame_id='wrong'
    elif failure=='owner':
        def denied(now): raise ValueError('owner disabled')
        state.ready=denied
    state.mpc_result(result)
    assert state.session.goal is None
    assert commands and all((m.linear.x,m.linear.y,m.angular.z)==(0.,0.,0.) for m in commands)


def test_reply_from_previous_goal_cannot_replace_current_goal(boundary):
    state,result,_=boundary
    state.reset_mpc()
    state.mpc_result(result)
    assert state.session.goal is not None
    assert state.mpc_velocity==(0.,0.)


def test_cancel_and_inflight_reply_cannot_restart_motion(boundary):
    state,result,commands=boundary
    state.cancel()
    state.mpc_result(result)
    assert state.session.goal is None and state.mpc_velocity==(0.,0.)
    assert commands[-1].linear.x==0.


@pytest.mark.parametrize('point', [[.50,0.],[.60,0.]])
def test_new_cloud_invalidates_cached_cpp_prediction(boundary,point):
    state,result,_=boundary
    state.mpc_result(result)
    state.points=np.array([point])
    state.cloud_stamp+=1
    assert state.mpc_drive((0.,0.,0.),10.05)==(0.,0.)
    assert state.mpc_velocity==(0.,0.)


def test_missing_and_stale_cpp_results_are_not_commands(boundary):
    state,result,_=boundary
    assert state.mpc_drive((0.,0.,0.),10.6) == (0., 0.)
    state.mpc_result(result)
    with pytest.raises(ValueError, match='expired'): state.mpc_drive((0.,0.,0.),11.01)


def test_mid_drive_invalidation_waits_for_new_result(boundary, monkeypatch):
    import local_planner_node as module
    state, result, commands = boundary
    state.started = 1.  # Drive phase started long before this recent reply.
    state.mpc_result(result)
    old_reply = result
    state.points = np.array([[.60,0.]])  # Obstruct the prediction, not the origin.
    state.cloud_stamp += 1
    module.LocalPlanner.tick(state)
    assert state.session.goal is not None and commands[-1].linear.x == 0.
    monkeypatch.setattr(module.time, 'monotonic', lambda:10.10)
    module.LocalPlanner.tick(state)
    assert state.session.goal is not None, state.session.reason
    assert commands[-1].linear.x == 0.
    # Cleared prediction and replies from its old epoch cannot restart motion.
    state.mpc_result(old_reply)
    assert state.mpc_received == -float('inf')
    state.points = np.empty((0,2))
    result.epoch = state.mpc_epoch
    state.mpc_requests[1_000_000_000] = ((0.,0.,0.),10.10,state.mpc_epoch,(0.,0.))
    monkeypatch.setattr(module.time, 'monotonic', lambda:10.15)
    state.mpc_result(result)
    module.LocalPlanner.tick(state)
    assert state.session.goal is not None and commands[-1].linear.x > 0.


def test_repeated_collision_checks_do_not_reset_wait_or_discard_inflight(boundary, monkeypatch):
    import local_planner_node as module
    state, result, commands = boundary
    state.started = 1.
    state.mpc_result(result)
    state.points = np.array([[.35,0.]])  # Both collision gates will reject.
    state.cloud_stamp += 1
    module.LocalPlanner.tick(state)
    epoch = state.mpc_epoch
    state.mpc_requests[2_000_000_000] = ((0.,0.,0.),10.08,epoch,(0.,0.))
    for now in (10.10,10.20,10.40):
        monkeypatch.setattr(module.time, 'monotonic', lambda:now)
        module.LocalPlanner.tick(state)
        assert state.session.goal is not None, state.session.reason
        assert state.mpc_epoch == epoch
        assert 2_000_000_000 in state.mpc_requests
        assert commands[-1].linear.x == 0.
    monkeypatch.setattr(module.time, 'monotonic', lambda:10.56)
    module.LocalPlanner.tick(state)
    assert state.session.goal == (1., 0.) and commands[-1].linear.x == 0.
    assert state.session.reason.startswith('blocked:')


def test_expired_command_keeps_goal_and_accepts_next_solution(boundary, monkeypatch):
    import local_planner_node as module
    state, result, commands = boundary
    state.mpc_result(result)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 11.1)
    module.LocalPlanner.tick(state)
    assert state.session.goal == (1., 0.)
    assert commands[-1].linear.x == commands[-1].linear.y == 0.
    # A reply from before the stop cannot revive motion.
    state.mpc_result(result)
    assert state.mpc_received == -float('inf')
    result.epoch = state.mpc_epoch
    result.header.stamp.sec = 2
    state.mpc_requests[2_000_000_000] = ((0.,0.,0.),11.1,state.mpc_epoch,(0.,0.))
    monkeypatch.setattr(module.time, 'monotonic', lambda: 11.15)
    state.mpc_result(result)
    module.LocalPlanner.tick(state)
    assert state.session.goal == (1., 0.) and commands[-1].linear.x > 0.


def test_solver_failure_waits_for_retry_without_losing_goal(boundary):
    state, result, commands = boundary
    result.success = False
    result.error = 'solver deadline exceeded'
    state.mpc_result(result)
    assert state.session.goal == (1., 0.)
    assert commands[-1].linear.x == commands[-1].linear.y == 0.


@pytest.mark.parametrize('samples',[1,21])
def test_moving_command_cannot_claim_stationary_prediction(boundary,samples):
    from copy import deepcopy
    state,result,commands=boundary
    result.path.poses=[deepcopy(result.path.poses[0]) for _ in range(samples)]
    state.points=np.array([[.6,0.]])
    state.mpc_result(result)
    assert state.session.goal is None and commands[-1].linear.x==0.


def test_acceleration_uses_elapsed_dispatch_time_not_reply_count(boundary):
    state,_,_=boundary
    # Repeated fast proposals cannot accumulate acceleration before a dispatch.
    for _ in range(5):
        vx,vy=state.bounded_mpc_command(.1,.0,10.05)
        assert vx==pytest.approx(.007) and vy==0.
    state.last_translation=(vx,vy)
    state.last_command_time=10.05
    assert state.bounded_mpc_command(.1,0.,10.1)[0]==pytest.approx(.014)
    assert state.bounded_mpc_command(0.,0.,10.051)==(0.,0.)


def test_clipped_reversal_checks_actual_command_clearance(boundary):
    import local_planner_node as module
    state,result,commands=boundary
    state.last_translation=(.010,0.)
    state.mpc_requests[1000000000]=((0.,0.,0.),10.,1,state.last_translation)
    result.command.linear.x=-.004
    for i,item in enumerate(result.path.poses):
        item.pose.position.x=-.004*.1*i
    state.points=np.array([[.5501,0.]])
    state.mpc_result(result)
    assert state.mpc_velocity==(-.004,0.)  # Prediction moves away from the obstacle.
    assert state.bounded_mpc_command(-.004,0.,10.05)[0]==pytest.approx(.003)
    module.LocalPlanner.tick(state)
    assert commands[-1].linear.x==0. and commands[-1].linear.y==0.


@pytest.mark.parametrize('backend',['mpc','library'])
def test_launch_uses_existing_cpp_package_for_both_backends(monkeypatch,tmp_path,backend):
    import importlib.util
    from launch import LaunchContext
    from launch_ros.actions import Node
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path))
    spec=importlib.util.spec_from_file_location('planner_launch',ROOT/'launch/astribot.launch.py')
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    lookups=[]
    def lookup(package):
        lookups.append(package)
        return '/tmp/fake-library-share'
    monkeypatch.setattr(module,'get_package_share_directory',lookup)
    context=LaunchContext()
    context.launch_configurations.update(planner_backend=backend,dispatch='false')
    actions=module.components(context)
    assert sum(isinstance(action,Node) for action in actions)==2
    assert lookups==['local_planner']


def test_launcher_defaults_to_mpc_and_allows_explicit_library():
    import yaml
    script=str(ROOT/'scripts/run_astribot.sh')
    config=yaml.safe_load((ROOT/'config/astribot.yaml').read_text())
    assert config['/**']['ros__parameters']['planner_backend']=='mpc'
    assert config['/**']['ros__parameters']['collision_lidar']=='front'
    for args,backend in [([],'mpc'),(['--planner=library'],'library')]:
        result=subprocess.run(['bash',script,'--dry-run',*args],capture_output=True,text=True)
        assert result.returncode==0,result.stderr
        assert 'ros2 launch local_planner astribot.launch.xml' in result.stdout
        assert 'dispatch:=false' in result.stdout
        assert 'python_debug:=true' in result.stdout
        if args:
            assert 'planner_backend:='+backend in result.stdout
    assert subprocess.run(['bash',script,'--planner=unknown'],capture_output=True).returncode==2


def test_cpp_coordinator_requires_explicit_selection():
    script=str(ROOT/'scripts/run_astribot.sh')
    result=subprocess.run(['bash',script,'--dry-run','--cpp'],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert 'python_debug:=false' in result.stdout
