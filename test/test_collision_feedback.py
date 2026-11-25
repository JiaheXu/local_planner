from types import SimpleNamespace as NS
import numpy as np
import pytest

from local_planner_core import front_mount, rear_cloud_mount
from collision_feedback import CollisionCloud, choose_collision_source


def clock():
    return NS(reference_problem=lambda now: None, reference_stamp=1_100_000_000)


def cloud(stamp=1_000_000_000, frame='livox_frame', xyz=None):
    xyz=np.asarray([[1.,0.,.2]] if xyz is None else xyz,dtype='<f4')
    return NS(header=NS(frame_id=frame,stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9)),
        width=len(xyz),height=1,point_step=12,row_step=12*len(xyz),data=xyz.tobytes(),
        is_bigendian=False,fields=[NS(name=k,offset=4*i,datatype=7,count=1) for i,k in enumerate(('x','y','z'))])


def inputs():
    reference=clock()
    return {side:CollisionCloud(side,np.eye(4),reference) for side in ('front','rear')}


@pytest.mark.parametrize('age_ns',[500_000_001,1_500_000_000,2_000_000_000])
def test_scan_up_to_two_seconds_old_is_valid_on_receive_and_reuse(age_ns):
    sample=inputs()['rear']
    sample.clock.reference_stamp=1_000_000_000+age_ns
    sample.receive(cloud(),now=10.)
    assert sample.problem(10.,1_000_000_000+age_ns) is None
    assert sample.problem(12.,1_000_000_000+age_ns) is None
    assert sample.problem(12.000001,1_000_000_000+age_ns) is not None


def test_scan_source_age_over_two_seconds_and_future_are_rejected():
    for reference in (3_000_000_001,899_999_999):
        sample=inputs()['rear'];sample.clock.reference_stamp=reference
        sample.receive(cloud(),now=10.)
        assert sample.problem(10.,1_000_000_000) is not None
    sample=inputs()['rear'];sample.receive(cloud(),now=10.)
    sample.clock.reference_stamp=3_000_000_001
    assert 'source time' in sample.problem(10.,1_000_000_000)


def test_scan_pose_separation_over_two_seconds_is_rejected():
    sample=inputs()['rear'];sample.receive(cloud(),now=10.)
    for odom in (-1_000_000_001,3_000_000_001):
        assert 'not time-aligned' in sample.problem(10.,odom)


def test_rear_payload_uses_documented_front_coordinates_once(monkeypatch):
    from pathlib import Path
    import local_planner_core
    monkeypatch.setattr(local_planner_core,'get_package_share_directory',
                        lambda _:str(Path(__file__).resolve().parents[1]))
    np.testing.assert_allclose(rear_cloud_mount(),front_mount())


def test_unknown_rear_coordinate_convention_is_rejected(tmp_path,monkeypatch):
    import local_planner_core
    (tmp_path/'config').mkdir()
    (tmp_path/'config/astribot_geometry.yaml').write_text('rear_cloud_coordinates: rear_lidar\n')
    monkeypatch.setattr(local_planner_core,'get_package_share_directory',lambda _:str(tmp_path))
    with pytest.raises(ValueError,match='Unqualified rear cloud'):rear_cloud_mount()


def test_missing_front_uses_rear_and_recovered_front_is_preferred_for_new_selection():
    streams=inputs()
    streams['rear'].receive(cloud(),now=10.)
    assert choose_collision_source(streams,10.05,1_000_000_000,mode='auto')=='rear'
    streams['front'].receive(cloud(),now=10.04)
    assert choose_collision_source(streams,10.05,1_000_000_000)=='front'


@pytest.mark.parametrize('failure',['missing','stale','frame','zeros','skew','regressed'])
def test_unusable_front_cannot_prevent_healthy_rear_fallback(failure):
    streams=inputs()
    streams['rear'].receive(cloud(),now=10.)
    f=streams['front']
    if failure!='missing':
        f.receive(cloud(frame='wrong' if failure=='frame' else 'livox_frame',
                        stamp=2_000_000_000 if failure=='skew' else 1_000_000_000,
                        xyz=[[0.,0.,0.]] if failure=='zeros' else None),
                  now=7.9 if failure=='stale' else 10.)
    if failure=='regressed':f.receive(cloud(stamp=999_000_000),now=10.01)
    assert choose_collision_source(streams,10.05,1_000_000_000,mode='auto')=='rear'


@pytest.mark.parametrize('failure',['missing','stale','frame','zeros','skew','regressed'])
def test_front_default_never_falls_back_to_healthy_rear(failure):
    streams=inputs();streams['rear'].receive(cloud(),now=10.)
    front=streams['front']
    if failure!='missing':
        front.receive(cloud(frame='wrong' if failure=='frame' else 'livox_frame',
                            stamp=2_000_000_000 if failure=='skew' else 1_000_000_000,
                            xyz=[[0.,0.,0.]] if failure=='zeros' else None),
                      now=7.9 if failure=='stale' else 10.)
    if failure=='regressed':front.receive(cloud(stamp=999_000_000),now=10.01)
    for kwargs in ({},{'mode':'front'}):
        with pytest.raises(ValueError,match='front'):
            choose_collision_source(streams,10.05,1_000_000_000,**kwargs)


@pytest.mark.parametrize('rear_healthy',[True,False])
def test_front_only_checks_only_front_publishers(rear_healthy):
    streams=inputs();streams['front'].receive(cloud(),now=10.)
    if rear_healthy:streams['rear'].receive(cloud(),now=10.)
    checked=[]
    assert choose_collision_source(streams,10.05,1_000_000_000,
        mode='front',check_publishers=checked.append)=='front'
    assert checked==['front']


@pytest.mark.parametrize('mode,expected',[(None,{'front'}),('front',{'front'}),
                                        ('rear',{'rear'}),('auto',{'front','rear'})])
def test_python_subscribes_only_to_configured_clouds(mode,expected,monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from local_planner_node import LocalPlanner,LIDAR_TOPICS
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    context=Context();rclpy.init(context=context,domain_id=228)
    node=None
    try:
        overrides=[] if mode is None else [Parameter('collision_lidar',value=mode)]
        node=LocalPlanner(context=context,parameter_overrides=overrides)
        topics={subscription.topic_name for subscription in node.subscriptions}
        assert {side for side,(topic,) in LIDAR_TOPICS.items() if topic in topics}==expected
        assert node.dispatch is False
    finally:
        if node is not None:node.destroy_node()
        context.shutdown()


def test_no_fresh_source_fails_closed():
    streams=inputs()
    with pytest.raises(ValueError,match='No usable collision LiDAR'):
        choose_collision_source(streams,10.,1_000_000_000,mode='auto')
    streams['rear'].receive(cloud(),now=10.)
    with pytest.raises(ValueError):choose_collision_source(streams,12.001,1_000_000_000)


@pytest.mark.parametrize('front_healthy',[True,False])
def test_rear_only_ignores_front_and_checks_only_rear_publishers(front_healthy):
    streams=inputs()
    streams['rear'].receive(cloud(),now=10.)
    if front_healthy:streams['front'].receive(cloud(),now=10.)
    checked=[]
    assert choose_collision_source(streams,10.05,1_000_000_000,
        mode='rear',check_publishers=checked.append)=='rear'
    assert checked==['rear']


def test_rear_only_never_falls_back_to_front():
    streams=inputs();streams['front'].receive(cloud(),now=10.)
    with pytest.raises(ValueError,match='rear'):
        choose_collision_source(streams,10.,1_000_000_000,mode='rear')
    with pytest.raises(ValueError,match='collision_lidar'):
        choose_collision_source(streams,10.,1_000_000_000,mode='typo')


def test_shared_odometry_reference_required_for_both_clouds():
    streams=inputs()
    streams['rear'].clock.reference_problem=lambda now:'odometry missing'
    for sample in streams.values():sample.receive(cloud(),now=10.)
    with pytest.raises(ValueError,match='odometry missing'):
        choose_collision_source(streams,10.,1_000_000_000)


def test_duplicate_selected_publisher_is_not_bypassed_by_fallback():
    streams=inputs()
    for sample in streams.values():sample.receive(cloud(),now=10.)
    def check(side):raise ValueError('expected exactly one publisher')
    with pytest.raises(ValueError,match='exactly one publisher'):
        choose_collision_source(streams,10.,1_000_000_000,check_publishers=check)


@pytest.mark.parametrize('valid',[True,False])
@pytest.mark.parametrize('selected,other',[('front','rear'),('rear','front')])
def test_inactive_arrival_cannot_change_goal_or_obstacles(monkeypatch,valid,selected,other):
    import local_planner_node as module
    streams=inputs()
    streams[selected].receive(cloud(),now=10.)
    state=NS(collision_inputs=streams,selected_lidar=selected,points=streams[selected].points,
             session=NS(goal=(1.,0.)),cloud_stamp=streams[selected].stamp)
    monkeypatch.setattr(module.time,'monotonic',lambda:10.01)
    module.LocalPlanner.cloud(state,cloud(xyz=None if valid else [[0.,0.,0.]]),other)
    assert state.selected_lidar==selected and state.session.goal==(1.,0.)
    assert state.points is streams[selected].points and state.cloud_stamp==1_000_000_000


def test_selected_cloud_loss_stops_even_when_other_source_is_healthy(monkeypatch):
    from types import MethodType
    import local_planner_node as module
    from local_planner_core import GoalSequence,PathSession
    streams=inputs()
    streams['front'].receive(cloud(),now=7.9)
    streams['rear'].receive(cloud(),now=10.)
    commands=[]; transport=NS(publish=lambda msg:None)
    state=NS(collision_inputs=streams,selected_lidar='front',collision_checks_enabled=True,
        nav=NS(fields=lambda now:dict(position_navigation=[0.,0.,0.],navigation_age_ms=0.),
               stamp=1_000_000_000,reference_stamp=1_100_000_000, pose=(0.,0.,0.),
               error=None, clock_fault=False, reference_error=None, received=10.),
        session=PathSession(),sequence=GoalSequence(),snapshots={},source_ids=None,
        owner_generation=None,owner_status={},dispatch=False,backend='mpc',mpc_epoch=1,
        mpc_solve_ms=None,commands=NS(publish=commands.append),cancels=transport,paths=transport,
        status=transport,output_topic='shadow',cloud_stamp=1_000_000_000,
        mpc_received=-float('inf'), mpc_requests={})
    state.session.begin((1.,0.),state.cloud_stamp);state.sequence.begin((1.,0.),0.)
    for method in ('ready','cancel','reset_mpc','wait_mpc'):
        setattr(state,method,MethodType(getattr(module.LocalPlanner,method),state))
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    module.LocalPlanner.tick(state)
    assert state.session.goal == (1., 0.) and state.selected_lidar == 'front'
    assert 'fresh front cloud' in state.session.reason
    assert commands and all((m.linear.x,m.linear.y,m.angular.z)==(0.,0.,0.) for m in commands)
    assert choose_collision_source(streams,10.,state.nav.stamp,mode='auto')=='rear'


def test_rear_goal_pins_only_relevant_publishers():
    from local_planner_node import LocalPlanner,FRONT,REAR,ODOM
    topics=[]
    def endpoints(topic):
        topics.append(topic)
        return [NS(endpoint_gid=topic.encode())] if topic != FRONT else []
    state=NS(selected_lidar='rear',backend='mpc',output_topic='shadow',dispatch=False,
             get_publishers_info_by_topic=endpoints)
    ids=LocalPlanner.sources(state)
    assert len(ids)==len(topics)==5
    assert REAR in topics and ODOM in topics
    assert FRONT not in topics and all("imu" not in topic for topic in topics)
