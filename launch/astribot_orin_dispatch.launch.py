"""Orin MPC -> basic Twist or leased PC relay -> existing base owner."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def components(context):
    basic = LaunchConfiguration('basic_control').perform(context).lower() == 'true'
    package = Path(get_package_share_directory('local_planner'))
    local, remote = '/astribot/local_planner', '/cocomatrix/local_planner'
    config = str(package / 'config/astribot.yaml')
    backend = Node(package='local_planner', executable='localPlanner',
        name='astribot_orin_mpc', output='screen',
        parameters=[config, {'pathFolder': str(package / 'paths'), 'planner_backend': 'mpc'}],
        remappings=[('/mpc_request', remote + '/mpc_request'),
                    ('/mpc_result', remote + '/mpc_result')])
    coordinator = Node(executable='/usr/bin/python3',
        arguments=[PathJoinSubstitution([LaunchConfiguration('planner_source'),
                                        'python', 'orin_basic_coordinator.py' if basic else 'orin_dispatch_coordinator.py'])],
        name='astribot_orin_coordinator', output='screen',
        parameters=[config, {'dispatch': True, 'ignore_lidar': True, 'planner_backend': 'mpc'}],
        remappings=[(local + '/' + suffix, remote + '/' + suffix) for suffix in
                    ('scan', 'way_point', 'cancel', 'path', 'path_map', 'status',
                     'mpc_request', 'mpc_result')]
        + [('/cmd_vel', remote + ('/cmd_vel' if basic else '/cmd_vel_unleased')),
           ('/astribot_viz/goal_pose', remote + '/goal_pose'),
           ('/fast_lio/chassis/odometry', remote + '/chassis_odometry'),
           ('/astribot/base/status', remote + '/owner_status'),
           ('/astribot_viz/goal_cancel', remote + '/goal_cancel')])
    odometry = Node(executable='/usr/bin/python3',
        arguments=[PathJoinSubstitution([LaunchConfiguration('bringup_dir'), 'odometry.py'])],
        name='astribot_orin_chassis', output='screen')
    nodes = [backend, coordinator, odometry]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Orin dispatch component exited'))])) for node in nodes]
    return [*handlers, *nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('basic_control', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('bringup_dir', default_value='/home/astribot/ros2_ws/src/astribot_planner_bringup'),
        DeclareLaunchArgument('planner_source', default_value='/home/astribot/ros2_ws/src/local_planner'),
        OpaqueFunction(function=components)])
