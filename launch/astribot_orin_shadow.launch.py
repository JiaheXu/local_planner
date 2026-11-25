"""Onboard MPC connectivity test: native odometry, shadow commands, no owner enable."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('local_planner'))
    local, remote = '/astribot/local_planner', '/cocomatrix/local_planner'
    config = str(package / 'config/astribot.yaml')
    backend = Node(package='local_planner', executable='localPlanner',
        name='astribot_orin_mpc', output='screen',
        parameters=[config, {'pathFolder': str(package / 'paths'), 'planner_backend': 'mpc'}],
        remappings=[('/mpc_request', remote + '/mpc_request'),
                    ('/mpc_result', remote + '/mpc_result')])
    coordinator = Node(package='local_planner', executable='astribotCoordinatorPython',
        name='astribot_orin_coordinator', output='screen',
        parameters=[config, {'dispatch': False, 'ignore_lidar': True, 'planner_backend': 'mpc'}],
        remappings=[(local + '/' + suffix, remote + '/' + suffix) for suffix in
                    ('scan', 'way_point', 'cancel', 'path', 'path_map', 'status',
                     'mpc_request', 'mpc_result', 'cmd_vel_shadow')]
        + [('/fast_lio/chassis/odometry', remote + '/chassis_odometry'),
           ('/astribot_viz/goal_pose', remote + '/goal_pose'),
           ('/astribot_viz/goal_cancel', remote + '/goal_cancel')])
    odometry = Node(executable='/usr/bin/python3',
        arguments=[PathJoinSubstitution([LaunchConfiguration('bringup_dir'), 'odometry.py'])],
        name='astribot_orin_chassis', output='screen')
    nodes = [backend, coordinator, odometry]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Orin shadow planner component exited'))]))
        for node in nodes]
    return LaunchDescription([
        DeclareLaunchArgument('bringup_dir',
            default_value='/home/astribot/ros2_ws/src/astribot_planner_bringup'),
        *handlers, *nodes])
