"""Use XPolicy observations and its existing base owner; no SDK or hardware launch."""
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROOT = Path(get_package_share_directory('local_planner'))
PREFIX = '/astribot/local_planner'


def components(context):
    backend = LaunchConfiguration('planner_backend').perform(context)
    if backend not in ('mpc', 'library'):
        raise ValueError('planner_backend must be mpc or library')
    share = Path(get_package_share_directory('local_planner'))
    planner = Node(package='local_planner', executable='localPlanner', name='astribot_cpp_planner',
                       output='screen', parameters=[str(ROOT / 'config/astribot.yaml'),
                                                   {'pathFolder': str(share / 'paths'),
                                                    'planner_backend': backend}],
                       remappings=[('/scan', PREFIX + '/scan'), ('/way_point', PREFIX + '/way_point'),
                                   ('/planner_cancel', PREFIX + '/cancel'), ('/path', PREFIX + '/path'),
                                   ('/free_paths', PREFIX + '/free_paths'),
                                   ('/mpc_request', PREFIX + '/mpc_request'),
                                   ('/mpc_result', PREFIX + '/mpc_result')])
    nodes = [planner]
    follower = Node(package='local_planner', executable='astribotCoordinatorPython',
                    name='astribot_local_planner', output='screen', parameters=[str(ROOT / 'config/astribot.yaml'), {
                        'dispatch': ParameterValue(LaunchConfiguration('dispatch'), value_type=bool),
                        'ignore_lidar': ParameterValue(LaunchConfiguration('ignore_lidar', default='false'), value_type=bool),
                        'planner_backend': backend}])
    nodes.append(follower)
    exits = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='Local planner component exited'))])) for node in nodes]
    return [*exits, *nodes]


def generate_launch_description():
    config = yaml.safe_load((ROOT / 'config/astribot.yaml').read_text())['/**']['ros__parameters']
    return LaunchDescription([
        DeclareLaunchArgument('dispatch', default_value='false',
                              description='Explicitly publish /cmd_vel instead of private shadow output'),
        DeclareLaunchArgument('ignore_lidar', default_value='false',
                              description='MPC test without cloud input or obstacle avoidance'),
        DeclareLaunchArgument('planner_backend', default_value=config['planner_backend'],
                              choices=['mpc', 'library'], description='Drive-stage controller'),
        OpaqueFunction(function=components)])
