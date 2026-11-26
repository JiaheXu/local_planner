"""Explicit Orin obstacle mode has no implicit live side effects."""
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def test_obstacle_and_no_obstacle_modes_preview_without_environment_or_nodes():
    script=str(ROOT/'scripts/run_orin_dispatch.sh')
    for option,flag in [('--avoid-obstacles','ignore_lidar:=false'),('--ignore-lidar','ignore_lidar:=true')]:
        result=subprocess.run(['bash',script,option,'--basic-control','--dry-run'],capture_output=True,text=True)
        assert result.returncode==0,result.stderr
        assert flag in result.stdout and 'basic_control:=true' in result.stdout
    for options in [[],['--avoid-obstacles','--ignore-lidar']]:
        result=subprocess.run(['bash',script,*options,'--dry-run'],capture_output=True)
        assert result.returncode==2


def test_orin_launch_exposes_mode_and_native_front_cloud():
    import importlib.util
    from launch import LaunchContext
    spec=importlib.util.spec_from_file_location('obstacle_launch',ROOT/'launch/astribot_orin_dispatch.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    context.launch_configurations.update(basic_control='true',ignore_lidar='false',
        planner_source=str(ROOT),bringup_dir='/tmp/no-live-bringup')
    nodes=module.components(context)
    from launch_ros.actions import Node
    from launch.utilities import perform_substitutions
    coordinator=[n for n in nodes if isinstance(n,Node)][1]
    coordinator._perform_substitutions(context)
    remaps=dict(coordinator.expanded_remapping_rules)
    assert remaps['/sdk/astribot_lidar/front_scan']=='/livox/lidar_front'
    assert remaps['/astribot/local_planner/obstacles']=='/cocomatrix/local_planner/obstacles'
