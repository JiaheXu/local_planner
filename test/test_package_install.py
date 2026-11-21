"""The installed package must run without an XPolicy checkout on sys.path."""
from pathlib import Path
import os
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def test_installed_coordinator_dependencies_without_xpolicy(tmp_path):
    prefix = Path(get_package_prefix('local_planner'))
    program = prefix / 'lib/local_planner/astribotCoordinator'
    environment = os.environ.copy()
    environment.pop('PYTHONPATH', None)
    # Only this package and ROS, no application checkout or Python environment.
    environment['PYTHONPATH'] = os.pathsep.join([
        str(prefix / 'local/lib/python3.10/dist-packages'),
        '/opt/ros/humble/local/lib/python3.10/dist-packages',
        '/opt/ros/humble/lib/python3.10/site-packages'])
    environment.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1')
    code = ('import sys,runpy; sys.path.insert(0,sys.argv[1]); '
            'from local_planner.msg import MpcRequest,MpcResult; '
            'runpy.run_path(sys.argv[2],run_name="dependency_check"); '
            'assert not any(name.startswith("XPolicyLab") for name in sys.modules)')
    result = subprocess.run(['/usr/bin/python3','-c',code,str(program.parent),str(program)],
                            cwd=tmp_path, env=environment, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_installed_launch_and_config_belong_to_package():
    share = Path(get_package_share_directory('local_planner'))
    assert (share / 'launch/astribot.launch.py').is_file()
    assert (share / 'config/astribot.yaml').is_file()
    assert (share / 'config/astribot_geometry.yaml').is_file()
