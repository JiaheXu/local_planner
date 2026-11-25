"""The installed package must run without an XPolicy checkout on sys.path."""
from pathlib import Path
import os
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def test_installed_coordinator_dependencies_without_xpolicy(tmp_path):
    prefix = Path(get_package_prefix('local_planner'))
    program = prefix / 'lib/local_planner/astribotCoordinatorPython'
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
    assert (share / 'launch/astribot.launch.xml').is_file()
    assert (share / 'config/astribot.yaml').is_file()
    assert (share / 'config/astribot_geometry.yaml').is_file()
    import yaml
    parameters=yaml.safe_load((share / 'config/astribot.yaml').read_text())['/**']['ros__parameters']
    assert parameters['collision_lidar']=='front'
    assert parameters['python_collision_checks_enabled'] is True


def test_default_coordinator_is_native_and_backend_default_reads_yaml(tmp_path):
    import yaml
    prefix=Path(get_package_prefix('local_planner'))
    program=prefix/'lib/local_planner/astribotCoordinator'
    assert program.read_bytes()[:4]==b'\x7fELF'
    marker=tmp_path/'share/ament_index/resource_index/packages/local_planner'
    marker.parent.mkdir(parents=True);marker.write_text('')
    config_dir=tmp_path/'share/local_planner/config';config_dir.mkdir(parents=True)
    config=yaml.safe_load((Path(get_package_share_directory('local_planner'))/'config/astribot.yaml').read_text())
    config['/**']['ros__parameters']['planner_backend']='library'
    (config_dir/'astribot.yaml').write_text(yaml.safe_dump(config))
    env=os.environ.copy();env['AMENT_PREFIX_PATH']=str(tmp_path)+os.pathsep+env.get('AMENT_PREFIX_PATH','')
    result=subprocess.run([str(program),'--default-backend'],env=env,text=True,capture_output=True,timeout=5)
    assert result.returncode==0,result.stderr
    assert result.stdout=='library'
