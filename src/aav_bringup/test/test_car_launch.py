"""
car.launch.py: the machine files, and the launch arguments that win over them.

Each test runs a launch file's own setup, car.launch.py's or sensors.launch.py's, with
some arguments, and looks at what it would start. Nothing starts: no ROS, no devices.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.conditions import InvalidConditionExpressionError
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest


def load(package, name):
    path = os.path.join(get_package_share_directory(package), 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CAR = load('aav_bringup', 'car.launch.py')
SENSORS = load('aav_sensors', 'sensors.launch.py')
MACHINES = sorted(f[:-len('.yaml')] for f in os.listdir(
    os.path.join(get_package_share_directory('aav_bringup'), 'config', 'machines')))


def context_for(launch_file, **args):
    """Return a launch context with the file's defaults, and these arguments on top."""
    context = LaunchContext()
    for declared in launch_file.generate_launch_description().get_launch_arguments():
        context.launch_configurations[declared.name] = perform_substitutions(
            context, declared.default_value)
    context.launch_configurations.update(args)
    return context


def started(launch_file, **args):
    return launch_file.launch_setup(context_for(launch_file, **args))


def included(actions):
    """Map each launch file included to the arguments it is given."""
    found = {}
    for a in actions:
        if isinstance(a, IncludeLaunchDescription):
            # Loading the file expands its path; it also checks that the file loads.
            a.launch_description_source.get_launch_description(LaunchContext())
            name = os.path.basename(a.launch_description_source.location)
            found[name] = dict(a.launch_arguments)
    return found


def nodes(actions):
    return {(a.node_package, a.node_executable) for a in actions if isinstance(a, Node)}


@pytest.mark.parametrize('machine', MACHINES)
def test_every_machine_file_loads_and_starts(machine):
    assert CAR.load_machine(machine)['parts']
    started(CAR, machine=machine)


def test_the_model_suits_each_machine():
    assert CAR.load_machine('orin')['detector']['model'] == 'models/yolo11n.engine'
    assert CAR.load_machine('laptop')['detector']['model'] == 'models/yolo11n.mlpackage'


def test_gps_false_turns_the_gps_off_where_the_machine_file_has_it():
    assert CAR.load_machine('all')['parts']['gps']
    on = included(started(CAR, machine='all'))
    assert on['sensors.launch.py']['gps'] == 'true'
    assert on['localization.launch.py']['gps'] == 'true'
    off = included(started(CAR, machine='all', gps='false'))
    assert off['sensors.launch.py']['gps'] == 'false'
    assert off['localization.launch.py']['gps'] == 'false'
    assert started(SENSORS, imu='false', gps='false') == []


def test_gps_true_starts_no_gps_where_none_is_plugged_in():
    for machine in ('laptop', 'motor_pi'):
        assert 'sensors.launch.py' not in included(started(CAR, machine=machine, gps='true'))


@pytest.mark.parametrize('value', ['False', '0'])
def test_false_is_read_as_ros_reads_it(value):
    assert included(started(CAR, machine='all', gps=value))['sensors.launch.py']['gps'] == \
        'false'


def test_the_gps_driver_argument_picks_the_driver():
    sent = included(started(CAR, machine='all', gps_driver='nmea'))
    assert sent['sensors.launch.py']['gps_driver'] == 'nmea'
    ublox = started(SENSORS, imu='false', gps_driver='ublox')
    assert nodes(ublox) == {('ublox_gps', 'ublox_gps_node')}
    nmea = included(started(SENSORS, imu='false', gps_driver='nmea'))
    assert nmea['gps_nmea.launch.py']['port'] == '/dev/aav/gps'


def test_ublox_gps_is_the_default():
    assert ('ublox_gps', 'ublox_gps_node') in nodes(started(SENSORS))


def test_the_imu_driver_argument_picks_the_language():
    sent = included(started(CAR, machine='all', imu_driver='cpp'))
    assert sent['sensors.launch.py']['imu_driver'] == 'cpp'
    assert ('aav_imu_yahboom', 'imu_node') in nodes(started(SENSORS, imu_driver='rust'))
    assert ('yahboom_imu_cpp', 'imu_node') in nodes(started(SENSORS, imu_driver='cpp'))


def test_the_rust_imu_driver_is_the_default():
    assert included(started(CAR, machine='all'))['sensors.launch.py']['imu_driver'] == 'rust'
    assert ('aav_imu_yahboom', 'imu_node') in nodes(started(SENSORS))


def test_the_map_and_the_start_pose_reach_their_launch_files():
    sent = included(started(CAR, machine='all', map='/maps/site.yaml', start_x='1.5',
                            start_y='-2.0', start_yaw='0.3'))
    assert sent['navigation.launch.py']['map'] == '/maps/site.yaml'
    assert (sent['localization.launch.py']['start_x'], sent['localization.launch.py']['start_y'],
            sent['localization.launch.py']['start_yaw']) == ('1.5', '-2.0', '0.3')
    assert sent['localization.launch.py']['gps'] == 'true'
    default = included(started(CAR, machine='all'))['navigation.launch.py']['map']
    assert default.endswith(os.path.join('maps', 'empty.yaml'))


def test_bad_values_stop_the_launch():
    with pytest.raises(RuntimeError, match='no such machine file'):
        started(CAR, machine='nowhere')
    with pytest.raises(InvalidConditionExpressionError):
        started(CAR, machine='all', gps='maybe')
    with pytest.raises(RuntimeError, match='pick one of'):
        started(SENSORS, gps_driver='garmin')
    with pytest.raises(RuntimeError, match='pick one of'):
        started(SENSORS, imu_driver='python')


def test_one_stop_sign_watcher_next_to_the_mux():
    for machine in MACHINES:
        parts = CAR.load_machine(machine)['parts']
        watcher = ('aav_yolo', 'stop_sign_watcher') in nodes(started(CAR, machine=machine))
        assert watcher == bool(parts.get('mux')), machine


def test_lane_keeping_is_off_until_a_machine_file_turns_it_on(monkeypatch):
    # control_interface drives whenever it sees a lane, so no machine file has it on yet.
    for machine in MACHINES:
        assert not CAR.load_machine(machine)['parts'].get('lane'), machine
        assert 'lane_follower.launch.py' not in included(started(CAR, machine=machine))
    monkeypatch.setattr(CAR, 'load_machine', lambda machine: {'parts': {'lane': True}})
    sent = included(started(CAR, machine='orin'))
    assert sent['lane_finder.launch.py'] == {'debug_image': 'false'}
    assert sent['lane_follower.launch.py'] == {'fake': 'false'}


def test_the_detector_is_named_after_its_machine(monkeypatch, tmp_path):
    model = tmp_path / 'yolo11n.engine'
    model.write_text('')
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', []))
    context = context_for(CAR, machine='orin')
    sent = included(CAR.detector(context, 'orin', {'model': str(model), 'confidence': 0.6}))
    assert sent['detector.launch.py']['node_name'] == 'yolo_detector_orin'
    assert sent['detector.launch.py']['model'] == str(model)
    assert sent['detector.launch.py']['labels'] == 'stop sign'
    assert sent['detector.launch.py']['picture'] == ''
    assert sent['detector.launch.py']['confidence'] == '0.6'
    # The rest are passed too, at the detector's own defaults: none can leak in.
    assert sent['detector.launch.py']['image'] == '/camera/image_raw'
    assert sent['detector.launch.py']['device'] == 'auto'


def test_the_detector_is_skipped_without_pytorch_or_its_model(monkeypatch, tmp_path):
    context = context_for(CAR, machine='orin')
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', ['torch']))
    assert CAR.detector(context, 'orin', {'model': 'models/yolo11n.pt'}) == []
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', []))
    assert CAR.detector(context, 'orin', {'model': str(tmp_path / 'none.engine')}) == []


@pytest.mark.parametrize('machine', MACHINES)
def test_each_machine_files_detector_settings_are_known(machine, monkeypatch):
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', ['torch']))
    settings = CAR.load_machine(machine).get('detector') or {}
    CAR.detector(context_for(CAR, machine=machine), machine, settings)
    with pytest.raises(RuntimeError, match='takes no'):
        CAR.detector(context_for(CAR, machine=machine), machine, {'modle': 'x.pt'})
