"""
The real car. One launch file for every machine; a machine file picks the parts.

  ros2 launch aav_bringup car.launch.py machine:=orin
      [gps:=true] [gps_driver:=ublox] [map:=<map yaml>] [start_x:=0] [start_y:=0]
      [start_yaw:=0]

machine: config/machines/<machine>.yaml lists which parts run on that computer
(orin, motor_pi, laptop, all), and the detector's settings there. Every machine uses the
same ROS_DOMAIN_ID, so the parts find each other over the network; see docs/platforms.md.
The arguments win over the machine file: gps:=false starts no GPS driver anywhere. A part
that can't run here (no PyTorch, no model file, no usb_cam) is skipped with a warning.

Parts and what they start:
  description     robot_state_publisher (+ joint_state_publisher): the car's frames
  mux, ego        twist_mux -> /cmd_vel -> ego -> /ego/actuator_cmd
  teleop          joystick -> /cmd_vel/joy, e-stop button -> /estop
  scripted        /scripted_drive action -> /cmd_vel/scripted
  imu, gps,       sensor drivers (on the machine the device is plugged into);
  radar, battery  gps_driver picks the GPS's: ublox (the default), nmea or ublox_dgnss
  camera          usb_cam -> /camera/image_raw (Linux only)
  detector        YOLO on /camera/image_raw -> /perception/detections, as
                  yolo_detector_<machine>, with the machine file's model
  stop_signs      stop_sign_watcher -> /perception/stop, twist_mux's stop-sign lock (the
                  lock is in aav_mux's config). One per car, next to the mux
  radar_clusters  DBSCAN on /radar/points -> /radar/clusters, /radar/cluster_boxes
  camera_obstacles
                  the detectors' boxes as points on the road -> /perception/camera_obstacles,
                  for Nav2's costmap. Off until the cameras are calibrated (Q22)
  lane            lane keeping: lane_finder on /camera/image_raw -> /lane_geometry, and
                  control_interface -> /cmd_vel/ai (twist_mux's AI slot). Off on every
                  machine: it drives whenever it sees a lane, unless a driver above it does
  localization    ekf_local, ekf_global, navsat_transform
  navigation      Nav2 -> /cmd_vel/nav. Its local costmap reads the sensors this machine
                  runs: with radar, /radar/points, and collision_monitor stops Nav2 for
                  radar points just ahead; with camera_obstacles, the camera's points
  spine           backend: CAN spine (ros2_socketcan + spine_bridge)
  legacy_uart     backend: today's ESP32 over UART (the Pi wired to it)
  legacy_encoder  today's wheel encoder over UDP
  foxglove        foxglove_bridge on port 8765, for Foxglove Studio on a laptop

The detector runs as `python3 -m aav_yolo.detector`, the python3 of the environment that
starts this file: on a laptop, the ml environment's PyTorch (pixi run -e ml car laptop);
on the Orin, JetPack's (docs/perception.md).
"""

import os
import re
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import evaluate_condition_expression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import yaml

PARTS = ('description', 'mux', 'ego', 'teleop', 'scripted', 'imu', 'gps', 'radar', 'battery',
         'camera', 'detector', 'stop_signs', 'radar_clusters', 'camera_obstacles', 'lane',
         'localization', 'navigation', 'spine', 'legacy_uart', 'legacy_encoder', 'foxglove')
# What a machine file holds: the parts, and the detector's settings.
SECTIONS = ('parts', 'detector')
# Each model format: what runs it, besides PyTorch and Ultralytics, and how to make it.
FORMATS = {
    '.pt': ([], 'pixi run get-models'),
    '.mlpackage': (['coremltools'], 'pixi run -e ml export-coreml (on a Mac)'),
    '.engine': (['tensorrt'], 'python3 tools/build_engine.py (on this computer)'),
}
# Prints the modules a python3 lacks. find_spec looks without importing: no time at all.
MISSING = ('import importlib.util as u, sys; '
           'print(" ".join(m for m in sys.argv[1:] if not u.find_spec(m)))')


def launch_file(package, name):
    return os.path.join(get_package_share_directory(package), 'launch', name)


def include(package, name, **args):
    # Pass every argument the included file needs: included files share launch
    # arguments, so one left out may arrive from another file.
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file(package, name)),
        launch_arguments={k: str(v).lower() if isinstance(v, bool) else str(v)
                          for k, v in args.items()}.items())


def installed(package):
    try:
        get_package_share_directory(package)
    except PackageNotFoundError:
        return False
    return True


def skipped(part, why):
    """Warn that a part the machine file asks for can't run here; the rest still starts."""
    get_logger('car.launch.py').warning(f'{part}: not started: {why}')
    return []


def workspace():
    """Return this repository: Pixi's project folder, or the folder holding colcon's install/."""
    if os.environ.get('PIXI_PROJECT_ROOT'):
        return os.environ['PIXI_PROJECT_ROOT']
    head, found, _ = get_package_share_directory('aav_bringup').partition(
        f'{os.sep}install{os.sep}')
    return head if found else os.getcwd()


def load_machine(machine):
    folder = os.path.join(get_package_share_directory('aav_bringup'), 'config', 'machines')
    path = os.path.join(folder, f'{machine}.yaml')
    if not os.path.isfile(path):
        known = sorted(f[:-len('.yaml')] for f in os.listdir(folder) if f.endswith('.yaml'))
        raise RuntimeError(f'machine:={machine}: no such machine file. Pick one of {known}')
    with open(path) as f:
        config = yaml.safe_load(f)
    for what, found, known in (('sections', config, SECTIONS),
                               ('parts', config['parts'], PARTS)):
        unknown = set(found) - set(known)
        if unknown:
            raise RuntimeError(f'{machine}.yaml: unknown {what} {sorted(unknown)}')
    return config


def lacks(modules):
    """Return the python3 on PATH, which the detector runs in, and the modules it lacks."""
    python = shutil.which('python3') or 'python3'
    return python, subprocess.run([python, '-c', MISSING, *modules],
                                  capture_output=True, text=True).stdout.split()


def detector(context, machine, settings):
    """Return the detector part: YOLO as yolo_detector_<machine>, or why it can't start."""
    if not isinstance(settings, dict):
        raise RuntimeError(f'{machine}.yaml: give the detector settings, such as '
                           f'"detector: {{model: models/yolo11n.pt}}"')
    path = launch_file('aav_yolo', 'detector.launch.py')
    declared = (PythonLaunchDescriptionSource(path).get_launch_description(context)
                .get_launch_arguments())
    unknown = set(settings) - ({a.name for a in declared} - {'node_name', 'picture'})
    if unknown:
        raise RuntimeError(f'{machine}.yaml: the detector takes no {sorted(unknown)}')
    # Every argument, its own defaults included, so none can leak in from another file.
    defaults = {a.name: perform_substitutions(context, a.default_value) for a in declared
                if a.default_value is not None}
    args = {**defaults, 'labels': 'stop sign', 'model': 'models/yolo11n.pt', **settings}
    model = os.path.join(workspace(), os.path.expanduser(str(args['model'])))
    runtime, make = FORMATS.get(os.path.splitext(model)[1], ([], 'docs/perception.md'))
    python, missing = lacks(['torch', 'ultralytics', *runtime])
    if missing:
        return skipped('detector', f'{python} has no {", ".join(missing)}. On a laptop: '
                                   f'pixi run -e ml car {machine}. On the Orin: '
                                   "JetPack's PyTorch and TensorRT (docs/perception.md)")
    if not os.path.exists(model):
        return skipped('detector', f'{model} is missing. Make it with {make}')
    args.update(model=model, picture='',
                node_name='yolo_detector_' + re.sub(r'\W', '_', machine))
    return [include('aav_yolo', 'detector.launch.py', **args)]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        # As IfCondition reads it: true or 1, false or 0; anything else is an error.
        return evaluate_condition_expression(context, [LaunchConfiguration(name)])

    machine = arg('machine')
    config = load_machine(machine)
    parts = config['parts']

    def on(name):
        # A launch argument can switch a part off, wherever the machine file has it.
        return bool(parts.get(name, False)) and (name != 'gps' or flag('gps'))

    actions = []
    if on('description'):
        actions.append(include('aav_description', 'description.launch.py'))
    if on('mux'):
        actions.append(include('aav_mux', 'mux.launch.py'))
    if on('ego'):
        actions.append(include('aav_ego', 'ego.launch.py'))
    if on('teleop'):
        actions.append(include('aav_teleop', 'teleop.launch.py'))
    if on('scripted'):
        actions.append(include('aav_scripted', 'scripted.launch.py'))
    if any(on(s) for s in ('imu', 'gps', 'radar', 'battery')):
        actions.append(include('aav_sensors', 'sensors.launch.py',
                               imu=on('imu'), gps=on('gps'), radar=on('radar'),
                               battery=on('battery'), gps_driver=arg('gps_driver')))
    if on('camera'):
        if installed('usb_cam'):
            actions.append(include('aav_sensors', 'cameras.launch.py',
                                   left='/dev/aav/camera_left', right=''))
        else:
            actions += skipped('camera', 'usb_cam is not installed here (it is Linux only)')
    if on('detector'):
        actions += detector(context, machine, config.get('detector') or {})
    if on('stop_signs'):
        actions.append(Node(package='aav_yolo', executable='stop_sign_watcher',
                            name='stop_sign_watcher'))
    if on('radar_clusters'):
        actions.append(include('aav_object_detection', 'radar_clusters.launch.py',
                               mock=False, eps=0.8, minimum_points=15, radar_rviz=False))
    if on('camera_obstacles'):
        actions.append(Node(package='aav_yolo', executable='camera_obstacles',
                            name='camera_obstacles',
                            parameters=[os.path.join(get_package_share_directory('aav_yolo'),
                                                     'config', 'camera_obstacles.yaml')]))
    if on('lane'):
        actions += [include('aav_lane_finder', 'lane_finder.launch.py', debug_image=False),
                    include('aav_lane_geometry', 'lane_follower.launch.py', fake=False)]
    if on('localization'):
        actions.append(include('aav_localization', 'localization.launch.py',
                               gps=flag('gps'), start_x=arg('start_x'),
                               start_y=arg('start_y'), start_yaw=arg('start_yaw')))
    if on('navigation'):
        # Always pass a map: an empty `map` here would override Nav2's own default,
        # because included launch files share launch arguments.
        empty_map = os.path.join(get_package_share_directory('aav_navigation'), 'maps',
                                 'empty.yaml')
        # The local costmap reads the sensors this machine runs. Each is passed, on or
        # off, so none can leak in from another file.
        actions.append(include('aav_navigation', 'navigation.launch.py',
                               map=arg('map') or empty_map, radar=on('radar'),
                               collision_monitor=on('radar'),
                               camera_obstacles=on('camera_obstacles')))
    if on('spine'):
        actions.append(include('aav_spine', 'spine.launch.py'))
    if on('legacy_uart') or on('legacy_encoder'):
        actions.append(include('aav_legacy', 'legacy.launch.py',
                               uart=on('legacy_uart'), encoder=on('legacy_encoder')))
    if on('foxglove'):
        actions.append(Node(package='foxglove_bridge', executable='foxglove_bridge',
                            name='foxglove_bridge', parameters=[{'port': 8765}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('machine', default_value='all',
                              description='config/machines/<machine>.yaml'),
        DeclareLaunchArgument('gps', default_value='true',
                              description='false: no GPS anywhere (odometry only), whatever '
                                          'the machine file says'),
        DeclareLaunchArgument('gps_driver', default_value='ublox',
                              description='ublox (u-blox, Pixi too), nmea (any NMEA receiver) '
                                          'or ublox_dgnss (u-blox F9P, apt only). Tentative '
                                          'until the receiver is known: Q18'),
        DeclareLaunchArgument('map', default_value='',
                              description='Map YAML for Nav2 (default: the empty map)'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='Heading at start, rad counter-clockwise from east'),
        OpaqueFunction(function=launch_setup),
    ])
