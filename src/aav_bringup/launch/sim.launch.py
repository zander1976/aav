"""
The whole stack in Gazebo: same nodes and configs as the car, simulated hardware.

  ros2 launch aav_bringup sim.launch.py [world:=empty|test_lot|road] [nav:=true]
      [teleop:=false] [gui:=true] [rviz:=true]
      [perception:=false] [lane:=false|true|fake] [camera_obstacles:=false] [hud:=false]
      [obstacle:=false] [start_y:=0.0] [start_yaw:=0.0]
  pixi run sim world:=road                         the road: the car's camera and radar
  pixi run -e ml sim world:=road perception:=true  adds YOLO and the stop-sign watcher

world:=empty pairs with the empty map (phase 2); world:=test_lot with
maps/example_lot.yaml (phase 3); world:=road, the bench's road in Gazebo, with the empty
map: chosen automatically. The car starts at the world origin facing east, which is also
the map origin (the datum); start_y and start_yaw move it, as on the bench.

The road world renders the car's camera and its radar stand-in (aav_gazebo
worlds.has_sensors), so the car's whole graph runs there, with the bench's argument names:

  the radar           always, where the world renders it: radar_clusters, and Nav2's
                      obstacle layer and collision_monitor (navigation.launch.py radar:=true)
  perception:=true    YOLO (the ml environment) and stop_sign_watcher: twist_mux's
                      stop-sign lock
  lane:=true          lane keeping: lane_finder and control_interface, on /cmd_vel/ai.
                      lane:=fake: control_interface on a pretend lane, in any world
  camera_obstacles    the detectors' boxes into Nav2's local costmap
  hud:=true           the HUD's web server, on 127.0.0.1:5000
  obstacle:=true      the bench's 1 m box in the lane, 15 m ahead (aav_gazebo)

The empty and test_lot worlds render nothing, so they run headless anywhere, and the
camera's parts need world:=road. Every node runs on simulated time.
"""

import os

from aav_gazebo import worlds
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

MAPS = {'empty': 'empty.yaml', 'test_lot': 'example_lot.yaml', 'road': 'empty.yaml'}
# What each world shows in RViz: the road world adds the camera, the radar and the truth.
LAYOUTS = {'road': 'road.rviz'}
# The parts that read the camera, so need a world that renders it.
CAMERA_PARTS = ('perception', 'camera_obstacles')


def include(context, package, launch_file, **args):
    share = FindPackageShare(package).perform(context)
    return IncludeLaunchDescription(
        os.path.join(share, 'launch', launch_file),
        launch_arguments={k: str(v).lower() if isinstance(v, bool) else str(v)
                          for k, v in args.items()}.items())


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = arg('world')
    if world not in MAPS:
        raise ValueError(f'world:={world}: pick one of {sorted(MAPS)}')
    flag = {n: arg(n).lower() == 'true'
            for n in ('nav', 'teleop', 'rviz', 'perception', 'camera_obstacles', 'hud')}
    lane = arg('lane').lower()
    if lane not in ('false', 'true', 'fake'):
        raise ValueError(f'lane:={lane}: use true (the lane finder), fake or false')
    gazebo_share = FindPackageShare('aav_gazebo').perform(context)
    sensors = worlds.has_sensors(worlds.world_file(gazebo_share, world))
    wanted = [n for n in CAMERA_PARTS if flag[n]] + (['lane'] if lane == 'true' else [])
    if wanted and not sensors:
        raise ValueError(f'{", ".join(wanted)}: world:={world} has no camera. Use world:=road')
    nav_share = FindPackageShare('aav_navigation').perform(context)
    actions = [
        # Every node on simulated time, those in the included files too.
        SetParameter(name='use_sim_time', value=True),
        include(context, 'aav_gazebo', 'gazebo.launch.py', world=world, gui=arg('gui'),
                obstacle=arg('obstacle'), start_y=arg('start_y'), start_yaw=arg('start_yaw')),
        include(context, 'aav_mux', 'mux.launch.py', use_sim_time='true'),
        include(context, 'aav_ego', 'ego.launch.py', use_sim_time='true'),
        include(context, 'aav_scripted', 'scripted.launch.py', use_sim_time='true'),
        # Only the IMU filter: Gazebo stands in for the drivers.
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick',
             parameters=[os.path.join(FindPackageShare('aav_sensors').perform(context),
                                      'config', 'imu_filter.yaml'), {'use_sim_time': True}],
             remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')]),
        include(context, 'aav_localization', 'localization.launch.py', use_sim_time='true'),
    ]
    if sensors:
        # The radar stand-in, as the bench's: few points per object, so small clusters.
        actions.append(include(context, 'aav_object_detection', 'radar_clusters.launch.py',
                               minimum_points='5'))
    if flag['nav']:
        # Where the world renders the radar, Nav2's local costmap reads it, and
        # collision_monitor guards Nav2's commands, as on a car with a radar.
        actions.append(include(context, 'aav_navigation', 'navigation.launch.py',
                               use_sim_time='true',
                               map=os.path.join(nav_share, 'maps', MAPS[world]),
                               radar=sensors, collision_monitor=sensors,
                               camera_obstacles=flag['camera_obstacles']))
    if flag['teleop']:
        actions.append(include(context, 'aav_teleop', 'teleop.launch.py', use_sim_time='true'))
    if flag['perception']:
        actions += [
            include(context, 'aav_yolo', 'detector.launch.py', labels='stop sign',
                    use_sim_time='true'),
            Node(package='aav_yolo', executable='stop_sign_watcher', name='stop_sign_watcher',
                 parameters=[{'use_sim_time': True}]),
        ]
    if flag['camera_obstacles']:
        actions.append(Node(package='aav_yolo', executable='camera_obstacles',
                            name='camera_obstacles',
                            parameters=[os.path.join(FindPackageShare('aav_yolo')
                                                     .perform(context), 'config',
                                                     'camera_obstacles.yaml'),
                                        {'use_sim_time': True}]))
    if lane in ('true', 'fake'):
        actions.append(include(context, 'aav_lane_geometry', 'lane_follower.launch.py',
                               fake=lane == 'fake'))
    if lane == 'true':
        actions.append(include(context, 'aav_lane_finder', 'lane_finder.launch.py'))
    if flag['hud']:
        actions.append(include(context, 'aav_hud', 'hud.launch.py', use_sim_time='true'))
    if flag['rviz']:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(FindPackageShare('aav_bringup').perform(context),
                                          'rviz', LAYOUTS.get(world, 'drive.rviz'))],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description='empty, test_lot or road'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='true if a joystick is plugged into this computer'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('perception', default_value='false',
                              description='YOLO and the stop-sign watcher (the ml '
                                          'environment; world:=road)'),
        DeclareLaunchArgument('lane', default_value='false',
                              description='true: lane keeping (lane_finder and '
                                          'control_interface, on /cmd_vel/ai; world:=road); '
                                          'fake: the same on a pretend lane'),
        DeclareLaunchArgument('camera_obstacles', default_value='false',
                              description="the detectors' boxes into the local costmap "
                                          '(world:=road)'),
        DeclareLaunchArgument('hud', default_value='false',
                              description="the HUD's web server (aav_hud), on 127.0.0.1:5000"),
        DeclareLaunchArgument('obstacle', default_value='false',
                              description="a 1 m box in the car's lane, 15 m ahead"),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description="m: start the car this far left of the road's centre"),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='rad: start the car turned this far left of east '
                                          '(the EKFs still start facing east)'),
        OpaqueFunction(function=launch_setup),
    ])
