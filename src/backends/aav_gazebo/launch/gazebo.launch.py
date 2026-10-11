"""
Gazebo Harmonic as the car: world, the car from the URDF, the bridge and the adapter.

  ros2 launch aav_gazebo gazebo.launch.py [world:=empty|test_lot|road] [gui:=true]
      [follow:=true] [obstacle:=false] [start_y:=0.0] [start_yaw:=0.0]

  /ego/actuator_cmd ──> gazebo_adapter ──> /gazebo/cmd_vel ──bridge──> Ackermann system
  Gazebo odometry, IMU, NavSat ──bridge──> gazebo_adapter ──> /vehicle/odom,
                                           /imu/data_raw, /gps/fix, /vehicle/status
  Gazebo joints ──bridge──> /joint_states;  Gazebo clock ──bridge──> /clock

The road world also renders the car's camera and its radar stand-in, a lidar shaped like
the TI radar (aav_gazebo worlds.has_sensors). A second bridge, gz_road_bridge, carries them:

  camera ──road bridge──> /camera/image_raw, /camera/camera_info (camera_left_link)
  lidar ──road bridge──> /gazebo/radar/points ──> gazebo_adapter ──> /radar/points
                                                  (radar_link, as the TI driver's)
  Gazebo's truth ──road bridge──> /gazebo/truth (where the car really is, for checks)

obstacle:=true puts the bench's 1 m box in the car's lane, 15 m ahead. start_y and
start_yaw put the car this far left of the world's origin, turned this far left of east
(the EKFs still start at the origin, facing east, as on the bench).

Everything else (sensors filter, localization, Nav2, mux, ego) runs as on the car,
with use_sim_time:=true. aav_bringup sim.launch.py starts the lot.

On a Mac, Gazebo's window can't run in the same program as the simulator, so it starts
as a second program here: `pixi run sim` works the same on a Mac and on Linux. The
cameras render without a window on a Mac (ogre2 on Metal). On Linux without a window,
the simulator renders with EGL (--headless-rendering). follow:=true points the window's
camera at the car once the window is up.
"""

import os
import sys

from aav_description import vehicle
from aav_gazebo import worlds
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MAC = sys.platform == 'darwin'

# Point the window's camera at the car, close behind it and to its right. Wait for the
# car to appear first: asked too early, the camera ends up looking at nothing. Then keep
# asking until the window answers (it takes a few seconds to start).
CAR = '"aav"'
FOLLOW = ('for i in $(seq 1 60); do gz model --list 2> /dev/null | grep -q -- "- aav$" && '
          'break; sleep 1; done; '
          'for i in $(seq 1 30); do '
          'gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean '
          f"--timeout 2000 --req 'data: {CAR}' > /dev/null 2>&1 && "
          'gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d '
          "--reptype gz.msgs.Boolean --timeout 2000 --req 'x: -3.5, y: -2.5, z: 1.8' "
          '> /dev/null 2>&1 && exit 0; sleep 1; done')


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def launch_setup(context):
    share = FindPackageShare('aav_gazebo').perform(context)
    world = worlds.world_file(share, LaunchConfiguration('world').perform(context))
    sensors = worlds.has_sensors(world)
    gui = flag(context, 'gui')
    # -r: start running; -s: server only (no window). On a Mac the window is always its
    # own program (below). On Linux with no window, the sensors render through EGL.
    if MAC or not gui:
        gz_args = '-r -s ' + ('--headless-rendering ' if sensors and not MAC else '') + world
    else:
        gz_args = '-r ' + world
    v = vehicle.load()['vehicle']
    sim_time = {'use_sim_time': True}
    start = ['-y', LaunchConfiguration('start_y').perform(context),
             '-Y', LaunchConfiguration('start_yaw').perform(context)]
    actions = [
        IncludeLaunchDescription(
            os.path.join(FindPackageShare('ros_gz_sim').perform(context), 'launch',
                         'gz_sim.launch.py'),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            os.path.join(FindPackageShare('aav_description').perform(context), 'launch',
                         'description.launch.py'),
            launch_arguments={'sim': 'true', 'use_sim_time': 'true',
                              'joint_state_publisher': 'false'}.items()),
        Node(package='ros_gz_sim', executable='create', name='spawn_aav', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'aav', '-z', '0.05', *start]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')},
                         sim_time]),
        Node(package='aav_gazebo', executable='gazebo_adapter', name='gazebo_adapter',
             parameters=[{'wheelbase': float(v['wheelbase']), 'radar': sensors}, sim_time]),
    ]
    if gui and MAC:
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g'], name='gz_window',
                                      output='screen'))
    if gui and flag(context, 'follow'):
        actions.append(ExecuteProcess(cmd=[FOLLOW], shell=True, name='follow_car'))
    if sensors:
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_road_bridge',
            parameters=[{'config_file': os.path.join(share, 'config', 'bridge_road.yaml')},
                        sim_time]))
    if flag(context, 'obstacle'):
        x, y = worlds.OBSTACLE
        actions.append(Node(
            package='ros_gz_sim', executable='create', name='spawn_obstacle', output='screen',
            arguments=['-file', os.path.join(share, 'models', 'obstacle', 'model.sdf'),
                       '-name', 'obstacle', '-x', str(x), '-y', str(y)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description='empty, test_lot or road'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('follow', default_value='true',
                              description="point the window's camera at the car"),
        DeclareLaunchArgument('obstacle', default_value='false',
                              description="the bench's 1 m box in the car's lane, 15 m ahead"),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description='m: start the car this far left (north) of the '
                                          "world's origin"),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='rad: start the car turned this far left of east'),
        OpaqueFunction(function=launch_setup),
    ])
