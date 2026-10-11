"""
Nav2 as one more driver: goals in, /cmd_vel/nav (TwistStamped) out to twist_mux.

  ros2 launch aav_navigation navigation.launch.py [map:=<path to map yaml>]
      [radar:=false] [collision_monitor:=<radar>] [camera_obstacles:=false]
      [use_sim_time:=false] [autostart:=true]

Starts only the Nav2 servers a car needs (not nav2_bringup's full list: no AMCL, since
robot_localization does localization, no docking or route server):

  map_server        the map (default: maps/empty.yaml, everything free)
  planner_server    SmacPlannerHybrid: drivable paths for a car
  controller_server Regulated Pure Pursuit -> cmd_vel_nav
  behavior_server   wait, backup, drive_on_heading (no spin)
  bt_navigator      navigate_to_pose, navigate_through_poses (behavior_trees/*_car.xml)
  waypoint_follower follow_waypoints, follow_gps_waypoints (lat/lon via /fromLL)
  velocity_smoother cmd_vel_nav -> /cmd_vel/nav (or -> collision_monitor -> /cmd_vel/nav)
  collision_monitor /cmd_vel_smoothed -> /cmd_vel/nav, stopping for radar points ahead
  lifecycle_manager brings them all up

Obstacles, from where the sensors are:
  radar:=true             where a radar publishes /radar/points (the bench; the car with
                          its radar part; Gazebo's road world): the local costmap's obstacle
                          layer reads it, so Nav2 stops for what the radar sees
  collision_monitor       follows radar unless set: it stops the car for radar points in
                          the stop zone ahead, whatever the costmap says. A silent radar
                          stops Nav2 too
  camera_obstacles:=true  the local costmap's camera layer reads aav_yolo camera_obstacles
                          (the detectors' boxes on the road). Off until the cameras are
                          calibrated (Q22)

The car's footprint and turning radius come from aav_description/config/vehicle.yaml.
Send a goal: RViz "2D Goal Pose" (publishes /goal_pose), or
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 20.0, y: 5.0}}}}"
"""

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


# The local costmap's parameters, as a whole path for RewrittenYaml: a leaf key such as
# `enabled` would change every key of that name in nav2.yaml.
LOCAL = 'local_costmap.local_costmap.ros__parameters.'


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def nodes(context):
    share = FindPackageShare('aav_navigation').perform(context)
    use_sim_time = flag(context, 'use_sim_time')
    radar = flag(context, 'radar')
    collision_monitor = flag(context, 'collision_monitor')
    camera = flag(context, 'camera_obstacles')

    rewrites = vehicle.nav2_rewrites(vehicle.load()['vehicle'])
    rewrites.update({
        'yaml_filename': LaunchConfiguration('map').perform(context),
        'default_nav_to_pose_bt_xml': f'{share}/behavior_trees/navigate_to_pose_car.xml',
        'default_nav_through_poses_bt_xml':
            f'{share}/behavior_trees/navigate_through_poses_car.xml',
        LOCAL + 'obstacle_layer.enabled': str(radar).lower(),
        LOCAL + 'camera_layer.enabled': str(camera).lower(),
    })
    params = ParameterFile(
        RewrittenYaml(source_file=f'{share}/config/nav2.yaml', param_rewrites=rewrites,
                      convert_types=True),
        allow_substs=True)

    smoothed = 'cmd_vel_smoothed' if collision_monitor else '/cmd_vel/nav'
    servers = [
        ('nav2_map_server', 'map_server', []),
        ('nav2_planner', 'planner_server', []),
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', smoothed)]),
    ]
    if collision_monitor:
        servers.append(('nav2_collision_monitor', 'collision_monitor', []))

    # use_sim_time is passed on its own: RewrittenYaml only rewrites keys that already
    # exist, and nav2.yaml has none, so as a rewrite it was silently dropped.
    actions = [
        Node(package=pkg, executable=exe, name=exe, output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}], remappings=remaps)
        for pkg, exe, remaps in servers
    ]
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': flag(context, 'autostart'),
                     'node_names': [name for _, name, _ in servers]}],
    ))
    return actions


def generate_launch_description():
    empty_map = PathJoinSubstitution([FindPackageShare('aav_navigation'), 'maps', 'empty.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=empty_map,
                              description='Map YAML (see maps/README.md)'),
        DeclareLaunchArgument('radar', default_value='false',
                              description='true where a radar publishes /radar/points: the '
                                          "local costmap's obstacle layer reads it"),
        # The radar's value unless set: collision_monitor needs /radar/points, and stops
        # Nav2 when it goes silent.
        DeclareLaunchArgument('collision_monitor', default_value=LaunchConfiguration('radar'),
                              description='Insert collision_monitor (needs /radar/points)'),
        DeclareLaunchArgument('camera_obstacles', default_value='false',
                              description="true: the local costmap's camera layer reads "
                                          '/perception/camera_obstacles'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=nodes),
    ])
