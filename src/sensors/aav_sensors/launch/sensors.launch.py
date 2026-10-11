"""
Sensor drivers: each publishes raw data in its own frame; nothing here fuses anything.

  ros2 launch aav_sensors sensors.launch.py [imu:=true] [gps:=true] [radar:=false]
      [battery:=false] [imu_driver:=rust] [gps_driver:=ublox]

  IMU     imu_node -> /imu/data_raw -> imu_filter_madgwick -> /imu/data
          imu_node is aav_imu_yahboom (Rust), or with imu_driver:=cpp the 2025-26
          C++ driver yahboom_imu_cpp: the same node name, parameters and topics.
  GPS     one of three drivers, each -> /gps/fix (NavSatFix, frame gps_link):
            ublox        ublox_gps (KumarRobotics' ublox): a u-blox on /dev/aav/gps, as
                         the receiver is already set up (config/ublox_gps.yaml). The
                         default, Tentative until the receiver is known (Q18). In Pixi too
            nmea         nmea_navsat_driver: any NMEA receiver (gps_nmea.launch.py)
            ublox_dgnss  ublox_dgnss -> ublox_nav_sat_fix_hp: a u-blox F9P over libusb,
                         the 2024-25 choice. apt only: the car, not Pixi
  radar   ti_ros2_driver pcl_pub -> /radar/points (PointCloud2, frame radar_link)
  battery battery_monitor -> /battery_status (BatteryState)

Frames (imu_link, gps_link, radar_link) come from aav_description; run its
description.launch.py too, or nothing can place these readings on the car.
Run this file on the machine the sensors are plugged into.
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import evaluate_condition_expression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

IMU_DRIVERS = {'rust': 'aav_imu_yahboom', 'cpp': 'yahboom_imu_cpp'}
# Each GPS driver, and the package it needs.
GPS_DRIVERS = {'ublox': 'ublox_gps', 'nmea': 'nmea_navsat_driver',
               'ublox_dgnss': 'ublox_dgnss_node'}

# u-blox receiver settings for ublox_dgnss, as in its ublox_rover_hpposllh_navsatfix.launch.py
# (0.7.4) except the rate. The NavSatFix node needs HPPOSLLH, STATUS and COV.
UBLOX_DGNSS_PARAMS = {
    'DEVICE_FAMILY': 'F9P',             # TODO confirm the team's receiver (F9P, F9R or X20P)
    'DEVICE_SERIAL_STRING': '',         # '' = the first u-blox found on USB
    'FRAME_ID': 'gps_link',
    'CFG_USBOUTPROT_NMEA': False,
    'CFG_RATE_MEAS': 200,               # ms between measurements: 5 Hz. TODO tune
    'CFG_RATE_NAV': 1,                  # one solution per measurement
    'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1,
    'CFG_MSGOUT_UBX_NAV_STATUS_USB': 1,
    'CFG_MSGOUT_UBX_NAV_COV_USB': 1,
    'CFG_MSGOUT_UBX_NAV_PVT_USB': 1,
}


def installed(package):
    try:
        get_package_share_directory(package)
    except PackageNotFoundError:
        return False
    return True


def gps(driver, config, use_sim_time):
    """Return the GPS driver's actions: every driver publishes /gps/fix in gps_link."""
    if not installed(GPS_DRIVERS[driver]):
        hint = (' ublox_dgnss is apt only, for the car: gps_driver:=ublox runs anywhere'
                if driver == 'ublox_dgnss' else '')
        get_logger('sensors.launch.py').warning(
            f'gps: not started: {GPS_DRIVERS[driver]} is not installed here.{hint}')
        return []
    if driver == 'ublox':
        return [Node(
            package='ublox_gps', executable='ublox_gps_node', name='ublox_gps',
            parameters=[os.path.join(config, 'ublox_gps.yaml'), use_sim_time],
            remappings=[('fix', '/gps/fix'), ('fix_velocity', '/gps/fix_velocity')],
        )]
    if driver == 'nmea':
        nmea = os.path.join(get_package_share_directory('aav_sensors'), 'launch',
                            'gps_nmea.launch.py')
        # Pass both: included files share launch arguments.
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(nmea),
                                         launch_arguments={'port': '/dev/aav/gps',
                                                           'baud': '9600'}.items())]
    # ublox_dgnss: the driver and the NavSatFix converter, in one process.
    return [ComposableNodeContainer(
        name='gps_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                parameters=[UBLOX_DGNSS_PARAMS, use_sim_time],
            ),
            ComposableNode(
                package='ublox_nav_sat_fix_hp_node',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                name='ublox_nav_sat_fix_hp',
                parameters=[use_sim_time],
                remappings=[('fix', '/gps/fix')],
            ),
        ],
    )]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def on(name):
        # As IfCondition reads it: true or 1, false or 0; anything else is an error.
        return evaluate_condition_expression(context, [LaunchConfiguration(name)])

    def choice(name, options):
        value = arg(name)
        if value not in options:
            raise RuntimeError(f'{name}:={value}: pick one of {", ".join(options)}')
        return value

    config = os.path.join(get_package_share_directory('aav_sensors'), 'config')
    use_sim_time = {'use_sim_time': on('use_sim_time')}
    actions = []
    if on('imu'):
        # Either driver takes the same config/imu.yaml.
        actions += [
            Node(package=IMU_DRIVERS[choice('imu_driver', IMU_DRIVERS)], executable='imu_node',
                 name='imu_node',
                 parameters=[os.path.join(config, 'imu.yaml'), use_sim_time]),
            Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
                 name='imu_filter_madgwick',
                 parameters=[os.path.join(config, 'imu_filter.yaml'), use_sim_time],
                 remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data'),
                             ('imu/mag', '/imu/mag')]),
        ]
    if on('gps'):
        actions += gps(choice('gps_driver', GPS_DRIVERS), config, use_sim_time)
    if on('radar'):
        # Perception, later: the out-of-box cfg, which publishes x forward, y left.
        actions.append(Node(
            package='ti_ros2_driver', executable='pcl_pub', name='radar',
            parameters=[{
                'cfg_path': os.path.join(get_package_share_directory('ti_ros2_driver'),
                                         'cfg', 'base.cfg'),
                'command_port': '/dev/aav/radar_cmd',
                'data_port': '/dev/aav/radar_data',
                'frame_id': 'radar_link',
                'topic': '/radar/points',
            }, use_sim_time],
        ))
    if on('battery'):
        # The Victron VE.Direct cable. Its `port` parameter defaults to the udev name
        # /dev/aav/battery.
        actions.append(Node(package='battery_monitor', executable='battery_monitor'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('gps', default_value='true'),
        DeclareLaunchArgument('radar', default_value='false'),
        DeclareLaunchArgument('battery', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu_driver', default_value='rust',
                              description='rust: aav_imu_yahboom; cpp: yahboom_imu_cpp'),
        DeclareLaunchArgument('gps_driver', default_value='ublox',
                              description='ublox: ublox_gps; nmea: nmea_navsat_driver; '
                                          'ublox_dgnss: ublox_dgnss (apt only). Tentative: '
                                          'Q18'),
        OpaqueFunction(function=launch_setup),
    ])
