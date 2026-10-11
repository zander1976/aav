"""
Radar clustering: DBSCAN groups the radar's points into objects, with a box round each.

  ros2 launch aav_radar_clusters radar_clusters.launch.py               # the real radar
  ros2 launch aav_radar_clusters radar_clusters.launch.py mock:=true minimum_points:=5 \
      radar_rviz:=true                                          # pretend points, in RViz

Subscribes  /radar/points          sensor_msgs/PointCloud2, best effort (aav_sensors starts
                                   the radar; the bench, its stand-in)
Publishes   /radar/clusters        sensor_msgs/PointCloud2: the clustered points, coloured
            /radar/cluster_boxes   visualization_msgs/MarkerArray: a box round each cluster

You can set: eps (metres: points this close are neighbours), minimum_points (the
neighbours a point needs, itself included, to start or grow a cluster), mock (pretend
radar points, from ti_ros2_driver's mock_radar_publisher: clusters of 10 to 14 points, so
give minimum_points:=5 with it), radar_rviz (RViz with these three topics, in radar_link).
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    rviz = get_package_share_path('aav_radar_clusters') / 'rviz' / 'radar_clusters.rviz'
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('eps', default_value='0.8', description='m'),
        DeclareLaunchArgument('minimum_points', default_value='15'),
        DeclareLaunchArgument('radar_rviz', default_value='false',
                              description='RViz with the radar points, clusters and boxes'),
        Node(
            package='ti_ros2_driver', executable='mock_radar_publisher', name='mock_radar',
            parameters=[{'topic': '/radar/points', 'frame_id': 'radar_link', 'mode': 'static'}],
            condition=IfCondition(arg('mock')),
        ),
        Node(
            package='aav_radar_clusters', executable='radar_clusters', name='radar_clusters',
            parameters=[{
                'eps': ParameterValue(arg('eps'), value_type=float),
                'minimum_points': ParameterValue(arg('minimum_points'), value_type=int),
            }],
            output='screen',
        ),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(rviz)],
             condition=IfCondition(arg('radar_rviz'))),
    ])
