"""
sim.launch.py and aav_gazebo's gazebo.launch.py: what each world and argument starts.

Each test runs a launch file's own setup with some arguments, and looks at what it would
start. Nothing starts: no ROS, no Gazebo. The defaults must start what `pixi run sim`
always has; the road world adds the car's sensors and the parts that read them.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetParameter
from launch_ros.utilities import evaluate_parameters
import pytest


def load(package, name):
    path = os.path.join(get_package_share_directory(package), 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SIM = load('aav_bringup', 'sim.launch.py')
GAZEBO = load('aav_gazebo', 'gazebo.launch.py')
# What the parts that read the camera are called: they need the road world.
CAMERA_PARTS = [{'perception': 'true'}, {'camera_obstacles': 'true'}, {'lane': 'true'}]


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


# A Node's name, arguments and parameters: launch_ros keeps them to itself until the
# node starts, so the tests read its own fields.
def node(actions, name):
    """Return the node started directly under this name."""
    return next(a for a in actions if isinstance(a, Node) and a._Node__node_name == name)


def arguments(a):
    return [str(x) for x in a._Node__arguments]


def parameters(a):
    """Return the node's parameters that are given as values (not files), as one dict."""
    found = {}
    for p in evaluate_parameters(LaunchContext(), a._Node__parameters):
        if isinstance(p, dict):
            found.update(p)
    return found


def names(actions):
    """Return the names of the nodes and processes started directly."""
    found = set()
    for a in actions:
        if isinstance(a, Node):
            found.add(a._Node__node_name)
        elif isinstance(a, ExecuteProcess):
            found.add(perform_substitutions(LaunchContext(), a.name))
    return found


def test_the_defaults_start_what_pixi_run_sim_always_has():
    actions = started(SIM)
    sent = included(actions)
    assert set(sent) == {'gazebo.launch.py', 'mux.launch.py', 'ego.launch.py',
                         'scripted.launch.py', 'localization.launch.py',
                         'navigation.launch.py'}
    assert sent['gazebo.launch.py']['world'] == 'empty'
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['collision_monitor'], nav['camera_obstacles']) == (
        'false', 'false', 'false')
    assert nav['map'].endswith(os.path.join('maps', 'empty.yaml'))
    assert names(actions) == {'imu_filter_madgwick', 'rviz2'}
    assert arguments(node(actions, 'rviz2'))[1].endswith('drive.rviz')


def test_every_node_runs_on_simulated_time():
    first = started(SIM)[0]
    assert isinstance(first, SetParameter)
    assert perform_substitutions(LaunchContext(), list(first.name)) == 'use_sim_time'
    assert first.value is True


def test_the_road_world_adds_the_radar_to_nav2_and_its_clusters():
    actions = started(SIM, world='road')
    assert arguments(node(actions, 'rviz2'))[1].endswith('road.rviz')
    sent = included(actions)
    assert sent['radar_clusters.launch.py'] == {'minimum_points': '5'}
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['collision_monitor']) == ('true', 'true')
    assert nav['map'].endswith(os.path.join('maps', 'empty.yaml'))


def test_the_road_worlds_full_graph():
    actions = started(SIM, world='road', perception='true', lane='true',
                      camera_obstacles='true', hud='true', obstacle='true', start_y='0.3')
    sent = included(actions)
    assert sent['detector.launch.py'] == {'labels': 'stop sign', 'use_sim_time': 'true'}
    assert sent['lane_finder.launch.py'] == {}
    assert sent['lane_follower.launch.py'] == {'fake': 'false'}
    assert sent['hud.launch.py'] == {'use_sim_time': 'true'}
    assert sent['navigation.launch.py']['camera_obstacles'] == 'true'
    assert (sent['gazebo.launch.py']['obstacle'], sent['gazebo.launch.py']['start_y']) == (
        'true', '0.3')
    assert {'stop_sign_watcher', 'camera_obstacles'} <= names(actions)


@pytest.mark.parametrize('part', CAMERA_PARTS)
def test_the_cameras_parts_need_a_world_with_a_camera(part):
    for world in ('empty', 'test_lot'):
        with pytest.raises(ValueError, match='world:=road'):
            started(SIM, world=world, **part)


def test_a_pretend_lane_needs_no_camera():
    assert included(started(SIM, lane='fake'))['lane_follower.launch.py'] == {'fake': 'true'}


def test_bad_values_stop_the_launch():
    with pytest.raises(ValueError, match='pick one of'):
        started(SIM, world='moon')
    with pytest.raises(ValueError, match='lane:=maybe'):
        started(SIM, lane='maybe')


def test_gazebo_bridges_the_sensors_only_where_the_world_renders_them():
    for world in ('empty', 'test_lot'):
        actions = started(GAZEBO, world=world, gui='false')
        assert 'gz_road_bridge' not in names(actions), world
        assert parameters(node(actions, 'gazebo_adapter'))['radar'] is False, world
    road = started(GAZEBO, world='road', gui='false')
    assert 'gz_road_bridge' in names(road)
    assert parameters(node(road, 'gazebo_adapter'))['radar'] is True
    assert parameters(node(road, 'gz_road_bridge'))['config_file'].endswith('bridge_road.yaml')


def test_gazebo_obstacle_and_start_pose():
    actions = started(GAZEBO, world='road', gui='false', obstacle='true', start_y='0.3',
                      start_yaw='0.1')
    spawn = arguments(node(actions, 'spawn_aav'))
    assert spawn[spawn.index('-y') + 1] == '0.3' and spawn[spawn.index('-Y') + 1] == '0.1'
    box = arguments(node(actions, 'spawn_obstacle'))
    assert box[box.index('-file') + 1].endswith(os.path.join('obstacle', 'model.sdf'))
    assert (box[box.index('-x') + 1], box[box.index('-y') + 1]) == ('15.5', '0.0')
    assert 'spawn_obstacle' not in names(started(GAZEBO, world='road', gui='false'))


def test_gazebo_runs_the_simulator_headless_without_a_window():
    gz = included(started(GAZEBO, world='empty', gui='false'))['gz_sim.launch.py']['gz_args']
    assert gz.startswith('-r -s ') and gz.endswith('empty.sdf')
    # Only Linux renders through EGL; a Mac's ogre2 renders headless on Metal as it is.
    road = included(started(GAZEBO, world='road', gui='false'))['gz_sim.launch.py']['gz_args']
    assert ('--headless-rendering' in road) == (not GAZEBO.MAC)
    assert '--headless-rendering' not in gz
