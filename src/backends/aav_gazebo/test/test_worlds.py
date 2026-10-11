"""
The Gazebo worlds: which ones render the car's sensors, and the road world against the bench.

No ROS needed. The road world must stay the bench world (aav_bench/config/bench.yaml): the
same lane, paint, stop sign and obstacle, so a check passed on one means the same on the
other. Every world sits on the GPS datum (aav_localization/config/navsat.yaml).
"""

from pathlib import Path
from xml.etree import ElementTree

from aav_gazebo import worlds
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE.parents[1]
BENCH = yaml.safe_load((SRC / 'bench' / 'aav_bench' / 'config' / 'bench.yaml').read_text())
CAMERA = BENCH['mock_camera']['ros__parameters']
RADAR = BENCH['mock_radar']['ros__parameters']


def sdf(path):
    return ElementTree.parse(path).getroot()


def numbers(text):
    return [float(v) for v in text.split()]


def visual(root, name):
    found = [v for v in root.iter('visual') if v.get('name') == name]
    assert len(found) == 1, f'one visual called {name}'
    return found[0]


def test_world_files_and_an_unknown_world():
    for world in ('empty', 'test_lot', 'road'):
        assert worlds.world_file(str(PACKAGE), world).endswith(f'worlds/{world}.sdf')
    with pytest.raises(ValueError, match='road'):
        worlds.world_file(str(PACKAGE), 'moon')


def test_only_the_road_world_renders_sensors():
    def renders(world):
        return worlds.has_sensors(worlds.world_file(str(PACKAGE), world))
    assert renders('road')
    assert not renders('empty')      # headless anywhere: CI on Linux, oirp-spss
    assert not renders('test_lot')


def test_every_world_is_named_aav_and_sits_on_the_datum():
    navsat = SRC / 'localization' / 'aav_localization' / 'config' / 'navsat.yaml'
    lat, lon, _ = yaml.safe_load(navsat.read_text())['navsat_transform']['ros__parameters'][
        'datum']
    for path in sorted((PACKAGE / 'worlds').glob('*.sdf')):
        world = sdf(path).find('world')
        assert world.get('name') == 'aav', path.name   # the bridges' topics name the world
        coordinates = world.find('spherical_coordinates')
        assert float(coordinates.findtext('latitude_deg')) == lat, path.name
        assert float(coordinates.findtext('longitude_deg')) == lon, path.name
        assert float(coordinates.findtext('heading_deg')) == 0.0, path.name


def test_the_road_is_the_bench_road():
    road = sdf(PACKAGE / 'worlds' / 'road.sdf')
    half = CAMERA['lane_width'] / 2.0
    tarmac = visual(road, 'tarmac')
    assert numbers(tarmac.find('pose').text)[1] == CAMERA['road_y']
    assert numbers(tarmac.find('geometry/box/size').text)[1] == pytest.approx(
        2.0 * (half + CAMERA['shoulder']))
    for name, side in (('left_line', 1.0), ('right_line', -1.0)):
        line = visual(road, name)
        assert numbers(line.find('pose').text)[1] == pytest.approx(CAMERA['road_y'] + side * half)
        assert numbers(line.find('geometry/box/size').text)[1] == CAMERA['line_width']
    # The road starts behind the car and runs well past the stop sign.
    x, length = numbers(tarmac.find('pose').text)[0], numbers(tarmac.find('geometry/box/size')
                                                              .text)[0]
    assert x - length / 2.0 <= -10.0 and x + length / 2.0 >= 60.0


def test_the_stop_sign_is_the_benchs():
    road = sdf(PACKAGE / 'worlds' / 'road.sdf')
    signs = [i for i in road.iter('include') if i.findtext('uri') == 'model://stop_sign']
    assert len(signs) == 1
    x, y, z, _, _, yaw = numbers(signs[0].findtext('pose'))
    assert [x, y] == CAMERA['stop_signs']
    assert yaw == pytest.approx(3.14159265)      # facing the traffic, which drives east
    plate = visual(sdf(PACKAGE / 'models' / 'stop_sign' / 'model.sdf'), 'plate')
    assert numbers(plate.find('pose').text)[2] == CAMERA['sign_height']
    mesh = (PACKAGE / 'models' / 'stop_sign' / 'meshes' / 'stop_sign.obj').read_text()
    corners = [numbers(line[2:]) for line in mesh.splitlines() if line.startswith('v ')]
    assert 2.0 * max(abs(c[1]) for c in corners) == pytest.approx(CAMERA['sign_size'])


def test_the_obstacle_is_the_benchs():
    x1, y1, x2, y2 = RADAR['boxes']
    assert worlds.OBSTACLE == ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
    box = sdf(PACKAGE / 'models' / 'obstacle' / 'model.sdf')
    size = numbers(box.find('.//visual/geometry/box/size').text)
    assert size == [x2 - x1, y2 - y1, RADAR['box_height']]
    assert numbers(box.find('.//link/pose').text)[2] == RADAR['box_height'] / 2.0  # on the ground
