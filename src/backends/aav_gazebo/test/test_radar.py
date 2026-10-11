"""The radar stand-in's points, turned into the TI radar's: no ROS needed."""

import math

from aav_gazebo import radar
import numpy as np


def test_keeps_only_the_rays_that_hit_something():
    inf = math.inf
    xyz = np.array([[2.0, 0.0, 0.1],          # a hit
                    [inf, inf, -inf],          # a ray that hit nothing
                    [inf, math.nan, inf],      # inf times 0: NaN
                    [9.0, 0.0, 0.0],           # past 8.4 m
                    [8.4, 0.0, 0.0]])          # just in range
    assert np.allclose(radar.returns(xyz, 8.4), [[2.0, 0.0, 0.1], [8.4, 0.0, 0.0]])


def test_nothing_seen_is_an_empty_cloud():
    kept = radar.returns(np.full((183, 3), math.inf), 8.4)
    assert kept.shape == (0, 3)
    assert radar.ti_points(kept, 1.0).shape == (0, len(radar.FIELDS))


def test_radial_speed_is_the_cars_own_along_the_line_of_sight():
    points = radar.ti_points(np.array([[4.0, 0.0, 0.0], [3.0, 4.0, 0.0], [0.0, 2.0, 0.0]]),
                             speed=1.0)
    assert points.dtype == np.float32
    assert np.allclose(points[:, :3], [[4.0, 0.0, 0.0], [3.0, 4.0, 0.0], [0.0, 2.0, 0.0]])
    assert np.allclose(points[:, 3], 1.0)
    # Dead ahead it closes at the car's speed; at 53 degrees, at 0.6 of it; abeam, not at all.
    assert np.allclose(points[:, 4], [-1.0, -0.6, 0.0])


def test_a_point_at_the_radar_itself_has_no_speed():
    assert radar.ti_points(np.zeros((1, 3)), 2.0)[0, 4] == 0.0


def test_fields_are_the_ti_drivers():
    assert radar.FIELDS == ('x', 'y', 'z', 'intensity', 'velocity')
