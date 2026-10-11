"""
What each Gazebo world gives the car. No ROS imports: the launch files and tests read it.

A world renders the car's camera and radar stand-in (aav_description's
aav.sensors.gazebo.xacro) only if it loads Gazebo's Sensors system, which needs a GPU,
or ogre2 on a Mac. So the world file itself says whether the car has them: today only
worlds/road.sdf does. The empty and test_lot worlds render nothing, and run headless
anywhere.
"""

from __future__ import annotations

import os
from typing import Tuple
from xml.etree import ElementTree

SENSORS = 'gz::sim::systems::Sensors'

# The bench's obstacle (aav_bench/config/bench.yaml mock_radar: boxes 15-16 m east, -0.5 to
# 0.5 m north): where obstacle:=true puts models/obstacle, the centre of its base (m).
OBSTACLE: Tuple[float, float] = (15.5, 0.0)


def world_file(share: str, world: str) -> str:
    """Return the path of a world in aav_gazebo's share folder, or say which ones exist."""
    folder = os.path.join(share, 'worlds')
    path = os.path.join(folder, f'{world}.sdf')
    if not os.path.isfile(path):
        known = sorted(f[:-len('.sdf')] for f in os.listdir(folder) if f.endswith('.sdf'))
        raise ValueError(f'world:={world}: no such world. Pick one of {known}')
    return path


def has_sensors(path: str) -> bool:
    """Return True if the world loads the Sensors system, so it renders cameras and lidars."""
    root = ElementTree.parse(path).getroot()
    return any(p.get('name') == SENSORS for p in root.iter('plugin'))
