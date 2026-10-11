"""
The Gazebo car's radar stand-in, made to look like the TI radar. No ROS imports.

Gazebo's lidar (aav_description's aav.sensors.gazebo.xacro) sends a point for every ray:
where a ray hit nothing, the point is endless (inf, or NaN where inf meets 0), and its
fields are Gazebo's own (x, y, z, intensity, ring). The TI driver (ti_ros2_driver) sends
only what it saw, with x, y, z, intensity and velocity, the radial speed. These turn the
first into the second, so radar_clusters, Nav2's costmap and collision_monitor read the
same kind of cloud in Gazebo as on the car, and PCL finds every field it expects.
"""

from __future__ import annotations

import numpy as np

# The TI driver's fields, in its order (ti_ros2_driver pc_publisher.py), all float32.
FIELDS = ('x', 'y', 'z', 'intensity', 'velocity')


def returns(xyz: np.ndarray, max_range: float) -> np.ndarray:
    """Return the rays that hit something: the finite points no farther than max_range (m)."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    finite = np.isfinite(xyz).all(axis=1)
    near = np.zeros(len(xyz), dtype=bool)
    near[finite] = np.linalg.norm(xyz[finite], axis=1) <= max_range
    return xyz[near]


def ti_points(xyz: np.ndarray, speed: float, intensity: float = 1.0) -> np.ndarray:
    """
    Return each point as the TI driver's x, y, z, intensity and velocity (N x 5, float32).

    The world stands still, so a point's radial speed is the car's own, seen along the line
    from the radar to it: negative while the car closes on it, as the TI radar reports.
    The car's turning is left out, as the bench's pretend radar leaves it out.
    """
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    distance = np.linalg.norm(xyz, axis=1)
    along = np.divide(xyz[:, 0], distance, out=np.zeros_like(distance), where=distance > 0)
    out = np.empty((len(xyz), len(FIELDS)), dtype=np.float32)
    out[:, :3] = xyz
    out[:, 3] = intensity
    out[:, 4] = -speed * along
    return out
