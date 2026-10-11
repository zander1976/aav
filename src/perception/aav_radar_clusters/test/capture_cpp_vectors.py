#!/usr/bin/env python3
"""
Record what the C++ radar clustering node publishes for generated clouds, as test vectors.

Purpose: keep the C++ node's answers once aav_object_detection is gone, and check the Rust
node against them.

    pixi run --frozen tools/ros.sh --local python3 \
        src/perception/aav_radar_clusters/test/capture_cpp_vectors.py [--node cpp|rust]

--node cpp (the default) needs aav_object_detection built. It plays each cloud into the C++
node (`dbscan`, as radar_clusters.launch.py ran it) and writes what it published to
crates/radar-clustering/tests/cpp_vectors.txt, which tests/cpp_vectors.rs reads.
--node rust plays the file's clouds into the Rust node (radar_clusters) instead, and
checks that its clusters and boxes are the C++ node's.

The C++ compared eps with the squared distance, so its neighbours were within sqrt(eps):
the Rust node gets that radius. Its own ROS domain (AAV_TEST_DOMAIN_ID, else 77), on this
computer only.
"""

import argparse
import math
import os
from pathlib import Path
import random
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

OUT = (Path(__file__).resolve().parents[1] / 'crates' / 'radar-clustering' / 'tests'
       / 'cpp_vectors.txt')
# The C++ node's eps and minimum_points, as car.launch.py and bench.launch.py gave them.
SETTINGS = [('the car', 0.8, 15), ('the bench', 0.8, 5)]
FIELDS = ('x', 'y', 'z', 'intensity', 'velocity')   # as the TI driver sends them
RANDOM_CLOUDS = 40                                  # per setting


def f32(v):
    """Return v rounded to a float32, as a Python float."""
    return struct.unpack('<f', struct.pack('<f', v))[0]


def cpp_threshold(eps):
    """The C++ node's test, squared distance <= eps, with eps through a float, as it did."""
    return f32(eps)


def distance_squared(a, b):
    """As the C++ node: float32 differences, squared and summed as doubles."""
    return sum(f32(p - q) ** 2 for p, q in zip(a[:3], b[:3]))


def near_the_edge(points, eps):
    """Whether a pair sits so close to the radius that rounding could decide it."""
    limit = cpp_threshold(eps)
    return any(abs(distance_squared(a, b) - limit) < 1e-9
               for i, a in enumerate(points) for b in points[i + 1:])


def point(x, y, z, rng):
    return (f32(x), f32(y), f32(z), f32(rng.uniform(5.0, 30.0)), f32(rng.uniform(-3.0, 3.0)))


def blob(rng, centre, spread, count):
    return [point(*(c + rng.gauss(0.0, spread) for c in centre), rng) for _ in range(count)]


def noise(rng, count):
    return [point(rng.uniform(0.5, 9.0), rng.uniform(-5.0, 5.0), rng.uniform(-0.5, 1.5), rng)
            for _ in range(count)]


def line(rng, start, step, count):
    return [point(start[0], start[1] + step * i, start[2], rng) for i in range(count)]


def clouds(rng, eps, min_points):
    """Return [(what it shows, points)]: shapes first, then random scenes."""
    radius = math.sqrt(cpp_threshold(eps))
    gap = 0.95 * radius             # a border point this far from each cluster's edge
    shapes = [
        ('two objects and some noise',
         blob(rng, (3.0, 1.0, 0.3), 0.2, 2 * min_points)
         + blob(rng, (6.0, -2.0, 0.5), 0.3, 3 * min_points) + noise(rng, 10)),
        ('a wall: a line of points 0.3 m apart', line(rng, (5.0, -4.0, 0.2), 0.3, 30)),
        ('two objects, and a border point that both reach',
         line(rng, (4.0, -2.0, 0.0), 0.05, min_points + 2)
         + [point(4.0, -2.0 + 0.05 * (min_points + 1) + gap, 0.0, rng)]
         + line(rng, (4.0, -2.0 + 0.05 * (min_points + 1) + 2 * gap, 0.0), 0.05,
                min_points + 2)),
        ('the same point many times, and a few others',
         [point(2.0, 0.5, 0.1, rng)] * min_points + noise(rng, 5)),
        ('noise only: points far apart',
         [point(1.0 + 1.5 * (i % 6), -4.0 + 1.5 * (i // 6), 0.0, rng) for i in range(24)]),
        ('one object of exactly minimum_points points',
         blob(rng, (3.0, 0.0, 0.0), 0.05, min_points)),
        ('one point fewer than that', blob(rng, (3.0, 0.0, 0.0), 0.05, min_points - 1)),
    ]
    scenes = []
    while len(scenes) < RANDOM_CLOUDS:
        points = noise(rng, rng.randint(0, 30))
        for _ in range(rng.randint(0, 4)):
            centre = (rng.uniform(1.0, 8.5), rng.uniform(-4.0, 4.0), rng.uniform(-0.3, 1.2))
            points += blob(rng, centre, rng.uniform(0.1, 0.6),
                           rng.randint(max(1, min_points // 2), 4 * min_points))
        rng.shuffle(points)
        if points and not near_the_edge(points, eps):
            scenes.append(('random', points))
    out = shapes + scenes
    for what, points in out:
        if near_the_edge(points, eps):
            raise RuntimeError(f'{what}: a pair sits right at the radius')
    return out


def cloud_message(points, frame_id, stamp):
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec, msg.header.stamp.nanosec = stamp, 0
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(FIELDS)]
    msg.point_step, msg.row_step = 20, 20 * len(points)
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<5f', *p) for p in points)
    return msg


def read_cloud(msg):
    """Return each point's fields, by name, from a published cloud."""
    fields = {f.name: f for f in msg.fields}
    out = []
    for i in range(msg.width * msg.height):
        base, values = i * msg.point_step, {}
        for name, f in fields.items():
            kind = '<f' if f.datatype == PointField.FLOAT32 else '<I'
            values[name] = struct.unpack_from(kind, bytes(msg.data), base + f.offset)[0]
        out.append(values)
    return out


class Collector(Node):
    """Plays clouds into /radar/points, and keeps what comes back."""

    def __init__(self):
        super().__init__('radar_compare')
        self.lock = threading.Lock()
        self.clouds, self.boxes = [], []
        self.pub = self.create_publisher(PointCloud2, '/radar/points', 10)
        # The C++ published its clusters best effort, the Rust node reliable: this hears both.
        best_effort = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/radar/clusters', self.on_cloud, best_effort)
        self.create_subscription(MarkerArray, '/radar/cluster_boxes', self.on_boxes, 100)

    def on_cloud(self, msg):
        with self.lock:
            self.clouds.append(msg)

    def on_boxes(self, msg):
        with self.lock:
            self.boxes.append(msg)

    def wait_for_node(self, timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if (self.pub.get_subscription_count() and self.count_publishers('/radar/clusters')
                    and self.count_publishers('/radar/cluster_boxes')):
                time.sleep(1.0)   # let both sides finish matching
                return
            time.sleep(0.1)
        raise RuntimeError('the node never showed up on /radar/points')

    def play(self, points, tag, timeout=5.0):
        """Publish one cloud; return the cloud and the boxes the node published for it."""
        for _ in range(2):   # once more if nothing came: a new node may still be matching
            with self.lock:
                self.clouds.clear()
                self.boxes.clear()
            self.pub.publish(cloud_message(points, tag, len(points)))
            end = time.monotonic() + timeout
            while time.monotonic() < end and not self.answered(tag):
                time.sleep(0.01)
            if self.answered(tag):
                break
        time.sleep(0.1)   # anything else that comes would be a second answer
        with self.lock:
            ours = [c for c in self.clouds if c.header.frame_id == tag]
            if len(ours) != 1 or len(self.boxes) != 1:
                raise RuntimeError(f'{tag}: {len(ours)} clouds and {len(self.boxes)} box '
                                   'messages came back, not one of each')
            return ours[0], self.boxes[0]

    def answered(self, tag):
        with self.lock:
            return bool(self.boxes) and any(c.header.frame_id == tag for c in self.clouds)


def start(node, eps, min_points):
    if node == 'cpp':
        exe = os.path.join(get_package_prefix('aav_object_detection'), 'lib',
                           'aav_object_detection', 'dbscan')
        args = ['-p', f'eps:={eps}', '-p', f'minimum_points:={min_points}',
                '-p', 'bounding_boxes:=true', '-p', 'target_frame:=radar_link',
                '-p', 'name_box:=radar_clusters',
                '-r', '/ti_mmwave/radar_scan_pcl:=/radar/points',
                '-r', 'dbscan_output:=/radar/clusters', '-r', 'dbscan_box:=/radar/cluster_boxes']
    else:
        exe = os.path.join(get_package_prefix('aav_radar_clusters'), 'lib',
                           'aav_radar_clusters', 'radar_clusters')
        radius = math.sqrt(cpp_threshold(eps))
        args = ['-p', f'eps:={radius!r}', '-p', f'minimum_points:={min_points}']
    # Its log goes to a file, not a pipe: a full pipe would stop the node.
    log = tempfile.TemporaryFile(mode='w+')
    proc = subprocess.Popen([exe, '--ros-args', '-r', '__node:=radar_clusters', *args],
                            stdout=log, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
    proc.log = log
    return proc


def stop(proc, timeout=5.0):
    """Ctrl-C the node's process group; return its exit code, or None if it hung."""
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        proc.send_signal(signal.SIGINT)
    try:
        code = proc.wait(timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
        code = None
    if code not in (0, None):
        proc.log.seek(0)
        print(proc.log.read()[-2000:])
    proc.log.close()
    return code


def members_of(points, coordinates):
    """Map published coordinates back to indices of the input points, in input order."""
    where = {}
    for i, p in enumerate(points):
        where.setdefault(p[:3], []).append(i)
    return sorted(where[c].pop(0) for c in coordinates)


def cpp_answer(points, cloud, boxes):
    """The C++ node's clusters (from its boxes' points), boxes, and zero padding."""
    clusters, cubes = [], []
    for m in boxes.markers:
        if (m.type, m.action, m.ns, m.header.frame_id) != (Marker.CUBE, Marker.ADD,
                                                          'radar_clusters', 'radar_link'):
            raise RuntimeError(f'an unexpected marker: {m}')
        coordinates = [(p.x, p.y, p.z) for p in m.points]
        clusters.append((m.id, members_of(points, coordinates)))
        cubes.append((m.id, (m.pose.position.x, m.pose.position.y, m.pose.position.z),
                      (m.scale.x, m.scale.y, m.scale.z)))
    published = read_cloud(cloud)
    zeros = sum(1 for p in published if (p['x'], p['y'], p['z'], p['rgb']) == (0, 0, 0, 0))
    clustered = sorted((p['x'], p['y'], p['z']) for p in published
                       if (p['x'], p['y'], p['z'], p['rgb']) != (0, 0, 0, 0))
    in_boxes = sorted(points[i][:3] for _, ids in clusters for i in ids)
    if clustered != in_boxes:
        raise RuntimeError("the C++ cloud's points aren't its boxes' points")
    if zeros + len(clustered) != len(points):
        raise RuntimeError('the C++ cloud is not one point per input point')
    return clusters, cubes, zeros


def rust_answer(points, cloud, boxes, tag):
    """The Rust node's clusters (from the cloud's cluster field) and boxes, checked."""
    first, *cubes = boxes.markers
    if (first.action, first.ns) != (Marker.DELETEALL, 'radar_clusters'):
        raise RuntimeError(f'{tag}: the boxes do not start with a DELETEALL')
    for msg in (cloud, *boxes.markers):
        if msg.header.frame_id != tag or msg.header.stamp.sec != len(points):
            raise RuntimeError(f"{tag}: an output without the input's header")
    by_id = {}
    for p in read_cloud(cloud):
        by_id.setdefault(p['cluster'], []).append((p['x'], p['y'], p['z']))
    clusters = [(i, members_of(points, by_id[i])) for i in sorted(by_id)]
    return clusters, [(m.id, (m.pose.position.x, m.pose.position.y, m.pose.position.z),
                       (m.scale.x, m.scale.y, m.scale.z)) for m in cubes]


def numbers(values):
    # repr() is the shortest text that reads back to the same float, bit for bit.
    return ' '.join(repr(float(v)) for v in values)


def capture(collector, rng):
    lines = [
        '# What the 2025-26 C++ radar clustering node published for these clouds: its',
        "# clusters (its boxes' points) and its boxes. Made by test/capture_cpp_vectors.py on",
        f'# {time.strftime("%Y-%m-%d")}, with the node as it was in aav just before it was',
        '# removed (aav_object_detection, dbscan). Read by cpp_vectors.rs.',
        '#',
        '# settings: the C++ eps and minimum_points. It compared eps with the squared',
        '#   distance, so its neighbours were within sqrt(eps) m.',
        '# point x y z intensity velocity: the cloud, in order (float32, in radar_link).',
        '# cluster id i...: the points in the C++ cluster with that id (its box id).',
        '# box id cx cy cz sx sy sz: its box, centre and size (float64, bit for bit).',
        '# padding n: the C++ cloud had n more points, at (0, 0, 0): one per noise point.',
    ]
    count = 0
    for name, eps, min_points in SETTINGS:
        proc = start('cpp', eps, min_points)
        try:
            collector.wait_for_node()
            lines += ['', f'settings {name!r} eps {eps!r} minimum_points {min_points}']
            for k, (what, points) in enumerate(clouds(rng, eps, min_points)):
                cloud, boxes = collector.play(points, f'cloud_{count}')
                clusters, cubes, zeros = cpp_answer(points, cloud, boxes)
                lines += ['', f'cloud {what}']
                lines += [f'point {numbers(p)}' for p in points]
                lines += [f'cluster {i} ' + ' '.join(map(str, ids)) for i, ids in clusters]
                lines += [f'box {i} {numbers(c)} {numbers(s)}' for i, c, s in cubes]
                lines.append(f'padding {zeros}')
                count += 1
        finally:
            code = stop(proc)
            print(f'C++ node ({name}) stopped: exit code {code}')
    OUT.write_text('\n'.join(lines) + '\n')
    print(f'{count} clouds: {OUT}')
    return 0


def parse_vectors():
    """Return [(name, eps, min_points, [(what, points, clusters, cubes)])] from the file."""
    groups = []
    for line in OUT.read_text().splitlines():
        word, _, rest = line.partition(' ')
        if word == 'settings':
            parts = rest.rsplit(' ', 4)
            groups.append((parts[0].strip("'"), float(parts[2]), int(parts[4]), []))
        elif word == 'cloud':
            groups[-1][3].append((rest, [], [], []))
        elif word == 'point':
            groups[-1][3][-1][1].append(tuple(float(v) for v in rest.split()))
        elif word == 'cluster':
            i, *ids = rest.split()
            groups[-1][3][-1][2].append((int(i), [int(v) for v in ids]))
        elif word == 'box':
            i, *v = rest.split()
            groups[-1][3][-1][3].append((int(i), tuple(map(float, v[:3])),
                                         tuple(map(float, v[3:]))))
    return groups


def compare(collector):
    rows, ok, count = [], True, 0
    for name, eps, min_points, cases in parse_vectors():
        proc = start('rust', eps, min_points)
        same = clusters_seen = 0
        try:
            collector.wait_for_node()
            for what, points, want_clusters, want_cubes in cases:
                tag = f'cloud_{count}'
                count += 1
                cloud, boxes = collector.play(points, tag)
                got_clusters, got_cubes = rust_answer(points, cloud, boxes, tag)
                clusters_seen += len(want_clusters)
                if got_clusters == want_clusters and got_cubes == want_cubes:
                    same += 1
                else:
                    print(f'DIFFERENT ({name}, {what}):\n  C++  {want_clusters}\n'
                          f'  Rust {got_clusters}')
            cloud, boxes = collector.play([], 'empty')
            empty_ok = cloud.width == 0 and len(boxes.markers) == 1
        finally:
            code = stop(proc)
        rows.append((name, eps, min_points, len(cases), clusters_seen, same, empty_ok, code))
        ok = ok and same == len(cases) and empty_ok and code == 0
    print(f"\n{'Settings':<34} {'Clouds':>6} {'Clusters':>8} {'Same as C++':>11}  "
          'Empty cloud clears  Ctrl-C')
    for name, eps, n, clouds_, clusters, same, empty_ok, code in rows:
        label = f'{name}: eps {eps} (C++), minimum {n}'
        print(f'{label:<34} {clouds_:>6} {clusters:>8} {same:>11}  '
              f'{"yes" if empty_ok else "NO":<18}  exit code {code}')
    print('\nRust node: the C++ answers' if ok else '\nRust node: DIFFERENT')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--node', choices=['cpp', 'rust'], default='cpp')
    args = ap.parse_args()
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('AAV_TEST_DOMAIN_ID', '77')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    rclpy.init()
    collector = Collector()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    try:
        if args.node == 'cpp':
            return capture(collector, random.Random(2026))
        return compare(collector)
    except RuntimeError as err:
        print(f'Could not finish: {err}')
        return 1
    finally:
        executor.shutdown()
        spinner.join()
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
