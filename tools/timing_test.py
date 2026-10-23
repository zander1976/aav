#!/usr/bin/env python3
"""
Timing test: whether every message arrives on time, for minutes, parked and driving.

Starts Gazebo's test lot headless with the car's whole graph, then measures every topic's
arrivals, first with the car parked, then while Nav2 drives controller_compare.py's lot
tour round and round. Made to compare the messaging (TODO 11 and 32, docs/mac.md):

    pixi run timing-test                          (Cyclone DDS: 5 min parked, 5 driving)
    AAV_RMW=fastdds pixi run timing-test          (the same on Fast DDS)
    pixi run timing-test --parked 60 --driving 60     (a quick look)

For each phase it prints:
- the key topics (the IMU, wheels, GPS, both EKFs, /tf, /clock, the cameras, the lidar,
  the radar, Nav2's command, ego's, the sensor watchdog): worst gap, and how many gaps
  were more than 0.1 s and 0.5 s late (late = the gap minus the topic's usual gap);
- stalls: three or more key topics quiet at the same time;
- the sensor watchdog's STALE warnings and locks, and "too old" TF warnings (/rosout);
- how fast simulated time ran (/clock), the run's CPU (its process group) and the Mac's
  idle share (top), and the load average.

`(probe)` in a stall means this program itself paused: then the stall says nothing about
the messaging. The numbers: tools/stalls.py (no ROS; tests in tools/tests/test_stalls.py).

The report, every topic's table and each late gap go to log/timing-test/<time>-<rmw>/; the
launch log beside them. A ROS domain and Gazebo partition of its own (AAV_SIM_DOMAIN picks
the domain), on this computer only; stopped by its process group; Fast DDS's shared memory
is cleared after a Fast DDS run.
"""

import argparse
import collections
import datetime
import json
import math
import os
import platform
import random
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
DOMAIN = os.environ.get('AAV_SIM_DOMAIN', str(random.randint(150, 230)))
os.environ['ROS_DOMAIN_ID'] = DOMAIN
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_STATIC_PEERS', None)
os.environ['GZ_IP'] = '127.0.0.1'
os.environ['GZ_PARTITION'] = f'timing_test_{DOMAIN}'

from action_msgs.msg import GoalStatus  # noqa: E402
import bandwidth  # noqa: E402
from controller_compare import alive, stop, TOURS  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
import stalls  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
import topic_load  # noqa: E402

# (topic, what it is): the ones the stack can't drive without, and the big ones.
KEY = [
    ('/clock', "Gazebo's clock"),
    ('/sensors/imu/data', 'IMU'),
    ('/vehicle/odom', 'wheel odometry'),
    ('/sensors/gps/fix', 'GPS'),
    ('/localization/odom/local', 'local EKF'),
    ('/localization/odom/global', 'global EKF'),
    ('/tf', '/tf'),
    ('/sensors/camera/front_left/image_raw', 'front left camera'),
    ('/sensors/camera/front_right/image_raw', 'front right camera'),
    ('/sensors/camera/rear/image_raw', 'rear camera'),
    ('/sensors/lidar/scan', 'lidar scan'),
    ('/sensors/radar/points', 'radar'),
    ('/drivers/nav/cmd_vel', "Nav2's command"),
    ('/ego/actuator_cmd', "ego's command"),
    ('/safety/sensor_watchdog/state', 'sensor watchdog'),
]
# Sent only while Nav2 drives a goal: a gap across a goal change isn't late.
WHILE_DRIVING = {'/drivers/nav/cmd_vel'}
PROBE = '(probe)'
STALL_TOPICS = 3        # key topics quiet together: a stall
UP_TIME = 150.0         # s for Gazebo and the EKF, then again for Nav2
READY_TIME = 30.0       # s for the sensor watchdog to let Nav2 drive
CPU_EVERY = 10.0        # s between samples of the computer's load


class Arrivals(bandwidth.Meters):
    """bandwidth.py's meters, which also keep every message's arrival time."""

    def __init__(self, node):
        super().__init__(node, [], [])
        self.arrivals = {}

    def counter(self, topic):
        count = super().counter(topic)

        def record(data):
            count(data)
            self.arrivals.setdefault(topic, []).append(time.monotonic())
        return record

    def restart(self):
        super().restart()
        self.arrivals = {topic: [] for topic in self.meters}


class Load(threading.Thread):
    """Sample the computer's idle share (top) and load average every CPU_EVERY s."""

    def __init__(self):
        super().__init__(daemon=True)
        self.samples = []       # (monotonic time, idle %, 1-minute load)
        self.running = True

    def run(self):
        command = (['top', '-l', '2', '-n', '0', '-s', '1'] if platform.system() == 'Darwin'
                   else ['top', '-b', '-n', '2', '-d', '1'])
        while self.running:
            out = subprocess.run(command, capture_output=True, text=True).stdout
            self.samples.append((time.monotonic(), stalls.top_idle(out), os.getloadavg()[0]))
            time.sleep(CPU_EVERY)

    def between(self, start, end):
        """Return (mean idle %, lowest idle %, mean load) of the samples in [start, end]."""
        inside = [s for s in self.samples if start <= s[0] <= end]
        idle = [s[1] for s in inside if s[1] is not None]
        return (stalls.mean(idle), min(idle) if idle else math.nan,
                stalls.mean([s[2] for s in inside]))


def group_cpu(pgid):
    """Return the CPU seconds used so far by the run's process group."""
    out = subprocess.run(['ps', '-A', '-o', 'pgid=,time='], capture_output=True,
                         text=True).stdout
    return stalls.group_cpu(out, pgid)


class Recorder(Node):
    """Every topic's arrivals, /rosout's warnings, the sensor watchdog, and Nav2."""

    def __init__(self):
        super().__init__('timing_test', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.meters = Arrivals(self)
        self.loop = []                  # this program's own spin loop, its times
        self.logs = []                  # (time, node, level, text): warnings and up
        self.lock = None
        self.locks = 0                  # times the watchdog's lock went on
        self.truth = self.glob = None
        self.recording = False
        self.create_subscription(Bool, '/safety/sensor_watchdog/lock', self.on_lock, 10)
        self.create_subscription(Odometry, '/vehicle/gazebo/truth', self.on_truth, 10)
        self.create_subscription(Odometry, '/localization/odom/global', self.on_global, 10)
        self.create_subscription(Log, '/rosout', self.on_log, QoSProfile(
            depth=1000, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE))
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def on_lock(self, msg):
        if self.recording and msg.data and self.lock is False:
            self.locks += 1
        self.lock = msg.data

    def on_truth(self, msg):
        self.truth = msg

    def on_global(self, msg):
        self.glob = msg

    def on_log(self, msg):
        if self.recording and msg.level >= Log.WARN:
            self.logs.append((time.monotonic(), msg.name, msg.level, msg.msg))

    def start(self):
        """Forget what was counted: a phase starts now."""
        self.meters.restart()
        self.loop, self.logs, self.locks = [], [], 0
        self.recording = True

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        next_look = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if self.recording:
                self.loop.append(now)
            if now >= next_look:          # topics that appear later
                self.meters.discover()
                next_look = now + 2.0
            if until is not None and until():
                return True
        return until is None

    def wait(self, future, seconds):
        self.spin_for(seconds, future.done)
        return future.result() if future.done() else None


def send_goal(node, x, y, heading):
    """Send Nav2 a goal in the map frame; return its handle, or None if refused."""
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    yaw = math.radians(heading)
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    handle = node.wait(node.nav.send_goal_async(goal), 10.0)
    return handle if handle is not None and handle.accepted else None


def drive(node, seconds):
    """Drive the lot tour round and round for `seconds` of wall time; return goal changes."""
    legs = TOURS['lot']['legs']
    changes, notes = [], []
    end = time.monotonic() + seconds
    leg = 0
    while time.monotonic() < end:
        name, x, y, heading = legs[leg % len(legs)]
        changes.append(time.monotonic())
        handle = send_goal(node, x, y, heading)
        if handle is None:
            notes.append(f'Nav2 refused "{name}"')
            node.spin_for(2.0)
            leg += 1
            continue
        result = handle.get_result_async()
        node.spin_for(end - time.monotonic(), result.done)
        if result.done():
            status = result.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                notes.append(f'"{name}": Nav2 gave up (status {status})')
        else:
            node.wait(handle.cancel_goal_async(), 5.0)
        leg += 1
    changes.append(time.monotonic())
    return changes, notes, leg


def phase(node, name, seconds, pgid, load, driving=False):
    """Measure one phase; return its results as a dict."""
    print(f'== {name}: {seconds:g} s...', flush=True)
    node.meters.discover()
    node.start()
    cpu0, start = group_cpu(pgid), time.monotonic()
    changes, notes, legs = [], [], 0
    if driving:
        changes, notes, legs = drive(node, seconds)
    else:
        node.spin_for(seconds)
    end = time.monotonic()
    node.recording = False
    cpu = (group_cpu(pgid) - cpu0) / (end - start)
    window = end - start
    arrivals = dict(node.meters.arrivals)
    rows = stalls.summary({t: arrivals.get(t, []) for t, _ in KEY}, window)
    gaps = []
    for topic, _ in KEY:
        found = stalls.late_gaps(topic, arrivals.get(topic, []))
        if topic in WHILE_DRIVING:
            found = [g for g in found if not any(g.start <= c <= g.end for c in changes)]
            row = next(r for r in rows if r.topic == topic)
            row.late = len(found)
            row.very_late = sum(1 for g in found if g.late > stalls.VERY_LATE)
        gaps += found
    probe_gaps = stalls.late_gaps(PROBE, node.loop)
    found = stalls.stalls(gaps + probe_gaps, least=STALL_TOPICS)
    stale = [entry for entry in node.logs if 'sensor_watchdog' in entry[1] and 'STALE' in entry[3]]
    too_old = [entry for entry in node.logs if 'too old' in entry[3].lower()
               or 'extrapolat' in entry[3].lower()]
    idle, lowest_idle, mean_load = load.between(start, end)
    every = topic_load.summary(node.meters.meters, window, node.meters.types)
    return {
        'name': name, 'seconds': window, 'rtf': node.meters.real_time_factor(),
        'cpu_cores': cpu, 'idle': idle, 'lowest_idle': lowest_idle, 'load': mean_load,
        'rows': rows, 'gaps': gaps, 'probe_gaps': probe_gaps, 'stalls': found,
        'start': start, 'stale': stale, 'too_old': too_old, 'locks': node.locks,
        'warnings': len(node.logs), 'logs': list(node.logs), 'legs': legs, 'notes': notes,
        'every': topic_load.table(every, markdown=True, quiet=False),
    }


def number(x, digits=2):
    return '' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.{digits}f}'


def report(result, rmw):
    """Return one phase's Markdown."""
    names = dict(KEY)
    lines = [f'## {result["name"]} ({rmw}, {result["seconds"]:.0f} s)', '',
             '| topic | what | Hz | worst gap s | worst late s | late > 0.1 s | late > 0.5 s |',
             '|---|---|---:|---:|---:|---:|---:|']
    for r in result['rows']:
        lines.append(f'| `{r.topic}` | {names[r.topic]} | {number(r.hz, 1)} | '
                     f'{number(r.worst_gap)} | {number(r.worst_late)} | {r.late} | '
                     f'{r.very_late} |')
    t0 = result['start']
    lines += ['', f'Stalls ({STALL_TOPICS} or more key topics quiet together): '
              f'{len(result["stalls"])}']
    for s in result['stalls']:
        lines.append(f'- at {s.start - t0:.1f} s, {s.length:.2f} s: {", ".join(s.topics)}')
    probe = result['probe_gaps']
    lines += [
        '',
        f'- simulated time: {number(result["rtf"])} x real time',
        f'- the run\'s CPU: {number(result["cpu_cores"], 1)} cores; the computer '
        f'{number(result["idle"], 0)}% idle on average '
        f'(lowest {number(result["lowest_idle"], 0)}%), '
        f'load {number(result["load"], 1)}',
        f'- the sensor watchdog: {len(result["stale"])} STALE warnings, its lock went on '
        f'{result["locks"]} times',
        f'- TF "too old" or extrapolation warnings: {len(result["too_old"])}; all warnings '
        f'and errors on /rosout: {result["warnings"]}',
        f'- this program paused more than 0.1 s: {len(probe)} times'
        + (f' (worst {max(g.length for g in probe):.2f} s)' if probe else ''),
    ]
    if result['legs']:
        lines.append(f'- Nav2 goals sent: {result["legs"]}' +
                     ''.join(f'; {n}' for n in result['notes']))
    for when, node, _, text in result['stale'][:5] + result['too_old'][:5]:
        lines.append(f'  - {when - t0:.1f} s, {node}: {text[:140]}')
    said = collections.Counter((node, text[:100]) for _, node, _, text in result['logs'])
    if said:
        lines += ['', 'Warnings and errors on /rosout, the most frequent:']
        lines += [f'- {n} x {node}: {text}' for (node, text), n in said.most_common(8)]
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--parked', type=float, default=300.0, help='s parked (300)')
    parser.add_argument('--driving', type=float, default=300.0, help='s driving (300)')
    parser.add_argument('--settle', type=float, default=30.0,
                        help='s after Nav2 is up, before measuring (30)')
    args = parser.parse_args()
    rmw = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    started = datetime.datetime.now()
    out_dir = os.path.join(ROOT, 'log', 'timing-test', f'{started:%Y%m%d-%H%M%S}-{rmw}')
    os.makedirs(out_dir, exist_ok=True)
    log_path = os.path.join(out_dir, 'sim.log')
    print(f'{rmw}: the test lot on ROS domain {DOMAIN}, Gazebo partition '
          f'{os.environ["GZ_PARTITION"]} (this computer only); load average '
          f'{os.getloadavg()[0]:.1f}; results in {out_dir}', flush=True)
    load = Load()
    load.start()
    results = []
    with open(log_path, 'w') as log:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'aav_bringup', 'sim.launch.py', 'world:=test_lot', 'gui:=false',
             'rviz:=false', 'reversing:=false'],
            cwd=ROOT, env=dict(os.environ), stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        rclpy.init()
        node = Recorder()
        try:
            up = node.spin_for(UP_TIME, lambda: node.truth is not None and node.glob)
            if not (up and node.spin_for(UP_TIME, node.nav.server_is_ready)):
                print(f'It never came up: see {log_path}', file=sys.stderr)
                return 1
            node.spin_for(args.settle)
            if args.parked > 0:
                results.append(phase(node, 'parked', args.parked, launch.pid, load))
            if args.driving > 0:
                if not node.spin_for(READY_TIME, lambda: node.lock is False):
                    print('The sensor watchdog still holds Nav2: driving anyway', flush=True)
                results.append(phase(node, 'driving', args.driving, launch.pid, load,
                                     driving=True))
        finally:
            load.running = False
            node.destroy_node()
            rclpy.try_shutdown()
            if not stop(launch):
                print(f'Some of process group {launch.pid} would not stop', flush=True)
            launch.wait()
            if 'fastrtps' in rmw:
                # Programs that die leave Fast DDS's shared-memory files behind (21 Oct).
                subprocess.run(['fastdds', 'shm', 'clean'], capture_output=True)
    text = '\n\n'.join(report(r, rmw) for r in results)
    print('\n' + text, flush=True)
    with open(os.path.join(out_dir, 'report.md'), 'w') as f:
        f.write(f'# Timing test, {started:%Y-%m-%d %H:%M}, {rmw}\n\n{text}\n\n'
                + '\n\n'.join(f'## Every topic, {r["name"]}\n\n{r["every"]}' for r in results)
                + '\n')
    with open(os.path.join(out_dir, 'gaps.json'), 'w') as f:
        json.dump({r['name']: [{'topic': g.topic, 'at': round(g.start - r['start'], 3),
                                'gap': round(g.length, 3), 'late': round(g.late, 3)}
                               for g in r['gaps'] + r['probe_gaps']] for r in results},
                  f, indent=1)
    print(f'\nWritten: {out_dir}/report.md and gaps.json', flush=True)
    return 0 if not alive(launch.pid) else 1


if __name__ == '__main__':
    sys.exit(main())
