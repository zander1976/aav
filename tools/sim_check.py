#!/usr/bin/env python3
"""
Sim check: start Gazebo's road world headless, then test the car's whole graph in it.

    pixi run sim-check                     (about 2 minutes, on this computer only)
    pixi run -e ml sim-check --perception  (adds YOLO and the stop sign: 2 more minutes)
    pixi run sim-check --obstacle          (adds a box for the radar stand-in: 2 more minutes)
    pixi run sim-check --lane              (adds lane keeping, in a second run: 2 more minutes)

bench_check.py tests the car's software against pretend sensors that draw a flat world;
this tests it against Gazebo's camera and radar stand-in, which see the road world
(aav_gazebo worlds/road.sdf: the bench road and its stop sign, in 3D).

1. Starts sim.launch.py world:=road gui:=false rviz:=false on a private ROS domain and
   Gazebo partition, on this computer only.
2. Start-up: Gazebo's truth, the global EKF, the camera and the radar stand-in all run,
   and Nav2 is up (unless nav:=false).
3. Checks the running graph against docs/interfaces.yaml (check_graph.py --profile road).
4. The sensors: the camera's picture and lens are the bench camera's, in camera_left_link;
   the radar stand-in's cloud has the TI driver's fields, in radar_link, and nothing in it
   while nothing is within 8.4 m.
5. Drives a scripted profile, and compares where the car really is (/gazebo/truth) with
   where the car's own software thinks it is (/odometry/global).
6. With --perception: Nav2 drives to 40 m east, past the stop sign at 25 m. The car must
   stop with its front bumper before the sign, wait, and go on to the goal. The camera's
   picture with YOLO's boxes, as the lock went on, is saved to /tmp/aav-sim-stop-sign.png.
7. With --obstacle: the bench's 1 m box appears in the lane 15 m ahead (Gazebo's create
   service), and Nav2 drives 30 m. collision_monitor's stop zone must stop the car before
   the box; once the box is taken away, the car must go on to the goal.
8. With --lane: Gazebo again, with lane keeping (lane:=true) and nothing else driving, the
   car 0.3 m left of the road's centre. lane_finder and control_interface must bring it
   back within 5 cm, and keep it there.

Times are Gazebo's, simulated; only the limits on waiting are wall-clock. Prints a table and
exits 1 if anything failed. The launch logs go to /tmp/aav-sim-check.log and, with --lane,
/tmp/aav-sim-check-lane.log: AAV_SIM_LOG=/tmp/mine.log moves them. AAV_SIM_DOMAIN and
AAV_SIM_PARTITION pick the ROS domain and the Gazebo partition (a random domain otherwise).
"""

import bisect
import importlib.util
import math
import os
import random
import signal
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# A private ROS domain and Gazebo partition, on this computer only, set before rclpy
# starts: the check can't disturb a car, another laptop or someone else's simulator.
DOMAIN = os.environ.get('AAV_SIM_DOMAIN', str(random.randint(150, 230)))
os.environ['ROS_DOMAIN_ID'] = DOMAIN
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['GZ_IP'] = '127.0.0.1'
os.environ['GZ_PARTITION'] = os.environ.get('AAV_SIM_PARTITION', f'sim_check_{DOMAIN}')

from aav_description import vehicle  # noqa: E402
from aav_interfaces.action import ScriptedDrive  # noqa: E402
from aav_lane_geometry.msg import LaneGeometry  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav2_msgs.msg import CollisionMonitorState  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, PointCloud2  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from vision_msgs.msg import Detection2DArray  # noqa: E402
import yaml  # noqa: E402

RESULTS = []

# The road world is the bench world (test_worlds.py checks it): its road, sign and camera.
with open(os.path.join(ROOT, 'src', 'bench', 'aav_bench', 'config', 'bench.yaml')) as f:
    BENCH = yaml.safe_load(f)['mock_camera']['ros__parameters']
ROAD_Y = BENCH['road_y']                # m: the lane's centre line
SIGN_X = BENCH['stop_signs'][0]         # m east: the stop sign
OBSTACLE = os.path.join(ROOT, 'src', 'backends', 'aav_gazebo', 'models', 'obstacle',
                        'model.sdf')
RADAR_FIELDS = ['x', 'y', 'z', 'intensity', 'velocity']   # the TI driver's

# Lane keeping (--lane), as bench_check.py: where the car starts, what counts as back on the
# centre line, and how long it has to get there.
LANE_START = 0.3        # m left of the road's centre
LANE_BAND = 0.05        # m either side of the centre
LANE_TIME = 30.0        # s of driving, from when the car starts moving
LOOKAHEAD = 3.0         # m: where lane_finder measures the offset (its config, lookahead)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def result(name, ok, detail):
    RESULTS.append((name, ok, detail))
    print(f'{"PASS" if ok else "FAIL"}  {name}: {detail}', flush=True)


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rate(stamps):
    """Return messages per second of simulated time, from their stamps (s)."""
    span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    return (len(stamps) - 1) / span if span > 0 else 0.0


def gz_service(service, reqtype, request):
    """Call one of the world's Gazebo services; True if it says it worked."""
    try:
        done = subprocess.run(
            ['gz', 'service', '-s', f'/world/aav/{service}', '--reqtype', reqtype,
             '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request],
            capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return False
    return done.returncode == 0 and 'data: true' in done.stdout


class Probe(Node):

    def __init__(self):
        # On Gazebo's clock, like every node in the sim.
        super().__init__('sim_check', parameter_overrides=[Parameter('use_sim_time',
                                                                     value=True)])
        self.truth = self.glob = self.image = self.info = self.cloud = None
        # Every truth sample, lane message and sensor stamp, from the start: lane keeping
        # starts as soon as the camera sees the lane, before its check begins.
        self.track, self.lanes, self.image_stamps, self.cloud_stamps = [], [], [], []
        self.create_subscription(Odometry, '/gazebo/truth', self.on_truth, 10)
        self.create_subscription(Odometry, '/odometry/global',
                                 lambda m: setattr(self, 'glob', m), 10)
        self.create_subscription(LaneGeometry, '/lane_geometry', self.on_lane, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 2)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 lambda m: setattr(self, 'info', m), 2)
        self.create_subscription(PointCloud2, '/radar/points', self.on_cloud,
                                 qos_profile_sensor_data)
        self.drive = ActionClient(self, ScriptedDrive, '/scripted_drive')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.stop, self.stop_picture, self.picture, self.stop_sign = False, None, None, None
        self.create_subscription(Bool, '/perception/stop', self.on_stop, 10)
        self.create_subscription(Image, '/perception/detections/image',
                                 lambda m: setattr(self, 'picture', m), 2)
        # The stop signs YOLO saw: (where the car was, x m; score; box height, px).
        self.signs = []
        self.create_subscription(Detection2DArray, '/perception/detections', self.on_boxes, 10)
        self.monitor = []    # (action, polygon) from collision_monitor, as it changes
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 lambda m: self.monitor.append((m.action_type, m.polygon_name)),
                                 10)

    def on_truth(self, msg):
        self.truth = msg
        self.track.append((seconds(msg.header.stamp), msg.pose.pose.position.x,
                           msg.pose.pose.position.y, yaw_of(msg), msg.twist.twist.linear.x))

    def on_lane(self, msg):
        self.lanes.append((seconds(msg.header.stamp), msg.vehicle_offset_m,
                           msg.both_lanes_detected))

    def on_image(self, msg):
        self.image = msg
        self.image_stamps = self.image_stamps[-49:] + [seconds(msg.header.stamp)]

    def on_cloud(self, msg):
        self.cloud = msg
        self.cloud_stamps = self.cloud_stamps[-49:] + [seconds(msg.header.stamp)]

    def on_boxes(self, msg):
        x = self.truth.pose.pose.position.x if self.truth is not None else math.nan
        self.signs += [(x, d.results[0].hypothesis.score, d.bbox.size_y)
                       for d in msg.detections
                       if d.results and d.results[0].hypothesis.class_id == 'stop sign']

    def on_stop(self, msg):
        if msg.data and not self.stop:
            self.stop_picture = self.picture   # what the camera saw as the lock went on
            self.stop_sign = self.signs[-1] if self.signs else None
        self.stop = msg.data

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until is None

    def xy(self, msg):
        return msg.pose.pose.position.x, msg.pose.pose.position.y

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def go_to(self, x, y, yaw):
        """Send Nav2 a goal in the map frame; return the result's future, or None if refused."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        if not self.nav.wait_for_server(timeout_sec=30.0):
            return None
        sent = self.nav.send_goal_async(goal)
        self.spin_for(10.0, lambda: sent.done())
        if not sent.done() or not sent.result().accepted:
            return None
        return sent.result().get_result_async()


def check_contract(probe, env, tag):
    """Check the running graph against docs/interfaces.yaml (check_graph.py --profile road)."""
    # Up to three tries, 10 s apart, as in bench_check.py: a node's first start after a
    # build can be slow on a Mac. The probe spins meanwhile, so its track has no gaps.
    for attempt in range(3):
        with tempfile.TemporaryFile('w+') as out:
            graph = subprocess.Popen(
                [sys.executable, os.path.join(ROOT, 'tools', 'check_graph.py'), '--profile',
                 'road'], cwd=ROOT, env=env, stdout=out, stderr=subprocess.DEVNULL, text=True)
            probe.spin_for(60.0, lambda: graph.poll() is not None)
            code = graph.wait()
            out.seek(0)
            fails = [line for line in out.read().splitlines() if line.startswith('FAIL')]
        if code == 0 and not fails:
            break
        probe.spin_for(10.0)
    result(f'graph vs contract{tag}', code == 0 and not fails,
           f'{len(fails)} failures' + (': ' + '; '.join(fails[:3]) if fails else ''))


def check_sensors(probe):
    """Check the camera is the bench camera, in camera_left_link, and the radar the TI's."""
    probe.spin_for(5.0)     # a few seconds of each, for their rates
    image, info, cloud = probe.image, probe.info, probe.cloud
    camera_ok = (image.width, image.height, image.header.frame_id) == (
        BENCH['width'], BENCH['height'], 'camera_left_link')
    lens_ok = (info.header.frame_id == 'camera_left_link'
               and abs(info.k[0] - BENCH['focal_px']) < 1.0
               and abs(info.k[4] - BENCH['focal_px']) < 1.0
               and abs(info.k[2] - BENCH['width'] / 2.0) < 1.0
               and abs(info.k[5] - BENCH['height'] / 2.0) < 1.0)
    fields = [f.name for f in cloud.fields]
    radar_ok = cloud.header.frame_id == 'radar_link' and fields == RADAR_FIELDS
    empty = cloud.width * cloud.height == 0     # the sign is 25 m away, the radar sees 8.4 m
    camera_hz, radar_hz = rate(probe.image_stamps), rate(probe.cloud_stamps)
    ok = (camera_ok and lens_ok and radar_ok and empty and abs(camera_hz - 10.0) < 1.5
          and abs(radar_hz - 10.0) < 1.5)
    result('sensors', ok,
           f'camera {image.width} x {image.height} {image.encoding} in {image.header.frame_id} '
           f"at {camera_hz:.1f} Hz, focal {info.k[0]:.1f} px (the bench's "
           f'{BENCH["focal_px"]:.0f}); radar stand-in in {cloud.header.frame_id} at '
           f'{radar_hz:.1f} Hz, fields {",".join(fields)}, '
           f'{cloud.width * cloud.height} points with nothing within 8.4 m')


def check_drive(probe):
    """Drive the scripted profile; compare the distance with the plan, and the EKF with truth."""
    x0, y0 = probe.xy(probe.truth)
    if not probe.drive.wait_for_server(timeout_sec=20.0):
        result('scripted drive', False, 'no scripted driver')
        return
    sent = probe.drive.send_goal_async(ScriptedDrive.Goal(
        target_speed=1.0, acceleration=0.5, deceleration=1.0, hold_time=5.0))
    probe.spin_for(10.0, lambda: sent.done())
    if not sent.done() or not sent.result().accepted:
        result('scripted drive', False, 'refused')
        return
    done = sent.result().get_result_async()
    probe.spin_for(60.0, lambda: done.done())
    start = probe.now()
    probe.spin_for(20.0, lambda: probe.now() - start > 3.0)    # 3 s to come to rest
    x1, y1 = probe.xy(probe.truth)
    travelled = math.hypot(x1 - x0, y1 - y0)
    # The plan: 2 s up to 1 m/s (1 m), 5 s at 1 m/s (5 m), 1 s down (0.5 m). The range is
    # bench_check.py's: it catches a stretched plan, or no motion at all.
    result('scripted drive', 4.5 < travelled < 8.0,
           f'{travelled:.2f} m (plan: 6.5 m), {y1 - y0:+.2f} m to the side')
    gx, gy = probe.xy(probe.glob)
    error = math.hypot(gx - x1, gy - y1)
    result('global EKF vs truth', error < 1.5, f'{error:.2f} m apart (GPS noise is 0.5 m)')


def check_stop_sign(probe):
    """Nav2 drives to 40 m east, past the stop sign at 25 m: it must stop before it."""
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']   # base_link to the front bumper
    done = probe.go_to(40.0, ROAD_Y, 0.0)
    if done is None:
        result('stop sign', False, 'no Nav2, or it refused the goal')
        return
    seen = len(probe.signs)
    probe.stop_sign = None
    locked_at, still_at, end = None, None, time.monotonic() + 240.0
    while time.monotonic() < end and not done.done():
        probe.spin_for(0.1)
        if probe.stop and locked_at is None:
            locked_at = probe.truth.pose.pose.position.x
        if probe.stop and still_at is None and abs(probe.truth.twist.twist.linear.x) < 0.05:
            still_at = probe.truth.pose.pose.position.x
    x = probe.truth.pose.pose.position.x
    reached = done.done() and done.result().status == 4
    ok = (locked_at is not None and locked_at < SIGN_X - 1.0 and still_at is not None
          and still_at + front < SIGN_X and reached and x > 35.0)
    signs = probe.signs[seen:]
    first = (f'YOLO first saw it from x = {signs[0][0]:.1f} m' if signs
             else 'YOLO never saw it')
    lock = 'never locked'
    if locked_at is not None:
        lock = f'locked at x = {locked_at:.1f} m'
        if probe.stop_sign is not None:
            lock += f' (score {probe.stop_sign[1]:.2f}, {probe.stop_sign[2]:.0f} px tall)'
    still = (f'stood still at x = {still_at:.1f} m, its bumper '
             f'{SIGN_X - still_at - front:.1f} m before the sign'
             if still_at is not None else 'never stood still')
    result('stop sign', ok, f'{first}; {lock}; {still} (the sign is at {SIGN_X:.0f} m); '
                            f'Nav2 goal {"reached" if reached else "not reached"} at {x:.1f} m')
    if probe.stop_picture is not None:
        try:
            from cv_bridge import CvBridge
            import cv2
            cv2.imwrite('/tmp/aav-sim-stop-sign.png',
                        CvBridge().imgmsg_to_cv2(probe.stop_picture, 'bgr8'))
            print('The camera as the lock went on: /tmp/aav-sim-stop-sign.png', flush=True)
        except Exception as e:  # noqa: B902 - the picture is a bonus
            print(f'could not save the picture: {e}', flush=True)


def check_obstacle(probe, ahead=15.0, goal=30.0):
    """Put the bench's box `ahead` m in the lane: the stop zone must stop the car before it."""
    name = 'obstacle'
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']   # base_link to the front bumper
    x0, y0 = probe.xy(probe.truth)
    h = yaw_of(probe.truth)
    c, s = math.cos(h), math.sin(h)

    def along():
        x, y = probe.xy(probe.truth)
        return (x - x0) * c + (y - y0) * s

    # The 1 m box, its near face `ahead` m in front of the bumper, on the car's line.
    bx, by = x0 + (front + ahead + 0.5) * c, y0 + (front + ahead + 0.5) * s
    if not gz_service('create', 'gz.msgs.EntityFactory',
                      f'sdf_filename: "{OBSTACLE}", name: "obstacle", '
                      f'pose: {{position: {{x: {bx}, y: {by}, z: 0}}, '
                      f'orientation: {{z: {math.sin(h / 2.0)}, w: {math.cos(h / 2.0)}}}}}'):
        result(name, False, "could not place the box (Gazebo's create service)")
        return
    done = probe.go_to(x0 + goal * c, y0 + goal * s, h)
    if done is None:
        result(name, False, 'no Nav2, or it refused the goal')
        gz_service('remove', 'gz.msgs.Entity', 'name: "obstacle", type: MODEL')
        return
    seen, top, still, gap = len(probe.monitor), 0.0, None, None
    end = time.monotonic() + 240.0
    while time.monotonic() < end and not done.done():
        probe.spin_for(0.1)
        v = probe.truth.twist.twist.linear.x
        top = max(top, v)
        if gap is None and top > 0.5 and abs(v) < 0.02:
            still = still if still is not None else probe.now()
            if probe.now() - still > 3.0:      # stood still for 3 s: take the box away
                gap = ahead - along()
                gz_service('remove', 'gz.msgs.Entity', 'name: "obstacle", type: MODEL')
        else:
            still = None
    stop_zone = any(a == CollisionMonitorState.STOP and p == 'StopZone'
                    for a, p in probe.monitor[seen:])
    reached = done.done() and done.result().status == 4
    passed = along() > front + ahead + 1.0
    ok = gap is not None and 0.0 < gap < 2.0 and stop_zone and reached and passed
    where = f'stood still {gap:.2f} m before the box' if gap is not None else 'never stopped'
    result(name, ok, f'{where} (top speed {top:.2f} m/s), stop zone fired: {stop_zone}; box '
                     f'taken away, Nav2 goal {"reached" if reached else "not reached"} '
                     f'{along():.1f} m on')
    if gap is None:
        gz_service('remove', 'gz.msgs.Entity', 'name: "obstacle", type: MODEL')


def check_lane_keeping(probe):
    """
    Lane keeping alone: from LANE_START m left of the road's centre, the car must come back.

    control_interface drives from the moment it sees the lane, so this reads the track the
    probe has kept since start-up: LANE_TIME seconds of Gazebo's time, from when the car
    started moving.
    """
    def moved():
        return next((s for s in probe.track if s[4] > 0.1), None)

    if not probe.spin_for(60.0, lambda: moved() is not None):
        result('lane keeping', False, f'the car never moved ({len(probe.lanes)} lane messages)')
        return
    t0, x0, y0 = moved()[:3]
    probe.spin_for(4.0 * LANE_TIME, lambda: probe.track[-1][0] - t0 >= LANE_TIME)
    run = [s for s in probe.track if t0 <= s[0] <= t0 + LANE_TIME]
    stamps = [s[0] for s in run]

    def at(t):
        """Return the truth sample nearest to time t (s, Gazebo's)."""
        i = min(bisect.bisect_left(stamps, t), len(stamps) - 1)
        return run[i - 1 if i and t - stamps[i - 1] < stamps[i] - t else i]

    start = y0 - ROAD_Y
    outside = [s[0] - t0 for s in run if abs(s[2] - ROAD_Y) > LANE_BAND]
    settled = outside[-1] if outside else 0.0   # s of driving: within the band from then on
    over = max(0.0, max(-math.copysign(1.0, start) * (s[2] - ROAD_Y) for s in run))
    last = [s for s in run if s[0] >= stamps[-1] - 5.0]
    end = sum(s[2] for s in last) / len(last) - ROAD_Y
    heading = sum(s[3] for s in last) / len(last)
    # lane_finder against the truth: the car's offset LOOKAHEAD m ahead, square to the road.
    errors, both = [], 0
    for t, offset, two in (m for m in probe.lanes if t0 <= m[0] <= stamps[-1]):
        _, _, y, yaw, _ = at(t)
        errors.append(abs(offset - (y - ROAD_Y + LOOKAHEAD * math.sin(yaw)) / math.cos(yaw)))
        both += two
    every = '  '.join(f'{at(t0 + k)[2] - ROAD_Y:+.2f}' for k in range(0, int(LANE_TIME) + 1, 3))
    print(f'      the offset (m), every 3 s of driving: {every}', flush=True)
    driven = stamps[-1] - t0
    ok = (driven >= LANE_TIME - 1.0 and settled <= LANE_TIME - 5.0 and over < 0.10
          and abs(end) < LANE_BAND and bool(errors) and both >= 0.9 * len(errors))
    finder = (f'lane_finder {100 * sum(errors) / len(errors):.1f} cm from the truth on average, '
              f'both lines in {both} of {len(errors)} pictures' if errors else 'no lane messages')
    result('lane keeping', ok,
           f'{start:+.2f} m to {end:+.2f} m, within {100 * LANE_BAND:.0f} cm after '
           f'{settled:.1f} s and {at(t0 + settled)[1] - x0:.1f} m, overshoot {100 * over:.0f} '
           f'cm, heading {heading:+.3f} rad, over {driven:.0f} s; {finder}')


def session(launch_args, checks, log_path, label=''):
    """Start sim.launch.py's road world with these arguments, run the checks, and stop it."""
    tag = f' ({label})' if label else ''
    env = dict(os.environ)
    print(f"Starting Gazebo's road world{tag} on ROS domain {env['ROS_DOMAIN_ID']}, Gazebo "
          f"partition {env['GZ_PARTITION']} (this computer only)...", flush=True)
    log = open(log_path, 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'aav_bringup', 'sim.launch.py', 'world:=road',
                               'gui:=false', 'rviz:=false', *launch_args],
                              cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    rclpy.init()
    probe = Probe()
    try:
        def missing():
            return [n for n, m in (('truth', probe.truth), ('global EKF', probe.glob),
                                   ('camera', probe.image), ('lens', probe.info),
                                   ('radar', probe.cloud)) if m is None]

        up = probe.spin_for(120.0, lambda: not missing())
        # Nav2 comes up last: its lifecycle manager starts its servers one at a time. On a
        # busy computer it can stall on a lost reply, and then Nav2 never comes up at all.
        nav = 'nav:=false' not in launch_args
        nav_up = up and (not nav or probe.spin_for(120.0, probe.nav.server_is_ready))
        if not up:
            detail = f'nothing from: {", ".join(missing())}, in 120 s'
        elif not nav_up:
            detail = ('the sensors run, but Nav2 never came up in 120 s (its lifecycle '
                      f'manager stalled? See {log_path})')
        else:
            detail = ("Gazebo's truth, the global EKF, the camera and the radar stand-in are "
                      'running' + (', and Nav2 is up' if nav else ''))
        result(f'start-up{tag}', nav_up, detail)
        up = nav_up
        if up:
            check_contract(probe, env, tag)
            for check in checks:
                check(probe)
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        log.close()


def main():
    args = sys.argv[1:]
    perception = '--perception' in args
    if perception and importlib.util.find_spec('ultralytics') is None:
        print('--perception needs YOLO: pixi run -e ml sim-check --perception', file=sys.stderr)
        return 2
    checks = [check_sensors, check_drive]
    if perception:
        checks.append(check_stop_sign)
    if '--obstacle' in args:
        checks.append(check_obstacle)
    log = os.environ.get('AAV_SIM_LOG', '/tmp/aav-sim-check.log')
    logs = [log]
    session([f'perception:={str(perception).lower()}'], checks, logs[0])
    if '--lane' in args:
        # Its own run: control_interface drives whenever it sees the lane, so it would take
        # the wheel between the other checks. Nothing else drives here: no Nav2.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-lane{ext}')
        session(['lane:=true', 'nav:=false', f'start_y:={ROAD_Y + LANE_START}'],
                [check_lane_keeping], logs[1], 'lane')

    print('\n| Check | Result | Detail |\n|---|---|---|')
    for name, ok, detail in RESULTS:
        print(f'| {name} | {"✅" if ok else "❌"} | {detail} |')
    print(f'\nThe launch log: {", ".join(logs)}')
    return 0 if all(ok for _, ok, _ in RESULTS) else 1


if __name__ == '__main__':
    sys.exit(main())
