#!/usr/bin/env python3
"""
Feed the same bytes to the C++ and the Rust IMU drivers, and compare what they publish.

Purpose: check, with no IMU attached, that aav_imu_yahboom (Rust) is a drop-in twin of
yahboom_imu_cpp (C++). A pseudo-terminal stands in for the USB port; this script plays
the IMU's frames into it and checks every message against the C++ driver's formulas.

    pixi run imu-compare        (after `pixi run build`)

It uses its own ROS domain, on this computer only, so it can't disturb a running car.
"""

import os
import random
import signal
import struct
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, MagneticField

DRIVERS = [('C++', 'yahboom_imu_cpp'), ('Rust', 'aav_imu_yahboom')]
HEADER, ACCEL, GYRO, ANGLE, MAG = 0x55, 0x51, 0x52, 0x53, 0x54
CR = 0x0D          # a carriage-return byte: a terminal not in raw mode turns it into 0x0A
GYRO_VAR, ACCEL_VAR = 0.02 * 0.02, 0.2 * 0.2   # the default stddevs, squared


def frame(kind, values):
    """Build one 11-byte frame, as the IMU sends it."""
    body = bytes([HEADER, kind]) + struct.pack('<4h', *values)
    return body + bytes([sum(body) & 0xFF])


def make_sets(rng, count, with_cr):
    """Raw readings for `count` sets; with_cr puts a 0x0D byte in every set."""
    sets = []
    while len(sets) < count:
        raw = {k: [rng.randint(-32768, 32767) for _ in range(3)] for k in ('a', 'g', 'm')}
        if with_cr:
            raw['g'][rng.randrange(3)] = 0x0D00 + rng.randrange(256)   # high byte 0x0D
        data = set_bytes(raw)
        if with_cr == (CR in data):
            sets.append(raw)
    return sets


def set_bytes(raw):
    """The four frames of one set, in the IMU's order."""
    return (frame(ACCEL, raw['a'] + [0]) + frame(GYRO, raw['g'] + [0])
            + frame(MAG, raw['m'] + [0]) + frame(ANGLE, [0, 0, 0, 0]))


def expected(raw):
    """What the C++ driver's formulas give, in its order of operations."""
    accel = [r / 32768.0 * 16.0 * 9.8 for r in raw['a']]
    gyro = [r / 32768.0 * 2000.0 * 3.141592653589793 / 180.0 for r in raw['g']]
    mag = [float(raw['m'][0]), -float(raw['m'][1]), float(raw['m'][2])]
    return accel, gyro, mag


def as_values(imu, mag):
    """The fields that must match, from one Imu and one MagneticField message."""
    a, w, m, q = imu.linear_acceleration, imu.angular_velocity, mag.magnetic_field, imu.orientation
    return ([a.x, a.y, a.z], [w.x, w.y, w.z], [m.x, m.y, m.z],
            imu.header.frame_id, mag.header.frame_id, (q.x, q.y, q.z, q.w),
            imu.orientation_covariance[0],
            [imu.angular_velocity_covariance[i] for i in (0, 4, 8)],
            [imu.linear_acceleration_covariance[i] for i in (0, 4, 8)])


def as_expected(raw):
    accel, gyro, mag = expected(raw)
    return (accel, gyro, mag, 'imu_link', 'imu_link', (0.0, 0.0, 0.0, 1.0), -1.0,
            [GYRO_VAR] * 3, [ACCEL_VAR] * 3)


def matched(want, got):
    """How many of `want` appear in `got`, in order (frames are only ever dropped)."""
    count, i = 0, 0
    for w in want:
        while i < len(got) and got[i] != w:
            i += 1
        if i < len(got):
            count, i = count + 1, i + 1
    return count


class Collector(Node):
    """Keeps every message the driver publishes."""

    def __init__(self):
        super().__init__('imu_compare')
        qos = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE)
        self.imu, self.mag = [], []
        self.create_subscription(Imu, '/imu/data_raw', self.imu.append, qos)
        self.create_subscription(MagneticField, '/imu/mag', self.mag.append, qos)

    def publishers(self):
        return {bytes(p.endpoint_gid) for p in self.get_publishers_info_by_topic('/imu/mag')}

    def wait_for_driver(self, before, timeout=20.0):
        """Wait for a publisher not in `before`: a killed driver's lingers in the graph."""
        end = time.monotonic() + timeout
        while not self.publishers() - before and time.monotonic() < end:
            time.sleep(0.1)
        time.sleep(1.5)   # the driver configures the port, and the topics connect

    def wait_quiet(self, quiet=1.0):
        last = -1
        while len(self.imu) != last:
            last = len(self.imu)
            time.sleep(quiet)


def start(package, port):
    exe = os.path.join(get_package_prefix(package), 'lib', package, 'imu_node')
    # macOS refuses to set a speed on a pseudo-terminal, so there both drivers get 0: the
    # Rust one then leaves the speed alone, and the C++ one falls back to 9600, which a
    # pseudo-terminal ignores.
    baud = ['-p', 'baud_rate:=0'] if sys.platform == 'darwin' else []
    return subprocess.Popen([exe, '--ros-args', '-p', f'port:={port}', *baud],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)


def play(master, sets):
    for raw in sets:   # 200 sets a second, as the IMU sends them
        os.write(master, set_bytes(raw))
        time.sleep(0.005)


def cpu_seconds(pid):
    """Return the CPU time a process has used: /proc on Linux, ps elsewhere (macOS)."""
    try:
        with open(f'/proc/{pid}/stat', encoding='utf-8') as f:
            fields = f.read().rsplit(')', 1)[1].split()
        return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
    except FileNotFoundError:
        # ps prints [[dd-]hh:]mm:ss, with hundredths on macOS
        out = subprocess.run(['ps', '-o', 'time=', '-p', str(pid)], capture_output=True,
                             text=True).stdout.strip()
        days, _, clock = out.rpartition('-')
        seconds = 0.0
        for part in clock.split(':'):
            seconds = seconds * 60 + float(part)
        return seconds + 86400 * int(days or 0)


def signal_group(proc, sig):
    """Signal the driver's process group, even if the driver has just exited."""
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass
    except PermissionError:
        # macOS refuses (EPERM) when the group's one process has exited but isn't reaped.
        proc.send_signal(sig)   # it checks first, so a reaped process is left alone


def stop(proc, sig=signal.SIGINT, timeout=5.0):
    """Signal the driver's process group; return its exit code, or None if it hung."""
    signal_group(proc, sig)
    try:
        return proc.wait(timeout)
    except subprocess.TimeoutExpired:
        signal_group(proc, signal.SIGKILL)
        proc.wait()
        return None


def exited_early(proc, package):
    """Stop the comparison if the driver died at start-up: writes to its port would block."""
    if proc.poll() is not None:
        raise RuntimeError(f'{package} exited at start-up: {proc.stdout.read().strip()}')


def run(package, collector, clean, with_cr):
    """Play both kinds of sets into one driver; then Ctrl-C it, and then unplug a second."""
    result = {}
    master, slave = os.openpty()   # the slave keeps a fresh terminal's settings, as ttyUSB0
    before = collector.publishers()
    proc = start(package, os.ttyname(slave))
    collector.imu.clear()
    collector.mag.clear()
    collector.wait_for_driver(before)
    exited_early(proc, package)
    for name, sets in (('clean', clean), ('cr', with_cr)):
        before = len(collector.imu)
        play(master, sets)
        collector.wait_quiet()
        got = [as_values(i, m) for i, m in zip(collector.imu[before:], collector.mag[before:])]
        result[name] = (matched([as_expected(r) for r in sets], got), len(sets))
        result[name + '_msgs'] = got
    code = stop(proc)
    result['sigint'] = 'hangs: killed after 5 s' if code is None else f'stops, exit code {code}'
    result['log'] = proc.stdout.read()
    os.close(master)
    os.close(slave)

    # Unplugged: the port goes away under a running driver.
    master, slave = os.openpty()
    before = collector.publishers()
    proc = start(package, os.ttyname(slave))
    collector.wait_for_driver(before)
    exited_early(proc, package)
    os.close(slave)
    os.close(master)
    time.sleep(2.0)
    if proc.poll() is None:
        before = cpu_seconds(proc.pid)
        time.sleep(1.0)
        result['unplug'] = f'keeps running, {100 * (cpu_seconds(proc.pid) - before):.0f}% CPU'
        stop(proc, signal.SIGKILL)
    else:
        result['unplug'] = f'exits, code {proc.returncode}'
    return result


def main():
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('AAV_TEST_DOMAIN_ID', '77')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    rng = random.Random(2026)
    clean, with_cr = make_sets(rng, 200, False), make_sets(rng, 50, True)

    rclpy.init()
    collector = Collector()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    try:
        results = {lang: run(package, collector, clean, with_cr) for lang, package in DRIVERS}
    except RuntimeError as err:
        print(f'Could not compare: {err}')
        return 1
    finally:
        executor.shutdown()
        spinner.join()
        collector.destroy_node()
        rclpy.shutdown()

    cpp, rust = results['C++'], results['Rust']
    rows = [
        ('Clean sets: every field as the formulas say', 'clean'),
        ('Sets with a 0x0D byte in them', 'cr'),
    ]
    print(f"\n{'Check':<46} {'C++':<26} Rust")
    for title, key in rows:
        print(f'{title:<46} {"%d of %d" % cpp[key]:<26} {"%d of %d" % rust[key]}')
    same = cpp['clean_msgs'] == rust['clean_msgs']
    print(f"{'Clean sets: C++ and Rust messages identical':<46} {'yes' if same else 'NO'}")
    print(f"{'Ctrl-C after the data stops':<46} {cpp['sigint']:<26} {rust['sigint']}")
    print(f"{'The port goes away':<46} {cpp['unplug']:<26} {rust['unplug']}")
    warnings = [r['log'].count('Checksum mismatch') for r in (cpp, rust)]
    print(f"{'Checksum warnings logged':<46} {warnings[0]:<26} {warnings[1]}")

    ok = (rust['clean'][0] == len(clean) and rust['cr'][0] == len(with_cr)
          and cpp['clean'][0] == len(clean) and same and rust['sigint'].endswith(' 0'))
    print('\nRust twin: OK' if ok else '\nRust twin: FAILED')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
