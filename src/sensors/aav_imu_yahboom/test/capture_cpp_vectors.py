#!/usr/bin/env python3
"""
Record what the C++ IMU driver publishes for a fixed byte stream, as Rust test vectors.

Purpose: keep the C++ driver's answers once yahboom_imu_cpp is gone. The stream goes into
yahboom_imu_cpp through a pseudo-terminal, chunk by chunk; every Imu and MagneticField it
publishes is written to crates/yahboom-protocol/tests/cpp_vectors.txt, which
crates/yahboom-protocol/tests/cpp_vectors.rs feeds to the Rust decoder.

    pixi run --frozen tools/ros.sh --local python3 \
        src/sensors/aav_imu_yahboom/test/capture_cpp_vectors.py

It needs the C++ driver built. Its own ROS domain (AAV_TEST_DOMAIN_ID, else 77), on this
computer only. Bytes 0x0D are left out: the C++ driver leaves its port turning them
into 0x0A (compare_with_cpp.py shows it), and these vectors are about the decoding.
"""

import os
from pathlib import Path
import random
import sys
import threading
import time

from compare_with_cpp import (ACCEL, ANGLE, Collector, CR, exited_early, frame, GYRO, MAG,
                              start, stop)
import rclpy
from rclpy.executors import SingleThreadedExecutor

OUT = (Path(__file__).resolve().parents[1] / 'crates' / 'yahboom-protocol' / 'tests'
       / 'cpp_vectors.txt')
I16_MIN, I16_MAX = -32768, 32767


def one_set(accel, gyro, mag):
    return frame(ACCEL, [*accel, 0]) + frame(GYRO, [*gyro, 0]) + frame(MAG, [*mag, 0]) \
        + frame(ANGLE, [0, 0, 0, 0])


def chunks():
    """Return the stream as (what it shows, bytes): no 0x0D, and no 0x55 outside a frame."""
    out = [('all zero', one_set([0, 0, 0], [0, 0, 0], [0, 0, 0])),
           ('+1 g on z, 100 deg/s about z', one_set([0, 0, 2048], [0, 0, 1638], [100, 200, -300]))]
    for v in (I16_MIN, I16_MIN + 1, -12345, -1, 1, 777, 2048, 16384, I16_MAX):
        out.append((f'{v} on every axis', one_set([v, v, v], [v, v, v], [v, v, v])))
    out.append(('a different extreme on each axis',
                one_set([I16_MIN, 0, I16_MAX], [I16_MAX, I16_MIN, 0], [0, I16_MAX, I16_MIN])))
    rng = random.Random(2026)
    while len(out) < 40:
        raw = [[rng.randint(I16_MIN, I16_MAX) for _ in range(3)] for _ in range(3)]
        data = one_set(*raw)
        if CR not in data:
            out.append(('random', data))
    out.append(('an unknown frame type (0x59, quaternion) inside a set',
                frame(ACCEL, [10, 20, 30, 0]) + frame(0x59, [1, 2, 3, 4])
                + frame(GYRO, [40, 50, 60, 0]) + frame(MAG, [70, 80, 90, 0])
                + frame(ANGLE, [0, 0, 0, 0])))
    bad = bytearray(frame(ACCEL, [1000, 2000, 3000, 0]))
    bad[-1] ^= 0xFF
    out.append(('a bad checksum on the acceleration: the last acceleration stays',
                bytes(bad) + frame(GYRO, [-40, -50, -60, 0]) + frame(MAG, [-70, -80, -90, 0])
                + frame(ANGLE, [0, 0, 0, 0])))
    out.append(('two angle frames: the same readings, twice',
                frame(ANGLE, [0, 0, 0, 0]) + frame(ANGLE, [0, 0, 0, 0])))
    out.append(('bytes between frames are skipped',
                bytes([0x00, 0x13, 0xFF, 0x51, 0x54])
                + one_set([5, 6, 7], [8, 9, 10], [11, 12, 14])))
    out.append(('frames alone: acceleration, gyro and field, then no angle frame',
                frame(ACCEL, [-2048, 0, 0, 0]) + frame(GYRO, [0, -1638, 0, 0])
                + frame(MAG, [-1, -2, -3, 0])))
    out.append(('the angle frame that closes it', frame(ANGLE, [0, 0, 0, 0])))
    for _, data in out:
        assert CR not in data
    return out


def sample(imu, mag):
    a, w, m = imu.linear_acceleration, imu.angular_velocity, mag.magnetic_field
    return [a.x, a.y, a.z], [w.x, w.y, w.z], [m.x, m.y, m.z]


def fixed_fields(imu, mag):
    """Return the fields that are the same in every message: the node's settings."""
    q = imu.orientation

    def floats(values):
        return tuple(float(v) for v in values)

    return (imu.header.frame_id, mag.header.frame_id, floats((q.x, q.y, q.z, q.w)),
            floats(imu.orientation_covariance), floats(imu.angular_velocity_covariance),
            floats(imu.linear_acceleration_covariance), floats(mag.magnetic_field_covariance))


def capture(collector, stream):
    """Play each chunk into the C++ driver; return the samples each one made it publish."""
    master, slave = os.openpty()
    before = collector.publishers()
    proc = start('yahboom_imu_cpp', os.ttyname(slave))
    collector.wait_for_driver(before)
    exited_early(proc, 'yahboom_imu_cpp')
    collector.imu.clear()
    collector.mag.clear()
    results, fixed = [], set()
    for _, data in stream:
        start_at = len(collector.imu)
        os.write(master, data)
        # The C++ driver's port waits for 11 bytes, or 1 s of silence (VMIN 11, VTIME 10),
        # so a chunk's last frame comes out a second after it.
        collector.wait_quiet(1.5)
        imu, mag = collector.imu[start_at:], collector.mag[start_at:]
        if len(imu) != len(mag):
            raise RuntimeError('an Imu without its MagneticField, or the other way round')
        results.append([sample(i, m) for i, m in zip(imu, mag)])
        fixed |= {fixed_fields(i, m) for i, m in zip(imu, mag)}
    stop(proc)
    os.close(master)
    os.close(slave)
    if len(fixed) != 1:
        raise RuntimeError(f'the fixed fields changed between messages: {fixed}')
    return results, fixed.pop()


def numbers(values):
    # repr() is the shortest text that reads back to the same float, bit for bit.
    return ' '.join(repr(float(v)) for v in values)


def write(stream, results, fixed):
    frame_id, mag_frame, q, o_cov, g_cov, a_cov, m_cov = fixed
    lines = [
        '# What yahboom_imu_cpp (the 2025-26 C++ IMU driver) published for this byte stream.',
        '# Made by test/capture_cpp_vectors.py on '
        f'{time.strftime("%Y-%m-%d")}, through a pseudo-terminal, with the',
        '# C++ driver as it was in aav just before it was removed. Read by cpp_vectors.rs.',
        '#',
        "# Every message also had these, from the driver's default settings:",
        f'#   frame_id {frame_id} (Imu) and {mag_frame} (MagneticField); orientation {q};',
        f'#   orientation_covariance {list(o_cov)};',
        f'#   angular_velocity_covariance {list(g_cov)};',
        f'#   linear_acceleration_covariance {list(a_cov)};',
        f'#   magnetic_field_covariance {list(m_cov)}.',
        '#',
        '# "bytes" is one chunk of the stream, in hex, written to the port in one go. Each',
        '# "sample" under it is one Imu + MagneticField pair the chunk made the driver publish:',
        '#   accel x y z (m/s^2), gyro x y z (rad/s), mag x y z (raw counts, y negated).',
        '# The numbers read back to the published float64s bit for bit (-0.0 included).',
    ]
    for (what, data), samples in zip(stream, results):
        lines += ['', f'# {what}', 'bytes ' + data.hex(' ')]
        lines += [f'sample {numbers(a)} | {numbers(g)} | {numbers(m)}' for a, g, m in samples]
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text('\n'.join(lines) + '\n')


def main():
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('AAV_TEST_DOMAIN_ID', '77')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    stream = chunks()
    rclpy.init()
    collector = Collector()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    try:
        results, fixed = capture(collector, stream)
    except RuntimeError as err:
        print(f'Could not capture: {err}')
        return 1
    finally:
        executor.shutdown()
        spinner.join()
        collector.destroy_node()
        rclpy.shutdown()
    write(stream, results, fixed)
    count = sum(len(r) for r in results)
    print(f'{len(stream)} chunks, {count} samples: {OUT}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
