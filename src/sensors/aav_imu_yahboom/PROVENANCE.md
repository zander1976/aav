# Provenance

Written for `aav` on 2026-10-09 and 10 (Ben): a Rust rewrite of the 2025-26 team's C++
IMU driver, `yahboom_imu_cpp`. On 2026-10-11 it became the only IMU driver, and the C++
one was removed.

| Part | From the C++ driver |
|---|---|
| `crates/yahboom-protocol` | the frame layout, the units and the scale factors in `src/imu_node.cpp` |
| `crates/imu-node` | the node name, parameters, topics and covariances. Where it differs, on purpose, is listed at the top of `src/main.rs` |
| `crates/yahboom-protocol/tests/cpp_vectors.*` | what the C++ driver published for a fixed byte stream, recorded just before it was removed |

## The C++ driver it replaced

`yahboom_imu_cpp`: copied on 2026-10-09 from github.com/AAV-2025-2026/yahboom-IMU @ main
(95469f1a63), and removed on 2026-10-11. To find it in git:
`git log --oneline -- src/sensors/yahboom_imu_cpp`.

What `aav` changed in it, while it ran:
- 2026-10-09: `gyro_stddev` (0.02 rad/s) and `accel_stddev` (0.2 m/s²), published as the
  covariances' diagonals. They were all zero, which robot_localization reads as a perfect
  sensor. The Rust driver has the same two.
- 2026-10-10: it compiled on macOS (`B921600` behind `#ifdef`); the baud logged with `%d`;
  the port `/dev/aav/imu` (the udev name); lint fixes, so `colcon test` passed (13 checks
  failed before); its README said Jazzy, not Humble.

Its known issues. The Rust driver keeps them on purpose, so its messages are the same:
- `/imu/mag` is in raw counts with y negated, not tesla. So `imu_filter_madgwick` runs with
  `use_mag: false`, and yaw drifts. Convert it and calibrate before using the magnetometer.
- The topics are absolute (`/imu/data_raw`, `/imu/mag`): a namespace won't move them.

From its README (the 2025-26 team), still true of the IMU:
- It should send 200 sets a second. If `ros2 topic hz /imu/data_raw` shows less, set its
  output rate to 200 Hz with Yahboom's Windows tool, `MiniMU.exe`, in its config panel
  only. The download link is in the 2025-26 repo's README.
- `raw_check.py`, in the 2025-26 repo, prints raw frames and checks their checksums,
  without ROS. The Rust driver logs "Checksum mismatch" for the same faults.

## How the two compared, before the C++ went

`pixi run imu-compare`, on Ben's MacBook, 11 Oct: the same bytes into both, through a
pseudo-terminal.

| Check | C++ | Rust |
|---|---|---|
| 200 clean sets: every field as the formulas say | 200 of 200 | 200 of 200 |
| The clean sets' messages, C++ against Rust | identical | identical |
| 50 sets with a 0x0D byte in them | 0 of 50 | 50 of 50 |
| Ctrl-C after the data stops | hangs: killed after 5 s | stops, exit code 0 |
| The port goes away | keeps running, 107% CPU | exits, code 1 |
| Checksum warnings logged | 54 | 0 |

- `test/compare_with_cpp.py` made that table, and `test/capture_cpp_vectors.py` recorded
  `cpp_vectors.txt`. Both needed the C++ driver, so they went with it. To find them:
  `git log --oneline -- src/sensors/aav_imu_yahboom/test`.
- `tests/cpp_vectors.rs` keeps the comparison: the same 46 chunks of bytes must give the
  C++ driver's 46 samples, bit for bit.

## Changes

- 2026-10-10: `test/compare_with_cpp.py` reads CPU time with `ps` where there's no `/proc`
  (macOS); `pixi run imu-compare` runs it.
- 2026-10-10: `test/compare_with_cpp.py` stops with the driver's own message if a driver
  exits at start-up, instead of waiting forever on its port. On a Mac the Rust driver
  can't open the script's fake port (macOS's baud-rate call fails on it), so the
  comparison runs on Linux. A real USB port should accept that call: untested on a Mac.
- 2026-10-11: the comparison runs on a Mac too.
  - `crates/imu-node/src/main.rs`: `baud_rate: 0` is accepted (it was refused). It
    leaves the speed alone, for a pseudo-terminal in a test: serialport skips macOS's
    speed call for 0.
  - `test/compare_with_cpp.py`: on a Mac, both drivers get `baud_rate:=0`. It waits for
    the new driver's own publisher: a killed driver's lingers in the graph, so it used to
    play the sets before the Rust driver was connected, and Rust got none. macOS's EPERM,
    when it signals a driver that has just exited, no longer stops it.
  - Result on Ben's MacBook (`pixi run imu-compare`): both drivers 200 of 200 clean sets,
    with identical messages. Sets with a 0x0D byte: C++ 0 of 50, Rust 50 of 50.
- 2026-10-11: the C++ driver's answers kept as test vectors: `tests/cpp_vectors.rs` and
  `.txt`, made by `test/capture_cpp_vectors.py`.
- 2026-10-11: the only IMU driver.
  - `aav_sensors`' `sensors.launch.py` always starts it. The `imu_driver` argument (cpp or
    rust) is gone, from `car.launch.py` too.
  - `package.xml`, `crates/imu-node/Cargo.toml`, `src/main.rs` and
    `crates/yahboom-protocol/src/lib.rs`: their descriptions call the C++ driver the one
    this replaced, not a twin. No code changed.
  - `test/compare_with_cpp.py` and `test/capture_cpp_vectors.py` removed, with
    `pixi run imu-compare`: they needed the C++ driver.
