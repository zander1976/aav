# Names and code style

One naming system and one way to lay out code, for new code and inherited code alike.
Inherited code is ours to edit: bring it in line when you touch it, and note the change
in its `PROVENANCE.md`.

## Names

| Thing | Pattern | Examples |
|---|---|---|
| Our package | `aav_<what>` | `aav_ego`, `aav_spine`, `aav_teleop` |
| A device driver package | `aav_<kind>_<maker>`, so drivers of one kind sort together | `aav_imu_yahboom` |
| Folder under `src/` | the layer of the picture | `sensors/`, `localization/`, `drivers/`, `mux/`, `ego/`, `backends/` |
| Node and executable | what it is, in snake_case; both the same | `ego`, `spine_bridge`, `scripted_driver` |
| Logic module | a noun, no ROS imports | `ackermann.py`, `can_codec.py`, `vehicle.py` |
| Topic | absolute: `/<area>/<thing>` | `/cmd_vel/joy`, `/vehicle/odom`, `/ego/actuator_cmd` |
| Frame | `<thing>_link`, plus `map`, `odom`, `base_link` | `imu_link`, `gps_link` |
| Parameter | snake_case, SI units, the unit in a comment | `max_speed: 1.0  # m/s` |
| Config file | `config/<node>.yaml`; its top key is the node's name, or ROS ignores it | `config/ego.yaml` starts with `ego:` |
| Launch file | `launch/<what>.launch.py`; whole-system profiles live in `aav_bringup` | `ego.launch.py`, `turtle.launch.py` |
| Message, action | a CamelCase noun | `VehicleStatus`, `ScriptedDrive` |
| Rust crate | kebab-case | `spine-core` |
| Docs page | short kebab-case | `how-it-fits.md` |

## Renames

The inherited packages kept their old names until someone brought them in. Renaming
changes their launch files and executables.

| Was | Now | When |
|---|---|---|
| `maincar` (the HUD's web server) | `aav_hud` | done, 10 October |
| `lane_geometry`, `object_detection`, `yolo_ros2` | `aav_lane_geometry`, `aav_object_detection`, `aav_yolo` | done, 10 October, when perception came in |
| `yahboom_imu_cpp` | `aav_imu_yahboom` | done, 11 October: Ben's Rust rewrite replaced it |
| `ti_ros2_driver` | `aav_radar_ti` | proposed [[needs-you]] |
| `battery_monitor` | `aav_battery_victron` | proposed [[needs-you]] |

## How a package is laid out

```text
src/<layer>/aav_<what>/
  package.xml              its dependencies
  setup.py, setup.cfg      its executables (Python packages)
  aav_<what>/
    <logic>.py             the maths: no ROS imports, unit-tested
    <node>.py              the ROS node: subscribe, call the logic, publish
  config/<node>.yaml       its parameters
  launch/<what>.launch.py  how to start it
  test/test_<logic>.py     tests for the logic
  PROVENANCE.md            inherited code only: where it came from, what changed
```

## Writing code here

1. **Purpose first.** Every file opens with a short docstring: what it's for. A node's
   docstring also lists what it subscribes, publishes, serves and calls.
2. **Comments say why**, not what. The code already says what.
3. **Keep the maths out of ROS.** Put it in a plain module with tests; the node stays
   thin.
4. **Prefer configuration to code**: stock packages first, then inherited code, then
   new code.
5. **SI units and REP-103 frames**: metres, radians, seconds; x forward, y left, z up;
   positive steering turns left.
6. **No pins in ROS.** Pins, I2C addresses and calibration live in the firmware.
7. **No secrets in git**: only where a secret lives and what kind it is.
8. **Small commits**, each passing `pixi run test`, each message saying why.
9. **Write each language its own way** (Ben, 10 Oct). Ported code is rewritten in the
   new language's style, not copied line by line.

| In Rust | Not C written in Rust |
|---|---|
| an `enum` for a state or a mode | number codes and `#define`-style constants |
| `Result<T, E>`, with an error type | -1, 0 or a flag for an error |
| iterators: `iter()`, `map`, `filter` | index loops over arrays |
| small types for units: `Metres(f32)` | bare `f32` everywhere |
| `?`, and `expect` with a reason, only where failure is a bug | `unwrap()` in library code |
| traits for a device (`embedded-hal`) | functions that take a bus handle and a pin number |
| `cargo fmt`, `cargo clippy` clean | a style of your own |

## Style, and what checks it

| Language | Style | Checked by |
|---|---|---|
| Python | PEP 8 with 99-character lines, single quotes, docstrings that start with a summary line | `pixi run lint` (ROS's `ament_flake8` and `ament_pep257`) |
| C++ | the ROS 2 C++ style | `ament_uncrustify`, `ament_cpplint` (in `colcon test`) |
| Rust | rustfmt and clippy | `cargo fmt`, `cargo clippy` |
| Markdown docs | short pages, a purpose line, sections, tables | `pixi run docs`, and a look at the result |
