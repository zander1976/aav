# Infrastructure: how the project is put together

For whoever puts the whole thing together. [How it fits](how-it-fits.md) explains the
car's software: sensors, drivers, the mux, ego, backends. This page explains everything
around it: the repo, the environment, the contracts, the profiles, the computers, the
tests, and how a change goes from an idea to the car.

## The pieces at a glance

| Piece | What it is | Where |
|---|---|---|
| The car's software | ROS 2 Jazzy packages, one folder per layer | `src/` |
| The car's firmware | the CAN contract, the Rust spine logic and its Pico 2 program, the 2025-26 boards' code | `firmware/` |
| The touchscreen app | the 2025-26 Flutter HUD, changed for our back end | `hud/` |
| The environment | ROS, Gazebo, compilers, Python and Rust, all from Pixi | `pixi.toml`, `pixi.lock` |
| The tools | tests, the graph check, the network doctor, recording, docs, the mailer, the MCP server | `tools/` |
| The docs | short markdown pages, built into web pages and one PDF | `docs/` |
| The rules | names, units, secrets, git, testing | `CLAUDE.md` |
| The decisions | Decided, Tentative, Proposed, and Q1–Q34 | the design doc: summary in [Decisions](decisions.md) |

## The repo, folder by folder

```text
aav/
  CLAUDE.md           the rules, for people and Claude sessions alike
  README.md, TODO.md  the front page; the current work list
  pixi.toml           the environment, and the one-word tasks (pixi run <task>)
  pixi.lock           the exact version of every package: committed, so every laptop matches
  src/                ROS 2 packages (colcon builds these)
    aav_bringup/        whole-system launch profiles: turtle, sim, car, replay, bench
    aav_description/    the car's model (URDF) and vehicle.yaml, the car's measurements
    aav_interfaces/     our own messages and actions
    sensors/            drivers that turn devices into standard messages
    localization/       the two EKFs and navsat_transform: configuration only
    drivers/            everything that proposes a command: teleop, scripted, Nav2
    mux/                twist_mux: picks one command
    ego/                the virtual car: one command into speed and steering
    backends/           whatever plays the car: the real car, Gazebo, turtlesim, CARLA
    perception/         YOLO and the stop-sign watcher, radar clustering, the lane finder, lane keeping
    hud/                the HUD's web server (aav_hud)
    bench/              bench mode: the spine on a laptop, and pretend sensors
  firmware/           microcontroller code (colcon ignores it)
    can/                aav.dbc, the CAN contract, and its test vectors
    spine/              Rust: spine-core (the spine's logic), aav-inputs, the device drivers,
                        and boards/pico2, the spine's board program
    legacy/             the 2025-26 ESP32 and Pico code that runs the car today
  hud/                the 2025-26 Flutter app (colcon ignores it)
  examples/           rust-listener: a first ROS node in Rust
  models/             trained model files, not in git: pixi run get-models
  tools/              scripts: see the table below
  docs/               these pages; docs/history/ keeps the Q&A, Ben's requests and the audits
```

## The three contracts

Three files say how the parts must talk. Change the contract first, then the code, and
the checks tell you if the two disagree.

| Contract | Holds | Checked by | After you change it |
|---|---|---|---|
| `docs/interfaces.yaml` | every topic: its type, who publishes it, who reads it, and in which profiles it must be live | `pixi run check <profile>` on a running system; `pixi run test` checks that the Topics page matches | `pixi run python tools/gen_interfaces_md.py > docs/interfaces.md` |
| `firmware/can/aav.dbc` | every CAN frame between ROS and the spine | test vectors, shared by the Python codec and the Rust tests | update `can_codec.py` and `protocol.rs`, run `firmware/can/gen_test_vectors.py`, run both test suites |
| `src/aav_description/config/vehicle.yaml` | every measurement of the car | its unit test; ego, the URDF, Nav2 and the adapters all read it | measure the car: [Car numbers](car_measurements.md) |

## The environment: Pixi

Pixi installs ROS 2 Jazzy and every tool into the repo's own `.pixi` folder, from
RoboStack and conda-forge. No admin rights, and the same versions on every computer,
because `pixi.lock` pins them all.

| Platform in `pixi.toml` | Who uses it |
|---|---|
| `linux-64` | Linux laptops and oirp-spss |
| `linux-aarch64` | the car's ARM computers (Orin, Pis), to run Jazzy before a reflash |
| `osx-arm64`, `osx-64` | Macs, Apple Silicon and Intel (macOS 14 or newer) |

| Command | Does |
|---|---|
| `pixi install` | downloads and installs everything in `pixi.lock` (about 7 GB the first time) |
| `pixi run <task>` | runs a task inside the environment: see the next table |
| `pixi run <any command>` | runs any command inside it, e.g. `pixi run tools/ros.sh ros2 topic list` |
| `pixi shell` | a terminal inside it; then `source install/setup.zsh` (Mac) or `setup.bash` (Linux) |
| `pixi run -e ml <task>` | the same, in the `ml` environment: PyTorch and YOLO on top of everything else |

| Environment | Holds | Who uses it |
|---|---|---|
| `default` | ROS 2 Jazzy, Gazebo, Nav2, Rust, the compilers | everyone |
| `ml` | the default, plus PyTorch, Ultralytics (YOLO) and, on Macs, Core ML tools | whoever runs the detector: `pixi run detect`, `pixi run -e ml bench perception:=true`. Linux PCs and Macs only |
| `docs` | Markdown and WeasyPrint | `pixi run docs` |
| `mcp` | the default, plus the MCP Python SDK | an AI app, through `.mcp.json`: [AI apps (MCP)](mcp.md) |
| `firmware` | Python and PlatformIO, no ROS | today's C++ boards: `pixi run -e firmware legacy-esp32` |

!!! note "Macs only: GNU coreutils"
  On a Mac, macOS's System Integrity Protection deletes `DYLD_LIBRARY_PATH` whenever
  `/bin/sh` or `/usr/bin/env` starts, and ROS needs that setting to load our own messages.
  So the Mac environment adds GNU `coreutils` for `colcon test`, and scripts that run ROS
  commands go through `tools/ros.sh`. Details: [Your Mac](mac.md).

## The one-word tasks

All defined in `pixi.toml`. Run them from the repo's top folder.

| Task | Does |
|---|---|
| `build` | compiles our packages (`colcon build --symlink-install`) |
| `test` | every test: unit tests, the Rust spine, style, `colcon test` |
| `lint` | ROS's style rules on our Python |
| `turtle` | the stack, with turtlesim as the car |
| `keyboard` | drive from the keyboard (`--sim` with Gazebo) |
| `sim` | the stack, with Gazebo as the car (`gui:=false` on a Mac) |
| `bench` | the car's own software, with stand-ins for the hardware: [Bench mode](bench.md) |
| `bench-check` | starts the bench and tests the whole graph: contract, drive, e-stops. `--perception` adds the stop sign, `--lane` lane keeping |
| `gz-window` | Gazebo's window, for a simulator started with `gui:=false` |
| `car` | the real car: `pixi run car orin` (or `motor_pi`, `laptop`, `all`) |
| `view` | the car's model in RViz |
| `rviz` | RViz with the driving layout: map, costmaps, path, car |
| `graph` | rqt_graph: nodes and topics, live |
| `graph-picture` | saves the running graph as `docs/img/graph.svg` and `.png` |
| `check` | the running graph against the contract: `pixi run check sim` |
| `estop`, `estop-clear` | stop every driver; let them go again |
| `doctor` | can this computer join the car's ROS network? |
| `rust-listener` | the Rust example: prints ego's commands |
| `imu-compare` | feeds the same bytes to the C++ and Rust IMU drivers, and compares them |
| `vendor` | downloads the ROS 2 and rclrs source into `vendor/`, to look things up |
| `get-models` | downloads the stock YOLO model into `models/` |
| `export-coreml` | makes the Core ML model for a Mac's detector (the `ml` environment, Macs only) |
| `get-hud-map` | downloads the HUD app's place database (51 MB) |
| `get-flutter` | downloads Flutter 3.35.7 into `vendor/flutter`, for the HUD app |
| `flutter` | runs that Flutter in `hud/flutter_pi`: `pixi run flutter doctor` |
| `hud-test` | the HUD app's own checks and tests, against the real back end |
| `detect` | YOLO on the camera's images (the `ml` environment): `pixi run detect picture:=<file>` |
| `laptop-bench` | your laptop's parts, its own detector too, against `pixi run bench` on the same computer (the `ml` environment) |
| `docs` | rebuilds the docs pages and the PDF |
| `mcp` | the read-only MCP server, for AI apps (the `mcp` environment). The app starts it: [AI apps (MCP)](mcp.md) |
| `legacy-esp32` | builds today's ESP32 firmware with PlatformIO (the `firmware` environment); `-t upload` flashes it |
| `spine-uf2` | builds the spine's Pico 2 program as a UF2 file, to drag onto the board |
| `spine-tools` | installs probe-rs into `vendor/cargo-tools`, to flash through a debug probe (once) |
| `spine-flash` | flashes the spine's program through a debug probe, then shows its logs |

## The scripts in tools/

| Script | Purpose | How to run it |
|---|---|---|
| `test.sh` | the tests that don't need ROS: Python logic, Rust spine, the Topics page | inside `pixi run test` |
| `lint.sh` | `ament_flake8` and `ament_pep257` on our Python | `pixi run lint` |
| `ros.sh` | runs a command with the workspace sourced | used by most tasks |
| `keyboard.sh` | teleop_twist_keyboard into `/cmd_vel/remote` | `pixi run keyboard` |
| `check_graph.py` | compares a running system with `docs/interfaces.yaml` | `pixi run check <profile>` |
| `gen_interfaces_md.py` | makes the Topics page from the contract | see the contracts table |
| `doctor.py` | network checks for joining the car | `pixi run doctor` |
| `record.sh` | records a drive into `bags/` | `pixi run tools/record.sh <name>` |
| `replay_commands.sh` | plays a recorded drive's commands back into the mux | `pixi run tools/ros.sh tools/replay_commands.sh <bag>` |
| `make_map.py` | draws a Nav2 map from a short spec | `pixi run python tools/make_map.py <spec>` |
| `build_docs.py` | builds the docs pages and the PDF | `pixi run docs` |
| `progress_mail.py` | emails a red-and-white progress report | see its docstring |
| `bench_check.py` | starts bench mode and tests the whole graph | `pixi run bench-check` |
| `graph_picture.py` | draws the running graph, grouped by layer | `pixi run graph-picture` |
| `mcp_server.py`, `mcp_parts.py` | the read-only MCP server for AI apps, and its parts | an AI app, through `.mcp.json` |
| `legacy_firmware.sh` | builds today's ESP32 firmware with PlatformIO | `pixi run -e firmware legacy-esp32` |
| `tests/` | the tools' own tests: the MCP server's parts and its smoke test, the UF2 converter | inside `pixi run test` |
| `get_models.sh` | downloads the stock YOLO model, checksum pinned | `pixi run get-models` |
| `build_engine.py` | builds the Orin's TensorRT model, on the Orin itself (JetPack's Python) | `python3 tools/build_engine.py` on the Orin |
| `get_hud_map.sh` | downloads the HUD's place database, checksum pinned | `pixi run get-hud-map` |
| `get_flutter.sh` | downloads Flutter, checksum pinned | `pixi run get-flutter` |
| `flutter.sh` | runs that Flutter, with its analytics off | `pixi run flutter <arguments>` |
| `hud_test.sh`, `hud_fake_backend.py` | the HUD app's checks; its client code against the real back end, with a fake car | `pixi run hud-test` |
| `elf2uf2.py` | turns the Pico 2 program into a UF2 file | `pixi run spine-uf2` |
| `vendor.py` | downloads the source in `vendor/*.repos` | `pixi run vendor` |
| `udev/99-aav.rules` | stable device names on the car (`/dev/aav/imu`, ...) | install on the car's Linux computers |

## Launch profiles: the same nodes, a different car

Everything above the backend is the same in every profile. Only what plays the car
changes.

| Profile | Start it with | The car is | Runs |
|---|---|---|---|
| `turtle` | `pixi run turtle` | the turtlesim turtle | mux, ego, scripted driver, the car's model, `ekf_local` |
| `sim` | `pixi run sim` | Gazebo | the whole stack: both EKFs, `navsat_transform`, Nav2 |
| `car` | `pixi run car <machine>` | the real car | the parts its machine file switches on |
| `replay` | `ros2 launch aav_bringup replay.launch.py`, with a bag playing | a recorded drive | localization (and Nav2) on real sensor data |
| `bench` | `pixi run bench` | a model car, driven by the spine's own firmware logic | the car's whole graph: the spine bridge, both EKFs with GPS, Nav2, and pretend sensors |

## The computers, and what runs where

The car runs one ROS graph across several computers. Each one starts `pixi run car
<machine>`, and `src/aav_bringup/config/machines/<machine>.yaml` switches its parts on.

| Machine | Runs | Today |
|---|---|---|
| `orin` (Jetson AGX Orin) | the car's model, mux, ego, scripted driver, IMU, GPS, localization, Nav2, Foxglove, the detector (TensorRT) and the stop-sign watcher | Ubuntu 22.04 and Humble: it must move to Jazzy first (Q24) |
| `motor_pi` | the legacy bridges to today's ESP32 (UART) and the wheel encoder (UDP) | Humble |
| `laptop` | the safety driver's joystick | any laptop with Pixi |
| `all` | everything on one computer: for a bench test | |
| the spine | a microcontroller on CAN: throttle, brake, steering, watchdog | a Pico 2 program builds, never flashed; the car's chip is Q10–Q17 |

The development computers:

| Computer | Use |
|---|---|
| oirp-spss | Ben's AlmaLinux workstation: builds and tests headless (no screen) |
| Ben's Mac | development, rqt_graph, Gazebo, RViz; one day, driving and watching the car |
| team laptops | the same, with Pixi: [Your first day](first-day.md) |

## How a change goes from an idea to the car

| # | Step | Where |
|---|---|---|
| 1 | Change the contract first, if the change touches one | the contracts table above |
| 2 | Write the logic in a plain module, with tests | `src/<layer>/<package>/` |
| 3 | Keep the node thin, with a docstring of what it reads and writes | the same package |
| 4 | Register it: a launch profile, the mux (drivers), `tools/test.sh` (tests) | [Build a driver](howto-driver.md) |
| 5 | `pixi run test`, then the turtle, then `pixi run check sim` in Gazebo | [Simulation](simulation.md) |
| 6 | Update the docs page, then `pixi run docs` | `docs/` |
| 7 | Commit small, each commit passing the tests; inherited code gets a `PROVENANCE.md` line | `git` |
| 8 | On the car: stands first, then the ground at walking pace, with a safety driver | the test ladder |

## Sharing the code

The repo is on GitHub: `git@github.com:zander1976/aav.git`, private. Ask Ben to add you. The
previous teams' orgs are public, so nothing is pushed there.

| Task | Command |
|---|---|
| Get it | `git clone git@github.com:zander1976/aav.git aav` |
| Take newer work | `git pull` |
| Share your work | `pixi run test`, then `git push` |
| Hand it to a laptop with no GitHub access | `git bundle create ../aav.bundle HEAD main` (keep `HEAD`, or a clone checks out nothing), then `git clone aav.bundle aav` there |

CI (`.github/workflows/test.yml`) runs `pixi run test` on Linux for every push and pull
request, and on a Mac every night. The results are on the repo's Actions tab.

### Reading a CI failure without signing in

A job's log needs a GitHub sign-in. Its **annotations** don't: when the test step fails,
`tools/ci_annotate.py` prints one annotation per failing test, with its error message.

| Step | Open this (`curl -s` works too) |
|---|---|
| 1. Find the job | `https://api.github.com/repos/zander1976/aav/commits/<sha>/check-runs`: note the `linux` job's `id` |
| 2. Read its failures | `https://api.github.com/repos/zander1976/aav/check-runs/<id>/annotations` |

Without a sign-in GitHub allows 60 of these calls an hour, so don't poll fast. A step keeps
at most 10 annotations: 9 failing tests, then the rest by name, or the log's last 40 lines.

!!! warning "What never goes in git"
  Build output (`build/`, `install/`, `log/`, `.pixi/`, `target/`), recordings (`bags/`),
  and secrets. `.gitignore` lists them. Downloaded third-party source, such as ROS's own
  code and Flutter, lives in `vendor/`, which git doesn't track: `pixi run vendor`.

## Where to find what happened

| Want | Look in |
|---|---|
| Why something is the way it is | `docs/history/conversation.md`: the Q&A record, by topic, with its open threads |
| Ben's requests, word for word | `docs/history/messages-*.md` |
| The audits of the inherited code, the firmware and the live system | `docs/history/audit-2026-10-10-*.md` |
| What's next | `TODO.md` and [Work](work.md) |
| What changed, and why | `git log` |
