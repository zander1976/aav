# The older code: what we have, and what's missing

The previous teams left 32 repos in two public GitHub orgs: `AAV-2025-2026` (the 2025-26
team, 19 repos) and `Carleton-AAV-Capstone` (the 2024-25 team, 13 repos). This page says
what each one is, what's already in `aav`, what's worth pulling in next, and what no
repo provides. Surveyed on 10 October 2026, from the 14 clones in `~/Code/aav`.

## In one picture

| | Repos |
|---|---|
| Already in `aav` (all or part) | AAV_cortex, Motor_Steering (its numbers), User-Interface, Wheel_Encoders, yahboom-IMU, Lane_Detection_ROS2, Nav_Radar; from 2024-25: Battery-Monitor-ROS, camera_ros2_ws, camera_Yolo_detection, cameras |
| Replaced by stock ROS packages | Navigation_PathPlanning_ObstacleAvoidance, Navigation-LocalPathGenerator-rf_cpp, Obstacle_Avoidance_logic, Odometry, Real-Time-OS, joy_teleop_ackerman |
| Worth a look before they're dropped | Navigation-LocalPathGenerator's OSRM parser; the HUD's place database; Motor-Control's `FinalRev` branch (the only code that reads steering feedback) |
| Empty | Navigation-BehaviorPlanning |
| Not surveyed yet | three 2025-26 repos and seven 2024-25 repos (the tables below) |

## The 2025-26 repos, one by one

| Repo | What it is | Language, hardware | State | In `aav`? | Pull in next? |
|---|---|---|---|---|---|
| AAV_cortex | the ESP32 actuator firmware (RC override, the Pi's UART), a Pico 2 status display, a Pi serial bridge | C++ (PlatformIO, Pico SDK), Python ROS 2; ESP32, Pico 2 | working, on branch `6-pico2-aav-firmware` | partly: `firmware/legacy/`, and its UART protocol in `aav_legacy` | the Pi-side display script, if the status display stays |
| Motor-Control | the 2024-25 micro-ROS motor firmware: a fork | C++, micro-ROS; ESP32 | inactive since 2025 | no | reference only: branch `FinalRev` reads the steering's position |
| Motor_Steering | a Pi speed controller and UART bridge, an ESP32 add-on, sims, MATLAB steering geometry | Python ROS 2, C++, MATLAB; Pi 5, ESP32 | prototype | partly: its geometry in `docs/reference/` and `vehicle.yaml` | its 250 ms link timeout idea, for today's ESP32 (but braking, not releasing) |
| Navigation-BehaviorPlanning | a README title | none | empty | no | no |
| Navigation-LocalPathGenerator | follows an OSRM route from GPS: the current step, a local path, a speed | Python ROS 2 | prototype; nothing feeds it | no | maybe its OSRM parser, to turn the HUD's route into Nav2 GPS waypoints |
| Navigation-LocalPathGenerator-rf_cpp | a C++ rewrite of the above | C++17 | prototype | no | no: Nav2 does this |
| Navigation_PathPlanning_ObstacleAvoidance | a GPS-goal grid planner (A*, D* Lite) with lidar obstacles | C++, PCL; Orin | prototype, with a crash bug | no | no: Nav2 does this |
| Obstacle_Avoidance_logic | a lidar costmap and braking-distance e-stop | one C++ file | can't build: no package files | no | no: Nav2's obstacle layer and collision monitor |
| Odometry | merges a SLAM pose and the IMU into `/odom` | Python ROS 2 | prototype; nothing publishes its input | no | no: robot_localization does this |
| Real-Time-OS | a QNX layer: command picking, a SQLite logger, body control, a QNX image for a Pi 4 | C, C++; Pi 4 on QNX 8 | prototype: its own README lists the watchdog, e-stop and CAN as not done | no | no: twist_mux, the spine's watchdog and `ros2 bag` cover it ("No QNX" is Tentative, Q23) |
| User-Interface | the in-car HUD: a Flutter app, a Flask ROS node, offline routing and place search | Dart/Flutter, Python ROS 2; the UI Pi | a working demo, on mock data | partly: `hud/`, `src/hud/maincar` | **its place database `ottawa.db` (51 MB)**: the ported app still needs it |
| Wheel_Encoders | a Pico 2 that counts wheel spokes and sends them over Ethernet | Arduino C++; Pico 2, W5500 | working demo | yes: `firmware/legacy/wheel_encoder_pico2`, `aav_legacy` | its 3D-printed case files |
| joy_teleop_ackerman | joystick to Ackermann commands, with deadman buttons | Python ROS 2 | working | no: stock teleop replaced it; its button map lives on in `aav_teleop` | no |
| yahboom-IMU | the Yahboom IMU driver | C++ ROS 2; Pi 4 | working | yes, rewritten in Rust: `src/sensors/aav_imu_yahboom` (the C++ copy went on 11 Oct) | no |

The five not cloned here:

| Repo | What GitHub says | Status |
|---|---|---|
| Lane_Detection_ROS2 | none | the controller is in `aav`: `src/perception/aav_lane_geometry`, a driver on `/cmd_vel/ai`. The lane finder isn't in the repo |
| Nav_Radar | none | in `aav`: `ti_ros2_driver`, and the clustering as `aav_object_detection` |
| AAV_dashboard | a stop-sign badge and bounding-box overlay | not surveyed |
| Speed | none | not surveyed |
| Navigation-GlobalMapRouter---Abandoned--- | none | abandoned |

## The 2024-25 repos

| Repo | What GitHub says | In `aav`? |
|---|---|---|
| GNSS | GNSS code for the car (u-blox, with RTK corrections) | surveyed in depth; replaced by the stock u-blox driver |
| Battery-Monitor-ROS | battery monitor ROS code | yes: `src/sensors/battery_monitor` |
| camera_ros2_ws | Python | yes: rewritten as `src/perception/aav_yolo` |
| camera_Yolo_detection | Python, 72 MB | 2 files: `aav_yolo/training`. The trained `best.pt` isn't in it |
| cameras | publishes camera feeds to ROS 2 | its 4 calibration files: `aav_sensors/config/cameras`, used by `cameras.launch.py` |
| AutoCtrl_uRos | C++ | Motor-Control is a fork of it |
| emergency-stop | a browser e-stop | not surveyed: it may answer Q6 |
| Braking | C++ | not surveyed |
| Simulation | Python (CARLA work) | not surveyed |
| Power-Management-Documentation-2024-25 | power documentation | not surveyed: it may answer Q8 |
| AAV_board | none | not surveyed |
| AAV_ROS_Interfaces | common ROS interfaces | not surveyed |
| ROS2_test | a test repo | no |

## What no repo provides

| Area | Missing | Where `aav` stands |
|---|---|---|
| Sensors | a GPS driver that anything runs; a lidar driver; camera streaming and the stop-sign detector (both only on the car's Orin); steering feedback (only 2024-25's `FinalRev`); a fine enough wheel speed; magnetometer calibration | GPS: the stock driver, model unknown (Q18). Lidar and cameras: next year |
| Localization | a URDF, TF, a working estimator | [[works]] the EKFs and `navsat_transform`, in sim |
| Navigation | any planner whose output reached the motors; the HUD's route into GPS waypoints; lidar or radar costmap inputs | [[works]] Nav2 in Gazebo |
| Drivers | anything beyond RC and the joystick | [[works]] scripted, keyboard; the joystick is [[untested]] |
| Ego | a command layer with limits and a timeout; calibrations from radians to the actuators' counts | [[works]] ego; the calibrations are TODO |
| The spine | any CAN code; a watchdog that brakes; feedback frames; the hardware e-stop in code | [[works]] the logic (`spine-core`); the board code is [[missing]] |
| HUD | real data: it only ever showed mock data | [[untested]] its port is still to do |
| Simulation | a whole-car simulator | [[works]] Gazebo |

!!! warning "Back up the car before any reflash"
  Some work exists only on the car's computers: the stop-sign detector, `camera_decode`,
  the three nodes in the 2025-26 team's rqt_graph photo, and the OSRM map data. Image the
  Orin and the Pis first.

## Red flags in the old repos

The repos are public. Deleting a file doesn't remove it from the history.

| What | Where (no values here) | What to do |
|---|---|---|
| A GitHub personal access token | User-Interface: `Project_Documents/UI_notes.txt` | revoke it (Q34) |
| Two Firebase service-account private keys | User-Interface: 7 JSON files under `Database/` and `MainCar/` | revoke them (Q34) |
| The team WiFi password | AAV_cortex and Motor-Control: several firmware files | change it (Q34) |
| Build output and binaries | User-Interface (`node_modules`, 24,438 files), AAV_cortex, Real-Time-OS (a 33 MB QNX image), Motor_Steering, joy_teleop_ackerman | don't copy them into `aav` |

`aav` and its history contain none of these secrets.

## Bringing more code in

| Rule | Why |
|---|---|
| Copy only source, never build output | the old repos committed `build/`, `install/`, `log/` and `node_modules` |
| Give it a `PROVENANCE.md`: the repo, branch and commit, and every change since | so anyone can trace it back |
| Prefer a stock ROS package when one does the job | most of the old bugs were in home-made versions of stock packages |
| Big data (like the 51 MB `ottawa.db`) gets a download step, not git | GitHub warns at 50 MB and refuses files over 100 MB |
| Check for secrets before the first commit | the old repos leaked three kinds |
