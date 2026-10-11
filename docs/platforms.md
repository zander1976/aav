# Platforms: Ubuntu, the car's computers, networking

Every computer runs ROS 2 Jazzy. On the car's computers that means Ubuntu 24.04 with the
official packages. On a Mac or any other Linux, use Pixi: see [Run it](run.md).

## Which computer gets Jazzy how

| Computer | OS | Jazzy from |
|---|---|---|
| Jetson AGX Orin | JetPack 7.2 (Ubuntu 24.04) | apt. JetPack 6 is Ubuntu 22.04, so reflash first, after a backup. To try it before then: Pixi, below. |
| Raspberry Pi 4 or 5 | Ubuntu 24.04 Server (arm64) | apt; or Pixi on today's OS, below |
| Linux laptop | Ubuntu 24.04 | apt, or Pixi |
| Mac | macOS 14 or newer | Pixi: [Your Mac](mac.md) |
| oirp-spss | AlmaLinux 9 | Pixi, with no admin rights |
| Windows | Windows 10 or 11 | WSL2 with Ubuntu 24.04 |

Every computer on one ROS network must run the same release: Humble and Jazzy don't mix.

## Ubuntu 24.04: install Jazzy

The official steps, from docs.ros.org ("Ubuntu (deb packages)"):

```bash
sudo apt install software-properties-common curl
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt full-upgrade -y
sudo apt install ros-jazzy-desktop ros-dev-tools     # ros-jazzy-ros-base on the Pis and the Orin
sudo rosdep init && rosdep update
```

Add `source /opt/ros/jazzy/setup.bash` to `~/.bashrc`. To check it, run
`ros2 run demo_nodes_cpp talker` in one terminal and `ros2 run demo_nodes_py listener` in
another.

## Ubuntu: build this workspace

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y    # every stock package we use
colcon build --symlink-install
source install/setup.bash                          # in every new terminal
```

- `--symlink-install` links the Python files and configs instead of copying them, so an
  edit only needs a relaunch. C++ and message changes still need a rebuild.
- `colcon build --packages-select aav_ego` builds just one package.
- After switching branches, delete `build/`, `install/` and `log/`, and rebuild.

## Networking: many computers, one ROS graph

ROS 2 has no master. Nodes find each other by multicast on the local network.

| # | Rule | Why |
|---|---|---|
| 1 | The same `ROS_DOMAIN_ID` (1–101) in every machine's `~/.bashrc` | the 2025-26 dashboard used 10 and its UI used 0, so they never met |
| 2 | One network that allows multicast, such as the car's own router | campus WiFi often blocks it; then list the computers in `ROS_STATIC_PEERS` |
| 3 | Keep `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` | set it to `LOCALHOST` to keep a laptop's experiments off the car |
| 4 | One middleware everywhere: Fast DDS, the default | different middlewares may not talk to each other |
| 5 | Clocks in sync: chrony, with the Orin as the time server | TF and the EKFs reject messages from a wrong clock |
| 6 | Virtual machines on bridged networking, not NAT | so the VM gets its own address on the car's network |

`ros2 doctor --report` and `ros2 multicast send` / `receive` diagnose most problems.

## On the car: what runs where

The design puts everything on the Orin. Until the spine exists, the Motor Pi still talks
to today's ESP32. The files in `src/aav_bringup/config/machines/` say what runs where:

| Machine file | Runs |
|---|---|
| `orin.yaml` | description, mux, ego, scripted, the IMU and GPS drivers, localization, Nav2, Foxglove; the YOLO detector (`yolo_detector_orin`, TensorRT) and the stop-sign watcher |
| `motor_pi.yaml` | `legacy_uart_bridge` and `encoder_udp`: the ESP32 is wired to it, and the encoder sends its packets to it |
| `laptop.yaml` | the joystick, on the safety driver's laptop. Its detector (`yolo_detector_laptop`, Core ML) is off until it reads compressed images |
| `all.yaml` | everything on one computer, for the bench. Its detector is `yolo_detector_all` |

On each computer, run `ros2 launch aav_bringup car.launch.py machine:=<its file>`, or
`pixi run car <its file>` with Pixi.

!!! warning "The first hardware test"
  Wheels off the ground, the e-stop in reach, and a low speed limit in `ego.yaml`. Check
  that positive steering turns left, that the throttle goes the right way, and that the
  car stops when you release the deadman and when you kill the bridge. Today's firmware
  fails the last one: see [Firmware](firmware.md).

### The parts

Each machine file turns parts on (`true`) or off. A `TODO` marks a part waiting for its
hardware: the cameras (Q22), the radar, or which USB port a device is on.

| Part | Starts |
|---|---|
| `description` | the car's frames: `robot_state_publisher` |
| `mux`, `ego` | `twist_mux`, then `ego` |
| `teleop` | the joystick, and its e-stop button |
| `scripted` | the scripted driver |
| `imu`, `gps`, `radar`, `battery` | the sensor drivers, on the computer each is plugged into |
| `camera` | the front camera: `usb_cam`, Linux only |
| `detector` | YOLO, as `yolo_detector_<machine>`, with the model the file names ([Models](perception.md#models)) |
| `stop_signs` | the stop-sign watcher: one per car, next to the mux |
| `radar_clusters` | the radar's clustering |
| `camera_obstacles` | the detectors' boxes as points on the road, for Nav2's costmap ([Costmaps](costmaps.md)) |
| `localization`, `navigation` | the two EKFs and `navsat_transform`; Nav2. Its costmap reads the `radar` and `camera_obstacles` parts on the same computer, and `collision_monitor` comes with the radar |
| `spine`, `legacy_uart`, `legacy_encoder` | the backends: the CAN spine, or today's ESP32 and wheel encoder |
| `foxglove` | `foxglove_bridge`, on port 8765 |

A part that can't run on that computer (no PyTorch, no model file, no `usb_cam`) is
skipped with a warning, and the rest starts.

### The arguments

They win over the machine file. With Pixi, put them after `--`:
`pixi run car orin -- gps:=false`.

| Argument | Does |
|---|---|
| `gps:=false` | no GPS anywhere: no driver, and localization without it |
| `gps_driver:=ublox` | which GPS driver: `ublox` (the default, Tentative: Q18), `nmea`, or `ublox_dgnss` (apt only, the car) |
| `map:=`, `start_x:=`, `start_y:=`, `start_yaw:=` | Nav2's map, and where the car starts on it |

## Try Jazzy on the car before the reflash

Pixi also runs on the car's ARM computers, on the Ubuntu 22.04 they have now. So the
team can try this stack on the car first, and reflash later. It resolves, but is
untested on the car.

1. Stop the 2025-26 software: it is Humble, and Humble and Jazzy don't mix.
2. On the Orin and the Motor Pi, install Pixi and copy the code, as on
   [Your Mac](mac.md) (steps 2 to 4). It needs a 64-bit OS: `uname -m` must say
   `aarch64`.
3. Run `pixi run car orin` on the Orin, and `pixi run car motor_pi` on the Motor Pi.

| | Pixi on today's Ubuntu 22.04 | Reflash to JetPack 7.2 |
|---|---|---|
| What changes on the car | one folder; delete it to undo | everything: back up first |
| Disk | about 7 GB on each computer: check the Pis | the new OS |
| Good for | trying the stack on the car | the long term: apt, and NVIDIA's ROS packages |

## Moving the Orin to JetPack 7.2

It's a reflash, not an upgrade:
1. **Back up** `/home/aavjetson` and every workspace first. It holds the only copy of the
   2025-26 YOLO detector and camera/LiDAR setup. Better still, take a full image with
   `l4t_backup_restore.sh`.
2. **Check** `cat /etc/nv_tegra_release` and the module's part number. Module
   699-13701-0005-502 has a reported flashing problem; -501 works.
3. **Flash** with SDK Manager from an Ubuntu 22.04 or 24.04 x86 PC, or use the dev kit's
   USB installer.
4. **Install Jazzy** as above. Move the Pis to Ubuntu 24.04 at the same time.

The reasons and the risks: [history/conversation.md](history/conversation.md#so-we-can-upgrade-the-orin-to-jazzy-jetpack-72-test-it-before-committing).
