# How it fits together

Everything flows one way: sensors, then "where am I", then drivers, the mux, ego, and one
backend. The layers only talk through ROS topics, so any box can be swapped without
touching the others.

## The picture

![How the stack fits together](img/architecture.svg)

## 1. Sensors: raw data

| Sensor | Driver | Publishes |
|---|---|---|
| Wheel speed | today `encoder_udp` (in `aav_legacy`); later the spine | `/vehicle/odom` |
| IMU (Yahboom, USB) | `aav_imu_yahboom` (Rust), then the stock `imu_filter_madgwick` | `/imu/data_raw`, then `/imu/data` |
| GPS (u-blox, USB) | the stock `ublox_gps`; or `nmea_navsat_driver`, or `ublox_dgnss` on the car | `/gps/fix` |
| The car's shape | the stock `robot_state_publisher`, from `vehicle.yaml` | `/tf_static`: where each sensor sits |
| Lidar, cameras | next year | |

Each driver publishes a standard message in its own frame. Another IMU or GPS can replace
it without changing anything downstream.

## 2. Where am I: two filters

![The two EKFs](img/ekf.svg)

- **`ekf_local`** fuses the wheels and the IMU. It's smooth and never jumps, but it drifts
  slowly, like counting your steps. Use it to steer.
- **`ekf_global`** fuses the same inputs **plus GPS**. It stays near the truth, but can
  hop a little when a GPS fix arrives. Use it to plan.
- **`navsat_transform`** turns GPS latitude and longitude into map x and y for
  `ekf_global`.

!!! note "Does the global EKF read the local one?"
  In effect, yes. `ekf_global` publishes only the correction `map → odom`, worked out on
  top of `ekf_local`'s `odom → base_link`. Chained together, they give the car's true
  position, and the local estimate stays smooth.

## 3. Drivers propose, the mux picks

Every driver publishes a `TwistStamped` (speed in m/s, turn rate in rad/s) on its own
topic. None of them knows what kind of vehicle it's driving.

| Driver | Topic | Priority |
|---|---|---|
| Joystick: the safety driver | `/cmd_vel/joy` | 100 |
| Keyboard, or another remote | `/cmd_vel/remote` | 90 |
| Scripted: speed up, hold, slow down | `/cmd_vel/scripted` | 50 |
| Replay of a recorded drive | `/cmd_vel/replay` | 40 |
| Nav2: drive to a point on the map | `/cmd_vel/nav` | 20 |
| AI | `/cmd_vel/ai` | 10, later |

`twist_mux` passes on the highest priority that has spoken in the last 0.5 s, as
`/cmd_vel`. Publishing `true` on `/estop` blocks every driver until someone publishes
`false`.

Teleop is just another driver. The keyboard's commands take the same path as Nav2's:
mux, ego, then the turtle or the car.

## 4. ego: the virtual car

ego is the only node that knows the car is a car. Fifty times a second it turns `/cmd_vel`
into `/ego/actuator_cmd`: an `AckermannDriveStamped` with a speed and a steering angle
(positive is left).

| Step | What it does |
|---|---|
| Bicycle model | steering angle = atan(wheelbase × turn rate ÷ speed). A car can't turn on the spot, so at zero speed ego holds the wheels where they are. |
| Limits | 1.0 m/s top speed to start; acceleration; steering ±0.5 rad; steering rate |
| Timeout | no command for 0.3 s: brake to a stop |

## 5. One car model, everywhere

The car's numbers live in one file, `src/aav_description/config/vehicle.yaml`. The launch
files hand them to every part that needs them.

| Part | Uses |
|---|---|
| ego | the wheelbase and steering limit, for the bicycle model |
| URDF (`robot_state_publisher`) | where each sensor sits |
| Nav2 | the footprint, and the tightest turn: wheelbase ÷ tan(max steer) = 2.86 m |
| `turtlesim_adapter`, `gazebo_adapter` | the wheelbase, so the turtle and the simulated car turn like the car |
| `aav_spine`, `aav_legacy` | the wheelbase, track and wheel size |

Most of the numbers are still guesses: see [Car numbers](car_measurements.md).

## 6. Backends: one plays the car

| Backend | Package | Does |
|---|---|---|
| The real car | `aav_spine` | `/ego/actuator_cmd` into CAN frames, for the spine firmware |
| Today's car | `aav_legacy` | into UART frames, for the 2025-26 ESP32 firmware |
| turtlesim | `aav_turtlesim` | the stock turtlesim, steered with the bicycle model |
| Gazebo | `aav_gazebo` | a simulated car with an IMU and GPS, on the real car's topics |
| CARLA | `aav_carla` | a stub, parked (Q31) |

Each backend reports the car's speed back on `/vehicle/odom`. That closes the loop into
the EKFs.

## 7. Safety layers

From the person outward. Each layer works even if everything after it fails.

| # | Layer | Stops the car when |
|---|---|---|
| 1 | Joystick deadman (priority 100) | held with the sticks centred |
| 2 | `/estop` | it's true: the mux blocks every driver, and the car backends brake |
| 3 | ego's timeout | no command arrives for 0.3 s |
| 4 | `spine_bridge` | ego is quiet for 0.1 s: it stops sending drive frames |
| 5 | The spine's watchdog | no valid frame for 200 ms: throttle off, brake on |
| 6 | RC override switch, hardware e-stop, driver faults | always: handled in the firmware |

!!! warning "Today's car has no layer 5"
  The 2025-26 ESP32 firmware has no timeout. If the bridge dies, the last throttle stays
  on. Keep the wheels off the ground and the e-stop in hand: see [Firmware](firmware.md).
