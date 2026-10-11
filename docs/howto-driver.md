# Build a driver

A driver is anything that proposes how to move: a joystick, a script, Nav2, one day a
learned model. Every driver publishes the same kind of message on its own topic, and
`twist_mux` picks one. This page builds a small example driver, `aav_circle`, from an
empty folder to a turtle driving in circles.

## What a driver does, and doesn't

| A driver does | A driver doesn't |
|---|---|
| publish `geometry_msgs/TwistStamped` on `/cmd_vel/<name>` | publish `/cmd_vel` or `/ego/actuator_cmd` |
| put the speed (m/s) in `twist.linear.x` and the turn rate (rad/s) in `twist.angular.z` | work out steering angles: that's ego's job |
| use the frame `base_link`: x forward, y left, positive turn rate = left | talk to hardware: that's a backend's job |
| go quiet when it has nothing to say | handle the e-stop: the mux and the backends do |

Because every driver speaks the same message, ego and the backends never change when a
driver is added.

## The priorities today

`twist_mux` passes on the highest priority that has spoken in the last 0.5 s.

| Input | Topic | Priority |
|---|---|---|
| E-stop (a lock: blocks everything below it) | `/estop` | 255 |
| Joystick, the safety driver | `/cmd_vel/joy` | 100 |
| Keyboard or another remote | `/cmd_vel/remote` | 90 |
| Scripted | `/cmd_vel/scripted` | 50 |
| Replay | `/cmd_vel/replay` | 40 |
| Nav2 | `/cmd_vel/nav` | 20 |
| AI | `/cmd_vel/ai` | 10 |

**People outrank software.** Give a new software driver a priority below 90. The example
uses 30.

## 1. Make the package

ROS makes the skeleton for you:

```bash
pixi shell                     # a terminal with ROS in it
cd src/drivers
ros2 pkg create aav_circle --build-type ament_python \
  --dependencies rclpy geometry_msgs --node-name circle_driver
```

That makes `src/drivers/aav_circle/` with `package.xml`, `setup.py`, a node file and
style tests. It also prints a list of licences: that's fine, the team hasn't picked one
yet. The layout rules are in [Conventions](conventions.md).

## 2. Write the node

Replace `src/drivers/aav_circle/aav_circle/circle_driver.py` with:

```python
"""
circle_driver: drive a slow left circle, to show how a driver works.

Publishes   cmd_vel/circle   geometry_msgs/TwistStamped, at 20 Hz
"""

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CircleDriver(Node):

    def __init__(self):
        super().__init__('circle_driver')
        self.speed = self.declare_parameter('speed', 0.5).value          # m/s
        self.turn_rate = self.declare_parameter('turn_rate', 0.2).value  # rad/s
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel/circle', 10)
        self.create_timer(0.05, self.tick)                               # 20 Hz

    def tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.speed
        msg.twist.angular.z = self.turn_rate
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

A real driver has a decision to make: when to speed up, where to turn. Put that logic in
a plain module with no ROS imports, and test it. For example, `aav_scripted` keeps its
logic in `speed_profile.py`, tested by `test/test_speed_profile.py`. Then the node stays
thin.

## 3. Register it

Four files learn about the new driver:

| File | Add |
|---|---|
| `src/mux/aav_mux/config/twist_mux.yaml` | an input under `topics:`, shown below |
| `docs/interfaces.yaml` | the topic, shown below. Then run `pixi run python tools/gen_interfaces_md.py > docs/interfaces.md` |
| `src/aav_bringup/launch/turtle.launch.py` (and `sim`, and the car's machine files) | a line that starts it, once it has a launch file |
| `tools/test.sh` | the package's folder, once it has unit tests |

The mux input:

```yaml
      circle:                   # aav_circle: an example driver
        topic: /cmd_vel/circle
        timeout: 0.5
        priority: 30
```

The contract entry:

```yaml
  /cmd_vel/circle:
    type: geometry_msgs/msg/TwistStamped
    pub: [circle_driver]
    sub: [twist_mux]
    note: Priority 30. An example driver that drives in circles.
```

## 4. Try it

```bash
pixi run build
pixi run turtle                                            # terminal 1
pixi run tools/ros.sh ros2 run aav_circle circle_driver    # terminal 2
pixi run graph                                             # terminal 3
```

**You should see:**
- the turtle drives a slow circle;
- rqt_graph shows `circle_driver` → `/cmd_vel/circle` → `twist_mux`;
- `pixi run keyboard` in another terminal: hold `i`, and the keyboard (90) takes over at
  once. Let go, and the circle comes back after half a second;
- `pixi run estop` stops everything, and `pixi run estop-clear` lets it go again.

Then `pixi run check turtle` should still say `0 failed`, and Gazebo is the next rung:
`pixi run sim`.

## 5. Before you commit

| Check | Command |
|---|---|
| Every test still passes | `pixi run test` |
| ROS's style rules | `pixi run lint` |
| The topics page matches the contract | part of `pixi run test` |
| Small commits, each saying why | `git commit` |

## Safety rules

1. **People outrank software:** priorities below 90.
2. **Go quiet when you're done:** stop publishing, and the mux hands control to the next
   driver after 0.5 s.
3. **Never publish `/cmd_vel` or `/ego/actuator_cmd` yourself.** Everything goes through
   the mux and ego, so the limits and the e-stop always apply.
4. **Climb the test ladder:** the turtle, then Gazebo, then the car on stands, then the
   car with a safety driver. See [Simulation](simulation.md#the-test-ladder).

## What's been tested

On a Mac, 10 October, in a copy of the repo: every step on this page as written.

| Check | Result |
|---|---|
| `ros2 pkg create`, then the node, the mux input and the contract entry from this page | [[works]] 20 packages build |
| The driver alone | [[works]] the turtle circles left |
| A priority-90 command on `/cmd_vel/remote` (what the keyboard sends) | [[works]] it takes over and drives straight; the circle returns after it stops |
| `pixi run estop`, then `pixi run estop-clear` | [[works]] the turtle stops, then drives again |
| `pixi run check turtle` | [[works]] `/cmd_vel/circle` ok, 0 failed |

## A sensor driver is different

A sensor driver turns a device into a standard ROS message: an IMU into
`sensor_msgs/Imu`, a GPS into `sensor_msgs/NavSatFix`. It publishes in its own frame
(`imu_link`, `gps_link`), and the car's model says where that frame sits.

| Rule | Why |
|---|---|
| Publish the standard message, with honest covariances | robot_localization reads all-zero covariances as a perfect sensor |
| Keep the topic names | another driver for the same kind of sensor can replace it without changing anything downstream |
| Pins, addresses and calibration stay in the firmware | ROS never sees a pin |

The IMU shows the pattern: `aav_imu_yahboom` (Rust) replaced the 2025-26 C++ driver. It
takes the same config and publishes the same topics, so nothing downstream changed.
