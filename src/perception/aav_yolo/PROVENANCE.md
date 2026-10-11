# Provenance

Copied on 2026-10-09 from github.com/Carleton-AAV-Capstone/camera_ros2_ws @ default branch (downloaded 2026-10-09), src/yolo_ros2.

Files, unchanged unless listed under Changes:

- `package.xml`
- `setup.cfg`
- `setup.py`
- `yolo_ros2/__init__.py`
- `yolo_ros2/yolo_inference.py`

`training/` came from a second repo: see `training/PROVENANCE.md`.

## Changes

- 2026-10-10: `yolo_ros2/yolo_inference.py` `.warn(` -> `.warning(` (deprecated in Jazzy).
- 2026-10-10: brought in from parked as `aav_yolo` (the name in docs/conventions.md).
  - The package, its folder and its module are renamed; `yolo_inference.py` is now
    `aav_yolo/detector.py`, and its executable `detector`.
  - `detector.py` is rewritten. It keeps the idea: camera images in, YOLO, results out.
    Changed:
    - the model file is a parameter, instead of `~/Yolo/Capstone/runs/detect/train7/weights/best.pt`;
    - it publishes `vision_msgs/Detection2DArray`, instead of only an annotated image;
    - the annotated image is still there, on `detections/image`, but no window opens
      (`cv2.imshow` is gone);
    - the confidence is a parameter, 0.5 by default. The old code used 0.81, tuned for its
      own model;
    - it keeps only the labels asked for, and picks the GPU: CUDA, else Metal, else the CPU.
  - New: `aav_yolo/boxes.py` (the logic, with tests), `launch/detector.launch.py`,
    `test/`, and the missing `resource/aav_yolo` marker.
  - `package.xml` and `setup.py` are rewritten with the real dependencies.
- 2026-10-10: `detector.py` can read the JPEG pictures (`compressed:=true`, for a laptop
  on WiFi). The device setting takes PyTorch's own names (`auto`, `cuda`, `cuda:N`, `mps`,
  `cpu`): the aliases (`gpu`, `metal`, `cuda` meaning Metal on a Mac) are gone, and a
  device the computer lacks is a clear error. The launch file takes the node's `name`, so
  each computer's detector has its own.
- 2026-10-11: new, our own: `aav_yolo/ground.py` (a pinhole camera over a flat road: a
  box's bottom edge to points on the road, with tests in `test/test_ground.py`) and the
  `camera_obstacles` node (`aav_yolo/camera_obstacles.py`, `config/camera_obstacles.yaml`),
  which publishes them for Nav2's local costmap. `setup.py` installs `config/`;
  `package.xml` adds `sensor_msgs_py` and `tf2_ros_py`.
- 2026-10-11: `launch/detector.launch.py` takes `use_sim_time` (false by default), so
  the detector runs on Gazebo's clock in `sim.launch.py perception:=true`.
- 2026-10-11: our own `aav_yolo/stop_sign.py`: `min_height_px` 55 by default, was 72. In
  Gazebo's road world a sign 2 m right of the lane left the picture before its box was
  72 px tall whenever Nav2 drove the car 0.5-0.7 m left of its line, so the car drove on.
