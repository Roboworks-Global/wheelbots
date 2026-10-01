# TF Tree — mini_akm

Expected tf2 frame tree when the full `turn_on_wheeltec_robot.launch.py` stack is running against the physical robot. `slam_toolbox` extends the tree with a `map → odom` transform once it starts consuming `/scan`.

```
map                             (published by slam_toolbox when mapping)
└── odom                        (world-fixed, drifts with the base)
    └── base_footprint          ← wheeltec_robot_node + ekf_filter_node, ~20 Hz
        ├── base_link           ← static, base_to_link
        ├── laser_link          ← static, base_to_laser  (M10 scan frame)
        ├── camera_link         ← static, base_to_camera (Astra RGB frame)
        └── gyro_link           ← static, base_to_gyro
```

## Broadcasters

| Edge | Broadcaster | Rate | Source |
|---|---|---|---|
| `map → odom` | `slam_toolbox` (async_slam_toolbox_node) | 20 Hz (`transform_publish_period: 0.02`) | `mapping/wheeltec_slam_toolbox/config/mapper_params_online_async.yaml` |
| `odom → base_footprint` | `wheeltec_robot_node` and reinforced by `ekf_filter_node` (robot_localization) | ~20 Hz | `core/turn_on_wheeltec_robot/src/wheeltec_robot.cpp` + `config/ekf.yaml` |
| `base_footprint → {base_link, laser_link, camera_link, gyro_link}` | 4× `static_transform_publisher` | static | `core/turn_on_wheeltec_robot/launch/robot_mode_description.launch.py` |

The static `base_footprint → laser_link` and `base_footprint → camera_link` offsets are per-platform; the mini_akm values are at `robot_mode_description.launch.py:23-28`.

## Regenerate the live tree

With the full stack running (`ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py`):

```bash
cd /tmp   # or any writable dir; the tool writes frames_<timestamp>.pdf + .gv here
ros2 run tf2_tools view_frames
```

Options:

- `ros2 run tf2_ros tf2_echo odom base_footprint` — watch a single edge stream numerically.
- `ros2 run rqt_tf_tree rqt_tf_tree` — live GUI viewer (needs a display).

## What "unhealthy" looks like

- **Only `odom → base_footprint`** — the base is up but `robot_mode_description.launch.py` didn't run. Missing static TFs. Fix: launch the full stack, not just `base_serial.launch.py`.
- **No `map`** — slam_toolbox is up but has no `/scan` yet (M10 not spinning, or hardware fault) OR no `/odometry/filtered` reaching it.
- **Extra frames from another variant** — active platform in `robot_mode_description.launch.py` was accidentally set to something other than `mini_akm` (bottom of the file).
