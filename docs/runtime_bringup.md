# Full-stack runtime bring-up (mini_akm)

The exact commands to bring up every ROS 2 subsystem currently running against the physical robot. Assumes `scripts/bootstrap.sh` has been run and the workspace is built. Each command block is one terminal — open five, or background them in one.

## Preflight (once per shell)

```sh
source /opt/ros/kilted/setup.bash
source ~/.git/wheelbots/install/setup.bash
```

Verify hardware is present:

```sh
ls -la /dev/wheeltec_controller /dev/wheeltec_laser  # both must exist
lsusb | grep -iE "orbbec|silicon"                    # 2× CP2102 + Orbbec RGB
```

## 1. Base + TF tree + EKF

```sh
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

Brings up: `/wheeltec_robot` (STM32), `/cmd_vel_to_ackermann_drive`, `/ekf_filter_node`, `joint_state_publisher`, four `static_transform_publisher` nodes (`base_to_link`, `base_to_laser`, `base_to_camera`, `base_to_gyro`).

Publishes: `/odom` @ 20 Hz, `/mobile_base/sensors/imu_data` @ 20 Hz, `/odometry/filtered` @ 20 Hz, `/PowerVoltage`, `/tf` (`odom → base_footprint`), `/tf_static` (base_footprint → laser_link, camera_link, base_link, gyro_link).

## 2. RPLIDAR A1M8

```sh
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py
```

Wraps upstream `sllidar_a1_launch.py` with `serial_port=/dev/wheeltec_laser`, `frame_id=laser_link`.

Publishes: `/scan` @ ~7 Hz, 1080 rays over 360°, range 0.05–12.0 m.

## 3. Camera (v4l2 UVC RGB fallback)

```sh
ros2 launch turn_on_wheeltec_robot wheeltec_camera_uvc.launch.py
```

Publishes: `/image_raw` YUYV 640×480 @ 30 Hz, `/camera_info`. Depth from the classic Astra Pro is not available on this path (Orbbec SDK v1 not yet ported to Kilted).

## 4. slam_toolbox

```sh
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args \
    --params-file ~/.git/wheelbots/install/wheeltec_slam_toolbox/share/wheeltec_slam_toolbox/config/mapper_params_online_async.yaml \
    -r odom:=odometry/filtered
```

slam_toolbox is a `LifecycleNode`. `ros2 run` starts it in the **unconfigured** state — it won't publish `/map` or the `map → odom` TF until you transition it. In another shell:

```sh
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

Once activated, publishes: `/map` (occupancy grid) every 5 s, `/map_metadata`, `/slam_toolbox/{feedback,graph_visualization,scan_visualization,update}`, and the `map → odom` TF.

## 5. Web preview (optional, headless-friendly)

```sh
ros2 run web_video_server web_video_server
```

Listens on `:8080`.

- Camera stream (embedded HTML player): http://<orin-ip>:8080/stream_viewer?topic=/image_raw
- Topic index: http://<orin-ip>:8080/

Use `ip -brief addr show` if you don't remember the Orin's IP.

## Verifying everything is healthy

```sh
ros2 node list
ros2 topic hz /odom                          # 20 Hz
ros2 topic hz /odometry/filtered             # 20 Hz
ros2 topic hz /scan                          # 7–10 Hz
ros2 topic hz /image_raw                     # 30 Hz
ros2 lifecycle get /slam_toolbox             # active [3]
ros2 topic list | grep -E 'map|slam'         # should include /map
ros2 run tf2_ros tf2_echo map odom           # non-fatal wait once mapping runs
```

For a full TF picture see [`docs/tf_tree.md`](tf_tree.md).

## Shutting down cleanly

Ctrl-C each `ros2 launch`/`ros2 run` shell. Order doesn't matter — the driver processes handle SIGTERM. Or, if backgrounded:

```sh
pkill -f "ros2 launch|sllidar_node|v4l2_camera_node|async_slam_toolbox_node|web_video_server|wheeltec_robot_node"
```

Full system power-off:

```sh
sudo shutdown -h now
```

## Backgrounded / one-shell variant (for scripts or SSH sessions)

If you want the same stack from a single shell (staggered so each stage settles before the next), see the pattern used in the transcript log at `/tmp/claude-.../scratchpad/*.log`. Not committed as a script yet — worth doing next if this becomes a routine bring-up. Sketch:

```sh
(ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py &) ; sleep 3
(ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py &)         ; sleep 2
(ros2 launch turn_on_wheeltec_robot wheeltec_camera_uvc.launch.py &)    ; sleep 2
(ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
     --params-file ~/.git/wheelbots/install/wheeltec_slam_toolbox/share/wheeltec_slam_toolbox/config/mapper_params_online_async.yaml \
     -r odom:=odometry/filtered &)                                       ; sleep 4
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 run web_video_server web_video_server &
```

## Known gotchas

- **slam_toolbox stuck at `unconfigured`** — you forgot the two `ros2 lifecycle set` commands after `ros2 run`. Vendor `online_async_launch.py` would do this for you but it also tries to launch the base + lidar, colliding with what's already running.
- **`/scan` shows no data** — LiDAR motor not spinning. Check that USB power is reaching it (see the RPLIDAR troubleshooting thread in the project memory).
- **Camera browser preview blank** — `/image_raw` isn't publishing; check `ros2 topic hz /image_raw`. Common cause: another process holds `/dev/video0`.
- **`odom → base_footprint` drifting when the robot is stationary** — likely wheel-encoder noise being integrated; not fatal but worth eyeballing tuning of `ekf.yaml` covariances later.
