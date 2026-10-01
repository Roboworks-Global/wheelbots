# Odometry calibration for the mini_akm

## Observation

The Wheeltec STM32 firmware ships with an encoder-tick / wheel-radius calibration that produces a `nav_msgs/Odometry` stream on `/odom` — but on this mini_akm the integrated distance **overcounts by ~65 %** relative to ground truth. Empirical measurement on 2026-09-29:

| Segment | Real (tape) | Odom Δx | Ratio (real / odom) |
|---|---|---|---|
| Forward drive at 0.05 m/s | ~0.91 m (3 ft) | 1.53 m | 0.60 |
| Reverse drive at 0.10 m/s | ~0.93 m (measured) | 1.56 m | 0.60 |

Consistent ~0.60× real / odom factor. Root cause is in the STM32 firmware (closed-source Wheeltec) — probably wrong `WHEEL_DIAMETER` or `ENCODER_RESOLUTION` constants for the MD36N motors + this mini_akm's wheels. Not fixable from ROS.

## Workaround: `odom_scaler` node

`core/turn_on_wheeltec_robot/scripts/odom_scaler.py` is a small standalone Python node that reads `/odom` and republishes on `/odom_scaled` with `pose.position` and `twist.linear` multiplied by a `scale_factor` param (default `0.60`). Orientation and angular velocity pass through unchanged.

Not wired into any default launch — it changes the odom used by downstream navigation, and the correct factor is platform-specific.

### Run standalone

```sh
ros2 run turn_on_wheeltec_robot odom_scaler.py \
    --ros-args -p scale_factor:=0.60
```

Then in another shell:

```sh
ros2 topic hz /odom_scaled          # should mirror /odom rate
ros2 topic echo /odom_scaled | head # confirm scaled values
```

### Wire into the stack

Two places need the swap for full-stack use:

1. **`core/turn_on_wheeltec_robot/config/ekf.yaml`** — change `odom0: odom` to `odom0: odom_scaled` so the EKF integrates the corrected velocity.
2. **`navigation/wheeltec_robot_nav2/param/wheeltec_param/param_mini_akm.yaml`** — the `odom_topic` for `bt_navigator` and Nav2 costmaps is already `/odometry/filtered` (EKF output), so no change needed as long as the EKF is using the scaled input.

Then add the scaler to `turn_on_wheeltec_robot.launch.py` right after the base_serial include:

```python
odom_scaler = launch_ros.actions.Node(
    package='turn_on_wheeltec_robot',
    executable='odom_scaler.py',
    name='odom_scaler',
    parameters=[{'scale_factor': 0.60}],
)
ld.add_action(odom_scaler)
```

## Re-measuring the factor

Whenever a wheel is swapped or the STM32 firmware is updated, re-measure:

1. Put the robot on a flat, non-slick surface. Mark the front-wheel contact point on the floor.
2. Snapshot `/odom` — record `pose.position.x`.
3. Command a slow forward for a fixed duration: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" -r 10` for ~10 s, then a zero-Twist stop.
4. Wait 2-3 s for the STM32 to finish coasting.
5. Snapshot `/odom` again. Compute `Δx_odom`.
6. Tape-measure the actual translation on the floor. Compute `Δx_real`.
7. `scale_factor = Δx_real / Δx_odom`. Update `odom_scaler.py`'s default or the launch parameter.

## Caveats

- **Only translation is scaled.** Yaw / angular velocity come from the IMU + wheel-differential integration in the STM32; those have their own error sources and are not touched here.
- **Stop latency (~2-4 s of coast at 0.10 m/s)** is a separate issue and not addressed by odom scaling. Nav2 tuning (or an explicit deceleration ramp before goal reach) needs to account for it.
- **Best long-term fix:** patch the STM32 firmware constants. Requires access to Wheeltec's firmware source, which we do not have on this branch. Until then the scaler is a workable band-aid.
