# Bootstrap — provisioning a fresh Orin for this workspace

`scripts/bootstrap.sh` installs every piece of state that the `kilted` branch expects but that doesn't live in git: the ROS 2 Kilted apt source, workspace dependencies, udev rules for the Wheeltec CP2102 serial devices and the Orbbec camera, `rosdep`, a `~/.bashrc` source line, and the initial `colcon build`.

Target host: **Jetson AGX Orin, JetPack 7.2.x, Ubuntu 24.04 Noble, arm64.** The script will warn (not fail) on other Ubuntus / architectures.

## Run

```sh
git clone <repo> ~/.git/wheelbots
cd ~/.git/wheelbots
./scripts/bootstrap.sh
```

Flags:

| Flag | Effect |
|---|---|
| `--dry-run` | Print every action, change nothing. Good first pass. |
| `--no-build` | Skip the final `colcon build --symlink-install`. |
| `-h`, `--help` | Print header comment. |

Environment overrides:

| Variable | Default | Notes |
|---|---|---|
| `ROS_DISTRO` | `kilted` | Point at a different distro (Lyrical, when Jetson support catches up). |

The script is idempotent: running it twice is safe. It skips work that's already done and reports what it skipped.

## What it does

1. **Preflight** — checks `/etc/os-release` codename and `uname -m`; warns on mismatches, continues.
2. **ROS 2 apt source** — adds `packages.ros.org` and its GPG key if `ros2.list` / `ros2.sources` isn't already there.
3. **Apt packages** — installs 28 packages: ROS Kilted desktop + build tooling, workspace runtime deps that rosdep would otherwise pull, the camera fallback (`v4l2-camera`) and web preview (`web-video-server`), plus system libs (PCL, PCAP, FFmpeg, YAML, Eigen, gflags) and dev tools.
4. **rosdep init + update** — bootstraps rosdep, then refreshes the cache.
5. **udev rules** — installs `scripts/udev/99-wheeltec.rules` (STM32 + M10 symlinks) and, if the vendor tree is present, `vendor/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules` (Astra libusb permissions). Reloads udev and triggers.
6. **NetworkManager overrides** — installs `scripts/networkmanager/zz-wifi-powersave-off.conf` to `/etc/NetworkManager/conf.d/`, disabling WiFi power save. Removes 10–100 ms SSH keystroke wake-up latency; on this Orin it cut ping avg 4.3 → 2.3 ms and mdev 3.6 → 1.1 ms.
7. **`~/.bashrc`** — appends `source /opt/ros/kilted/setup.bash` unless the line is already there.
8. **Workspace rosdep** — resolves per-package.xml deps under `core/`, `mapping/`, `navigation/`, `utils/`, `vendor/`.
9. **`colcon build --symlink-install`** — full workspace build (unless `--no-build`).
10. **Summary** — prints "next steps" (source overlay, launch commands, diagnostic pointers).

## What it does NOT do

- **Does not flash JetPack.** Assumes JetPack 7.2.1 is already on the board.
- **Does not modify user group membership.** If you tighten the udev rules to `MODE:="0660"` with `GROUP:="dialout"`, you must `sudo usermod -aG dialout $USER` and log out/in.
- **Does not clean stale `~/.local/bin` shims.** If your Orin was cloned from an older SD card image with pip user-installs from Python 3.10, you may have hundreds of broken shims in `~/.local/bin` shadowing system tools (notably `cmake` and `pip`). Diagnose with `head -1 ~/.local/bin/cmake` — if it says `#!/usr/bin/python3.10`, they're broken. Simplest cleanup: `rm -rf ~/.local/bin` and reinstall whatever pip user-packages you actually use.
- **Does not port the OrbbecSDK v1 wrapper to Kilted.** Depth from the classic Astra Pro isn't supported by the current vendor tree; `wheeltec_camera_uvc.launch.py` runs the RGB-only fallback via `v4l2_camera`. See the note in `vendor/OrbbecSDK_ROS2/orbbec_camera/COLCON_IGNORE`.

## After it finishes

Verify the udev symlinks (only appear when the robot is plugged in):

```sh
ls -la /dev/wheeltec_controller /dev/wheeltec_laser
```

Source the workspace overlay and launch:

```sh
source ~/.git/wheelbots/install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

## Backup / port to another machine

The idea is that this script + the git tree is enough. To move the setup:

```sh
git clone <repo> && cd wheelbots && git checkout kilted
./scripts/bootstrap.sh
```

No manual apt/udev/rosdep steps needed.
