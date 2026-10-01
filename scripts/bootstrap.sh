#!/usr/bin/env bash
# Bootstrap the out-of-repo state needed to build and run the wheelbots
# workspace on a fresh Jetson AGX Orin running JetPack 7.2.x (Ubuntu 24.04
# Noble, arm64) with ROS 2 Kilted.
#
# Idempotent: safe to re-run. Requires sudo for apt / udev / rosdep init.
#
# Usage:
#   ./scripts/bootstrap.sh              # full bring-up
#   ./scripts/bootstrap.sh --no-build   # skip the colcon build step
#   ./scripts/bootstrap.sh --dry-run    # print what would be done
#
# What this does NOT do:
#   - Flash JetPack (assumes JetPack 7.2.1 already installed).
#   - Modify user group membership (see comment inside if you want to tighten
#     serial device permissions).
#   - Install python-pip / ~/.local/bin hygiene (documented as an aside).

set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-kilted}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRY_RUN=0
DO_BUILD=1

for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        --no-build) DO_BUILD=0 ;;
        -h|--help)
            sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's|^# \{0,1\}||'
            exit 0
            ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

log()  { printf '\033[36m[bootstrap]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[bootstrap]\033[0m %s\n' "$*" >&2; }
run() {
    if [[ "$DRY_RUN" == 1 ]]; then
        printf '\033[90m[dry-run]\033[0m %s\n' "$*"
    else
        eval "$@"
    fi
}

# --- 0. Preflight -----------------------------------------------------------

log "repo: $REPO_DIR"
log "target ROS distro: $ROS_DISTRO"

if ! grep -q '^VERSION_CODENAME=noble' /etc/os-release 2>/dev/null; then
    warn "This is tuned for Ubuntu 24.04 (noble). /etc/os-release says:"
    grep '^\(NAME\|VERSION\)' /etc/os-release || true
    warn "Continuing anyway; some apt package names may differ."
fi

if [[ "$(uname -m)" != "aarch64" ]]; then
    warn "arch is $(uname -m); rules and vendored .so files target aarch64."
fi

# --- 1. ROS 2 apt source ----------------------------------------------------

if [[ ! -f /etc/apt/sources.list.d/ros2.list && ! -f /etc/apt/sources.list.d/ros2.sources ]]; then
    log "installing ROS 2 apt source"
    run "sudo apt-get update"
    run "sudo apt-get install -y ca-certificates curl gnupg lsb-release software-properties-common"
    run "sudo add-apt-repository -y universe"
    run "sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
    run "echo 'deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main' | sudo tee /etc/apt/sources.list.d/ros2.list"
else
    log "ROS 2 apt source already present, skipping"
fi

run "sudo apt-get update"

# --- 2. Apt packages --------------------------------------------------------

APT_PACKAGES=(
    # ROS 2 base + tooling
    "ros-${ROS_DISTRO}-desktop"
    "python3-colcon-common-extensions"
    "python3-rosdep"
    "python3-vcstool"

    # Workspace runtime deps that rosdep would otherwise pull
    "ros-${ROS_DISTRO}-nav2-bringup"
    "ros-${ROS_DISTRO}-slam-toolbox"
    "ros-${ROS_DISTRO}-robot-localization"
    "ros-${ROS_DISTRO}-ackermann-msgs"
    "ros-${ROS_DISTRO}-diagnostic-updater"
    "ros-${ROS_DISTRO}-image-publisher"
    "ros-${ROS_DISTRO}-nav2-msgs"
    "ros-${ROS_DISTRO}-joint-state-publisher"
    "ros-${ROS_DISTRO}-filters"
    "ros-${ROS_DISTRO}-async-web-server-cpp"

    # Camera fallback (v4l2 UVC for classic Astra Pro RGB) + web preview
    "ros-${ROS_DISTRO}-v4l2-camera"
    "ros-${ROS_DISTRO}-web-video-server"

    # System libs and dev tools
    "libpcl-dev"
    "libpcap0.8-dev"
    "libavcodec-dev"
    "libavformat-dev"
    "libavutil-dev"
    "libswscale-dev"
    "libyaml-cpp-dev"
    "libeigen3-dev"
    "libssl-dev"
    "libgflags-dev"
    "v4l-utils"
    "git-lfs"
)

log "installing ${#APT_PACKAGES[@]} core apt packages (skipping any already installed)"
run "sudo apt-get install -y --no-install-recommends ${APT_PACKAGES[*]}"

# Front-end connectivity: Foxglove Studio (foxglove_bridge) and the iPhone
# Wheeltec teleop app (rosbridge_server). Nice to have but not core to
# building or running the robot, so install best-effort — if the arm64
# kilted release lags, don't abort the whole bootstrap.
OPTIONAL_FRONTEND_PACKAGES=(
    "ros-${ROS_DISTRO}-foxglove-bridge"
    "ros-${ROS_DISTRO}-rosbridge-server"
)
log "installing ${#OPTIONAL_FRONTEND_PACKAGES[@]} optional front-end packages (best-effort)"
if [[ "$DRY_RUN" == 1 ]]; then
    printf '\033[90m[dry-run]\033[0m %s\n' "sudo apt-get install -y --no-install-recommends ${OPTIONAL_FRONTEND_PACKAGES[*]}"
else
    if ! sudo apt-get install -y --no-install-recommends "${OPTIONAL_FRONTEND_PACKAGES[@]}"; then
        warn "front-end packages not installed — foxglove_bridge / rosbridge_server may not be released for kilted on arm64 yet"
        warn "workspace still usable; retry when the packages land, or install from source"
    fi
fi

# --- 3. rosdep init + update ------------------------------------------------

if [[ ! -d /etc/ros/rosdep/sources.list.d ]] || ! ls /etc/ros/rosdep/sources.list.d/*.list >/dev/null 2>&1; then
    log "rosdep init"
    run "sudo rosdep init"
else
    log "rosdep already initialised, skipping init"
fi
run "rosdep update"

# --- 4. udev rules ----------------------------------------------------------

WHEELTEC_RULE="$REPO_DIR/scripts/udev/99-wheeltec.rules"
ORBBEC_RULE="$REPO_DIR/vendor/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules"

install_rule() {
    local src="$1" dst="$2"
    if [[ ! -f "$src" ]]; then
        warn "expected udev rule missing: $src"
        return 1
    fi
    if [[ -f "$dst" ]] && cmp -s "$src" "$dst"; then
        log "udev rule already up to date: $dst"
        return 0
    fi
    log "installing udev rule: $dst"
    run "sudo install -m 0644 $src $dst"
}

install_rule "$WHEELTEC_RULE" /etc/udev/rules.d/99-wheeltec.rules
install_rule "$ORBBEC_RULE" /etc/udev/rules.d/99-obsensor-libusb.rules || \
    warn "OrbbecSDK vendor tree not present — skipping Astra libusb rule."

run "sudo udevadm control --reload-rules"
run "sudo udevadm trigger"

# --- 5. NetworkManager overrides --------------------------------------------

# WiFi power save adds tens of ms wake-up latency to small SSH/ROS bursts.
# Ship a conf.d override that outranks Ubuntu's shipped default-wifi-powersave-on.conf.
NM_OVERRIDE_SRC="$REPO_DIR/scripts/networkmanager/zz-wifi-powersave-off.conf"
NM_OVERRIDE_DST="/etc/NetworkManager/conf.d/zz-wifi-powersave-off.conf"

if [[ -f "$NM_OVERRIDE_SRC" && -d /etc/NetworkManager/conf.d ]]; then
    if [[ -f "$NM_OVERRIDE_DST" ]] && cmp -s "$NM_OVERRIDE_SRC" "$NM_OVERRIDE_DST"; then
        log "NetworkManager override already up to date: $NM_OVERRIDE_DST"
    else
        log "installing NetworkManager override: $NM_OVERRIDE_DST"
        run "sudo install -m 0644 $NM_OVERRIDE_SRC $NM_OVERRIDE_DST"
        run "sudo systemctl reload NetworkManager"
    fi
else
    log "NetworkManager not present or source missing; skipping WiFi PSM override"
fi

# --- 6. .bashrc source line -------------------------------------------------

BASHRC_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
if ! grep -qF "$BASHRC_LINE" "$HOME/.bashrc" 2>/dev/null; then
    log "adding '$BASHRC_LINE' to ~/.bashrc"
    if [[ "$DRY_RUN" != 1 ]]; then
        echo "$BASHRC_LINE" >> "$HOME/.bashrc"
    fi
else
    log "~/.bashrc already sources /opt/ros/${ROS_DISTRO}/setup.bash"
fi

# --- 7. Workspace rosdep + build --------------------------------------------

# ROS's setup.bash references vars that may be unset; nounset would abort.
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

log "rosdep install for workspace"
run "rosdep install --from-paths $REPO_DIR/core $REPO_DIR/mapping $REPO_DIR/navigation $REPO_DIR/utils $REPO_DIR/vendor --ignore-src --rosdistro ${ROS_DISTRO} -y"

if [[ "$DO_BUILD" == 1 ]]; then
    log "colcon build (symlink-install)"
    run "cd $REPO_DIR && colcon build --symlink-install"
else
    log "--no-build set; skipping colcon build"
fi

# --- 8. Summary -------------------------------------------------------------

cat <<EOF

$(printf '\033[32m[bootstrap]\033[0m done.')

Next steps:

  1. Verify serial symlinks (after physically connecting the robot):
       ls -la /dev/wheeltec_controller /dev/wheeltec_laser

  2. Source the workspace overlay in this shell:
       source $REPO_DIR/install/setup.bash

  3. Launch the full mini_akm stack:
       ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

  4. In another terminal, launch SLAM against the M10:
       ros2 launch wheeltec_slam_toolbox online_async_launch.py

Diagnostic aids:

  - Camera preview (headless):  ros2 run web_video_server web_video_server
                                open http://<orin-ip>:8080/stream_viewer?topic=/image_raw
  - TF tree:                    see docs/tf_tree.md
  - Serve local files:          see docs/local_http_view.md

EOF
