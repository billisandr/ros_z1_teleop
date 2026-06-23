#!/usr/bin/env bash
# Checks/repairs VcXsrv, ZED 2 USB passthrough (usbipd-win + WSL2), then launches
# the Z1 hand-teleop station fed by the physical ZED 2, and auto-opens RViz once
# ROS is up.
#
# Run from Git Bash ON THE WINDOWS HOST (not inside WSL or the container).
# `usbipd bind` requires an elevated (Administrator) shell the first time a
# device is bound — re-run from an admin Git Bash if that step fails.
#
# Usage: bash start_teleop.sh [launch_alias]
#   launch_alias   Defaults to z1_teleop_zed (pass z1_teleop_zed_headless to skip the GUI)
#
# This script handles the ZED real-camera path specifically (USB passthrough is
# its whole reason to exist). For the hardware-free webcam/video path, just run
# the sim launch directly — no USB dance needed, e.g.:
#   docker run -it --rm -e DISPLAY=host.docker.internal:0.0 ros-z1-teleop \
#       bash -ic "z1_teleop image_source:=video:/path/to/hand.mp4"
#
# Background on each check: docs/DOCKER_CMDS.md — Diagnosing corrupted video
# over usbipd (Windows), and the README Troubleshooting table.

set -euo pipefail

# Git Bash auto-converts POSIX-looking arguments (/dev/video0, /dev/bus/usb/...)
# into Windows paths before handing them to native .exe tools — this corrupts
# every `wsl` and `docker --device` call below (e.g. `wsl -d docker-desktop --
# ls /dev/video0` silently becomes `ls C:/Program Files/Git/dev/video0`, which
# can cascade into wsl.exe falling back to launching a shell that doesn't
# exist in the minimal docker-desktop distro — "execvpe(/bin/bash) failed").
export MSYS_NO_PATHCONV=1

USBIPD="/c/Program Files/usbipd-win/usbipd.exe"
VCXSRV="/c/Program Files/VcXsrv/vcxsrv.exe"
IMAGE="ros-z1-teleop"
CONTAINER="ros-z1-teleop"
DEVICE_NAME="ZED 2"          # change to "RealSense" to adapt this script for the D435
LAUNCH_ALIAS="${1:-z1_teleop_zed}"

log() { echo "[start_teleop] $*"; }
die() { echo "[start_teleop] ERROR: $*" >&2; exit 1; }

# Run a command in the docker-desktop WSL distro.
# --cd / is REQUIRED: without it, wsl.exe tries to chdir into the translation of
# the current Windows working directory (e.g. E:\...\ros_z1_teleop), which the
# minimal docker-desktop distro cannot enter — it fails with
#   "chdir(...) failed 19" then "execvpe(/bin/bash) failed: No such file..."
# Forcing the cwd to / sidesteps that entirely.
dwsl() { wsl -d docker-desktop --cd / -- "$@"; }

# --- 0. Docker must be up ----------------------------------------------------
docker info >/dev/null 2>&1 || die "Docker Desktop doesn't appear to be running."

# --- 1. VcXsrv (X server) — must be running WITH -wgl ------------------------
CMDLINE="$(powershell.exe -NoProfile -Command \
    "(Get-CimInstance Win32_Process -Filter \"Name='vcxsrv.exe'\").CommandLine" \
    2>/dev/null | tr -d '\r')"

if [ -z "$CMDLINE" ]; then
    log "VcXsrv is not running — starting it with -wgl"
    NEEDS_START=1
elif ! grep -q -- "-wgl" <<< "$CMDLINE"; then
    log "VcXsrv is running WITHOUT -wgl (gzclient will crash under Mesa software rendering) — restarting it"
    powershell.exe -NoProfile -Command \
        "Get-Process vcxsrv -ErrorAction SilentlyContinue | Stop-Process -Force" >/dev/null
    sleep 1
    NEEDS_START=1
else
    log "VcXsrv already running with -wgl — OK"
    NEEDS_START=0
fi

if [ "$NEEDS_START" = "1" ]; then
    powershell.exe -NoProfile -Command \
        "Start-Process -FilePath '$VCXSRV' -ArgumentList ':0 -multiwindow -clipboard -ac -wgl'" >/dev/null
    sleep 2
    LISTENING="$(powershell.exe -NoProfile -Command \
        "if (Get-NetTCPConnection -LocalPort 6000 -State Listen -ErrorAction SilentlyContinue) { 'yes' } else { 'no' }" \
        2>/dev/null | tr -d '\r')"
    [ "$LISTENING" = "yes" ] || log "WARNING: nothing listening on port 6000 after starting VcXsrv — check it manually"
fi

# --- 2. USB passthrough: bind + attach the device to WSL2 -------------------
# Only match a real "Connected:" entry (busid like "2-3" at the start of the
# line) — `usbipd list` also has a "Persisted:" section keyed by a GUID for
# devices it remembers but that aren't currently plugged in. Matching that
# section too would feed a GUID into --busid below and fail confusingly
# several steps later instead of with a clear "not connected" error here.
USBIPD_LIST="$("$USBIPD" list)"
DEV_LINE="$(grep -E '^[0-9]+-[0-9]+ ' <<< "$USBIPD_LIST" | grep "$DEVICE_NAME" || true)"
[ -n "$DEV_LINE" ] || die "'$DEVICE_NAME' not found under 'Connected:' in 'usbipd list' — it's not currently plugged in (it may still show under 'Persisted:', which doesn't count). Check the USB cable/port."

BUSID="$(awk '{print $1}' <<< "$DEV_LINE")"

if grep -q "Attached" <<< "$DEV_LINE"; then
    log "$DEVICE_NAME (bus $BUSID) already attached to WSL2 — OK"
else
    log "$DEVICE_NAME (bus $BUSID) not attached — binding + attaching"

    BIND_OUTPUT="$("$USBIPD" bind --busid "$BUSID" 2>&1)" || true
    if grep -qi "access is denied\|administrator" <<< "$BIND_OUTPUT"; then
        die "'usbipd bind' needs an elevated (Administrator) shell. Re-run this script from an admin Git Bash."
    fi

    ATTACH_OUTPUT="$("$USBIPD" attach --wsl --busid "$BUSID" 2>&1)" || true
    echo "$ATTACH_OUTPUT"
    grep -qi "error" <<< "$ATTACH_OUTPUT" && die "'usbipd attach' failed — see output above."
fi

# --- 3. uvcvideo module + /dev/videoX in the docker-desktop WSL VM ----------
# Not persistent across Docker Desktop / WSL2 restarts — reload every run.
if ! dwsl test -e /dev/video0 2>/dev/null; then
    log "uvcvideo not loaded in docker-desktop VM — loading it"
    dwsl modprobe uvcvideo
fi
dwsl test -e /dev/video0 2>/dev/null \
    || die "/dev/video0 still missing after modprobe — check the USB attach state above."

# Bus number is reassigned on every usbipd attach — always read it live,
# never trust a number from a previous run.
USB_BUS="$(dwsl sh -c "lsusb | grep '$DEVICE_NAME'" | awk '{print $2}')"
[ -n "$USB_BUS" ] || die "'$DEVICE_NAME' not visible via lsusb inside docker-desktop."
dwsl test -d "/dev/bus/usb/$USB_BUS" 2>/dev/null \
    || die "/dev/bus/usb/$USB_BUS missing inside docker-desktop."

log "$DEVICE_NAME is on bus $USB_BUS inside docker-desktop, /dev/video0 present — OK"

# --- 4. Launch the station ---------------------------------------------------
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    log "Removing existing '$CONTAINER' container"
    docker rm -f "$CONTAINER" >/dev/null
fi

# Detached (-d), not -it: only one process can actually hold interactive
# terminal control at a time. Running the main container in the foreground
# while a backgrounded `docker exec -it ... z1_rviz` tried to grab the same
# host terminal for RViz meant the two fought over it and RViz never
# properly attached. Detaching the main container frees the terminal for the
# RViz exec below, which is now the script's real foreground step.
# Ctrl+C has nothing to signal once detached, so trap it to stop the
# container explicitly instead of leaving it running orphaned.
trap 'log "Stopping container..."; docker stop "$CONTAINER" >/dev/null 2>&1' EXIT INT TERM

log "Starting container, launching '$LAUNCH_ALIAS' (ZED 2 hand-teleop)"
docker run -d --rm \
    --name "$CONTAINER" \
    -e DISPLAY=host.docker.internal:0.0 \
    -p 8501:8501 \
    --device "/dev/bus/usb/$USB_BUS:/dev/bus/usb/$USB_BUS" \
    --device /dev/video0:/dev/video0 \
    --device /dev/video1:/dev/video1 \
    --user root \
    "$IMAGE" bash -ic "$LAUNCH_ALIAS" >/dev/null

# Open RViz once ROS is up inside the container (mirrors the manual
# "Terminal 2" step in the README Quick Start). Runs after the container
# above is already started, in the foreground — a real interactive
# docker exec, not backgrounded. --user root is required here too — without
# it this exec fails (the main process owns ~/.ros / ~/.rviz as root, since
# the docker run above also uses --user root for /dev/video0 access).
log "Waiting for ROS to come up before opening RViz..."
for _ in $(seq 1 60); do
    docker exec "$CONTAINER" rostopic list >/dev/null 2>&1 && break
    sleep 1
done
docker exec -it --user root -e DISPLAY=host.docker.internal:0.0 "$CONTAINER" bash -ic "z1_rviz"

# RViz closed — follow the station's own logs so the terminal stays
# attached to something meaningful; Ctrl+C here triggers the trap above.
log "RViz closed. Following container logs (Ctrl+C stops the station)..."
docker logs -f "$CONTAINER"
