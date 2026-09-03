# Docker Commands Reference

## Inspect

```bash
# List images
docker images

# List containers (running)
docker ps

# List all containers (including stopped)
docker ps -a

# Show resource usage of running containers
docker stats
docker stats <container-name>
```

## Build

```bash
# Build an image from the Dockerfile in the current directory
docker build -t <image-name> .

# Build with a specific Dockerfile
docker build -f <Dockerfile-path> -t <image-name> .

# Build without using cache
docker build --no-cache -t <image-name> .
```

## Run

```bash
# Run a container interactively
docker run -it <image-name>

# Run with a name
docker run -it --name <container-name> <image-name>

# Run and remove container on exit
docker run -it --rm <image-name>

# Run with X11 GUI support (for rviz, rqt, Gazebo)
```

**Linux:**

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  <image-name>
```

**Windows:**

Docker Desktop runs containers in a WSL2 VM with no native X server, so swap
the Linux X11 flags for `-e DISPLAY=host.docker.internal:0.0` and drop the
`-v /tmp/.X11-unix...` mount entirely (that path doesn't exist on Windows).
This needs an X server such as VcXsrv running on the host; see
[README.md](../README.md#quick-start) for setup. The same rationale applies
to every `DISPLAY=host.docker.internal:0.0` command in this file.

```powershell
docker run -it --rm -e DISPLAY=host.docker.internal:0.0 <image-name>
```

```bash
# Run with a volume mount
docker run -it --rm -v <host-path>:<container-path> <image-name>

# Run in detached (background) mode
docker run -d --name <container-name> <image-name>

# Override the default CMD (e.g. start a shell instead of auto-launching)
docker run -it --rm <image-name> bash
```

## Volumes / Live File Sync

Bind mounts map a host directory into the container in real time. A file
edited on the host shows up instantly inside the container, with no rebuild
or `docker cp` needed.

```bash
# Mount a single directory
docker run -it --rm \
  -v <host-path>:<container-path> \
  <image-name>

# Mount multiple directories (one -v per path)
docker run -it --rm \
  -v <host-path>/z1_hand_detector:/home/rosuser/catkin_ws/src/z1_hand_detector \
  -v <host-path>/z1_arm_tracker:/home/rosuser/catkin_ws/src/z1_arm_tracker \
  -v <host-path>/z1_teleop:/home/rosuser/catkin_ws/src/z1_teleop \
  ros-z1-teleop bash

# Mount as read-only (container cannot write back to host)
docker run -it --rm \
  -v <host-path>:<container-path>:ro \
  <image-name>
```

> Bind mounts override files baked into the image at those paths. Keep host
> files as the source of truth.

---

## GPU / Rendering

The `ros-z1` image uses Mesa software rendering by default
(`LIBGL_ALWAYS_SOFTWARE=1` is set in the Dockerfile). No GPU flags are needed
to run Gazebo or RViz.

To attempt hardware GPU rendering, override the env vars at runtime:

```bash
# NVIDIA GPU (requires nvidia-container-toolkit on host)
docker run -it --rm \
  --gpus all \
  --device /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros-z1 bash

# Intel / Mesa hardware (DRI passthrough)
docker run -it --rm \
  --device /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros-z1 bash
```

> Install nvidia-container-toolkit once on the host:
> `sudo apt-get install nvidia-container-toolkit && sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`

**Windows:** `/dev/dri` doesn't exist, and the `nvidia-container-toolkit`
steps above are Linux-only. Docker Desktop's WSL2 backend has a different
NVIDIA GPU passthrough path via `--gpus all` and the host's WSL2 NVIDIA
driver, with no `/dev/dri` flag involved, but we haven't tested it for this
image. Software rendering (the default) is the only path we support on
Windows.

---

## USB Device Passthrough (RealSense D435)

The D435 is a USB device. Docker doesn't expose USB devices by default, and
the mechanism to bridge one in differs completely between Linux and Windows.

### Linux

**Host prerequisite — udev rules (run once):**

The Intel RealSense apt repo doesn't support Ubuntu 24 (Noble), so install
the udev rules directly from upstream instead:

```bash
sudo curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
  -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the D435. Verify it's visible on the host:

```bash
lsusb | grep RealSense
# expected: Bus 00X Device 00X: ID 8086:0b07 Intel Corp. RealSense D435
```

**Pass the D435 USB bus into the container:**

First find the bus number:

```bash
lsusb | grep RealSense
# example: Bus 004 Device 003: ID 8086:0b07 Intel Corp. RealSense D435
```

Then pass only that bus, which is cleaner than exposing all buses:

```bash
docker run -it --rm \
  --name ros-z1-teleop \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/bus/usb/004:/dev/bus/usb/004 \
  ros-z1-teleop bash
```

If the bus number changes after a replug, recheck with `lsusb` and update
the path.

If the driver logs `RS2_USB_STATUS_ACCESS`, the udev rules haven't taken
effect. Unplug and replug the D435 after reloading the rules. As a fallback,
use `--privileged`:

```bash
# Fallback — full device access (workshop / dev use only)
docker run -it --rm \
  --name ros-z1-teleop \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged \
  ros-z1-teleop bash
```

### Windows

Windows recognizing the D435, through Device Manager or `Get-PnpDevice`, is
not the same thing as the container seeing it. Docker Desktop's containers
run inside a WSL2 VM, which has no native access to host USB devices, so
there's no `/dev/bus/usb` equivalent to mount unless the device is
explicitly attached to WSL2 first. That bridge is `usbipd-win`.

**1. Install `usbipd-win` once (elevated PowerShell):**

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

**2. List USB devices and find the D435's BUSID:**

```powershell
usbipd list
# example: 4-3    8086:0b07  Intel(R) RealSense(TM) Depth Camera 435   Not shared
```

**3. Bind and attach it to WSL2** (replace `4-3` with your BUSID). `attach`
isn't persistent, so re-run it after every unplug/replug or reboot:

```powershell
usbipd bind --busid 4-3
usbipd attach --wsl --busid 4-3
```

**4. Confirm it landed in the right place, then note the bus number.**

`usbipd attach --wsl` attaches to whichever WSL distro is currently the
default (`wsl -l -v` shows which one that is), and that's not necessarily the
`docker-desktop` distro that actually backs Docker Desktop's containers.
Check `docker-desktop` directly rather than your default distro (Ubuntu, say):

```powershell
wsl -d docker-desktop -- lsusb
# example: Bus 002 Device 002: ID 8086:0b07 Intel Corp. RealSense D435
```

If it's missing from `docker-desktop` but shows up under plain `wsl lsusb`,
`usbipd attach` landed it in your default distro instead. Switch the default
with `wsl -l -v` / `wsl -s docker-desktop`, or pass `-d docker-desktop` if
your `usbipd` version supports targeting a distro directly, then re-attach.

**5. Load `uvcvideo` and confirm `/dev/videoX` nodes exist** (cameras with a
standard UVC interface, which the D435 is one of, need this step):

The WSL2 kernel ships the `uvcvideo` module but doesn't load it
automatically. Without it, `lsusb` shows the raw device but no `/dev/videoX`
node gets created, which breaks anything that talks V4L2 (`ffplay`,
`v4l2-ctl`, OpenCV's `cv2.VideoCapture`, and so on) even though the raw USB
bus is reachable:

```powershell
wsl -d docker-desktop -- modprobe uvcvideo
wsl -d docker-desktop -- ls /dev/video*
# example: /dev/video0  /dev/video1
```

This module load isn't persistent. Repeat it after every Docker
Desktop / WSL2 restart (it survives a camera unplug/replug, but not a VM
restart).

**6. Pass the bus (and, if present, the video nodes) into the container**
(single line, since PowerShell doesn't treat a trailing `\` as a line
continuation). `/dev/videoX` is `crw-------` (root-only), so without
`--user root`, anything reading it (the ZED bridge node, `ffplay`, and so on)
fails with a permission/open error even though the device is correctly
passed through:

```powershell
docker run -it --rm --name ros-z1-teleop -e DISPLAY=host.docker.internal:0.0 --device /dev/bus/usb/002:/dev/bus/usb/002 --device /dev/video0:/dev/video0 --device /dev/video1:/dev/video1 --user root ros-z1-teleop bash
```

If `usbipd attach` succeeds but the container still reports
`RS2_USB_STATUS_ACCESS`, fall back to `--privileged` (workshop / dev use only):

```powershell
docker run -it --rm --name ros-z1-teleop -e DISPLAY=host.docker.internal:0.0 --privileged ros-z1-teleop bash
```

Three things quietly break this setup even after the device shows up as
reachable in `docker-desktop`:

1. **Missing `--device` entirely.** Docker never auto-passes-through
   devices. If the `docker run` you actually ran doesn't have
   `--device /dev/bus/usb/002:/dev/bus/usb/002` on it, the container's `/dev`
   stays empty no matter what the VM sees. Double-check the exact command
   you ran, not just the one from this doc.
2. **Bus number mismatch.** The bus number in `--device` has to match the
   current `wsl -d docker-desktop -- lsusb` output. It gets reassigned on
   every `usbipd attach`, so a number copied from an earlier session (say,
   `004`) silently mounts nothing once it's actually `002`.
3. **Stale container.** Devices are snapshotted into a container at
   `docker run` time, not re-read live. If you `usbipd attach` after the
   container is already running, that container will never see the device.
   `docker stop`/`docker rm` it (or just re-run `docker run --rm`, which
   removes it on exit) and start a fresh one now that the device is attached.

### Diagnosing corrupted/garbled video over usbipd (Windows)

`usbipd-win` tunnels USB over TCP into the WSL2 VM. Bulk-transfer devices,
like flash drives or serial adapters, tunnel fine. Isochronous transfers,
which webcams use for continuous high-bandwidth video, frequently don't. At
full resolution this shows up as a wall of identical warnings, one per
frame, with the reported buffer size varying wildly instead of staying
constant for a given format:

```txt
[video4linux2,v4l2 @ ...] Dequeued v4l2 buffer contains corrupted data (229292 bytes)
[video4linux2,v4l2 @ ...] Dequeued v4l2 buffer contains corrupted data (3079064 bytes)
```

This is a usbip limitation, not a Docker, driver, or camera fault. It won't
be fixed by `--privileged`, a different `--device` mapping, or rebuilding
the image.

**Test raw frames directly** (run as root so `apt-get` and the device's
`crw-------` permissions both work; this needs `/dev/videoX` from step 5
above):

```powershell
docker run -it --rm --name ros-z1-teleop -e DISPLAY=host.docker.internal:0.0 --device /dev/video0:/dev/video0 --device /dev/video1:/dev/video1 --user root ros-z1-teleop bash
```

Inside the container, list the modes the camera actually supports:

```bash
apt-get update -qq && apt-get install -y -qq v4l-utils
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Then preview a given mode live (a window opens via VcXsrv through the
`DISPLAY` set above):

```bash
apt-get install -y -qq ffmpeg
ffplay -f v4l2 -video_size <WIDTHxHEIGHT> -framerate <FPS> -i /dev/video0
```

The corruption is bandwidth-dependent. A ZED 2 tested at its default
`2560x720@60fps` mode corrupted essentially every frame; dropping to its
lowest-bandwidth mode, `1344x376@15fps`, produced zero corrupted-frame
warnings over the same run. If your camera's native or default resolution
corrupts, work down through `--list-formats-ext`'s reported modes, lower
resolution first, then lower framerate, until the warnings disappear, and
use that as the known-good mode for your pipeline.

### Verify the camera is visible inside the container (both platforms)

```bash
# RealSense-specific (depth + metadata via librealsense)
docker exec -it ros-z1-teleop bash -c "rs-enumerate-devices | grep -A3 D435"

# Generic UVC raw-frame check (any camera, see corruption diagnosis above)
docker exec -it ros-z1-teleop bash -c "ffplay -f v4l2 -i /dev/video0"
```

---

## Enter (exec into running container)

```bash
# Open a bash shell in a running container
docker exec -it <container-name> bash

# Run a specific command inside a running container
docker exec -it <container-name> <command>

# Pass environment variables (e.g. for GUI tools)
docker exec -e DISPLAY=$DISPLAY -it <container-name> bash
```

## Logs

```bash
# Show container logs
docker logs <container-name>

# Follow container logs in real time
docker logs -f <container-name>

# Inspect container details (network, mounts, env, etc.)
docker inspect <container-name>
```

## Stop / Start

```bash
# Stop a running container
docker stop <container-name>

# Start a stopped container
docker start <container-name>

# Restart a container
docker restart <container-name>
```

## Delete

```bash
# Remove a stopped container
docker rm <container-name>

# Force remove a running container
docker rm -f <container-name>

# Remove an image
docker rmi <image-name>

# Remove all stopped containers
docker container prune

# Remove all unused images
docker image prune

# Remove all unused images, containers, volumes, and networks
docker system prune -a
```
