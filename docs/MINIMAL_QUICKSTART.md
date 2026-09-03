# Minimal Quick Start (Windows/PowerShell)

Commands only, for building the image and starting the terminals for each run
mode. Full details: [QUICKSTART.md](QUICKSTART.md).

Cheat sheet: open palm to drive, fist to freeze, show your right hand.

Two ways to feed the camera:

- **Variation A** — hardware-free looping video file (zero camera, always works).
- **Variation B** — a real ZED 2 / RealSense over `usbipd` USB forwarding, the
  same bind/attach/`uvcvideo` path the earlier ArUco version of this project
  used, wrapped in `start_teleop.sh`. This is the recommended real-camera path
  on Windows.

## 0. Build the image (once)

```powershell
git submodule update --init --recursive
docker build -t ros-z1-teleop .
```

## 1. Start X11 (VcXsrv) — once per Windows session

```powershell
& "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -ac -wgl
```

---

## 2. Variation A — hardware-free demo (looping video file)

**Terminal 1** — start the container and launch (point at a hand video on the host):

```powershell
docker run -it --rm --name ros-z1-teleop -e DISPLAY=host.docker.internal:0.0 ros-z1-teleop bash
```

Inside it:

```bash
z1_teleop image_source:=video:/path/to/hand.mp4
```

**Terminal 2** — unpause physics:

```powershell
docker exec -it ros-z1-teleop bash -ic "z1_unpause"
```

**Terminal 3** — RViz:

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 -it ros-z1-teleop bash -ic "z1_rviz"
```

**Terminal 4** *(optional)* — hand overlay view:

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 -it ros-z1-teleop bash -ic "z1_cam"
```

On Linux/macOS you can use a real webcam instead of a video file: add
`--device /dev/video0:/dev/video0` to `docker run` and launch plain
`z1_teleop` (default `image_source: webcam`). On Windows the built-in webcam
generally can't be passed into Docker, so use a video file or the ZED path.

---

## 3. Variation B — real camera over usbipd (ZED 2 / RealSense)

`start_teleop.sh` handles the whole Windows USB-forwarding sequence: a VcXsrv
check, `usbipd bind`/`attach` of the camera into WSL2, a `uvcvideo` reload,
then `docker run` with `--device /dev/bus/usb/<bus>` plus `/dev/video0` and
`/dev/video1`, and finally it opens RViz on its own. The first `usbipd bind`
needs an Administrator Git Bash.

**Terminal 1** — run from Git Bash on the host, not the WSL `bash`:

```powershell
& "C:\Program Files\Git\bin\bash.exe" ./start_teleop.sh
# headless (no Gazebo GUI):  ./start_teleop.sh z1_teleop_zed_headless
```

Defaults to the ZED 2 (`z1_teleop_zed` → `zed_camera_node`). For a D435
instead, edit `DEVICE_NAME="RealSense"` in the script and pass
`z1_teleop_realsense`. ZED resolution/fps/eye come from the `zed_camera:`
block in `teleop.yaml`. Use `1344x376@15fps`, the only mode we've confirmed
corruption-free over usbipd.

**Terminal 2** — unpause physics:

```powershell
docker exec -it ros-z1-teleop bash -ic "z1_unpause"
```

Prefer to drive the USB steps yourself? The full manual `usbipd list` /
`bind` / `attach` / `uvcvideo` sequence is in
[DOCKER_CMDS.md](DOCKER_CMDS.md).

---

## 4. Cleanup (between variations / when done)

```powershell
docker rm -f ros-z1-teleop
```

Variation A's container is removed automatically when you exit Terminal 1
(`--rm`). Variation B's script clears out any stale `ros-z1-teleop` container
itself before starting, so you can switch straight from A to B without
manual cleanup.
