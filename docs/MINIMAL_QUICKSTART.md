# Minimal Quick Start (Windows/PowerShell)

Commands only — build the image, then start the terminals for each run mode.
Full details: [QUICKSTART.md](QUICKSTART.md).

Cheat sheet: **open palm to drive, fist to freeze, show your right hand.**

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

> On **Linux/macOS** you can use a real webcam instead of a video file: add
> `--device /dev/video0:/dev/video0` to `docker run` and launch plain `z1_teleop`
> (default `image_source: webcam`). On Windows the built-in webcam generally
> can't be passed into Docker — use a video file or the ZED path.

---

## 3. Variation B — ZED 2 real-camera teleop

**Terminal 1** — one-shot script (VcXsrv check, `usbipd` bind/attach, `uvcvideo`
reload, container start, auto-opens RViz). Run from Git Bash, not the WSL `bash`:

```powershell
& "C:\Program Files\Git\bin\bash.exe" ./start_teleop.sh
```

**Terminal 2** — unpause physics:

```powershell
docker exec -it ros-z1-teleop bash -ic "z1_unpause"
```

---

## 4. Cleanup (between variations / when done)

```powershell
docker rm -f ros-z1-teleop
```

Note: Variation A's container is removed automatically when you exit Terminal 1
(`--rm`). Variation B's script removes any stale `ros-z1-teleop` container itself
before starting, so you can switch straight from A to B without manual cleanup.
