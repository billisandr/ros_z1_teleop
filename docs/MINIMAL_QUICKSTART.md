# Minimal Quick Start (Windows/PowerShell)

Commands only — build the image, then start the terminals for each run mode.
Full details: [QUICKSTART.md](QUICKSTART.md).

## 0. Build the image (once)

```powershell
git submodule update --init --recursive
docker build -t ros-z1-aruco-real .
```

## 1. Start X11 (VcXsrv) — once per Windows session

```powershell
& "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -ac -wgl
```

---

## 2. Variation A — full Gazebo simulation (simulated camera + animated marker)

**Terminal 1** — start the container:

```powershell
docker run -it --rm --name ros-z1-real -e DISPLAY=host.docker.internal:0.0 ros-z1-aruco-real bash
```

Inside it:

```bash
z1_sim
```

**Terminal 2** — unpause physics:

```powershell
docker exec -it ros-z1-real bash -ic "z1_unpause"
```

**Terminal 3** — RViz:

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 -it ros-z1-real bash -ic "z1_rviz"
```

**Terminal 4** *(optional)* — camera overlay view:

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 -it ros-z1-real bash -ic "z1_camera"
```

---

## 3. Variation B — ZED 2 real-camera tracking

**Terminal 1** — one-shot script (handles VcXsrv check, `usbipd` bind/attach, `uvcvideo` reload, container start, auto-opens RViz). Run from Git Bash, not the WSL `bash`:

```powershell
& "C:\Program Files\Git\bin\bash.exe" ./start_zed_sim.sh --dictionary DICT_5X5_50 --marker-size 0.05 --tracking-id 0
```

**Terminal 2** — unpause physics:

```powershell
docker exec -it ros-z1-real bash -ic "z1_unpause"
```

---

## 4. Cleanup (between variations / when done)

```powershell
docker rm -f ros-z1-real
```

Note: Variation A's container is removed automatically when you exit Terminal 1 (`--rm`). Variation B's script removes any stale `ros-z1-real` container itself before starting, so you can switch straight from A to B without manual cleanup.
