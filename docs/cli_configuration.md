# CLI & Configuration Reference

This document provides a comprehensive reference for configuring and launching **Orion-Shadow**, including CLI options, runner shell scripts, environment variables, and performance tuning recommendations.

---

## 🚀 Command-Line Options Reference

The primary server entrypoint is `orion_shadow.server`. It accepts arguments controlling networking, physics fidelity, terrain rendering, video multicast streaming, and initial flight kinematics.

```bash
python3 -m orion_shadow.server [OPTIONS]
```

### Complete Parameter Table

| CLI Flag | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| **Network Options** | | | |
| `--host` | `str` | `0.0.0.0` | Interface IP address to bind network servers to. |
| `--port`, `--udp-port` | `int` | `8745` | UDP port for inbound commands & discovery broadcast reception (`UDP_OUT_PORT`). |
| `--udp-in-port` | `int` | `8746` | Destination UDP port for outbound discovery responses (`UDP_IN_PORT`). |
| `--tcp-port` | `int` | `8747` | TCP port for persistent bidirectional command/telemetry streaming (`TCP_PORT`). |
| **Physics & Timing** | | | |
| `--dt` | `float` | `0.1` | Simulation integration step and telemetry update period in seconds. |
| **Terrain & Geospatial** | | | |
| `--dted-path` | `str` | `None` | Path to a DTED elevation file (`.dt0`, `.dt1`, `.dt2`) or a root directory of tiles. |
| `--drape-subsample` | `int` | `6` | Screen ray-marching subsampling divisor for 3D terrain draping (higher = faster). |
| **Map Tiles & Caching** | | | |
| `--tile-url` | `str` | `None` | XYZ slippy map tile URL template (e.g. `https://mt0.google.com/vt/lyrs=s&x={x}&y={y}&z={z}`). |
| `--tile-cache-dir` | `str` | `cache/tiles` | Local filesystem directory path to cache downloaded map tiles permanently. |
| `--tile-zoom`, `--zoom` | `int` | `None` | Fixes tile zoom level (e.g., 14–19). If omitted, zoom is calculated adaptively. |
| `--max-tile-zoom` | `int` | `17` | Upper limit on adaptive zoom level to bound download bandwidth and memory. |
| `--prefetch-distance` | `float` | `10000.0` | Lookahead distance in meters along flight vector for prefetching tiles. |
| `--no-prefetch` | `flag` | `False` | Disables lookahead tile prefetching. |
| `--distance-lod` | `flag` | `True` | Enables distance-dependent Level of Detail (LOD) for distant terrain tiles. |
| `--no-distance-lod` | `flag` | `False` | Disables distance LOD (forces uniform zoom across entire scene). |
| **Video & Multicast** | | | |
| `--no-video` | `flag` | `False` | Disables the synthetic multicast video server completely (headless mode). |
| `--multicast-group` | `str` | `239.255.0.1` | Multicast IPv4 address for H.264/MPEG-TS video streaming. |
| `--video-port` | `int` | `5004` | Destination UDP port for the multicast video stream. |
| `--fps`, `--video-fps` | `int` | `10` | Video stream framerate in frames per second. |
| `--video-width` | `int` | `1280` | Rendered video frame width in pixels (e.g. 1280 for 720p). |
| `--video-height` | `int` | `720` | Rendered video frame height in pixels (e.g. 720 for 720p). |
| `--gpu-encoding` | `flag` | `Auto` | Forces NVIDIA NVENC hardware H.264 encoding (`h264_nvenc`). |
| `--no-gpu-encoding` | `flag` | `False` | Forces CPU software encoding (`libx264`). |
| **Platform Kinematics** | | | |
| `--lat`, `--latitude` | `float` | `0.0` | Initial aircraft/camera geodetic latitude in degrees. |
| `--lon`, `--longitude` | `float` | `0.0` | Initial aircraft/camera geodetic longitude in degrees. |
| `--alt`, `--altitude` | `float` | `1000.0` | Initial aircraft altitude in meters MSL. |
| `--pan` | `float` | `0.0` | Initial gimbal pan angle in degrees. |
| `--tilt` | `float` | `None` | Initial gimbal tilt in degrees (defaults to `-20.0^\circ` if altitude is non-zero). |
| `--heading` | `float` | `0.0` | Initial aircraft true heading in degrees ($0^\circ = \text{North}$). |
| `--speed` | `float` | `0.0` | Initial aircraft forward groundspeed in knots. |
| **Diagnostics & Logging** | | | |
| `--logger`, `--log-level` | `str` | `warning` | Logging verbosity: `debug`, `info`, `warning`, `error`, `critical`. |

---

## 📜 Shell Runner Script (`scripts/run.sh`)

A production shell runner is included at `scripts/run.sh`. It automates environment setup, discovers NVIDIA CUDA runtime libraries, and configures default maps and terrain paths.

```bash
./scripts/run.sh [OPTIONS]
```

### Key Features of `scripts/run.sh`:
1. **CUDA Library Path Configuration**:
   Automatically inspects `$HOME/.local/lib/python3.9/site-packages/nvidia/` and prepends all required `.so` library folders (`libcudart.so`, `libnvrtc.so`, `libnvJitLink.so`, `libcublas.so`) to `LD_LIBRARY_PATH`.
2. **Default Map Sources**:
   Allows easy toggling between Google Road Map, Google Satellite, ESRI World Imagery, or custom local tile servers.
3. **Pass-through Arguments**:
   Any argument supported by `server.py` can be passed directly to `scripts/run.sh`:
   ```bash
   ./scripts/run.sh --lat 39.6 --lon -116.3 --alt 3000 --fps 15 --logger info
   ```

---

## ⚡ Performance Tuning & Deployment Modes

### 1. Ultra-Low Resource / Headless CI Mode
When running automated unit tests or executing in headless CI pipelines without a GPU or display:
```bash
python3 -m orion_shadow.server --no-video --dt 0.1 --log-level error
```
- **CPU Impact**: $< 1\%$ CPU utilization.
- **Network**: Only UDP `8745`/`8746` and TCP `8747` active.

### 2. High-Fidelity Hardware-in-the-Loop (HIL)
When testing embedded microcontrollers or high-bandwidth autopilots:
```bash
python3 -m orion_shadow.server \
  --dt 0.005 \
  --no-video \
  --log-level warning
```
- Runs the physics loop and telemetry generation at **200 Hz**.

### 3. Full 3D Visual Simulation with GPU Acceleration
For realistic operator training, computer vision algorithm testing, or live video evaluation:
```bash
python3 -m orion_shadow.server \
  --dted-path /mnt/data/DTED/ \
  --tile-url "https://mt0.google.com/vt/lyrs=s&x={x}&y={y}&z={z}" \
  --gpu-encoding \
  --drape-subsample 4 \
  --fps 20 \
  --video-width 1280 \
  --video-height 720
```
- Ray-marches DTED elevation on GPU via CuPy.
- Drapes satellite tiles onto terrain with $4\times$ subsampling.
- Encodes H.264 video using NVIDIA NVENC.
