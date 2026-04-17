# Northstar Setup Guide

## Overview

Northstar is an FRC vision coprocessor system that runs AprilTag detection and optional object detection using NetworkTables. It supports two launch modes:

- **Single-camera mode** — run `__init__.py` directly with a config and calibration file.
- **Manager mode** — run `manager.py` to manage multiple camera instances from a single process with a built-in web dashboard.

Each camera instance has its own config file, calibration file, and NetworkTables device ID.

---

## 1. OS & Dependencies

### Orange Pi / Linux

Flash **Armbian** (Ubuntu-based) to the Orange Pi's SD card or eMMC:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3 python3-pip python3-venv git ffmpeg v4l-utils
```

### macOS

Install Python 3 via [Homebrew](https://brew.sh) if not already present:

```bash
brew install python3 ffmpeg
```

---

## 2. Clone & Install

```bash
git clone <your-northstar-repo-url> northstar
cd northstar
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Platform-specific packages

Not all packages in `requirements.txt` are needed on every platform:

| Package                                                         | Required on                             |
| --------------------------------------------------------------- | --------------------------------------- |
| `numpy`, `opencv-contrib-python`, `pyntcore`, `robotpy-wpimath` | All platforms                           |
| `pyobjc-framework-AVFoundation`                                 | macOS only (for `avfoundation` capture) |
| `coremltools`                                                   | macOS only (CoreML object detection)    |
| `pypylon`                                                       | Basler cameras only                     |

On Linux you can install just the essentials:

```bash
pip install numpy opencv-contrib-python pyntcore robotpy-wpimath
```

---

## 3. Find Your Cameras

### Linux

```bash
v4l2-ctl --list-devices
```

Note which `/dev/video*` index maps to each physical USB camera:

- USB cam A → `/dev/video0` → camera index `"0"`
- USB cam B → `/dev/video2` → camera index `"2"`

### macOS

Cameras are identified by a `location:vendor:product` ID string (e.g. `0x14200000:0x1234:0x5678`). The `avfoundation` capture backend uses this to match devices.

These indices are set via `camera_id` in the robot code (NetworkTables).

---

## 4. Create Config Files

Each camera instance needs a JSON config file. Every instance **must** have a unique `device_id` and unique stream ports.

### Config fields

| Field                   | Description                                                                        |
| ----------------------- | ---------------------------------------------------------------------------------- |
| `device_id`             | NetworkTables identity (e.g. `"northstar/northstar_0"`)                            |
| `server_ip`             | roboRIO IP address (e.g. `"10.TE.AM.2"`) or `"127.0.0.1"` for local testing        |
| `apriltags_stream_port` | MJPEG stream port for the AprilTag overlay                                         |
| `objdetect_stream_port` | MJPEG stream port for object detection overlay                                     |
| `capture_impl`          | Capture backend: `""` (default OpenCV), `"avfoundation"`, `"pylon"`, `"gstreamer"` |
| `obj_detect_model`      | Path to a CoreML model file (leave `""` to disable)                                |
| `apriltag_max_fps`      | Max AprilTag FPS (`-1` = unlimited)                                                |
| `obj_detect_max_fps`    | Max object detection FPS (`-1` = unlimited)                                        |
| `apriltags_enable`      | Enable AprilTag pipeline                                                           |
| `objdetect_enable`      | Enable object detection pipeline                                                   |
| `video_folder`          | Directory for recorded video clips (leave `""` to disable)                         |

### Example: `config_cam0.json`

```json
{
  "device_id": "northstar/northstar_0",
  "server_ip": "10.TE.AM.2",
  "apriltags_stream_port": 8000,
  "objdetect_stream_port": 8001,
  "capture_impl": "",
  "obj_detect_model": "",
  "apriltag_max_fps": 60,
  "obj_detect_max_fps": -1,
  "apriltags_enable": true,
  "objdetect_enable": false,
  "video_folder": ""
}
```

### Example: `config_cam1.json`

```json
{
  "device_id": "northstar/northstar_1",
  "server_ip": "10.TE.AM.2",
  "apriltags_stream_port": 8002,
  "objdetect_stream_port": 8003,
  "capture_impl": "",
  "obj_detect_model": "",
  "apriltag_max_fps": 60,
  "obj_detect_max_fps": -1,
  "apriltags_enable": true,
  "objdetect_enable": false,
  "video_folder": ""
}
```

### Capture backends

| `capture_impl`    | Use case                                           |
| ----------------- | -------------------------------------------------- |
| `""`              | Default OpenCV `VideoCapture` (Linux USB cameras)  |
| `"avfoundation"`  | macOS with `ns-iokit-ctl` for UVC exposure control |
| `"pylon"`         | Basler cameras via pypylon                         |
| `"pylon-flipped"` | Basler cameras, image flipped                      |
| `"pylon-color"`   | Basler cameras, color mode                         |
| `"gstreamer"`     | GStreamer pipeline capture                         |

---

## 5. Calibrate Each Camera

Each camera needs its own calibration file (different lenses produce different intrinsics). Calibration files can be in OpenCV YAML (`.yml`) or JSON (`.json`) format.

```bash
source venv/bin/activate
python3 __init__.py --config config_cam0.json --calibration calibration_cam0.yml
```

1. Set the `camera_id` from the robot code so Northstar opens the correct camera.
2. Use the NetworkTables calibration interface to capture checkerboard images.
3. The calibration result is saved to the file specified by `--calibration`.
4. Repeat for each camera.

### Calibration file format (YAML)

```yaml
%YAML:1.0
---
calibration_date: "2026-04-17 12:03:45"
camera_resolution: !!opencv-matrix
  rows: 2
  cols: 1
  dt: d
  data: [720., 1280.]
camera_matrix: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [...]
distortion_coefficients: !!opencv-matrix
  rows: 1
  cols: 5
  dt: d
  data: [...]
```

---

## 6. Running Northstar

### Option A: Single-camera mode

Run one instance per camera directly:

```bash
source venv/bin/activate
python3 __init__.py --config config_cam0.json --calibration calibration_cam0.yml
```

### Option B: Manager mode (recommended for multi-camera)

The manager spawns and monitors all camera instances from a single process and serves a web dashboard.

#### `manager_config.json`

```json
{
  "manager_port": 5800,
  "instances": [
    {
      "name": "Front Left",
      "config": "config_cam0.json",
      "calibration": "calibration_cam0.yml"
    },
    {
      "name": "Front Right",
      "config": "config_cam1.json",
      "calibration": "calibration_cam1.yml"
    }
  ]
}
```

| Field                     | Description                                                                    |
| ------------------------- | ------------------------------------------------------------------------------ |
| `manager_port`            | Port for the web dashboard (default `5800`)                                    |
| `instances[].name`        | Display name for the camera                                                    |
| `instances[].config`      | Path to the camera's config JSON (relative to northstar directory or absolute) |
| `instances[].calibration` | Path to the camera's calibration file                                          |

#### Start the manager

```bash
source venv/bin/activate
python3 manager.py --config manager_config.json
```

The manager will:

- Spawn a subprocess for each configured camera instance.
- Automatically restart any instance that crashes (after a 3-second delay).
- Serve a web dashboard at `http://<coprocessor-ip>:5800` showing camera streams and process status.
- Expose a `/api/processes` endpoint with JSON status for all instances.

---

## 7. Network Configuration

### Static IP (Linux)

Set a static IP on the robot network:

```bash
sudo nmcli con mod "Wired connection 1" \
  ipv4.addresses 10.TE.AM.11/24 \
  ipv4.gateway 10.TE.AM.1 \
  ipv4.method manual
sudo nmcli con up "Wired connection 1"
```

Set `server_ip` in each config file to the roboRIO's IP (typically `10.TE.AM.2`).

### Stream access

Camera MJPEG streams are available at:

- `http://<coprocessor-ip>:<apriltags_stream_port>/` — AprilTag overlay
- `http://<coprocessor-ip>:<objdetect_stream_port>/` — Object detection overlay
- `http://<coprocessor-ip>:<manager_port>/` — Manager dashboard (manager mode only)

---

## 8. Auto-Start with systemd (Linux)

### Using the manager (recommended)

Create `/etc/systemd/system/northstar.service`:

```ini
[Unit]
Description=Northstar Vision Manager
After=network.target

[Service]
Type=simple
User=orangepi
WorkingDirectory=/home/orangepi/northstar
ExecStart=/home/orangepi/northstar/venv/bin/python3 manager.py --config manager_config.json
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

### Using individual services (alternative)

Create `/etc/systemd/system/northstar-cam0.service`:

```ini
[Unit]
Description=Northstar Camera 0
After=network.target

[Service]
Type=simple
User=orangepi
WorkingDirectory=/home/orangepi/northstar
ExecStart=/home/orangepi/northstar/venv/bin/python3 __init__.py --config config_cam0.json --calibration calibration_cam0.yml
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

Repeat for each camera with a unique service file.

### Enable & start

```bash
sudo systemctl daemon-reload
sudo systemctl enable northstar        # or northstar-cam0 northstar-cam1
sudo systemctl start northstar
```

### Check status & logs

```bash
sudo systemctl status northstar
journalctl -u northstar -f
```

---

## 9. macOS-Specific: Building ns-iokit-ctl

The `avfoundation` capture backend requires `ns-iokit-ctl` to set UVC camera exposure controls. Build it from the included source:

```bash
cd ns-iokit-ctl
mkdir -p build && cd build
cmake ..
make
```

This produces `ns-iokit-ctl/build/ns_iokit_ctl`. Set `"capture_impl": "avfoundation"` in your config to use it.

---

## 10. Robot-Side Configuration

On the robot, create a `VisionIONorthstar` instance per camera. Configure the camera index (matching the `/dev/video` index or macOS device ID from step 3) and the robot-to-camera transform.

### Example: VisionConstants.java

```java
public static final NorthstarCameraConfig[] northstarCameras =
    new NorthstarCameraConfig[] {
      new NorthstarCameraConfig(
          "0",              // camera_id (/dev/video index or macOS device ID)
          1280, 720,        // resolution
          3, 300, 0.0, 0.0, 1.0,
          new Transform3d(
              new Translation3d(0.3, 0.0, 0.5),
              new Rotation3d(0, Math.toRadians(-15), 0)
          )),
      new NorthstarCameraConfig(
          "2",
          1280, 720,
          3, 300, 0.0, 0.0, 1.0,
          new Transform3d(
              new Translation3d(-0.3, 0.0, 0.5),
              new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180))
          )),
    };

public static double[] cameraStdDevFactors = new double[] { 1.0, 1.0 };
```

---

## Troubleshooting

| Problem                                          | Fix                                                                                                                          |
| ------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------- |
| `No camera ID, waiting to start capture session` | Robot code hasn't published `camera_id` via NetworkTables yet. Ensure the robot is running and connected.                    |
| `Camera not found, restarting`                   | The `camera_id` doesn't match any connected device. Re-check indices with `v4l2-ctl --list-devices`.                         |
| `Capture session failed, restarting`             | Camera disconnected or returned an empty frame. Check USB connections.                                                       |
| Process keeps crashing in manager mode           | Check `journalctl` logs or the manager dashboard at `http://<ip>:5800`. The manager auto-restarts crashed instances.         |
| Calibration not loading                          | Ensure the calibration file path is correct and the file contains valid `camera_matrix` and `distortion_coefficients` nodes. |
| Stream not accessible                            | Verify the stream port isn't blocked by a firewall and isn't already in use by another instance.                             |

### RobotContainer

Create two `VisionIONorthstar` instances:

```java
new VisionIONorthstar(0, this::getAprilTagLayout, this::getRotation),
new VisionIONorthstar(1, this::getAprilTagLayout, this::getRotation),
```

## 9. Dashboard Access

Once running, dashboards are available at:

- Camera 0: `http://10.TE.AM.11:8000`
- Camera 1: `http://10.TE.AM.11:8002`

## 10. Match Video Recording

Set `video_folder` in each config to a writable directory (e.g. `"/home/orangepi/videos"`). The robot code controls recording via the `is_recording` NT topic. Videos are named using the event name, match type, and match number from NT.

---

## Quick Reference

| Item             | Camera 0                | Camera 1                |
| ---------------- | ----------------------- | ----------------------- |
| `device_id`      | `northstar_0`           | `northstar_1`           |
| Stream port      | `8000`                  | `8002`                  |
| Config file      | `config_cam0.json`      | `config_cam1.json`      |
| Calibration      | `calibration_cam0.json` | `calibration_cam1.json` |
| systemd service  | `northstar-cam0`        | `northstar-cam1`        |
| Robot-side index | `0`                     | `1`                     |
