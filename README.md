# Northstar

FRC vision coprocessor for AprilTag detection and optional object detection, publishing pose and tag observations over NetworkTables. This fork adds a multi-camera manager with a unified web dashboard so one coprocessor can drive several cameras at once.

## Features

- AprilTag pose estimation using OpenCV ArUco (36h11 family)
- Optional object detection pipeline (CoreML / configurable backends)
- Per-camera MJPEG stream with live telemetry overlay
- Multi-camera manager (`manager.py`) that spawns one subprocess per camera and serves a unified Limelight-style dashboard
- NetworkTables integration for config, telemetry, and `CameraPublisher` stream discovery (Elastic / Shuffleboard)
- Supports USB (V4L2 / AVFoundation), GStreamer, and Basler Pylon cameras

## Quick start

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# single camera
python3 __init__.py --config config.json --calibration calibration_new.yml

# multi-camera manager
python3 manager.py --config manager_config.json
```

Full setup, camera identification, and calibration instructions are in [SETUP_GUIDE.md](SETUP_GUIDE.md).

## Credits

This project is a fork of [Northstar](https://github.com/Mechanical-Advantage/RobotCode2026Public/tree/main/northstar) by **FRC Team 6328 (Mechanical Advantage)**, originally developed as part of their 2026 robot code. All credit for the core vision pipeline, calibration tooling, and overall architecture goes to them.

This fork adds the multi-camera manager, unified dashboard, and associated refactors on top of their work.

## License

MIT — see [LICENSE](LICENSE). Original copyright © 2022-2026 Littleton Robotics (FRC Team 6328).
