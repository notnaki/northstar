#!/usr/bin/env python3

# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

"""
Simple test harness that iterates a directory of image files,
feeds them into `objdetect_worker`, and prints received observations.

Usage:
  python3 utils/objdetect_testharness.py --input-dir path/to/images

This file starts `objdetect_worker` in a daemon thread, so the process
will exit when the main thread finishes.
"""
import sys
from pathlib import Path

# -- Temporary hack until we fix project structure to a proper python package
sys.path.append(str(Path(__file__).parent.parent.resolve()))
# ----

from utils.input_utils import build_source, FrameStreamer
from config.config import ConfigStore, LocalConfig, RemoteConfig
from objdetect_worker import objdetect_worker
import argparse


import time
import queue
import threading
import numpy as np
import cv2






def main():

    # Parse command line argument
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", required=True, help="Directory with images to test")
    parser.add_argument("--server-port", type=int, default=8080, help="Mjpeg server port for worker")
    parser.add_argument("--max-frames", type=int, default=0, help="Max frames to process (0 = all)")
    parser.add_argument("--step", type=int, default=1, help="Process every N-th image in directory (1 = all)")
    parser.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for worker output per frame")
    parser.add_argument("--sleep", type=float, default=0.0, help="Seconds to sleep between frames")
    parser.add_argument("--model", type=str, required=True, help="Object detection model to use")
    parser.add_argument("--repeat", type=int, default=1, help="Number of times to send each image")
    parser.add_argument("--verbosity", "-v", type=int, default=0,
                        help="Verbosity level (integer). >=1 prints per-observation details")

    args = parser.parse_args()

    # Define input/output queues
    q_in: queue.Queue = queue.Queue()
    q_out: queue.Queue = queue.Queue()

    # Start the worker under test
    th = threading.Thread(target=objdetect_worker, args=(q_in, q_out, args.server_port), daemon=True)
    th.start()

    # Build mock configuration
    local_config = LocalConfig()
    local_config.obj_detect_model = args.model
    local_config.distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # No distortion for test harness
    local_config.camera_matrix = np.c_[np.eye(3)]

    # Build a frame source (video file or image directory) and wrap in a FrameStreamer
    src = build_source(Path(args.input_dir))
    streamer = FrameStreamer(source=src,
                             repeat=max(1, args.repeat),
                             max_frames=None if args.max_frames == 0 else args.max_frames,
                             step=max(1, args.step))

    # Stream data
    processed = 0
    frame_index = 0
    for frame in streamer:

        # frame is BGR (OpenCV) image
        # Convert to RGB which CoreML/PIL expects
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_rgb = img_rgb.copy()

        ts = time.time()
        cfg = ConfigStore(local_config, RemoteConfig())
        q_in.put((ts, img_rgb, cfg))

        # Wait for a response (best-effort)
        try:
            in_ts, observations = q_out.get(timeout=args.timeout)
            out_ts = time.time()
        except queue.Empty:
            print(f"Timeout waiting for output for frame {frame_index}")
            processed += 1
            if args.sleep:
                time.sleep(args.sleep)
            frame_index += 1
            continue

        # Print a concise summary of results
        try:
            obs_count = len(observations) if observations is not None else 0
        except Exception:
            obs_count = 1

        if args.verbosity >= 1:
            print(f"Frame: {frame_index} -> timestamp_in={in_ts:.6f} timestamp_out={out_ts:.6f} observations={obs_count}")
        if obs_count and args.verbosity >= 2:
            for idx, o in enumerate(observations):
                print(f"  [{idx}] {o}")

        processed += 1
        if args.sleep:
            time.sleep(args.sleep)

        frame_index += 1

    streamer.close()

    if args.verbosity >= 1:
        print(f"Done; processed {processed} files. Exiting.")


if __name__ == "__main__":
    main()
