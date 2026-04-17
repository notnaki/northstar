# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math
import queue
import time
from typing import List, Tuple, Union

import cv2
import ntcore
from config.config import ConfigStore
from output.overlay_util import overlay_image_observation
from output.StreamServer import MjpegServer
from pipeline.CameraPoseEstimator import MultiTargetCameraPoseEstimator
from pipeline.FiducialDetector import ArucoFiducialDetector
from pipeline.PoseEstimator import SquareTargetPoseEstimator
from pipeline.TagAngleCalculator import CameraMatrixTagAngleCalculator
from vision_types import (
    CameraPoseObservation,
    FiducialImageObservation,
    FiducialPoseObservation,
    TagAngleObservation,
)

DEMO_ID = 42


def apriltag_worker(
    q_in: queue.Queue[Tuple[float, cv2.Mat, ConfigStore]],
    q_out: queue.Queue[
        Tuple[
            float,
            List[FiducialImageObservation],
            Union[CameraPoseObservation, None],
            List[TagAngleObservation],
            Union[FiducialPoseObservation, None],
        ]
    ],
    stream_server: MjpegServer,
):
    fiducial_detector = ArucoFiducialDetector(cv2.aruco.DICT_APRILTAG_36h11)
    camera_pose_estimator = MultiTargetCameraPoseEstimator()
    tag_angle_calculator = CameraMatrixTagAngleCalculator()
    tag_pose_estimator = SquareTargetPoseEstimator()

    frame_count = 0
    last_fps_time = time.time()
    current_fps = 0

    while True:
        sample = q_in.get()
        timestamp: float = sample[0]
        image: cv2.Mat = sample[1]
        config: ConfigStore = sample[2]

        image_observations = fiducial_detector.detect_fiducials(image, config)
        camera_pose_observation = camera_pose_estimator.solve_camera_pose(
            [x for x in image_observations if x.tag_id != DEMO_ID], config
        )
        tag_angle_observations = [
            tag_angle_calculator.calc_tag_angles(x, config)
            for x in image_observations
            if x.tag_id != DEMO_ID
        ]
        tag_angle_observations = [x for x in tag_angle_observations if x != None]
        demo_image_observations = [x for x in image_observations if x.tag_id == DEMO_ID]
        demo_pose_observation: Union[FiducialPoseObservation, None] = None
        if len(demo_image_observations) > 0:
            demo_pose_observation = tag_pose_estimator.solve_fiducial_pose(
                demo_image_observations[0], config
            )

        q_out.put(
            (
                timestamp,
                image_observations,
                camera_pose_observation,
                tag_angle_observations,
                demo_pose_observation,
            )
        )

        # Update FPS counter
        frame_count += 1
        now = time.time()
        if now - last_fps_time >= 1.0:
            current_fps = frame_count
            frame_count = 0
            last_fps_time = now

        # Build telemetry for the dashboard
        nt_connected = ntcore.NetworkTableInstance.getDefault().isConnected()
        telemetry = {
            "has_frame": True,
            "nt_connected": nt_connected,
            "fps": current_fps,
            "tag_count": len(image_observations),
            "tag_ids": [int(x.tag_id) for x in image_observations],
            "solve_type": None,
            "error": None,
            "pose": None,
            "config": {
                "device_id": config.local_config.device_id,
                "camera_id": config.remote_config.camera_id,
                "resolution": (
                    f"{config.remote_config.camera_resolution_width}x{config.remote_config.camera_resolution_height}"
                    if config.remote_config.camera_resolution_width > 0
                    else None
                ),
                "exposure": config.remote_config.camera_exposure,
                "gain": config.remote_config.camera_gain,
                "tag_size": config.remote_config.fiducial_size_m,
            },
        }
        if camera_pose_observation is not None:
            pose = camera_pose_observation.pose_0
            telemetry["solve_type"] = (
                "single" if camera_pose_observation.pose_1 is not None else "multi"
            )
            telemetry["error"] = float(camera_pose_observation.error_0)
            rot = pose.rotation()
            telemetry["pose"] = {
                "x": float(pose.translation().X()),
                "y": float(pose.translation().Y()),
                "z": float(pose.translation().Z()),
                "roll": float(math.degrees(rot.X())),
                "pitch": float(math.degrees(rot.Y())),
                "yaw": float(math.degrees(rot.Z())),
            }

        # Include tag layout positions for the 2D field map
        tag_layout = config.remote_config.tag_layout
        if tag_layout is not None:
            telemetry["tag_poses"] = [
                {
                    "id": t["ID"],
                    "x": t["pose"]["translation"]["x"],
                    "y": t["pose"]["translation"]["y"],
                }
                for t in tag_layout.get("tags", [])
            ]
            telemetry["field_length"] = tag_layout.get("field", {}).get("length", 16.54)
            telemetry["field_width"] = tag_layout.get("field", {}).get("width", 8.21)
        else:
            telemetry["tag_poses"] = None
            telemetry["field_length"] = 16.54
            telemetry["field_width"] = 8.21

        stream_server.set_telemetry(telemetry)

        if stream_server.get_client_count() > 0:
            image = image.copy()
            [overlay_image_observation(image, x, config) for x in image_observations]
            fps_text = f"{current_fps} FPS"
            h, w = image.shape[:2]
            scale = max(0.6, h / 400.0)
            thickness = max(1, int(scale * 2))
            (tw, th), _ = cv2.getTextSize(
                fps_text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness
            )
            pad = int(8 * scale)
            cv2.rectangle(
                image, (0, 0), (tw + 2 * pad, th + 2 * pad), (0, 0, 0), -1
            )
            cv2.putText(
                image, fps_text, (pad, th + pad),
                cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0), thickness, cv2.LINE_AA,
            )
            stream_server.set_frame(image)
