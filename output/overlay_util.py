# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

from typing import List

import cv2
import numpy
from config.config import ConfigStore
from vision_types import FiducialImageObservation, FiducialPoseObservation, ObjDetectObservation


def overlay_image_observation(image: cv2.Mat, observation: FiducialImageObservation, config_store: ConfigStore = None) -> None:
    cv2.aruco.drawDetectedMarkers(image, numpy.array([observation.corners]), numpy.array([observation.tag_id]))

    # Draw 3D bounding box if calibration is available
    if config_store is not None and config_store.local_config.has_calibration and config_store.remote_config.fiducial_size_m > 0:
        fid_size = config_store.remote_config.fiducial_size_m
        half = fid_size / 2.0
        # Tag face corners (coplanar, Z=0)
        object_points = numpy.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=numpy.float64)

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                observation.corners.reshape(4, 2).astype(numpy.float64),
                config_store.local_config.camera_matrix,
                config_store.local_config.distortion_coefficients,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                return
        except Exception:
            return

        # Project the 3D box: front face (Z=0) + back face (Z=-fid_size)
        depth = fid_size
        box_points = numpy.array([
            # Front face
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
            # Back face (extends behind the tag)
            [-half,  half, -depth],
            [ half,  half, -depth],
            [ half, -half, -depth],
            [-half, -half, -depth],
        ], dtype=numpy.float64)

        img_points, _ = cv2.projectPoints(
            box_points,
            rvec, tvec,
            config_store.local_config.camera_matrix,
            config_store.local_config.distortion_coefficients,
        )
        pts = img_points.reshape(-1, 2).astype(int)

        # Draw front face (green)
        for i in range(4):
            cv2.line(image, tuple(pts[i]), tuple(pts[(i + 1) % 4]), (0, 255, 0), 2)

        # Draw back face (green, thinner)
        for i in range(4):
            cv2.line(image, tuple(pts[4 + i]), tuple(pts[4 + (i + 1) % 4]), (0, 255, 0), 1)

        # Draw connecting edges (green)
        for i in range(4):
            cv2.line(image, tuple(pts[i]), tuple(pts[i + 4]), (0, 255, 0), 1)

        # Draw axis at center
        cv2.drawFrameAxes(
            image,
            config_store.local_config.camera_matrix,
            config_store.local_config.distortion_coefficients,
            rvec, tvec,
            fid_size * 0.5,
        )


def overlay_pose_observation(image: cv2.Mat, config_store: ConfigStore, observation: FiducialPoseObservation) -> None:
    cv2.drawFrameAxes(
        image,
        config_store.local_config.camera_matrix,
        config_store.local_config.distortion_coefficients,
        observation.rvec_0,
        observation.tvec_0,
        config_store.remote_config.fiducial_size_m / 2,
    )
    cv2.drawFrameAxes(
        image,
        config_store.local_config.camera_matrix,
        config_store.local_config.distortion_coefficients,
        observation.rvec_1,
        observation.tvec_1,
        config_store.remote_config.fiducial_size_m / 2,
    )


def overlay_obj_detect_observation(image: cv2.Mat, observation: ObjDetectObservation) -> None:
    cv2.rectangle(
        image,
        (int(observation.corner_pixels[0][0]), int(observation.corner_pixels[0][1])),
        (int(observation.corner_pixels[3][0]), int(observation.corner_pixels[3][1])),
        (0, 0, 225),
        2,
    )
    cv2.putText(
        image,
        str(round(observation.confidence * 100)) + "%",
        (int(observation.corner_pixels[0][0]), int(observation.corner_pixels[0][1] - 5)),
        cv2.FONT_HERSHEY_PLAIN,
        2,
        (0, 0, 225),
        2,
    )


def overlay_circle_obj_detect_observation(image: cv2.Mat, observation: ObjDetectObservation) -> None:
    cv2.ellipse(
        image,
        (int((observation.corner_pixels[0][0] + observation.corner_pixels[1][0]) / 2),
         int((observation.corner_pixels[0][1] + observation.corner_pixels[2][1]) / 2)),
        (int((observation.corner_pixels[1][0] - observation.corner_pixels[0][0]) / 2),
         int((observation.corner_pixels[2][1] - observation.corner_pixels[0][1]) / 2)),
        0,
        180,
        360,
        (0, 0, 225),
        2,
    )


def overlay_2026_obj_detect_observations(image: cv2.Mat, observations: List[ObjDetectObservation]):
    # Render robot boxes
    [overlay_obj_detect_observation(image, x) for x in observations if x.obj_class == 1]

    # Render fuel circles
    fuel_observations = [x for x in observations if x.obj_class == 0]
    [overlay_circle_obj_detect_observation(image, x) for x in fuel_observations]

    # Render fuel count
    count_text = str(len(fuel_observations))
    (text_width, text_height), _ = cv2.getTextSize(count_text, cv2.FONT_HERSHEY_PLAIN, 5, 5)
    cv2.putText(
        image,
        count_text,
        (image.shape[1] - text_width - 10, text_height + 25),
        cv2.FONT_HERSHEY_PLAIN,
        5,
        (255, 255, 225),
        5
    )
