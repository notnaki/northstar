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


def overlay_image_observation(image: cv2.Mat, observation: FiducialImageObservation) -> None:
    cv2.aruco.drawDetectedMarkers(image, numpy.array([observation.corners]), numpy.array([observation.tag_id]))


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
