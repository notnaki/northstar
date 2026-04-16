# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.


import logging
import time
import math
import cv2
import sys
import numpy as np
from typing import Dict, List, Optional, Protocol, Tuple
from pathlib import Path
from dataclasses import dataclass, field

from vision_types import ObjDetectObservation
from config.config import ConfigStore

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    stream=sys.stdout
)

logger = logging.getLogger(__name__)
logger.setLevel(logging.WARN)

# ------------------------
# -- Helper Classes
# ------------------------


class ObjectDetector:
    def __init__(self) -> None:
        raise NotImplementedError

    def detect(self, image: cv2.Mat, config: ConfigStore) -> List[ObjDetectObservation]:
        raise NotImplementedError


class Detector(Protocol):
    def __init__(self):
        ...

    def __call__(self, input: np.ndarray) -> List[np.ndarray]:
        ...


class InferenceModelBuilder:
    @staticmethod
    def build(model_path: Path) -> Detector:
        coreml = True if (model_path.suffix == ".mlpackage") else False

        if coreml:
            return CoreMlInferenceModel(model_path)
        else:
            return YoloInferenceModel(model_path)


class CoreMlInferenceModel(Detector):
    def __init__(self, model_path: Path):
        try:
            import coremltools
        except:
            raise RuntimeError("Unable to load coreml tools")
        self.model = coremltools.models.MLModel(str(model_path))

        # Get the model specification (TODO: check supported input shape)
        spec = self.model.get_spec()
        logger.info("CoreML model loaded")

    def __call__(self, input: np.ndarray) -> List[np.ndarray]:
        nbatches = input.shape[0]
        dets: List[np.ndarray] = []

        for i in range(nbatches):
            img = np.expand_dims(input[i], axis=0)
            prediction_single = self.model.predict({"input": img})
            dets.append(prediction_single["output"][0])

        return dets


class YoloInferenceModel(Detector):
    def __init__(self, model_path: Path):
        try:
            from ultralytics import YOLO
        except:
            raise RuntimeError("Unable to load ultralytics library")

        self.model = YOLO(model_path, task="detect", verbose=False)
        logger.info("Ultralytics model loaded")

    def __call__(self, input: np.ndarray) -> List[np.ndarray]:
        try:
            from torch import tensor
        except:
            raise RuntimeError("Unable to load to library")

        prediction = self.model(tensor(input), verbose=False)
        assert (isinstance(prediction, list) and hasattr(prediction[0], 'boxes')), "Unexpacted output from model"

        dets = [b.boxes.data.cpu().detach().numpy() for b in prediction]
        return dets


# ------------------------
# -- Helper Functions
# ------------------------
def _postprocess_detections(dets: np.ndarray, camera_matrix: np.typing.NDArray[np.float64], distortion_coefficients: np.typing.NDArray[np.float64],   conf_thresh: float = 0) -> List[ObjDetectObservation]:
    """
    Convert raw detection array into ObjDetectObservation instances.

    Parameters
    ----------
    dets : np.ndarray
        Array of detections with rows (x1, y1, x2, y2, conf, cls).
    camera_matrix : np.typing.NDArray[np.float64]
        Camera intrinsic matrix for undistortion / angle computation.
    distortion_coefficients : np.typing.NDArray[np.float64]
        Distortion coefficients used by cv2.undistortPoints.
    conf_thresh : float, optional
        Minimum confidence required to include a detection.

    Returns
    -------
    List[ObjDetectObservation]
        List of post-processed detection observations with corner angles
        and original corner pixel coordinates.
    """
    observations: List[ObjDetectObservation] = []

    for det in dets:
        if det[4] < conf_thresh:
            continue

        obj_class = int(det[5])
        confidence = det[4]
        corners = np.array([
            [det[0], det[1]],
            [det[2], det[1]],
            [det[0], det[3]],
            [det[2], det[3]]
        ])

        corners_undistorted = cv2.undistortPoints(
            corners,
            camera_matrix,
            distortion_coefficients,
            None,
            camera_matrix,
        )

        corner_angles = np.zeros((4, 2))
        for index, corner in enumerate(corners_undistorted):
            vec = np.linalg.inv(camera_matrix).dot(np.array([corner[0][0], corner[0][1], 1]).T)
            corner_angles[index][0] = math.atan(vec[0])
            corner_angles[index][1] = math.atan(vec[1])

        observations.append(ObjDetectObservation(obj_class, confidence, corner_angles, corners))

    return observations


def _visualize_detections(img: np.ndarray, detections: np.ndarray,
                          class_names: Optional[Dict[int, str]] = None,
                          show_class_label: bool = False,
                          show_conf: bool = False,
                          conf_thresh: float = 0.05,
                          max_boxes: int = 300,
                          out_path: Optional[str] = None,
                          title: str = "detections",
                          wait: bool = True,
                          fit_to_screen: bool = True) -> np.ndarray:
    """Draw detection boxes and optional labels onto a copy of ``img``.

    Parameters
    ----------
    img : np.ndarray
        HxWx3 uint8 RGB image to annotate. The original array is not
        modified; a copy is returned.
    detections : np.ndarray
        Nx6 array with rows ``(x1, y1, x2, y2, conf, cls)``. Coordinates
        may be absolute pixel values or normalized to [0,1].
    class_names : Dict[int,str], optional
        Mapping of class id to readable label used when ``show_class_label``
        is True.
    show_class_label : bool, optional
        If True draw the class label (or id) next to each box.
    show_conf : bool, optional
        If True append the confidence value to the label text.
    conf_thresh : float, optional
        Minimum confidence required to display a box.
    max_boxes : int, optional
        Maximum number of boxes to render.
    out_path : str, optional
        If provided the annotated image will be saved to this path.
    title : str, optional
        Window title used when displaying the image.
    wait : bool, optional
        If True, display a window and wait for a keypress before
        returning.
    fit_to_screen : bool, optional
        If True attempt to resize the display to fit the screen.

    Returns
    -------
    np.ndarray
        The annotated image (uint8 BGR).

    Raises
    ------
    ValueError
        If ``img`` is None or ``detections`` is not an (N,6) array.
    """

    if img is None:
        raise ValueError("img must be a valid image")

    # input is assumed to be RGB (pipeline convention); convert to BGR
    annotated_rgb = img.copy()
    annotated = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)

    dets = np.asarray(detections)
    if dets.ndim != 2 or dets.shape[1] < 6:
        raise ValueError("detections must be an array of shape (N,6)")

    # filter by confidence
    confs = dets[:, 4]
    keep = confs >= conf_thresh
    dets = dets[keep]

    # cap boxes
    dets = dets[:max_boxes]
    h, w = annotated.shape[:2]

    if dets.size > 0:
        coords = dets[:, :4].astype(float)

        # detect normalized coordinates (max <= 1.01)
        if coords.max() <= 1.01:
            coords[:, 0] *= w
            coords[:, 2] *= w
            coords[:, 1] *= h
            coords[:, 3] *= h

        # draw boxes
        for row in dets:
            x1, y1, x2, y2, conf, cls = row[:6]
            x1, y1, x2, y2 = int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))
            cls_id = int(round(cls))

            # color per class
            hue = (cls_id * 37) % 180
            color = tuple(int(c) for c in cv2.cvtColor(np.uint8([[[hue, 200, 200]]]), cv2.COLOR_HSV2BGR)[0, 0])

            # clamp coords
            x1 = max(0, min(w - 1, x1))
            x2 = max(0, min(w - 1, x2))
            y1 = max(0, min(h - 1, y1))
            y2 = max(0, min(h - 1, y2))

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            label = class_names.get(cls_id, str(cls_id)) if class_names else str(cls_id)
            parts: List[str] = []
            if show_class_label:
                parts.append(label)
            if show_conf:
                parts.append(f"{conf:.2f}")

            if parts:
                label_text = " ".join(parts)
                (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                # place background box
                bx1, by1 = x1, max(0, y1 - th - 6)
                bx2, by2 = x1 + tw + 8, by1 + th + 6
                cv2.rectangle(annotated, (bx1, by1), (bx2, by2), color, -1)
                cv2.putText(annotated, label_text, (bx1 + 4, by2 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # draw total count in top-right corner
    try:
        total_count = int(dets.shape[0])
    except Exception:
        total_count = 0

    count_text = f"Count: {total_count}"
    (ctw, cth), _ = cv2.getTextSize(count_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    pad_x = 8
    pad_y = 6
    bx2 = w - 6
    bx1 = max(0, bx2 - ctw - pad_x)
    by1 = 6
    by2 = by1 + cth + pad_y
    cv2.rectangle(annotated, (bx1, by1), (bx2, by2), (0, 0, 0), -1)
    cv2.putText(annotated, count_text, (bx1 + 4, by2 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    if out_path:
        cv2.imwrite(out_path, annotated)

    if wait:
        # Adapt display size to screen if requested
        display_img = annotated
        display_h, display_w = annotated.shape[:2]
        if fit_to_screen:
            try:
                import tkinter as tk
                root = tk.Tk()
                root.withdraw()
                screen_w = root.winfo_screenwidth()
                screen_h = root.winfo_screenheight()
                root.destroy()
                max_w = int(screen_w * 0.92)
                max_h = int(screen_h * 0.92)
                scale = min(max_w / display_w, max_h / display_h, 1.0)
                if scale < 1.0:
                    display_w = max(1, int(display_w * scale))
                    display_h = max(1, int(display_h * scale))
                    display_img = cv2.resize(annotated, (display_w, display_h), interpolation=cv2.INTER_AREA)
            except Exception:
                pass

        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        try:
            cv2.resizeWindow(title, display_w, display_h)
        except Exception:
            pass
        cv2.imshow(title, display_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return annotated

# ------------------------
# -- Detectors
# ------------------------


class FullImageObjectDetector(ObjectDetector):

    # -- Parameters
    conf_thresh: float = 0.05

    # -- Debug
    visualize_combined_dets = False

    # -- Model
    _model: Optional[Detector] = None

    def __init__(self, model_path: Optional[Path] = None) -> None:
        """ Constructor
        """
        self._model_path_str = ""
        self._model_path = model_path
        if model_path is not None:
            self._model = InferenceModelBuilder.build(model_path)
            self._model_path_str = str(model_path)

        self.timing: Dict[str, float] = {}

    def detect(self, image: cv2.Mat, config: ConfigStore) -> List[ObjDetectObservation]:
        """ Run model on image
        """
        # -- Check configuration
        if self._model == None or config.local_config.obj_detect_model != self._model_path_str:
            self._model_path = Path(config.local_config.obj_detect_model)
            self._model = InferenceModelBuilder.build(self._model_path)
            self._model_path_str = config.local_config.obj_detect_model

        input = np.zeros((1, 3, 640, 640), dtype=np.float32)
        image_orig_rgb = image
        image = image.copy()

        # Create scaled frame for model
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        if image.dtype == np.uint8:
            image = image.astype(np.float32)
            image /= 255.0

        scaled_height = int(640 / (image.shape[1] / image.shape[0]))
        bar_height = int((640 - scaled_height) / 2)
        image_scaled = cv2.resize(image, (640, scaled_height))
        image_scaled_chw = np.transpose(image_scaled, (2, 0, 1))

        input[0, :, bar_height: bar_height + scaled_height, 0:640] = image_scaled_chw

        start_time = time.time()
        dets = self._model(input)[0]
        self.timing["infer"] = time.time() - start_time

        # Scale detection coordinates in (x0,y0, x1,y1) format to original image shape
        dets[:, 1:4:2] = (dets[:, 1:4:2]-bar_height)/scaled_height * image.shape[0]
        dets[:, 0:4:2] = dets[:, 0:4:2]/640.0 * image.shape[1]

        if self.visualize_combined_dets:
            _visualize_detections(image_orig_rgb, dets, conf_thresh=self.conf_thresh, wait=True)

        observations: List[ObjDetectObservation] = _postprocess_detections(dets,
                                                                           config.local_config.camera_matrix,
                                                                           config.local_config.distortion_coefficients,
                                                                           conf_thresh=self.conf_thresh)
        return observations


class SlicedObjectDetector(ObjectDetector):

    @dataclass
    class SliceInfo:
        # Location of slice on the original image (Row, Column, H, W)
        img_slice_pos_rchw: Tuple[int, int, int, int]
        s2i_scale: float                                     # Slice to image scaling factor
        # Offset of detections in slice coordinates to account for border
        border_offset_rc: Tuple[int, int]
        # Edges to filter and how many rows/cols to include(Left, Right, Top, Bottom)
        filter_edges: List[int] = field(default_factory=list)
        filter_classes: List[int] = field(default_factory=list)   # Ids of classes to filter

    # -- Parameters
    edge_filter_width: int = 100
    nchan: int = 3
    imgsz: Tuple = (960, 1280, nchan)
    size: int = 640  # model-native input size

    run_nms: bool = True
    run_edge_filter: bool = True
    nms_iou_thresh: float = 0.80
    nms_conf_thresh: float = 0.05

    # -- Debug
    visualize_slice_intermediates = False
    visualize_combined_dets = False
    next_print_time = -1
    print_interval_sec = 3

    # -- Helper variables
    scale = imgsz[1]/size  # scale factor from full image to model native size
    det_offset_rc = (int((size - size * (imgsz[0] / imgsz[1])) / 2),  0)  # padding offset in model native size
    assert imgsz[1] > imgsz[0], "Vertically oriented images not supported yet"

    # ---------------------------
    # Slice Configurations
    # ---------------------------
    slice6_config = [SliceInfo((0, 0, size, size), 1, (0, 0), [0, edge_filter_width, 0, edge_filter_width], [0]),
                     SliceInfo((0, int(imgsz[1]/2-size/2), size, size), 1, (0, 0),
                               [edge_filter_width, edge_filter_width, 0, edge_filter_width], [0]),
                     SliceInfo((0, imgsz[1]-size, size, size), 1, (0, 0),
                               [edge_filter_width, 0, 0, edge_filter_width], [0]),
                     SliceInfo((imgsz[0]-size-1, 0, size, size), 1, (0, 0),
                               [0, edge_filter_width, edge_filter_width, 0], [0]),
                     SliceInfo((imgsz[0]-size-1, int(imgsz[1]/2 - size/2), size, size), 1, (0, 0),
                               [edge_filter_width, edge_filter_width, edge_filter_width, 0], [0]),
                     SliceInfo((imgsz[0]-size-1, int(imgsz[1] - size), size, size), 1,
                               (0, 0), [edge_filter_width, 0, edge_filter_width, 0], [0])
                     ]

    slice3_plusfull_config = [SliceInfo((int(imgsz[0] / 2 - size / 2), 0, size, size), 1, (0, 0), [0, edge_filter_width, 0, edge_filter_width], [0]),
                              SliceInfo((int(imgsz[0] / 2 - size / 2), int(imgsz[1]/2-size/2), size, size),
                                        1, (0, 0), [edge_filter_width, edge_filter_width, 0, edge_filter_width], [0]),
                              SliceInfo((int(imgsz[0] / 2 - size / 2), imgsz[1]-size, size, size),
                                        1, (0, 0), [edge_filter_width, 0, 0, edge_filter_width], [0]),
                              SliceInfo((0, 0, imgsz[0], imgsz[1]), scale, det_offset_rc,
                                        [0, 0, int(imgsz[0] * (0.6)), 0], [0])
                              ]

    slice6_plusfull_config = [SliceInfo((0, 0, size, size), 1, (0, 0), [0, edge_filter_width, 0, edge_filter_width], [0]),
                              SliceInfo((0, int(imgsz[1]/2-size/2), size, size),  1, (0, 0),
                                        [edge_filter_width, edge_filter_width, 0, edge_filter_width], [0]),
                              SliceInfo((0, imgsz[1]-size, size, size), 1, (0, 0),
                                        [edge_filter_width, 0, 0, edge_filter_width], [0]),
                              SliceInfo((imgsz[0]-size-1, 0, size, size), 1, (0, 0),
                                        [0, edge_filter_width, edge_filter_width, 0], [0]),
                              SliceInfo((imgsz[0]-size-1, int(imgsz[1]/2 - size/2), size, size), 1, (0, 0),
                                        [edge_filter_width, edge_filter_width, edge_filter_width, 0], [0]),
                              SliceInfo((imgsz[0]-size-1, int(imgsz[1] - size), size, size), 1,
                                        (0, 0), [edge_filter_width, 0, edge_filter_width, 0], [0]),
                              SliceInfo((0, 0, imgsz[0], imgsz[1]), scale, det_offset_rc,
                                        [0, 0, imgsz[0], 0], [0])  # Robot only
                              ]

    robotonly_config = [SliceInfo((0, 0, imgsz[0], imgsz[1]), scale, det_offset_rc, [0, 0, imgsz[0], 0], [0])  # Robot only
                        ]

    full_config = [SliceInfo((0, 0,  imgsz[0], imgsz[1]), scale, det_offset_rc, [0, 0, 0, 0], [])]
    # ---------------------------

    # -- Model
    _model: Optional[Detector] = None
    _tmp_robot_model : Optional[Detector] = None
    _robot_model_path = Path(__file__).resolve().parents[1] / "robots_v2_batch1.mlpackage"

    def __init__(self, model_path: Optional[Path] = None):
        """ Constructor

        Parameters
        ----------
        model_path : Path
            Path to a detection model file. The constructor will attempt to
            load the model (PyTorch/Ultralytics by default) via
            :meth:`_load_model`.

        Side effects
        -----------
        - Loads the model and stores it on ``self._model``.
        - Initializes ``self.timing`` for profiling stages.
        - Selects default slice bounding boxes and edge overlap map.
        """
        assert SlicedObjectDetector.size % 2 == 0, "Model native size must be even"

        self._model_path_str = ""
        self._model_path = model_path
        if model_path is not None:
            self._model = InferenceModelBuilder.build(model_path)
            self._model_path_str = str(model_path)

        if self._robot_model_path is not None:
            self._tmp_robot_model = InferenceModelBuilder.build(self._robot_model_path)
 	
        self.slices = SlicedObjectDetector.slice3_plusfull_config

        # -- Statistics
        self.timing: Dict[str, float] = {}
        self.next_print_time = -1
        self.last_print_time = -1
        self.num_frames = 0

    def _slice_image(self, slices : List[SliceInfo], image: cv2.Mat) -> np.ndarray:
        """Split an RGB image into the configured overlapping tiles.

        The detector operates on fixed-size tiles; this helper extracts the
        tiles defined in ``slices`` and returns both the slice
        bounding boxes and a batch array formatted for model input.

        Parameters
        ----------
        slices : List[SliceInfo]
            Meta data about each slice

        image : cv2.Mat
            Input image (RGB). Expected shape matches ``self.imgsz``.

        Returns
        -------
        Numpy array of shape (N, 3, H, W) with the tile pixels arranged as
        channel-first float32 values.

        Raises
        ------
        AssertionError
            If ``image.shape`` does not match ``self.imgsz``.
        """

        assert image.shape == self.imgsz, "Unsupported image size"

        img_batch = np.zeros((len(slices), 3, self.size, self.size), dtype=np.float32)
        for idx, slice_info in enumerate(slices):
            (r, c, h, w) = slice_info.img_slice_pos_rchw

            if h == w and h == self.size:
                img_batch[idx] = np.transpose(image[r:(r+h), c:(c+w)], (2, 0, 1))
            else:

                # Resize keeping aspect ratio
                slice_image = cv2.resize(image[r:(r+h), c:(c+w)], (self.size, int(h/slice_info.s2i_scale)))

                # Pad image if necessary
                if h != w:
                    border_type = cv2.BORDER_CONSTANT
                    border_color = (0, 0, 0)
                    slice_image = cv2.copyMakeBorder(src=slice_image,
                                                     top=slice_info.border_offset_rc[0],
                                                     bottom=slice_info.border_offset_rc[0],
                                                     left=slice_info.border_offset_rc[1],
                                                     right=slice_info.border_offset_rc[1],
                                                     borderType=border_type,
                                                     value=border_color)

                # (H,W,Ch) -> (Ch,H,W)
                img_batch[idx] = np.transpose(slice_image, (2, 0, 1))

        return img_batch

    def _filter_edge(self, dets: List[np.ndarray]) -> List[np.ndarray]:
        """Filter detections that fall within the configured edge overlap margins.

        Many of the tiles overlap; to avoid duplicate detections near tile
        borders we drop boxes that lie within ``self.edge_filter_width`` of
        the configured edges for a given slice. The per-slice edge mask is
        read from ``self.edge_overlap`` and uses the order
        ``[left, right, top, bottom]``.  

        Parameters
        ----------
        dets : List[np.ndarray]
            A list with one array per slice. Each array is expected to be
            Nx6 with columns (x1, y1, x2, y2, conf, cls) in slice-local
            coordinates.

        Returns
        -------
        List[np.ndarray]
            The same list of arrays but with boxes near configured edges
            removed according to the overlap rules.
        """

        for idx, slice_dets in enumerate(dets):

            keep = np.ones(slice_dets.shape[0], dtype=bool)
            edges = [e / self.slices[idx].s2i_scale for e in self.slices[idx].filter_edges]
            classes = self.slices[idx].filter_classes
            (_, _, h, w) = self.slices[idx].img_slice_pos_rchw
            offset = self.slices[idx].border_offset_rc

            x1 = dets[idx][:, 0]
            y1 = dets[idx][:, 1]
            x2 = dets[idx][:, 2]
            y2 = dets[idx][:, 3]

            # Left: drop boxes that start within `edge_filter` pixels of left edge
            if edges[0]:
                keep &= (x1 >= (edges[0] + offset[1]))

            # Right: drop boxes that end within `edge_filter` pixels of right edge
            if edges[1]:
                keep &= (x2 <= (w - (edges[1] + offset[1])))

            # Top: drop boxes that start within `edge_filter` pixels of top edge
            if edges[2]:
                keep &= (y1 >= (edges[2] + offset[0]))

            # Bottom: drop boxes that end within `edge_filter` pixels of bottom edge
            if edges[3]:
                keep &= (y2 <= (h - (edges[3] + offset[0])))

            # Keep any detections that are not in a class that is filtered
            ignore_mask = ~np.isin(element=slice_dets[:, 5], test_elements=classes)
            keep |= ignore_mask

            dets[idx] = dets[idx][keep]

        return dets

    def _merge_dets(self, slices: List[SliceInfo], dets: List[np.ndarray]) -> np.ndarray:
        """Merge per-slice detections back into full-image coordinates.

        This function optionally applies the edge filter to per-slice
        detections, concatenates all slice detections, offsets their box
        coordinates to the full-image coordinate system and finally runs
        class-aware non-maximum suppression (NMS) if enabled.

        Parameters
        ----------
        slices : List[SliceInfo]
            List of slice information that is used to compute the
            offset and scaling to apply to each slice's detections.
        dets : List[np.ndarray]
            Per-slice detection arrays (one per slice) with rows of the
            form ``[x1, y1, x2, y2, conf, cls]`` in slice-local pixels.

        Returns
        -------
        np.ndarray
            A single (M,6) array of merged detections in full-image
            coordinates. If no detections are present an empty array with
            shape (0,6) or a zero-row array is returned.
        """

        if SlicedObjectDetector.run_edge_filter == True:
            dets = self._filter_edge(dets)

        det_count = [d.shape[0] for d in dets]
        total_dets = np.sum(det_count, dtype=np.int32)
        merged_dets = np.zeros((total_dets, 6), dtype=np.float32)
        det_idx = 0
        for slice_idx, slice in enumerate(slices):
            offset = np.array([slice.img_slice_pos_rchw[1] - slice.border_offset_rc[1],
                               slice.img_slice_pos_rchw[0] - slice.border_offset_rc[0],
                               slice.img_slice_pos_rchw[1] - slice.border_offset_rc[1],
                               slice.img_slice_pos_rchw[0] - slice.border_offset_rc[0]])

            merged_dets[det_idx:det_idx + det_count[slice_idx]] = dets[slice_idx]
            merged_dets[det_idx:det_idx + det_count[slice_idx], 0:4] += offset
            merged_dets[det_idx:det_idx + det_count[slice_idx], 0:4] *= slice.s2i_scale
            det_idx += det_count[slice_idx]

        if total_dets == 0:
            return merged_dets

        # run class-aware non-max suppression and confidence thresholding
        if SlicedObjectDetector.run_nms == True:
            merged_dets = self._nms(merged_dets, iou_thresh=SlicedObjectDetector.nms_iou_thresh,
                                    conf_thresh=SlicedObjectDetector.nms_conf_thresh)

        return merged_dets

    def _nms(self, dets: np.ndarray, iou_thresh: float = 0.45, conf_thresh: float = 0.10) -> np.ndarray:
        """Perform class-aware non-maximum suppression (NMS).

        Parameters
        ----------
        dets : np.ndarray
            Array of detections with shape (N,6) where each row is
            ``x1, y1, x2, y2, conf, cls``. Coordinates may be in pixels.
        iou_thresh : float, optional
            IoU threshold used to suppress overlapping boxes of the same
            class (default 0.45). Boxes with IoU > iou_thresh are
            suppressed.
        conf_thresh : float, optional
            Minimum confidence required to keep a detection (default
            0.10). Detections with ``conf < conf_thresh`` are filtered
            before NMS.

        Returns
        -------
        np.ndarray
            The filtered detections in the same (x1,y1,x2,y2,conf,cls)
            format.
        """
        if dets is None or len(dets) == 0:
            return dets

        dets = np.asarray(dets).astype(np.float32)

        # filter by confidence
        keep_mask = dets[:, 4] >= conf_thresh
        dets = dets[keep_mask]
        if dets.shape[0] == 0:
            return dets

        # ensure box format and sane coords
        x1 = dets[:, 0].astype(float)
        y1 = dets[:, 1].astype(float)
        x2 = dets[:, 2].astype(float)
        y2 = dets[:, 3].astype(float)
        scores = dets[:, 4].astype(float)
        classes = dets[:, 5].astype(int)

        # clamp boxes to valid ranges (swap if necessary)
        x1c = np.minimum(x1, x2)
        x2c = np.maximum(x1, x2)
        y1c = np.minimum(y1, y2)
        y2c = np.maximum(y1, y2)

        out_boxes = []

        unique_classes = np.unique(classes)
        for cls in unique_classes:
            inds = np.where(classes == cls)[0]
            if inds.size == 0:
                continue

            cls_x1 = x1c[inds]
            cls_y1 = y1c[inds]
            cls_x2 = x2c[inds]
            cls_y2 = y2c[inds]
            cls_scores = scores[inds]
            cls_dets = dets[inds]

            # sort by score desc
            order = cls_scores.argsort()[::-1]

            keep = []
            while order.size > 0:
                i = order[0]
                keep.append(i)

                xx1 = np.maximum(cls_x1[i], cls_x1[order[1:]])
                yy1 = np.maximum(cls_y1[i], cls_y1[order[1:]])
                xx2 = np.minimum(cls_x2[i], cls_x2[order[1:]])
                yy2 = np.minimum(cls_y2[i], cls_y2[order[1:]])

                w = np.maximum(0.0, xx2 - xx1 + 1.0)
                h = np.maximum(0.0, yy2 - yy1 + 1.0)
                inter = w * h

                area_i = (cls_x2[i] - cls_x1[i] + 1.0) * (cls_y2[i] - cls_y1[i] + 1.0)
                area_others = (cls_x2[order[1:]] - cls_x1[order[1:]] + 1.0) * \
                    (cls_y2[order[1:]] - cls_y1[order[1:]] + 1.0)

                union = area_i + area_others - inter
                iou = inter / np.maximum(union, 1e-6)

                # keep boxes with IoU <= threshold
                inds_to_keep = np.where(iou <= iou_thresh)[0]
                order = order[inds_to_keep + 1]

            # append kept boxes for this class
            out_boxes.extend(cls_dets[keep].tolist())

        if len(out_boxes) == 0:
            return np.zeros((0, 6), dtype=np.float32)

        out_arr = np.asarray(out_boxes, dtype=np.float32)
        return out_arr

    def detect(self, image: cv2.Mat, config: ConfigStore) -> List[ObjDetectObservation]:
        """Run batched object detection on an input RGB image.

        The method performs the following high-level steps:
        1. Split input into overlapping tiles via :meth:`_slice_image`.
        2. Run the model on the tile batch and collect per-tile detections.
        3. Merge per-tile detections into full-image coordinates using
        :meth:`_merge_dets` and (optionally) visualize results.

        Parameters
        ----------
        image : cv2.Mat
            Input image in RGB color order. Expected shape is ``(960, 1280, 3)``.

        Returns
        -------
        None
            Currently this function displays visualizations of the
            intermediate and merged detections and prints timing
            information. It does not construct nor return
            ``ObjDetectObservation`` instances in the present
            implementation.
        """
        # -- Check configuration
        if self._model == None or config.local_config.obj_detect_model != self._model_path_str:
            self._model_path = Path(config.local_config.obj_detect_model)
            self._model = InferenceModelBuilder.build(self._model_path)
            self._model_path_str = config.local_config.obj_detect_model

        # -- Slice image
        start_time = time.time()
        img_batch = self._slice_image(self.slices, image)
        img_batch /= 255.0
        nbatches = len(self.slices)
        self.timing["slice"] = time.time() - start_time

        # -- Run inference
        start_time = time.time()
        dets = self._model(img_batch)
        self.timing["infer"] = time.time() - start_time

        #-- Robot model     
        if self._tmp_robot_model is not None: 
            robot_slice_conf = SlicedObjectDetector.full_config
            robot_img = self._slice_image(robot_slice_conf, image)
            robot_img /= 255.0
            robot_dets = self._tmp_robot_model(robot_img)
            robot_dets = self._merge_dets(robot_slice_conf, robot_dets)
            robot_dets[:,5] = 1 # override class id
            #_visualize_detections(image, robot_dets,  title=f"Robot Annotations", wait=True, out_path=f"tmp/output/robot.png")
        else:
            robot_dets = np.zeros((0,6), dtype=np.float32)

        if self.visualize_slice_intermediates:
            for i in range(nbatches):
                slice_img = (np.transpose(img_batch[i], (1, 2, 0))*255).astype(np.uint8)
                _visualize_detections(
                    slice_img, dets[i],  title=f"Slice{i} Annotations", wait=True, out_path=f"tmp/output/{i}.png")

        start_time = time.time()
        merged_dets = self._merge_dets(self.slices, dets)
        if len(robot_dets)>0:
            merged_dets = np.concatenate((merged_dets, robot_dets), axis=0)
        self.timing["merge"] = time.time() - start_time

        if self.visualize_combined_dets:
            annoted_img = _visualize_detections(
                image, merged_dets,  title="Merged Annotations",  wait=True, out_path="tmp/final.png")

        observations: List[ObjDetectObservation] = _postprocess_detections(
            merged_dets, config.local_config.camera_matrix, config.local_config.distortion_coefficients)

        self.num_frames += 1
        curr_time = time.time()
        if self.next_print_time < curr_time:

            timing = {f"'{key}': {value:.3f}" for key, value in self.timing.items()}

            if self.last_print_time == -1:
                elapsed = "-"
                fps = "-"
            else:
                elapsed = curr_time - self.last_print_time
                fps = self.num_frames/elapsed

            logger.info(f"Timing of last inference: {timing}")
            logger.info(f"Period stats: [num_frames: {self.num_frames}, elapsed: {elapsed}, fps: {fps}]")
            self.next_print_time = curr_time + self.print_interval_sec
            self.last_print_time = curr_time
            self.num_frames = 0

        return observations

    def _visualize_slices(self, slice_bbox: List, imgs: np.ndarray, out_path: Optional[str] = None, wait: bool = True):
        """Compose and display (or save) image slices arranged into a canvas.

        Parameters
        ----------
        slice_bbox : List
            List of slice bounding boxes in the form ``[row, col, height, width]``.
        imgs : np.ndarray
            Array shaped ``(N, 3, H, W)`` containing tile pixels in channel-first
            order.
        out_path : str, optional
            If provided, the composed canvas will be written to this path.
        wait : bool, optional
            When True (default) display the image window and wait for a
            keypress before returning.

        Returns
        -------
        np.ndarray
            The composed canvas image (H, W, 3) in uint8 BGR order.
        """

        # infer canvas size from bounding boxes
        max_row = max(x + h for (x, y, h, w) in slice_bbox)
        max_col = max(y + w for (x, y, h, w) in slice_bbox)
        canvas = np.zeros((max_row, max_col, 3), dtype=np.uint8)

        for idx, (x, y, h, w) in enumerate(slice_bbox):
            # convert from (3,H,W) -> (H,W,3)
            sl = imgs[idx]
            sl_img = np.transpose(sl, (1, 2, 0))
            # ensure uint8 for display
            if sl_img.dtype != np.uint8:
                sl_img = np.clip(sl_img, 0, 255).astype(np.uint8)

            # blend into canvas (handles overlaps)
            roi = canvas[x:(x + h), y:(y + w)]
            if roi.shape == sl_img.shape and roi.sum() != 0:
                blended = cv2.addWeighted(roi, 0.5, sl_img, 0.5, 0)
                canvas[x:(x + h), y:(y + w)] = blended
            else:
                canvas[x:(x + h), y:(y + w)] = sl_img

            # draw bbox and label
            cv2.rectangle(canvas, (y, x), (y + w - 1, x + h - 1), (0, 255, 0), 2)
            cv2.putText(canvas, f"{idx}", (y + 6, x + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if out_path:
            cv2.imwrite(out_path, canvas)

        if wait:
            cv2.imshow("slices", canvas)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return canvas
