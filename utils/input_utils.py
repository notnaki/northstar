# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

"""Utilities for creating iterable frame sources (video files or image folders).

Provides a lightweight Protocol describing an iterable frame source and a
factory to build an implementation from provided path.
"""

from __future__ import annotations

from typing import Iterator, Protocol, runtime_checkable
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from config.config import ConfigStore


@runtime_checkable
class FrameSource(Protocol):
    """Protocol for an iterable source of frames.

    Usage:
            for frame in source:
                    # frame is a numpy.ndarray (BGR image)
                            ...
    """

    def __iter__(self) -> Iterator[np.ndarray]:
        ...

    def close(self) -> None:
        """Close any resources (optional)."""


class VideoFileSource:
    """Iterate frames from a video file using OpenCV VideoCapture."""

    def __init__(self, path: str) -> None:
        self._path = path
        self._cap: Optional[cv2.VideoCapture] = None

    def __iter__(self) -> Iterator[np.ndarray]:
        self._cap = cv2.VideoCapture(self._path)
        if not self._cap.isOpened():
            raise IOError(f"Unable to open video file: {self._path}")

        while True:
            ok, frame = self._cap.read()
            if not ok:
                break
            yield frame

        self.close()

    def close(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None


class DirectoryImageSource:
    """Iterate image files from a directory in sorted order."""

    IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"}

    def __init__(self, path: str) -> None:
        self._path = Path(path)
        if not self._path.exists() or not self._path.is_dir():
            raise IOError(f"Image directory not found: {path}")

    def __iter__(self) -> Iterator[np.ndarray]:
        files = sorted(f for f in self._path.iterdir() if f.suffix.lower() in self.IMAGE_EXTS)
        for p in files:
            img = cv2.imread(str(p))
            if img is None:
                # skip unreadable files
                continue
            yield img

    def close(self) -> None:
        # nothing to close for directory source
        return None


def build_source(path: Path) -> FrameSource:
    """Create a FrameSource based on the provided configuration.

    Behavior:
    - If `path` is a path to a file with a video extension,
      returns a `VideoFileSource`.
    - If it's a directory, returns a `DirectoryImageSource`.
    - Raises ValueError when `video_folder` is empty or points to an unsupported path.
    """

    if path is None or path == "":
        raise ValueError("path is not set in ConfigStore")

    if path.is_file():
        return VideoFileSource(str(path))
    elif path.is_dir():
        return DirectoryImageSource(str(path))
    else:
        raise IOError(f"Path does not exist or unsupported type: {path}")


class FrameStreamer:
    """Stream frames from a `FrameSource` with repetition and maximum limit.

    Example:
            streamer = FrameStreamer(source, repeat=2, max_frames=100)
            for frame in streamer:
                    # frame will be each source frame repeated twice
                    ...

    Args:
            source: FrameSource to iterate.
            repeat: number of times to repeat each source frame (>=1).
            max_frames: optional max frames to yield (None or negative for unlimited).
    """

    def __init__(self, source: FrameSource, repeat: int = 1, max_frames: Optional[int] = None, step: int = 1) -> None:
        if repeat < 1:
            raise ValueError("repeat must be >= 1")
        if step < 1:
            raise ValueError("step must be >= 1")
        self._source = source
        self._repeat = int(repeat)
        self._step = int(step)
        self._max_frames: Optional[int] = None if max_frames is None or max_frames < 0 else int(max_frames)
        self._iter = iter(self._source)
        self._current_frame: Optional[np.ndarray] = None
        self._current_repeat = 0
        self._sent = 0

    def __iter__(self) -> "FrameStreamer":
        return self

    def __next__(self) -> np.ndarray:
        if self._max_frames is not None and self._sent >= self._max_frames:
            raise StopIteration

        if self._current_frame is None or self._current_repeat >= self._repeat:
            # fetch next source frame; if advancing from a previous frame,
            # skip (step-1) frames from the underlying source
            try:
                if self._current_frame is not None and self._step > 1:
                    for _ in range(self._step - 1):
                        next(self._iter)
                self._current_frame = next(self._iter)
                self._current_repeat = 0
            except StopIteration:
                self.close()
                raise

        frame = self._current_frame
        self._current_repeat += 1
        self._sent += 1
        return frame

    def next_frame(self) -> Optional[np.ndarray]:
        """Return the next frame or `None` if the source is exhausted."""
        try:
            return next(self)
        except StopIteration:
            return None

    def close(self) -> None:
        try:
            self._source.close()
        finally:
            # break iteration references
            self._iter = iter(())
            self._current_frame = None
