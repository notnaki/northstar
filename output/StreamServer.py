# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json
import random
import socketserver
import string
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Dict
from urllib.parse import parse_qs, urlparse

import cv2


CLIENT_COUNTS: Dict[str, int] = {}


class StreamServer:
    """Interface for outputing camera frames."""

    def start(self, port: int) -> None:
        """Starts the output stream."""
        raise NotImplementedError

    def set_frame(self, frame: cv2.Mat) -> None:
        """Sets the frame to serve."""
        raise NotImplementedError


class MjpegServer(StreamServer):
    _frame: cv2.Mat
    _has_frame: bool = False
    _frame_seq: int = 0
    _uuid: str = ""
    _telemetry: Dict[str, Any] = {}
    _default_fps: int = 15
    _default_quality: int = 50
    _default_width: int = 320

    def __init__(
        self, default_fps: int = 15, default_quality: int = 50, default_width: int = 320
    ):
        self._default_fps = default_fps if default_fps > 0 else 15
        self._default_quality = (
            max(1, min(100, default_quality)) if default_quality > 0 else 50
        )
        self._default_width = default_width if default_width > 0 else 320

    def _make_handler(self_mjpeg, uuid: str):  # type: ignore
        class StreamingHandler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def do_GET(self):
                global CLIENT_COUNTS
                try:
                    self._do_GET_inner()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass  # Client disconnected

            def _do_GET_inner(self):
                global CLIENT_COUNTS
                path = urlparse(self.path).path
                if path == "/api/status":
                    data = json.dumps(self_mjpeg._telemetry).encode("utf-8")
                    self.send_response_only(200)
                    self.send_header("Server", "CameraServer/1.0")
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control", "no-cache")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(data)
                elif path == "/stream.mjpg":
                    parsed = urlparse(self.path)
                    params = parse_qs(parsed.query)
                    stream_fps = int(
                        params.get("fps", [str(self_mjpeg._default_fps)])[0]
                    )
                    jpeg_quality = int(
                        params.get("compression", [str(self_mjpeg._default_quality)])[0]
                    )
                    # Parse resolution param (e.g. "320x240")
                    res_str = params.get("resolution", [""])[0]
                    if res_str and "x" in res_str:
                        try:
                            target_width = int(res_str.split("x")[0])
                        except ValueError:
                            target_width = self_mjpeg._default_width
                    else:
                        target_width = self_mjpeg._default_width
                    if stream_fps <= 0:
                        stream_fps = self_mjpeg._default_fps
                    if jpeg_quality < 0 or jpeg_quality > 100:
                        jpeg_quality = self_mjpeg._default_quality
                    if target_width <= 0:
                        target_width = self_mjpeg._default_width
                    min_interval = 1.0 / stream_fps
                    self.send_response_only(200)
                    self.send_header("Connection", "close")
                    self.send_header("Server", "CameraServer/1.0")
                    self.send_header(
                        "Cache-Control",
                        "no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0",
                    )
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Expires", "Mon, 3 Jan 2000 12:34:56 GMT")
                    self.send_header(
                        "Content-Type",
                        "multipart/x-mixed-replace;boundary=boundarydonotcross",
                    )
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.flush()
                    try:
                        CLIENT_COUNTS[uuid] += 1
                        last_seq = -1
                        last_send = 0.0
                        while True:
                            now = time.monotonic()
                            if (
                                not self_mjpeg._has_frame
                                or self_mjpeg._frame_seq == last_seq
                                or (now - last_send) < min_interval
                            ):
                                time.sleep(0.01)
                                continue
                            last_seq = self_mjpeg._frame_seq
                            last_send = now
                            frame = self_mjpeg._frame
                            h, w = frame.shape[:2]
                            if w > target_width:
                                scale = target_width / w
                                frame = cv2.resize(
                                    frame,
                                    (target_width, int(h * scale)),
                                    interpolation=cv2.INTER_AREA,
                                )
                            _, frame_data = cv2.imencode(
                                ".jpg",
                                frame,
                                [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
                            )
                            frame_data = frame_data.tobytes()

                            self.wfile.write(
                                b"\r\n--boundarydonotcross\r\n"
                                b"Content-Type: image/jpeg\r\n"
                                + f"Content-Length: {len(frame_data)}\r\n".encode()
                                + b"\r\n"
                            )
                            self.wfile.write(frame_data)
                            self.wfile.flush()
                    except Exception as e:
                        print(
                            "Removed streaming client %s: %s",
                            self.client_address,
                            str(e),
                        )
                    finally:
                        CLIENT_COUNTS[uuid] -= 1
                else:
                    self.send_error(404)
                    self.end_headers()

            def log_message(self, format, *args):
                pass  # Suppress HTTP request logs

        return StreamingHandler

    class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
        allow_reuse_address = True
        daemon_threads = True

    def _run(self, port: int) -> None:
        self._uuid = "".join(random.choice(string.ascii_lowercase) for i in range(12))
        CLIENT_COUNTS[self._uuid] = 0
        server = self.StreamingServer(("", port), self._make_handler(self._uuid))
        server.serve_forever()

    def start(self, port: int) -> None:
        threading.Thread(target=self._run, daemon=True, args=(port,)).start()

    def set_frame(self, frame: cv2.Mat) -> None:
        self._frame = frame
        self._has_frame = True
        self._frame_seq += 1

    def set_telemetry(self, telemetry: Dict[str, Any]) -> None:
        self._telemetry = telemetry

    def get_client_count(self) -> int:
        if len(self._uuid) > 0:
            return CLIENT_COUNTS[self._uuid]
        else:
            return 0
