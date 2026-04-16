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
from io import BytesIO
from typing import Any, Dict

import cv2
from PIL import Image


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
    _uuid: str = ""
    _telemetry: Dict[str, Any] = {}

    def _make_handler(self_mjpeg, uuid: str):  # type: ignore
        class StreamingHandler(BaseHTTPRequestHandler):
            DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Northstar</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  :root {
    --bg: #0a0a0a; --surface: #141414; --border: #1e1e1e;
    --text: #e0e0e0; --dim: #666; --accent: #22c55e;
    --red: #ef4444; --blue: #3b82f6; --yellow: #eab308;
  }
  body { background: var(--bg); color: var(--text); font-family: 'SF Mono', 'Cascadia Code', 'Fira Code', monospace; font-size: 13px; height: 100vh; overflow: hidden; }
  .layout { display: grid; grid-template-columns: 1fr 280px; grid-template-rows: 48px 1fr auto; height: 100vh; }
  .topbar { grid-column: 1 / -1; background: var(--surface); border-bottom: 1px solid var(--border); display: flex; align-items: center; padding: 0 16px; gap: 12px; }
  .topbar .logo { font-size: 15px; font-weight: 700; letter-spacing: 0.5px; color: var(--accent); }
  .topbar .logo span { color: var(--dim); font-weight: 400; }
  .topbar .status { margin-left: auto; display: flex; align-items: center; gap: 8px; }
  .topbar .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--red); }
  .topbar .dot.connected { background: var(--accent); }
  .topbar .fps { color: var(--dim); }
  .feed { position: relative; background: #000; overflow: hidden; display: flex; align-items: center; justify-content: center; }
  .feed img { max-width: 100%; max-height: 100%; object-fit: contain; }
  .feed .no-feed { color: var(--dim); font-size: 14px; }
  .field-wrap { grid-column: 1; background: var(--surface); border-top: 1px solid var(--border); padding: 10px; display: flex; align-items: center; justify-content: center; }
  .field-wrap canvas { border: 1px solid var(--border); border-radius: 4px; }
  .sidebar { grid-column: 2; grid-row: 2 / 4; background: var(--surface); border-left: 1px solid var(--border); overflow-y: auto; padding: 0; }
  .section { border-bottom: 1px solid var(--border); }
  .section-header { padding: 10px 14px; font-size: 10px; font-weight: 600; text-transform: uppercase; letter-spacing: 1.2px; color: var(--dim); background: var(--bg); }
  .section-body { padding: 8px 14px 12px; }
  .row { display: flex; justify-content: space-between; align-items: center; padding: 3px 0; }
  .row .label { color: var(--dim); }
  .row .value { color: var(--text); font-weight: 500; }
  .row .value.green { color: var(--accent); }
  .row .value.red { color: var(--red); }
  .row .value.blue { color: var(--blue); }
  .row .value.yellow { color: var(--yellow); }
  .tag-list { display: flex; flex-wrap: wrap; gap: 4px; margin-top: 4px; }
  .tag-badge { background: var(--bg); border: 1px solid var(--border); border-radius: 4px; padding: 2px 8px; font-size: 12px; font-weight: 500; }
  .tag-badge.seen { border-color: var(--accent); color: var(--accent); }
  .pose-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 4px; margin-top: 6px; }
  .pose-cell { background: var(--bg); border-radius: 4px; padding: 6px; text-align: center; }
  .pose-cell .axis { font-size: 10px; font-weight: 600; color: var(--dim); margin-bottom: 2px; }
  .pose-cell .val { font-size: 13px; font-weight: 600; }
  .pose-cell .val.x { color: var(--red); }
  .pose-cell .val.y { color: var(--accent); }
  .pose-cell .val.z { color: var(--blue); }
  .config-row { padding: 3px 0; color: var(--dim); font-size: 12px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .config-row span { color: var(--text); }
  .no-layout { color: var(--dim); font-size: 11px; text-align: center; padding: 8px 0; }
</style>
</head>
<body>
<div class="layout">
  <div class="topbar">
    <div class="logo">NORTHSTAR <span>// 6328</span></div>
    <div class="status">
      <span class="fps" id="fps">-- fps</span>
      <div class="dot" id="statusDot"></div>
    </div>
  </div>
  <div class="feed">
    <img src="stream.mjpg" id="feedImg" onerror="this.style.display='none'" onload="this.style.display='block'" />
    <div class="no-feed" id="noFeed">NO CAMERA FEED</div>
  </div>
  <div class="field-wrap">
    <canvas id="fieldCanvas" width="580" height="290"></canvas>
  </div>
  <div class="sidebar">
    <div class="section">
      <div class="section-header">Detection</div>
      <div class="section-body">
        <div class="row"><span class="label">Tags</span><span class="value" id="tagCount">0</span></div>
        <div class="row"><span class="label">Type</span><span class="value" id="solveType">--</span></div>
        <div class="row"><span class="label">Error</span><span class="value" id="reprojError">--</span></div>
        <div class="tag-list" id="tagList"></div>
      </div>
    </div>
    <div class="section">
      <div class="section-header">Camera Pose (field)</div>
      <div class="section-body">
        <div id="poseStatus"></div>
        <div class="pose-grid">
          <div class="pose-cell"><div class="axis">X</div><div class="val x" id="poseX">--</div></div>
          <div class="pose-cell"><div class="axis">Y</div><div class="val y" id="poseY">--</div></div>
          <div class="pose-cell"><div class="axis">Z</div><div class="val z" id="poseZ">--</div></div>
        </div>
        <div class="pose-grid" style="margin-top:4px">
          <div class="pose-cell"><div class="axis">Roll</div><div class="val" id="poseRoll">--</div></div>
          <div class="pose-cell"><div class="axis">Pitch</div><div class="val" id="posePitch">--</div></div>
          <div class="pose-cell"><div class="axis">Yaw</div><div class="val" id="poseYaw">--</div></div>
        </div>
      </div>
    </div>
    <div class="section">
      <div class="section-header">Config</div>
      <div class="section-body">
        <div class="config-row">device: <span id="cfgDevice">--</span></div>
        <div class="config-row">camera: <span id="cfgCamera">--</span></div>
        <div class="config-row">resolution: <span id="cfgRes">--</span></div>
        <div class="config-row">exposure: <span id="cfgExposure">--</span></div>
        <div class="config-row">gain: <span id="cfgGain">--</span></div>
        <div class="config-row">tag size: <span id="cfgTagSize">--</span></div>
      </div>
    </div>
  </div>
</div>
<script>
const $ = id => document.getElementById(id);
const feedImg = $('feedImg');
const noFeed = $('noFeed');
feedImg.addEventListener('load', () => { noFeed.style.display = 'none'; });
feedImg.addEventListener('error', () => { noFeed.style.display = 'block'; });

const canvas = $('fieldCanvas');
const ctx = canvas.getContext('2d');

function drawField(data) {
  const W = canvas.width, H = canvas.height;
  const pad = 20;
  const fL = data.field_length || 16.54;
  const fW = data.field_width || 8.21;
  const scale = Math.min((W - pad * 2) / fL, (H - pad * 2) / fW);
  const ox = (W - fL * scale) / 2;
  const oy = (H - fW * scale) / 2;
  function tx(x) { return ox + x * scale; }
  function ty(y) { return oy + (fW - y) * scale; }

  ctx.clearRect(0, 0, W, H);

  // Field background
  ctx.fillStyle = '#111';
  ctx.fillRect(0, 0, W, H);

  // Field outline
  ctx.strokeStyle = '#333';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(tx(0), ty(fW), fL * scale, fW * scale);

  // Center line
  ctx.strokeStyle = '#222';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(tx(fL / 2), ty(fW));
  ctx.lineTo(tx(fL / 2), ty(0));
  ctx.stroke();

  // Tag poses
  const seenIds = new Set(data.tag_ids || []);
  if (data.tag_poses) {
    data.tag_poses.forEach(t => {
      const sx = tx(t.x), sy = ty(t.y);
      const seen = seenIds.has(t.id);
      ctx.fillStyle = seen ? '#22c55e' : '#555';
      ctx.beginPath();
      ctx.arc(sx, sy, seen ? 5 : 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = seen ? '#22c55e' : '#444';
      ctx.font = '9px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(t.id, sx, sy - 7);
    });
  }

  // Camera pose
  if (data.pose) {
    const cx = tx(data.pose.x), cy = ty(data.pose.y);
    const yaw = -data.pose.yaw * Math.PI / 180;
    // Camera triangle
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(yaw);
    ctx.fillStyle = '#3b82f6';
    ctx.globalAlpha = 0.9;
    ctx.beginPath();
    ctx.moveTo(10, 0);
    ctx.lineTo(-6, -6);
    ctx.lineTo(-6, 6);
    ctx.closePath();
    ctx.fill();
    ctx.globalAlpha = 0.3;
    // FOV wedge
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.arc(0, 0, 30, -0.5, 0.5);
    ctx.closePath();
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.restore();
  }

  // No layout message
  if (!data.tag_poses) {
    ctx.fillStyle = '#555';
    ctx.font = '12px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('no tag layout from NT', W / 2, H / 2);
  }
}

async function poll() {
  try {
    const r = await fetch('/api/status');
    if (!r.ok) return;
    const d = await r.json();
    $('statusDot').classList.toggle('connected', d.has_frame);
    $('fps').textContent = (d.fps > 0 ? d.fps + ' fps' : '-- fps');
    $('tagCount').textContent = d.tag_count;
    $('tagCount').className = 'value ' + (d.tag_count > 0 ? 'green' : '');
    $('solveType').textContent = d.solve_type || '--';
    $('solveType').className = 'value ' + (d.solve_type === 'multi' ? 'green' : d.solve_type === 'single' ? 'yellow' : '');
    $('reprojError').textContent = d.error !== null && d.error !== undefined ? d.error.toFixed(4) : '--';

    // Tag badges — highlight seen tags
    const tl = $('tagList'); tl.innerHTML = '';
    (d.tag_ids || []).forEach(id => { const b = document.createElement('span'); b.className = 'tag-badge seen'; b.textContent = 'ID ' + id; tl.appendChild(b); });

    // Pose status
    const ps = $('poseStatus');
    if (!d.tag_poses) {
      ps.innerHTML = '<div class="no-layout">no tag layout from NT — pose unavailable</div>';
    } else if (!d.pose) {
      ps.innerHTML = '<div class="no-layout">no tags detected</div>';
    } else {
      ps.innerHTML = '';
    }

    if (d.pose) {
      $('poseX').textContent = d.pose.x.toFixed(3);
      $('poseY').textContent = d.pose.y.toFixed(3);
      $('poseZ').textContent = d.pose.z.toFixed(3);
      $('poseRoll').textContent = d.pose.roll.toFixed(1) + String.fromCharCode(176);
      $('posePitch').textContent = d.pose.pitch.toFixed(1) + String.fromCharCode(176);
      $('poseYaw').textContent = d.pose.yaw.toFixed(1) + String.fromCharCode(176);
    } else {
      ['poseX','poseY','poseZ','poseRoll','posePitch','poseYaw'].forEach(id => $(id).textContent = '--');
    }
    if (d.config) {
      $('cfgDevice').textContent = d.config.device_id || '--';
      $('cfgCamera').textContent = d.config.camera_id || '--';
      $('cfgRes').textContent = d.config.resolution || '--';
      $('cfgExposure').textContent = d.config.exposure ?? '--';
      $('cfgGain').textContent = d.config.gain ?? '--';
      $('cfgTagSize').textContent = d.config.tag_size ? (d.config.tag_size * 100).toFixed(1) + ' cm' : '--';
    }
    drawField(d);
  } catch(e) {}
}
setInterval(poll, 200);
poll();
</script>
</body>
</html>"""

            def do_GET(self):
                global CLIENT_COUNTS
                if self.path == "/":
                    content = self.DASHBOARD_HTML.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                elif self.path == "/api/status":
                    data = json.dumps(self_mjpeg._telemetry).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    self.wfile.write(data)
                elif self.path == "/stream.mjpg":
                    self.send_response(200)
                    self.send_header("Age", "0")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header(
                        "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
                    )
                    self.end_headers()
                    try:
                        CLIENT_COUNTS[uuid] += 1
                        while True:
                            if not self_mjpeg._has_frame:
                                time.sleep(0.1)
                            else:
                                pil_im = Image.fromarray(self_mjpeg._frame)
                                stream = BytesIO()
                                pil_im.save(stream, format="JPEG")
                                frame_data = stream.getvalue()

                                self.wfile.write(b"--FRAME\r\n")
                                self.send_header("Content-Type", "image/jpeg")
                                self.send_header("Content-Length", str(len(frame_data)))
                                self.end_headers()
                                self.wfile.write(frame_data)
                                self.wfile.write(b"\r\n")
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

    def set_telemetry(self, telemetry: Dict[str, Any]) -> None:
        self._telemetry = telemetry

    def get_client_count(self) -> int:
        if len(self._uuid) > 0:
            return CLIENT_COUNTS[self._uuid]
        else:
            return 0
