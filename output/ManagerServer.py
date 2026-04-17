# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json
import re
import socketserver
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Dict, List, Optional


class ManagerServer:
    """Serves a unified multi-camera dashboard that aggregates data from multiple northstar instances."""

    def __init__(self, instances: List[Dict[str, Any]]):
        self._instances = instances
        self._process_handles: Optional[List] = None  # List of InstanceProcess objects

    def set_instance_processes(self, processes: list):
        """Pass InstanceProcess handles so the server can report status and restart them."""
        self._process_handles = processes

    def _make_handler(self_mgr):  # type: ignore
        instances_json = json.dumps(self_mgr._instances)

        class ManagerHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/":
                    html = DASHBOARD_HTML.replace("__INSTANCES__", instances_json)
                    content = html.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                elif self.path == "/api/processes":
                    data = []
                    if self_mgr._process_handles:
                        for p in self_mgr._process_handles:
                            data.append({
                                "name": p.name,
                                "status": p.status,
                                "exit_code": p.exit_code,
                                "device_id": p.device_id,
                                "stream_port": p.stream_port,
                            })
                    body = json.dumps(data).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_error(404)
                    self.end_headers()

            def do_POST(self):
                # POST /api/processes/<idx>/restart
                m = re.match(r"^/api/processes/(\d+)/restart$", self.path)
                if m and self_mgr._process_handles:
                    idx = int(m.group(1))
                    if 0 <= idx < len(self_mgr._process_handles):
                        threading.Thread(
                            target=self_mgr._process_handles[idx].restart,
                            daemon=True,
                        ).start()
                        self._json_response(200, {"ok": True})
                        return
                # POST /api/processes/<idx>/stop
                m = re.match(r"^/api/processes/(\d+)/stop$", self.path)
                if m and self_mgr._process_handles:
                    idx = int(m.group(1))
                    if 0 <= idx < len(self_mgr._process_handles):
                        threading.Thread(
                            target=self_mgr._process_handles[idx].stop,
                            daemon=True,
                        ).start()
                        self._json_response(200, {"ok": True})
                        return
                # POST /api/processes/<idx>/start
                m = re.match(r"^/api/processes/(\d+)/start$", self.path)
                if m and self_mgr._process_handles:
                    idx = int(m.group(1))
                    if 0 <= idx < len(self_mgr._process_handles):
                        threading.Thread(
                            target=self_mgr._process_handles[idx].start,
                            daemon=True,
                        ).start()
                        self._json_response(200, {"ok": True})
                        return
                self.send_error(404)
                self.end_headers()

            def _json_response(self, code, obj):
                body = json.dumps(obj).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format, *args):
                pass

        return ManagerHandler

    class _ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
        allow_reuse_address = True
        daemon_threads = True

    def start(self, port: int) -> None:
        def _run():
            server = self._ThreadedHTTPServer(("", port), self._make_handler())
            print(f"Manager dashboard running on http://0.0.0.0:{port}")
            server.serve_forever()

        threading.Thread(target=_run, daemon=True).start()


DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Northstar Manager</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
:root {
  --bg: #0a0a0a; --surface: #141414; --border: #1e1e1e;
  --text: #e0e0e0; --dim: #666; --accent: #22c55e;
  --red: #ef4444; --blue: #3b82f6; --yellow: #eab308;
  --orange: #f97316; --purple: #a855f7; --cyan: #06b6d4;
}
body { background: var(--bg); color: var(--text); font-family: 'SF Mono', 'Cascadia Code', 'Fira Code', monospace; font-size: 13px; height: 100vh; overflow: hidden; }

.layout { display: grid; grid-template-rows: 48px 1fr auto; height: 100vh; }

/* Top bar */
.topbar { background: var(--surface); border-bottom: 1px solid var(--border); display: flex; align-items: center; padding: 0 16px; gap: 16px; }
.topbar .logo { font-size: 15px; font-weight: 700; letter-spacing: 0.5px; color: var(--accent); white-space: nowrap; }
.topbar .logo span { color: var(--dim); font-weight: 400; }
.topbar .cam-tabs { display: flex; gap: 2px; margin-left: 16px; }
.topbar .cam-tab { padding: 6px 14px; border-radius: 6px 6px 0 0; cursor: pointer; font-size: 11px; font-weight: 600; letter-spacing: 0.5px; text-transform: uppercase; background: transparent; color: var(--dim); border: 1px solid transparent; border-bottom: none; transition: all 0.15s; }
.topbar .cam-tab:hover { color: var(--text); background: rgba(255,255,255,0.03); }
.topbar .cam-tab.active { background: var(--bg); color: var(--accent); border-color: var(--border); }
.topbar .cam-tab .dot { display: inline-block; width: 6px; height: 6px; border-radius: 50%; margin-right: 6px; background: var(--red); }
.topbar .cam-tab .dot.online { background: var(--accent); }
.topbar .cam-tab .dot.crashed { background: var(--orange); }
.topbar .right { margin-left: auto; display: flex; align-items: center; gap: 10px; }
.topbar .nt-badge { font-size: 10px; font-weight: 700; padding: 2px 6px; border-radius: 4px; letter-spacing: 0.5px; }
.topbar .nt-badge.connected { background: var(--accent); color: #000; }
.topbar .nt-badge.disconnected { background: var(--red); color: #fff; }
.topbar .view-btn { padding: 4px 10px; border-radius: 4px; cursor: pointer; font-size: 11px; font-weight: 600; background: var(--bg); color: var(--dim); border: 1px solid var(--border); transition: all 0.15s; }
.topbar .view-btn:hover { color: var(--text); }
.topbar .view-btn.active { color: var(--accent); border-color: var(--accent); }

/* Main content area */
.main { display: grid; grid-template-columns: 1fr 280px; overflow: hidden; }

/* Camera feeds area */
.feeds { display: grid; gap: 2px; background: var(--bg); overflow: hidden; }
.feeds.grid-1 { grid-template-columns: 1fr; }
.feeds.grid-2 { grid-template-columns: 1fr 1fr; }
.feeds.grid-3 { grid-template-columns: 1fr 1fr; grid-template-rows: 1fr 1fr; }
.feeds.grid-4 { grid-template-columns: 1fr 1fr; grid-template-rows: 1fr 1fr; }

.feed-card { position: relative; background: #000; overflow: hidden; display: flex; align-items: center; justify-content: center; }
.feed-card.hidden { display: none; }
.feed-card img { max-width: 100%; max-height: 100%; object-fit: contain; }
.feed-card .feed-label { position: absolute; top: 8px; left: 8px; background: rgba(0,0,0,0.7); padding: 3px 8px; border-radius: 4px; font-size: 11px; font-weight: 600; letter-spacing: 0.5px; display: flex; align-items: center; gap: 6px; }
.feed-card .feed-fps { position: absolute; top: 8px; right: 8px; background: rgba(0,0,0,0.7); padding: 3px 8px; border-radius: 4px; font-size: 11px; color: var(--dim); }
.feed-card .no-feed { color: var(--dim); font-size: 13px; }

/* Sidebar */
.sidebar { grid-column: 2; background: var(--surface); border-left: 1px solid var(--border); overflow-y: auto; }
.section { border-bottom: 1px solid var(--border); }
.section-header { padding: 10px 14px; font-size: 10px; font-weight: 600; text-transform: uppercase; letter-spacing: 1.2px; color: var(--dim); background: var(--bg); display: flex; align-items: center; gap: 8px; }
.section-header .cam-dot { width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0; }
.section-header .proc-controls { margin-left: auto; display: flex; gap: 4px; }
.section-header .proc-btn { padding: 2px 6px; border-radius: 3px; cursor: pointer; font-size: 9px; font-weight: 700; letter-spacing: 0.5px; border: 1px solid var(--border); background: transparent; transition: all 0.15s; }
.section-header .proc-btn:hover { border-color: var(--text); }
.section-header .proc-btn.restart { color: var(--yellow); }
.section-header .proc-btn.restart:hover { border-color: var(--yellow); }
.section-header .proc-btn.stop { color: var(--red); }
.section-header .proc-btn.stop:hover { border-color: var(--red); }
.section-header .proc-btn.start { color: var(--accent); }
.section-header .proc-btn.start:hover { border-color: var(--accent); }
.section-body { padding: 8px 14px 12px; }
.row { display: flex; justify-content: space-between; align-items: center; padding: 3px 0; }
.row .label { color: var(--dim); }
.row .value { color: var(--text); font-weight: 500; }
.row .value.green { color: var(--accent); }
.row .value.red { color: var(--red); }
.row .value.yellow { color: var(--yellow); }
.row .value.orange { color: var(--orange); }
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

/* Field map at bottom */
.field-wrap { grid-column: 1 / -1; background: var(--surface); border-top: 1px solid var(--border); padding: 10px; display: flex; align-items: center; justify-content: center; }
.field-wrap canvas { border: 1px solid var(--border); border-radius: 4px; }
</style>
</head>
<body>
<div class="layout">
  <div class="topbar">
    <div class="logo">NORTHSTAR <span>// MANAGER</span></div>
    <div class="cam-tabs" id="camTabs"></div>
    <div class="right">
      <button class="view-btn active" id="btnGrid" onclick="setView('grid')">GRID</button>
      <button class="view-btn" id="btnSingle" onclick="setView('single')">SINGLE</button>
      <span class="nt-badge disconnected" id="ntBadge">NT</span>
    </div>
  </div>
  <div class="main">
    <div class="feeds grid-2" id="feeds"></div>
    <div class="sidebar" id="sidebar"></div>
  </div>
  <div class="field-wrap">
    <canvas id="fieldCanvas" width="700" height="260"></canvas>
  </div>
</div>

<script>
const INSTANCES = __INSTANCES__;
const CAM_COLORS = ['#3b82f6', '#22c55e', '#f97316', '#a855f7', '#06b6d4', '#eab308'];

let viewMode = 'grid';
let selectedCam = 0;
let camData = {};
let procData = [];

// Build camera tabs
const tabsEl = document.getElementById('camTabs');
INSTANCES.forEach((inst, i) => {
  const tab = document.createElement('div');
  tab.className = 'cam-tab' + (i === 0 ? ' active' : '');
  tab.id = 'tab-' + i;
  tab.innerHTML = '<span class="dot" id="tabDot-' + i + '"></span>' + inst.name;
  tab.onclick = () => selectCam(i);
  tabsEl.appendChild(tab);
});

// Build feed cards
const feedsEl = document.getElementById('feeds');
INSTANCES.forEach((inst, i) => {
  const url = 'http://' + inst.host + ':' + inst.port;
  const card = document.createElement('div');
  card.className = 'feed-card';
  card.id = 'feed-' + i;
  card.innerHTML =
    '<img src="' + url + '/stream.mjpg" onerror="this.style.display=\'none\'" onload="this.style.display=\'block\'" />' +
    '<div class="no-feed" id="noFeed-' + i + '">NO FEED</div>' +
    '<div class="feed-label"><span style="color:' + CAM_COLORS[i % CAM_COLORS.length] + '">' + inst.name + '</span></div>' +
    '<div class="feed-fps" id="feedFps-' + i + '">-- fps</div>';
  feedsEl.appendChild(card);
});

updateGridClass();

function updateGridClass() {
  const el = feedsEl;
  el.className = 'feeds';
  if (viewMode === 'single') {
    el.classList.add('grid-1');
    INSTANCES.forEach((_, i) => {
      document.getElementById('feed-' + i).classList.toggle('hidden', i !== selectedCam);
    });
  } else {
    const n = INSTANCES.length;
    el.classList.add(n <= 1 ? 'grid-1' : n <= 2 ? 'grid-2' : 'grid-4');
    INSTANCES.forEach((_, i) => {
      document.getElementById('feed-' + i).classList.remove('hidden');
    });
  }
}

function setView(mode) {
  viewMode = mode;
  document.getElementById('btnGrid').classList.toggle('active', mode === 'grid');
  document.getElementById('btnSingle').classList.toggle('active', mode === 'single');
  updateGridClass();
}

function selectCam(i) {
  selectedCam = i;
  document.querySelectorAll('.cam-tab').forEach((t, idx) => t.classList.toggle('active', idx === i));
  if (viewMode === 'single') updateGridClass();
  renderSidebar();
}

function procAction(idx, action) {
  fetch('/api/processes/' + idx + '/' + action, { method: 'POST' });
}

function renderSidebar() {
  const sb = document.getElementById('sidebar');
  let html = '';

  const indices = viewMode === 'single' ? [selectedCam] : INSTANCES.map((_, i) => i);

  indices.forEach(i => {
    const inst = INSTANCES[i];
    const d = camData[i] || {};
    const p = procData[i] || {};
    const color = CAM_COLORS[i % CAM_COLORS.length];
    const status = p.status || 'unknown';

    // Status badge
    const statusClass = status === 'running' ? 'green' : status === 'crashed' ? 'orange' : status === 'starting' ? 'yellow' : 'red';

    // Controls — show contextual buttons
    let controls = '';
    if (status === 'running') {
      controls = '<button class="proc-btn restart" onclick="procAction(' + i + ',\'restart\')">RESTART</button>' +
                 '<button class="proc-btn stop" onclick="procAction(' + i + ',\'stop\')">STOP</button>';
    } else if (status === 'stopped') {
      controls = '<button class="proc-btn start" onclick="procAction(' + i + ',\'start\')">START</button>';
    } else if (status === 'crashed') {
      controls = '<button class="proc-btn start" onclick="procAction(' + i + ',\'restart\')">RESTART</button>';
    }

    html += '<div class="section">';
    html += '<div class="section-header">';
    html += '<span class="cam-dot" style="background:' + color + '"></span>';
    html += inst.name;
    html += '<div class="proc-controls">';
    html += '<span class="value ' + statusClass + '" style="font-size:9px;margin-right:4px;">' + status.toUpperCase() + '</span>';
    html += controls;
    html += '</div></div>';
    html += '<div class="section-body">';

    if (status !== 'running' && status !== 'starting') {
      html += '<div class="no-layout">process ' + status + (p.exit_code !== null && p.exit_code !== undefined ? ' (exit ' + p.exit_code + ')' : '') + '</div>';
      html += '</div></div>';
      return;
    }

    // Detection
    const tagCount = d.tag_count || 0;
    const solveType = d.solve_type || '--';
    const solveClass = solveType === 'multi' ? 'green' : solveType === 'single' ? 'yellow' : '';
    const error = (d.error !== null && d.error !== undefined) ? d.error.toFixed(4) : '--';

    html += '<div class="row"><span class="label">Tags</span><span class="value ' + (tagCount > 0 ? 'green' : '') + '">' + tagCount + '</span></div>';
    html += '<div class="row"><span class="label">Solve</span><span class="value ' + solveClass + '">' + solveType + '</span></div>';
    html += '<div class="row"><span class="label">Error</span><span class="value">' + error + '</span></div>';
    html += '<div class="row"><span class="label">FPS</span><span class="value">' + (d.fps > 0 ? d.fps : '--') + '</span></div>';

    // Tag badges
    if (d.tag_ids && d.tag_ids.length > 0) {
      html += '<div class="tag-list">';
      d.tag_ids.forEach(id => { html += '<span class="tag-badge seen">ID ' + id + '</span>'; });
      html += '</div>';
    }

    // Pose
    if (d.pose) {
      html += '<div class="pose-grid">';
      html += '<div class="pose-cell"><div class="axis">X</div><div class="val x">' + d.pose.x.toFixed(3) + '</div></div>';
      html += '<div class="pose-cell"><div class="axis">Y</div><div class="val y">' + d.pose.y.toFixed(3) + '</div></div>';
      html += '<div class="pose-cell"><div class="axis">Z</div><div class="val z">' + d.pose.z.toFixed(3) + '</div></div>';
      html += '</div>';
      html += '<div class="pose-grid" style="margin-top:4px">';
      html += '<div class="pose-cell"><div class="axis">Roll</div><div class="val">' + d.pose.roll.toFixed(1) + '\u00B0</div></div>';
      html += '<div class="pose-cell"><div class="axis">Pitch</div><div class="val">' + d.pose.pitch.toFixed(1) + '\u00B0</div></div>';
      html += '<div class="pose-cell"><div class="axis">Yaw</div><div class="val">' + d.pose.yaw.toFixed(1) + '\u00B0</div></div>';
      html += '</div>';
    } else {
      html += '<div class="no-layout">no pose</div>';
    }

    // Config
    if (d.config) {
      html += '<div style="margin-top:8px">';
      html += '<div class="config-row">device: <span>' + (d.config.device_id || '--') + '</span></div>';
      html += '<div class="config-row">resolution: <span>' + (d.config.resolution || '--') + '</span></div>';
      html += '</div>';
    }

    html += '</div></div>';
  });

  sb.innerHTML = html;
}

// Field map drawing
const canvas = document.getElementById('fieldCanvas');
const ctx = canvas.getContext('2d');

function drawField() {
  const W = canvas.width, H = canvas.height;
  const pad = 20;

  // Get field dimensions from first instance with data
  let fL = 16.54, fW = 8.21, tagPoses = null;
  for (const i in camData) {
    const d = camData[i];
    if (d.field_length) fL = d.field_length;
    if (d.field_width) fW = d.field_width;
    if (d.tag_poses) tagPoses = d.tag_poses;
    if (tagPoses) break;
  }

  const scale = Math.min((W - pad * 2) / fL, (H - pad * 2) / fW);
  const ox = (W - fL * scale) / 2;
  const oy = (H - fW * scale) / 2;
  function tx(x) { return ox + x * scale; }
  function ty(y) { return oy + (fW - y) * scale; }

  ctx.clearRect(0, 0, W, H);
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

  // Collect all seen tag IDs across all cameras
  const allSeenIds = new Set();
  for (const i in camData) {
    (camData[i].tag_ids || []).forEach(id => allSeenIds.add(id));
  }

  // Draw tag poses
  if (tagPoses) {
    tagPoses.forEach(t => {
      const sx = tx(t.x), sy = ty(t.y);
      const seen = allSeenIds.has(t.id);
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

  // Draw each camera pose with its own color
  INSTANCES.forEach((inst, i) => {
    const d = camData[i];
    if (!d || !d.pose) return;
    const cx = tx(d.pose.x), cy = ty(d.pose.y);
    const yaw = -d.pose.yaw * Math.PI / 180;
    const color = CAM_COLORS[i % CAM_COLORS.length];

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(yaw);
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.9;
    ctx.beginPath();
    ctx.moveTo(10, 0);
    ctx.lineTo(-6, -6);
    ctx.lineTo(-6, 6);
    ctx.closePath();
    ctx.fill();
    ctx.globalAlpha = 0.25;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.arc(0, 0, 30, -0.5, 0.5);
    ctx.closePath();
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.restore();

    // Label
    ctx.fillStyle = color;
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(inst.name, cx, cy + 16);
  });

  // No layout message
  if (!tagPoses) {
    ctx.fillStyle = '#555';
    ctx.font = '12px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('no tag layout from NT', W / 2, H / 2);
  }
}

// Poll each instance for telemetry + process status
async function pollAll() {
  // Fetch process status from manager
  try {
    const pr = await fetch('/api/processes');
    if (pr.ok) procData = await pr.json();
  } catch(e) {}

  // Fetch telemetry from each instance
  const fetches = INSTANCES.map(async (inst, i) => {
    try {
      const url = 'http://' + inst.host + ':' + inst.port + '/api/status';
      const r = await fetch(url, { signal: AbortSignal.timeout(1000) });
      if (r.ok) {
        camData[i] = await r.json();
        updateTabDot(i, 'online');
        const fpsEl = document.getElementById('feedFps-' + i);
        if (fpsEl) fpsEl.textContent = (camData[i].fps > 0 ? camData[i].fps + ' fps' : '-- fps');
        return;
      }
    } catch(e) {}
    // Instance not responding — check process status for dot color
    const p = procData[i] || {};
    if (p.status === 'crashed') updateTabDot(i, 'crashed');
    else updateTabDot(i, 'offline');
    camData[i] = {};
  });
  await Promise.all(fetches);

  // NT status — connected if any instance reports connected
  const anyNt = Object.values(camData).some(d => d.nt_connected);
  const ntb = document.getElementById('ntBadge');
  ntb.classList.toggle('connected', anyNt);
  ntb.classList.toggle('disconnected', !anyNt);

  renderSidebar();
  drawField();
}

function updateTabDot(i, state) {
  const dot = document.getElementById('tabDot-' + i);
  if (!dot) return;
  dot.classList.remove('online', 'crashed');
  if (state === 'online') dot.classList.add('online');
  else if (state === 'crashed') dot.classList.add('crashed');
}

setInterval(pollAll, 250);
pollAll();
renderSidebar();
</script>
</body>
</html>"""
