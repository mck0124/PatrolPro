# web_dashboard.py
"""
Small HTTP dashboard for the Patrol Pro Jetson runtime.

It serves:
- live MJPEG video from the frame already produced by detect_people.py
- saved snapshots from snapshot_writer.SNAP_DIR
- event rows from snapshot_writer.LOG_FILE

The module intentionally uses only the Python standard library plus OpenCV so
it works on the Jetson Nano Python 3.6 environment without a Flask install.
"""

import csv
import json
import mimetypes
import os
import posixpath
import socketserver
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler
from http.server import HTTPServer
from urllib.parse import parse_qs
from urllib.parse import quote
from urllib.parse import unquote
from urllib.parse import urlparse

import cv2

import snapshot_writer


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8080
DEFAULT_JPEG_QUALITY = 82
DEFAULT_STREAM_FPS = 8.0
DEFAULT_STREAM_WIDTH = 960

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".webp", ".bmp")

SNAPSHOT_KINDS = (
    ("arrival", "Arrival", ("arrival", "arrived", "route")),
    ("unknown", "Unknown", ("unknown", "security", "intruder")),
    ("fire", "Fire / Gas", ("fire", "fire_immediate", "hazard", "gas", "smoke", "smoke_gas", "flame", "both")),
    ("verified", "Verified", ("verified",)),
    ("person", "Person", ("person",)),
    ("face", "Face", ("face",)),
)


class _ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class DashboardState(object):
    def __init__(self, jpeg_quality, max_fps, stream_width):
        self.jpeg_quality = int(jpeg_quality)
        self.min_interval = 1.0 / max(float(max_fps), 1.0)
        self.stream_width = int(stream_width)
        self._condition = threading.Condition()
        self._latest_jpeg = None
        self._latest_frame_at = 0.0
        self._last_encode_at = 0.0
        self._status = {}

    def update_frame(self, frame, status=None):
        now = time.time()
        if status is not None:
            with self._condition:
                self._status = dict(status)
                self._status["status_updated_at"] = _format_time(now)

        if frame is None or (now - self._last_encode_at) < self.min_interval:
            return

        stream_frame = self._resize_for_stream(frame)
        ok, encoded = cv2.imencode(
            ".jpg",
            stream_frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            return

        with self._condition:
            self._latest_jpeg = encoded.tobytes()
            self._latest_frame_at = now
            self._last_encode_at = now
            self._condition.notify_all()

    def wait_for_frame(self, last_seen_at, timeout=2.0):
        with self._condition:
            if self._latest_frame_at <= last_seen_at:
                self._condition.wait(timeout)
            return self._latest_jpeg, self._latest_frame_at

    def status_payload(self):
        now = time.time()
        with self._condition:
            payload = dict(self._status)
            payload["has_frame"] = self._latest_jpeg is not None
            payload["frame_updated_at"] = (
                _format_time(self._latest_frame_at)
                if self._latest_frame_at > 0.0
                else ""
            )
            payload["frame_age_seconds"] = (
                round(now - self._latest_frame_at, 2)
                if self._latest_frame_at > 0.0
                else None
            )
            return payload

    def _resize_for_stream(self, frame):
        if self.stream_width <= 0:
            return frame

        height, width = frame.shape[:2]
        if width <= self.stream_width:
            return frame

        scale = float(self.stream_width) / float(width)
        target_size = (self.stream_width, max(1, int(height * scale)))
        return cv2.resize(frame, target_size)


class DashboardServer(object):
    def __init__(
        self,
        host=DEFAULT_HOST,
        port=DEFAULT_PORT,
        snapshot_root=None,
        log_file=None,
        jpeg_quality=DEFAULT_JPEG_QUALITY,
        max_fps=DEFAULT_STREAM_FPS,
        stream_width=DEFAULT_STREAM_WIDTH,
        control_handler=None,
    ):
        self.host = host
        self.port = int(port)
        self.snapshot_root = snapshot_root or snapshot_writer.SNAP_DIR
        self.log_file = log_file or snapshot_writer.LOG_FILE
        self.control_handler = control_handler
        self.state = DashboardState(jpeg_quality, max_fps, stream_width)
        self._httpd = None
        self._thread = None

    def start(self):
        if self._httpd is not None:
            return

        handler = _make_handler(self)
        self._httpd = _ThreadedHTTPServer((self.host, self.port), handler)
        self.port = self._httpd.server_address[1]
        self._thread = threading.Thread(target=self._httpd.serve_forever)
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        if self._httpd is None:
            return
        self._httpd.shutdown()
        self._httpd.server_close()
        self._httpd = None
        self._thread = None

    def update_frame(self, frame, status=None):
        self.state.update_frame(frame, status=status)

    def url_hint(self):
        host = self.host
        if host in ("", "0.0.0.0"):
            host = "<jetson-ip>"
        return "http://{}:{}/".format(host, self.port)


def _make_handler(dashboard):
    class DashboardRequestHandler(BaseHTTPRequestHandler):
        server_version = "PatrolProDashboard/1.0"

        def do_GET(self):
            parsed = urlparse(self.path)
            path = parsed.path

            try:
                if path in ("", "/", "/index.html"):
                    self._send_html(DASHBOARD_HTML)
                elif path == "/stream.mjpg":
                    self._send_stream()
                elif path == "/api/status":
                    self._send_status()
                elif path == "/api/snapshots":
                    self._send_snapshots(parsed.query)
                elif path == "/api/events":
                    self._send_events(parsed.query)
                elif path.startswith("/snapshots/"):
                    self._send_snapshot_file(path)
                else:
                    self.send_error(404, "Not found")
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                return
            except Exception as exc:
                self.send_error(500, "Dashboard error: {}".format(exc))

        def do_POST(self):
            parsed = urlparse(self.path)
            path = parsed.path

            try:
                if path == "/api/control/emergency-stop":
                    self._handle_emergency_stop()
                elif path == "/api/control/auto-patrol":
                    self._handle_auto_patrol()
                elif path == "/api/control/patrol-stop":
                    self._handle_patrol_stop()
                else:
                    self.send_error(404, "Not found")
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                return
            except Exception as exc:
                self.send_error(500, "Dashboard error: {}".format(exc))

        def log_message(self, fmt, *args):
            return

        def _send_html(self, body_text):
            body = body_text.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, payload):
            body = json.dumps(payload, sort_keys=True).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_stream(self):
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.send_header("Cache-Control", "no-store")
            self.send_header("Pragma", "no-cache")
            self.end_headers()

            last_seen_at = 0.0
            while True:
                jpeg, frame_at = dashboard.state.wait_for_frame(last_seen_at)
                if jpeg is None or frame_at <= last_seen_at:
                    continue

                last_seen_at = frame_at
                header = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + b"Content-Length: "
                    + str(len(jpeg)).encode("ascii")
                    + b"\r\n\r\n"
                )
                self.wfile.write(header)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()

        def _send_status(self):
            payload = dashboard.state.status_payload()
            payload["snapshot_counts"] = _snapshot_counts(dashboard.snapshot_root)
            events = _read_events(dashboard.log_file, 1)
            payload["latest_event"] = events[0] if events else None
            payload["server_time"] = _format_time(time.time())
            self._send_json(payload)

        def _send_snapshots(self, query):
            params = parse_qs(query)
            limit = _query_int(params, "limit", 120, 1, 500)
            snapshots, counts = _scan_snapshots(dashboard.snapshot_root, limit)
            self._send_json({
                "counts": counts,
                "generated_at": _format_time(time.time()),
                "snapshots": snapshots,
            })

        def _send_events(self, query):
            params = parse_qs(query)
            limit = _query_int(params, "limit", 100, 1, 500)
            self._send_json({
                "events": _read_events(dashboard.log_file, limit),
                "generated_at": _format_time(time.time()),
            })

        def _handle_emergency_stop(self):
            if dashboard.control_handler is None:
                self.send_error(503, "Dashboard control unavailable")
                return

            result = dashboard.control_handler("EMERGENCY_STOP")
            if result is None:
                result = {}
            result.setdefault("ok", True)
            result.setdefault("command", "EMERGENCY_STOP")
            result["server_time"] = _format_time(time.time())
            self._send_json(result)

        def _handle_auto_patrol(self):
            if dashboard.control_handler is None:
                self.send_error(503, "Dashboard control unavailable")
                return

            result = dashboard.control_handler("AUTO_PATROL")
            if result is None:
                result = {}
            result.setdefault("ok", True)
            result.setdefault("command", "AUTO_PATROL")
            result["server_time"] = _format_time(time.time())
            self._send_json(result)

        def _handle_patrol_stop(self):
            if dashboard.control_handler is None:
                self.send_error(503, "Dashboard control unavailable")
                return

            result = dashboard.control_handler("PATROL_STOP")
            if result is None:
                result = {}
            result.setdefault("ok", True)
            result.setdefault("command", "PATROL_STOP")
            result["server_time"] = _format_time(time.time())
            self._send_json(result)

        def _send_snapshot_file(self, path):
            target = _resolve_snapshot_path(dashboard.snapshot_root, path)
            if target is None or not os.path.isfile(target):
                self.send_error(404, "Snapshot not found")
                return

            content_type = mimetypes.guess_type(target)[0] or "application/octet-stream"
            with open(target, "rb") as handle:
                body = handle.read()

            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "private, max-age=30")
            self.end_headers()
            self.wfile.write(body)

    return DashboardRequestHandler


def _query_int(params, key, default, minimum, maximum):
    values = params.get(key)
    if not values:
        return default
    try:
        value = int(values[0])
    except (TypeError, ValueError):
        return default
    return max(minimum, min(maximum, value))


def _format_time(epoch_seconds):
    if not epoch_seconds:
        return ""
    return datetime.fromtimestamp(epoch_seconds).strftime("%Y-%m-%d %H:%M:%S")


def _snapshot_counts(root):
    counts = {}
    if not os.path.isdir(root):
        return counts

    for category in sorted(os.listdir(root)):
        cat_path = os.path.join(root, category)
        if not os.path.isdir(cat_path) or category.startswith("."):
            continue
        count = 0
        for filename in os.listdir(cat_path):
            if filename.startswith("."):
                continue
            if os.path.splitext(filename)[1].lower() in IMAGE_EXTENSIONS:
                count += 1
        kind, _ = _snapshot_kind(category)
        counts[kind] = counts.get(kind, 0) + count
    return counts


def _scan_snapshots(root, limit):
    counts = {}
    items = []
    if not os.path.isdir(root):
        return items, counts

    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [name for name in dirnames if not name.startswith(".")]
        rel_dir = os.path.relpath(dirpath, root)
        category = "root" if rel_dir == "." else rel_dir.split(os.sep)[0]

        for filename in filenames:
            if filename.startswith("."):
                continue
            ext = os.path.splitext(filename)[1].lower()
            if ext not in IMAGE_EXTENSIONS:
                continue

            path = os.path.join(dirpath, filename)
            try:
                stat_result = os.stat(path)
            except OSError:
                continue

            rel_path = os.path.relpath(path, root)
            url_path = rel_path.replace(os.sep, "/")
            kind, kind_label = _snapshot_kind(category)
            counts[kind] = counts.get(kind, 0) + 1
            items.append({
                "category": kind,
                "category_label": kind_label,
                "raw_category": category,
                "filename": filename,
                "mtime": stat_result.st_mtime,
                "saved_at": _format_time(stat_result.st_mtime),
                "size_bytes": stat_result.st_size,
                "track_id": _track_id_from_filename(filename),
                "url": "/snapshots/{}".format(quote(url_path, safe="/")),
            })

    items.sort(key=lambda item: item["mtime"], reverse=True)
    return items[:limit], counts


def _track_id_from_filename(filename):
    stem = os.path.splitext(filename)[0]
    parts = stem.split("_")
    if len(parts) < 2:
        return ""
    try:
        return str(int(parts[1]))
    except ValueError:
        return ""


def _snapshot_kind(category):
    normalized = (category or "").strip().lower().replace("-", "_").replace(" ", "_")
    for kind, label, aliases in SNAPSHOT_KINDS:
        if normalized in aliases:
            return kind, label
    return "other", "Other"


def _read_events(log_file, limit):
    if not os.path.isfile(log_file):
        return []

    try:
        with open(log_file, "r", newline="") as handle:
            rows = list(csv.DictReader(handle))
    except (OSError, csv.Error):
        return []

    events = []
    for row in rows[-limit:]:
        events.append({
            "timestamp": row.get("timestamp", ""),
            "event": row.get("event", ""),
            "track_id": row.get("track_id", ""),
            "name": row.get("name", ""),
            "mode": row.get("mode", ""),
        })
    events.reverse()
    return events


def _resolve_snapshot_path(root, request_path):
    rel_url = request_path[len("/snapshots/"):]
    rel_posix = posixpath.normpath(unquote(rel_url))
    if rel_posix in ("", ".") or rel_posix.startswith("../") or rel_posix.startswith("/"):
        return None

    root_abs = os.path.abspath(root)
    target = os.path.abspath(os.path.join(root_abs, rel_posix.replace("/", os.sep)))
    if target != root_abs and not target.startswith(root_abs + os.sep):
        return None
    return target


DASHBOARD_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Patrol Pro Dashboard</title>
  <style>
    :root {
      --bg: #f4f5f2;
      --surface: #ffffff;
      --surface-2: #ecefec;
      --ink: #171b1a;
      --muted: #65706b;
      --line: #d5ddd7;
      --accent: #0b7a61;
      --accent-2: #0f5f78;
      --danger: #b42318;
      --warning: #9a5a00;
      --ok: #177245;
      --shadow: 0 16px 48px rgba(23, 27, 26, 0.10);
    }

    * {
      box-sizing: border-box;
    }

    html, body {
      margin: 0;
      min-height: 100%;
      background: var(--bg);
      color: var(--ink);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      letter-spacing: 0;
    }

    body {
      padding: 24px;
    }

    button {
      font: inherit;
    }

    .shell {
      max-width: 1480px;
      margin: 0 auto;
      display: grid;
      gap: 18px;
    }

    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      min-height: 52px;
    }

    .brand {
      display: flex;
      align-items: center;
      gap: 12px;
      min-width: 0;
    }

    .mark {
      width: 42px;
      height: 42px;
      border-radius: 8px;
      background: #171b1a;
      color: #ffffff;
      display: grid;
      place-items: center;
      font-weight: 800;
      flex: 0 0 auto;
    }

    h1, h2, h3, p {
      margin: 0;
    }

    h1 {
      font-size: clamp(1.45rem, 2vw, 2.15rem);
      line-height: 1.05;
    }

    .subtle {
      color: var(--muted);
      font-size: 0.95rem;
    }

    .status-dot {
      width: 11px;
      height: 11px;
      border-radius: 50%;
      background: var(--warning);
      box-shadow: 0 0 0 6px rgba(154, 90, 0, 0.14);
      flex: 0 0 auto;
    }

    .status-dot.live {
      background: var(--ok);
      box-shadow: 0 0 0 6px rgba(23, 114, 69, 0.14);
    }

    .status-dot.stale {
      background: var(--danger);
      box-shadow: 0 0 0 6px rgba(180, 35, 24, 0.14);
    }

    .toolbar {
      display: flex;
      align-items: center;
      gap: 10px;
      flex-wrap: wrap;
      justify-content: flex-end;
    }

    .icon-button {
      width: 42px;
      height: 42px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--surface);
      color: var(--ink);
      display: grid;
      place-items: center;
      cursor: pointer;
    }

    .icon-button:hover {
      border-color: var(--accent);
    }

    .danger-button {
      min-height: 42px;
      border: 1px solid #8f1c14;
      border-radius: 8px;
      background: var(--danger);
      color: #ffffff;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      padding: 0 14px;
      font-weight: 900;
      cursor: pointer;
      white-space: nowrap;
    }

    .danger-button:hover {
      background: #8f1c14;
    }

    .danger-button:disabled {
      cursor: wait;
      opacity: 0.64;
    }

    .primary-button {
      min-height: 42px;
      border: 1px solid #085f4b;
      border-radius: 8px;
      background: var(--accent);
      color: #ffffff;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      padding: 0 14px;
      font-weight: 900;
      cursor: pointer;
      white-space: nowrap;
    }

    .primary-button:hover {
      background: #085f4b;
    }

    .primary-button:disabled {
      cursor: wait;
      opacity: 0.64;
    }

    .secondary-button {
      min-height: 42px;
      border: 1px solid var(--border);
      border-radius: 8px;
      background: #f8fafc;
      color: var(--text);
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      padding: 0 14px;
      font-weight: 900;
      cursor: pointer;
      white-space: nowrap;
    }

    .secondary-button:hover {
      border-color: var(--accent);
      color: var(--accent);
    }

    .secondary-button:disabled {
      cursor: wait;
      opacity: 0.64;
    }

    .control-status {
      color: var(--muted);
      font-size: 0.88rem;
      min-width: 120px;
      text-align: right;
    }

    .dashboard-grid {
      display: grid;
      grid-template-columns: minmax(0, 1.8fr) minmax(300px, 0.82fr);
      gap: 18px;
      align-items: start;
    }

    .panel {
      background: var(--surface);
      border: 1px solid var(--line);
      border-radius: 8px;
      box-shadow: var(--shadow);
      overflow: hidden;
    }

    .panel-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      padding: 16px 18px;
      border-bottom: 1px solid var(--line);
      background: #fbfcfa;
    }

    .panel-title {
      font-size: 1rem;
      font-weight: 800;
    }

    .video-wrap {
      background: #0f1412;
      aspect-ratio: 16 / 9;
      display: grid;
      place-items: center;
      position: relative;
    }

    .video-wrap img {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
    }

    .video-placeholder {
      color: #dce4df;
      font-size: 0.95rem;
      position: absolute;
      inset: 0;
      display: grid;
      place-items: center;
      padding: 24px;
      text-align: center;
    }

    .metric-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      padding: 16px;
    }

    .metric {
      min-height: 86px;
      background: var(--surface-2);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      display: grid;
      gap: 8px;
      align-content: center;
    }

    .metric .label {
      color: var(--muted);
      font-size: 0.8rem;
      text-transform: uppercase;
      font-weight: 800;
    }

    .metric .value {
      font-size: 1.4rem;
      font-weight: 850;
      overflow-wrap: anywhere;
    }

    .content-grid {
      display: grid;
      grid-template-columns: minmax(0, 1.2fr) minmax(340px, 0.8fr);
      gap: 18px;
    }

    .segmented {
      display: inline-grid;
      grid-auto-flow: column;
      gap: 2px;
      background: var(--surface-2);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 3px;
    }

    .segmented button {
      border: 0;
      border-radius: 6px;
      padding: 8px 11px;
      background: transparent;
      color: var(--muted);
      font-weight: 800;
      cursor: pointer;
    }

    .segmented button.active {
      background: var(--surface);
      color: var(--ink);
      box-shadow: 0 1px 8px rgba(23, 27, 26, 0.10);
    }

    .gallery {
      padding: 16px;
      display: grid;
      grid-template-columns: repeat(auto-fill, minmax(164px, 1fr));
      gap: 12px;
      min-height: 260px;
    }

    .snapshot {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fbfcfa;
      overflow: hidden;
      cursor: pointer;
      text-align: left;
      padding: 0;
    }

    .snapshot.active {
      border-color: var(--accent);
      box-shadow: 0 0 0 3px rgba(11, 122, 97, 0.18);
    }

    .snapshot img {
      display: block;
      width: 100%;
      aspect-ratio: 4 / 3;
      object-fit: cover;
      background: #dfe5df;
    }

    .snapshot-meta {
      padding: 10px;
      display: grid;
      gap: 4px;
    }

    .category {
      display: inline-flex;
      align-items: center;
      min-height: 24px;
      width: fit-content;
      padding: 3px 8px;
      border-radius: 999px;
      background: rgba(15, 95, 120, 0.12);
      color: var(--accent-2);
      font-size: 0.78rem;
      font-weight: 850;
      text-transform: uppercase;
    }

    .category.kind-unknown {
      background: rgba(180, 35, 24, 0.12);
      color: var(--danger);
    }

    .category.kind-fire {
      background: rgba(154, 90, 0, 0.16);
      color: var(--warning);
    }

    .category.kind-verified {
      background: rgba(23, 114, 69, 0.14);
      color: var(--ok);
    }

    .category.kind-arrival {
      background: rgba(11, 122, 97, 0.14);
      color: var(--accent);
    }

    .category.kind-person,
    .category.kind-face {
      background: rgba(15, 95, 120, 0.12);
      color: var(--accent-2);
    }

    .filename {
      color: var(--ink);
      font-size: 0.84rem;
      line-height: 1.25;
      overflow-wrap: anywhere;
    }

    .detail {
      padding: 16px;
      display: grid;
      gap: 14px;
    }

    .detail-image {
      width: 100%;
      aspect-ratio: 4 / 3;
      object-fit: contain;
      background: #101512;
      border-radius: 8px;
      border: 1px solid var(--line);
    }

    .detail-list {
      display: grid;
      gap: 8px;
      color: var(--muted);
      font-size: 0.92rem;
    }

    .detail-list strong {
      color: var(--ink);
      font-weight: 850;
    }

    .events {
      padding: 0 16px 16px;
      overflow-x: auto;
    }

    table {
      width: 100%;
      border-collapse: collapse;
      min-width: 560px;
      font-size: 0.9rem;
    }

    th, td {
      border-bottom: 1px solid var(--line);
      padding: 12px 8px;
      text-align: left;
      vertical-align: top;
    }

    th {
      color: var(--muted);
      font-size: 0.78rem;
      text-transform: uppercase;
      font-weight: 850;
      background: #fbfcfa;
      position: sticky;
      top: 0;
    }

    .event-alert {
      color: var(--danger);
      font-weight: 850;
    }

    .event-ok {
      color: var(--ok);
      font-weight: 850;
    }

    .empty {
      min-height: 190px;
      display: grid;
      place-items: center;
      color: var(--muted);
      text-align: center;
      padding: 24px;
    }

    .alert-overlay {
      position: fixed;
      inset: 0;
      z-index: 80;
      display: none;
      place-items: center;
      padding: 22px;
      background: rgba(10, 14, 13, 0.55);
    }

    .alert-overlay.show {
      display: grid;
    }

    .alert-card {
      width: min(560px, 100%);
      border-radius: 8px;
      background: #fffaf7;
      border: 1px solid rgba(180, 35, 24, 0.34);
      border-left: 10px solid var(--danger);
      box-shadow: 0 28px 90px rgba(0, 0, 0, 0.35);
      padding: 28px;
      display: grid;
      gap: 12px;
      position: relative;
    }

    .alert-card.fire {
      border-left-color: var(--warning);
      border-color: rgba(154, 90, 0, 0.42);
    }

    .alert-close {
      position: absolute;
      top: 14px;
      right: 14px;
      background: transparent;
      border-color: transparent;
    }

    .alert-eyebrow {
      color: var(--danger);
      font-size: 0.86rem;
      font-weight: 900;
      text-transform: uppercase;
    }

    .alert-card.fire .alert-eyebrow {
      color: var(--warning);
    }

    .alert-title {
      font-size: clamp(1.85rem, 4vw, 3rem);
      line-height: 1;
      font-weight: 900;
      padding-right: 42px;
    }

    .alert-body {
      color: #373f3b;
      font-size: 1.04rem;
      line-height: 1.45;
    }

    @media (max-width: 1040px) {
      body {
        padding: 16px;
      }

      .dashboard-grid,
      .content-grid {
        grid-template-columns: 1fr;
      }
    }

    @media (max-width: 640px) {
      .topbar {
        align-items: flex-start;
        flex-direction: column;
      }

      .toolbar {
        width: 100%;
        justify-content: space-between;
      }

      .control-status {
        min-width: 0;
        width: 100%;
        text-align: left;
      }

      .metric-grid {
        grid-template-columns: 1fr;
      }

      .segmented {
        width: 100%;
        grid-auto-flow: row;
      }

      .segmented button {
        width: 100%;
      }

      .alert-card {
        padding: 22px;
      }
    }
  </style>
</head>
<body>
  <main class="shell">
    <header class="topbar">
      <div class="brand">
        <div class="mark">PP</div>
        <div>
          <h1>Patrol Pro Dashboard</h1>
          <p class="subtle" id="serverTime">Waiting for runtime data</p>
        </div>
      </div>
      <div class="toolbar">
        <span class="status-dot" id="liveDot" aria-label="stream status"></span>
        <button class="primary-button" id="autoPatrolButton" aria-label="Start patrol" title="Start patrol">
          <svg width="18" height="18" viewBox="0 0 24 24" aria-hidden="true">
            <path d="M8 5l11 7-11 7z" fill="currentColor"/>
          </svg>
          <span>Start Patrol</span>
        </button>
        <button class="secondary-button" id="patrolStopButton" aria-label="Stop patrol" title="Stop patrol">
          <svg width="18" height="18" viewBox="0 0 24 24" aria-hidden="true">
            <path d="M7 7h10v10H7z" fill="currentColor"/>
          </svg>
          <span>Stop Patrol</span>
        </button>
        <button class="danger-button" id="emergencyStopButton" aria-label="Emergency stop" title="Emergency stop">
          <svg width="18" height="18" viewBox="0 0 24 24" aria-hidden="true">
            <path d="M7 3h10l4 4v10l-4 4H7l-4-4V7z" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/>
            <path d="M8 8l8 8M16 8l-8 8" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round"/>
          </svg>
          <span>Emergency Stop</span>
        </button>
        <span class="control-status" id="controlStatus" aria-live="polite"></span>
        <button class="icon-button" id="refreshButton" aria-label="Refresh dashboard" title="Refresh">
          <svg width="19" height="19" viewBox="0 0 24 24" aria-hidden="true">
            <path d="M20 12a8 8 0 1 1-2.34-5.66" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
            <path d="M20 4v6h-6" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
        </button>
      </div>
    </header>

    <section class="dashboard-grid">
      <section class="panel">
        <div class="panel-header">
          <h2 class="panel-title">Live Camera</h2>
          <p class="subtle" id="frameAge">No frame yet</p>
        </div>
        <div class="video-wrap">
          <div class="video-placeholder" id="videoPlaceholder">Waiting for the first frame</div>
          <img id="liveStream" src="/stream.mjpg" alt="Live Patrol Pro camera stream">
        </div>
      </section>

      <aside class="panel">
        <div class="panel-header">
          <h2 class="panel-title">Runtime</h2>
        </div>
        <div class="metric-grid">
          <div class="metric"><div class="label">FPS</div><div class="value" id="metricFps">-</div></div>
          <div class="metric"><div class="label">People</div><div class="value" id="metricPeople">-</div></div>
          <div class="metric"><div class="label">Verified</div><div class="value" id="metricVerified">-</div></div>
          <div class="metric"><div class="label">Unknown</div><div class="value" id="metricUnknown">-</div></div>
          <div class="metric"><div class="label">Pending</div><div class="value" id="metricPending">-</div></div>
          <div class="metric"><div class="label">Snapshots</div><div class="value" id="metricSnapshots">-</div></div>
        </div>
      </aside>
    </section>

    <section class="content-grid">
      <section class="panel">
        <div class="panel-header">
          <h2 class="panel-title">Saved Snapshots</h2>
          <div class="segmented" role="tablist" aria-label="Snapshot filters">
            <button class="active" data-filter="all">All</button>
            <button data-filter="arrival">Arrival</button>
            <button data-filter="unknown">Unknown</button>
            <button data-filter="fire">Fire / Gas</button>
            <button data-filter="verified">Verified</button>
            <button data-filter="person">Person</button>
            <button data-filter="face">Face</button>
            <button data-filter="other">Other</button>
          </div>
        </div>
        <div class="gallery" id="gallery"></div>
      </section>

      <section class="panel">
        <div class="panel-header">
          <h2 class="panel-title">Snapshot Detail</h2>
        </div>
        <div class="detail" id="snapshotDetail">
          <div class="empty">Select a snapshot</div>
        </div>
      </section>
    </section>

    <section class="panel">
      <div class="panel-header">
        <h2 class="panel-title">Event Log</h2>
        <p class="subtle" id="eventCount">-</p>
      </div>
      <div class="events">
        <table>
          <thead>
            <tr>
              <th>Time</th>
              <th>Event</th>
              <th>Track</th>
              <th>Name</th>
              <th>Mode</th>
            </tr>
          </thead>
          <tbody id="eventBody"></tbody>
        </table>
      </div>
    </section>
  </main>

  <div class="alert-overlay" id="dangerAlert" role="alert" aria-live="assertive">
    <section class="alert-card" id="dangerAlertCard">
      <button class="icon-button alert-close" id="alertClose" aria-label="Dismiss alert" title="Dismiss">
        <svg width="18" height="18" viewBox="0 0 24 24" aria-hidden="true">
          <path d="M6 6l12 12M18 6L6 18" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"/>
        </svg>
      </button>
      <div class="alert-eyebrow" id="alertEyebrow">Alert</div>
      <h2 class="alert-title" id="alertTitle">Danger detected</h2>
      <p class="alert-body" id="alertBody">A new high-priority event was recorded.</p>
    </section>
  </div>

  <script>
    const state = {
      filter: "all",
      snapshots: [],
      selectedUrl: "",
      statusPrimed: false,
      lastEventKey: "",
      seenAlertKeys: new Set(),
      alertTimer: null
    };

    const $ = (id) => document.getElementById(id);

    async function getJson(url) {
      const response = await fetch(url, { cache: "no-store" });
      if (!response.ok) {
        throw new Error(response.status + " " + response.statusText);
      }
      return response.json();
    }

    async function postJson(url) {
      const response = await fetch(url, { method: "POST", cache: "no-store" });
      if (!response.ok) {
        throw new Error(response.status + " " + response.statusText);
      }
      return response.json();
    }

    function text(value, fallback) {
      if (value === null || value === undefined || value === "") {
        return fallback || "-";
      }
      return String(value);
    }

    function formatBytes(bytes) {
      const n = Number(bytes || 0);
      if (n >= 1024 * 1024) return (n / (1024 * 1024)).toFixed(1) + " MB";
      if (n >= 1024) return (n / 1024).toFixed(1) + " KB";
      return n + " B";
    }

    function eventClass(eventName) {
      const name = String(eventName || "");
      if (name.indexOf("VERIFIED") >= 0) return "event-ok";
      if (name.indexOf("ARRIVAL") >= 0 || name.indexOf("ARRIVED") >= 0) return "event-ok";
      if (
        name.indexOf("UNKNOWN") >= 0 ||
        name.indexOf("ALERT") >= 0 ||
        name.indexOf("TIMEOUT") >= 0 ||
        name.indexOf("FIRE") >= 0 ||
        name.indexOf("GAS") >= 0 ||
        name.indexOf("SECURITY") >= 0 ||
        name.indexOf("INTRUDER") >= 0 ||
        name.indexOf("EMERGENCY") >= 0
      ) return "event-alert";
      return "";
    }

    function eventKey(event) {
      if (!event) return "";
      return [
        event.timestamp || "",
        event.event || "",
        event.track_id || "",
        event.name || "",
        event.mode || ""
      ].join("|");
    }

    function alertInfo(event) {
      if (!event) return null;
      const name = String(event.event || "").toUpperCase();
      const mode = String(event.mode || "").toUpperCase();
      const combined = name + " " + mode;

      if (combined.indexOf("EMERGENCY") >= 0) {
        return {
          kind: "stop",
          eyebrow: "Emergency Stop",
          title: "Emergency Stop Active",
          body: "The robot stop command was recorded at " + text(event.timestamp) + "."
        };
      }

      if (
        combined.indexOf("FIRE") >= 0 ||
        combined.indexOf("GAS") >= 0 ||
        combined.indexOf("SMOKE") >= 0 ||
        combined.indexOf("FLAME") >= 0 ||
        combined.indexOf("HAZARD") >= 0
      ) {
        return {
          kind: "fire",
          eyebrow: "Fire / Gas Alert",
          title: "Danger Detected",
          body: "Fire, gas, smoke, or hazard activity was recorded at " + text(event.timestamp) + "."
        };
      }

      if (
        combined.indexOf("UNKNOWN") >= 0 ||
        combined.indexOf("SECURITY") >= 0 ||
        combined.indexOf("INTRUDER") >= 0
      ) {
        const trackText = event.track_id ? " Track " + event.track_id + "." : "";
        return {
          kind: "unknown",
          eyebrow: "Security Alert",
          title: "Unrecognized Person",
          body: "An unverified person alert was recorded at " + text(event.timestamp) + "." + trackText
        };
      }

      return null;
    }

    function showDangerAlert(info) {
      const overlay = $("dangerAlert");
      const card = $("dangerAlertCard");
      $("alertEyebrow").textContent = info.eyebrow;
      $("alertTitle").textContent = info.title;
      $("alertBody").textContent = info.body;
      card.className = "alert-card " + info.kind;
      overlay.classList.add("show");

      if (state.alertTimer) {
        clearTimeout(state.alertTimer);
      }
      state.alertTimer = setTimeout(hideDangerAlert, 7000);
    }

    function hideDangerAlert() {
      $("dangerAlert").classList.remove("show");
      if (state.alertTimer) {
        clearTimeout(state.alertTimer);
        state.alertTimer = null;
      }
    }

    function handleLatestEvent(event) {
      const key = eventKey(event);
      if (!state.statusPrimed) {
        state.statusPrimed = true;
        state.lastEventKey = key;
        return;
      }
      if (!key || key === state.lastEventKey || state.seenAlertKeys.has(key)) {
        return;
      }

      state.lastEventKey = key;
      const info = alertInfo(event);
      if (!info) {
        return;
      }
      state.seenAlertKeys.add(key);
      showDangerAlert(info);
    }

    function updateStatus(payload) {
      $("serverTime").textContent = "Server time " + text(payload.server_time);
      $("metricFps").textContent = payload.fps === undefined ? "-" : Number(payload.fps).toFixed(1);
      $("metricPeople").textContent = text(payload.people);
      $("metricVerified").textContent = text(payload.verified);
      $("metricUnknown").textContent = text(payload.unknown);
      $("metricPending").textContent = text(payload.pending);

      const counts = payload.snapshot_counts || {};
      const total = Object.keys(counts).reduce((sum, key) => sum + Number(counts[key] || 0), 0);
      $("metricSnapshots").textContent = total;

      const hasFrame = Boolean(payload.has_frame);
      const age = payload.frame_age_seconds;
      $("videoPlaceholder").style.display = hasFrame ? "none" : "grid";
      $("frameAge").textContent = hasFrame ? "Frame age " + age + " s" : "No frame yet";

      $("liveDot").className = "status-dot";
      if (hasFrame && Number(age) <= 3) {
        $("liveDot").classList.add("live");
      } else if (hasFrame) {
        $("liveDot").classList.add("stale");
      }

      handleLatestEvent(payload.latest_event);
    }

    function renderSnapshots() {
      const gallery = $("gallery");
      gallery.innerHTML = "";
      const visible = state.snapshots.filter((item) => state.filter === "all" || item.category === state.filter);

      if (!visible.length) {
        const empty = document.createElement("div");
        empty.className = "empty";
        empty.textContent = "No snapshots";
        gallery.appendChild(empty);
        renderDetail(null);
        return;
      }

      if (!state.selectedUrl || !visible.some((item) => item.url === state.selectedUrl)) {
        state.selectedUrl = visible[0].url;
      }

      visible.forEach((item) => {
        const button = document.createElement("button");
        button.className = "snapshot" + (item.url === state.selectedUrl ? " active" : "");
        button.type = "button";
        button.addEventListener("click", () => {
          state.selectedUrl = item.url;
          renderSnapshots();
        });

        const img = document.createElement("img");
        img.src = item.url;
        img.alt = item.filename;
        button.appendChild(img);

        const meta = document.createElement("div");
        meta.className = "snapshot-meta";

        const category = document.createElement("span");
        category.className = "category kind-" + item.category;
        category.textContent = item.category_label || item.category;
        meta.appendChild(category);

        const filename = document.createElement("div");
        filename.className = "filename";
        filename.textContent = item.saved_at || item.filename;
        meta.appendChild(filename);

        button.appendChild(meta);
        gallery.appendChild(button);
      });

      const selected = state.snapshots.find((item) => item.url === state.selectedUrl);
      renderDetail(selected || visible[0]);
    }

    function renderDetail(item) {
      const detail = $("snapshotDetail");
      detail.innerHTML = "";
      if (!item) {
        const empty = document.createElement("div");
        empty.className = "empty";
        empty.textContent = "Select a snapshot";
        detail.appendChild(empty);
        return;
      }

      const img = document.createElement("img");
      img.className = "detail-image";
      img.src = item.url;
      img.alt = item.filename;
      detail.appendChild(img);

      const list = document.createElement("div");
      list.className = "detail-list";
      [
        ["Group", item.category_label || item.category],
        ["Folder", item.raw_category],
        ["Track", item.track_id],
        ["Saved", item.saved_at],
        ["Size", formatBytes(item.size_bytes)],
        ["File", item.filename]
      ].forEach((entry) => {
        const row = document.createElement("div");
        const label = document.createElement("strong");
        label.textContent = entry[0] + ":";
        row.appendChild(label);
        row.appendChild(document.createTextNode(" " + text(entry[1])));
        list.appendChild(row);
      });
      detail.appendChild(list);
    }

    function renderEvents(events) {
      const body = $("eventBody");
      body.innerHTML = "";
      $("eventCount").textContent = events.length + " recent";

      if (!events.length) {
        const row = document.createElement("tr");
        const cell = document.createElement("td");
        cell.colSpan = 5;
        cell.className = "empty";
        cell.textContent = "No events";
        row.appendChild(cell);
        body.appendChild(row);
        return;
      }

      events.forEach((event) => {
        const row = document.createElement("tr");
        const eventName = text(event.event);
        [
          text(event.timestamp),
          eventName,
          text(event.track_id),
          text(event.name),
          text(event.mode)
        ].forEach((value, index) => {
          const cell = document.createElement("td");
          cell.textContent = value;
          if (index === 1) {
            cell.className = eventClass(eventName);
          }
          row.appendChild(cell);
        });
        body.appendChild(row);
      });
    }

    async function refreshAll() {
      try {
        const status = await getJson("/api/status");
        updateStatus(status);
      } catch (error) {
        $("serverTime").textContent = "Dashboard connection lost";
        $("liveDot").className = "status-dot stale";
      }

      try {
        const data = await getJson("/api/snapshots?limit=160");
        state.snapshots = data.snapshots || [];
        renderSnapshots();
      } catch (error) {
        state.snapshots = [];
        renderSnapshots();
      }

      try {
        const data = await getJson("/api/events?limit=80");
        renderEvents(data.events || []);
      } catch (error) {
        renderEvents([]);
      }
    }

    async function sendEmergencyStop() {
      const button = $("emergencyStopButton");
      const status = $("controlStatus");
      button.disabled = true;
      status.textContent = "Sending stop";

      try {
        const result = await postJson("/api/control/emergency-stop");
        status.textContent = result.arduino_connected
          ? "Emergency stop sent"
          : "Stop queued; Arduino offline";
        const event = {
          timestamp: result.server_time || "",
          event: "EMERGENCY_STOP",
          track_id: "-1",
          name: "",
          mode: "WEB"
        };
        const info = alertInfo(event);
        if (info) {
          const key = eventKey(event);
          state.statusPrimed = true;
          state.lastEventKey = key;
          state.seenAlertKeys.add(key);
          showDangerAlert(info);
        }
        await refreshAll();
      } catch (error) {
        status.textContent = "Emergency stop failed";
      } finally {
        setTimeout(() => {
          button.disabled = false;
        }, 900);
      }
    }

    async function sendAutoPatrol() {
      const button = $("autoPatrolButton");
      const status = $("controlStatus");
      button.disabled = true;
      status.textContent = "Starting patrol";

      try {
        const result = await postJson("/api/control/auto-patrol");
        status.textContent = result.arduino_connected
          ? "Start patrol sent"
          : "Start queued; Arduino offline";
        await refreshAll();
      } catch (error) {
        status.textContent = "Start patrol failed";
      } finally {
        setTimeout(() => {
          button.disabled = false;
        }, 900);
      }
    }

    async function sendPatrolStop() {
      const button = $("patrolStopButton");
      const status = $("controlStatus");
      button.disabled = true;
      status.textContent = "Stopping patrol";

      try {
        const result = await postJson("/api/control/patrol-stop");
        status.textContent = result.arduino_connected
          ? "Stop patrol sent"
          : "Stop queued; Arduino offline";
        await refreshAll();
      } catch (error) {
        status.textContent = "Stop patrol failed";
      } finally {
        setTimeout(() => {
          button.disabled = false;
        }, 900);
      }
    }

    document.querySelectorAll("[data-filter]").forEach((button) => {
      button.addEventListener("click", () => {
        document.querySelectorAll("[data-filter]").forEach((item) => item.classList.remove("active"));
        button.classList.add("active");
        state.filter = button.dataset.filter;
        state.selectedUrl = "";
        renderSnapshots();
      });
    });

    $("alertClose").addEventListener("click", hideDangerAlert);
    $("dangerAlert").addEventListener("click", (event) => {
      if (event.target === $("dangerAlert")) {
        hideDangerAlert();
      }
    });
    $("refreshButton").addEventListener("click", refreshAll);
    $("emergencyStopButton").addEventListener("click", sendEmergencyStop);
    $("autoPatrolButton").addEventListener("click", sendAutoPatrol);
    $("patrolStopButton").addEventListener("click", sendPatrolStop);
    refreshAll();
    setInterval(refreshAll, 2500);
  </script>
</body>
</html>
"""
