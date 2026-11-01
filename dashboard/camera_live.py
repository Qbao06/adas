"""Real camera acquisition for the existing cockpit; no inference or simulated data."""
from __future__ import annotations
import glob
import json
import threading
import time
from pathlib import Path
from dashboard.server import DashboardHandler, DashboardServer


class CameraFeed:
    def __init__(self, sources):
        self.sources = sources
        self.frames = {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.active = next(iter(sources))
        self.threads = []
        self.announced = set()

    def start(self):
        # USB 2.0 controllers can reserve all isochronous bandwidth for the
        # first UVC camera opened. Read one frame from each camera in turn so
        # a shared hub can still display every connected view.
        thread = threading.Thread(target=self.capture_round_robin, daemon=True)
        thread.start()
        self.threads.append(thread)

    def capture_round_robin(self):
        import cv2
        while not self.stop.is_set():
            for name, source in self.sources.items():
                if self.stop.is_set():
                    break
                cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
                try:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 352)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 288)
                    cap.set(cv2.CAP_PROP_FPS, 10)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    ok, frame = cap.read()
                    if ok and frame is not None:
                        encoded, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                        if encoded:
                            with self.lock:
                                self.frames[name] = (time.monotonic(), jpg.tobytes())
                            if name not in self.announced:
                                print(f"[camera-only] LIVE {name}: {source}", flush=True)
                                self.announced.add(name)
                finally:
                    cap.release()

    def fresh(self):
        with self.lock:
            return {k: v for k, v in self.frames.items() if time.monotonic() - v[0] < 5}

    def status(self):
        available = list(self.fresh())
        # Do not leave the cockpit bound to a camera that has briefly reset.
        # The next healthy camera remains visible until the selected one returns.
        if available and self.active not in available:
            self.active = available[0]
        return {
            "ready": bool(available), "demo": False, "mode": "camera-only",
            "errors": [f"Chưa có hình: {name} ({source})" for name, source in self.sources.items() if name not in available],
            "snapshot": {
                "timestamp": time.time(), "mode": "camera-only", "demo": False,
                "active_camera": self.active, "cameras_available": available,
                "camera_sources": self.sources,
                "warning": {"level": "unknown", "distance_m": None,
                            "message": "CAMERA THẬT · AI ĐANG TẮT · VỊ TRÍ CAMERA GÁN TẠM"},
                "vehicle": {"gear": "P", "turn_signal": "none"},
                "detections": [], "lidar_points": [], "side_radar": {"link_ok": False},
                "fps": None, "latency_ms": None,
                "health": {"camera": "ok" if self.active in available else "error",
                           "ai": "disabled", "lidar": "disabled", "fusion": "disabled",
                           "calibration": "disabled", "system": "ok" if available else "error"}
            }
        }

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=2)


class CameraHandler(DashboardHandler):
    def do_GET(self):
        path = self.path.split("?")[0]
        feed = self.server.camera_feed
        if path == "/api/status":
            self._json(feed.status())
        elif path.startswith("/api/camera/") and path.endswith(".jpg"):
            name = path[len("/api/camera/"):-4]
            item = feed.fresh().get(feed.active if name == "active" else name)
            self._send(item[1] if item else b"Camera offline", "image/jpeg" if item else "text/plain", 200 if item else 503)
        else:
            super().do_GET()

    def do_POST(self):
        if self.path != "/api/camera/select":
            self._json({"error": "Chế độ chỉ camera: chức năng này đang tắt."}, 403)
            return
        try:
            size = int(self.headers.get("Content-Length", 0))
            if not 0 < size <= 1024:
                raise ValueError("Invalid body length")
            data = json.loads(self.rfile.read(size))
            name = data.get("camera")
            if name not in self.server.camera_feed.sources:
                raise ValueError("Camera không có trong cấu hình")
            self.server.camera_feed.active = name
            self._json({"ok": True})
        except (ValueError, AttributeError, TypeError) as exc:
            self._json({"error": str(exc)}, 400)

    def do_PUT(self):
        self._json({"error": "Chế độ chỉ camera: không sửa cấu hình ADAS."}, 403)

    def log_message(self, *args):
        pass


def discover_sources():
    """Return external UVC capture nodes only; never use the laptop webcam."""
    # UVC sysfs index 0 is the capture node; index 1 is metadata.
    nodes = []
    for node in sorted(glob.glob("/sys/class/video4linux/video*"), key=lambda p: int(Path(p).name[5:])):
        node_path = Path(node)
        try:
            index = (node_path / "index").read_text().strip()
            device_name = (node_path / "name").read_text().strip().lower()
        except OSError:
            continue
        if index != "0" or "integrated_webcam" in device_name or "integrated webcam" in device_name:
            continue
        device = "/dev/" + node_path.name
        if Path(device).exists():
            nodes.append(device)
    return dict(zip(("front", "left", "right", "rear", "left_2", "right_2"), nodes))


def run_camera_dashboard(root, *, port=8765, source=None):
    sources = {"front": int(source) if source.isdigit() else source} if source else discover_sources()
    if not sources:
        print("Không tìm thấy camera thật.", flush=True)
        return 1
    feed = CameraFeed(sources)
    server = DashboardServer(("127.0.0.1", port), CameraHandler,
                             root / "logs/preflight_status.json", root / "config/jetson_live.yaml")
    server.camera_feed = feed
    feed.start()
    print(f"[camera-only] http://127.0.0.1:{port} sources={sources}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        feed.close()
    return 0
