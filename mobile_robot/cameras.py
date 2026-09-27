"""Independent capture, JPEG publishing and latest-frame reception."""

import json
import logging
import threading
import time

from .protocol import VERSION

LOG = logging.getLogger(__name__)


def describe_mode(cap, cfg):
    """V4L2/OpenCV reports negotiated settings; zero means unknown, not verified."""
    import cv2
    import math

    reported = {}
    for key, prop in (
        ("width", cv2.CAP_PROP_FRAME_WIDTH),
        ("height", cv2.CAP_PROP_FRAME_HEIGHT),
        ("fps", cv2.CAP_PROP_FPS),
        ("fourcc", cv2.CAP_PROP_FOURCC),
    ):
        value = cap.get(prop)
        if not math.isfinite(value) or value <= 0:
            reported[key] = None
        elif key == "fourcc":
            reported[key] = "".join(chr((int(value) >> (8 * i)) & 255) for i in range(4))
        else:
            reported[key] = value
    mismatches = []
    for key in ("width", "height", "fps", "fourcc"):
        value = reported[key]
        if value is None:
            LOG.warning("Camera %s does not report %s; verify it on hardware", cfg["name"], key)
        elif (key == "fourcc" and value != cfg[key]) or (key != "fourcc" and abs(value - cfg[key]) > 0.5):
            mismatches.append(f"{key}: requested {cfg[key]}, reported {value}")
    if mismatches:
        detail = f"Camera {cfg['name']} mode mismatch: " + "; ".join(mismatches)
        if cfg.get("strict_mode"):
            raise RuntimeError(detail)
        LOG.warning("%s", detail)
    return reported


class Capture:
    def __init__(self, config, *, mock=False):
        self.config, self.mock = config, mock
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name=f"camera-{config['name']}")

    def start(self):
        self.thread.start()
        return self

    def snapshot(self):
        with self.lock:
            return self.latest

    def run(self):
        import cv2
        import numpy as np

        cfg = self.config
        cap = None
        seq = 0
        first_capture = None
        requested = {key: cfg[key] for key in ("width", "height", "fps", "fourcc", "jpeg_quality")}
        identity = {
            key: cfg[key]
            for key in ("profile", "model", "device", "label", "fov_claim", "source")
            if key in cfg
        }
        reported = {key: cfg[key] for key in ("width", "height", "fps", "fourcc")} if self.mock else None
        try:
            if not self.mock:
                cap = cv2.VideoCapture(cfg["device"], cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError(f"Cannot open camera {cfg['name']}: {cfg['device']}")
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cfg["fourcc"]))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["width"])
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["height"])
                cap.set(cv2.CAP_PROP_FPS, cfg["fps"])
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                reported = describe_mode(cap, cfg)
            while not self.stop_event.is_set():
                start = time.monotonic()
                if self.mock:
                    frame = np.zeros((cfg["height"], cfg["width"], 3), dtype=np.uint8)
                    frame[:, :, 1] = seq % 255
                    cv2.putText(
                        frame,
                        f"{cfg['name']} {seq}",
                        (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        1,
                    )
                else:
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        raise RuntimeError(f"Camera read failed: {cfg['name']}")
                # Receipt time, not a claim about exposure time or hardware synchronization.
                captured_mono, captured_wall = time.monotonic(), time.time()
                if first_capture is None:
                    first_capture = captured_mono
                if cfg.get("strict_mode") and frame.shape[:2] != (cfg["height"], cfg["width"]):
                    raise RuntimeError(f"Camera {cfg['name']} frame dimensions differ from requested mode")
                ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, cfg["jpeg_quality"]])
                if not ok:
                    raise RuntimeError(f"JPEG encoding failed: {cfg['name']}")
                meta = {
                    "version": VERSION,
                    "name": cfg["name"],
                    "seq": seq,
                    "timestamp": captured_wall,
                    "capture_monotonic": captured_mono,
                    "width": frame.shape[1],
                    "height": frame.shape[0],
                    "mock": self.mock,
                    "camera": identity,
                    "requested_mode": requested,
                    "reported_mode": reported,
                    "measured_capture_fps": seq / (captured_mono - first_capture) if seq else None,
                }
                with self.lock:
                    self.latest = (meta, jpeg.tobytes())
                seq += 1
                if self.mock:
                    self.stop_event.wait(max(0, 1 / cfg["fps"] - (time.monotonic() - start)))
        except Exception as error:
            self.error = str(error)
            LOG.error("%s", error)
        finally:
            if cap is not None:
                cap.release()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            LOG.warning("Camera driver is blocked; capture thread will exit with the process")


class Publisher:
    def __init__(self, endpoint, captures):
        self.endpoint, self.captures = endpoint, captures
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True, name="camera-publisher")

    def start(self):
        self.thread.start()
        if not self.ready.wait(3) or self.error:
            raise RuntimeError(self.error or "Camera publisher did not start")
        return self

    def run(self):
        import zmq

        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.setsockopt(zmq.SNDHWM, 2)
        socket.setsockopt(zmq.LINGER, 0)
        sent = {}
        try:
            socket.bind(self.endpoint)
            self.ready.set()
            while not self.stop_event.is_set():
                for capture in self.captures:
                    item = capture.snapshot()
                    if item is None:
                        continue
                    meta, jpeg = item
                    name = meta["name"]
                    if sent.get(name) == meta["seq"]:
                        continue
                    meta = {**meta, "publish_monotonic": time.monotonic()}
                    socket.send_multipart([name.encode(), json.dumps(meta).encode(), jpeg], zmq.NOBLOCK)
                    sent[name] = meta["seq"]
                self.stop_event.wait(0.002)
        except Exception as error:
            self.error = str(error)
            self.ready.set()
            LOG.error("Camera publisher: %s", error)
        finally:
            socket.close()
            context.term()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=3)


class Subscriber:
    def __init__(self, endpoint):
        self.endpoint = endpoint
        self.lock = threading.Lock()
        self.latest = {}
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name="camera-subscriber")

    def start(self):
        self.thread.start()
        return self

    def run(self):
        import zmq

        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        socket.setsockopt(zmq.RCVHWM, 4)
        socket.setsockopt(zmq.LINGER, 0)
        socket.connect(self.endpoint)
        try:
            while not self.stop_event.is_set():
                if not socket.poll(50):
                    continue
                parts = socket.recv_multipart()
                try:
                    if len(parts) != 3:
                        continue
                    meta = json.loads(parts[1])
                    if meta["version"] != VERSION or parts[0].decode() != meta["name"]:
                        continue
                    meta["received_monotonic"] = time.monotonic()
                    meta["received_timestamp"] = time.time()
                    with self.lock:
                        self.latest[meta["name"]] = (meta, parts[2])
                except (ValueError, KeyError, UnicodeError, TypeError):
                    continue
        finally:
            socket.close()
            context.term()

    def snapshot(self):
        with self.lock:
            return dict(self.latest)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=3)


def frame_age(meta, *, now=None, server_monotonic=None, state_received_monotonic=None):
    """Use server monotonic times plus local elapsed time; no synchronized clocks required."""
    now = time.monotonic() if now is None else now
    if server_monotonic is not None:
        return max(0, server_monotonic - meta["capture_monotonic"] + now - state_received_monotonic)
    return max(0, now - meta["capture_monotonic"])
