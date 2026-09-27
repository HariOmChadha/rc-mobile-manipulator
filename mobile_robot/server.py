import argparse
import json
import logging
import os
import signal
import threading

from .cameras import Capture, Publisher
from .config import load_config
from .control import Controller
from .hardware import Arm, MockArm
from .protocol import VERSION

LOG = logging.getLogger(__name__)


def get_token(mock=False):
    token = os.environ.get("ROBOT_TOKEN", "simulation-only" if mock else "")
    if not token or not token.isascii():
        raise ValueError("Set ROBOT_TOKEN to the same ASCII secret on the Pi and laptop")
    return token


def serve(cfg, *, mock=False, stop_event=None, ready=None):
    import zmq

    stop_event = stop_event or threading.Event()
    token = get_token(mock)
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.MAXMSGSIZE, 65536)
    arm = controller = publisher = None
    captures = []
    try:
        socket.bind(cfg["control_bind"])
        arm = (
            MockArm(cfg["max_relative_target"])
            if mock
            else Arm(cfg["follower"], max_step=cfg["max_relative_target"])
        )
        controller = Controller(arm, token, timeout=cfg["command_timeout_s"], mock=mock)
        for camera in cfg["cameras"]:
            captures.append(Capture(camera, mock=mock).start())
        publisher = Publisher(cfg["video_bind"], captures).start()
        LOG.info("Pi ready: control=%s video=%s mock=%s", cfg["control_bind"], cfg["video_bind"], mock)
        if ready:
            ready.set()
        while not stop_event.is_set():
            controller.tick()
            if not socket.poll(10):
                continue
            try:
                message = socket.recv_json()
                response = controller.handle(message)
                response["cameras"] = {
                    c.config["name"]: {"ready": c.snapshot() is not None, "error": c.error} for c in captures
                }
                response["video_error"] = publisher.error
            except (ValueError, TypeError) as error:
                response = {"version": VERSION, "ok": False, "error": str(error)}
            except Exception:
                # A hardware failure terminates the server instead of accepting further motion.
                LOG.exception("Arm control failed")
                socket.send_json(
                    {"version": VERSION, "ok": False, "error": "Arm hardware failure; inspect Pi logs"}
                )
                raise
            socket.send_json(response)
    finally:
        if controller:
            try:
                controller.stop("server_shutdown")
            except Exception:
                LOG.exception("Could not hold arm during shutdown")
        try:
            if arm:
                arm.close()
        finally:
            if publisher:
                publisher.close()
            for capture in captures:
                capture.close()
            socket.close()
            context.term()


def main():
    parser = argparse.ArgumentParser(description="SO101 Pi host: local arm USB, independent camera streams")
    parser.add_argument("--config", default="config/pi.json")
    parser.add_argument("--mock", action="store_true", help="Synthetic arm/cameras; never touches USB")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        serve(load_config(args.config), mock=args.mock, stop_event=stop)
    except (ValueError, OSError, RuntimeError, ImportError, json.JSONDecodeError) as error:
        parser.exit(1, f"Pi startup/runtime error: {error}\n")


if __name__ == "__main__":
    main()
