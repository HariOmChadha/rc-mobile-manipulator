"""Bounded asynchronous storage. Never silently drop a row or overwrite an episode."""

import json
import queue
import threading
import time
import uuid
from pathlib import Path


def update_metadata(episode, **fields):
    path = Path(episode) / "metadata.json"
    metadata = json.loads(path.read_text())
    metadata.update(fields)
    temporary = path.with_name(f".metadata-{uuid.uuid4().hex}.tmp")
    try:
        temporary.write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def classify_episode(episode, quality, root):
    """Move a closed episode intact, preserving all relative image references."""
    if quality not in ("good", "bad", "unreviewed"):
        raise ValueError("Quality must be good, bad or unreviewed")
    source = Path(episode).resolve()
    metadata = json.loads((source / "metadata.json").read_text())
    if metadata.get("recording_status") == "recording":
        raise ValueError("Episode is still marked as recording; do not move an active recording")
    if not (source / "telemetry.jsonl").is_file():
        raise ValueError("Episode has no telemetry file")
    if quality == "good" and metadata.get("recording_status") == "error":
        raise ValueError("Recording ended with an error; keep it unreviewed or classify it as bad")
    destination = Path(root).resolve() / quality / source.name
    if destination != source and destination.exists():
        raise FileExistsError(f"Episode already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    old_metadata = (source / "metadata.json").read_text()
    update_metadata(source, quality=quality, classified_at=time.time())
    try:
        if destination != source:
            source.rename(destination)
    except OSError:
        (source / "metadata.json").write_text(old_metadata)
        raise
    return destination


class Recorder:
    def __init__(self, root, metadata, *, max_queue=60):
        self.path = Path(root) / f"episode_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / "images").mkdir()
        (self.path / "metadata.json").write_text(json.dumps({"schema_version": 1, **metadata}, indent=2))
        self.queue = queue.Queue(maxsize=max_queue)
        self.error = None
        self.saved = 0
        self.thread = threading.Thread(target=self.run, daemon=True, name="dataset-writer")
        self.thread.start()

    def submit(self, row, frames):
        if self.error:
            raise RuntimeError(f"Dataset writer failed: {self.error}")
        try:
            self.queue.put_nowait((row, frames))
        except queue.Full as error:
            raise RuntimeError(
                "Dataset disk writer cannot keep up; stopping instead of losing rows"
            ) from error

    def run(self):
        known = set()
        try:
            with (self.path / "telemetry.jsonl").open("x") as stream:
                while True:
                    item = self.queue.get()
                    if item is None:
                        break
                    row, frames = item
                    images = {}
                    for name, (meta, jpeg) in frames.items():
                        filename = f"{name}_{meta['seq']:08d}.jpg"
                        if filename not in known:
                            (self.path / "images" / filename).write_bytes(jpeg)
                            known.add(filename)
                        images[name] = {**meta, "file": f"images/{filename}"}
                    stream.write(json.dumps({**row, "images": images}, allow_nan=False) + "\n")
                    stream.flush()
                    self.saved += 1
        except Exception as error:
            self.error = error

    def close(self):
        deadline = time.monotonic() + 10
        while self.thread.is_alive():
            if time.monotonic() >= deadline:
                raise RuntimeError("Dataset writer did not finish within 10 seconds")
            try:
                self.queue.put(None, timeout=0.1)
                break
            except queue.Full:
                continue
        self.thread.join(timeout=max(0, deadline - time.monotonic()))
        if self.thread.is_alive():
            raise RuntimeError("Dataset writer did not finish within 10 seconds")
        if self.error:
            raise RuntimeError(f"Dataset writer failed: {self.error}")
