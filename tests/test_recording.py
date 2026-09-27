import json
import threading

import pytest

from mobile_robot.recording import Recorder


def test_episode_uniqueness_frame_reuse_and_final_drain(tmp_path):
    first = Recorder(tmp_path, {"mock": True})
    second = Recorder(tmp_path, {"mock": True})
    assert first.path != second.path
    frames = {"wrist": ({"seq": 1}, b"jpeg")}
    for i in range(5):
        first.submit({"row": i}, frames)
    first.close()
    second.close()
    rows = [json.loads(row) for row in (first.path / "telemetry.jsonl").read_text().splitlines()]
    assert len(rows) == first.saved == 5
    assert len(list((first.path / "images").iterdir())) == 1


def test_disk_failure_is_reported(tmp_path, monkeypatch):
    from pathlib import Path

    recorder = Recorder(tmp_path, {})

    def broken(*args):
        raise OSError("disk full")

    monkeypatch.setattr(Path, "write_bytes", broken)
    recorder.submit({}, {"wrist": ({"seq": 0}, b"data")})
    with pytest.raises(RuntimeError, match="disk full"):
        recorder.close()


def test_full_queue_fails_without_blocking_control(tmp_path, monkeypatch):
    gate = threading.Event()
    monkeypatch.setattr(Recorder, "run", lambda self: gate.wait(2))
    recorder = Recorder(tmp_path, {}, max_queue=1)
    try:
        recorder.submit({}, {})
        with pytest.raises(RuntimeError, match="cannot keep up"):
            recorder.submit({}, {})
    finally:
        gate.set()
        recorder.thread.join(2)
        recorder.close()
