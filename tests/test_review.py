import json
from pathlib import Path

import pytest

from mobile_robot.recording import Recorder, classify_episode, update_metadata
from mobile_robot.review import review_result


@pytest.fixture
def episode(tmp_path):
    recorder = Recorder(tmp_path / "unreviewed", {"recording_status": "complete"})
    recorder.submit({"sample": 1}, {"car": ({"seq": 1}, b"jpeg-data")})
    recorder.close()
    return recorder.path


@pytest.mark.parametrize("quality", ["good", "bad", "unreviewed"])
def test_classification_preserves_episode(episode, tmp_path, quality):
    original = (episode / "telemetry.jsonl").read_bytes()
    destination = classify_episode(episode, quality, tmp_path)
    assert destination.parent == tmp_path / quality
    assert (destination / "telemetry.jsonl").read_bytes() == original
    row = json.loads(original)
    assert (destination / row["images"]["car"]["file"]).read_bytes() == b"jpeg-data"
    assert json.loads((destination / "metadata.json").read_text())["quality"] == quality


def test_classification_refuses_overwrite_and_active_recording(episode, tmp_path):
    (tmp_path / "good" / episode.name).mkdir(parents=True)
    with pytest.raises(FileExistsError):
        classify_episode(episode, "good", tmp_path)
    update_metadata(episode, recording_status="recording")
    with pytest.raises(ValueError, match="active"):
        classify_episode(episode, "bad", tmp_path)
    update_metadata(episode, recording_status="error")
    with pytest.raises(ValueError, match="error"):
        classify_episode(episode, "good", tmp_path)
    assert episode.exists()


def test_failed_move_restores_metadata(episode, tmp_path, monkeypatch):
    original = (episode / "metadata.json").read_bytes()

    def fail(*args):
        raise OSError("move failed")

    monkeypatch.setattr(Path, "rename", fail)
    with pytest.raises(OSError, match="move failed"):
        classify_episode(episode, "good", tmp_path)
    assert (episode / "metadata.json").read_bytes() == original


@pytest.mark.parametrize(
    "answer,quality",
    [
        ("G", "good"),
        ("bad", "bad"),
        ("", "unreviewed"),
        (EOFError, "unreviewed"),
        (KeyboardInterrupt, "unreviewed"),
    ],
)
def test_prompt_handles_choices_and_cancellation(episode, tmp_path, monkeypatch, answer, quality):
    monkeypatch.setattr("sys.stdin.isatty", lambda: True)
    answers = iter(["invalid", answer])

    def respond(_prompt):
        value = next(answers)
        if isinstance(value, type):
            raise value()
        return value

    monkeypatch.setattr("builtins.input", respond)
    result = review_result({"path": str(episode), "rows": 1}, tmp_path)
    assert result["quality"] == quality
    assert Path(result["path"]).parent == tmp_path / quality


def test_noninteractive_review_never_prompts(episode, tmp_path, monkeypatch):
    monkeypatch.setattr("sys.stdin.isatty", lambda: False)
    monkeypatch.setattr("builtins.input", lambda _: pytest.fail("Unexpected prompt"))
    assert review_result({"path": str(episode)}, tmp_path)["quality"] == "unreviewed"
