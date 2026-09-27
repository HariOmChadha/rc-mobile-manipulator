import json
import threading

import cv2
import numpy as np
import pytest

from mobile_robot.inference import MockCar
from mobile_robot.protocol import JOINTS
from mobile_robot.replay import Episode, render, stream_robot
from test_integration import network  # noqa: F401


@pytest.fixture
def episode(tmp_path):
    folder = tmp_path / "episode"
    folder.mkdir()
    (folder / "metadata.json").write_text(
        json.dumps(
            {
                "recording_status": "complete",
                "mock": False,
                "joint_units": "normalized_minus100_100_gripper_0_100",
                "rc_units": "raw_dac_0_255",
            }
        )
    )
    images = {}
    for name, color in [("scene", (0, 0, 240)), ("wrist", (0, 240, 0)), ("car", (240, 0, 0))]:
        cv2.imwrite(str(folder / f"{name}.jpg"), np.full((90, 160, 3), color, np.uint8))
        images[name] = {"file": f"{name}.jpg"}
    rows = [
        {
            "timestamp": 100 + i / 30,
            "applied_action": dict.fromkeys(JOINTS, float(i)),
            "rc_state": {"values": {"steer": 17 + i, "throttle": 139 - i}},
            "images": images,
            "phase": "drive to object",
        }
        for i in range(5)
    ]
    (folder / "telemetry.jsonl").write_text("\n".join(map(json.dumps, rows)))
    return Episode(folder)


def test_time_selection_and_raw_dac(episode):
    assert episode.index_at(-1) == 0
    assert episode.index_at(0.05) == 1
    assert episode.index_at(100) == 4
    assert [dac for _, dac in episode.commands()] == [(17 + i, 139 - i) for i in range(5)]


@pytest.mark.parametrize("error", ["dac", "joint", "time", "missing", "path"])
def test_invalid_episode_preflight(episode, error):
    if error == "dac":
        episode.rows[1]["rc_state"]["values"]["throttle"] = 146
    elif error == "joint":
        episode.rows[1]["applied_action"][JOINTS[0]] = float("nan")
    elif error == "time":
        episode.times[2] = episode.times[1] + 1
    elif error == "missing":
        episode.rows[1]["rc_state"]["values"] = None
    else:
        with pytest.raises(ValueError):
            episode.image_path("../outside.jpg")
        return
    with pytest.raises(ValueError):
        episode.commands()


def test_video_has_three_views_on_one_frame(episode, tmp_path):
    av = pytest.importorskip("av")
    output = tmp_path / "replay.mp4"
    report = render(episode, output)
    with av.open(str(output)) as video:
        frames = list(video.decode(video=0))
    assert len(frames) == report["frames"]
    frame = frames[0].to_ndarray(format="bgr24")
    assert frame.shape == (360, 1440, 3)
    assert frame[170, 240, 2] > 200  # scene red
    assert frame[170, 720, 1] > 200  # wrist green
    assert frame[170, 1200, 0] > 200  # car blue


def test_replay_over_tcp_preserves_dac_and_stops(network, episode):  # noqa: F811
    from mobile_robot.control import Remote

    cfg, _, _ = network
    cars = []

    class RecordingCar(MockCar):
        def __init__(self, *args):
            super().__init__(*args)
            cars.append(self)

    result = stream_robot(
        episode, cfg, {"neutral_steer": 128, "neutral_throttle": 100}, mock=True, car_factory=RecordingCar
    )
    assert result["commands_sent"] == 5
    assert cars[0].commands == [(17 + i, 139 - i) for i in range(5)]
    assert cars[0].stopped
    remote = Remote(cfg["control_endpoint"], "integration-test")
    try:
        state = remote.call("status")
        assert state["reason"] == "client_stop"
        assert state["follower_joints"] == dict.fromkeys(JOINTS, 4)
    finally:
        remote.close()


def test_initial_pose_mismatch_sends_no_commands(network, episode):  # noqa: F811
    cfg, _, _ = network
    episode.rows[0]["applied_action"][JOINTS[0]] = 50
    with pytest.raises(ValueError, match="starting pose"):
        stream_robot(
            episode,
            cfg,
            {"neutral_steer": 128, "neutral_throttle": 100},
            mock=True,
            car_factory=lambda *args: pytest.fail("ESP must not be opened"),
        )


def test_interrupted_replay_stops_car(network, episode):  # noqa: F811
    cfg, _, _ = network
    stop = threading.Event()
    cars = []

    class StopCar(MockCar):
        def __init__(self, *args):
            super().__init__(*args)
            cars.append(self)

        def drive(self, *args):
            super().drive(*args)
            stop.set()

    result = stream_robot(
        episode,
        cfg,
        {"neutral_steer": 128, "neutral_throttle": 100},
        mock=True,
        car_factory=StopCar,
        stop_event=stop,
    )
    assert result["commands_sent"] == 1
    assert cars[0].stopped
