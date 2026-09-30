import importlib.util
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from mobile_robot.protocol import JOINTS, parse_rc_line


@pytest.fixture
def recorder():
    spec = importlib.util.spec_from_file_location("wired", Path("scripts/record_wired.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_wired_row_separates_targets_feedback_and_correct_rc_labels(recorder, tmp_path):
    (tmp_path / "images").mkdir()
    leader = SimpleNamespace(get_action=lambda: dict.fromkeys(JOINTS, np.float64(20)))
    follower = SimpleNamespace(
        send_action=lambda target: dict.fromkeys(JOINTS, 15),
        get_observation=lambda: dict.fromkeys(JOINTS, np.float64(12)),
    )
    rc = SimpleNamespace(
        read=lambda: {"values": parse_rc_line("100,200"), "age_s": 0.01, "units": "raw_dac_0_255"}
    )
    camera = SimpleNamespace(read=lambda: (True, np.zeros((24, 32, 3), np.uint8)))
    row = recorder.record_frame(leader, follower, rc, [camera, camera], tmp_path, 0)
    assert row["rc_state"] == {"steer": 100, "throttle": 200}
    assert set(row["leader_action"].values()) == {20}
    assert set(row["applied_action"].values()) == {15}
    assert set(row["follower_joints"].values()) == {12}
    assert len(row["image_files"]) == 2
    for filename in row["image_files"].values():
        assert cv2.imread(str(tmp_path / "images" / filename)).shape == (24, 32, 3)


def test_stale_rc_never_sends_wired_motion(recorder, tmp_path):
    rc = SimpleNamespace(read=lambda: {"values": None, "age_s": None})
    with pytest.raises(RuntimeError, match="stale"):
        recorder.record_frame(None, None, rc, [], tmp_path, 0)
