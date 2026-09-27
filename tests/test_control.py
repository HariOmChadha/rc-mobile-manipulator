import pytest

from mobile_robot.control import Controller
from mobile_robot.hardware import MockArm
from mobile_robot.protocol import JOINTS, VERSION


@pytest.fixture
def rig():
    clock = [1.0]
    arm = MockArm()
    ctrl = Controller(arm, "secret", timeout=0.5, mock=True, clock=lambda: clock[0])
    return ctrl, arm, clock


def message(op, **fields):
    return {"version": VERSION, "token": "secret", "op": op, **fields}


def command(start, seq=0, value=20):
    return message(
        "action", session=start["session"], grant=start["grant"], seq=seq, action=dict.fromkeys(JOINTS, value)
    )


def test_feedback_and_clipped_target(rig):
    ctrl, arm, _ = rig
    start = ctrl.handle(message("start", mock=True))
    response = ctrl.handle(command(start))
    assert response["follower_joints"] == dict.fromkeys(JOINTS, 5)
    assert response["applied_action"] == arm.actions[-1]
    assert response["follower_joints"] != dict.fromkeys(JOINTS, 20)


def test_watchdog_latches_and_rejects_late_commands(rig):
    ctrl, arm, clock = rig
    start = ctrl.handle(message("start", mock=True))
    clock[0] += 0.5
    ctrl.tick()
    assert ctrl.reason == "command_timeout"
    holds = arm.holds
    ctrl.tick()
    assert arm.holds == holds
    with pytest.raises(ValueError, match="expired"):
        ctrl.handle(command(start))
    new = ctrl.handle(message("start", mock=True))
    assert new["session"] != start["session"]
    with pytest.raises(ValueError):
        ctrl.handle(command(start))
    ctrl.handle(command(new))


def test_replay_and_competing_client(rig):
    ctrl, arm, _ = rig
    start = ctrl.handle(message("start", mock=True))
    with pytest.raises(ValueError, match="Another"):
        ctrl.handle(message("start", mock=True))
    response = ctrl.handle(command(start))
    with pytest.raises(ValueError, match="Stale"):
        ctrl.handle(command(start))
    with pytest.raises(ValueError, match="Stale"):
        ctrl.handle(command(response, seq=0))
    ctrl.handle(command(response, seq=1))
    assert len(arm.actions) == 2


@pytest.mark.parametrize(
    "bad",
    [
        None,
        [],
        {},
        {"version": 99},
        message("start", token="bad", mock=True),
        message("start", mock=False),
        message("unknown"),
    ],
)
def test_invalid_requests_never_move(rig, bad):
    ctrl, arm, _ = rig
    with pytest.raises(ValueError):
        ctrl.handle(bad)
    assert arm.actions == []


def test_bad_action_does_not_renew_watchdog(rig):
    ctrl, arm, clock = rig
    start = ctrl.handle(message("start", mock=True))
    clock[0] += 0.4
    bad = command(start)
    bad["action"] = {}
    with pytest.raises(ValueError):
        ctrl.handle(bad)
    clock[0] += 0.11
    ctrl.tick()
    assert ctrl.session is None
    assert arm.actions == []


def test_stop_invalidates_even_when_hold_fails(rig):
    ctrl, arm, _ = rig
    start = ctrl.handle(message("start", mock=True))

    def broken():
        raise OSError("USB disconnected")

    arm.hold = broken
    with pytest.raises(OSError):
        ctrl.handle(message("stop", session=start["session"]))
    assert ctrl.session is None


def test_read_only_server_rejects_all_writes_and_never_holds():
    arm = MockArm()
    ctrl = Controller(arm, "secret", read_only=True)
    assert ctrl.handle(message("status"))["read_only"] is True
    for op in ("start", "action", "stop"):
        with pytest.raises(ValueError, match="Read-only"):
            ctrl.handle(message(op, mock=False))
    ctrl.stop("server_shutdown")
    assert arm.holds == 0
    assert arm.actions == []
