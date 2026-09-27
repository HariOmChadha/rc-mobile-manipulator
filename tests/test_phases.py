import io
import os
import pty
import select

from mobile_robot.phases import PHASES, EnterKey, Phases


def test_phase_boundaries_and_partial_saved_episode():
    phases = Phases()
    for index, name in enumerate(PHASES):
        for row in range(index * 3, index * 3 + 3):
            fields = phases.fields(row, 100 + row / 30, row / 30)
            assert fields["phase"] == name
            assert fields["phase_index"] == index + 1
        assert phases.advance() is (index < 3)
    assert [event["start_row"] for event in phases.saved_transitions(12)] == [0, 3, 6, 9]
    assert [event["phase"] for event in phases.saved_transitions(6)] == list(PHASES[:2])
    assert phases.saved_transitions(0) == []
    assert not phases.advance()
    assert Phases().index == 0


def test_nonterminal_input_does_not_block_or_consume():
    stream = io.StringIO("\n")
    keys = EnterKey(stream)
    keys.start()
    assert not keys.poll()
    keys.close()
    assert stream.read() == "\n"


def test_terminal_keys_flush_startup_and_review_input():
    master, slave = pty.openpty()
    with os.fdopen(slave, "r") as stream:
        keys = EnterKey(stream)
        try:

            def send(data):
                os.write(master, data)
                assert select.select([slave], [], [], 1)[0]

            send(b"\n")
            keys.start()
            assert not keys.poll()  # Startup key discarded.
            send(b"some text\n")
            assert not keys.poll()
            send(b"\n\n\n")
            assert keys.poll()
            assert not keys.poll()  # Burst cannot skip multiple phases.
            send(b"\n")
            keys.close()
            assert not select.select([slave], [], [], 0)[0]
            send(b"g\n")
            assert stream.readline() == "g\n"  # Review still owns fresh input.
        finally:
            keys.close()
            os.close(master)


def test_terminal_eof_disables_phase_input():
    master, slave = pty.openpty()
    with os.fdopen(slave, "r") as stream:
        keys = EnterKey(stream)
        try:
            keys.start()
            os.write(master, b"\x04")
            assert select.select([slave], [], [], 1)[0]
            assert not keys.poll()
            assert keys.fd is None
            assert not keys.poll()
        finally:
            keys.close()
            os.close(master)
