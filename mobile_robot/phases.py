"""Ordered task labels and nonblocking Enter-key input for recording."""

import os
import select
import sys
import termios


PHASES = ("drive to object", "pick up object", "drive to the bin", "drop in bin")


class Phases:
    def __init__(self):
        self.index = 0
        self.transitions = []

    @property
    def name(self):
        return PHASES[self.index]

    def advance(self):
        if self.index == len(PHASES) - 1:
            return False
        self.index += 1
        return True

    def fields(self, row_index, timestamp, elapsed_s):
        if not self.transitions or self.transitions[-1]["phase_index"] != self.index + 1:
            self.transitions.append(
                {
                    "phase": self.name,
                    "phase_index": self.index + 1,
                    "start_row": row_index,
                    "timestamp": timestamp,
                    "elapsed_s": elapsed_s,
                }
            )
        return {"phase": self.name, "phase_index": self.index + 1, "episode_elapsed_s": elapsed_s}

    def saved_transitions(self, rows):
        # A failed/asynchronous write must not claim a transition with no saved row.
        return [event for event in self.transitions if event["start_row"] < rows]


class EnterKey:
    """Poll a canonical terminal without a reader thread competing with review input."""

    def __init__(self, stream=None):
        self.stream = sys.stdin if stream is None else stream
        self.fd = None
        self.buffer = b""

    def start(self):
        if self.stream.isatty():
            self.fd = self.stream.fileno()
            # Keys pressed while USB/cameras were initializing are not phase changes.
            termios.tcflush(self.fd, termios.TCIFLUSH)

    def poll(self):
        if self.fd is None or not select.select([self.fd], [], [], 0)[0]:
            return False
        chunk = os.read(self.fd, 4096)
        if not chunk:  # Ctrl+D / closed input: continue recording with the current phase.
            self.fd = None
            return False
        self.buffer += chunk
        lines = self.buffer.split(b"\n")
        self.buffer = lines.pop()
        # Only a blank submitted line advances; typing text does not label a phase.
        # Collapse a burst of Enters into one transition per control sample.
        advance = any(not line.strip() for line in lines)
        if advance:
            termios.tcflush(self.fd, termios.TCIFLUSH)
            self.buffer = b""
        return advance

    def close(self):
        if self.fd is not None:
            # Discard leftover recording keys before the separate good/bad prompt.
            termios.tcflush(self.fd, termios.TCIFLUSH)
            self.fd = None
        self.buffer = b""
