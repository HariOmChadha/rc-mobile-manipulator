"""Single-owner arm controller and bounded request/reply transport."""

import hmac
import time
import uuid

from .protocol import VERSION, validate_action


class Controller:
    def __init__(self, arm, token, *, timeout=0.5, mock=False, clock=time.monotonic, read_only=False):
        self.arm, self.token, self.timeout, self.mock, self.clock = arm, token, timeout, mock, clock
        self.read_only = read_only
        self.session = None
        self.grant = None
        self.deadline = 0
        self.seq = -1
        self.reason = "idle"

    def tick(self):
        if self.session is not None and self.clock() >= self.deadline:
            self.stop("command_timeout")

    def stop(self, reason):
        # Invalidate before hardware I/O, including when the hold itself fails.
        self.session = self.grant = None
        self.reason = reason
        if not self.read_only:
            self.arm.hold()

    def handle(self, message):
        self.tick()
        if not isinstance(message, dict) or message.get("version") != VERSION:
            raise ValueError("Protocol version mismatch")
        token = message.get("token")
        if not isinstance(token, str) or not hmac.compare_digest(token, self.token):
            raise ValueError("Authentication failed")
        op = message.get("op")
        if self.read_only and op != "status":
            raise ValueError("Read-only server accepts status requests only")
        if op == "status":
            pass
        elif op == "start":
            if message.get("mock") is not self.mock:
                raise ValueError("Client/server simulation modes differ")
            if self.session is not None:
                raise ValueError("Another control session is active")
            self.arm.hold()
            self.session = uuid.uuid4().hex
            self.grant = uuid.uuid4().hex
            self.seq = -1
            self.deadline = self.clock() + self.timeout
            self.reason = "active"
        elif op in ("action", "stop"):
            if self.session is None or message.get("session") != self.session:
                raise ValueError("Session expired or invalid; restart the client explicitly")
            if op == "stop":
                self.stop("client_stop")
            else:
                seq = message.get("seq")
                if type(seq) is not int or seq <= self.seq or message.get("grant") != self.grant:
                    raise ValueError("Stale or out-of-order command")
                target = validate_action(message.get("action"))
                applied = self.arm.send(target)
                self.seq = seq
                self.grant = uuid.uuid4().hex
                self.deadline = self.clock() + self.timeout
                result = self.state()
                result["applied_action"] = applied
                return result
        else:
            raise ValueError("Unknown operation")
        return self.state()

    def state(self):
        position = self.arm.observe()
        return {
            "version": VERSION,
            "ok": True,
            "mock": self.mock,
            "read_only": self.read_only,
            "session": self.session,
            "grant": self.grant,
            "seq": self.seq,
            "reason": self.reason,
            "follower_joints": position,
            "state_timestamp": time.time(),
            "server_monotonic": self.clock(),
            "command_timeout_s": self.timeout,
            "units": "normalized_minus100_100_gripper_0_100",
        }


class Remote:
    """A timed-out request closes the socket. Never replay motion after reconnect."""

    def __init__(self, endpoint, token, timeout=0.3):
        import zmq

        self.zmq = zmq
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.setsockopt(zmq.RCVHWM, 1)
        self.socket.setsockopt(zmq.IMMEDIATE, 1)
        self.socket.setsockopt(zmq.SNDTIMEO, max(1, int(timeout * 1000)))
        self.socket.setsockopt(zmq.RCVTIMEO, max(1, int(timeout * 1000)))
        self.socket.connect(endpoint)
        self.token = token
        self.session = self.grant = None
        self.seq = 0

    def call(self, op, **fields):
        if self.socket is None:
            raise RuntimeError("Connection closed; restart the client")
        start = time.monotonic()
        try:
            self.socket.send_json({"version": VERSION, "token": self.token, "op": op, **fields})
            response = self.socket.recv_json()
        except (self.zmq.ZMQError, ValueError) as error:
            self.close()
            raise TimeoutError("Pi request failed; motion will stop at the Pi watchdog timeout") from error
        if response.get("version") != VERSION or not response.get("ok"):
            raise RuntimeError(response.get("error", "Invalid Pi response"))
        response["round_trip_s"] = time.monotonic() - start
        response["received_timestamp"] = time.time()
        return response

    def start(self, mock):
        response = self.call("start", mock=mock)
        self.session, self.grant = response["session"], response["grant"]
        return response

    def action(self, action):
        response = self.call("action", session=self.session, grant=self.grant, seq=self.seq, action=action)
        self.seq += 1
        self.grant = response["grant"]
        return response

    def stop(self):
        if self.socket is not None and self.session is not None:
            self.call("stop", session=self.session)
            self.session = None

    def close(self):
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None
        self.context.term()
