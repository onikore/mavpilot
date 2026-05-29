"""A blocking recv must not stall sends.

The receiver thread is the sole reader and reads lock-free, so an in-progress
(blocking) ``recv`` must never gate a concurrent ``send``. These tests fail
under the old design where ``recv`` held the send lock across the blocking read.
"""

from __future__ import annotations

import threading
import time

from mavpilot.core.connection import MAVLinkConnection


class _Builder:
    """Records mav.<method>_send(...) calls onto the parent's `sent` list."""

    def __init__(self, parent: GatedMav) -> None:
        self._parent = parent

    def __getattr__(self, name: str):
        def _record(*args, **kwargs):
            self._parent.sent.append((name, args, kwargs))

        return _record


class GatedMav:
    """Fake mavfile whose recv_match blocks until the test releases it."""

    def __init__(self) -> None:
        self.mav = _Builder(self)
        self.sent: list[tuple] = []
        self.in_recv = threading.Event()
        self.release = threading.Event()

    def recv_match(self, blocking: bool = True, timeout: float = 0.05, **_):
        self.in_recv.set()
        # Block until the test releases us (bounded so a failure can't hang CI).
        self.release.wait(timeout=5.0)
        return None


def _make_conn(fake) -> MAVLinkConnection:
    conn = MAVLinkConnection("udp:127.0.0.1:14540")
    conn.mav = fake  # type: ignore[assignment]
    conn.target_system = 1
    conn.target_component = 1
    return conn


def test_send_not_blocked_by_in_progress_recv():
    fake = GatedMav()
    conn = _make_conn(fake)

    reader = threading.Thread(target=lambda: conn.recv(blocking=True, timeout=0.05), daemon=True)
    reader.start()
    assert fake.in_recv.wait(timeout=2.0), "recv never started"

    # A send issued while recv is blocked must complete promptly.
    t0 = time.monotonic()
    conn.send("heartbeat_send", 1, 2, 0, 0, 0)
    elapsed = time.monotonic() - t0

    assert elapsed < 0.5, f"send was gated by the blocking recv ({elapsed:.2f}s)"
    assert fake.sent and fake.sent[-1][0] == "heartbeat_send"

    fake.release.set()
    reader.join(timeout=2.0)


class _QueueMav:
    """Fake mavfile that returns queued messages FIFO, then None."""

    def __init__(self) -> None:
        self.mav = _Builder(self)  # type: ignore[arg-type]
        self.sent: list[tuple] = []
        self._lock = threading.Lock()
        self._q: list = []

    def inject(self, msg) -> None:
        with self._lock:
            self._q.append(msg)

    def recv_match(self, blocking: bool = True, timeout: float = 0.05, **_):
        with self._lock:
            if self._q:
                return self._q.pop(0)
        time.sleep(0.005)
        return None


class _Msg:
    def __init__(self, name: str) -> None:
        self._name = name

    def get_type(self) -> str:
        return self._name


def test_receiver_delivers_messages():
    fake = _QueueMav()
    conn = _make_conn(fake)

    received: list = []
    got = threading.Event()

    def handler(msg) -> None:
        received.append(msg)
        got.set()

    conn.start_receiver(handler)
    try:
        fake.inject(_Msg("HEARTBEAT"))
        assert got.wait(timeout=2.0), "receiver did not deliver the injected message"
        assert received[0].get_type() == "HEARTBEAT"
    finally:
        conn.close()
