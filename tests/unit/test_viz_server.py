"""Unit tests for VizServer publish/JSON encoding behaviour.

These tests do NOT bind a real port. They poke `publish()` directly and
inspect the per-client queue.
"""
import json
import math
import queue

from mavpilot.viz import VizServer


def _drain(q: queue.Queue) -> list[str]:
    out: list[str] = []
    while True:
        try:
            out.append(q.get_nowait())
        except queue.Empty:
            break
    return out


def test_publish_nan_becomes_null():
    v = VizServer(port=0)
    sink: queue.Queue = queue.Queue(maxsize=10)
    with v._clients_lock:
        v._clients.append(sink)

    v.publish({"type": "telemetry", "yaw": float("nan"), "x": 1.0})
    msgs = _drain(sink)

    assert len(msgs) == 1, f"expected one published msg, got {msgs}"
    parsed = json.loads(msgs[0])
    assert parsed["yaw"] is None
    assert parsed["x"] == 1.0


def test_publish_inf_becomes_null():
    v = VizServer(port=0)
    sink: queue.Queue = queue.Queue(maxsize=10)
    with v._clients_lock:
        v._clients.append(sink)

    v.publish({"type": "telemetry", "vx": math.inf, "vy": -math.inf})
    msgs = _drain(sink)
    parsed = json.loads(msgs[0])
    assert parsed["vx"] is None
    assert parsed["vy"] is None


def test_publish_nested_dict_nan_sanitized():
    v = VizServer(port=0)
    sink: queue.Queue = queue.Queue(maxsize=10)
    with v._clients_lock:
        v._clients.append(sink)

    v.publish({"type": "telemetry", "setpoint": {"yaw": float("nan"), "x": 2.0}})
    parsed = json.loads(_drain(sink)[0])
    assert parsed["setpoint"]["yaw"] is None
    assert parsed["setpoint"]["x"] == 2.0


def test_publish_normal_event_unchanged():
    v = VizServer(port=0)
    sink: queue.Queue = queue.Queue(maxsize=10)
    with v._clients_lock:
        v._clients.append(sink)

    payload = {"type": "command", "command": "takeoff", "altitude_m": 2.0}
    v.publish(payload)
    parsed = json.loads(_drain(sink)[0])
    assert parsed == payload


def test_viz_server_default_host_is_localhost():
    v = VizServer(port=0)
    assert v.host == "127.0.0.1"


def test_viz_server_explicit_host_passed_through():
    v = VizServer(host="0.0.0.0", port=0)
    assert v.host == "0.0.0.0"


import socket
import threading
import time
import urllib.error
import urllib.request


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_max_clients_cap_returns_503():
    port = _free_port()
    v = VizServer(port=port, max_clients=2)
    v.start()
    try:
        # Open 2 SSE streams concurrently — should succeed.
        threads = []
        responses = []

        def open_sse():
            try:
                r = urllib.request.urlopen(f"http://127.0.0.1:{port}/events", timeout=1.0)
                responses.append(r)
                r.read(20)  # read a chunk
            except Exception as e:
                responses.append(e)
        for _ in range(2):
            t = threading.Thread(target=open_sse, daemon=True)
            t.start()
            threads.append(t)
        time.sleep(0.3)  # let them register
        # Third should get 503.
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/events", timeout=1.0).read(1)
            third_status = 200
        except urllib.error.HTTPError as e:
            third_status = e.code
        assert third_status == 503
    finally:
        v.stop()


def test_stop_unblocks_sse_workers_quickly():
    """stop() must signal SSE worker threads via a None sentinel; total
    shutdown time should be << 15 s (the q.get timeout)."""
    port = _free_port()
    v = VizServer(port=port)
    v.start()
    try:
        def open_sse():
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{port}/events", timeout=1.0).read(20)
            except Exception:
                pass
        t = threading.Thread(target=open_sse, daemon=True)
        t.start()
        time.sleep(0.3)  # let it register
        stop_start = time.monotonic()
        v.stop()
        elapsed = time.monotonic() - stop_start
        assert elapsed < 2.0, f"stop() took {elapsed:.2f}s; expected <2s"
    finally:
        try:
            v.stop()
        except Exception:
            pass
