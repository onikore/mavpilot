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
