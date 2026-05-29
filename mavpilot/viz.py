"""Visualization server: HTTP + SSE, serves the bundled single-page 3D UI."""
import json
import logging
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources
from typing import Optional

logger = logging.getLogger("drone")


def _load_viz_html() -> bytes:
    """Read the bundled _viz.html via importlib.resources so it works in
    zipped wheels / PyInstaller installs where os.path.dirname is fragile."""
    return (resources.files("mavpilot") / "_viz.html").read_bytes()


def _sanitize_for_json(obj):
    """Recursively replace NaN/±Inf floats with None so json.dumps(allow_nan=False) succeeds.

    Browsers' JSON.parse rejects bare NaN tokens; without this, any NaN-tainted
    field would silently drop the whole event. Lists, tuples, and dicts are
    traversed.
    """
    import math as _math
    if isinstance(obj, float):
        if _math.isnan(obj) or _math.isinf(obj):
            return None
        return obj
    if isinstance(obj, dict):
        return {k: _sanitize_for_json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize_for_json(v) for v in obj]
    return obj


class VizServer:
    """Stdlib HTTP+SSE server for live browser visualization.

    Thread-safe: a background ThreadingHTTPServer plus per-client queues.
    Open http://localhost:<port> while the drone is running.
    """

    def __init__(self, port: int = 8765, host: str = "127.0.0.1"):
        self.port = port
        self.host = host
        self._clients_lock = threading.Lock()
        self._clients: list[queue.Queue] = []
        self._server: Optional[ThreadingHTTPServer] = None
        self._server_thread: Optional[threading.Thread] = None

    def start(self):
        viz_ref = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "mavpilot-viz/1.0"

            def do_GET(self):  # noqa: N802
                if self.path in ("/", "/index.html"):
                    self._serve_html()
                elif self.path == "/events":
                    self._serve_sse()
                else:
                    self.send_response(404)
                    self.end_headers()

            def _serve_html(self):
                try:
                    data = _load_viz_html()
                except Exception:
                    self.send_response(500)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)

            def _serve_sse(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                try:
                    self.wfile.write(b": connected\n\n")
                    self.wfile.flush()
                except Exception:
                    return

                q: queue.Queue = queue.Queue(maxsize=200)
                with viz_ref._clients_lock:
                    viz_ref._clients.append(q)
                try:
                    while True:
                        try:
                            data = q.get(timeout=15)
                        except queue.Empty:
                            self.wfile.write(b": ping\n\n")
                            self.wfile.flush()
                            continue
                        self.wfile.write(b"data: ")
                        self.wfile.write(data.encode("utf-8"))
                        self.wfile.write(b"\n\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
                except Exception as e:
                    logger.debug(f"SSE client error: {e}")
                finally:
                    with viz_ref._clients_lock:
                        if q in viz_ref._clients:
                            viz_ref._clients.remove(q)

            def log_message(self, format, *args):  # noqa: A002
                pass

        self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
            name=f"viz-http-{self.port}",
        )
        self._server_thread.start()
        logger.info(f"Visualization: http://localhost:{self.port}")

    def stop(self):
        if self._server is not None:
            try:
                self._server.shutdown()
                self._server.server_close()
            except Exception:
                pass
        with self._clients_lock:
            self._clients.clear()

    def publish(self, event: dict):
        try:
            sanitized = _sanitize_for_json(event)
            data = json.dumps(sanitized, ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError) as e:
            logger.debug(f"viz publish: dropping unencodable event: {e}")
            return
        with self._clients_lock:
            for q in list(self._clients):
                try:
                    q.put_nowait(data)
                except queue.Full:
                    try:
                        q.get_nowait()
                        q.put_nowait(data)
                    except (queue.Empty, queue.Full):
                        pass
