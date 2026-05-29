"""Visualization server: HTTP + SSE, serves the bundled 3D UI (ES modules)."""
import json
import logging
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources
from typing import Optional

logger = logging.getLogger("drone")

# Static UI files served from mavpilot/viz/static/, loaded via
# importlib.resources so it works inside zipped wheels / PyInstaller.
_STATIC_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/main.js": ("main.js", "application/javascript; charset=utf-8"),
    "/sse.js": ("sse.js", "application/javascript; charset=utf-8"),
    "/scene.js": ("scene.js", "application/javascript; charset=utf-8"),
    "/telemetry.js": ("telemetry.js", "application/javascript; charset=utf-8"),
    "/log.js": ("log.js", "application/javascript; charset=utf-8"),
    "/styles.css": ("styles.css", "text/css; charset=utf-8"),
}


def _load_static(filename: str) -> bytes:
    return (resources.files("mavpilot.viz") / "static" / filename).read_bytes()


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

    def __init__(self, port: int = 8765, host: str = "127.0.0.1", max_clients: int = 32):
        self.port = port
        self.host = host
        self.max_clients = max_clients
        self._clients_lock = threading.Lock()
        self._clients: list[queue.Queue] = []
        self._server: Optional[ThreadingHTTPServer] = None
        self._server_thread: Optional[threading.Thread] = None

    def start(self):
        viz_ref = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "mavpilot-viz/1.0"

            def do_GET(self):  # noqa: N802
                if self.path in _STATIC_FILES:
                    self._serve_static(self.path)
                elif self.path == "/events":
                    self._serve_sse()
                else:
                    self.send_response(404)
                    self.end_headers()

            def _serve_static(self, path):
                filename, content_type = _STATIC_FILES[path]
                try:
                    data = _load_static(filename)
                except Exception:
                    self.send_response(500)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)

            def _serve_sse(self):
                # Cap concurrent SSE clients to avoid unbounded thread spawn.
                with viz_ref._clients_lock:
                    if len(viz_ref._clients) >= viz_ref.max_clients:
                        self.send_response(503)
                        self.send_header("Content-Type", "text/plain")
                        self.end_headers()
                        try:
                            self.wfile.write(b"viz client cap reached\n")
                        except Exception:
                            pass
                        return

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
                        if data is None:
                            # Shutdown sentinel pushed by stop().
                            break
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
        # Push a sentinel so SSE workers exit promptly instead of waiting up
        # to 15 s for their q.get timeout to expire.
        with self._clients_lock:
            for q in list(self._clients):
                try:
                    q.put_nowait(None)
                except Exception:
                    pass
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
