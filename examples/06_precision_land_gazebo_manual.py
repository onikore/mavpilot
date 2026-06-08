"""Точная посадка: ручной полёт → OFFBOARD → precision landing (Gazebo).

Сценарий:
  1. Запустить скрипт (откроется окно настроек)
  2. Выбрать контроллер (P / PID / FO-PID / ADRC) и настроить параметры
  3. Нажать «Старт»; лететь к площадке вручную
  4. Переключить FC в OFFBOARD → скрипт берёт управление:
       • слепое снижение до approach-alt (высота надёжной детекции)
       • центровка + снижение на маркер (precision_land)

Вкладка «Графики» показывает в реальном времени:
  • горизонтальную ошибку и управляющий сигнал выбранного контроллера
  • высоту дрона на протяжении всей миссии

Запуск без GUI (headless):
    python examples/06_precision_land_gazebo_manual.py --no-gui \\
        --controller adrc --b0 -2.5 --omega-obs 3.0

Просмотр детекции:
    ros2 run rqt_image_view rqt_image_view /mavpilot/detection_image
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import math
import queue
import threading
import time
from typing import Any

from mavpilot import DroneController
from mavpilot.core.controllers import (
    ADRCController,
    FOPIDController,
    LateralController,
    PController,
    PIDController,
)
from mavpilot.integrations.gazebo import (
    DEFAULT_CAMERA_INFO_TOPIC,
    DEFAULT_IMAGE_TOPIC,
    GazeboFractalSource,
)

try:
    import tkinter as tk
    from tkinter import ttk
    _HAS_TK = True
except ImportError:
    _HAS_TK = False

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("drone")

# ---------------------------------------------------------------------------
# Controller factory
# ---------------------------------------------------------------------------

def _make_controller(params: dict) -> LateralController:
    t = params["controller"]
    if t == "P":
        return PController(kp=params["kp"])
    if t == "PID":
        return PIDController(
            kp=params["kp"], ki=params["ki"], kd=params["kd"],
            windup_limit=params.get("windup_limit", 2.0),
            derivative_alpha=params.get("derivative_alpha", 0.5),
        )
    if t == "FO-PID":
        return FOPIDController(
            kp=params["kp"], ki=params["ki"], kd=params["kd"],
            lambda_order=params.get("lambda_order", 0.8),
            mu_order=params.get("mu_order", 0.9),
            N=int(params.get("N", 20)),
        )
    if t == "ADRC":
        return ADRCController(
            b0=params["b0"],
            omega_obs=params["omega_obs"],
            omega_ctrl=params["omega_ctrl"],
        )
    raise ValueError(f"Unknown controller: {t}")


# ---------------------------------------------------------------------------
# Telemetry-recording controller wrapper
# ---------------------------------------------------------------------------

class _TrackedController(LateralController):
    """Wraps any LateralController; sends (t, err, u) telemetry into status_q."""

    _INTERVAL = 0.1  # seconds between telem messages (10 Hz)

    def __init__(self, inner: LateralController, q: queue.Queue) -> None:
        self._inner = inner
        self._q = q
        self._t0: float | None = None
        self._last = 0.0

    def update(self, err_x: float, err_y: float, dt: float) -> tuple[float, float]:
        u_x, u_y = self._inner.update(err_x, err_y, dt)
        now = time.time()
        if self._t0 is None:
            self._t0 = now
        if now - self._last >= self._INTERVAL:
            self._last = now
            self._q.put({"telem": {
                "t": now,
                "err": math.hypot(err_x, err_y),
                "u": math.hypot(u_x, u_y),
            }})
        return u_x, u_y

    def reset(self) -> None:
        self._inner.reset()
        self._t0 = None
        self._last = 0.0


# ---------------------------------------------------------------------------
# Async mission
# ---------------------------------------------------------------------------

async def run_mission(params: dict, status_q: queue.Queue) -> None:
    """Core flight logic. Sends status / telemetry dicts to status_q."""

    def put(text: str, **kw: Any) -> None:
        status_q.put({"text": text, **kw})

    _last_marker = [0.0]

    def wrap_marker(cb):
        def wrapped():
            obs = cb()
            now = time.time()
            if now - _last_marker[0] > 0.2:
                _last_marker[0] = now
                status_q.put({"marker_err": math.hypot(obs.dx, obs.dy) if obs else None})
            return obs
        return wrapped

    source = GazeboFractalSource(
        image_topic=params["image_topic"],
        camera_info_topic=params["camera_info_topic"],
        marker_size=params["marker_size"],
        camera_yaw_deg=params["camera_yaw"],
    )
    drone = DroneController(connection_string=params["connection"])

    try:
        async with source as src, drone:
            put("Подключение…", phase="connect")
            await drone.connect(timeout_s=30.0)
            await drone.wait_until_ready(timeout_s=60.0)
            put("Готов. Переключите в OFFBOARD.", phase="wait_offboard")

            await drone.wait_for_offboard()
            put("OFFBOARD активен.", phase="offboard")

            if params.get("landing_yaw") is not None:
                put(f"Разворот на {params['landing_yaw']}°", phase="yaw")
                await drone.set_yaw(params["landing_yaw"], timeout_s=30.0)

            async def _alt_monitor() -> None:
                while True:
                    pos = drone.get_local_position()
                    status_q.put({"alt_telem": {"t": time.time(), "alt": -pos.z}})
                    await asyncio.sleep(0.5)

            alt_task = asyncio.create_task(_alt_monitor())
            try:
                pos = drone.get_local_position()
                current_alt = -pos.z
                approach_alt = params["approach_alt"]
                if current_alt > approach_alt + 0.1:
                    put(f"Слепое снижение {current_alt:.1f} → {approach_alt:.1f} м",
                        phase="approach", alt=current_alt)
                    await drone.goto(x=pos.x, y=pos.y, z=-approach_alt, timeout_s=60.0)
                else:
                    put(f"Уже на {current_alt:.1f} м — спуск пропущен", phase="approach")

                put(f"Точная посадка [{params['controller']}]…", phase="precision_land")
                ctrl = _TrackedController(_make_controller(params), status_q)
                result = await drone.precision_land(
                    get_marker_offset=wrap_marker(src.marker_callback),
                    descent_rate_mps=params["descent_rate"],
                    final_altitude_m=params["land_distance"],
                    horizontal_tolerance_m=params["h_tolerance"],
                    marker_lost_timeout_s=params["marker_timeout"],
                    lateral_controller=ctrl,
                    timeout_s=120.0,
                )
                put(f"Готово: {result.status.value}", phase="done")
                log.info(f"result: {result.status.value}  pos={result.final_position}")
                if not result:
                    log.warning("precision_land failed — fallback land")
                    await drone.land()
            finally:
                alt_task.cancel()
                await asyncio.gather(alt_task, return_exceptions=True)

    except asyncio.CancelledError:
        put("Остановлено оператором", phase="cancelled")
        raise
    except Exception as e:
        put(f"Ошибка: {e}", phase="error", error=str(e))
        log.exception("Mission error")
    finally:
        status_q.put({"_done": True})


# ---------------------------------------------------------------------------
# GUI helpers
# ---------------------------------------------------------------------------

class _SliderRow:
    """Label + value display + Scale in a grid frame."""

    def __init__(self, parent, row: int, label: str, from_: float, to: float,
                 default: float, fmt: str = ".2f") -> None:
        self.var = tk.DoubleVar(value=default)
        self._fmt = fmt
        tk.Label(parent, text=label, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(4, 2), pady=2)
        self._lbl = tk.Label(parent, width=8, anchor="e", font=("Courier", 9))
        self._lbl.grid(row=row, column=1, padx=2, pady=2)
        ttk.Scale(parent, from_=from_, to=to, variable=self.var,
                  orient="horizontal", length=180).grid(
            row=row, column=2, sticky="ew", padx=(2, 4), pady=2)
        parent.columnconfigure(2, weight=1)
        self.var.trace_add("write", lambda *_: self._refresh())
        self._refresh()

    def _refresh(self) -> None:
        self._lbl.config(text=format(self.var.get(), self._fmt))

    def get(self) -> float:
        return self.var.get()


class _EntryRow:
    """Label + Entry in a grid frame."""

    def __init__(self, parent, row: int, label: str, default: str, width: int = 28) -> None:
        self.var = tk.StringVar(value=str(default))
        tk.Label(parent, text=label, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(4, 2), pady=2)
        ttk.Entry(parent, textvariable=self.var, width=width).grid(
            row=row, column=1, columnspan=2, sticky="ew", padx=(2, 4), pady=2)
        parent.columnconfigure(2, weight=1)

    def get(self) -> str:
        return self.var.get()


# ---------------------------------------------------------------------------
# Main GUI window
# ---------------------------------------------------------------------------

class LandingGUI:
    def __init__(self, defaults: dict) -> None:
        self.root = tk.Tk()
        self.root.title("Точная посадка — настройки")
        self.root.resizable(True, True)

        self._status_q: queue.Queue = queue.Queue()
        self._running = False
        self._rows: dict[str, Any] = {}
        # For task cancellation
        self._loop: asyncio.AbstractEventLoop | None = None
        self._async_task: asyncio.Task | None = None  # type: ignore[type-arg]
        # Telemetry arrays
        self._telem_t0: float | None = None
        self._telem_t: list[float] = []
        self._telem_err: list[float] = []
        self._telem_u: list[float] = []
        self._alt_t: list[float] = []
        self._alt_v: list[float] = []

        self._build(defaults)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build(self, d: dict) -> None:
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=6, pady=6)

        cf = ttk.Frame(nb, padding=4)
        nb.add(cf, text="Контроллер")
        self._build_ctrl_tab(cf, d)

        ff = ttk.Frame(nb, padding=4)
        nb.add(ff, text="Полёт")
        self._build_flight_tab(ff, d)

        xf = ttk.Frame(nb, padding=4)
        nb.add(xf, text="Соединение")
        self._build_conn_tab(xf, d)

        gf = ttk.Frame(nb, padding=4)
        nb.add(gf, text="Графики")
        self._build_graph_tab(gf)

        # Status panel
        sp = ttk.LabelFrame(self.root, text="Статус", padding=4)
        sp.pack(fill="x", padx=6, pady=(0, 2))
        self._phase_var = tk.StringVar(value="Ожидание запуска")
        self._alt_var = tk.StringVar(value="Высота: —")
        self._marker_var = tk.StringVar(value="Маркер: —")
        ttk.Label(sp, textvariable=self._phase_var,
                  font=("", 10, "bold"), foreground="#1a6b1a").grid(
            row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(sp, textvariable=self._alt_var).grid(
            row=1, column=0, sticky="w", padx=(0, 16))
        ttk.Label(sp, textvariable=self._marker_var).grid(
            row=1, column=1, sticky="w")

        bf = ttk.Frame(self.root, padding=(6, 2, 6, 6))
        bf.pack(fill="x")
        self._stop_btn = ttk.Button(bf, text="■ Стоп", command=self._on_stop,
                                    state="disabled", width=10)
        self._stop_btn.pack(side="right")
        self._start_btn = ttk.Button(bf, text="▶ Старт", command=self._on_start, width=10)
        self._start_btn.pack(side="right", padx=4)

    def _build_ctrl_tab(self, parent, d: dict) -> None:
        self._ctrl_type = tk.StringVar(value=d.get("controller", "P"))
        rb = ttk.Frame(parent)
        rb.pack(fill="x", pady=(0, 4))
        for t in ("P", "PID", "FO-PID", "ADRC"):
            ttk.Radiobutton(rb, text=t, variable=self._ctrl_type,
                            value=t, command=self._on_ctrl_change).pack(
                side="left", padx=8)

        self._ctrl_container = ttk.Frame(parent)
        self._ctrl_container.pack(fill="both", expand=True)
        self._ctrl_frames: dict[str, ttk.LabelFrame] = {}

        # P
        pf = ttk.LabelFrame(self._ctrl_container, text="P-регулятор", padding=4)
        self._rows["p_kp"] = _SliderRow(pf, 0, "kp", 0.1, 3.0, d.get("kp", 0.70))
        self._ctrl_frames["P"] = pf

        # PID
        pidf = ttk.LabelFrame(self._ctrl_container, text="PID-регулятор", padding=4)
        self._rows["pid_kp"] = _SliderRow(pidf, 0, "kp", 0.05, 2.0, d.get("kp", 0.50))
        self._rows["pid_ki"] = _SliderRow(pidf, 1, "ki", 0.0, 0.5, d.get("ki", 0.05))
        self._rows["pid_kd"] = _SliderRow(pidf, 2, "kd", 0.0, 0.5, d.get("kd", 0.15))
        self._rows["pid_windup"] = _SliderRow(pidf, 3, "windup_limit", 0.5, 5.0, 2.0)
        self._rows["pid_alpha"] = _SliderRow(pidf, 4, "deriv. alpha", 0.0, 1.0, 0.5)
        self._ctrl_frames["PID"] = pidf

        # FO-PID
        fof = ttk.LabelFrame(self._ctrl_container, text="FO-PID  (PIλDμ)", padding=4)
        self._rows["fo_kp"] = _SliderRow(fof, 0, "kp", 0.05, 2.0, 0.50)
        self._rows["fo_ki"] = _SliderRow(fof, 1, "ki", 0.0, 0.5, 0.05)
        self._rows["fo_kd"] = _SliderRow(fof, 2, "kd", 0.0, 0.5, 0.15)
        self._rows["fo_lambda"] = _SliderRow(fof, 3, "λ (integral order)", 0.1, 1.0, 0.8)
        self._rows["fo_mu"] = _SliderRow(fof, 4, "μ (deriv. order)", 0.1, 1.0, 0.9)
        tk.Label(fof, text="N (memory window)", anchor="w").grid(
            row=5, column=0, sticky="w", padx=(4, 2), pady=2)
        self._fo_N = tk.IntVar(value=20)
        ttk.Spinbox(fof, from_=5, to=100, textvariable=self._fo_N, width=6).grid(
            row=5, column=1, sticky="w", padx=2, pady=2)
        self._ctrl_frames["FO-PID"] = fof

        # ADRC
        af = ttk.LabelFrame(self._ctrl_container, text="ADRC  (ESO 2-го порядка)", padding=4)
        tk.Label(af, text="b0 = −1/tau  (напр. tau=0.4 → b0=−2.5)",
                 foreground="#884400", font=("", 8)).grid(
            row=0, column=0, columnspan=3, sticky="w", padx=4, pady=(0, 2))
        self._rows["adrc_b0"] = _SliderRow(af, 1, "b0", -10.0, -0.05, -2.5)
        self._rows["adrc_omega_obs"] = _SliderRow(af, 2, "ω_obs (ESO)", 0.5, 10.0, 3.0)
        self._rows["adrc_omega_ctrl"] = _SliderRow(af, 3, "ω_ctrl (CL)", 0.1, 5.0, 1.5)
        self._ctrl_frames["ADRC"] = af

        self._on_ctrl_change()

    def _build_flight_tab(self, parent, d: dict) -> None:
        specs = [
            ("approach_alt",   "Подход (м)",             0.5, 8.0,  d.get("approach_alt", 2.5)),
            ("descent_rate",   "Скорость снижения (м/с)", 0.05, 1.0, d.get("descent_rate", 0.3)),
            ("h_tolerance",    "Допуск центровки (м)",    0.05, 0.5, d.get("h_tolerance", 0.15)),
            ("marker_timeout", "Таймаут маркера (с)",     1.0, 30.0, d.get("marker_timeout", 5.0)),
            ("land_distance",  "Высота AUTO_LAND (м)",    0.1, 2.0,  d.get("land_distance", 0.5)),
        ]
        for i, (name, label, lo, hi, default) in enumerate(specs):
            self._rows[name] = _SliderRow(parent, i, label, lo, hi, default)

        r = len(specs)
        self._use_yaw = tk.BooleanVar(value=d.get("landing_yaw") is not None)
        self._yaw_val = tk.DoubleVar(value=d.get("landing_yaw") or 0.0)
        ttk.Checkbutton(parent, text="Задать yaw при посадке",
                        variable=self._use_yaw,
                        command=self._on_yaw_toggle).grid(
            row=r, column=0, columnspan=3, sticky="w", padx=4, pady=(6, 0))
        tk.Label(parent, text="Yaw (°)", anchor="w").grid(
            row=r + 1, column=0, sticky="w", padx=(4, 2), pady=2)
        self._yaw_lbl = tk.Label(parent, width=8, anchor="e", font=("Courier", 9))
        self._yaw_lbl.grid(row=r + 1, column=1, padx=2)
        self._yaw_scale = ttk.Scale(parent, from_=-180, to=180, variable=self._yaw_val,
                                    orient="horizontal", length=180)
        self._yaw_scale.grid(row=r + 1, column=2, sticky="ew", padx=(2, 4))
        parent.columnconfigure(2, weight=1)
        self._yaw_val.trace_add(
            "write", lambda *_: self._yaw_lbl.config(text=f"{self._yaw_val.get():.1f}°"))
        self._yaw_lbl.config(text=f"{self._yaw_val.get():.1f}°")
        self._on_yaw_toggle()

    def _build_conn_tab(self, parent, d: dict) -> None:
        str_rows = [
            ("connection",        "MAVLink URL",       d.get("connection", "udp:127.0.0.1:14540")),
            ("image_topic",       "Image topic",       d.get("image_topic", DEFAULT_IMAGE_TOPIC)),
            ("camera_info_topic", "Camera info topic", d.get("camera_info_topic", DEFAULT_CAMERA_INFO_TOPIC)),
        ]
        for i, (name, label, default) in enumerate(str_rows):
            self._rows[name] = _EntryRow(parent, i, label, default)
        self._rows["marker_size"] = _SliderRow(
            parent, len(str_rows), "Маркер (м)", 0.05, 1.0,
            d.get("marker_size", 0.17), ".3f")
        self._rows["camera_yaw"] = _SliderRow(
            parent, len(str_rows) + 1, "Camera yaw (°)", -180, 180,
            d.get("camera_yaw", 0.0), ".1f")

    def _build_graph_tab(self, parent) -> None:
        if not _HAS_MPL:
            tk.Label(parent,
                     text="matplotlib не установлен\npip install matplotlib",
                     justify="center", foreground="gray").pack(expand=True)
            return

        fig = Figure(figsize=(6, 4), dpi=85)
        self._ax_err = fig.add_subplot(211)
        self._ax_alt = fig.add_subplot(212)
        fig.tight_layout(pad=2.5)
        self._mpl_fig = fig

        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True, padx=2, pady=2)
        self._mpl_canvas = canvas
        self._draw_empty_graph()

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_ctrl_change(self) -> None:
        sel = self._ctrl_type.get()
        for name, frame in self._ctrl_frames.items():
            if name == sel:
                frame.pack(fill="both", expand=True)
            else:
                frame.pack_forget()

    def _on_yaw_toggle(self) -> None:
        state = "normal" if self._use_yaw.get() else "disabled"
        self._yaw_scale.config(state=state)

    def _collect_params(self) -> dict:
        t = self._ctrl_type.get()
        p: dict = {
            "controller": t,
            "connection": self._rows["connection"].get(),
            "image_topic": self._rows["image_topic"].get(),
            "camera_info_topic": self._rows["camera_info_topic"].get(),
            "marker_size": self._rows["marker_size"].get(),
            "camera_yaw": self._rows["camera_yaw"].get(),
            "approach_alt": self._rows["approach_alt"].get(),
            "descent_rate": self._rows["descent_rate"].get(),
            "h_tolerance": self._rows["h_tolerance"].get(),
            "marker_timeout": self._rows["marker_timeout"].get(),
            "land_distance": self._rows["land_distance"].get(),
            "landing_yaw": self._yaw_val.get() if self._use_yaw.get() else None,
        }
        if t == "P":
            p.update(kp=self._rows["p_kp"].get())
        elif t == "PID":
            p.update(kp=self._rows["pid_kp"].get(), ki=self._rows["pid_ki"].get(),
                     kd=self._rows["pid_kd"].get(),
                     windup_limit=self._rows["pid_windup"].get(),
                     derivative_alpha=self._rows["pid_alpha"].get())
        elif t == "FO-PID":
            p.update(kp=self._rows["fo_kp"].get(), ki=self._rows["fo_ki"].get(),
                     kd=self._rows["fo_kd"].get(),
                     lambda_order=self._rows["fo_lambda"].get(),
                     mu_order=self._rows["fo_mu"].get(),
                     N=self._fo_N.get())
        elif t == "ADRC":
            p.update(b0=self._rows["adrc_b0"].get(),
                     omega_obs=self._rows["adrc_omega_obs"].get(),
                     omega_ctrl=self._rows["adrc_omega_ctrl"].get())
        return p

    def _on_start(self) -> None:
        params = self._collect_params()
        self._status_q = queue.Queue()
        self._running = True
        self._loop = None
        self._async_task = None
        # Reset telemetry
        self._telem_t0 = None
        self._telem_t.clear()
        self._telem_err.clear()
        self._telem_u.clear()
        self._alt_t.clear()
        self._alt_v.clear()
        if _HAS_MPL:
            self._draw_empty_graph()

        self._start_btn.config(state="disabled")
        self._stop_btn.config(state="normal")
        self._phase_var.set("Запуск миссии…")

        def _thread() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._loop = loop

            async def _wrapper() -> None:
                self._async_task = asyncio.current_task()
                try:
                    await run_mission(params, self._status_q)
                except asyncio.CancelledError:
                    pass  # run_mission already queued "Остановлено" + _done

            try:
                loop.run_until_complete(_wrapper())
            finally:
                loop.close()
                self._loop = None
                self._async_task = None

        threading.Thread(target=_thread, daemon=True).start()
        self.root.after(200, self._poll)
        if _HAS_MPL:
            self.root.after(600, self._refresh_graph)

    def _on_stop(self) -> None:
        loop = self._loop
        task = self._async_task
        if loop is not None and task is not None:
            loop.call_soon_threadsafe(task.cancel)
        else:
            # Mission hasn't started the async loop yet — mark done directly
            self._running = False
            self._start_btn.config(state="normal")
            self._stop_btn.config(state="disabled")
        self._phase_var.set("Остановка…")
        self._stop_btn.config(state="disabled")

    def _poll(self) -> None:
        try:
            while True:
                msg = self._status_q.get_nowait()
                if msg.get("_done"):
                    self._running = False
                    self._start_btn.config(state="normal")
                    self._stop_btn.config(state="disabled")
                    if _HAS_MPL:
                        self._refresh_graph()  # final draw
                    break
                if "text" in msg:
                    self._phase_var.set(msg["text"])
                if "alt" in msg:
                    self._alt_var.set(f"Высота: {msg['alt']:.1f} м")
                if "marker_err" in msg:
                    err = msg["marker_err"]
                    self._marker_var.set(
                        "Маркер: нет" if err is None
                        else f"Маркер: ✓  ош. {err:.2f} м")
                if "telem" in msg:
                    tm = msg["telem"]
                    if self._telem_t0 is None:
                        self._telem_t0 = tm["t"]
                    self._telem_t.append(tm["t"] - self._telem_t0)
                    self._telem_err.append(tm["err"])
                    self._telem_u.append(tm["u"])
                if "alt_telem" in msg:
                    tm = msg["alt_telem"]
                    if self._telem_t0 is None:
                        self._telem_t0 = tm["t"]
                    self._alt_t.append(tm["t"] - self._telem_t0)
                    self._alt_v.append(tm["alt"])
                    self._alt_var.set(f"Высота: {tm['alt']:.1f} м")
        except queue.Empty:
            pass
        if self._running:
            self.root.after(200, self._poll)

    # ------------------------------------------------------------------
    # Graph helpers
    # ------------------------------------------------------------------

    def _draw_empty_graph(self) -> None:
        if not _HAS_MPL:
            return
        for ax, ylabel in [(self._ax_err, "Ошибка / сигнал (м)"),
                           (self._ax_alt, "Высота (м)")]:
            ax.clear()
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
        self._ax_err.set_title("Данные появятся после Старта", color="gray", fontsize=9)
        self._ax_alt.set_xlabel("Время (с)")
        self._mpl_fig.tight_layout(pad=2.5)
        self._mpl_canvas.draw()

    def _refresh_graph(self) -> None:
        if not _HAS_MPL:
            return
        ctrl_name = self._ctrl_type.get()

        self._ax_err.clear()
        if self._telem_t:
            self._ax_err.plot(self._telem_t, self._telem_err,
                              color="#1f77b4", linewidth=1.2, label="Ошибка (м)")
            self._ax_err.plot(self._telem_t, self._telem_u,
                              color="#d62728", linewidth=1.0, linestyle="--",
                              label="Упр. сигнал (м/шаг)")
            self._ax_err.legend(fontsize=8, loc="upper right")
        self._ax_err.set_title(f"Контроллер: {ctrl_name}", fontsize=9)
        self._ax_err.set_ylabel("(м)")
        self._ax_err.grid(True, alpha=0.3)

        self._ax_alt.clear()
        if self._alt_t:
            self._ax_alt.plot(self._alt_t, self._alt_v,
                              color="#2ca02c", linewidth=1.2)
        self._ax_alt.set_ylabel("Высота (м)")
        self._ax_alt.set_xlabel("Время (с)")
        self._ax_alt.grid(True, alpha=0.3)

        self._mpl_fig.tight_layout(pad=2.5)
        self._mpl_canvas.draw()

        if self._running:
            self.root.after(500, self._refresh_graph)

    # ------------------------------------------------------------------

    def _on_close(self) -> None:
        loop = self._loop
        task = self._async_task
        if loop is not None and task is not None:
            loop.call_soon_threadsafe(task.cancel)
        self._running = False
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


# ---------------------------------------------------------------------------
# Headless mode
# ---------------------------------------------------------------------------

async def run_headless(args) -> None:
    ctrl_map = {"p": "P", "pid": "PID", "fo-pid": "FO-PID", "adrc": "ADRC"}
    params = {
        "controller": ctrl_map[args.controller.lower()],
        "connection": args.connection,
        "image_topic": args.image_topic,
        "camera_info_topic": args.camera_info_topic,
        "marker_size": args.marker_size,
        "camera_yaw": args.camera_yaw,
        "landing_yaw": args.landing_yaw,
        "approach_alt": args.approach_alt,
        "descent_rate": args.descent_rate,
        "land_distance": args.land_distance,
        "h_tolerance": args.h_tolerance,
        "marker_timeout": args.marker_timeout,
        "kp": args.kp, "ki": args.ki, "kd": args.kd,
        "lambda_order": args.lambda_order, "mu_order": args.mu_order,
        "b0": args.b0, "omega_obs": args.omega_obs, "omega_ctrl": args.omega_ctrl,
    }
    q: queue.Queue = queue.Queue()
    await run_mission(params, q)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    p = argparse.ArgumentParser(
        description="Manual fly → OFFBOARD → precision land (Gazebo)")
    p.add_argument("--no-gui", action="store_true")
    p.add_argument("--connection", default="udp:127.0.0.1:14540")
    p.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    p.add_argument("--camera-info-topic", default=DEFAULT_CAMERA_INFO_TOPIC)
    p.add_argument("--marker-size", type=float, default=0.17)
    p.add_argument("--camera-yaw", type=float, default=0.0)
    p.add_argument("--landing-yaw", type=float, default=None)
    p.add_argument("--approach-alt", type=float, default=2.5)
    p.add_argument("--descent-rate", type=float, default=0.3)
    p.add_argument("--land-distance", type=float, default=0.5)
    p.add_argument("--h-tolerance", type=float, default=0.15)
    p.add_argument("--marker-timeout", type=float, default=5.0)
    p.add_argument("--controller", default="p",
                   choices=["p", "pid", "fo-pid", "adrc"])
    p.add_argument("--kp", type=float, default=0.7)
    p.add_argument("--ki", type=float, default=0.05)
    p.add_argument("--kd", type=float, default=0.15)
    p.add_argument("--lambda-order", type=float, default=0.8, dest="lambda_order")
    p.add_argument("--mu-order", type=float, default=0.9, dest="mu_order")
    p.add_argument("--b0", type=float, default=-2.5,
                   help="ADRC b0 = −1/tau (must be negative)")
    p.add_argument("--omega-obs", type=float, default=3.0, dest="omega_obs")
    p.add_argument("--omega-ctrl", type=float, default=1.5, dest="omega_ctrl")
    args = p.parse_args()

    if args.no_gui or not _HAS_TK:
        if not _HAS_TK and not args.no_gui:
            log.warning("tkinter недоступен — запуск в headless-режиме")
        try:
            asyncio.run(run_headless(args))
        except KeyboardInterrupt:
            log.info("stopped")
        return

    ctrl_map = {"p": "P", "pid": "PID", "fo-pid": "FO-PID", "adrc": "ADRC"}
    defaults = {
        "controller": ctrl_map.get(args.controller.lower(), "P"),
        "connection": args.connection,
        "image_topic": args.image_topic,
        "camera_info_topic": args.camera_info_topic,
        "marker_size": args.marker_size,
        "camera_yaw": args.camera_yaw,
        "approach_alt": args.approach_alt,
        "descent_rate": args.descent_rate,
        "h_tolerance": args.h_tolerance,
        "marker_timeout": args.marker_timeout,
        "land_distance": args.land_distance,
        "landing_yaw": args.landing_yaw,
        "kp": args.kp, "ki": args.ki, "kd": args.kd,
    }
    LandingGUI(defaults).run()


if __name__ == "__main__":
    main()
