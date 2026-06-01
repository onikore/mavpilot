# Changelog

## 0.3.0 — 2026-06-01

### Safety / correctness
- **OFFBOARD setpoint starvation fixed.** The receiver thread no longer holds
  the MAVLink I/O lock across its 50 ms blocking read; the lock now serializes
  sends only. A blocking receive can no longer stall the 50 Hz setpoint stream
  (which PX4 punishes by dropping OFFBOARD).
- **`apply_safe_params()` now verifies every write.** `PARAM_SET` is
  unacknowledged in MAVLink, so a dropped frame previously left a safety
  parameter silently unset. Each param is now written via `set_param_checked`,
  which reads it back (`PARAM_REQUEST_READ` → `PARAM_VALUE`), retries, and
  raises `DroneError` if it can't confirm the value.
- **Outbound send-fault detection.** The setpoint streamer latches a
  `send_fault` after ~0.5 s of consecutive send failures (the outbound link is
  down); mission methods then raise `DroneError` (`emergency_land` still
  bypasses). Streamer warnings are rate-limited to once per second.
- **`__aexit__` emergency-land hardened.** Telemetry tracks a sticky
  `ever_armed` flag; the context-manager exit lands on error when
  `is_armed()` **or** `ever_armed()`, so frozen telemetry reporting
  `armed=False` can't silently skip the safety path.

### Robustness
- **`reconnect()`** — explicit link-loss recovery on `DroneController` and
  `MAVLinkConnection`: reopens the link, restarts the heartbeat/receiver
  threads, re-requests data streams, and clears the watchdog/send-fault
  latches. No automatic mid-mission reconnection (it would race live mission
  state); the caller must re-arm and re-enter OFFBOARD afterwards. Added a
  `link_alive()` helper.
- **Data-stream fallback.** `request_data_streams` also sends the legacy
  `REQUEST_DATA_STREAM` after `SET_MESSAGE_INTERVAL`, for stacks that ignore
  the interval command. `wait_until_ready` timeouts now name which gate failed
  (no fresh `LOCAL_POSITION_NED` vs unset `SYS_STATUS` AHRS health).

### Features
- **Telemetry subscription API** — `subscribe(callback) -> unsubscribe` and an
  async `telemetry_stream()` generator deliver ~10 Hz snapshots (the same data
  the visualizer receives), working with `enable_viz=False`.
- CLI: new `--version`, `--log-level`, `--loop-hz`, and `--watchdog-s` flags.

### Internal / tooling
- Telemetry and setpoint state are now typed via `TelemetryState` /
  `SetpointState` `TypedDict`s (mypy checks keys/types; no call-site changes).
- Property tests for `normalize_yaw_deg` range and `pixel_to_body_offset`
  sign/monotonicity.
- Dependabot for `pip` and GitHub Actions; PyPI publish now emits provenance
  attestations.

## 0.2.3 — 2026-05-29

### Docs
- Expanded docstrings across the public API so the generated reference is
  description-rich rather than bare signatures: full Google-style
  Args/Returns/Raises (plus examples) on `DroneController` and all its methods,
  field-level docs on `Position`/`MarkerObservation`/`PrecisionLandResult` and
  the `PrecisionLandStatus` members, and thorough `mavpilot.utils` docs.
- `goto`, `precision_land`, and `send_command_long` now have explicit
  signatures (were `*args, **kwargs`), so the docs and editor autocomplete show
  their real parameters.
- The docs build now also renders `mavpilot.utils`; internal `mavpilot.core.*`
  stays hidden via the package's `__all__`.

## 0.2.2 — 2026-05-29

### Docs
- English `README.md` is now the primary readme; the Russian translation moved
  to `README.ru.md` (language switcher links updated both ways).
- Auto-generated API documentation via [pdoc](https://pdoc.dev/), built from
  the source docstrings — `scripts/build_docs.sh` locally, and a new `Docs`
  GitHub Actions workflow that deploys to GitHub Pages on `main` and `v*` tags.
- A real `docs/` folder is now tracked (only `docs/superpowers/` planning notes
  and the generated `docs/api/` output stay git-ignored).

## 0.2.1 — 2026-05-29

### Internal
- Moved the eight internal collaborator modules into a `mavpilot/core/`
  subpackage (`connection`, `telemetry`, `commands`, `streamer`, `mission`,
  `precision_land`, `safety`, `mock`) to declutter the top-level package.
  No public API change — `mavpilot`, `mavpilot.controller`, `mavpilot.errors`,
  `mavpilot.types`, `mavpilot.utils`, and `mavpilot.viz` are unchanged.
- CI: PyPI publish now triggers only on `v*` tags and requires the test job
  to pass first.

## 0.2.0 — 2026-05-29

### Breaking changes
- `DroneController.close()` is deprecated. Use `await drone.aclose()` or
  `async with DroneController(...) as drone:`.
- `DroneController.precision_land()` now returns `PrecisionLandResult`
  (not `bool`). Use `if result:` / `result.status` to dispatch on outcome.
- `VizServer` and `DroneController(viz_host=...)` default to `127.0.0.1`.
  Use `--viz-host 0.0.0.0` (CLI) or `viz_host="0.0.0.0"` (programmatic)
  to expose telemetry on the LAN.
- `DroneError` lives in `mavpilot.errors` (still re-exported from
  `mavpilot` top-level).
- The package layout changed: `controller.py` is now a facade composing
  `_connection`, `_telemetry`, `_commands`, `_streamer`, `_mission`,
  `_precision_land`, `_safety`, `_mock`. The public API is unchanged
  except where noted above.

### Fixes (safety / correctness)
- Pinhole math in `pixel_to_body_offset` now uses
  `dy = alt * px_norm * tan(half_fov)` (was the linear approximation,
  ~17% error at the edge of a 90° FOV).
- `emergency_land()` chain now: `AUTO_LAND → MAV_CMD_NAV_LAND → DO_FLIGHTTERMINATION`
  with 10 s / 5 s waits. Previously only the exception path triggered
  termination — `land()` timeout returning False was silently ignored.
- `return_to_launch()` requires `landed_state == ON_GROUND AND not armed`.
  Previously the OR condition reported "RTL complete" on a kill-switch
  disarm mid-flight.
- `precision_land()` adds `min_altitude_floor_m` (default 0.3): descent
  below floor is permitted only with a centered marker. Marker lost at
  or below floor → `ABORTED_AT_FLOOR`, not blind landing.
- CLI `KeyboardInterrupt` / `SIGINT` / `SIGTERM` now trigger
  `emergency_land()`. Previously `KeyboardInterrupt` bypassed the safety
  catch block.
- All MAVLink sends and the receiver `recv_match` go through a single
  lock inside `MAVLinkConnection` — fixes the v0.1.0 race where serial/TCP
  frames could interleave. Receiver `recv_match` timeout clamped to 50 ms.
- `takeoff()` order: stream → arm → set OFFBOARD. PX4 ≥1.13 sometimes
  refuses arm-in-OFFBOARD.
- `wait_until_ready()` now also gates on `SYS_STATUS.AHRS` health.
- `time_boot_ms` field uses monotonic clock with process-start offset.
- All telemetry handlers filter by `srcSystem == target_system` (was
  only HEARTBEAT in v0.1.0).
- `get_yaw_deg()` is normalized to `[-180, 180]`.

### Features
- `COMMAND_ACK` routing via `asyncio.Future` (`send_command_long(...)`).
  IN_PROGRESS results extend the deadline; non-ACCEPTED terminal results
  raise `DroneError`; duplicate in-flight commands raise immediately.
- Telemetry watchdog: 2 s (configurable via `telemetry_watchdog_s`)
  silence on `LOCAL_POSITION_NED` sets a flag; the next mission method
  raises `DroneError`. `emergency_land` intentionally bypasses the flag.
- NaN/Inf in viz telemetry is sanitized to `null` before JSON encoding
  — previously such events were silently dropped at `JSON.parse`.
- VizServer split into `viz/static/{index.html, main.js, sse.js,
  scene.js, telemetry.js, log.js, styles.css}` (ES modules; no build
  step). `max_clients` cap (default 32) returns HTTP 503 on overflow.
- VizServer shutdown sentinel: stop() unblocks SSE workers in <1 s
  instead of waiting up to 15 s.

### Tooling / packaging
- New dev dependencies: `pytest-asyncio`, `hypothesis`, `pytest-cov`.
- Tests reorganized into `tests/unit/`, `tests/protocol/`,
  `tests/integration/`.
- `pymavlink>=2.4,<3` pin.
- CI matrix adds Python 3.13, a black format check, and coverage.
- mypy global `disable_error_code` removed; per-line `# type: ignore`
  at pymavlink call sites instead.
- ruff rule set tightened: `["E","F","I","B","UP","S","SIM"]`.

## 0.1.0 — 2026-05-13

Initial alpha. Single `DroneController` class, monolithic `_viz.html`.
