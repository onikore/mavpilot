// Entry point: owns shared state, wires SSE → state + HUD/log/scene updates,
// and drives the animation loop.
import { initScene } from './scene.js';
import { subscribe } from './sse.js';
import { updateTelemetryHud, updateActiveCmd } from './telemetry.js';
import { appendCommand, appendLog } from './log.js';

// Trail retention (must match the cap scene.js draws within).
const TRAIL_MAX = 600;
const TRAIL_MS = 30000;

const drone = {
  x: 0, y: 0, z: 0, vx: 0, vy: 0, vz: 0, yaw: 0,
  armed: false, main_mode: 0, sub_mode: 0,
  battery: 1.0, landed: 0, streaming: false,
  setpoint: null,
};
let activeCommand = null;
let marker = null;
const trailData = []; // {x, y, z, t}

const stage = document.getElementById('stage');
const loading = document.getElementById('loading');
const scene = initScene(stage, loading);

subscribe({
  onTelemetry(msg) {
    drone.x = msg.x; drone.y = msg.y; drone.z = msg.z;
    drone.vx = msg.vx; drone.vy = msg.vy; drone.vz = msg.vz;
    drone.yaw = msg.yaw;
    drone.armed = msg.armed;
    drone.main_mode = msg.main_mode;
    drone.sub_mode = msg.sub_mode;
    drone.battery = msg.battery;
    drone.landed = msg.landed;
    drone.streaming = msg.streaming;
    drone.setpoint = msg.setpoint;
    const now = Date.now();
    trailData.push({ x: msg.x, y: msg.y, z: msg.z, t: now });
    if (trailData.length > TRAIL_MAX) trailData.shift();
    while (trailData.length > 0 && now - trailData[0].t > TRAIL_MS) trailData.shift();
    updateTelemetryHud(drone);
  },
  onCommand(msg) {
    activeCommand = msg;
    appendCommand(msg);
    updateActiveCmd(activeCommand);
  },
  onMarker(msg) {
    marker = { ned: msg.marker_ned, ts: msg.ts * 1000, err: msg.horizontal_err };
  },
  onLog(msg) {
    appendLog(msg);
  },
});

function render() {
  scene.frame(drone, activeCommand, marker, trailData);
  requestAnimationFrame(render);
}
requestAnimationFrame(render);
