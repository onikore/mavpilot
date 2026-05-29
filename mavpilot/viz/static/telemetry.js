// Telemetry HUD: updates the right-hand panel DOM from drone state.
const MODE_NAMES = {
  1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL',
  4: 'AUTO', 5: 'ACRO', 6: 'OFFBOARD', 7: 'STAB',
};
const SUBMODE_NAMES = {
  1: 'LOITER', 2: 'MISSION', 3: 'RTL', 4: 'TAKEOFF',
  5: 'LAND', 6: 'FOLLOW', 9: 'PRECLAND',
};
const LANDED_NAMES = { 0: '—', 1: 'ON_GROUND', 2: 'IN_AIR', 3: 'TAKING_OFF', 4: 'LANDING' };

export function updateTelemetryHud(drone) {
  const armedBadge = document.getElementById('armed-badge');
  armedBadge.textContent = drone.armed ? 'ARMED' : 'DISARMED';
  armedBadge.className = 'badge ' + (drone.armed ? 'armed' : 'disarmed');

  let modeName = MODE_NAMES[drone.main_mode] || ('m' + drone.main_mode);
  if (drone.main_mode === 4 && drone.sub_mode in SUBMODE_NAMES) {
    modeName += '.' + SUBMODE_NAMES[drone.sub_mode];
  }
  const modeEl = document.getElementById('mode');
  modeEl.textContent = modeName;
  modeEl.className = 'v mode-' + (MODE_NAMES[drone.main_mode] || '');

  document.getElementById('landed').textContent = LANDED_NAMES[drone.landed] || '—';
  document.getElementById('stream').textContent = drone.streaming ? 'OFFBOARD ↻' : '—';
  document.getElementById('battery').firstChild.textContent =
    (drone.battery * 100).toFixed(0) + '% ';
  document.getElementById('bbar').style.width = (drone.battery * 100) + '%';

  document.getElementById('px').textContent = drone.x.toFixed(2);
  document.getElementById('py').textContent = drone.y.toFixed(2);
  document.getElementById('pz').textContent = drone.z.toFixed(2);
  document.getElementById('alt').textContent = (-drone.z).toFixed(2) + ' m';
  document.getElementById('yaw').textContent = (drone.yaw * 180 / Math.PI).toFixed(1) + '°';
  const spd = Math.sqrt(drone.vx*drone.vx + drone.vy*drone.vy + drone.vz*drone.vz);
  document.getElementById('spd').textContent = spd.toFixed(2) + ' m/s';

  if (drone.setpoint) {
    document.getElementById('spx').textContent = drone.setpoint.x.toFixed(2);
    document.getElementById('spy').textContent = drone.setpoint.y.toFixed(2);
    document.getElementById('spz').textContent = drone.setpoint.z.toFixed(2);
    document.getElementById('spyaw').textContent =
      drone.setpoint.yaw == null
        ? 'keep'
        : (drone.setpoint.yaw * 180 / Math.PI).toFixed(1) + '°';
  }
}

export function updateActiveCmd(activeCommand) {
  const el = document.getElementById('active-cmd');
  if (!activeCommand) { el.textContent = 'none'; return; }
  const c = activeCommand;
  let s = '<span class="cmd-name">' + c.command + '</span>';
  if (c.target) s += '<br>→ (' + c.target.x.toFixed(1) + ', ' + c.target.y.toFixed(1) + ', ' + c.target.z.toFixed(1) + ')';
  if (c.yaw_deg != null) s += ' yaw=' + c.yaw_deg + '°';
  if (c.altitude_m != null) s += '<br>altitude ' + c.altitude_m + ' m';
  if (c.timeout_s != null) s += '<br>timeout ' + c.timeout_s + ' s';
  if (c.hover_time_s != null) s += ', hover ' + c.hover_time_s + ' s';
  if (c.duration_s != null) s += '<br>' + c.duration_s + ' s';
  if (c.descent_rate_mps != null) s += '<br>descent ' + c.descent_rate_mps + ' m/s, final ' + c.final_altitude_m + ' m';
  el.innerHTML = s;
}
