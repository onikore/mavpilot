// Append-only command log and PX4 STATUSTEXT log views.

export function appendCommand(c) {
  const cmdLog = document.getElementById('cmdlog');
  const line = document.createElement('div');
  line.className = 'line';
  const time = new Date(c.ts * 1000).toLocaleTimeString();
  let html = '<span style="color:#555">' + time + '</span> <span class="cmd-name">' + c.command + '</span>';
  if (c.target) html += ' → (' + c.target.x.toFixed(1) + ', ' + c.target.y.toFixed(1) + ', ' + c.target.z.toFixed(1) + ')';
  line.innerHTML = html;
  cmdLog.insertBefore(line, cmdLog.firstChild);
  while (cmdLog.children.length > 30) cmdLog.lastChild.remove();
}

export function appendLog(m) {
  const logEl = document.getElementById('log');
  const line = document.createElement('div');
  const sev = m.severity <= 3 ? 'err' : m.severity <= 5 ? 'warn' : 'info';
  line.className = 'line sev-' + sev;
  const time = new Date(m.ts * 1000).toLocaleTimeString();
  line.textContent = '[' + time + '] ' + m.text;
  logEl.insertBefore(line, logEl.firstChild);
  while (logEl.children.length > 50) logEl.lastChild.remove();
}
