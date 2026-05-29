// Server-Sent Events wrapper: connects to /events, tracks the connection
// badge, and routes parsed messages by type to the supplied handlers.

export function subscribe({ onTelemetry, onCommand, onMarker, onLog }) {
  const conn = document.getElementById('conn');
  const es = new EventSource('/events');
  es.addEventListener('open', () => { conn.className = 'online'; conn.textContent = 'live'; });
  es.addEventListener('error', () => { conn.className = 'offline'; conn.textContent = 'disconnected'; });
  es.addEventListener('message', (e) => {
    let msg;
    try { msg = JSON.parse(e.data); } catch { return; }
    switch (msg.type) {
      case 'telemetry': if (onTelemetry) onTelemetry(msg); break;
      case 'command': if (onCommand) onCommand(msg); break;
      case 'marker': if (onMarker) onMarker(msg); break;
      case 'log': if (onLog) onLog(msg); break;
    }
  });
  return es;
}
