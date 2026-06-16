"""HTTP /metrics endpoint for Prometheus scraping (Phase 4 observability).

Wraps the dependency-free metrics renderer (skyvolt_safety.metrics) in a tiny
stdlib HTTP server so Prometheus can scrape live fleet health and Grafana can
chart it. A snapshot provider callback supplies the current FleetSnapshot on
each scrape, so the served metrics are always live.

    server = MetricsServer(lambda: collect_snapshot()).start()
    # Prometheus scrapes http://host:9090/metrics ; server.stop() to shut down

stdlib only (http.server) — no prometheus_client or web framework.
"""
from __future__ import annotations

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable

from .metrics import FleetSnapshot, fleet_health_text

SnapshotProvider = Callable[[], FleetSnapshot]


def _make_handler(provider: SnapshotProvider):
    class _Handler(BaseHTTPRequestHandler):
        def _send(self, body: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802
            path = self.path.rstrip("/")
            if path == "/metrics":
                self._send(fleet_health_text(provider()).encode("utf-8"),
                           "text/plain; version=0.0.4; charset=utf-8")
            elif path == "":   # "/" — human-facing health dashboard
                self._send(_DASHBOARD_HTML.encode("utf-8"),
                           "text/html; charset=utf-8")
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, *args):  # silence access logging
            pass

    return _Handler


_DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>SkyvoltRobot Fleet Health & Safety</title>
<style>
 :root{--bg:#0f1722;--panel:#172230;--line:#26354a;--txt:#e6edf5;--muted:#8aa0b8;
       --ok:#3fb27f;--warn:#e0a23a;--bad:#e0533a;}
 *{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);
   font-family:system-ui,Segoe UI,Roboto,sans-serif}
 header{display:flex;align-items:center;gap:12px;padding:16px 24px;border-bottom:1px solid var(--line)}
 header h1{font-size:18px;margin:0;font-weight:600}.sp{flex:1}#upd{font-size:12px;color:var(--muted)}
 main{padding:24px;max-width:980px;margin:0 auto}
 #estop{border-radius:12px;padding:20px;text-align:center;font-size:22px;font-weight:700;
        letter-spacing:.04em;margin-bottom:20px;border:1px solid var(--line)}
 .armed{background:#10311f;color:var(--ok)}.tripped{background:#3a1410;color:var(--bad)}
 .cards{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}
 .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px 16px}
 .k{font-size:12px;color:var(--muted)}.v{font-size:26px;font-weight:700;margin-top:4px}
 .panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:16px 18px;margin-top:20px}
 .panel h2{font-size:12px;color:var(--muted);margin:0 0 10px;text-transform:uppercase;letter-spacing:.04em}
 .bar{height:22px;border-radius:6px;background:#0e1620;overflow:hidden;border:1px solid var(--line)}
 .fill{height:100%;transition:width .3s,background .3s}
 .row{display:flex;justify-content:space-between;font-size:13px;color:var(--muted);margin-top:6px}
</style></head><body>
<header><h1>SkyvoltRobot — Fleet Health &amp; Safety</h1>
 <span class="sp"></span><span id="upd">connecting…</span></header>
<main>
 <div id="estop">…</div>
 <div class="cards">
  <div class="card"><div class="k">Robots busy / total</div><div class="v" id="rb">–</div></div>
  <div class="card"><div class="k">Jobs queued</div><div class="v" id="jq">–</div></div>
  <div class="card"><div class="k">Jobs completed</div><div class="v" id="jc">–</div></div>
  <div class="card"><div class="k">Reservations</div><div class="v" id="rs">–</div></div>
 </div>
 <div class="panel"><h2>Rail load</h2>
  <div class="bar"><div class="fill" id="loadfill"></div></div>
  <div class="row"><span id="loadval">– N</span><span>rated 2000 N</span></div></div>
 <div class="panel"><h2>Track tilt</h2>
  <div class="v" id="tilt" style="font-size:22px">–</div>
  <div class="row"><span>limit ±2.0°</span></div></div>
</main>
<script>
async function tick(){
 try{
  const txt = await (await fetch('/metrics')).text();
  const m={}; txt.split('\\n').forEach(l=>{if(l&&!l.startsWith('#')){const i=l.lastIndexOf(' ');m[l.slice(0,i)]=parseFloat(l.slice(i+1));}});
  const estop = m['skyvolt_estop_tripped']===1;
  const e=document.getElementById('estop');
  e.textContent = estop ? '⛔  E-STOP TRIPPED — power cut' : '✓  ARMED — power enabled';
  e.className = estop ? 'tripped' : 'armed';
  document.getElementById('rb').textContent = (m['skyvolt_robots_busy']||0)+' / '+(m['skyvolt_robots_total']||0);
  document.getElementById('jq').textContent = m['skyvolt_jobs_queued']||0;
  document.getElementById('jc').textContent = m['skyvolt_jobs_completed_total']||0;
  document.getElementById('rs').textContent = m['skyvolt_reservations_active']||0;
  const load=m['skyvolt_rail_load_newtons']||0, pct=Math.min(100,load/2000*100);
  const f=document.getElementById('loadfill'); f.style.width=pct+'%';
  f.style.background = load>1900?'var(--bad)':load>1500?'var(--warn)':'var(--ok)';
  document.getElementById('loadval').textContent = load.toFixed(0)+' N';
  const tilt=m['skyvolt_track_tilt_degrees']||0, t=document.getElementById('tilt');
  t.textContent = tilt.toFixed(2)+' °'; t.style.color = Math.abs(tilt)>2?'var(--bad)':'var(--ok)';
  document.getElementById('upd').textContent='updated '+new Date().toLocaleTimeString();
 }catch(e){document.getElementById('upd').textContent='disconnected';}
}
tick(); setInterval(tick,1000);
</script></body></html>"""


class MetricsServer:
    """Threaded HTTP server exposing /metrics."""

    def __init__(self, provider: SnapshotProvider,
                 host: str = "127.0.0.1", port: int = 9090) -> None:
        self._provider = provider
        self._host = host
        self._port = port
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> "MetricsServer":
        self._httpd = ThreadingHTTPServer((self._host, self._port),
                                          _make_handler(self._provider))
        self._port = self._httpd.server_address[1]   # resolve ephemeral port
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        daemon=True)
        self._thread.start()
        return self

    @property
    def port(self) -> int:
        return self._port

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
