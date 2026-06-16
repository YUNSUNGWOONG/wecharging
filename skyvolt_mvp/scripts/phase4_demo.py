#!/usr/bin/env python3
"""Phase 4 live demo — walk through the safety IPs and serve /metrics.

Prints how the load monitor, e-stop chain, and OTA updater behave, then starts
the HTTP /metrics endpoint so you can open it in a browser (Grafana scrapes the
same endpoint). Ctrl+C to stop.
"""
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "skyvolt_safety"))

from skyvolt_safety import (   # noqa: E402
    LoadMonitor, EStopChain, OtaUpdater, FirmwareImage,
    FleetSnapshot, fleet_health_text, MetricsServer, SafetyAction,
)


def hr(title):
    print("\n" + "=" * 64 + f"\n {title}\n" + "=" * 64)


# 1) Load / tilt monitor -------------------------------------------------
hr("1) 트랙 하중·기울기 모니터  (로드셀 + 경사계 -> 안전 조치)")
mon = LoadMonitor(rated_load_n=2000.0)
for label, loads, tilt in [
    ("정상   ", [400, 400, 400, 400], 0.2),
    ("불균형 ", [900, 300, 300, 300], 0.1),
    ("과부하 ", [700, 700, 700, 700], 0.1),
    ("기울기 ", [400, 400, 400, 400], 3.0),
]:
    r = mon.assess(loads, tilt)
    print(f"   {label}: 총 {r.total_n:5.0f}N, 불균형 {r.max_imbalance:4.0%}, "
          f"기울기 {r.tilt_deg:.1f}° -> {r.state.name:11s} / {r.action.name}")

# 2) E-stop chain --------------------------------------------------------
hr("2) E-stop 체인  (래칭 + 통신 감시)")
c = EStopChain(heartbeat_timeout_s=1.0)
c.heartbeat(0.0)
print(f"   시작        : power_enabled={c.power_enabled}")
c.trip("hw_button")
print(f"   버튼 트립   : tripped={c.tripped}, power_enabled={c.power_enabled}")
c.set_source("hw_button", False)
print(f"   소스 해제   : tripped={c.tripped}  (래치 유지)")
print(f"   reset()     : {c.reset()} -> power_enabled={c.power_enabled}")
c2 = EStopChain(); c2.heartbeat(0.0); c2.poll(2.0)
print(f"   통신 두절   : tripped={c2.tripped}  (하트비트 stale -> 자동 트립)")

# 3) OTA updater ---------------------------------------------------------
hr("3) OTA 펌웨어 업데이트  (검증 + 롤백)")
img = FirmwareImage(version="1.2.0", sha256="abc123")
ok = OtaUpdater(current_version="1.1.0")
ok.begin(img); ok.complete_download("abc123"); ok.activate(); ok.confirm_health(True)
print(f"   정상 경로   : {ok.state.name}, 실행버전={ok.current_version}  (커밋)")
bad = OtaUpdater(current_version="1.1.0")
bad.begin(img); bad.complete_download("abc123"); bad.activate(); bad.confirm_health(False)
print(f"   헬스 실패   : {bad.state.name}, 실행버전={bad.current_version}  (롤백)")
cs = OtaUpdater(current_version="1.1.0")
cs.begin(img); cs.complete_download("WRONG")
print(f"   체크섬 불일치: {cs.state.name}, 실행버전={cs.current_version}  (거부)")

# 4) Live health dashboard + /metrics -----------------------------------
hr("4) 라이브 헬스 대시보드  (부하 급증 -> E-stop 자동 작동 시나리오)")
_t0 = time.time()
_mon = LoadMonitor(rated_load_n=2000.0)
_chain = EStopChain()
_chain.heartbeat(_t0)


def provider() -> FleetSnapshot:
    """Evolving scenario: ~14 s of normal running, then a load spike that the
    LoadMonitor flags OVERLOAD -> trips the (latching) e-stop chain."""
    el = time.time() - _t0
    _chain.heartbeat(time.time())                      # comms alive
    tilt = 0.3 + 0.15 * math.sin(el / 2.0)
    if el < 25:                                        # normal operation
        load = 1500 + 150 * math.sin(el)
        busy, queued = 2, max(0, 6 - int(el / 2))
    else:                                              # overload spike
        load = 2100.0
        busy, queued = 0, 4
    reading = _mon.assess([load / 4] * 4, tilt)        # rail cells
    _chain.set_source("load_estop", reading.action == SafetyAction.ESTOP)
    tripped = _chain.tripped
    return FleetSnapshot(
        robots_total=3, robots_busy=0 if tripped else busy,
        jobs_queued=queued, jobs_completed=10 + int(min(el, 25)),
        reservations_active=0 if tripped else 9,
        estop_tripped=tripped, rail_load_n=load, track_tilt_deg=tilt)


server = MetricsServer(provider, host="127.0.0.1", port=9090).start()
print(f"   대시보드:  http://localhost:{server.port}/        <- 브라우저로 여기 열기")
print(f"   원시 메트릭: http://localhost:{server.port}/metrics")
print("   시나리오: ~14초 정상 운영 후 부하 급증 -> E-stop 자동 트립(배너 빨강)")
print("   (Ctrl+C 로 종료)")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    server.stop()
    print("\n[demo] stopped.")
