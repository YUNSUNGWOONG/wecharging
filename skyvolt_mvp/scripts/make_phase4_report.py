#!/usr/bin/env python3
"""Generate the standalone Phase 4 report (PHASE4_REPORT.pdf).

Phase 4 = operations & safety. Load/tilt monitor and e-stop chain are done;
OTA and metrics are planned. Korean text uses reportlab's bundled CID fonts.
Run:  python3 scripts/make_phase4_report.py
"""
import os

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, HRFlowable,
)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "PHASE4_REPORT.pdf")

pdfmetrics.registerFont(UnicodeCIDFont("HYSMyeongJo-Medium"))
pdfmetrics.registerFont(UnicodeCIDFont("HYGothic-Medium"))
SERIF, SANS = "HYSMyeongJo-Medium", "HYGothic-Medium"

BLUE = colors.HexColor("#2974C7")
DARK = colors.HexColor("#1f2933")
GREY = colors.HexColor("#52606d")
LIGHT = colors.HexColor("#f0f3f7")
DOT = '<font name="Helvetica">&#183;</font>'


def _fix(t):
    return t.replace("·", DOT)


body = ParagraphStyle("body", fontName=SERIF, fontSize=10, leading=15, textColor=DARK, spaceAfter=4)
h1 = ParagraphStyle("h1", fontName=SANS, fontSize=15, leading=19, textColor=BLUE, spaceBefore=14, spaceAfter=6)
h2 = ParagraphStyle("h2", fontName=SANS, fontSize=11.5, leading=15, textColor=DARK, spaceBefore=8, spaceAfter=3)
title = ParagraphStyle("title", fontName="Helvetica-Bold", fontSize=24, leading=28, textColor=DARK, alignment=TA_CENTER)
sub = ParagraphStyle("sub", fontName=SERIF, fontSize=11, leading=15, textColor=GREY, alignment=TA_CENTER)
cap = ParagraphStyle("cap", fontName=SERIF, fontSize=8.5, leading=11, textColor=GREY, alignment=TA_CENTER, spaceBefore=2)
cell = ParagraphStyle("cell", fontName=SERIF, fontSize=9, leading=12, textColor=DARK)
cellb = ParagraphStyle("cellb", fontName=SANS, fontSize=9, leading=12, textColor=DARK)
code = ParagraphStyle("code", fontName="Courier", fontSize=8.5, leading=12, textColor=DARK, backColor=LIGHT, leftIndent=4, spaceAfter=1)
bstyle = ParagraphStyle("bul", fontName=SERIF, fontSize=10, leading=15, textColor=DARK, spaceAfter=4,
                        leftIndent=14, bulletIndent=2, bulletFontName="Helvetica", bulletFontSize=9, bulletColor=BLUE)


def P(t, s=body):
    return Paragraph(_fix(t), s)


def bullet(items):
    return [Paragraph(_fix(t), bstyle, bulletText="•") for t in items]


def table(rows, widths):
    data = [[Paragraph(f'<font color="white">{_fix(c)}</font>', cellb) for c in rows[0]]]
    data += [[Paragraph(_fix(c), cell) for c in r] for r in rows[1:]]
    t = Table(data, colWidths=widths, hAlign="LEFT")
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), BLUE), ("FONTNAME", (0, 0), (-1, 0), SANS),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, LIGHT]),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#cbd2d9")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 5), ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]))
    return t


def codeblock(lines):
    return [Paragraph(ln.replace(" ", "&nbsp;"), code) for ln in lines]


story = []
story += [Spacer(1, 1.0 * cm), P("SkyvoltRobot MVP", title), Spacer(1, 0.2 * cm),
          P("Phase 4 보고서 — 운영 & 안전 (Operations & Safety)", sub),
          P("작성일: 2026-06-15", sub), Spacer(1, 0.4 * cm),
          HRFlowable(width="100%", thickness=0.6, color=colors.HexColor("#cbd2d9"), spaceBefore=4, spaceAfter=6)]

# 1. 개요
story += [P("1. Phase 4 개요", h1)]
story += [P("Phase 4는 운영·안전 계층으로, 다른 단계와 병행하며 지속된다. 매달린 트랙에서 로봇이 "
            "도는 동안 구조 안전을 지키고, 어떤 고장에도 안전하게 정지하며, 플릿 상태를 관측하고, "
            "펌웨어를 안전하게 갱신할 수 있게 한다. 4개 작업의 소프트웨어 핵심 IP를 모두 순수 "
            "파이썬으로 구현했다.")]
story += [table([
    ["작업", "성격", "상태"],
    ["트랙 하중·기울기 모니터링", "순수 SW", "완료"],
    ["E-stop 체인 (HW + SW 폴백)", "순수 SW", "완료"],
    ["Prometheus/Grafana 플릿 헬스", "순수 SW", "메트릭 완료"],
    ["OTA 펌웨어 업데이트 경로", "순수 SW", "FSM 완료"],
], [8.6 * cm, 3.0 * cm, 2.4 * cm])]
story += [P("이번 증분: skyvolt_safety 패키지 + 테스트 33개. 저장소 전체 138개 통과.", cap)]

# 2. 하중 모니터
story += [P("2. 트랙 하중·기울기 모니터링", h1)]
story += [P("각 레일 지지대는 로드셀로, 경사계는 트랙 기울기를 보고한다. LoadMonitor가 이를 받아 "
            "구조 위험을 분류한다 — 총 하중 정격 초과(OVERLOAD), 트랙 기울기 한계 초과"
            "(TILT_FAULT), 한 지지대의 과편중(IMBALANCED) — 그리고 운영 조치를 권고한다.")]
story += [table([
    ["상태", "의미", "조치"],
    ["NOMINAL", "정상 분포", "OK"],
    ["IMBALANCED", "한 지지대 과편중 (잼·지지대 열화)", "HOLD (정지·점검)"],
    ["OVERLOAD", "총 하중 정격 초과", "ESTOP"],
    ["TILT_FAULT", "기울기 한계 초과 (처짐·지지 실패)", "ESTOP"],
], [3.0 * cm, 8.4 * cm, 4.2 * cm])]
story += [P("구조 위험(과부하·기울기)은 ESTOP을 우선한다. 단위 테스트 8개.", cap)]

# 3. E-stop
story += [P("3. E-stop 체인 (하드웨어 + 소프트웨어 폴백)", h1)]
story += [P("모든 정지 소스 — 하드웨어 버튼, 소프트웨어 폴트(하중 ESTOP·서보 FAULT 등), 통신 두절 "
            "감시 — 를 하나의 래칭 체인으로 묶는다.")]
story += bullet([
    "어느 한 소스라도 작동하면 ESTOPPED로 래치되고 전원을 차단한다.",
    "래치 유지 — 소스가 해제돼도 전원이 자동 복구되지 않는다.",
    "reset()은 모든 소스가 해제된 경우에만 재무장한다(명시적 확인).",
    "통신 감시(소프트웨어 폴백): 하트비트가 끊기면 트립 — 멈춘 컨트롤러가 트랙을 활성으로 "
    "둘 수 없다.",
])
story += [P("단위 테스트 9개. LoadMonitor의 ESTOP 조치가 체인 소스로 직접 연결된다.", cap)]

# 4. 통합
story += [P("4. 통합 — 하중 모니터 → E-stop", h1)]
story += [P("두 안전 IP는 직접 연결된다: 하중/기울기 위험이 감지되면 그 ESTOP 조치가 E-stop "
            "체인을 트립해 전원을 차단한다.")]
story += codeblock([
    "reading = LoadMonitor(rated_load_n=1000).assess([900, 900])  # 1800 N",
    "  -> state=OVERLOAD, action=ESTOP",
    "chain.set_source('load_estop', reading.action == ESTOP)",
    "  -> chain.tripped == True   (power cut, latched)",
])

# 5. 메트릭
story += [P("5. Prometheus 플릿 헬스 메트릭", h1)]
story += [P("플릿·안전 상태 스냅샷을 Prometheus 노출 형식 텍스트로 렌더링해 Grafana가 플릿 "
            "헬스(가동률·대기열·예약·E-stop·레일 하중·기울기)를 차트로 그릴 수 있게 한다. "
            "의존성 없이 노출 형식을 직접 생성하므로 prometheus_client 없이도 동작·테스트된다. "
            "HTTP /metrics 엔드포인트는 이 위의 얇은 래퍼다.")]
story += codeblock([
    "# TYPE skyvolt_robots_busy gauge",
    "skyvolt_robots_busy 2",
    "# TYPE skyvolt_jobs_completed_total counter",
    "skyvolt_jobs_completed_total 17",
    "skyvolt_estop_tripped 0      # 1 = tripped",
])
story += [P("단위 테스트 7개(HELP/TYPE 방출·counter·bool→1/0·라벨·스냅샷 값).", cap)]

# 6. OTA
story += [P("6. OTA 펌웨어 업데이트 경로", h1)]
story += [P("스테이징·검증·롤백을 갖춘 페일세이프 펌웨어 롤아웃 FSM이다. "
            "IDLE→DOWNLOADING→VERIFYING→STAGED→ACTIVATING→ACTIVE로 진행하되, 체크섬 불일치는 "
            "FAILED, 활성화 후 헬스체크 실패는 ROLLED_BACK으로 간다. 실행 중인 '검증된' 버전은 "
            "헬스체크 통과 시에만 커밋되므로, 잘못된 업데이트가 로봇을 벽돌로 만들 수 없다.")]
story += [P("단위 테스트 9개(정상 커밋·체크섬 불일치 유지·헬스 실패 롤백·동일버전/진행중 거부·"
            "잘못된 전이·중단·재시도).", cap)]

# 7. 파일
story += [P("7. Phase 4 추가 파일", h1)]
story += [table([
    ["파일", "구분", "내용"],
    ["skyvolt_safety/ (패키지)", "신규", "운영·안전 모니터 패키지(ament_python)"],
    ["load_monitor.py / estop.py", "신규", "하중·기울기 모니터(8) + 래칭 E-stop(9)"],
    ["metrics.py / ota.py", "신규", "Prometheus 메트릭(7) + OTA 업데이트 FSM(9)"],
    ["pytest.ini / run_tests.sh", "수정", "safety 테스트 경로 추가"],
], [6.0 * cm, 1.8 * cm, 7.8 * cm])]

# 8. 실행
story += [P("8. 실행 방법", h1)]
story += codeblock([
    "cd skyvolt_mvp",
    "./scripts/run_tests.sh    # all tests (138, incl. 33 safety)",
    "python3 -c \"from skyvolt_safety import LoadMonitor, EStopChain, \\",
    "  OtaUpdater, fleet_health_text; ...\"",
])

# 9. 결론
story += [P("9. 결론", h1)]
story += bullet([
    "Phase 4의 소프트웨어 핵심 4건을 순수 파이썬으로 구현·검증했다 — 하중·기울기 모니터, "
    "래칭 E-stop 체인, Prometheus 플릿 헬스 메트릭, OTA 업데이트 FSM.",
    "위험 감지 → 전원 차단(E-stop), 안전한 펌웨어 갱신, 관측성까지 운영 안전 계층의 토대가 "
    "갖춰졌다. 남은 것은 하드웨어/전송 연동(GPIO, HTTP /metrics, OTA 전송, Grafana 보드)이다.",
])

doc = SimpleDocTemplate(OUT, pagesize=A4, leftMargin=2 * cm, rightMargin=2 * cm,
                        topMargin=1.6 * cm, bottomMargin=1.6 * cm,
                        title="SkyvoltRobot MVP Phase 4 보고서")
doc.build(story)
print("PDF 생성 완료:", OUT)
