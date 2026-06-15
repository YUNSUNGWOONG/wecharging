#!/usr/bin/env python3
"""Generate the standalone Phase 2 report (PHASE2_REPORT.pdf).

Separate from PROGRESS_REPORT.pdf — focuses only on Phase 2 (multi-robot in
sim). Korean text uses reportlab's bundled Adobe CID fonts (no font files).
Run:  python3 scripts/make_phase2_report.py
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
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, HRFlowable,
)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "PHASE2_REPORT.pdf")

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


body = ParagraphStyle("body", fontName=SERIF, fontSize=10, leading=15,
                      textColor=DARK, spaceAfter=4)
h1 = ParagraphStyle("h1", fontName=SANS, fontSize=15, leading=19,
                    textColor=BLUE, spaceBefore=14, spaceAfter=6)
h2 = ParagraphStyle("h2", fontName=SANS, fontSize=11.5, leading=15,
                    textColor=DARK, spaceBefore=8, spaceAfter=3)
title = ParagraphStyle("title", fontName="Helvetica-Bold", fontSize=24,
                       leading=28, textColor=DARK, alignment=TA_CENTER)
sub = ParagraphStyle("sub", fontName=SERIF, fontSize=11, leading=15,
                     textColor=GREY, alignment=TA_CENTER)
cap = ParagraphStyle("cap", fontName=SERIF, fontSize=8.5, leading=11,
                     textColor=GREY, alignment=TA_CENTER, spaceBefore=2)
cell = ParagraphStyle("cell", fontName=SERIF, fontSize=9, leading=12, textColor=DARK)
cellb = ParagraphStyle("cellb", fontName=SANS, fontSize=9, leading=12, textColor=DARK)
code = ParagraphStyle("code", fontName="Courier", fontSize=8.5, leading=12,
                      textColor=DARK, backColor=LIGHT, leftIndent=4, spaceAfter=1)
bstyle = ParagraphStyle("bul", fontName=SERIF, fontSize=10, leading=15,
                        textColor=DARK, spaceAfter=4, leftIndent=14,
                        bulletIndent=2, bulletFontName="Helvetica",
                        bulletFontSize=9, bulletColor=BLUE)


def P(t, s=body):
    return Paragraph(_fix(t), s)


def bullet(items):
    return [Paragraph(_fix(t), bstyle, bulletText="•") for t in items]


def table(rows, widths):
    data = [[Paragraph(f'<font color="white">{_fix(c)}</font>', cellb) for c in rows[0]]]
    data += [[Paragraph(_fix(c), cell) for c in r] for r in rows[1:]]
    t = Table(data, colWidths=widths, hAlign="LEFT")
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), BLUE),
        ("FONTNAME", (0, 0), (-1, 0), SANS),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, LIGHT]),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#cbd2d9")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 5), ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]))
    return t


def picture(path, width_cm, capt):
    if not os.path.exists(path):
        return []
    from PIL import Image as PILImage
    w, h = PILImage.open(path).size
    dw = width_cm * cm
    im = Image(path, width=dw, height=dw * h / w)
    box = Table([[im]], colWidths=[dw])
    box.setStyle(TableStyle([("ALIGN", (0, 0), (-1, -1), "CENTER"),
                             ("BOX", (0, 0), (-1, -1), 0.4, colors.HexColor("#cbd2d9"))]))
    return [box, Paragraph(_fix(capt), cap)]


story = []

# ---- Cover ----
story += [Spacer(1, 1.0 * cm), P("SkyvoltRobot MVP", title), Spacer(1, 0.2 * cm),
          P("Phase 2 보고서 — 다중로봇 시뮬레이션", sub),
          P("작성일: 2026-06-14", sub), Spacer(1, 0.4 * cm),
          HRFlowable(width="100%", thickness=0.6, color=colors.HexColor("#cbd2d9"),
                     spaceBefore=4, spaceAfter=6)]

# ---- 1. 개요 ----
story += [P("1. Phase 2 개요", h1)]
story += [P("Phase 2의 목표는 '다중로봇을 시뮬레이션에서 운용'하는 것이다. 로드맵의 4개 작업을 "
            "모두 완료했다 — 스케줄러를 대규모로 검증하고, 멈춘 로봇에 대한 데드락 복구를 넣고, "
            "운영자가 플릿을 관찰할 대시보드를 만들고, 마지막으로 이 모든 로직으로 Gazebo의 "
            "로봇 3대를 실제로 구동했다. 알고리즘은 순수 파이썬이라 ROS/Gazebo 없이도 게이트로 "
            "검증되며, 시뮬레이션 통합만 Gazebo를 사용한다.")]
story += [table([
    ["작업", "성격", "상태"],
    ["스케줄러 스트레스 테스트 (1000 작업/10 로봇)", "순수 SW", "완료"],
    ["예약 데드락 타임아웃 + 복구 정책", "순수 SW", "완료"],
    ["운영자 대시보드 (읽기 전용)", "SW (웹)", "완료"],
    ["폐루프 트랙 Gazebo 3+ 로봇", "SW + sim", "완료"],
], [8.6 * cm, 3.0 * cm, 2.2 * cm])]
story += [P("Phase 2 관련 자동화 테스트 27개 추가, 저장소 전체 <b>65개 테스트 통과</b>.", cap)]

# ---- 2. 스트레스 테스트 ----
story += [P("2. 스케줄러 스트레스 테스트", h1)]
story += [P("Scheduler.assign()은 호출당 유휴 로봇 1대에 1작업만 배정하므로, 1000개 작업을 "
            "처리하려면 시간을 진행시키는 폐루프 시뮬레이션이 필요하다. 배정 → 이동(busy) → "
            "도착 시 예약 해제·유휴 복귀 → 재배정을 반복하며, '대기 작업과 유휴 로봇이 있는데 "
            "admit 가능한 경로가 없고 풀어줄 작업도 없는' 진짜 교착을 감지한다. 게이트는 데드락 "
            "없음 + 100% 완료 + 잔여 예약 0이다.")]
story += [table([
    ["지표 (1000 작업 / 10 로봇)", "결과"],
    ["완료율", "1000 / 1000 (100%)"],
    ["데드락", "없음"],
    ["잔여 예약", "0 (전부 해제)"],
    ["처리량", "약 1.63 작업/초"],
    ["판정", "PASS"],
], [7.0 * cm, 6.8 * cm])]
story += [P("부하 확장성: 버스트 도착 시 로봇 2대(makespan 134.7 s) → 12대(45.5 s)로 "
            "처리량이 선형에 가깝게 확장된다.", cap)]

# ---- 3. 데드락 복구 ----
story += [P("3. 예약 데드락 — 타임아웃 + 복구 정책", h1)]
story += [P("로봇이 멈추면(잼·크래시) 그 예약이 해제되지 않아 트랙을 영구히 막는다. 예약 테이블에 "
            "타임아웃 회수(reclaim_expired)를 추가했다 — 경로 전체가 만료 시한을 넘긴 로봇을 "
            "'멈춤'으로 간주하고 예약을 회수한다. 복구 정책은 멈춘 로봇을 서비스에서 제외하고 그 "
            "작업을 다시 큐에 넣어 정상 로봇이 마저 처리하게 한다.")]
story += [table([
    ["멈춘 로봇 시나리오", "완료율", "데드락", "잔여 예약", "판정"],
    ["복구 OFF", "0.995", "발생", "3 (영구 점유)", "FAIL"],
    ["복구 ON", "1.000", "없음", "0", "PASS"],
], [4.6 * cm, 2.4 * cm, 2.4 * cm, 3.2 * cm, 1.8 * cm])]
story += [P("같은 멈춤 상황에서 복구 정책 하나로 데드락이 100% 완료로 바뀐다.", cap)]

# ---- 4. 대시보드 ----
story += [P("4. 운영자 대시보드 (읽기 전용)", h1)]
story += [P("기존 FastAPI 서버에 빌드 도구 없이 동작하는 바닐라 HTML/JS 대시보드를 추가했다. "
            "GET / 는 /fleet 을 1초마다 폴링해 트랙 위 로봇 위치(초록=유휴, 주황=작업중)와 "
            "예약·대기 작업 수를 실시간 표시한다. 작업 등록 컨트롤이 없는 읽기 전용이다.")]
story += picture("/tmp/dashboard_crop.png", 16.0,
                 "운영자 대시보드 — 로봇 3대 작업중, 트랙 위 위치·상태 카드")

# ---- 5. 폐루프 Gazebo ----
story += [P("5. 폐루프 트랙 Gazebo — 로봇 3대", h1)]
story += [P("Phase 2의 핵심: 위 스케줄러·예약·복구 로직으로 Gazebo의 로봇 3대를 실제로 "
            "구동한다. 각 로봇에 VelocityControl 플러그인을 달고 링크 중력을 꺼서(매달린 "
            "운동학 트윈) /model/<id>/cmd_vel 로 레일 위를 이동시킨다. 디렉터는 작업을 "
            "생성하고, 스케줄러로 배정하고, 도착 시 예약을 해제하고 재배정한다.")]
story += picture("/tmp/fleet_view.png", 9.5,
                 "Gazebo 주차장 월드의 SkyvoltRobot 3대 (북측 레일)")
story += [P("폐루프 로그 — 배정 ↔ 도착 ↔ 재배정 사이클 (40초 / 17작업 완료):", h2)]
for line in [
    "[  0.0s] r0 -> job j0 s=0.94 (eta 1.4s)",
    "[  1.4s] r0 arrived @ s=0.94 (done 1)",
    "[  2.4s] r0 -> job j1 s=0.60 (eta 0.7s)     <- immediate re-assign",
    "[  4.5s] r0 -> job j2 s=1.67 (eta 2.1s)",
    "[  8.8s] r2 -> job j4 s=3.02 (eta 2.3s)     <- all 3 robots active",
    "[ 11.8s] r2 arrived @ s=3.02 (done 5)",
    "[director] stopped. jobs completed: 17",
]:
    story.append(Paragraph(line.replace(" ", "&nbsp;"), code))
story += [P("해결한 엔지니어링 난점", h2)]
story += bullet([
    "로봇 낙하(z=2.2→0.4): prismatic 조인트+중력 → 전 링크 중력 off로 레일 유지.",
    "느린 침하(VelocityControl 잔류 드리프트): 주기적 set_pose z-보정으로 상쇄.",
    "set_pose CLI 지연(326 ms/회): velocity-control(부드러운 이동)+저빈도 z-보정 하이브리드.",
])
story += [P("참고: 시뮬레이션은 운동학 모델(접촉 물리 아님 — 프로젝트 Phase 0 설계)이라, 로봇 "
            "모션은 충전 작업 흐름의 시각화 수준이다. 실제 동역학은 Phase 3에서 매니퓰레이터와 "
            "함께 다룬다.", cap)]

# ---- 6. 파일 ----
story += [P("6. Phase 2 추가 / 변경 파일", h1)]
story += [table([
    ["파일", "구분", "내용"],
    ["skyvolt_fleet/fleet_sim.py", "신규", "폐루프 스케줄러 스트레스 테스트 + 복구 시뮬레이션"],
    ["scripts/fleet_director.py", "신규", "Gazebo 3대 로봇 폐루프 디렉터"],
    ["reservation.py", "수정", "reclaim_expired() 타임아웃 회수(데드락 복구)"],
    ["api/server.py", "수정", "운영자 대시보드(GET /) + /fleet 트랙길이"],
    ["skyvolt_robot.urdf.xacro", "수정", "VelocityControl 플러그인 + 링크 중력 off"],
    ["test_fleet_sim / dashboard / reservation", "신규/수정", "스트레스·복구·대시보드 테스트"],
], [5.8 * cm, 1.9 * cm, 8.1 * cm])]

# ---- 7. 실행 방법 ----
story += [P("7. 실행 방법", h1)]
for line in [
    "cd skyvolt_mvp",
    "python -m skyvolt_fleet.fleet_sim --jobs 1000 --robots 10   # stress test",
    "python -m skyvolt_fleet.fleet_sim --stuck 0 --recovery      # recovery demo",
    "cd api-server && ./run.sh        # dashboard at http://localhost:8080/",
    "./scripts/run_sim.sh fleet       # Gazebo 3 robots (terminal 1)",
    "python3 scripts/fleet_director.py --robots 3   # closed loop (terminal 2)",
]:
    story.append(Paragraph(line.replace(" ", "&nbsp;"), code))

# ---- 8. 결론 ----
story += [P("8. 결론", h1)]
story += bullet([
    "Phase 2의 4개 작업을 모두 완료 — 스케줄러는 1000작업·10로봇에서 데드락 없이 동작하고, "
    "멈춘 로봇은 복구되며, 운영자 대시보드와 Gazebo 폐루프 구동이 갖춰졌다.",
    "Phase 2 잔여 하드웨어 의존 작업은 없음. 남은 항목은 Phase 1 #1(RFID 실측, HW 필요)과 "
    "Phase 3(매니퓰레이터·비전).",
])

doc = SimpleDocTemplate(OUT, pagesize=A4, leftMargin=2 * cm, rightMargin=2 * cm,
                        topMargin=1.6 * cm, bottomMargin=1.6 * cm,
                        title="SkyvoltRobot MVP Phase 2 보고서")
doc.build(story)
print("PDF 생성 완료:", OUT)
