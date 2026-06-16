#!/usr/bin/env python3
"""One-page at-a-glance summary (SUMMARY.pdf)."""
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

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "SUMMARY.pdf")
pdfmetrics.registerFont(UnicodeCIDFont("HYSMyeongJo-Medium"))
pdfmetrics.registerFont(UnicodeCIDFont("HYGothic-Medium"))
SERIF, SANS = "HYSMyeongJo-Medium", "HYGothic-Medium"
BLUE = colors.HexColor("#2974C7"); DARK = colors.HexColor("#1f2933")
GREY = colors.HexColor("#52606d"); LIGHT = colors.HexColor("#eef2f7")
GREEN = colors.HexColor("#2f8f5b")
DOT = '<font name="Helvetica">&#183;</font>'


def fx(t):
    return t.replace("·", DOT)


title = ParagraphStyle("t", fontName="Helvetica-Bold", fontSize=19, leading=22,
                       textColor=DARK, alignment=TA_CENTER)
sub = ParagraphStyle("s", fontName=SERIF, fontSize=9.5, leading=13, textColor=GREY,
                     alignment=TA_CENTER)
h = ParagraphStyle("h", fontName=SANS, fontSize=11.5, leading=14, textColor=BLUE,
                   spaceBefore=9, spaceAfter=4)
body = ParagraphStyle("b", fontName=SERIF, fontSize=9, leading=12.5, textColor=DARK)
cell = ParagraphStyle("c", fontName=SERIF, fontSize=8.3, leading=10.5, textColor=DARK)
cb = ParagraphStyle("cb", fontName=SANS, fontSize=8.3, leading=10.5, textColor=DARK)


def P(t, s=body):
    return Paragraph(fx(t), s)


def table(rows, widths, aligns=None):
    data = [[Paragraph(f'<font color="white">{fx(c)}</font>', cb) for c in rows[0]]]
    data += [[Paragraph(fx(c), cell) for c in r] for r in rows[1:]]
    t = Table(data, colWidths=widths, hAlign="LEFT")
    st = [("BACKGROUND", (0, 0), (-1, 0), BLUE), ("FONTNAME", (0, 0), (-1, 0), SANS),
          ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, LIGHT]),
          ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#cbd2d9")),
          ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
          ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4),
          ("TOPPADDING", (0, 0), (-1, -1), 2.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5)]
    for col, a in (aligns or {}).items():
        st.append(("ALIGN", (col, 0), (col, -1), a))
    t.setStyle(TableStyle(st))
    return t


story = []
story += [P("SkyvoltRobot MVP", title),
          P("전체 작업 요약 — 레일형 EV 충전 로봇 디지털 트윈 · Phase 0~4 · 2026-06-16", sub),
          Spacer(1, 0.15 * cm),
          HRFlowable(width="100%", thickness=0.6, color=colors.HexColor("#cbd2d9"),
                     spaceBefore=3, spaceAfter=3)]

story += [P("Phase별 진행 — 0~4 전부 착수, 소프트웨어 핵심 완료", h)]
story += [table([
    ["Phase", "내용", "상태", "가시성"],
    ["0  디지털 트윈", "ROS2 워크스페이스 · 도킹평가 · Gazebo 실행", "완료", "Gazebo 로봇"],
    ["1  위치추정 견고화", "IMU 융합 · 속성 테스트 · Replay (RFID 실측만 HW)", "3/4", "알고리즘"],
    ["2  다중로봇 sim", "스케줄러 스트레스 · 데드락 복구 · 대시보드 · Gazebo 3대", "4/4", "로봇·대시보드"],
    ["3  매니퓰레이터·비전", "리드스크류 · 충전구 검출 · 서보잉 · 힘/토크", "4/4", "일부"],
    ["4  운영·안전", "하중 모니터 · E-stop · 메트릭 · OTA", "4/4", "백그라운드"],
], [3.4 * cm, 8.0 * cm, 1.7 * cm, 3.0 * cm], aligns={2: "CENTER"})]

story += [P("핵심 코드 — 6개 패키지 (순수 파이썬 IP)", h)]
story += [table([
    ["패키지", "무엇을 하나", "테스트"],
    ["skyvolt_localization", "칼만필터 위치추정 + 도킹 속도 FSM + IMU 융합", "27"],
    ["skyvolt_fleet", "다중로봇 스케줄러 + 예약/복구 + REST API", "31"],
    ["skyvolt_eval", "도킹 평가 하니스 + Replay 도구", "7"],
    ["skyvolt_manipulator", "리드스크류 기구학 + 힘/토크 가드", "22"],
    ["skyvolt_vision", "충전구 자세추정(Kabsch) + 서보잉 + RGB-D 검출", "24"],
    ["skyvolt_safety", "하중 모니터 + E-stop + 메트릭 + OTA", "39"],
    ["합계", "Phase 0~4 소프트웨어 핵심 IP", "150"],
], [4.2 * cm, 9.0 * cm, 1.9 * cm], aligns={2: "CENTER"})]

story += [P("산출물 · 남은 작업", h)]
story += [P("<b>산출물</b> — 보고서 6종 PDF(전체 + Phase 0~4), 실행 스크립트(run_tests · run_sim · "
            "run_docking_eval · fleet_director · phase4_demo).")]
story += [P("<b>남은 작업(전부 실물/배포 환경 필요)</b> — Phase 1: 실측 RFID(리더 장비) · "
            "Phase 3: 접촉 물리 sim, 실제 카메라 튜닝 · Phase 4: HW GPIO, OTA 전송, "
            "Prometheus/Grafana 배포.")]

story += [Spacer(1, 0.2 * cm)]
box = Table([[Paragraph(fx("<b>한 줄 요약</b> — 로드맵 Phase 0~4의 순수 소프트웨어로 가능한 "
                           "모든 작업을 구현·검증 완료(테스트 150개 통과). 남은 건 실물 장비/"
                           "배포 인프라가 있어야 하는 연동뿐."),
                        ParagraphStyle("box", fontName=SERIF, fontSize=9.5, leading=13,
                                       textColor=DARK))]], colWidths=[16.4 * cm])
box.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), colors.HexColor("#eaf4ec")),
                         ("BOX", (0, 0), (-1, -1), 0.6, GREEN),
                         ("LEFTPADDING", (0, 0), (-1, -1), 10),
                         ("RIGHTPADDING", (0, 0), (-1, -1), 10),
                         ("TOPPADDING", (0, 0), (-1, -1), 8),
                         ("BOTTOMPADDING", (0, 0), (-1, -1), 8)]))
story += [box]

SimpleDocTemplate(OUT, pagesize=A4, leftMargin=2.3 * cm, rightMargin=2.3 * cm,
                  topMargin=1.5 * cm, bottomMargin=1.3 * cm,
                  title="SkyvoltRobot MVP 요약").build(story)
print("PDF 생성 완료:", OUT)
