# -*- coding: utf-8 -*-
"""harness_sitl/docs/img/src/make_diagrams.py — 보고서용 SVG 다이어그램 생성.

외부 라이브러리를 쓰지 않는다 (표준 라이브러리만). 스킬 formal-report-readme §4 규칙:
흰 바탕, 직각 상자(rx=0), 선 굵기 1.0-1.4, 회색 계열 + 강조 1색(남색 #1f3a5f),
미사용·선택 요소는 회색 점선, 그림 안에 제목을 넣지 않는다 (캡션으로 대체).

실행
----
    ../plan_pontoon_rl/.venv/Scripts/python.exe -X utf8 docs/img/src/make_diagrams.py

설계 규칙 (2026-11-22 재작성)
---------------------------
* 모든 그림에 **범례**를 넣는다. 색·점선·번호의 뜻을 그림 안에서 닫는다
* 신호 흐름에는 **번호 1 2 3 …** 을 붙여 읽는 순서를 고정한다
* 블록에는 **하네스 파일명**을 같이 적어 그림과 코드를 1:1 로 잇는다
* 같은 칸을 반복해 채우지 않는다. 반복은 말로 적고 1 주기만 그린다
* 되먹임선은 **만드는 블록에서 출발**시킨다 (플랜트 / QP / 정책을 섞지 않는다)

주의
----
PDF 본문 높이가 248 mm 로 제한되므로 그림의 h/w 는 1.0 이하로 유지한다.
(2026-09-17 실측: 비율 1.44 를 넘는 그림이 쪽을 넘어가 잘렸다.)
생성 후 반드시 PNG 로 렌더해 겹침을 눈으로 확인한다.
"""
from __future__ import annotations

import os

FONT = "Malgun Gothic, Noto Sans CJK KR, sans-serif"
MONO = "Consolas, D2Coding, monospace"
NAVY = "#1f3a5f"
NAVY_FILL = "#e9eef5"
PALE = "#f4f7fb"
GRAY = "#555555"
LGRAY = "#8c8c8c"
BOX_FILL = "#ffffff"
GRAY_FILL = "#f4f4f4"

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)


# -- 기본 도형 --------------------------------------------------------------
def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def box(x, y, w, h, stroke=GRAY, fill=BOX_FILL, sw=1.2, dash=None):
    d = ' stroke-dasharray="5 3"' if dash else ""
    return ('<rect x="%g" y="%g" width="%g" height="%g" rx="0" '
            'fill="%s" stroke="%s" stroke-width="%g"%s/>\n'
            % (x, y, w, h, fill, stroke, sw, d))


def txt(x, y, s, size=12.5, anchor="middle", fill="#1a1a1a", font=FONT,
        weight="normal"):
    return ('<text x="%g" y="%g" font-family="%s" font-size="%g" '
            'text-anchor="%s" fill="%s" font-weight="%s">%s</text>\n'
            % (x, y, font, size, anchor, fill, weight, esc(s)))


def mono(x, y, s, size=11.5, anchor="middle", fill="#1a1a1a", weight="normal"):
    return txt(x, y, s, size=size, anchor=anchor, fill=fill, font=MONO,
               weight=weight)


def line(x1, y1, x2, y2, stroke=GRAY, sw=1.2, head=False, dash=None,
         marker="ah"):
    d = ' stroke-dasharray="5 3"' if dash else ""
    m = ' marker-end="url(#%s)"' % marker if head else ""
    return ('<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="%s" '
            'stroke-width="%g"%s%s/>\n' % (x1, y1, x2, y2, stroke, sw, d, m))


def path(pts, stroke=GRAY, sw=1.2, head=False, dash=None, marker="ah"):
    d = ' stroke-dasharray="5 3"' if dash else ""
    m = ' marker-end="url(#%s)"' % marker if head else ""
    p = " ".join(("M" if i == 0 else "L") + "%g,%g" % q
                 for i, q in enumerate(pts))
    return ('<path d="%s" fill="none" stroke="%s" stroke-width="%g"%s%s/>\n'
            % (p, stroke, sw, d, m))


def circ(x, y, r, stroke=NAVY, fill="#ffffff", sw=1.2):
    return ('<circle cx="%g" cy="%g" r="%g" fill="%s" stroke="%s" '
            'stroke-width="%g"/>\n' % (x, y, r, fill, stroke, sw))


def num(x, y, n, fill=NAVY):
    """흐름 번호. 흰 원 안에 숫자 — 선 위에 올려도 선이 가려진다."""
    return circ(x, y, 11, stroke=fill) + txt(x, y + 4.5, str(n), size=12,
                                             fill=fill, weight="bold")


def blk(x, y, w, h, title, lines, mlines=(), accent=True, rate=None):
    """표준 블록: 제목(굵게) + 설명줄 + 모노줄. 하단 오른쪽에 주기."""
    st = NAVY if accent else GRAY
    fl = NAVY_FILL if accent else BOX_FILL
    s = box(x, y, w, h, stroke=st, fill=fl, sw=1.4 if accent else 1.2)
    s += txt(x + w / 2, y + 24, title, size=13,
             fill=NAVY if accent else "#1a1a1a", weight="bold")
    yy = y + 46
    for t in lines:
        s += txt(x + w / 2, yy, t, size=11.5, fill="#243447")
        yy += 18
    for t in mlines:
        s += mono(x + w / 2, yy, t, size=10.5, fill="#4a5a6a")
        yy += 16
    if rate:
        s += txt(x + w - 10, y + h - 9, rate, size=10.5, anchor="end",
                 fill=NAVY if accent else "#6b6b6b", weight="bold")
    return s


def legend(x, y, w, items, title="범례"):
    """범례 상자. items = [(kind, label), ...]."""
    h = 30 + 20 * len(items)
    s = box(x, y, w, h, stroke=LGRAY, fill="#fbfbfb", sw=1.0)
    s += txt(x + 14, y + 21, title, size=11.5, anchor="start", fill=NAVY,
             weight="bold")
    yy = y + 42
    for kind, label in items:
        cx = x + 22
        if kind == "box":
            s += box(cx - 12, yy - 9, 24, 13, stroke=NAVY, fill=NAVY_FILL,
                     sw=1.4)
        elif kind == "pbox":
            s += box(cx - 12, yy - 9, 24, 13, stroke=NAVY, fill=PALE, sw=1.2)
        elif kind == "gbox":
            s += box(cx - 12, yy - 9, 24, 13, stroke=GRAY, fill=BOX_FILL)
        elif kind == "dbox":
            s += box(cx - 12, yy - 9, 24, 13, stroke=LGRAY, fill=GRAY_FILL,
                     sw=1.0, dash=True)
        elif kind == "arrow":
            s += line(cx - 12, yy - 3, cx + 12, yy - 3, stroke=NAVY, sw=1.4,
                      head=True, marker="ahn")
        elif kind == "darrow":
            s += line(cx - 12, yy - 3, cx + 12, yy - 3, stroke=LGRAY, sw=1.0,
                      head=True, dash=True)
        s += txt(x + 44, yy, label, size=11, anchor="start", fill="#333333")
        yy += 20
    return s


def note(x, y, w, title, lines, accent_last=False):
    """주석 상자: 제목 + 설명줄. 높이는 줄 수에서 계산한다."""
    h = 38 + 19 * len(lines)
    s = box(x, y, w, h, stroke=LGRAY, fill="#fbfbfb", sw=1.0)
    s += txt(x + 14, y + 21, title, size=11.5, anchor="start", fill=NAVY,
             weight="bold")
    yy = y + 44
    for i, t in enumerate(lines):
        last = accent_last and i == len(lines) - 1
        s += txt(x + 14, yy, t, size=11, anchor="start",
                 fill=NAVY if last else "#333333")
        yy += 19
    return s


def svg(w, h, body):
    head = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<svg xmlns="http://www.w3.org/2000/svg" width="%g" height="%g" '
            'viewBox="0 0 %g %g">\n' % (w, h, w, h))
    defs = ('<defs>\n'
            '<marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto-start-reverse">\n'
            '<path d="M0,1 L9,5 L0,9 z" fill="%s"/></marker>\n'
            '<marker id="ahn" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto-start-reverse">\n'
            '<path d="M0,1 L9,5 L0,9 z" fill="%s"/></marker>\n'
            '</defs>\n' % (LGRAY, NAVY))
    bg = '<rect x="0" y="0" width="%g" height="%g" fill="#ffffff"/>\n' % (w, h)
    return head + defs + bg + body + "</svg>\n"


def write(name, s):
    p = os.path.join(OUT, name)
    with open(p, "w", encoding="utf-8") as f:
        f.write(s)
    print("  %s  (%d B)" % (name, len(s.encode("utf-8"))))


# -- 그림 1. 시스템 구성 ----------------------------------------------------
def diagram_overview():
    """닫힌 제어루프. 위 = 제어 경로 3 블록, 아래 = 플랜트, 사이 = 되먹임."""
    W, H = 1180, 760
    b = ""

    y0, bh, bw = 72, 132, 318
    xs = [36, 431, 826]

    b += blk(xs[0], y0, bw, bh, "마더십",
             ["폰툰 자세·속도를 측정해 보냄", "RL 목표자세도 같이 보냄"],
             ["nodes/node_b_mothership.py"], rate="50 Hz")
    b += blk(xs[1], y0, bw, bh, "마더십 NUC — 상위 제어",
             ["ONNX 정책 10 Hz → 폰툰 목표력 τ_d",
              "Latch-aware QP 50 Hz → 터그 2대 몫"],
             ["nuc/controller.py (실물과 동일)",
              "nodes/node_c_nuc.py (전송 어댑터)"], rate="10 / 50 Hz")
    b += blk(xs[2], y0, bw, bh, "터그 2대",
             ["받은 τ 를 추진기 3기로 분해", "하위 역행렬 det = 2d = 0.6"],
             ["nodes/node_d_esp32.py", "plant/tug_lower.py"], rate="50 Hz")

    ry, rh = y0 + bh + 26, 40
    reals = ["마더십 Pixhawk + Simulink 앱",
             "마더십 NUC (ONNX Runtime)",
             "터그 ESP32 → 터그 Pixhawk ×2"]
    for x, r in zip(xs, reals):
        b += line(x + bw / 2, y0 + bh, x + bw / 2, ry, stroke=LGRAY, sw=1.0,
                  dash=True)
        b += box(x, ry, bw, rh, stroke=LGRAY, fill=GRAY_FILL, sw=1.0,
                 dash=True)
        b += txt(x + bw / 2, ry + 25, r, size=11, fill="#4a4a4a")

    py, ph, px, pw = 440, 104, 236, 708
    b += blk(px, py, pw, ph, "플랜트 — 폰툰 + 터그 2대 합동운동",
             ["전자석으로 묶인 3-DOF 강체. M2 하이브리드 UDE · RK4 적분"],
             ["nodes/node_a_plant.py (학습 환경과 같은 모델)"], accent=False,
             rate="50 Hz")
    b += line(px + pw / 2, py + ph, px + pw / 2, py + ph + 24, stroke=LGRAY,
              sw=1.0, dash=True)
    b += box(px, py + ph + 24, pw, rh, stroke=LGRAY, fill=GRAY_FILL, sw=1.0,
             dash=True)
    b += txt(px + pw / 2, py + ph + 49, "수조 · 실제 폰툰과 터그", size=11,
             fill="#4a4a4a")

    ay = y0 + 62
    for k, (xa, xb, mid, what) in enumerate(
            [(xs[0] + bw, xs[1], "52501", "상태 9개"),
             (xs[1] + bw, xs[2], "52503", "τ 6성분")]):
        cx = (xa + xb) / 2
        b += line(xa, ay, xb, ay, stroke=NAVY, sw=1.4, head=True,
                  marker="ahn")
        b += num(cx, ay - 28, k + 1)
        b += mono(cx, ay + 22, mid, size=11, fill=NAVY, weight="bold")
        b += txt(cx, ay + 38, what, size=10.5, fill="#4a5a6a")

    fy = 400
    rx = xs[2] + bw / 2
    b += path([(rx, ry + rh), (rx, fy), (px + pw - 90, fy),
               (px + pw - 90, py)], stroke=NAVY, sw=1.4, head=True,
              marker="ahn")
    b += num(rx, 336, 3)
    b += txt(rx + 20, 340, "추력 6성분 (터그별 X, Y, N)", size=10.5,
             anchor="start", fill=NAVY)

    lx = xs[0] + bw / 2
    b += path([(px + 90, py), (px + 90, fy), (lx, fy), (lx, ry + rh)],
              stroke=NAVY, sw=1.4, head=True, marker="ahn")
    b += num(lx, 336, 4)
    b += txt(lx - 20, 340, "상태 (x, y, ψ, u, v, r)", size=10.5, anchor="end",
             fill=NAVY)

    b += legend(36, 630, 520, [
        ("box", "본 과제 산출물 — 하네스 노드 (실물 코드와 동일)"),
        ("dbox", "하네스가 대신하는 실물 구성요소"),
        ("arrow", "MAVLink v2 over UDP. 숫자는 1 주기 안의 순서"),
    ])
    b += note(596, 630, 548, "읽는 법",
              ["1 → 2 → 3 → 4 가 한 바퀴 = 0.02 s (50 Hz)",
               "정책만 5 바퀴에 한 번 갱신됨 (그림 4)",
               "점선 상자를 실물로 바꿔 끼우면 그대로 수조시험이 됨",
               "위 3 블록의 코드는 바뀌지 않음 (docs/FILE_SWAP.md)"],
              accent_last=True)

    write("diagram_overview.svg", svg(W, H, b))


# -- 그림 2. 제어 1스텝 타이밍 ----------------------------------------------
def diagram_tick():
    """50 Hz 물리 5틱 = 제어 1스텝. 1 주기만 그리고 반복은 말로 적는다."""
    W, H = 1120, 520
    b = ""

    x0, dx, n = 196, 168, 5
    y_top, rh, gap = 76, 50, 8

    rows = [
        ("상태 송신", "50 Hz · node_b_mothership", True,
         ["측정", "측정", "측정", "측정", "측정"]),
        ("정책 (ONNX)", "10 Hz · nuc/onnx_policy", False,
         ["추론 → τ_d", "보류", "보류", "보류", "보류"]),
        ("배분 QP", "50 Hz · nuc/controller", True,
         ["solve", "solve", "solve", "solve", "solve"]),
        ("τ 송신", "50 Hz · node_d_esp32", True,
         ["52503", "52503", "52503", "52503", "52503"]),
        ("플랜트 적분", "50 Hz · node_a_plant", True,
         ["RK4", "RK4", "RK4", "RK4", "RK4"]),
    ]
    bottom = y_top + len(rows) * (rh + gap) - gap

    for i in range(n + 1):
        x = x0 + i * dx
        acc = (i % 5 == 0)
        b += line(x, y_top - 16, x, bottom + 6,
                  stroke=(NAVY if acc else "#dcdcdc"), sw=1.0,
                  dash=True if acc else None)
        b += mono(x, y_top - 24, "%.2f s" % (i * 0.02), size=10.5,
                  fill=(NAVY if acc else "#8c8c8c"))

    for k, (name, sub, every, cells) in enumerate(rows):
        y = y_top + k * (rh + gap)
        b += txt(184, y + 22, name, size=12, anchor="end", fill=NAVY,
                 weight="bold")
        b += mono(184, y + 39, sub, size=9.5, anchor="end", fill="#8c8c8c")
        for i, c in enumerate(cells):
            x = x0 + i * dx
            if every or i == 0:
                b += box(x + 8, y + 7, dx - 16, rh - 14, stroke=NAVY,
                         fill=NAVY_FILL, sw=1.3)
                b += txt(x + dx / 2, y + 29, c, size=11, fill=NAVY,
                         weight="bold")
            else:
                b += box(x + 8, y + 7, dx - 16, rh - 14, stroke=LGRAY,
                         fill=GRAY_FILL, sw=1.0, dash=True)
                b += txt(x + dx / 2, y + 26, c, size=10.5, fill="#8c8c8c")
                b += txt(x + dx / 2, y + 39, "직전 τ_d 유지", size=9.5,
                         fill="#8c8c8c")

    yb = bottom + 18
    b += path([(x0, yb + 12), (x0, yb), (x0 + n * dx, yb),
               (x0 + n * dx, yb + 12)], stroke=NAVY, sw=1.3)
    b += txt(x0 + n * dx / 2, yb + 32,
             "제어 1스텝 = 물리 5틱 = 0.10 s — 이 주기가 에피소드당 600 번 반복됨",
             size=11.5, fill=NAVY)

    b += legend(36, 430, 500, [
        ("box", "그 틱에 실제로 계산이 도는 행"),
        ("dbox", "계산하지 않고 직전 출력을 유지함 (sample-and-hold)"),
    ])
    b += note(560, 430, 524, "QP 를 10 Hz 가 아니라 50 Hz 로 도는 이유",
              ["실물 할당 노드가 50 Hz 이기 때문임",
               "τ_d 는 10 Hz 로만 바뀌고 solve 는 결정론이므로",
               "수치결과는 10 Hz 로 돌릴 때와 같음"])

    write("diagram_tick.svg", svg(W, H, b))


# -- 그림 3. 관측 조립 · 정책 · 추력배분 ------------------------------------
def diagram_policy():
    """19차 관측이 어디서 오는지를 출처별로 보인다 (외부 9 / 설정 4 / 내부 8)."""
    W, H = 1180, 1000
    b = ""
    LT, RT = 22, 1152

    gy, gh = 60, 118
    groups = [
        (60, 460, 290, "외부 입력 — 9 개", NAVY, NAVY_FILL,
         [("x, y, ψ", "폰툰 자세 [m, rad]"),
          ("u, v, r", "바디 속도 [m/s, rad/s]"),
          ("x_ref, y_ref, ψ_ref", "목표자세")],
         "마더십이 52501 로 보냄 — 밖에서 줘야 하는 값은 이것뿐임"),
        (556, 220, 666, "설정 상수 — 4 개", GRAY, BOX_FILL,
         [("r1 (2)", "터그1 부착위치"),
          ("r2 (2)", "터그2 부착위치")],
         "sc3.yaml 의 attach — 매 스텝 같은 값"),
        (812, 308, 966, "제어기 내부 — 8 개", LGRAY, GRAY_FILL,
         [("a_prev (3)", "직전 행동"),
          ("res (3)", "QP 잔차"),
          ("marg (2)", "래치 여유")],
         "직전 스텝이 남긴 값 — reset() 이 0·0·1 로 되돌림"),
    ]
    for gx, gw, gc, title, st, fl, items, nt in groups:
        b += box(gx, gy, gw, gh, stroke=st, fill=fl,
                 sw=1.4 if st == NAVY else 1.0,
                 dash=True if st == LGRAY else None)
        b += txt(gc, gy + 22, title, size=12.5,
                 fill=st if st != LGRAY else "#5a5a5a", weight="bold")
        iw = (gw - 20) / len(items)
        for j, (nm, desc) in enumerate(items):
            cx = gx + 10 + iw * (j + 0.5)
            b += mono(cx, gy + 52, nm, size=11.5, fill="#243447",
                      weight="bold")
            b += txt(cx, gy + 70, desc, size=10, fill="#5a5a5a")
        b += txt(gc, gy + 102, nt, size=10.5, fill="#4a4a4a")

    by, bx, bw = 216, 180, 820
    for _, _, gc, _, _, _, _, _ in groups:
        b += line(gc, gy + gh, gc, by, stroke=NAVY, sw=1.3, head=True,
                  marker="ahn")
    b += blk(bx, by, bw, 72, "build_obs()  —  nuc/controller.py:126",
             ["월드 위치오차를 폰툰 바디좌표로 회전 + 방위오차를 cos·sin 으로 폄"],
             rate="10 Hz")
    b += num(bx - 16, by + 36, 1)

    sy, cw, sx = 332, 55, 58
    cells = ([("eb_x", 0), ("eb_y", 0), ("cos", 0), ("sin", 0),
              ("u", 0), ("v", 0), ("r", 0)]
             + [("a_prev", 2)] * 3 + [("res", 2)] * 3 + [("marg", 2)] * 2
             + [("r1x", 1), ("r1y", 1), ("r2x", 1), ("r2y", 1)])
    b += line(590, by + 72, 590, sy, stroke=NAVY, sw=1.4, head=True,
              marker="ahn")
    for i, (nm, kind) in enumerate(cells):
        x = sx + i * cw
        st, fl, dash = ((NAVY, NAVY_FILL, None) if kind == 0 else
                        (GRAY, BOX_FILL, None) if kind == 1 else
                        (LGRAY, GRAY_FILL, True))
        b += box(x, sy, cw, 44, stroke=st, fill=fl,
                 sw=1.3 if kind == 0 else 1.0, dash=dash)
        b += mono(x + cw / 2, sy + 19, nm, size=9.5,
                  fill="#243447" if kind != 2 else "#6b6b6b")
        b += mono(x + cw / 2, sy + 36, str(i), size=10,
                  fill=NAVY if kind == 0 else "#8c8c8c", weight="bold")
    b += txt(sx + 19 * cw / 2, sy + 64,
             "관측 19 차 — 칸 안 아래 숫자는 색인 "
             "(VecNormalize 미사용. 배포에 동반 파일이 없음)",
             size=11.5, fill="#4a4a4a")

    py = 426
    b += line(590, sy + 72, 590, py, stroke=NAVY, sw=1.4, head=True,
              marker="ahn")
    b += blk(330, py, 520, 78, "ONNX 정책",
             ["19 → 128 → 128 → 3, tanh · opset 17 · 78 867 B"],
             ["out/ppo_sc3_final.onnx"], rate="10 Hz")
    b += num(314, py + 39, 2)

    ay, aw = 534, 380
    b += line(590, py + 78, 590, ay, stroke=NAVY, sw=1.4, head=True,
              marker="ahn")
    b += box(590 - aw / 2, ay, aw, 54, stroke=GRAY, fill=BOX_FILL, sw=1.2)
    b += txt(590, ay + 22, "행동 3 차 → 폰툰 목표력", size=12.5,
             weight="bold")
    b += mono(590, ay + 41, "tau_d = clip(a, -1, 1) * [60, 60, 60]", size=11,
              fill="#4a4a4a")
    b += num(590 - aw / 2 - 16, ay + 27, 3)

    qy, qh = 630, 112
    b += path([(590 + aw / 2, ay + 27), (910, ay + 27), (910, qy)],
              stroke=NAVY, sw=1.4, head=True, marker="ahn")
    b += blk(700, qy, 420, qh, "Latch-aware QP",
             ["제약 22 행 = 추진기 포화 + 전자석 3종",
              "(인장 F_t · 전단 F_s · 박리 M_max)",
              "RL 로 대체하지 않음 — 이탈을 막는 하드 보장임"],
             ["pontoon_rl/env/latch_qp.py"], rate="50 Hz")
    b += num(684, qy + 24, 4)

    b += box(60, qy, 420, qh, stroke=GRAY, fill=BOX_FILL, sw=1.2)
    b += txt(270, qy + 26, "플랜트", size=13, weight="bold")
    b += mono(270, qy + 50, "u6 = 터그 2대 x (X, Y, N)", size=11,
              fill="#4a4a4a")
    b += txt(270, qy + 74, "폰툰·터그 합동운동 적분 (RK4 ×5)", size=11,
             fill="#333333")
    b += txt(270, qy + 97, "실물에서는 수조의 실제 폰툰", size=10.5,
             fill="#6b6b6b")
    b += line(700, qy + qh / 2, 480, qy + qh / 2, stroke=NAVY, sw=1.4,
              head=True, marker="ahn")
    b += num(590, qy + qh / 2, 5)

    b += line(1120, qy + qh / 2, RT, qy + qh / 2, stroke=LGRAY, sw=1.0,
              dash=True)
    b += line(850, py + 39, RT, py + 39, stroke=LGRAY, sw=1.0, dash=True)
    b += path([(RT, qy + qh / 2), (RT, 32), (966, 32), (966, gy)],
              stroke=LGRAY, sw=1.0, head=True, dash=True)
    b += txt(1146, 580, "res · marg", size=10, anchor="end", fill="#6b6b6b")
    b += txt(1146, 410, "a_prev", size=10, anchor="end", fill="#6b6b6b")
    b += path([(60, qy + qh / 2), (LT, qy + qh / 2), (LT, 32), (290, 32),
               (290, gy)], stroke=LGRAY, sw=1.0, head=True, dash=True)
    b += txt(34, 560, "x, y, ψ, u, v, r", size=10, anchor="start",
             fill="#6b6b6b")

    ly = 790
    b += legend(60, ly, 520, [
        ("box", "52501 로 들어온 9 개에서 나온 성분 (색인 0–6)"),
        ("gbox", "설정 상수 (색인 15–18)"),
        ("dbox", "제어기가 직전 스텝에 만든 값 (색인 7–14)"),
        ("darrow", "되먹임. 만드는 블록에서 출발함"),
    ])
    b += note(620, ly, 500, "핵심",
              ["19 개를 사람이 채우는 것이 아님",
               "밖에서 줄 값은 9 개, 나머지 10 개는 상수와 제어기 자기 상태임",
               "a_prev 를 관측에 넣으므로 정책은 상태를 가짐",
               "→ 자율모드 진입마다 reset() 필요 (docs/RL_SPEC.md §2.3)",
               "보상은 학습에서만 계산됨 — 배포 코드에는 없음"],
              accent_last=True)

    write("diagram_policy.svg", svg(W, H, b))


# -- 그림 4. MAVLink v2 프레임과 메시지 3종 ----------------------------------
def diagram_frame():
    W, H = 1120, 690
    b = ""

    cells = [("STX", "0xFD", 70, False),
             ("len", "1 B", 56, False),
             ("incompat", "1 B", 74, False),
             ("compat", "1 B", 66, False),
             ("seq", "1 B", 56, False),
             ("sysid", "1 B", 64, False),
             ("compid", "1 B", 70, False),
             ("msgid", "3 B LE", 84, True),
             ("payload", "len B", 300, True),
             ("crc", "2 B LE", 70, False)]
    x, y, h = 36, 44, 54
    spans = {}
    for nm, sub, w, acc in cells:
        st = NAVY if acc else GRAY
        fl = NAVY_FILL if acc else BOX_FILL
        b += box(x, y, w, h, stroke=st, fill=fl, sw=1.4 if acc else 1.2)
        b += mono(x + w / 2, y + 23, nm, size=11.5,
                  fill=NAVY if acc else "#1a1a1a", weight="bold")
        b += txt(x + w / 2, y + 41, sub, size=10.5, fill="#5a5a5a")
        spans[nm] = (x, w)
        x += w

    x1 = spans["len"][0]
    x2 = spans["payload"][0] + spans["payload"][1]
    by = y + h + 14
    b += path([(x1, by + 10), (x1, by), (x2, by), (x2, by + 10)], stroke=GRAY,
              sw=1.2)
    b += txt((x1 + x2) / 2, by + 28,
             "CRC-16/MCRF4XX 계산 범위 (+ crc_extra 1 B 추가 투입)",
             size=11, fill="#4a4a4a")
    b += txt(W - 36, by + 28, "payload 뒤쪽 0 바이트는 잘려 전송됨",
             size=10.5, anchor="end", fill="#6b6b6b")

    msgs = [("52501", "MOTHERSHIP_STATE", True,
             "마더십 → NUC", "50 Hz", "<Q9f3B", "47 B", "119",
             "RL 관측용 상태 9 개 + 수동개입 플래그 3"),
            ("52502", "MOTHERSHIP_DATA", False,
             "마더십 → NUC", "50 Hz", "<Qfff", "20 B", "123",
             "기존 LOS/PID 가 만든 τ. 그대로 둠"),
            ("52503", "TUG_COMMAND", False,
             "NUC → 터그 ×2", "50 Hz", "<Qdddddd", "56 B", "16",
             "터그 2대 몫을 한 메시지에 실음 (float64)")]
    ry0, rh = 190, 62
    for lx, lb in ((52, "메시지"), (330, "방향 · 주기"),
                   (530, "payload 형식"), (720, "내용")):
        b += txt(lx, ry0 - 12, lb, size=10.5, anchor="start", fill="#8c8c8c")
    for k, (mid, nm, acc, dirn, rate, fmt, nb, crc, desc) in enumerate(msgs):
        ry = ry0 + k * (rh + 8)
        b += box(36, ry, W - 72, rh, stroke=NAVY if acc else GRAY,
                 fill=NAVY_FILL if acc else BOX_FILL, sw=1.4 if acc else 1.2)
        b += mono(52, ry + 25, mid, size=13, anchor="start", fill=NAVY,
                  weight="bold")
        b += txt(52, ry + 45, nm, size=10.5, anchor="start", fill="#4a5a6a")
        b += txt(176, ry + 25, "신설" if acc else "기존", size=10.5,
                 anchor="start", fill=NAVY if acc else "#8c8c8c",
                 weight="bold")
        b += txt(330, ry + 25, dirn, size=11.5, anchor="start", fill="#1a1a1a")
        b += txt(330, ry + 45, rate, size=10.5, anchor="start", fill="#6b6b6b")
        b += mono(530, ry + 25, fmt, size=11.5, anchor="start",
                  fill="#243447", weight="bold")
        b += mono(530, ry + 45, "%s · crc_extra %s" % (nb, crc), size=10,
                  anchor="start", fill="#6b6b6b")
        b += txt(720, ry + 33, desc, size=11, anchor="start", fill="#333333")

    my = 432
    b += txt(36, my - 12, "52501 payload 47 B 배치 (리틀엔디언)", size=11.5,
             anchor="start", fill=NAVY, weight="bold")
    fields = [("timestamp", 8, 2), ("x", 4, 0), ("y", 4, 0), ("psi", 4, 0),
              ("u", 4, 0), ("v", 4, 0), ("r", 4, 0), ("x_ref", 4, 1),
              ("y_ref", 4, 1), ("psi_ref", 4, 1), ("scen", 1, 3),
              ("rc", 1, 3), ("on", 1, 3)]
    total = sum(f[1] for f in fields)
    bx, bw2, bh2, off = 36, W - 72, 52, 0
    for nm, nb, kind in fields:
        w = bw2 * nb / total
        st, fl, dash = ((NAVY, NAVY_FILL, None) if kind == 0 else
                        (NAVY, PALE, None) if kind == 1 else
                        (GRAY, BOX_FILL, None) if kind == 2 else
                        (LGRAY, GRAY_FILL, True))
        b += box(bx, my, w, bh2, stroke=st, fill=fl, sw=1.2, dash=dash)
        b += mono(bx + w / 2, my + 22, nm, size=10 if nb >= 4 else 8.5,
                  fill="#243447")
        b += mono(bx + w / 2, my + 40, "%d" % off, size=9, fill="#8c8c8c")
        bx += w
        off += nb
    b += txt(W - 36, my + bh2 + 18, "칸 안 아래 숫자 = 바이트 오프셋", size=10,
             anchor="end", fill="#8c8c8c")

    ly = 536
    b += legend(36, ly, 520, [
        ("box", "RL 이 관측으로 쓰는 상태 6 (x, y, psi, u, v, r)"),
        ("pbox", "목표자세 3 (x_ref, y_ref, psi_ref) — set_goal() 로 들어감"),
        ("gbox", "타임스탬프 (uint64, µs)"),
        ("dbox", "수동개입 플래그 3 (scenario_mode · rc_on_off · on_off)"),
    ], title="52501 필드 구분")
    b += note(580, ly, 504, "52502 를 고치지 않고 52501 을 새로 만든 이유",
              ["52502 를 늘리면 마더십 수동제어·기존 배분 경로가 깨짐",
               "메시지 추가는 하위호환이고 52502·52503 의 crc_extra 가",
               "불변이므로 구 펌웨어와 섞여도 기존 두 메시지는 통함",
               "터그는 52503 만 받음 → 터그 Pixhawk 는 변경 없음"],
              accent_last=True)

    write("diagram_frame.svg", svg(W, H, b))


# -- 그림 5. 부착 기하 ------------------------------------------------------
def diagram_geometry():
    """폰툰 바디좌표계와 터그 부착점 r1 · r2 (평면도).

    화면 위 = 선수(+x), 화면 오른쪽 = 우현(+y) → 좌현은 화면 왼쪽이고 y 가 음수다.
    값은 `pontoon_rl/params/sc3.yaml` 의 `attach` 와 `config/geometry.yaml` 기본값.
    """
    W, H = 1120, 620
    b = ""
    S = 130.0                     # px / m
    cx, cy = 400.0, 300.0
    LX, BY = 2.60, 2.15
    hl, hb = LX / 2 * S, BY / 2 * S
    TL, TB = 1.10, 0.53
    R1 = (0.4894, -1.08)
    R2 = (-0.6106, -1.08)

    def sx(y_m):                  # 바디 y(우현 +) → 화면 x
        return cx + y_m * S

    def sy(x_m):                  # 바디 x(선수 +) → 화면 y (화면은 아래가 +)
        return cy - x_m * S

    # 폰툰
    b += box(sx(-BY / 2), sy(LX / 2), 2 * hb, 2 * hl, stroke=NAVY,
             fill=NAVY_FILL, sw=1.4)
    b += txt(cx, sy(-LX / 2) - 20, "폰툰", size=13, fill=NAVY, weight="bold")
    b += mono(cx, sy(-LX / 2) - 4, "2.60 m (x) x 2.15 m (y)", size=10.5,
              fill="#4a5a6a")

    # 중심선 (점선)
    b += line(cx, sy(LX / 2), cx, sy(-LX / 2), stroke=LGRAY, sw=1.0, dash=True)
    b += line(sx(-BY / 2), cy, sx(BY / 2), cy, stroke=LGRAY, sw=1.0, dash=True)

    # 좌표축
    b += line(cx, cy, cx, sy(1.02), stroke=NAVY, sw=1.6, head=True,
              marker="ahn")
    b += line(cx, cy, sx(0.86), cy, stroke=NAVY, sw=1.6, head=True,
              marker="ahn")
    b += txt(cx + 16, sy(1.02) + 4, "+x  선수 (bow)", size=12, anchor="start",
             fill=NAVY, weight="bold")
    b += txt(sx(0.86) - 6, cy - 12, "+y  우현 (starboard)", size=12,
             anchor="end", fill=NAVY, weight="bold")
    b += circ(cx, cy, 5, stroke=NAVY, fill=NAVY)
    b += txt(cx + 14, cy + 20, "원점 = 기하중심", size=11, anchor="start",
             fill="#333333")

    # 회전 + 방향 (위에서 봤을 때 시계방향 = +x → +y)
    ar = 62.0
    b += path([(cx, cy - ar), (cx + ar * 0.72, cy - ar * 0.72),
               (cx + ar, cy)], stroke=GRAY, sw=1.3, head=True)
    b += txt(cx + ar * 0.80, cy - ar * 0.86, "회전 +", size=11, anchor="start",
             fill="#4a4a4a")

    # 터그 2 대 (좌현 바깥쪽. 선체 외형은 개략)
    tw, th = TB * S, TL * S
    tx0 = sx(-BY / 2) - tw
    for nm, rr, lab in (("T1", R1, "터그 1 (선수쪽)"), ("T2", R2, "터그 2 (선미쪽)")):
        ty0 = sy(rr[0]) - th / 2
        b += box(tx0, ty0, tw, th, stroke=GRAY, fill=GRAY_FILL, sw=1.2,
                 dash=True)
        b += mono(tx0 + tw / 2, ty0 + th / 2 + 5, nm, size=13, fill="#3a3a3a",
                  weight="bold")
        b += txt(tx0 - 14, ty0 + th / 2 - 8, lab, size=11.5, anchor="end",
                 fill="#333333")
        b += mono(tx0 - 14, ty0 + th / 2 + 10,
                  "r%s = (%+0.4f, %+0.2f) m" % (nm[1], rr[0], rr[1]),
                  size=11, anchor="end", fill=NAVY, weight="bold")
        # 부착점
        b += circ(sx(rr[1]), sy(rr[0]), 5, stroke=NAVY, fill="#ffffff", sw=1.6)
        b += line(tx0 - 10, ty0 + th / 2, sx(rr[1]) - 6, sy(rr[0]),
                  stroke=LGRAY, sw=1.0)

    # 간격 치수 (1.10 m)
    dxx = sx(-BY / 2) + 34
    b += line(dxx, sy(R1[0]), dxx, sy(R2[0]), stroke=GRAY, sw=1.0)
    b += line(dxx - 5, sy(R1[0]), dxx + 5, sy(R1[0]), stroke=GRAY, sw=1.0)
    b += line(dxx - 5, sy(R2[0]), dxx + 5, sy(R2[0]), stroke=GRAY, sw=1.0)
    b += mono(dxx + 10, (sy(R1[0]) + sy(R2[0])) / 2 + 4, "1.10 m", size=11,
              anchor="start", fill="#4a4a4a")
    b += txt(dxx + 10, (sy(R1[0]) + sy(R2[0])) / 2 + 20, "= 터그 전장", size=10,
             anchor="start", fill="#8c8c8c")

    # 반폭 치수
    dyy = sy(-1.02)
    b += line(cx, dyy, sx(-BY / 2), dyy, stroke=GRAY, sw=1.0)
    b += line(cx, dyy - 5, cx, dyy + 5, stroke=GRAY, sw=1.0)
    b += line(sx(-BY / 2), dyy - 5, sx(-BY / 2), dyy + 5, stroke=GRAY, sw=1.0)
    b += mono((cx + sx(-BY / 2)) / 2, dyy - 8, "1.075 m", size=10.5,
              fill="#4a4a4a")
    b += txt((cx + sx(-BY / 2)) / 2, dyy + 16, "반폭", size=10, fill="#8c8c8c")

    # 오른쪽: 재는 방법
    b += note(620, 44, 464, "재는 방법", [
        "1. 폰툰 네 변의 중점을 찍어 기하중심을 바닥에 표시",
        "2. 선수-선미 축(+x)에 줄자를 깔고 수직 거리를 측정",
        "3. 터그를 붙인 상태에서 터그 선체 중심까지 (전후, 좌우)",
        "4. 소수 2 자리(0.01 m)까지. 그 이상은 의미 없음",
        "좌현은 y 가 음수다 — 부호를 빼면 전단 방향이 뒤집힌다",
    ], accent_last=True)

    # 오른쪽: yaml
    yb, ybh = 196, 136
    b += box(620, yb, 464, ybh, stroke=NAVY, fill=PALE, sw=1.2)
    b += txt(634, yb + 21, "config/geometry.yaml — 이 파일만 고친다", size=11.5,
             anchor="start", fill=NAVY, weight="bold")
    for i, ln in enumerate(["frame: pontoon_body_frd",
                            "attach:",
                            "  r1: [ 0.4894, -1.08]   # 선수쪽",
                            "  r2: [-0.6106, -1.08]   # 선미쪽",
                            "dr:",
                            "  r_jit: 0.15            # 측정 불확실도"]):
        b += mono(634, yb + 44 + i * 15.5, ln, size=10.5, anchor="start",
                  fill="#243447")

    b += legend(620, 350, 464, [
        ("box", "폰툰 (바디좌표계 기준물체)"),
        ("dbox", "터그 선체 외형 — 개략. 치수는 1.10 x 0.53 m"),
        ("arrow", "좌표축 · 회전 + 방향 (FRD)"),
    ], title="범례")

    b += note(620, 466, 464, "확인 필요", [
        "r1 · r2 는 sc3.yaml 에서 터그 선체 원점으로 정의됨",
        "전자석 패드 중심이 선체 중심에서 떨어져 있으면 보정 필요",
        "y = -1.08 m 는 반폭 1.075 m 보다 0.005 m 바깥 — 재측정 대상",
    ])

    write("diagram_geometry.svg", svg(W, H, b))


def main():
    print("SVG 생성 -> %s" % OUT)
    diagram_overview()
    diagram_tick()
    diagram_policy()
    diagram_frame()
    diagram_geometry()


if __name__ == "__main__":
    main()
