# -*- coding: utf-8 -*-
"""SITL 궤적 npz 를 mp4 로 가시화한다.

- 입력: `sitl/run_sitl.py --traj out/traj_sitl.npz` 가 저장한 npz
  (`S` 플랜트 상태 · `A` 정책 행동 · `U` 터그 6 성분 · `M` 래치 여유 ·
   `C` 와이어 카운터 · `L` 마감시한 지연 · `r1`/`r2`/`goal`/`dt`)
- 출력: mp4 (ffmpeg). 좌측 장면 + 우측 3 패널 + 상단 HUD
- `pontoon_rl/` 을 수정하지 않는다. 본 파일은 하네스 전용임

실행:
    ..\\plan_pontoon_rl\\.venv\\Scripts\\python.exe viz\\make_video_sitl.py
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

PONT_L, PONT_B = 2.60, 2.15      # params/sc3.yaml  pontoon.Lx / By
TUG_L, TUG_B = 1.10, 0.53        # params/sc3.yaml  tug.L / B
POS_TOL, YAW_TOL = 0.30, 5.0     # m, deg           env.pos_tol / yaw_tol

NAVY = "#1f3a5f"
PALE = "#e9eef5"
GREY = "#8a8f98"

for _f in ("Malgun Gothic", "NanumGothic", "Noto Sans CJK KR"):
    if any(_f == f.name for f in matplotlib.font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _f
        break
plt.rcParams["axes.unicode_minus"] = False


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rot(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s], [s, c]])


def rect_ned(cx, cy, psi, L, B):
    """NED 중심(x=북, y=동)·선수각 psi 인 직사각형 → 플롯좌표(동, 북) 꼭짓점."""
    half = np.array([[+L / 2, -B / 2], [+L / 2, +B / 2],
                     [-L / 2, +B / 2], [-L / 2, -B / 2]])
    R = rot(psi)
    pts = (R @ half.T).T + np.array([cx, cy])
    return np.column_stack([pts[:, 1], pts[:, 0]])   # (east, north)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", default=os.path.join(ROOT, "out", "traj_sitl.npz"))
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "sitl_sc3.mp4"))
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--stride", type=int, default=2,
                    help="물리틱 간격. 50 Hz 기록이므로 2 면 25 fps 실시간")
    ap.add_argument("--stills", default="",
                    help="정지화면 모드. 초 단위 시각을 쉼표로 (예 2.4,8,60)")
    ap.add_argument("--stills-out",
                    default=os.path.join(ROOT, "docs", "img",
                                         "video_frames.png"),
                    help="정지화면을 세로로 이어붙인 PNG 경로")
    a = ap.parse_args(argv)

    ff = shutil.which("ffmpeg")
    if ff is None:
        print("[FAIL] ffmpeg 를 PATH 에서 찾지 못했다")
        return 1
    plt.rcParams["animation.ffmpeg_path"] = ff

    if not os.path.isfile(a.traj):
        print("[FAIL] 궤적 파일이 없다: %s" % a.traj)
        print("       sitl\\run_sitl.py --traj out\\traj_sitl.npz 로 먼저 기록할 것")
        return 1

    z = np.load(a.traj)
    S, U, M, C, L = z["S"], z["U"], z["M"], z["C"], z["L"]
    r1, r2, goal, dt = z["r1"], z["r2"], z["goal"], float(z["dt"])
    seed = int(z["seed"]) if "seed" in z else -1
    n = len(S)
    nL = len(L)

    t = np.arange(n) * dt
    ep = np.linalg.norm(S[:, 3:5] - goal[:2], axis=1)
    ey = np.degrees(wrap(S[:, 5] - goal[2]))
    mg = M.min(axis=1)
    lag = np.concatenate([L, np.full(max(0, n - nL), np.nan)])[:n]

    idx = list(range(0, n, max(1, a.stride)))
    if idx[-1] != n - 1:
        idx.append(n - 1)

    ok = (ep[-1] < POS_TOL) and (abs(ey[-1]) < YAW_TOL)

    fig = plt.figure(figsize=(16, 9), dpi=120)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.35, 1.0],
                          left=0.05, right=0.97, top=0.88, bottom=0.07,
                          hspace=0.45, wspace=0.22)
    axs = fig.add_subplot(gs[:, 0])
    ax1 = fig.add_subplot(gs[0, 1])
    ax2 = fig.add_subplot(gs[1, 1])
    ax3 = fig.add_subplot(gs[2, 1])

    # ---- 장면 ----------------------------------------------------------
    lim = max(4.0, np.abs(S[:, 3:5]).max() * 1.35 + 1.5)
    axs.set_xlim(-lim, lim)
    axs.set_ylim(-lim, lim)
    axs.set_aspect("equal")
    axs.set_xlabel("동 E [m]")
    axs.set_ylabel("북 N [m]")
    axs.grid(True, color="#dde3ea", lw=0.8)

    # 목표 — ㄷ 형 도킹 스테이션(개념) + 허용원
    gpts = rect_ned(goal[0], goal[1], goal[2], PONT_L, PONT_B)
    axs.add_patch(Polygon(gpts, closed=True, fill=False, ec=NAVY,
                          ls="--", lw=1.4, zorder=2))
    axs.add_patch(Circle((goal[1], goal[0]), POS_TOL, fill=False,
                         ec=NAVY, ls=":", lw=1.2, zorder=2))
    for sgn in (+1, -1):
        w = rect_ned(goal[0] + sgn * (PONT_L / 2 + 0.45), goal[1], goal[2],
                     0.35, PONT_B + 1.2)
        axs.add_patch(Polygon(w, closed=True, fc="#f2f4f7", ec=GREY, lw=1.0,
                              zorder=1))
    wb = rect_ned(goal[0], goal[1] + (PONT_B / 2 + 0.80), goal[2],
                  PONT_L + 1.3, 0.35)
    axs.add_patch(Polygon(wb, closed=True, fc="#f2f4f7", ec=GREY, lw=1.0,
                          zorder=1))

    (trail,) = axs.plot([], [], color=NAVY, lw=1.0, alpha=0.5, zorder=3)
    pont = Polygon(rect_ned(0, 0, 0, PONT_L, PONT_B), closed=True,
                   fc=PALE, ec=NAVY, lw=1.6, zorder=4)
    axs.add_patch(pont)
    tug1 = Polygon(rect_ned(0, 0, 0, TUG_L, TUG_B), closed=True,
                   fc="#ffffff", ec="#b4261a", lw=1.4, zorder=5)
    tug2 = Polygon(rect_ned(0, 0, 0, TUG_L, TUG_B), closed=True,
                   fc="#ffffff", ec="#b4261a", lw=1.4, zorder=5)
    axs.add_patch(tug1)
    axs.add_patch(tug2)
    (bow,) = axs.plot([], [], color=NAVY, lw=2.0, zorder=6)
    q1 = axs.quiver([0], [0], [0], [0], color="#b4261a", scale=120,
                    width=0.005, zorder=7)
    q2 = axs.quiver([0], [0], [0], [0], color="#b4261a", scale=120,
                    width=0.005, zorder=7)
    axs.plot([], [], color=NAVY, ls="--", lw=1.4, label="목표 자세 · 도킹부")
    axs.plot([], [], color=NAVY, lw=1.6, label="폰툰 (2.60 × 2.15 m)")
    axs.plot([], [], color="#b4261a", lw=1.4, label="무인터그 ×2 · 합력")
    axs.legend(loc="upper left", fontsize=9, framealpha=0.9)

    # ---- 패널 ----------------------------------------------------------
    sh1, sh2 = np.abs(U[:, 0]), np.abs(U[:, 3])
    for ax, ylab in ((ax1, "위치오차 |e| [m]"), (ax2, "방위오차 ψ [deg]"),
                     (ax3, "전자석 전단력 |X| [N]")):
        ax.set_xlim(0, t[-1])
        ax.grid(True, color="#dde3ea", lw=0.8)
        ax.set_ylabel(ylab, fontsize=10)
    ax3.set_xlabel("시간 [s]")

    ax1.set_ylim(0, max(0.5, ep.max() * 1.1))
    ax1.axhline(POS_TOL, color="#b4261a", ls="--", lw=1.0)
    ax1.text(t[-1] * 0.99, POS_TOL, " 허용 0.30 m", color="#b4261a",
             fontsize=9, ha="right", va="bottom")
    ax2.set_ylim(min(-8, ey.min() * 1.2), max(8, ey.max() * 1.2))
    for v in (+YAW_TOL, -YAW_TOL):
        ax2.axhline(v, color="#b4261a", ls="--", lw=1.0)
    ax3.set_ylim(0, max(45.0, max(sh1.max(), sh2.max()) * 1.15))
    ax3.axhline(40.0, color="#b4261a", ls="--", lw=1.0)
    ax3.text(t[-1] * 0.99, 40.0, " 설정 한계 F_s 40 N (추정)", color="#b4261a",
             fontsize=9, ha="right", va="bottom")
    ax3.axhline(18.0, color=GREY, ls=":", lw=1.2)
    ax3.text(t[-1] * 0.99, 18.0, " 유일 실측치 18 N (PWM 1700 µs)", color=GREY,
             fontsize=9, ha="right", va="top")

    (l1,) = ax1.plot([], [], color=NAVY, lw=1.4)
    (l2,) = ax2.plot([], [], color=NAVY, lw=1.4)
    (l3,) = ax3.plot([], [], color=NAVY, lw=1.4, label="터그 1")
    (l3b,) = ax3.plot([], [], color="#b4261a", lw=1.2, label="터그 2")
    ax3.legend(loc="center right", fontsize=9, framealpha=0.9)

    fig.text(0.05, 0.955,
             "시나리오 3 — 무인터그 2 대 좌현 결합 폰툰 정밀 정렬 (SITL · MAVLink over UDP)",
             fontsize=15, color=NAVY, weight="bold")
    hud = fig.text(0.05, 0.915, "", fontsize=11)
    badge = fig.text(0.97, 0.955, "", fontsize=14, weight="bold",
                     ha="right", color="#1a7f37" if ok else "#b4261a")

    def draw(k):
        x, y, psi = S[k, 3], S[k, 4], S[k, 5]
        pont.set_xy(rect_ned(x, y, psi, PONT_L, PONT_B))
        R = rot(psi)
        p1 = np.array([x, y]) + R @ r1
        p2 = np.array([x, y]) + R @ r2
        tug1.set_xy(rect_ned(p1[0], p1[1], psi, TUG_L, TUG_B))
        tug2.set_xy(rect_ned(p2[0], p2[1], psi, TUG_L, TUG_B))
        nose = np.array([x, y]) + R @ np.array([PONT_L / 2 + 0.45, 0.0])
        bow.set_data([y, nose[1]], [x, nose[0]])
        f1 = R @ U[k, 0:2]
        f2 = R @ U[k, 3:5]
        q1.set_offsets([[p1[1], p1[0]]])
        q1.set_UVC([f1[1]], [f1[0]])
        q2.set_offsets([[p2[1], p2[0]]])
        q2.set_UVC([f2[1]], [f2[0]])
        trail.set_data(S[:k + 1, 4], S[:k + 1, 3])

        l1.set_data(t[:k + 1], ep[:k + 1])
        l2.set_data(t[:k + 1], ey[:k + 1])
        l3.set_data(t[:k + 1], sh1[:k + 1])
        l3b.set_data(t[:k + 1], sh2[:k + 1])

        c = C[k]
        lg = lag[k]
        hud.set_text(
            "t %.2f s    위치오차 %.3f m    방위오차 %.3f deg    "
            "래치여유 %.3f\n"
            "UDP  상태수신 %d · τ수신 %d · 정책발행 %d · QP발행 %d    "
            "마감지연 %.1f ms"
            % (t[k], ep[k], ey[k], mg[k],
               c[0], c[1], c[2], c[3], 0.0 if np.isnan(lg) else lg))
        badge.set_text("PASS" if ok else "FAIL")

    # ---- 정지화면 모드 (보고서 그림용) ---------------------------------
    if a.stills:
        from PIL import Image

        secs = [float(s) for s in a.stills.split(",") if s.strip()]
        tiles = []
        for s in secs:
            k = min(n - 1, max(0, int(round(s / dt))))
            draw(k)
            p = os.path.join(ROOT, "out", "vframes", "s%06d.png" % k)
            os.makedirs(os.path.dirname(p), exist_ok=True)
            fig.savefig(p, dpi=100, facecolor="white")
            tiles.append(p)
            print("  정지화면 t=%.2f s (k=%d) → %s" % (t[k], k, p))
        ims = [Image.open(p).convert("RGB") for p in tiles]
        w = min(i.width for i in ims)
        ims = [i if i.width == w else
               i.resize((w, round(i.height * w / i.width)), Image.LANCZOS)
               for i in ims]
        comb = Image.new("RGB", (w, sum(i.height for i in ims)), "white")
        yy = 0
        for i in ims:
            comb.paste(i, (0, yy))
            yy += i.height
        comb.save(a.stills_out)
        print("완료: %s (%d×%d, %d 장)"
              % (a.stills_out, comb.width, comb.height, len(ims)))
        return 0

    wr = FFMpegWriter(fps=a.fps, bitrate=6000, codec="libx264",
                      metadata={"title": "harness_sitl scenario 3"})
    print("렌더링: %d 프레임 → %s" % (len(idx), a.out))
    with wr.saving(fig, a.out, dpi=120):
        for f, k in enumerate(idx):
            draw(k)
            wr.grab_frame()
            if f % 100 == 0:
                sys.stdout.write("  %d/%d\n" % (f, len(idx)))
                sys.stdout.flush()

    sz = os.path.getsize(a.out)
    print("완료: %s (%.2f MB, seed %d, %d 프레임)"
          % (a.out, sz / 1e6, seed, len(idx)))
    print("최종  |e| %.4f m   psi %.4f deg   래치여유 최소 %.4f   판정 %s"
          % (ep[-1], ey[-1], mg.min(), "PASS" if ok else "FAIL"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
