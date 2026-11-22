# -*- coding: utf-8 -*-
"""통신 열화(지연·패킷손실) 스윕 — 하네스가 실물 열화에 얼마나 버티나.

왜 하는가
---------
하네스의 기본값은 "지연 0 / 손실 0" 이다(과제 지시). 실물 Wi-Fi 는 그렇지
않다. 정책을 **재학습하지 않고(zero-shot)** 지연·손실만 주입해 어디서 깨지는지
측정해 둔다. 이것이 실물 전환 전 위험 목록이다.

★ 지표 선택 (중요 — 처음에 틀렸던 부분)
---------------------------------------
**종단 위치오차만 보면 안 된다.** 60 s 정지정렬 과제라 제어기가 끝에서
정착해 버려, 상태지연 1~5 틱에서 종단오차가 소수 4자리까지 공칭과 같게
나온다. 그런데 **궤적 중간은 최대 4.7e-2 m 까지 달라진다**(진단:
`diag_delay`). 즉 종단오차는 열화에 둔감한 지표다.

따라서 과도구간 지표를 함께 본다:
* `IAE`  = Σ|위치오차|·dt  [m·s] — 수렴 경로 전체의 품질
* `t_정착` = 위치오차가 끝까지 0.05 m 미만으로 남는 첫 시각 [s]
* `상태결손` = 정책 틱에서 상태를 못 받아 **직전 행동을 유지**한 횟수

1 틱 = 물리 20 ms. 제어주기 100 ms = 5 틱.
"""
from __future__ import annotations

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from _console import utf8_stdout                         # noqa: E402
from run_harness import POS_TOL_OK, YAW_TOL_OK, Harness   # noqa: E402

N_EP = 10
SEED0 = 1000
SETTLE_TOL = 0.05          # m

CASES = [
    ("공칭 (지연 0 / 손실 0)",             dict()),
    ("state 1틱 (20 ms)",                  dict(delay_state=1)),
    ("state 2틱 (40 ms)",                  dict(delay_state=2)),
    ("state 5틱 (100 ms = 제어 1주기)",    dict(delay_state=5)),
    ("state 10틱 (200 ms)",                dict(delay_state=10)),
    ("state 25틱 (500 ms)",                dict(delay_state=25)),
    ("cmd 2틱 (40 ms)",                    dict(delay_cmd=2)),
    ("cmd 5틱 (100 ms)",                   dict(delay_cmd=5)),
    ("cmd 25틱 (500 ms)",                  dict(delay_cmd=25)),
    ("왕복 2+2틱 (80 ms RTT)",             dict(delay_state=2, delay_cmd=2)),
    ("왕복 10+10틱 (400 ms RTT)",          dict(delay_state=10, delay_cmd=10)),
    ("손실 1 %",                           dict(drop=0.01)),
    ("손실 5 %",                           dict(drop=0.05)),
    ("손실 20 %",                          dict(drop=0.20)),
    ("손실 50 %",                          dict(drop=0.50)),
    ("손실 90 %",                           dict(drop=0.90)),
    ("왕복 2+2틱 + 손실 5 %",              dict(delay_state=2, delay_cmd=2, drop=0.05)),
    ("PWM 양자화 ON (확인 필요)",          dict(pwm_quantize=True)),
]


def _traj_metrics(r, dt):
    """궤적에서 과도구간 지표를 뽑는다. s = [u,v,r, x,y,psi]."""
    eta = r["S"][:, 3:5]
    e = np.linalg.norm(r["goal"][:2][None, :] - eta, axis=1)
    iae = float(e.sum() * dt)
    bad = np.nonzero(e >= SETTLE_TOL)[0]
    k = (int(bad[-1]) + 1) if len(bad) else 0
    t_set = float(k * dt) if k < len(e) else float("nan")   # nan = 끝까지 못 정착
    return iae, t_set, float(e.max())


def run_case(kw):
    h = Harness(transport="inproc", dr=True, link_seed=7, **kw)
    dps, dys, iaes, tss, oks, terms = [], [], [], [], 0, 0
    for e in range(N_EP):
        r = h.episode(SEED0 + e)
        ok = (r["dpos"] < POS_TOL_OK) and (r["dyaw"] < YAW_TOL_OK) and not r["term"]
        oks += int(ok)
        terms += int(r["term"])
        dps.append(r["dpos"])
        dys.append(r["dyaw"])
        iae, ts, _pk = _traj_metrics(r, h.dt_ctrl)
        iaes.append(iae)
        tss.append(ts)
    return dict(ok=oks, term=terms,
                dpos=np.asarray(dps), dyaw=np.asarray(dys),
                iae=np.asarray(iaes), tset=np.asarray(tss),
                miss=int(h.C.n_miss))


def main():
    utf8_stdout()
    rows = []
    base_iae = None
    for name, kw in CASES:
        m = run_case(kw)
        iae = float(m["iae"].mean())
        if base_iae is None:
            base_iae = iae
        ts = m["tset"]
        ts_txt = ("%.1f" % np.nanmean(ts)) if np.isfinite(ts).any() else "미정착"
        rows.append((name, m["ok"], float(m["dpos"].mean()), iae,
                     iae / base_iae, ts_txt, m["miss"], m["term"]))
        print("  %-32s %2d/%d  종단 %.4f m  IAE %6.2f (x%.2f)  정착 %5s s  결손 %d"
              % (name, m["ok"], N_EP, rows[-1][2], iae, rows[-1][4], ts_txt, m["miss"]))

    md = os.path.join(_HERE, "out", "SWEEP_COMM.md")
    with open(md, "w", encoding="utf-8") as f:
        f.write("# 통신 열화 스윕 — harness_sitl (zero-shot, 재학습 없음)\n\n")
        f.write("## 1. 조건\n\n")
        f.write("- 정책: `pontoon_rl/out/n1/ppo_sc3_final.zip` → ONNX. **재학습하지 않았다.**\n")
        f.write("- 에피소드 %d개 (시드 %d~%d), 도메인 랜덤화 ON, 링크 시드 7.\n"
                % (N_EP, SEED0, SEED0 + N_EP - 1))
        f.write("- 1 틱 = 물리 20 ms. 제어주기 = 5 틱 = 100 ms. 에피소드 60 s.\n")
        f.write("- 지연은 `/pontoon/state`(상행) 과 `/tug_command`(하행) 에 건다.\n")
        f.write("- 손실은 두 토픽에 동시에 건다. 손실 시 수신측은 **직전 값을 유지**\n")
        f.write("  한다(sample-and-hold = 실물 Pixhawk 거동, 수신 타임아웃 없음).\n\n")
        f.write("## 2. ★ 지표를 종단오차만 쓰면 안 된다\n\n")
        f.write("- 60 s 정지정렬 과제라 제어기가 끝에서 정착해, **상태지연 1~5 틱에서\n")
        f.write("  종단오차가 공칭과 소수 4자리까지 같게 나온다.**\n")
        f.write("- 그러나 같은 조건에서 **궤적 중간은 최대 4.7e-2 m 까지 다르다**\n")
        f.write("  (시드 1000 단일 에피소드 직접 대조).\n")
        f.write("- → 종단오차는 이 과제에서 **열화에 둔감한 지표**다. 아래 표는\n")
        f.write("  `IAE = Σ|위치오차|·dt` 와 정착시간을 주 지표로 쓴다.\n\n")
        f.write("## 3. 결과\n\n")
        f.write("| 조건 | 성공 | 종단오차 [m] | IAE [m·s] | IAE 배수 | 정착 [s] | 상태결손 | 조기종료 |\n")
        f.write("|---|---|---|---|---|---|---|---|\n")
        for n, ok, dp, iae, rel, ts, miss, tm in rows:
            f.write("| %s | %d/%d | %.4f | %.2f | %.2f | %s | %d | %d |\n"
                    % (n, ok, N_EP, dp, iae, rel, ts, miss, tm))
        f.write("\n- `상태결손` = 정책 틱에서 `/pontoon/state` 를 못 받아 **직전 행동을\n")
        f.write("  유지한 횟수** (10 에피소드 합계, 총 정책틱 = 6000).\n")
        f.write("- `정착` = 위치오차가 끝까지 %.2f m 미만으로 남는 첫 시각의 평균.\n"
                % SETTLE_TOL)
        f.write("\n> PWM 양자화 행은 **확인 필요** — 실측 PWM↔추력 곡선이 없어\n")
        f.write("> 선형 µs↔N 가정으로 계산한 값이다 (`plant/tug_lower.py`).\n")
        f.write("> 실측 곡선이 나오면 이 행만 다시 돌리면 된다.\n")
    print("\n기록: %s" % md)


if __name__ == "__main__":
    main()
