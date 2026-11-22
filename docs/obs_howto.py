# -*- coding: utf-8 -*-
"""관측 19차를 **무엇으로부터** 만드는지 보이는 최소 예제.

요지
----
19 개 값을 사람이 다 만들어 넣는 것이 아니다. **밖에서 들어와야 하는 수는 9 개**
(`x, y, psi, u, v, r` + `x_ref, y_ref, psi_ref`)이고, 부착위치 `r1`·`r2` 는
설정파일 상수, 나머지 8 개(`a_prev`·`res`·`marg`)는 **제어기가 스스로 들고 있는 값**이다.

실행:
    ..\\plan_pontoon_rl\\.venv\\Scripts\\python.exe -X utf8 docs\\obs_howto.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from nuc.controller import NucController  # noqa: E402

NAMES = (["eb_x", "eb_y", "cos_ey", "sin_ey", "u", "v", "r"]
         + ["a_prev_%d" % i for i in range(3)]
         + ["res_%d" % i for i in range(3)]
         + ["marg_1", "marg_2", "r1_x", "r1_y", "r2_x", "r2_y"])

# 출처: 밖(52501) / 설정 / 제어기 내부
SRC = (["52501"] * 7 + ["내부"] * 3 + ["내부(QP)"] * 3 + ["내부(QP)"] * 2
       + ["설정"] * 4)


def main():
    ctl = NucController()                       # params/sc3.yaml + ONNX 정책 적재

    # ── ① 밖에서 들어오는 9 개 수 (MOTHERSHIP_STATE 52501) ──────────────────
    x, y, psi = 2.0, -1.0, 0.20              # NED [m], [rad]
    u, v, r = 0.10, 0.02, -0.01              # 바디 FRD [m/s], [rad/s]
    x_ref, y_ref, psi_ref = 0.0, 0.0, 0.0    # 목표자세 (LOS 와 같은 출처)

    # ── ② 설정에서 오는 상수 4 개 (sc3.yaml: attach.r1 / attach.r2) ─────────
    r1 = ctl.c["attach"]["r1"]
    r2 = ctl.c["attach"]["r2"]

    # ── ③ 제어기가 스스로 들고 있는 8 개 (a_prev 3 · res 3 · marg 2) ────────
    #     reset() 직후 값이며 매 스텝 policy_step / allocate 가 갱신한다.
    ctl.set_goal([x_ref, y_ref, psi_ref])

    print("[1 스텝째] 밖에서 받은 9 개 + 설정 4 개로 19 차가 완성된다")
    obs = ctl.build_obs([u, v, r], [x, y, psi], r1, r2)
    show(obs)

    # 1 스텝 돌린다: 정책 → τ_d → QP. 여기서 a_prev·res·marg 가 채워진다.
    tau_d, a = ctl.policy_step([u, v, r], [x, y, psi], r1, r2)
    u6, tau_a = ctl.allocate(tau_d)
    print("")
    print("정책 출력 a      = %s" % np.array2string(a, precision=4))
    print("목표력  tau_d    = %s  [N, N, N·m]" % np.array2string(tau_d, precision=2))
    print("QP 실현 tau_a    = %s" % np.array2string(tau_a, precision=2))
    print("터그 6성분 u6    = %s" % np.array2string(u6, precision=2))

    print("")
    print("[2 스텝째] 같은 상태를 넣어도 내부 3 블록이 바뀌어 관측이 달라진다")
    obs2 = ctl.build_obs([u, v, r], [x, y, psi], r1, r2)
    show(obs2)
    print("")
    print("바뀐 성분: %s"
          % ", ".join(NAMES[i] for i in np.flatnonzero(obs != obs2)))


def show(o):
    print("%-4s %-10s %-10s %10s" % ("i", "이름", "출처", "값"))
    for i, (n, s, v) in enumerate(zip(NAMES, SRC, o)):
        print("%-4d %-10s %-10s %10.4f" % (i, n, s, v))


if __name__ == "__main__":
    main()
