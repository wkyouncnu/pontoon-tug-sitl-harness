# -*- coding: utf-8 -*-
"""harness_sitl/nuc/controller.py — 마더십 NUC 상위 제어기 (**production**).

이 파일의 지위
--------------
**실물 NUC 에서 그대로 실행되는 파일이다.** 하네스도 이 파일을 import 하고,
실물 ROS 2 노드도 이 파일을 import 한다. 한 글자도 다르지 않다.
→ `harness_sitl/docs/FILE_SWAP.md` 의 "그대로 가는 파일" 표.

그래서 이 파일은 **전송계층을 모른다**
--------------------------------------
* 토픽·소켓·MAVLink·ROS 2 를 import 하지 않는다.
* 입력은 평범한 numpy 배열(`nu`, `eta`, `r1`, `r2`)이고 출력도 평범한 배열
  (`tau_d`, `u6`)이다. 메시지 객체를 받지 않는다.
* 전송계층(토픽 구독·발행, MAVLink 프레이밍)은 **호출자**의 일이다.
  하네스에서는 `harness_sitl/nodes/node_c_nuc.py`, 실물에서는 ROS 2 노드.

의존성
------
`numpy` + `onnxruntime` + `quadprog`(`pontoon_rl/env/latch_qp.py` 경유) 뿐.
`stable_baselines3`·`torch` 는 import 하지 않는다 — 실물 NUC 에 설치하지 않는
것이 목적이다 (재학습 = ONNX 파일 교체).

제어 구조 (사용자 확정)
-----------------------
    상태 (50 Hz) ─ 관측 19차 ─ ONNX 정책 (10 Hz) ─ τ_d = a ⊙ τ_max
                                       └─ Latch-aware QP (50 Hz) ─ u6 (터그 2대)

* **RL 은 상위 2층만 맡는다.** Latch-aware QP 는 RL 로 대체하지 않고
  `pontoon_rl/env/latch_qp.py::LatchQP` 를 **그대로 import** 한다.
* QP 를 50 Hz 로 도는 이유: 실물 할당 노드가 50 Hz 다. `τ_d` 는 10 Hz 로만
  바뀌고 `LatchQP.solve` 는 결정론이므로 **수치결과는 10 Hz 와 같다.**

관측 19차 (`pontoon_rl/env/pontoon_env.py::PontoonSc3Env._obs()` 와 동일)
-------------------------------------------------------------------------
    [0:2]   목표까지의 위치오차 (바디좌표)      eb
    [2:4]   cos(ey), sin(ey)                   요오차
    [4:7]   nu = (u, v, r)
    [7:10]  a_prev                             이전 행동
    [10:13] res / tau_max                      QP 가 못 낸 잔차 (정규화)
    [13:15] marg                               터그별 최소 여유
    [15:17] r1                                 터그1 부착위치
    [17:19] r2                                 터그2 부착위치

* `a_prev`, `res`, `marg` 는 **플랜트 상태가 아니라 이 제어기의 내부 상태**다
  (정책·QP 의 출력). `reset()` 에서 `0, 0, ones(2)` 로 둔다 — `env.reset()` 동일.
* QP 는 **공칭 기하**(`sc3.yaml` 의 `attach.r1/r2`)로 만들고, 관측의 `r1/r2`
  는 호출자가 준 값이다. 학습 때와 같은 불일치를 의도적으로 재현한다.

수동개입 (`rc_on_off`)
----------------------
`rc_on_off = 1` 이면 **정책 출력을 버리고 τ=0 을 낸다.** 실물에서 조종기
개입이 걸렸을 때 RL 이 끼어들지 않게 하는 경로이고, 같은 코드가 하네스에서도
시험된다 (`tests/test_same_code_both_paths.py`).

⚠️ 확인 필요
------------
* 폰툰 `I_z` — `sc3.yaml` 147 kg·m² vs `CLAUDE.md` §9 457~914 kg·m² (3~6배).
  같은 물체인지 미확인. 이 제어기는 `sc3.yaml` 값을 쓴다 (QP 는 `I_z` 를 쓰지
  않으므로 직접 영향은 없다).
* 목표자세의 출처 — 실물에서는 `MOTHERSHIP_STATE`(52501) 의
  `x_ref/y_ref/psi_ref` 를 `set_goal()` 로 넣는다. 그 값이 **NED 절대좌표인지
  상대좌표인지 미확인** (`PROTOCOL_esp32_nuc.md` 확인필요 ③).
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np

# `pontoon_rl` 은 실물 NUC 에도 함께 배치한다 (`latch_qp.py`·`sc3.yaml` 때문).
_HARNESS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(_HARNESS)
for _p in (os.path.join(_ROOT, "pontoon_rl"),):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from env.latch_qp import LatchQP                        # noqa: E402
from env.pontoon_env import wrap                        # noqa: E402

from nuc.cfg import geometry_summary, load_cfg          # noqa: E402,F401
from nuc.onnx_policy import OnnxPolicy                  # noqa: E402

OBS_DIM = 19
ACT_DIM = 3


class NucController(object):
    """상위 제어기 본체. 전송계층을 모른다."""

    def __init__(self, onnx_path=None, cfg=None, goal=None, geometry=None):
        # 부착기하는 `config/geometry.yaml` 이 `sc3.yaml` 위에 덮어쓴다.
        # `geometry=False` 면 학습 당시 값 그대로 (재현시험용).
        self.c = load_cfg(cfg, geometry=geometry)
        e, t, la = self.c["env"], self.c["tug"], self.c["latch"]
        ca = self.c["attach"]
        self.tau_max = np.array(e["tau_max"], dtype=np.float64)
        self.qp = LatchQP(ca["r1"], ca["r2"],
                          d=t["d"], l_b=t["l_b"], f_max=t["f_max"], f_min=t["f_min"],
                          F_t=la["F_t"], F_s=la["F_s"], M_max=la["M_max"])
        self.policy = OnnxPolicy(onnx_path) if onnx_path else OnnxPolicy()
        self.goal = np.zeros(3) if goal is None else np.asarray(goal, dtype=np.float64)
        self.reset()

    # -- 수명주기 -----------------------------------------------------------
    def reset(self):
        self.a_prev = np.zeros(ACT_DIM)
        self.res = np.zeros(3)
        self.marg = np.ones(2)
        self.tau_d = np.zeros(3)
        self.u6 = np.zeros(6)
        self.last_obs = None
        self.n_policy = 0
        self.n_qp = 0
        self.qp_us = 0.0
        self.n_override = 0

    def set_goal(self, goal):
        """목표자세 갱신. 실물에서는 52501 의 `x_ref/y_ref/psi_ref` 가 들어온다.

        **하네스가 목표를 하드코딩하지 않는다** — 기존 LOS 제어기와 같은
        출처에서 받는다 (사용자 확정 지시).
        """
        self.goal = np.asarray(goal, dtype=np.float64)

    # -- 관측 조립 (env._obs() 와 동일) -------------------------------------
    def build_obs(self, nu, eta, r1, r2) -> np.ndarray:
        """19차 관측. 인자는 **평범한 배열**이다 (메시지 객체가 아니다).

        nu  : (3,) = (u, v, r)       바디 FRD
        eta : (3,) = (x, y, psi)     월드 NED
        r1  : (2,) 터그1 부착위치 (폰툰 바디좌표)
        r2  : (2,) 터그2 부착위치
        """
        nu = np.asarray(nu, dtype=np.float64)
        eta = np.asarray(eta, dtype=np.float64)
        ep = self.goal[:2] - eta[:2]
        cs, sn = np.cos(eta[2]), np.sin(eta[2])
        eb = np.array([cs * ep[0] + sn * ep[1], -sn * ep[0] + cs * ep[1]])
        ey = wrap(self.goal[2] - eta[2])
        return np.concatenate([eb, [np.cos(ey), np.sin(ey)], nu, self.a_prev,
                               self.res / self.tau_max, self.marg,
                               np.asarray(r1, dtype=np.float64),
                               np.asarray(r2, dtype=np.float64)])

    # -- 10 Hz: 정책 --------------------------------------------------------
    def policy_step(self, nu, eta, r1, r2, rc_on_off=0, on_off=1):
        """ONNX 정책 1회. 반환 `(tau_d (3,), a (3,))`.

        `rc_on_off = 1` 또는 `on_off = 0` 이면 **정책을 돌리지 않고 τ=0**.
        실물 수동개입·자율해제 경로이며 하네스에서 같은 코드로 시험된다.
        """
        if int(rc_on_off) != 0 or int(on_off) == 0:
            self.n_override += 1
            self.a_prev = np.zeros(ACT_DIM)
            self.tau_d = np.zeros(3)
            self.last_obs = None
            return self.tau_d, self.a_prev
        obs = self.build_obs(nu, eta, r1, r2)
        self.last_obs = obs
        a = self.policy.act(obs)
        self.tau_d = a * self.tau_max
        self.a_prev = a
        self.n_policy += 1
        return self.tau_d, a

    # -- 50 Hz: Latch-aware 추력배분 QP -------------------------------------
    def allocate(self, tau_d):
        """QP 1회. 반환 `(u6 (6,), tau_a (3,))`. `res`·`marg` 를 갱신한다.

        `tau_d` 를 **인자로 받는다** — 실물에서 rl_node 와 latch_qp_node 가
        분리된 두 프로세스이고 그 사이에 float32 토픽(`<Qfff`)이 있기 때문이다.
        양자화를 넣을지 말지는 호출자가 결정한다 (전송계층의 일).
        """
        tau_d = np.asarray(tau_d, dtype=np.float64)
        t0 = time.perf_counter()
        u6, tau_a = self.qp.solve(tau_d)
        self.qp_us = (time.perf_counter() - t0) * 1e6
        self.u6 = np.asarray(u6, dtype=np.float64)
        self.res = np.asarray(tau_a, dtype=np.float64) - tau_d
        self.marg = np.asarray(self.qp.margins(self.u6), dtype=np.float64)
        self.n_qp += 1
        return self.u6, np.asarray(tau_a, dtype=np.float64)

    # -- 진단 ---------------------------------------------------------------
    @property
    def saturated(self) -> int:
        return int(float(np.min(self.marg)) <= 1e-9)
