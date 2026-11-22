# -*- coding: utf-8 -*-
"""harness_sitl/nodes/node_a_plant.py — 노드 A: 플랜트 (실물 수조 대응).

역할
----
* 폰툰 + 터그 2대 **결합 운동**을 적분한다 (M2 하이브리드 UDE + 래치 결합).
* 입력: `/tug/thrust` (TugThrust) — 터그가 실제로 낸 일반화력 u6
* 출력: `/plant/state` (PontoonState) — 실물에서는 센서가 측정할 양

왜 `env.step()` 을 쓰지 않는가
------------------------------
`PontoonSc3Env.step()` 안에는 **Latch-aware QP 가 들어 있다.** 실물에서 QP 는
마더십 NUC(노드 C)에서 돌고 플랜트(수조)에는 없다. 그래서 노드 A 는
`env._rk4(s, u6, env.hsub)` 만 직접 호출해 **적분기만** 쓴다.
→ `pontoon_rl/` 은 **한 줄도 고치지 않는다** (사용자 상시 지시: 인터페이스만 맞춘다).

수치 동등성
-----------
`env.reset(seed=S)` 를 그대로 호출하므로 도메인 랜덤화 추출(질량·감쇠·모멘트암
지터·추력 스케일)과 초기상태 `s0` 가 `eval_policy.py` 와 **완전히 같다**.
한 틱 = RK4 서브스텝 1회 = `hsub` = `dt_ctrl / n_sub` = 0.02 s (50 Hz).

종료 판정
---------
`env.step()` 의 판정을 그대로 옮긴다 (제어스텝 경계에서만 검사):
    terminate : dpos > 12.0  또는  상태가 유한하지 않음
    truncate  : k >= N  (N = T_ep / dt_ctrl = 600)
"""
from __future__ import annotations

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, os.path.join(_ROOT, "pontoon_rl")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from iface.messages import PontoonState, TugThrust          # noqa: E402
from nodes.topics import TOPIC_PLANT, TOPIC_THRUST          # noqa: E402

from env.pontoon_env import PontoonSc3Env                   # noqa: E402
from nuc.cfg import load_cfg                                # noqa: E402


class PlantNode(object):
    """수조 대응. 틱마다 RK4 서브스텝 1회."""

    def __init__(self, transport, cfg=None, dr=True, seed=None, geometry=None):
        self.tp = transport
        # 제어기와 **같은 병합설정**을 쓴다 (`config/geometry.yaml`).
        # 플랜트만 학습값으로 남으면 기하를 고쳐도 시험이 거짓양성이 된다.
        self.env = PontoonSc3Env(cfg=load_cfg(cfg, geometry=geometry),
                                 dr=dr, seed=seed)
        self.dt_ctrl = float(self.env.dt)
        self.nsub = int(self.env.nsub)
        self.hsub = float(self.env.hsub)
        self.N = int(self.env.N)
        self.t_us = 0
        self.k_tick = 0
        self.u6 = np.zeros(6)

    # -- 수명주기 -----------------------------------------------------------
    def reset(self, seed=None):
        """`eval_policy.py` 와 같은 시드 규약. 초기 상태를 즉시 1회 발행한다."""
        self.env.reset(seed=seed)
        self.t_us = 0
        self.k_tick = 0
        self.u6 = np.zeros(6)
        self.publish()
        return self.state()

    # -- 입출력 -------------------------------------------------------------
    def state(self) -> PontoonState:
        return PontoonState.from_plant(self.t_us, self.env.s,
                                       self.env.r1, self.env.r2, valid=1)

    def publish(self) -> None:
        self.tp.publish(TOPIC_PLANT, self.state())

    def _latch_input(self) -> None:
        """`/tug/thrust` 의 최신값을 받아 들고 있는다 (sample-and-hold).

        아직 아무것도 안 왔으면 0 을 유지한다 — 실물에서도 터그가 명령을
        못 받은 구간은 추력 0 이다 (tug_tx.py 워치독: shutdown_zero_count).
        """
        m = self.tp.latest(TOPIC_THRUST)
        if isinstance(m, TugThrust):
            self.u6 = np.asarray(m.tau6(), dtype=np.float64)

    # -- 적분 ---------------------------------------------------------------
    def tick(self) -> PontoonState:
        """50 Hz 틱 1회: 입력 래치 → RK4 서브스텝 1회 → 상태 발행."""
        self._latch_input()
        self.env.s = self.env._rk4(self.env.s, self.u6, self.hsub)
        self.k_tick += 1
        self.t_us += int(round(self.hsub * 1e6))
        self.publish()
        return self.state()

    # -- 진단 (러너가 종료 판정에 쓴다) -------------------------------------
    def dpos(self) -> float:
        return float(np.linalg.norm(self.env.goal[:2] - self.env.s[3:5]))

    def dyaw(self) -> float:
        from env.pontoon_env import wrap
        return float(abs(wrap(self.env.goal[2] - self.env.s[5])))

    def finite(self) -> bool:
        return bool(np.all(np.isfinite(self.env.s)))

    def n_ctrl_done(self) -> int:
        return self.k_tick // self.nsub
