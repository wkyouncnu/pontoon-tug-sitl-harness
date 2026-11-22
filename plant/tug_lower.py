# -*- coding: utf-8 -*-
"""harness_sitl/plant/tug_lower.py — 터그 1대의 하위 추력배분 (**가상 전용**).

이 파일의 지위
--------------
**실물에서는 이 코드를 쓰지 않는다.** 실물에서 이 수식은 **터그 Pixhawk 안**
에서 돈다 (우리가 배포하지 않는다). 하네스가 그 거동을 모형으로 재현할 뿐이다.
→ `docs/FILE_SWAP.md` §2 "실물이 대신하는 파일" 표.

⚠️ 이 하위 역행렬은 **RL 로 대체하지 않는다** (사용자 확정). 정방행렬이고
`det = 2d ≠ 0` 이라 닫힌형 역해가 **수학적으로 정확**하기 때문이다. 근사로
바꿀 이유가 없다.

수식
----
터그 1대는 추진기 3기 (후미 2기 + 바우 레터럴 1기):

    B_F = [[ 1,  1,  0 ],        f   = B_F⁻¹ τ        (닫힌형, det = 2d)
           [ 0,  0,  1 ],        τ_a = B_F f
           [ d, -d, l_b]]

* `d   = 0.30 m` — 후미 추진기 반간격
* `l_b = 0.50 m` — 바우 레터럴 추진기의 CoG 전방거리
* 둘 다 `pontoon_rl/params/sc3.yaml` 의 `tug` 절에서 읽는다. 하드코딩하지 않는다.
* `det = 2d = 0.6` → 비축퇴. 역행렬 왕복오차 실측 **1.8e-15** (`out/test_suite.txt`).

포화
----
`f ∈ [f_min, f_max] = [-28.33, +35.85] N` 로 클립한다 (T200 역추력이 약한
비대칭을 반영).

**공칭에서 이 클립은 작동하지 않는다.** Latch-aware QP 가 이미 같은 부등식을
제약으로 걸고 있기 때문이다 (→ `τ_a == τ_cmd`). 지연·드랍을 주입해 QP 가 모르는
상태로 돌 때만 여기서 잘린다 → **그 차이 자체가 진단 지표**다
(`n_sat` 카운터, `tools_sweep_comm.py`).

⚠️ 확인 필요 — PWM 양자화
-------------------------
실물 ESP32/Pixhawk 는 힘이 아니라 **PWM(µs)** 을 다룬다. 그런데 이 터그의
**PWM ↔ 추력 곡선 실측치가 없다** (`tug1_pinn/export/` 전체에 PWM 산출물 없음).
따라서 PWM 양자화는 **옵션으로 구현하고 기본값 OFF** 다. 아래 `k` 는
`1100~1900 µs` 선형이라는 **가정**이고 실측이 아니다. 계류추력 시험으로 곡선을
계측하면 `quantize()` 를 실측 곡선으로 교체하면 된다.
"""
from __future__ import annotations

import numpy as np


def body_force_matrix(d: float, l_b: float) -> np.ndarray:
    """터그 1대의 추진기 → 일반화력 사상 `B_F` (3×3)."""
    return np.array([[1.0, 1.0, 0.0],
                     [0.0, 0.0, 1.0],
                     [float(d), -float(d), float(l_b)]], dtype=np.float64)


class TugLowerAllocator(object):
    """`τ (3,) → (f (3,), τ_actual (3,))`. 터그 1대분. 상태는 포화 카운터뿐이다."""

    def __init__(self, d: float, l_b: float, f_max: float, f_min: float,
                 pwm_quantize: bool = False, pwm_step_us: float = 1.0,
                 k_force_per_us=None):
        self.d = float(d)
        self.l_b = float(l_b)
        self.B = body_force_matrix(self.d, self.l_b)
        self.Binv = np.linalg.inv(self.B)
        self.f_max = float(f_max)
        self.f_min = float(f_min)
        # ⚠️ 확인 필요: 실측 PWM↔추력 곡선이 없다. 기본 OFF.
        self.pwm_quantize = bool(pwm_quantize)
        self.pwm_step_us = float(pwm_step_us)
        self.k = (float(k_force_per_us) if k_force_per_us is not None
                  else (self.f_max - self.f_min) / 800.0)   # 1100~1900 µs 선형 가정
        self.n_sat = 0
        self.n_calls = 0

    # -- 설정에서 만들기 ----------------------------------------------------
    @staticmethod
    def from_cfg(cfg: dict, **kw) -> "TugLowerAllocator":
        """`sc3.yaml` 의 `tug` 절에서 만든다. 기하를 하드코딩하지 않는다."""
        t = cfg["tug"]
        return TugLowerAllocator(d=t["d"], l_b=t["l_b"],
                                 f_max=t["f_max"], f_min=t["f_min"], **kw)

    def reset(self):
        self.n_sat = 0
        self.n_calls = 0

    # -- PWM 양자화 (옵션, 기본 OFF) ----------------------------------------
    def quantize(self, f: np.ndarray) -> np.ndarray:
        """⚠️ 선형 µs↔N **가정**. 실측 곡선으로 교체해야 한다."""
        step = self.k * self.pwm_step_us
        return np.round(np.asarray(f, dtype=np.float64) / step) * step

    # -- 본체 ---------------------------------------------------------------
    def allocate(self, tau):
        """`τ (3,) → (f (3,), τ_actual (3,))`.

        `τ_actual != τ` 가 되는 경우는 **포화(또는 PWM 양자화)뿐**이다.
        역행렬 자체는 정확하므로 포화가 없으면 `τ_actual == τ`.
        """
        f = self.Binv @ np.asarray(tau, dtype=np.float64)
        if self.pwm_quantize:
            f = self.quantize(f)
        fc = np.clip(f, self.f_min, self.f_max)
        if not np.allclose(fc, f):
            self.n_sat += 1
        self.n_calls += 1
        return fc, self.B @ fc

    def allocate_pair(self, tau_one, tau_two):
        """터그 2대분을 한 번에. 반환 `(f1, f2, τ_a1, τ_a2)`."""
        f1, ta1 = self.allocate(tau_one)
        f2, ta2 = self.allocate(tau_two)
        return f1, f2, ta1, ta2

    def __repr__(self):
        return ("TugLowerAllocator(d=%.3f, l_b=%.3f, f=[%.2f, %.2f], det=%.3f)"
                % (self.d, self.l_b, self.f_min, self.f_max, 2.0 * self.d))
