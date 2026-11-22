# -*- coding: utf-8 -*-
"""harness_sitl/nuc/px4_state.py — PX4 표준 상태로부터 외부입력 9 개를 조립한다.

- 전송(ROS·MAVLink·DDS)을 모르는 **순수 함수와 상태기**만 둔다. `nuc/` 규율.
- 참조 구현: 캡스톤 UGV `auto_usv_pkg/coordinate_transformer_node.py`
  (LLA→NED 행렬 66–70 · `lla_to_ned` 102–113 · NED→body 속도 변환 183–196).
- 근거·대조표: `docs/INPUT_PATHS.md`.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

# WGS84
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)

DEFAULT_STALE_S = 0.5          # tug_tx.input_timeout_s 와 같은 값


# ---------------------------------------------------------------- 순수 함수
def wrap_pi(a: float) -> float:
    """(-pi, pi] 로 접는다. `nuc/controller.py` 의 wrap 과 같은 식."""
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q) -> float:
    """PX4 쿼터니언(**w, x, y, z** 순서)에서 NED yaw(rad).

    `VehicleAttitude.q` 는 (w,x,y,z) 다 — UGV 노드가 scipy 에 넘길 때
    `[q[1],q[2],q[3],q[0]]` 로 재배열하는 이유(coordinate_transformer_node.py:188).
    """
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if not (n > 1e-6) or not all(math.isfinite(v) for v in (w, x, y, z)):
        raise ValueError("quaternion invalid")
    w, x, y, z = w / n, x / n, y / n, z / n
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def body_from_ned(vx: float, vy: float, psi: float) -> Tuple[float, float]:
    """NED 속도 → 선체고정 (u, v). 평면 3-DOF 이므로 2×2 회전이면 된다.

    UGV 는 3축 쿼터니언 역회전을 쓴다(coordinate_transformer_node.py:188–190).
    roll·pitch = 0 인 평면에서 두 결과는 같다 — `tests/test_px4_state.py` 가 잰다.
    """
    c, s = math.cos(float(psi)), math.sin(float(psi))
    u = c * float(vx) + s * float(vy)
    v = -s * float(vx) + c * float(vy)
    return u, v


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    la, lo = math.radians(float(lat_deg)), math.radians(float(lon_deg))
    h = float(alt_m)
    sla, cla = math.sin(la), math.cos(la)
    slo, clo = math.sin(lo), math.cos(lo)
    n = _A / math.sqrt(1.0 - _E2 * sla * sla)
    return np.array([(n + h) * cla * clo,
                     (n + h) * cla * slo,
                     (n * (1.0 - _E2) + h) * sla], dtype=np.float64)


def ecef_to_ned_matrix(lat0_deg: float, lon0_deg: float) -> np.ndarray:
    """UGV 노드가 손으로 쓴 행렬과 **같다** (coordinate_transformer_node.py:66–70)."""
    la, lo = math.radians(float(lat0_deg)), math.radians(float(lon0_deg))
    sla, cla = math.sin(la), math.cos(la)
    slo, clo = math.sin(lo), math.cos(lo)
    return np.array([[-sla * clo, -sla * slo,  cla],
                     [-slo,        clo,        0.0],
                     [-cla * clo, -cla * slo, -sla]], dtype=np.float64)


class LlaOrigin(object):
    """LLA 기준점. `GLOBAL_POSITION_INT` 만 들어오는 경우의 대체경로용."""

    def __init__(self, lat_deg: float, lon_deg: float, alt_m: float) -> None:
        self.lat = float(lat_deg)
        self.lon = float(lon_deg)
        self.alt = float(alt_m)
        self._p0 = lla_to_ecef(self.lat, self.lon, self.alt)
        self._R = ecef_to_ned_matrix(self.lat, self.lon)

    def to_ned(self, lat_deg: float, lon_deg: float,
               alt_m: Optional[float] = None) -> np.ndarray:
        h = self.alt if alt_m is None else float(alt_m)
        return self._R @ (lla_to_ecef(lat_deg, lon_deg, h) - self._p0)


# ------------------------------------------------------------------ 상태기
class ExternalState(object):
    """정책이 요구하는 **외부입력 9 개** + 수동개입 2 개."""

    __slots__ = ("t_us", "x", "y", "psi", "u", "v", "r",
                 "x_ref", "y_ref", "psi_ref", "rc_on_off", "on_off",
                 "psi_ref_src")

    def __init__(self, t_us, x, y, psi, u, v, r, x_ref, y_ref, psi_ref,
                 rc_on_off=0, on_off=1, psi_ref_src="target") -> None:
        self.t_us = int(t_us)
        self.x, self.y, self.psi = float(x), float(y), float(psi)
        self.u, self.v, self.r = float(u), float(v), float(r)
        self.x_ref, self.y_ref = float(x_ref), float(y_ref)
        self.psi_ref = float(psi_ref)
        self.rc_on_off, self.on_off = int(rc_on_off), int(on_off)
        self.psi_ref_src = str(psi_ref_src)

    def eta(self) -> np.ndarray:
        return np.array([self.x, self.y, self.psi], dtype=np.float64)

    def nu(self) -> np.ndarray:
        return np.array([self.u, self.v, self.r], dtype=np.float64)

    def goal(self) -> Tuple[float, float, float]:
        return (self.x_ref, self.y_ref, self.psi_ref)

    def finite(self) -> bool:
        return all(math.isfinite(v) for v in
                   (self.x, self.y, self.psi, self.u, self.v, self.r,
                    self.x_ref, self.y_ref, self.psi_ref))

    def as_tuple9(self) -> Tuple[float, ...]:
        return (self.x, self.y, self.psi, self.u, self.v, self.r,
                self.x_ref, self.y_ref, self.psi_ref)


class Px4StateAssembler(object):
    """PX4 표준 메시지 조각 → `ExternalState`.

    - 조각마다 **수신시각을 따로** 들고, 하나라도 `stale_s` 를 넘기면 `None` 을
      돌려준다 → 상위 노드는 발행을 멈추고 `tug_tx` 의 τ=0 이 나간다.
    - `psi_ref` 는 ① 설정점 메시지의 yaw, ② (없으면) 목표점 방위각(LOS) 중
      하나다. 어느 쪽을 썼는지 `psi_ref_src` 에 남긴다 — 보고에서 숨기지 않는다.
    """

    def __init__(self, stale_s: float = DEFAULT_STALE_S,
                 origin: Optional[LlaOrigin] = None,
                 psi_ref_from_los: bool = False) -> None:
        self.stale_s = float(stale_s)
        self.origin = origin
        self.psi_ref_from_los = bool(psi_ref_from_los)

        self._att: Optional[Tuple[float, float, float]] = None   # (t, psi, r)
        self._pos: Optional[Tuple[float, float, float, float, float]] = None
        #                      (t, x, y, vx, vy)
        self._ref: Optional[Tuple[float, float, float, Optional[float]]] = None
        #                      (t, x_ref, y_ref, psi_ref|None)
        self._t_us = 0
        self.rc_on_off = 0
        self.on_off = 1
        self.n_att = 0
        self.n_pos = 0
        self.n_ref = 0
        self.n_reject = 0

    # -- 조각 투입 ---------------------------------------------------------
    def on_attitude(self, t_mono: float, yaw: float, yawspeed: float,
                    t_us: Optional[int] = None) -> None:
        """`ATTITUDE` (또는 `VehicleAttitude`+`VehicleAngularVelocity`).

        `yawspeed` 는 **선체고정 z 각속도** = Fossen 3-DOF 의 `r` 이다.
        """
        if not (math.isfinite(yaw) and math.isfinite(yawspeed)):
            self.n_reject += 1
            return
        self._att = (float(t_mono), wrap_pi(yaw), float(yawspeed))
        self.n_att += 1
        if t_us is not None:
            self._t_us = int(t_us)

    def on_attitude_quat(self, t_mono: float, q, yawspeed: float,
                         t_us: Optional[int] = None) -> None:
        """`VehicleAttitude.q`(w,x,y,z) 경로. 잘못된 쿼터니언은 **직전 값 유지**.

        UGV 노드의 `_is_valid_quaternion` 과 같은 판정이다
        (shape 4 · 유한 · norm > 1e-6; coordinate_transformer_node.py:119–131).
        """
        try:
            yaw = quat_to_yaw(q)
        except (ValueError, TypeError):
            self.n_reject += 1
            return
        self.on_attitude(t_mono, yaw, yawspeed, t_us=t_us)

    def on_local_position(self, t_mono: float, x: float, y: float,
                          vx: float, vy: float,
                          t_us: Optional[int] = None) -> None:
        """`LOCAL_POSITION_NED` / `VehicleLocalPosition`. 속도는 **NED**."""
        if not all(math.isfinite(v) for v in (x, y, vx, vy)):
            self.n_reject += 1
            return
        self._pos = (float(t_mono), float(x), float(y), float(vx), float(vy))
        self.n_pos += 1
        if t_us is not None:
            self._t_us = int(t_us)

    def on_global_position(self, t_mono: float, lat_deg: float, lon_deg: float,
                           alt_m: Optional[float], vx: float, vy: float,
                           t_us: Optional[int] = None) -> None:
        """`GLOBAL_POSITION_INT` 대체경로. `origin` 이 없으면 **버린다**."""
        if self.origin is None:
            self.n_reject += 1
            return
        if not all(math.isfinite(v) for v in (lat_deg, lon_deg, vx, vy)):
            self.n_reject += 1
            return
        ned = self.origin.to_ned(lat_deg, lon_deg, alt_m)
        self.on_local_position(t_mono, ned[0], ned[1], vx, vy, t_us=t_us)

    def on_position_target(self, t_mono: float, x: float, y: float,
                           yaw: Optional[float] = None,
                           t_us: Optional[int] = None) -> None:
        """`POSITION_TARGET_LOCAL_NED` / `PositionSetpointTriplet.current`."""
        if not (math.isfinite(x) and math.isfinite(y)):
            self.n_reject += 1
            return
        py = None
        if yaw is not None and math.isfinite(yaw):
            py = wrap_pi(yaw)
        self._ref = (float(t_mono), float(x), float(y), py)
        self.n_ref += 1
        if t_us is not None:
            self._t_us = int(t_us)

    def on_override(self, rc_on_off: int, on_off: int) -> None:
        self.rc_on_off = int(rc_on_off)
        self.on_off = int(on_off)

    # -- 조립 --------------------------------------------------------------
    def missing(self, t_mono: float) -> Tuple[str, ...]:
        out = []
        for name, slot in (("attitude", self._att), ("position", self._pos),
                           ("reference", self._ref)):
            if slot is None:
                out.append(name)
            elif t_mono - slot[0] > self.stale_s:
                out.append(name + ":stale")
        return tuple(out)

    def state(self, t_mono: float) -> Optional[ExternalState]:
        if self.missing(t_mono):
            return None
        _, psi, r = self._att
        _, x, y, vx, vy = self._pos
        _, xr, yr, pr = self._ref
        u, v = body_from_ned(vx, vy, psi)

        src = "target"
        if pr is None:
            if not self.psi_ref_from_los:
                return None
            pr = math.atan2(yr - y, xr - x)      # 목표점 방위각 (NED)
            src = "los"
        st = ExternalState(self._t_us, x, y, psi, u, v, r, xr, yr, pr,
                           rc_on_off=self.rc_on_off, on_off=self.on_off,
                           psi_ref_src=src)
        return st if st.finite() else None

    def stats(self) -> dict:
        return {"n_att": self.n_att, "n_pos": self.n_pos, "n_ref": self.n_ref,
                "n_reject": self.n_reject}
