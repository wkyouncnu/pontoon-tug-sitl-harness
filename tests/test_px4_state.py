# -*- coding: utf-8 -*-
"""PX4 표준 입력경로 시험 — 외부입력 9 개가 두 경로에서 같은가.

이 시험이 존재하는 이유
-----------------------
사용자 기준은 하나다: **실험할 때 시뮬레이션한 코드를 그대로 쓸 수 있는가.**
그래서 "같은 9 개 값이 나온다"를 수치로 닫아 둔다. 깨지면 두 경로 중
하나가 바뀐 것이다.

| 시험 | 닫는 질문 |
|---|---|
| `test_quat_to_yaw` | PX4 쿼터니언(w,x,y,z)에서 yaw 가 맞게 나오는가 |
| `test_quat_matches_rotation_matrix` | UGV 노드의 3축 역회전과 평면 2×2 가 같은가 |
| `test_body_ned_roundtrip` | NED↔바디 왕복이 항등인가 |
| `test_lla_ned_matrix_matches_ugv` | LLA→NED 행렬이 UGV 원문과 같은가 |
| `test_lla_ned_small_offset` | 100 m 변위가 NED 로 맞게 들어오는가 |
| `test_staleness_blocks_state` | 조각이 늦으면 `None` 이 나오는가 (τ=0 경로) |
| `test_rejects_nonfinite` | NaN/inf 가 상태로 새어 나가지 않는가 |
| `test_psi_ref_los_fallback` | 설정점 yaw 가 없을 때의 거동이 명시적인가 |
| `test_nine_values_parity` | **52501 경로와 PX4 표준 경로의 9 개 값이 같은가** |

참조 원문: `auto_usv_pkg/coordinate_transformer_node.py`
(행렬 66–70 · `lla_to_ned` 102–113 · 쿼터니언 검사 119–131 · 속도 183–196).
"""
from __future__ import annotations

import math

import numpy as np

import _common  # noqa: F401  (sys.path 설정)

from nuc.px4_state import (ExternalState, LlaOrigin, Px4StateAssembler,
                           body_from_ned, ecef_to_ned_matrix, lla_to_ecef,
                           quat_to_yaw, wrap_pi)

TOL = 1e-12
PARITY_TOL = 1e-9          # 두 경로 9 개 값의 허용차


def _quat_from_yaw(psi):
    """평면 회전(roll=pitch=0)의 PX4 순서 쿼터니언 (w,x,y,z)."""
    return [math.cos(psi / 2.0), 0.0, 0.0, math.sin(psi / 2.0)]


# ----------------------------------------------------------------- 자세
def test_quat_to_yaw():
    for psi in (-3.0, -1.5, -0.3, 0.0, 0.3, 1.5, 3.0):
        got = quat_to_yaw(_quat_from_yaw(psi))
        assert abs(wrap_pi(got - psi)) < 1e-12, (psi, got)
    # 잘못된 쿼터니언은 예외 (UGV 노드의 _is_valid_quaternion 과 같은 판정)
    for bad in ([0.0, 0.0, 0.0, 0.0], [float("nan"), 0, 0, 1], [1.0, 0.0]):
        try:
            quat_to_yaw(bad)
        except (ValueError, TypeError):
            continue
        raise AssertionError("잘못된 쿼터니언을 통과시켰다: %r" % (bad,))
    return "yaw 7점 오차 < 1e-12, 불량 3종 전부 거부"


def test_quat_matches_rotation_matrix():
    """UGV 는 3축 쿼터니언 **역회전**으로 바디속도를 만든다.

    roll = pitch = 0 평면에서 그 결과가 우리 2×2 회전과 같아야 한다.
    scipy 가 없으므로 쿼터니언 → 회전행렬을 직접 써서 비교한다.
    """
    worst = 0.0
    for psi in (-2.5, -0.7, 0.0, 0.7, 2.5):
        w, x, y, z = _quat_from_yaw(psi)
        # NED→body = R(q)^T (UGV 의 apply(..., inverse=True) 와 같다)
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)
        for vx, vy in ((1.0, 0.0), (0.0, 1.0), (0.4, -0.9)):
            ref = R.T @ np.array([vx, vy, 0.0])
            u, v = body_from_ned(vx, vy, psi)
            worst = max(worst, abs(u - ref[0]), abs(v - ref[1]))
    assert worst < 1e-12, worst
    return "3축 역회전과 평면 2×2 최대차 %.2e" % worst


def test_body_ned_roundtrip():
    worst = 0.0
    for psi in (-3.1, -1.0, 0.0, 0.55, 2.9):
        for u0, v0 in ((0.0, 0.0), (0.8, 0.0), (0.0, -0.3), (0.45, 0.21)):
            c, s = math.cos(psi), math.sin(psi)
            vx = c * u0 - s * v0          # wire.state_px4std 와 같은 식
            vy = s * u0 + c * v0
            u1, v1 = body_from_ned(vx, vy, psi)
            worst = max(worst, abs(u1 - u0), abs(v1 - v0))
    assert worst < TOL, worst
    return "왕복 최대차 %.2e" % worst


# -------------------------------------------------------------- 좌표변환
def test_lla_ned_matrix_matches_ugv():
    """UGV 노드가 손으로 쓴 행렬(66–70)과 원소별로 같아야 한다."""
    lat, lon = 36.395991, 127.401794          # UGV 기본 원점
    la, lo = math.radians(lat), math.radians(lon)
    ref = np.array([
        [-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)],
        [-np.sin(lo), np.cos(lo), 0],
        [-np.cos(la) * np.cos(lo), -np.cos(la) * np.sin(lo), -np.sin(la)],
    ])
    d = float(np.max(np.abs(ecef_to_ned_matrix(lat, lon) - ref)))
    assert d == 0.0 or d < 1e-15, d
    return "행렬 원소 최대차 %.1e (UGV 원문과 동일)" % d


def test_lla_ned_small_offset():
    """100 m 급 변위가 NED 로 맞게 들어오는가.

    기준값은 **WGS84 곡률반경**으로 만든다. `111320 m/deg` 같은 평면근사를
    기준으로 쓰면 0.3 m 의 오차가 나는데 그것은 기준의 오차이고 함수의
    오차가 아니다 — 그래서 자오선·묘유선 반경을 직접 쓴다.
    """
    lat0, lon0, h0 = 36.395991, 127.401794, 25.111
    o = LlaOrigin(lat0, lon0, h0)
    la = math.radians(lat0)
    e2 = 1.0 - (1.0 - 1.0 / 298.257223563) ** 2
    w = 1.0 - e2 * math.sin(la) ** 2
    r_m = 6378137.0 * (1.0 - e2) / w ** 1.5          # 자오선 곡률반경
    r_n = 6378137.0 / math.sqrt(w)                   # 묘유선 곡률반경
    dlat = 100.0 / (r_m + h0)
    dlon = 100.0 / ((r_n + h0) * math.cos(la))
    n = o.to_ned(lat0 + math.degrees(dlat), lon0 + math.degrees(dlon))
    assert abs(n[0] - 100.0) < 0.01 and abs(n[1] - 100.0) < 0.01, n
    z0 = o.to_ned(lat0, lon0)
    assert float(np.max(np.abs(z0))) < 1e-6, z0
    # ECEF 왕복 위생검사 — 반경이 WGS84 범위 안인가
    p = lla_to_ecef(lat0, lon0, h0)
    assert 6.35e6 < float(np.linalg.norm(p)) < 6.39e6, p
    return "N %.3f m · E %.3f m (원점 오차 < 1e-6 m)" % (n[0], n[1])


# ------------------------------------------------------------ 상태기 거동
def _feed(asm, t, psi=0.3, r=0.02, x=1.0, y=2.0, vx=0.5, vy=-0.1,
          xr=3.0, yr=4.0, pr=0.5):
    asm.on_attitude(t, psi, r, t_us=int(t * 1e6))
    asm.on_local_position(t, x, y, vx, vy, t_us=int(t * 1e6))
    asm.on_position_target(t, xr, yr, pr, t_us=int(t * 1e6))


def test_staleness_blocks_state():
    asm = Px4StateAssembler(stale_s=0.5)
    assert asm.state(100.0) is None, "조각이 없는데 상태가 나왔다"
    assert "attitude" in asm.missing(100.0)
    _feed(asm, 100.0)
    assert asm.state(100.0) is not None
    assert asm.state(100.4) is not None
    assert asm.state(100.6) is None, "0.5 s 를 넘겼는데 상태가 나왔다"
    assert any(m.endswith(":stale") for m in asm.missing(100.6))
    # 자세만 늦은 경우도 막혀야 한다 (한 조각이라도 늦으면 발행 중단)
    asm2 = Px4StateAssembler(stale_s=0.5)
    _feed(asm2, 200.0)
    asm2.on_local_position(200.6, 1.0, 2.0, 0.5, -0.1)
    asm2.on_position_target(200.6, 3.0, 4.0, 0.5)
    assert asm2.state(200.6) is None
    return "stale 0.5 s 경계에서 None (τ=0 경로로 이어진다)"


def test_rejects_nonfinite():
    asm = Px4StateAssembler(stale_s=0.5)
    _feed(asm, 300.0)
    n0 = asm.n_reject
    asm.on_attitude(300.0, float("nan"), 0.0)
    asm.on_local_position(300.0, float("inf"), 0.0, 0.0, 0.0)
    asm.on_position_target(300.0, float("nan"), 0.0, 0.0)
    assert asm.n_reject == n0 + 3, (n0, asm.n_reject)
    st = asm.state(300.0)
    assert st is not None and st.finite(), "NaN 이 상태로 새어 나갔다"
    assert st.psi == wrap_pi(0.3), st.psi      # 직전 정상값 유지
    # origin 이 없으면 전지구 좌표는 버린다 (조용히 0 을 넣지 않는다)
    npos = asm.n_pos
    asm.on_global_position(300.0, 36.4, 127.4, 25.0, 0.0, 0.0)
    assert asm.n_reject == n0 + 4 and asm.n_pos == npos
    return "NaN/inf 3종 거부 + 직전값 유지, origin 없는 LLA 거부"


def test_psi_ref_los_fallback():
    """설정점에 yaw 가 없을 때의 거동을 **명시적으로** 고정한다."""
    off = Px4StateAssembler(stale_s=0.5, psi_ref_from_los=False)
    off.on_attitude(400.0, 0.0, 0.0)
    off.on_local_position(400.0, 0.0, 0.0, 0.0, 0.0)
    off.on_position_target(400.0, 1.0, 0.0, None)
    assert off.state(400.0) is None, "yaw 없이 임의값을 만들어 냈다"

    on = Px4StateAssembler(stale_s=0.5, psi_ref_from_los=True)
    on.on_attitude(400.0, 0.0, 0.0)
    on.on_local_position(400.0, 0.0, 0.0, 0.0, 0.0)
    on.on_position_target(400.0, 0.0, 1.0, None)     # 정동(東) 목표
    st = on.state(400.0)
    assert st is not None and abs(st.psi_ref - math.pi / 2) < 1e-12, st.psi_ref
    assert st.psi_ref_src == "los"
    return "yaw 없음 → 기본 None, 옵션 시 LOS(π/2) 와 출처 표기"


# ------------------------------------------------- ★ 두 경로 9 개 값 동치
def test_nine_values_parity():
    """52501 경로와 PX4 표준 경로가 **같은 9 개 값**을 만드는가.

    52501 은 `(x, y, psi, u, v, r, x_ref, y_ref, psi_ref)` 를 그대로 싣는다.
    PX4 표준은 자세·위치·설정점 3 개로 쪼개 보내고 NUC 가 다시 조립한다.
    여기서는 전송계층을 빼고 **조립식만** 비교한다 (프레임 양자화는
    `test_wire_roundtrip.py` 가 따로 닫는다).
    """
    rng = np.random.default_rng(7)
    worst = 0.0
    for _ in range(200):
        x, y = rng.uniform(-5, 5, 2)
        psi = rng.uniform(-math.pi, math.pi)
        u, v = rng.uniform(-0.8, 0.8, 2)
        r = rng.uniform(-0.4, 0.4)
        xr, yr = rng.uniform(-5, 5, 2)
        pr = rng.uniform(-math.pi, math.pi)

        # ① 52501 경로 — 값이 그대로 실린다
        direct = ExternalState(0, x, y, psi, u, v, r, xr, yr, pr).as_tuple9()

        # ② PX4 표준 경로 — 송신측(wire.state_px4std)이 NED 로 바꿔 보내고
        #    수신측(Px4StateAssembler)이 다시 바디로 돌린다
        c, s = math.cos(psi), math.sin(psi)
        vx, vy = c * u - s * v, s * u + c * v
        asm = Px4StateAssembler(stale_s=0.5)
        asm.on_attitude_quat(0.0, _quat_from_yaw(psi), r, t_us=0)
        asm.on_local_position(0.0, x, y, vx, vy, t_us=0)
        asm.on_position_target(0.0, xr, yr, pr, t_us=0)
        st = asm.state(0.0)
        assert st is not None
        got = st.as_tuple9()

        for a, b in zip(direct, got):
            worst = max(worst, abs(wrap_pi(a - b)))   # ±pi 경계도 안전
    assert worst < PARITY_TOL, worst
    return "랜덤 200 점 × 9 값, 최대 절대차 %.2e (< %.0e)" % (worst,
                                                              PARITY_TOL)
