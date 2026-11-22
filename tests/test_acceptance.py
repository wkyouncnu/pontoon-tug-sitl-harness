# -*- coding: utf-8 -*-
"""합격 기준 시험 — A→B→C→D→A 폐루프 20 에피소드.

과제가 요구한 기준
------------------
    성공 20/20, 평균 위치오차 0.01 m 이내
    (pontoon_rl 직접 평가 = 0.003 m / 20-20 과 같은 수준)

실측 (out/acceptance_20ep.txt)
------------------------------
    20/20, 평균 0.00252 m (최대 0.00517 m), 방위 0.0296 deg
    약 0.75 s/에피소드 → 20 에피소드 약 15 s

진단 보조 시험
--------------
합격이 깨지면 원인을 좁힐 수 있도록 다음을 함께 고정한다.
  * 하위 역배분 B_F^-1 왕복 (노드 D)
  * 레이트 분할 10 Hz : 50 Hz 가 전송 통계로 실제 재현되는지
  * npz 궤적 포맷이 make_video.py 계약과 맞는지
느린 시험이므로 run_tests.py --fast 에서는 건너뛴다.
"""
from __future__ import annotations

import os

import numpy as np

import _common  # noqa: F401

from run_harness import ACCEPT_MEAN_POS, POS_TOL_OK, YAW_TOL_OK, Harness

N_EP = 20
SEED0 = 1000

_CACHE = {}


def _run20():
    """20 에피소드를 한 번만 돌리고 결과를 재사용한다."""
    if "r" in _CACHE:
        return _CACHE["r"]
    h = Harness(transport="inproc", dr=True)
    rows = []
    for e in range(N_EP):
        rows.append(h.episode(SEED0 + e))
    _CACHE["r"] = (h, rows)
    return _CACHE["r"]


def test_closed_loop_acceptance():
    """★ 본 시험. 20/20 · 평균 위치오차 <= 0.01 m."""
    _, rows = _run20()
    dps = np.array([r["dpos"] for r in rows])
    dys = np.array([r["dyaw"] for r in rows])
    oks = int(sum((r["dpos"] < POS_TOL_OK) and (r["dyaw"] < YAW_TOL_OK)
                  and not r["term"] for r in rows))
    assert oks == N_EP, "성공 %d/%d (실패 에피소드 존재)" % (oks, N_EP)
    assert float(dps.mean()) <= ACCEPT_MEAN_POS, (
        "평균 위치오차 %.5f m > 허용 %.3f m" % (dps.mean(), ACCEPT_MEAN_POS))
    return ("%d/%d · 평균 %.5f m (최대 %.5f m) · 방위 %.4f deg"
            % (oks, N_EP, dps.mean(), dps.max(), np.degrees(dys.mean())))


def test_rate_split_reproduced():
    """정책 10 Hz : 상태 50 Hz 가 전송 통계에 실제로 나타나야 한다."""
    h, _ = _run20()
    st = h.tp.stats()
    n_state = st["/pontoon/state"]["pub"]
    n_tau = st["/mothership_data"]["pub"]
    ratio = n_state / max(n_tau, 1)
    assert abs(ratio - 5.0) < 0.02, (
        "레이트 비가 5 가 아니다: state %d / tau_d %d = %.3f"
        % (n_state, n_tau, ratio))
    return "state %d : tau_d %d = %.2f : 1 (= 50 Hz : 10 Hz)" % (
        n_state, n_tau, ratio)


def test_no_packet_loss_by_default():
    """기본 설정(지연 0·드랍 0)에서는 손실이 0 이어야 한다."""
    h, _ = _run20()
    bad = {t: s["drop"] for t, s in h.tp.stats().items() if s["drop"]}
    assert not bad, "기본 설정에서 드랍 발생: %s" % bad
    return "모든 토픽 drop 0 (지연·드랍 주입은 옵션이며 기본 0)"


def test_lower_allocation_roundtrip():
    """노드 D 의 B_F^-1 역배분이 τ 를 수치적으로 복원해야 한다.

    τ → f (3기) → τ 왕복이 깨지면 하위 배분 행렬이 잘못 세워진 것이다.
    포화 clip 에 걸리지 않는 작은 τ 로만 확인한다.
    """
    from iface.transport import make_transport     # noqa: WPS433
    from nodes.node_d_esp32 import Esp32LinkNode   # noqa: WPS433

    d = Esp32LinkNode(make_transport("inproc"))
    rng = np.random.default_rng(3)
    worst = 0.0
    for _ in range(200):
        tau = rng.uniform(-8.0, 8.0, 3)
        f, back = d._allocate(tau)
        worst = max(worst, float(np.abs(back - tau).max()))
        assert f.shape == (3,), "추진기 3기가 아니다: %s" % (f.shape,)
    assert worst < 1e-9, "역배분 왕복 오차 %.3e" % worst
    return "τ→f→τ 왕복 오차 %.1e (det T = 2d ≠ 0)" % worst


def test_npz_video_contract():
    """저장된 궤적이 pontoon_rl/viz/make_video.py 계약을 만족해야 한다."""
    p = os.path.join(_common.HARNESS, "out", "traj_harness.npz")
    assert os.path.isfile(p), "궤적 npz 가 없다. run_harness.py 를 먼저 돌려라: %s" % p
    z = np.load(p)
    need = ["S", "A", "U", "M", "r1", "r2", "goal", "dt"]
    miss = [k for k in need if k not in z]
    assert not miss, "npz 키 누락: %s" % miss
    assert z["S"].shape[1] == 6, "S 가 6열이 아니다: %s" % (z["S"].shape,)
    assert z["U"].shape[1] == 6, "U 가 6열이 아니다: %s" % (z["U"].shape,)
    assert abs(float(z["dt"]) - 0.1) < 1e-12, "dt 가 0.1 이 아니다: %s" % z["dt"]
    return "키 %d개 · S %s · U %s · dt %.2f" % (
        len(need), z["S"].shape, z["U"].shape, float(z["dt"]))
