# -*- coding: utf-8 -*-
"""ONNX 정책 ↔ sb3 정책 수치 일치 시험.

이 시험이 존재하는 이유
-----------------------
합격 기준이 깨질 때의 원인 후보 중 **첫 번째(ONNX)** 를 닫는다.
`ppo_sc3_final.zip` 은 재학습 금지 자산이고, 하네스는 그것을 ONNX 로
내보낸 것을 쓴다. 두 추론이 같다는 보장이 없으면 하네스 결과를
`pontoon_rl` 직접 평가와 비교할 수 없다.

★ 비교 규약 — 반드시 양쪽을 같은 방식으로 자른다
-------------------------------------------------
`PPO.predict(deterministic=True)` 는 **행동을 action space 로 clip 해서**
돌려준다. ONNX 그래프는 `action_net(mlp(obs))` 원값을 그대로 낸다
(clip 노드가 없다). 따라서 **clip 하지 않은 ONNX 출력과 sb3 출력을 직접
빼면 max|Δ| 가 6 을 넘는다** — 이것은 수치오차가 아니라 포화 구간을
비교한 것이다. 실제 배포 경로(`policy/onnx_policy.py::OnnxPolicy.act`)는
`lo=-1, hi=+1` 로 clip 하므로, 비교도 clip 후에 해야 등가다.
이 사실 자체를 `test_clipping_is_the_only_difference` 가 고정한다.

관측 분포
---------
난수 `U(-3,3)` 는 실제 관측 다양체에서 멀다. 그런 입력은 두 추론 모두
포화 영역으로 보내므로 일치 여부를 재는 의미가 없다. 그래서
**환경을 ONNX 정책으로 실제 롤아웃해 모은 관측**으로 비교한다.

허용오차
--------
max|Δaction| < 1.0e-5  (float32 추론 경로 2개의 누적 차이 상한)
기록된 실측: 9.090e-07 (out/onnx_parity.txt)
"""
from __future__ import annotations

import os

import numpy as np

import _common  # noqa: F401

TOL = 1.0e-5
LO, HI = -1.0, 1.0            # action space 경계 = OnnxPolicy 의 clip 범위
N_ROLLOUT_EP = 4              # 관측 수집용 에피소드 (1 ep = 600 제어스텝)

ZIP = os.path.join(_common.ROOT, "pontoon_rl", "out", "n1", "ppo_sc3_final.zip")
ONNX = os.path.join(_common.HARNESS, "out", "ppo_sc3_final.onnx")

_CACHE = {}


def _session():
    import onnxruntime as ort                       # noqa: WPS433
    s = ort.InferenceSession(ONNX, providers=["CPUExecutionProvider"])
    return s, s.get_inputs()[0].name


def _onmanifold_obs():
    """ONNX 정책으로 환경을 실제로 돌려 관측을 모은다 (운용 분포)."""
    if "obs" in _CACHE:
        return _CACHE["obs"]
    from env.pontoon_env import PontoonSc3Env        # noqa: WPS433
    from policy.onnx_policy import OnnxPolicy        # noqa: WPS433

    p = OnnxPolicy(ONNX)
    e = PontoonSc3Env(dr=True)
    rows = []
    for ep in range(N_ROLLOUT_EP):
        o, _ = e.reset(seed=2000 + ep)
        rows.append(np.asarray(o, dtype=np.float32))
        done = False
        while not done:
            a = p.act(o)
            o, _r, term, trunc, _i = e.step(a)
            rows.append(np.asarray(o, dtype=np.float32))
            done = bool(term or trunc)
    _CACHE["obs"] = np.asarray(rows, dtype=np.float32)
    return _CACHE["obs"]


def test_artifacts_exist():
    assert os.path.isfile(ZIP), "정책 zip 이 없다: %s" % ZIP
    assert os.path.isfile(ONNX), (
        "ONNX 가 없다: %s\n  harness_sitl/policy/export_onnx.py 를 먼저 돌려라." % ONNX)
    return "zip %.1f MB / onnx %.0f kB" % (
        os.path.getsize(ZIP) / 1e6, os.path.getsize(ONNX) / 1e3)


def test_onnx_matches_sb3():
    """운용 분포 관측에서 두 추론이 TOL 미만으로 일치해야 한다 (양쪽 clip)."""
    from stable_baselines3 import PPO               # noqa: WPS433

    m = PPO.load(ZIP, device="cpu")
    obs_dim = int(m.observation_space.shape[0])
    act_dim = int(m.action_space.shape[0])
    assert obs_dim == 19, "관측 차원이 19 가 아니다: %d" % obs_dim
    assert act_dim == 3, "행동 차원이 3 이 아니다: %d" % act_dim
    assert float(m.action_space.low.min()) == LO
    assert float(m.action_space.high.max()) == HI

    obs = _onmanifold_obs()
    sess, iname = _session()

    a_sb3 = np.asarray(m.predict(obs, deterministic=True)[0], dtype=np.float64)
    a_onnx = np.clip(np.asarray(sess.run(None, {iname: obs})[0],
                                dtype=np.float64), LO, HI)

    d = np.abs(a_sb3 - a_onnx)
    mx, mn = float(d.max()), float(d.mean())
    assert mx < TOL, "ONNX 불일치: max|Δ| = %.3e (허용 %.1e)" % (mx, TOL)
    return "max|Δ| %.3e / mean %.3e (허용 %.1e, 운용관측 n=%d)" % (
        mx, mn, TOL, obs.shape[0])


def test_clipping_is_the_only_difference():
    """★ clip 을 빼면 왜 커지는지를 수치로 고정한다 (오진 방지).

    난수 U(-3,3) 입력에서 sb3(clip) vs ONNX(원값) 의 차이는 **1 을 넘는다**.
    같은 입력에서 양쪽을 clip 하면 TOL 미만으로 떨어진다. 즉 불일치의
    원인은 내보내기가 아니라 clip 유무다.
    """
    from stable_baselines3 import PPO               # noqa: WPS433

    m = PPO.load(ZIP, device="cpu")
    sess, iname = _session()
    rng = np.random.default_rng(12345)
    obs = rng.uniform(-3.0, 3.0, size=(2000, 19)).astype(np.float32)

    a_sb3 = np.asarray(m.predict(obs, deterministic=True)[0], dtype=np.float64)
    raw = np.asarray(sess.run(None, {iname: obs})[0], dtype=np.float64)

    d_raw = float(np.abs(a_sb3 - raw).max())
    d_clip = float(np.abs(a_sb3 - np.clip(raw, LO, HI)).max())
    assert d_raw > 1.0, "clip 차이가 안 보인다 (%.3e) — 그래프가 바뀌었다" % d_raw
    assert d_clip < TOL, "clip 후에도 불일치: %.3e" % d_clip
    assert float(np.abs(a_sb3).max()) <= HI + 1e-9, "sb3 가 clip 하지 않았다"
    return "clip 전 %.3e / clip 후 %.3e → 원인은 clip 유무" % (d_raw, d_clip)


def test_no_obs_normalization():
    """학습에 VecNormalize 가 없었음을 고정한다.

    있었다면 ONNX 입력 앞에 정규화 상수를 붙여야 하고, 그걸 빠뜨리면
    궤적이 조용히 틀린다. 저장된 zip 에 정규화 통계가 없음을 확인한다.
    """
    import zipfile                                 # noqa: WPS433
    with zipfile.ZipFile(ZIP) as z:
        names = z.namelist()
    bad = [n for n in names if "vecnormalize" in n.lower() or "obs_rms" in n.lower()]
    assert not bad, "관측 정규화 통계가 존재한다: %s" % bad
    return "VecNormalize 없음 → ONNX 입력 = 환경 원 관측"


def test_harness_policy_wrapper():
    """하네스 래퍼 = ONNX 세션 + clip(lo,hi) 임을 고정한다.

    래퍼가 clip 외의 가공(스케일·offset·정규화)을 하면 여기서 깨진다.
    """
    from policy.onnx_policy import OnnxPolicy      # noqa: WPS433

    p = OnnxPolicy(ONNX)
    assert (p.lo, p.hi) == (LO, HI), "래퍼 clip 범위가 바뀠다: %s" % ((p.lo, p.hi),)
    sess, iname = _session()

    rng = np.random.default_rng(777)
    worst = 0.0
    n_sat = 0
    for _ in range(50):
        o = rng.uniform(-3.0, 3.0, 19)
        a1 = np.asarray(p.act(o), dtype=np.float64).ravel()
        raw = np.asarray(sess.run(None, {iname: o.astype(np.float32)[None, :]})[0],
                         dtype=np.float64).ravel()
        n_sat += int(np.any((raw < LO) | (raw > HI)))
        worst = max(worst, float(np.abs(a1 - np.clip(raw, LO, HI)).max()))
    assert worst == 0.0, "래퍼가 clip 외의 가공을 한다: %.3e" % worst
    return "래퍼 = 세션+clip, 오차 %.1e (포화 샘플 %d/50)" % (worst, n_sat)
