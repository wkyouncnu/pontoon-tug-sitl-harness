# -*- coding: utf-8 -*-
"""harness_sitl/policy/export_onnx.py — sb3 PPO 정책 → ONNX 내보내기 + 수치일치 검증.

왜 ONNX 인가
------------
확정된 배포 결정(§BEST_PLAN, 사용자 확인): **정책은 Pixhawk 가 아니라
마더십 NUC 에 ONNX 로 배포한다.** 따라서 하네스의 노드 C 도 sb3 가 아니라
**ONNX 만** 읽어야 실물과 같은 경로가 된다.

무엇을 내보내는가
-----------------
sb3 `ActorCriticPolicy` 의 **결정론적 행동 경로만** 뽑는다:

    obs(19) → FlattenExtractor → mlp_extractor.policy_net → action_net → (3)
              (Flatten 은 1-D 관측에서 항등)

* value_net / log_std / 탐색은 **내보내지 않는다** (배포에 불필요).
* sb3 `predict(deterministic=True)` 는 마지막에 action_space 로
  `np.clip` 한다 → ONNX 바깥(onnx_policy.py)에서 똑같이 clip 한다.
* 학습에 **VecNormalize 를 쓰지 않았다** (train_ppo.py 는 DummyVecEnv/
  SubprocVecEnv + VecMonitor 만 사용) → **관측 정규화 상수가 없다.**
  즉 ONNX 입력은 환경이 주는 raw 관측 그대로다.

재학습은 하지 않는다. `ppo_sc3_final.zip` 의 가중치를 그대로 옮긴다.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _console import utf8_stdout
utf8_stdout()

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.dirname(HERE)
ROOT = os.path.dirname(HARNESS)
PONTOON = os.path.join(ROOT, "pontoon_rl")

DEFAULT_ZIP = os.path.join(PONTOON, "out", "n1", "ppo_sc3_final.zip")
DEFAULT_ONNX = os.path.join(HARNESS, "out", "ppo_sc3_final.onnx")
DEFAULT_LOG = os.path.join(HARNESS, "out", "onnx_parity.txt")

# 허용오차 — float32 MLP 2층(128) 의 누적 반올림 수준으로 잡는다.
TOL_ABS = 1e-5


def _build_actor(policy):
    import torch
    import torch.nn as nn

    class _Actor(nn.Module):
        def __init__(self, pol):
            super().__init__()
            self.extractor = pol.mlp_extractor.policy_net
            self.action_net = pol.action_net

        def forward(self, obs):
            return self.action_net(self.extractor(obs))

    m = _Actor(policy).eval()
    for p in m.parameters():
        p.requires_grad_(False)
    return m


def export(zip_path=DEFAULT_ZIP, onnx_path=DEFAULT_ONNX, log_path=DEFAULT_LOG,
           n_test=2000, seed=0, tol=TOL_ABS, opset=17):
    import torch
    from stable_baselines3 import PPO

    os.makedirs(os.path.dirname(onnx_path), exist_ok=True)
    model = PPO.load(zip_path, device="cpu")
    pol = model.policy.eval()

    obs_dim = int(np.prod(model.observation_space.shape))
    act_dim = int(np.prod(model.action_space.shape))
    lo = np.asarray(model.action_space.low, dtype=np.float32)
    hi = np.asarray(model.action_space.high, dtype=np.float32)

    actor = _build_actor(pol)
    dummy = torch.zeros(1, obs_dim, dtype=torch.float32)
    torch.onnx.export(
        actor, dummy, onnx_path,
        input_names=["obs"], output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        opset_version=opset, do_constant_folding=True)

    # ── 수치일치 검증 ────────────────────────────────────────────────────
    import onnxruntime as ort
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])

    rng = np.random.default_rng(seed)
    # 관측 범위를 모르는 채로 넓게 흔든다: 정규분포 + 꼬리 과장
    X = rng.standard_normal((n_test, obs_dim)).astype(np.float32)
    X[: n_test // 4] *= 5.0          # 꼬리
    X[n_test // 4: n_test // 2] *= 0.1

    a_sb3 = np.empty((n_test, act_dim), dtype=np.float64)
    for i in range(n_test):
        a_sb3[i] = model.predict(X[i], deterministic=True)[0]

    a_onx = sess.run(["action"], {"obs": X})[0].astype(np.float64)
    a_onx = np.clip(a_onx, lo, hi)   # sb3 predict 와 같은 후처리

    err = np.abs(a_sb3 - a_onx)
    emax, emean = float(err.max()), float(err.mean())

    lines = [
        "ONNX vs stable-baselines3 결정론 추론 수치일치 검증",
        "=" * 64,
        "zip        : %s" % os.path.relpath(zip_path, ROOT),
        "onnx       : %s" % os.path.relpath(onnx_path, ROOT),
        "obs_dim    : %d   act_dim : %d   action_space : [%g, %g]"
        % (obs_dim, act_dim, lo.min(), hi.max()),
        "opset      : %d" % opset,
        "n_test     : %d  (정규분포 샘플, 1/4 는 ×5 꼬리, 1/4 는 ×0.1)" % n_test,
        "seed       : %d" % seed,
        "",
        "내보낸 경로: obs → mlp_extractor.policy_net(128-tanh-128-tanh) → action_net(3)",
        "후처리     : np.clip(action, low, high)  — sb3 predict 와 동일",
        "관측 정규화: **없음** (학습에 VecNormalize 미사용 — train_ppo.py 확인)",
        "",
        "허용오차   : max|Δ| < %.1e" % tol,
        "실측 max|Δ|: %.3e" % emax,
        "실측 mean|Δ|: %.3e" % emean,
        "판정       : %s" % ("PASS" if emax < tol else "FAIL"),
    ]
    txt = "\n".join(lines)
    with open(log_path, "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print(txt)
    print("\n로그 -> %s" % log_path)
    return emax < tol, emax


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zip", default=DEFAULT_ZIP)
    ap.add_argument("--onnx", default=DEFAULT_ONNX)
    ap.add_argument("--log", default=DEFAULT_LOG)
    ap.add_argument("--n", type=int, default=2000)
    ap.add_argument("--tol", type=float, default=TOL_ABS)
    a = ap.parse_args()
    ok, _ = export(a.zip, a.onnx, a.log, n_test=a.n, tol=a.tol)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
