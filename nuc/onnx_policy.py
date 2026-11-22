# -*- coding: utf-8 -*-
"""harness_sitl/nuc/onnx_policy.py — ONNX 정책 추론기 (**production**).

이 파일의 지위
--------------
**실물 NUC 에서 그대로 실행되는 파일이다.** `nuc/controller.py` 와 함께
`nuc/` 패키지의 production 경계 안에 있다 (`docs/FILE_SWAP.md` 참조).
→ import 하는 것은 `os` · `numpy` · `onnxruntime` **뿐이다.**
   `stable_baselines3` · `torch` 는 import 하지 않는다. 실물 NUC 에 설치하지
   않는 것이 목적이다 (재학습 = ONNX 파일 교체).

후처리
------
sb3 `predict(deterministic=True)` 와 동일하게 **action_space clip 뿐**이다.
학습에 `VecNormalize` 를 쓰지 않았으므로 관측 정규화가 없다
(`harness_sitl/policy/export_onnx.py` 참조).
sb3 ↔ ONNX 차이가 clip 하나뿐이라는 것은 실측했다:
clip 전 최대 6.627e+00 → clip 후 **6.557e-07** (`out/onnx_parity.txt`).

스레드 수를 1 로 고정하는 이유
------------------------------
① 결정론 (스레드 수가 바뀌면 reduction 순서가 바뀐다)
② 실물 NUC 의 단일코어 예산 — 상위 분배기와 같은 보드에서 돌기 때문이다.

⚠️ 실물 배치 시
---------------
`DEFAULT_ONNX` 는 **하네스 편의용 기본값**(`harness_sitl/out/ppo_sc3_final.onnx`)
이다. 실물 NUC 에서는 `OnnxPolicy(path)` 로 **경로를 명시해 넘겨라**
(ROS 2 파라미터 등). `docs/FILE_SWAP.md` §3 참조.
"""
from __future__ import annotations

import os

import numpy as np

DEFAULT_ONNX = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "out", "ppo_sc3_final.onnx")


class OnnxPolicy(object):
    """obs(19) → action(3), 결정론."""

    def __init__(self, path: str = DEFAULT_ONNX, lo: float = -1.0, hi: float = 1.0):
        if not os.path.isfile(path):
            raise FileNotFoundError(
                "ONNX 파일이 없다: %s\n"
                "먼저 harness_sitl/policy/export_onnx.py 를 실행해라." % path)
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1       # 결정론 + 실물 NUC 단일코어 예산
        so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(path, so,
                                         providers=["CPUExecutionProvider"])
        self.path = path
        self.lo, self.hi = lo, hi
        self.obs_dim = int(self.sess.get_inputs()[0].shape[-1])
        self.act_dim = int(self.sess.get_outputs()[0].shape[-1])
        self.n_calls = 0

    def act(self, obs) -> np.ndarray:
        o = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        a = self.sess.run(["action"], {"obs": o})[0][0]
        self.n_calls += 1
        return np.clip(np.asarray(a, dtype=np.float64), self.lo, self.hi)

    def __repr__(self):
        return "OnnxPolicy(%s, %d->%d)" % (os.path.basename(self.path),
                                           self.obs_dim, self.act_dim)
