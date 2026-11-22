# -*- coding: utf-8 -*-
"""harness_sitl/policy/onnx_policy.py — **re-export 껍데기** (하위호환용).

본체는 `harness_sitl/nuc/onnx_policy.py` 로 옮겼다 (S4).

왜 옮겼나
---------
이 추론기는 **실물 NUC 에서 그대로 돌아야 하는 production 코드**다. 그런데
`policy/` 에는 `export_onnx.py` 가 있고 그것은 `stable_baselines3` · `torch` 를
import 하는 **학습측 전용** 스크립트다. 둘을 한 패키지에 두면
"이 패키지를 실물에 배치해야 하나"가 모호해진다.
→ production 은 `nuc/`, 학습·내보내기는 `policy/` 로 경계를 그었다.
   `docs/FILE_SWAP.md` 참조.

이 껍데기를 남기는 이유
-----------------------
기존 테스트·스크립트가 `from policy.onnx_policy import OnnxPolicy` 로 import
하고 있다. 그 import 를 깨지 않기 위해 남긴다. **새 코드는
`from nuc.onnx_policy import OnnxPolicy` 를 써라.**
"""
from __future__ import annotations

import os
import sys

_HARNESS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _HARNESS not in sys.path:
    sys.path.insert(0, _HARNESS)

from nuc.onnx_policy import DEFAULT_ONNX, OnnxPolicy   # noqa: E402,F401

__all__ = ["OnnxPolicy", "DEFAULT_ONNX"]
