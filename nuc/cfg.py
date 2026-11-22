# -*- coding: utf-8 -*-
"""harness_sitl/nuc/cfg.py — 설정 병합 (학습값 + 현장 기하 덮어쓰기).

왜 있는가
---------
부착점 `r1` · `r2` 는 **현장에서 바뀌는 값**인데 `pontoon_rl/params/sc3.yaml`
안에 있었다. 저 파일은 학습 당시 설정이라 고치면 학습 재현이 깨진다.
→ `harness_sitl/config/geometry.yaml` 하나만 고치면 되게 가른다.

    pontoon_rl/params/sc3.yaml      (읽기전용, 학습 당시 값)
              │
              ├── harness_sitl/config/geometry.yaml   ← **현장에서 고치는 파일**
              │
              ▼
          load_cfg()  →  NucController · PontoonSc3Env · node_d_esp32

한 군데서 병합하므로 **제어기·QP·플랜트·관측이 같은 기하를 쓴다.**
덮어쓰기 파일이 없으면 `sc3.yaml` 값이 그대로 쓰인다 (기존 동작과 동일).

기준좌표계·재는 방법은 `config/geometry.yaml` 머리말과 `docs/GEOMETRY.md`.

전송계층 의존성 0 (`nuc/` 규율). `tests/test_geometry_cfg.py` 가 검사한다.
"""
from __future__ import annotations

import copy
import os
import sys
from typing import Any, Dict, Optional, Union

_HARNESS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(_HARNESS)
for _p in (os.path.join(_ROOT, "pontoon_rl"),):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from env.pontoon_env import CFG_DEFAULT, _load            # noqa: E402

#: 현장에서 고치는 파일. 없으면 덮어쓰기를 하지 않는다.
GEOM_DEFAULT = os.path.join(_HARNESS, "config", "geometry.yaml")

#: `frame` 이 이 값이 아니면 거부한다 — 좌표계를 조용히 바꾸지 못하게 한다.
FRAME_EXPECTED = "pontoon_body_frd"

_TOP_KEYS = ("frame", "attach", "dr")
_ATTACH_KEYS = ("r1", "r2")
_DR_KEYS = ("r_jit",)
_LIMIT_M = 10.0          # 폰툰이 2.6 × 2.15 m 이므로 이보다 크면 단위 착오다


def _vec2(name: str, v: Any) -> list:
    """길이 2 · 유한 · 범위 안인지 검사하고 float 리스트로 돌려준다."""
    try:
        seq = list(v)
    except TypeError:
        raise ValueError("geometry.yaml: %s 는 [x, y] 두 숫자여야 한다 (%r)"
                         % (name, v))
    if len(seq) != 2:
        raise ValueError("geometry.yaml: %s 의 길이가 %d 다 — [x, y] 두 숫자여야 한다"
                         % (name, len(seq)))
    out = []
    for i, x in enumerate(seq):
        try:
            f = float(x)
        except (TypeError, ValueError):
            raise ValueError("geometry.yaml: %s[%d] 가 숫자가 아니다 (%r)"
                             % (name, i, x))
        if f != f or abs(f) == float("inf"):
            raise ValueError("geometry.yaml: %s[%d] 가 유한수가 아니다" % (name, i))
        if abs(f) > _LIMIT_M:
            raise ValueError(
                "geometry.yaml: %s[%d] = %g m 는 폰툰(2.60 × 2.15 m) 밖이다 — "
                "단위가 mm 거나 부호가 틀렸을 수 있다" % (name, i, f))
        out.append(f)
    return out


def _check_keys(doc: Dict[str, Any]) -> None:
    """모르는 키는 **거부**한다 — 오타가 조용히 무시되지 않게."""
    for k in doc:
        if k not in _TOP_KEYS:
            raise ValueError("geometry.yaml: 모르는 항목 %r (쓸 수 있는 것: %s)"
                             % (k, ", ".join(_TOP_KEYS)))
    for sect, allowed in (("attach", _ATTACH_KEYS), ("dr", _DR_KEYS)):
        d = doc.get(sect)
        if d is None:
            continue
        if not isinstance(d, dict):
            raise ValueError("geometry.yaml: %s 는 사전이어야 한다" % sect)
        for k in d:
            if k not in allowed:
                raise ValueError("geometry.yaml: %s 아래 모르는 항목 %r "
                                 "(쓸 수 있는 것: %s)"
                                 % (sect, k, ", ".join(allowed)))


def load_geometry(path: Optional[str] = None) -> Dict[str, Any]:
    """`geometry.yaml` 을 읽고 검사한다. 없으면 빈 사전."""
    p = GEOM_DEFAULT if path is None else path
    if not os.path.isfile(p):
        return {}
    doc = _load(p) or {}
    if not isinstance(doc, dict):
        raise ValueError("geometry.yaml: 최상위가 사전이 아니다 (%s)" % p)
    _check_keys(doc)

    frame = doc.get("frame", FRAME_EXPECTED)
    if str(frame) != FRAME_EXPECTED:
        raise ValueError(
            "geometry.yaml: frame 이 %r 다. 이 코드는 %r (원점=폰툰 기하중심, "
            "+x 선수, +y 우현) 만 쓴다 — 좌표계를 바꾸려면 코드를 고쳐야 한다"
            % (frame, FRAME_EXPECTED))

    out = {}  # type: Dict[str, Any]
    att = doc.get("attach") or {}
    for k in _ATTACH_KEYS:
        if k in att:
            out.setdefault("attach", {})[k] = _vec2(k, att[k])
    dr = doc.get("dr") or {}
    if "r_jit" in dr:
        j = float(dr["r_jit"])
        if not (0.0 <= j <= 1.0):
            raise ValueError("geometry.yaml: dr.r_jit = %g 는 0–1 m 밖이다" % j)
        out.setdefault("dr", {})["r_jit"] = j
    return out


def load_cfg(cfg: Optional[Union[str, Dict[str, Any]]] = None,
             geometry: Union[None, bool, str] = None) -> Dict[str, Any]:
    """학습 설정에 현장 기하를 덮어쓴 **새 사전**을 돌려준다.

    Parameters
    ----------
    cfg : None | str | dict
        `None` → `pontoon_rl/params/sc3.yaml`. 사전을 주면 그대로 바탕으로 쓴다.
    geometry : None | False | str
        `None` → `config/geometry.yaml` (있으면 적용, 없으면 통과).
        `False` → 덮어쓰기 금지 (학습 당시 값 그대로. 재현시험용).
        문자열 → 그 경로의 파일.

    원본 사전을 바꾸지 않는다 (`copy.deepcopy`).
    """
    base = cfg if isinstance(cfg, dict) else _load(cfg or CFG_DEFAULT)
    out = copy.deepcopy(base)
    if geometry is False:
        return out
    ov = load_geometry(None if geometry is None else geometry)
    for sect, vals in ov.items():
        out.setdefault(sect, {}).update(vals)
    return out


def geometry_summary(c: Dict[str, Any]) -> str:
    """로그 한 줄 — 어느 기하로 돌고 있는지 사람이 보게 한다."""
    a = c["attach"]
    r1, r2 = a["r1"], a["r2"]
    dx = abs(float(r1[0]) - float(r2[0]))
    return ("부착기하 r1=(%+.4f, %+.4f) r2=(%+.4f, %+.4f) m  간격 %.3f m  "
            "r_jit %.3f m  [폰툰 바디 FRD, 원점=기하중심]"
            % (r1[0], r1[1], r2[0], r2[1], dx,
               float(c.get("dr", {}).get("r_jit", 0.0))))
