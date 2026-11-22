# -*- coding: utf-8 -*-
"""harness_sitl/tests/test_geometry_cfg.py — 부착기하 외부화 시험.

무엇을 닫는가
-------------
| 시험 | 닫는 질문 |
|---|---|
| `test_disabled_matches_sc3` | 덮어쓰기를 끄면 **학습 당시 값 그대로**인가 |
| `test_default_file_is_valid` | 저장소에 들어 있는 `config/geometry.yaml` 이 통과하는가 |
| `test_override_reaches_controller_and_qp` | 파일만 고치면 **제어기와 QP 가 같이** 바뀌는가 |
| `test_override_reaches_plant` | **플랜트도** 같이 바뀌는가 (안 바뀌면 시험이 거짓양성) |
| `test_unknown_key_rejected` | 오타가 조용히 무시되지 않는가 |
| `test_bad_frame_rejected` | 좌표계를 조용히 바꾸지 못하는가 |
| `test_unit_mistake_rejected` | mm 로 적으면 걸리는가 |
| `test_base_cfg_not_mutated` | 원본 설정을 건드리지 않는가 |
| `test_summary_line` | 로그 한 줄이 기하를 사람이 읽게 적는가 |
"""
from __future__ import annotations

import os
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_HARNESS = os.path.dirname(_HERE)
for _p in (_HARNESS, os.path.join(os.path.dirname(_HARNESS), "pontoon_rl")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from nuc.cfg import (CFG_DEFAULT, FRAME_EXPECTED, GEOM_DEFAULT,  # noqa: E402
                     geometry_summary, load_cfg, load_geometry)
from env.pontoon_env import _load                                # noqa: E402

R1_NEW = [0.800, -1.000]
R2_NEW = [-0.400, -1.000]


def _write(body: str) -> str:
    fd, p = tempfile.mkstemp(suffix=".yaml", prefix="geom_")
    os.close(fd)
    with open(p, "w", encoding="utf-8") as f:
        f.write(body)
    return p


def _yaml_override(r1=R1_NEW, r2=R2_NEW, extra: str = "") -> str:
    return ("frame: %s\nattach:\n  r1: [%g, %g]\n  r2: [%g, %g]\n%s"
            % (FRAME_EXPECTED, r1[0], r1[1], r2[0], r2[1], extra))


def test_disabled_matches_sc3():
    """`geometry=False` = 학습 당시 값. 재현시험이 이 경로를 쓴다."""
    base = _load(CFG_DEFAULT)
    c = load_cfg(geometry=False)
    assert c["attach"]["r1"] == base["attach"]["r1"], c["attach"]
    assert c["attach"]["r2"] == base["attach"]["r2"], c["attach"]


def test_default_file_is_valid():
    """저장소의 `config/geometry.yaml` 이 검사를 통과하고, 값이 적용된다."""
    assert os.path.isfile(GEOM_DEFAULT), GEOM_DEFAULT
    ov = load_geometry()
    assert "attach" in ov and "r1" in ov["attach"], ov
    c = load_cfg()
    assert c["attach"]["r1"] == ov["attach"]["r1"]
    # 두 터그가 같은 좌현 변 → 둘 다 y < 0 (설계 전제)
    assert c["attach"]["r1"][1] < 0.0 and c["attach"]["r2"][1] < 0.0


def test_override_reaches_controller_and_qp():
    """파일 하나로 제어기 설정과 **QP 기하행렬**이 같이 바뀌는지."""
    from env.latch_qp import body_force_map
    from nuc.controller import NucController

    p = _write(_yaml_override())
    try:
        ctl = NucController(geometry=p)
    finally:
        os.unlink(p)
    assert ctl.c["attach"]["r1"] == R1_NEW, ctl.c["attach"]
    assert ctl.c["attach"]["r2"] == R2_NEW, ctl.c["attach"]

    want = np.hstack([body_force_map(R1_NEW), body_force_map(R2_NEW)])
    d = float(np.max(np.abs(ctl.qp.A - want)))
    assert d < 1e-12, "QP 기하행렬이 덮어쓴 값과 다르다: %g" % d

    base = np.hstack([body_force_map(_load(CFG_DEFAULT)["attach"]["r1"]),
                      body_force_map(_load(CFG_DEFAULT)["attach"]["r2"])])
    assert float(np.max(np.abs(base - want))) > 1e-3, "시험값이 기본값과 같다"


def test_override_reaches_plant():
    """플랜트(`PontoonSc3Env`)도 같은 기하를 쓰는지 — 안 쓰면 시험이 거짓양성."""
    from iface.transport import InprocTransport
    from nodes.node_a_plant import PlantNode

    p = _write(_yaml_override())
    try:
        A = PlantNode(InprocTransport(), dr=False, seed=0, geometry=p)
    finally:
        os.unlink(p)
    assert np.allclose(A.env.r1, R1_NEW, atol=1e-12), A.env.r1
    assert np.allclose(A.env.r2, R2_NEW, atol=1e-12), A.env.r2


def test_unknown_key_rejected():
    for body in ("frame: %s\nattatch:\n  r1: [0, -1]\n" % FRAME_EXPECTED,
                 "frame: %s\nattach:\n  r3: [0, -1]\n" % FRAME_EXPECTED,
                 "frame: %s\ndr:\n  rjit: 0.1\n" % FRAME_EXPECTED):
        p = _write(body)
        try:
            load_geometry(p)
        except ValueError as e:
            assert "모르는 항목" in str(e), str(e)
        else:
            raise AssertionError("오타가 통과했다: %r" % body)
        finally:
            os.unlink(p)


def test_bad_frame_rejected():
    p = _write("frame: ned_absolute\nattach:\n  r1: [0.5, -1.08]\n")
    try:
        load_geometry(p)
    except ValueError as e:
        assert "frame" in str(e), str(e)
    else:
        raise AssertionError("좌표계가 조용히 바뀌었다")
    finally:
        os.unlink(p)


def test_unit_mistake_rejected():
    """mm 로 적은 경우 — 489.4 mm 를 m 로 읽으면 폰툰 밖이다."""
    p = _write(_yaml_override(r1=[489.4, -1080.0]))
    try:
        load_geometry(p)
    except ValueError as e:
        assert "밖" in str(e), str(e)
    else:
        raise AssertionError("mm 입력이 통과했다")
    finally:
        os.unlink(p)

    p = _write("frame: %s\nattach:\n  r1: [0.5]\n" % FRAME_EXPECTED)
    try:
        load_geometry(p)
    except ValueError as e:
        assert "길이" in str(e), str(e)
    else:
        raise AssertionError("길이 1 벡터가 통과했다")
    finally:
        os.unlink(p)


def test_base_cfg_not_mutated():
    base = _load(CFG_DEFAULT)
    before = list(base["attach"]["r1"])
    p = _write(_yaml_override())
    try:
        c = load_cfg(base, geometry=p)
    finally:
        os.unlink(p)
    assert base["attach"]["r1"] == before, "원본 설정이 바뀌었다"
    assert c["attach"]["r1"] == R1_NEW


def test_summary_line():
    s = geometry_summary(load_cfg())
    for k in ("부착기하", "간격", "FRD", "기하중심"):
        assert k in s, s
