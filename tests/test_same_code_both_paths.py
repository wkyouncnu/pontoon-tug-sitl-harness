# -*- coding: utf-8 -*-
"""같은 `nuc/` 코드가 가상·실물 두 경로에서 같은 결과를 내는지 고정하는 시험.

이 시험이 존재하는 이유
-----------------------
이 하네스의 존재 이유 자체가 **"가상과 실물의 차이는 전송계층과 플랜트
두 가지뿐이다"** 라는 주장이다 (사용자 확정). 주장이 문서에만 있으면
리팩터링 한 번에 조용히 깨진다. 그래서 **코드로 고정한다.**

두 경로의 정의
--------------
| 경로 | 호출 방식 | 실물에서 누구 |
|---|---|---|
| **A (production 직접)** | `NucController` 를 plain numpy 로 직접 호출 | **실물 NUC 의 ROS 2 노드** |
| **B (하네스 경유)** | `NucNode` + `transport` + 메시지 | **가상 하네스** |

경로 A 는 전송계층을 전혀 모른다. 경로 B 는 같은 `NucController` 를
토픽으로 감싼 것이다. 따라서 **같은 숫자를 넣으면 같은 숫자가 나와야
한다** — 그것을 `test_node_and_controller_agree_exactly` 가 재고,
전송계층이 더하는 유일한 차이(float32 양자화)의 크기를
`test_transport_quantization_is_the_only_difference` 가 재서 보고한다.

경계 검사
---------
`nuc/` 가 전송계층이나 `plant/` 를 import 하면 "그대로 배포한다"가
거짓이 된다. 두 시험이 **import 줄만** 훑어서 그것을 막는다
(docstring 에는 그 단어들이 당연히 나오므로 본문 전체를 훑으면 안 된다).

관측 분포
---------
난수 상태는 정책을 포화(clip ±1)로 보내 **두 경로가 자동으로 같아져**
시험이 공허해진다. 그래서 목표 근방의 좁은 띠에서 상태를 만들고,
메시지로 **포화되지 않은 스텝의 비율**을 함께 보고한다.
"""
from __future__ import annotations

import os
import re

import numpy as np

import _common  # noqa: F401

from iface.messages import MothershipData, PontoonState        # noqa: E402
from iface.transport import make_transport                      # noqa: E402
from nodes.node_c_nuc import NucNode                            # noqa: E402
from nodes.node_d_esp32 import Esp32LinkNode                    # noqa: E402
from nodes.topics import TOPIC_STATE, TOPIC_TAU_D               # noqa: E402
from nuc.controller import NucController                        # noqa: E402

from env.pontoon_env import _load, CFG_DEFAULT                  # noqa: E402

NUC_DIR = os.path.join(_common.HARNESS, "nuc")

# import 줄에 나타나면 "그대로 배포"가 거짓이 되는 모듈들
FORBIDDEN = ("iface", "nodes", "plant", "rclpy", "socket", "struct",
             "pymavlink", "mavlink", "stable_baselines3", "torch",
             "gymnasium", "gym")

N_STEP = 24          # 비교 스텝 수 (정책이 `a_prev` 를 들고 있어 상태가 누적된다)
TOL_EXACT = 0.0      # 같은 숫자 입력 → 비트 단위 동일을 기대한다
TOL_Q = 1.0e-2       # float32 양자화가 `u6` [N] 에 남기는 차이의 상한

_IMPORT_RE = re.compile(
    r"^\s*(?:from\s+([A-Za-z_][A-Za-z0-9_.]*)"
    r"|import\s+([A-Za-z_][A-Za-z0-9_.]*))")
# ASCII 식별자만 — 한글 docstring 의 "import 하지 않는다" 가 걸리기 때문이다


def _import_roots(path):
    """파일의 **import 줄에서만** 최상위 모듈명을 모은다."""
    roots = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            m = _IMPORT_RE.match(line)
            if not m:
                continue
            mod = m.group(1) or m.group(2)
            roots.append(mod.split(".")[0].lstrip("."))
    return roots


def _nuc_files():
    return sorted(os.path.join(NUC_DIR, f) for f in os.listdir(NUC_DIR)
                  if f.endswith(".py"))


# ---------------------------------------------------------------- 경계 검사
def test_nuc_imports_no_transport():
    """`nuc/` 는 전송계층을 import 하지 않는다 (= 전송 무관)."""
    bad = []
    for p in _nuc_files():
        for root in _import_roots(p):
            if root in FORBIDDEN:
                bad.append("%s: %s" % (os.path.basename(p), root))
    assert not bad, "nuc/ 가 금지 모듈을 import 한다: %s" % ", ".join(bad)
    return "nuc/*.py %d개, 금지 import 0건 (%s)" % (
        len(_nuc_files()), "|".join(FORBIDDEN[:4]) + "|...")


def test_nuc_deps_are_numerics_only():
    """`nuc/` 의 외부 의존성은 numpy·onnxruntime·quadprog·env(설정) 뿐이다."""
    allowed = {"__future__", "os", "sys", "time", "math", "copy", "typing",
               "dataclasses",
               "numpy", "np", "onnxruntime", "quadprog", "yaml",
               "env", "latch_qp", "nuc"}
    seen = set()
    for p in _nuc_files():
        seen |= set(_import_roots(p))
    extra = sorted(seen - allowed)
    assert not extra, "예상 밖 의존성: %s" % extra
    return "의존성 %s" % sorted(seen)


# ------------------------------------------------- production 코드 단독 동작
def test_controller_standalone_no_transport():
    """`NucController` 를 전송계층 **없이** 돌린다 — 실물 NUC 의 호출 방식."""
    c = _load(CFG_DEFAULT)
    r1 = np.asarray(c["attach"]["r1"], dtype=np.float64)
    r2 = np.asarray(c["attach"]["r2"], dtype=np.float64)
    ctl = NucController(goal=np.zeros(3))
    ctl.reset()

    rng = np.random.default_rng(11)
    for _ in range(N_STEP):
        nu = rng.uniform(-0.08, 0.08, 3)
        eta = np.array([rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4),
                        rng.uniform(-0.08, 0.08)])
        tau_d, _a = ctl.policy_step(nu, eta, r1, r2)
        u6, _ta = ctl.allocate(tau_d)
        assert np.all(np.isfinite(u6)) and u6.shape == (6,)
    return ("전송계층 없이 %d스텝 동작, |u6|max %.2f N·N·N·m"
            % (N_STEP, float(np.max(np.abs(u6)))))


# ------------------------------------------- 두 경로가 같은 숫자를 내는지
def _paths(quantize_direct: bool):
    """경로 A(직접)·B(노드)를 같은 상태열로 돌려 `u6` 를 모은다.

    `quantize_direct=True` 면 경로 A 에도 **전송 양자화와 같은 반올림**을
    적용한다 → 두 경로는 **비트 단위로 같아야 한다**.
    `False` 면 경로 A 는 순수 float64 → 차이는 양자화뿐이다.
    """
    c = _load(CFG_DEFAULT)
    r1 = np.asarray(c["attach"]["r1"], dtype=np.float64)
    r2 = np.asarray(c["attach"]["r2"], dtype=np.float64)
    goal = np.zeros(3)

    tp = make_transport("inproc")
    node = NucNode(tp, goal=goal)
    node.reset()
    ctl = NucController(goal=goal)
    ctl.reset()

    rng = np.random.default_rng(5)
    ua, ub, nclip = [], [], 0
    for k in range(N_STEP):
        t_us = int(k * 20000)
        nu = rng.uniform(-0.08, 0.08, 3)
        eta = np.array([rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4),
                        rng.uniform(-0.08, 0.08)])
        s6 = np.array([nu[0], nu[1], nu[2], eta[0], eta[1], eta[2]])

        # --- 경로 B: 메시지 → 토픽 → 노드 ---
        tp.publish(TOPIC_STATE, PontoonState.from_plant(t_us, s6, r1, r2))
        a = node.tick_policy(t_us)
        assert a is not None, "상태 메시지가 노드에 도착하지 않았다"
        if np.any(np.abs(a) >= 1.0 - 1e-9):
            nclip += 1
        ub.append(node.tick_qp(t_us, from_topic=True).copy())

        # --- 경로 A: production 직접 호출 ---
        if quantize_direct:
            # 전송이 하는 일과 **같은 반올림**을 손으로 적용한다
            st = tp.latest(TOPIC_STATE)
            nu_a, eta_a = st.nu(), st.eta()
            r1_a, r2_a = st.r1(), st.r2()
        else:
            nu_a, eta_a, r1_a, r2_a = nu, eta, r1, r2
        tau_d, _ = ctl.policy_step(nu_a, eta_a, r1_a, r2_a)
        if quantize_direct:
            tau_d = np.asarray(MothershipData.from_tau(t_us, tau_d).tau(),
                               dtype=np.float64)
        ua.append(ctl.allocate(tau_d)[0].copy())

    return np.asarray(ua), np.asarray(ub), nclip


def test_node_and_controller_agree_exactly():
    """같은 숫자를 넣으면 **비트 단위로 같은 `u6`** 가 나온다."""
    ua, ub, nclip = _paths(quantize_direct=True)
    err = float(np.max(np.abs(ua - ub)))
    assert err <= TOL_EXACT, "두 경로 불일치 max %.3e (기대 0)" % err
    return ("경로 A(직접) == 경로 B(노드) 오차 %.1e, %d스텝 "
            "(비포화 %d/%d)" % (err, len(ua), len(ua) - nclip, len(ua)))


def test_transport_quantization_is_the_only_difference():
    """전송계층이 더하는 차이는 **float32 양자화뿐**이고 그 크기를 보고한다."""
    ua, ub, nclip = _paths(quantize_direct=False)
    err = float(np.max(np.abs(ua - ub)))
    assert err < TOL_Q, "양자화 차이가 상한 %.1e N 을 넘었다: %.3e" % (TOL_Q, err)
    assert nclip < len(ua), "모든 스텝이 포화됐다 — 시험이 공허하다"
    return ("float64 직접 vs 전송경유 `u6` 차이 max %.2e N (상한 %.0e), "
            "비포화 %d/%d" % (err, TOL_Q, len(ua) - nclip, len(ua)))


# --------------------------------------- 어댑터가 상태를 복제하지 않는지
def test_adapters_hold_no_duplicate_state():
    """위임한 상태는 전부 **읽기전용 property** 다 (어댑터가 복사본을 못 든다).

    평범한 속성으로 두면 어댑터가 낡은 값을 들고 조용히 틀린 값을 보고한다.
    실제로 `Esp32LinkNode.n_sat` 이 그 사고가 날 자리였다 (`reset()` 이
    `self.n_sat = 0` 으로 배분기의 카운터를 가려 영원히 0 을 보고한다).
    """
    c_names = ["c", "qp", "policy", "tau_max", "goal", "a_prev", "res",
               "marg", "tau_d", "u6", "last_obs", "n_policy", "n_qp", "qp_us"]
    d_names = ["n_sat", "B", "Binv", "f_max", "f_min", "pwm_quantize"]
    bad = []
    for cls, names in ((NucNode, c_names), (Esp32LinkNode, d_names)):
        for n in names:
            p = getattr(cls, n, None)
            if not isinstance(p, property):
                bad.append("%s.%s" % (cls.__name__, n))
            elif p.fset is not None:
                bad.append("%s.%s (쓰기 가능)" % (cls.__name__, n))
    assert not bad, "위임 상태가 property 가 아니다: %s" % bad
    return ("NucNode %d개 + Esp32LinkNode %d개 전부 읽기전용 property"
            % (len(c_names), len(d_names)))
