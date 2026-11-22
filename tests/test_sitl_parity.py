# -*- coding: utf-8 -*-
"""동일코드 증명 — **같은 제어기, 다른 어댑터, 같은 숫자** (S6).

무엇을 재는가
-------------
`nuc/` 의 `NucController` 는 전송을 모른다. 그것을 부르는 어댑터가 둘 있다.

| 경로 | 어댑터 | 전송 |
|---|---|---|
| inproc 하네스 | `nodes/node_c_nuc.py::NucNode` | 딕셔너리 토픽 (**상태 float32**) |
| **SITL / 실물** | `sitl/nuc_nodes.py::{RlPolicyNode, LatchQpNode}` | **`rclpy` 토픽** |

`tests/test_same_code_both_paths.py` 는 **정적**으로 "`nuc/` 가 전송을
import 하지 않는다"를 보이고, 이 파일은 **수치로** "두 어댑터가 같은 τ 를
낸다"를 보인다. 둘이 합쳐져야 "가상과 실물의 차이는 전송계층과 플랜트뿐"이
주장이 아니라 측정이 된다. (`sitl/nuc_nodes.py` 머리말이 가리키는 파일이다.)

★ 시험이 **두 개로 갈라진 이유** — 실측으로 알게 된 것
-------------------------------------------------------
처음에는 "두 경로가 비트까지 같다"를 걸었는데 **τ_d 가 2.503e-05 N 달랐다.**
원인은 어댑터가 아니라 **inproc 전송이다**: `iface/transport.py:94` 의
`publish` 가 `msg.to_wire()` 를 불러 `PontoonState` 를 **float32 로
양자화**한다 (`iface/messages.py` `STRUCT = "<Q10fB"`). 반면 SITL 경로의
`52501` 은 shim 메시지가 `float32` 필드를 **파이썬 double 에 담으므로**
(의도된 차이, `shim/tug_interfaces/msg`) 양자화가 없다.

→ 그래서 **효과를 분리해 따로 잰다.**

1. `test_two_adapters_same_tau_exactly` — **같은 비트**를 두 경로에 먹인다
   (상태·부착점을 미리 float32 로 눌러서 `to_wire()` 가 무연산이 되게 한다).
   남는 차이는 **어댑터 코드뿐**이므로 **정확히 0** 이어야 한다. 0 이 아니면
   어느 한쪽 어댑터가 수식을 들고 있다는 뜻이다 (= 설계 위반).
2. `test_inproc_float32_state_is_conservative` — 양자화를 **켠 채로** 재서
   그 크기를 기록한다. inproc 하네스가 실물보다 **더 나쁜 입력**으로 돈다는
   뜻이므로 그 합격판정은 **보수적**이다. 이것은 결함이 아니라 성질이다.

τ_d 자체는 두 경로 모두 양자화되지 않는다 — inproc 은
`tick_qp(from_topic=False)` 로 토픽(`<Qfff`)을 우회하고, SITL 의
`/rl/tau_d` 는 **NUC 내부 ROS 토픽이고 `.msg` 가 float64** 다 (와이어가 아니다).

★ **타이머를 돌리지 않는다.** 콜백을 직접 부르고 구독큐를 손으로 비운다
(`Subscription._drain` 은 콜백을 동기 호출한다). 그래서 이 시험은 벽시계와
무관하게 **결정론적**이다 — `--speed` 류의 레이트 교란이 끼어들 수 없다.
"""
from __future__ import annotations

import numpy as np

import _common  # noqa: F401  (sys.path 설정)

from sitl import bootstrap

bootstrap.setup()                       # shim/ · vendor/ 를 sys.path 에 올린다

import rclpy                                                   # noqa: E402
from rclpy.node import Node                                    # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy            # noqa: E402

from tug_interfaces.msg import MothershipState, TugCommand     # noqa: E402

from iface.messages import PontoonState                        # noqa: E402
from iface.transport import make_transport                     # noqa: E402
from nodes.node_c_nuc import NucNode                           # noqa: E402
from nodes.topics import TOPIC_CMD, TOPIC_STATE                # noqa: E402
from nuc.controller import NucController                       # noqa: E402
from sitl.nuc_nodes import (TOPIC_CMD_OUT, TOPIC_STATE_IN,     # noqa: E402
                            LatchQpNode, RlPolicyNode)

N_STEPS = 40
GOAL = (0.0, 0.0, 0.0)

# inproc 양자화가 만드는 τ 차이의 상한. 실측 2.5e-05 N 대비 40 배 여유.
# 이 값을 넘으면 "보수적"이 아니라 **다른 궤적**이라는 뜻이다.
Q32_TAU_TOL_N = 1.0e-3


def _q32(a):
    """float32 로 눌렀다가 되돌린다 → `to_wire()` 가 무연산이 되는 값."""
    return np.asarray(np.asarray(a, dtype=np.float32), dtype=np.float64)


def _qos():
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class _Driver(Node):
    """52501 을 밀어 넣고 52503 직전의 `TugCommand` 를 받아 둔다."""

    def __init__(self) -> None:
        super().__init__("parity_driver")
        self.pub = self.create_publisher(MothershipState, TOPIC_STATE_IN,
                                         _qos())
        self.got = []
        self.create_subscription(TugCommand, TOPIC_CMD_OUT,
                                 self._on_cmd, _qos())

    def _on_cmd(self, m: TugCommand) -> None:
        self.got.append(np.array([m.tau_x_one, m.tau_y_one, m.tau_n_one,
                                  m.tau_x_two, m.tau_y_two, m.tau_n_two],
                                 dtype=np.float64))


def _drain(node: Node) -> int:
    """타이머는 건드리지 않고 구독큐만 비운다 (결정론 유지)."""
    return sum(s._drain() for s in list(node._subs))


def _states(n: int = N_STEPS):
    """운용범위 전체에서 뽑은 상태열. 두 경로에 **같은 값**을 먹인다."""
    rng = np.random.default_rng(7)
    out = []
    for _ in range(int(n)):
        out.append(np.array([
            rng.uniform(-0.4, 0.4),      # u
            rng.uniform(-0.4, 0.4),      # v
            rng.uniform(-0.25, 0.25),    # r
            rng.uniform(-3.0, 3.0),      # x
            rng.uniform(-3.0, 3.0),      # y
            rng.uniform(-0.6, 0.6),      # psi
        ], dtype=np.float64))
    return out


def _nominal_attach():
    ctl = NucController()
    ca = ctl.c["attach"]
    return (np.asarray(ca["r1"], dtype=np.float64),
            np.asarray(ca["r2"], dtype=np.float64))


def _run_inproc(seq, r1, r2):
    """경로 A: `NucNode` + inproc 토픽. `publish` 가 상태를 float32 로 누른다."""
    tp = make_transport("inproc")
    node = NucNode(tp, goal=np.asarray(GOAL, dtype=np.float64))
    u6s, taus = [], []
    for i, s6 in enumerate(seq):
        tp.publish(TOPIC_STATE, PontoonState.from_plant(i * 100000, s6, r1, r2))
        node.tick_policy(i * 100000)
        taus.append(np.asarray(node.tau_d, dtype=np.float64).copy())
        node.tick_qp(i * 100000, from_topic=False)   # ★ τ_d 양자화 지점 우회
        m = tp.latest(TOPIC_CMD)
        u6s.append(np.concatenate([m.tau_one(), m.tau_two()]))
    return np.asarray(u6s), np.asarray(taus)


def _run_rclpy(seq, r1=None, r2=None):
    """경로 B: `RlPolicyNode` + `LatchQpNode` + shim `rclpy` 토픽.

    `r1`/`r2` 를 주면 노드의 **입력**(설정 공칭값)만 같은 비트로 맞춘다.
    노드 코드는 건드리지 않는다 — 맞추는 것은 입력이고, 재는 것은 출력이다.
    """
    rclpy.init()
    ctl = NucController()
    rl = RlPolicyNode(ctl)
    if r1 is not None:
        rl.r1 = np.asarray(r1, dtype=np.float64)
        rl.r2 = np.asarray(r2, dtype=np.float64)
    qp = LatchQpNode(ctl)
    drv = _Driver()
    u6s, taus = [], []
    try:
        for i, s6 in enumerate(seq):
            st = MothershipState()
            st.timestamp = i * 100000
            st.u, st.v, st.r = float(s6[0]), float(s6[1]), float(s6[2])
            st.x, st.y, st.psi = float(s6[3]), float(s6[4]), float(s6[5])
            st.x_ref, st.y_ref, st.psi_ref = (float(GOAL[0]), float(GOAL[1]),
                                              float(GOAL[2]))
            st.scenario_mode, st.rc_on_off, st.on_off = 3, 0, 1
            drv.pub.publish(st)
            assert _drain(rl) == 1, "52501 이 rl_policy 에 1 개로 도착하지 않았다"
            rl._on_tick()
            taus.append(np.asarray(ctl.tau_d, dtype=np.float64).copy())
            assert _drain(qp) == 1, "/rl/tau_d 가 latch_qp 에 1 개로 도착하지 않았다"
            qp._on_tick()
            assert _drain(drv) == 1, "/tug_command 가 1 개로 도착하지 않았다"
            u6s.append(drv.got[-1])
    finally:
        for nd in (drv, qp, rl):
            try:
                nd.destroy_node()
            except Exception:                                   # noqa: BLE001
                pass
    return np.asarray(u6s), np.asarray(taus)


def test_two_adapters_same_tau_exactly():
    """같은 비트를 먹이면 inproc 어댑터와 `rclpy` 어댑터가 **비트까지 같다**.

    상태·부착점을 미리 float32 로 눌러 inproc `to_wire()` 를 무연산으로
    만든다 → 남는 차이는 어댑터 코드뿐이고, 그것은 0 이어야 한다.
    """
    r1, r2 = _nominal_attach()
    r1q, r2q = _q32(r1), _q32(r2)
    seq = [_q32(s) for s in _states()]

    u6_a, tau_a = _run_inproc(seq, r1q, r2q)
    u6_b, tau_b = _run_rclpy(seq, r1q, r2q)

    assert u6_a.shape == u6_b.shape == (len(seq), 6), \
        "표본수가 다르다: %r vs %r" % (u6_a.shape, u6_b.shape)
    d_tau = float(np.abs(tau_a - tau_b).max())
    d_u6 = float(np.abs(u6_a - u6_b).max())
    assert d_tau == 0.0, "tau_d 가 어댑터에 따라 달라졌다 (max %.3e)" % d_tau
    assert d_u6 == 0.0, "tau_i 가 어댑터에 따라 달라졌다 (max %.3e)" % d_u6
    assert float(np.abs(u6_a).max()) > 0.0, "둘 다 0 이면 아무것도 증명하지 못한다"
    return ("%d 스텝 x 6 성분, tau_d/tau_i 차이 **정확히 0**, |tau_i|max %.2f N"
            % (len(seq), float(np.abs(u6_a).max())))


def test_inproc_float32_state_is_conservative():
    """inproc 상태 float32 양자화의 **크기를 기록한다** (결함이 아니라 성질).

    같은 float64 상태열을 두 경로에 주면 inproc 만 `to_wire()` 로 눌린다.
    그 차이가 작고 유계임을 보이면 "inproc 합격판정은 실물보다 보수적"이
    말이 아니라 측정이 된다. 상한을 넘으면 보수적인 게 아니라 **다른
    궤적**이라는 뜻이므로 실패시킨다.
    """
    r1, r2 = _nominal_attach()
    seq = _states(20)

    _, tau_a = _run_inproc(seq, r1, r2)          # float32 로 눌린다
    _, tau_b = _run_rclpy(seq)                   # 설정 공칭값 float64 그대로

    d = float(np.abs(tau_a - tau_b).max())
    scale = float(np.abs(tau_b).max())
    assert d <= Q32_TAU_TOL_N, \
        "양자화 차이가 상한을 넘었다: %.3e N > %.1e N" % (d, Q32_TAU_TOL_N)
    assert d > 0.0, \
        "차이가 0 이면 양자화가 일어나지 않은 것이다 - 전제를 다시 확인하라"
    return ("inproc float32 양자화 -> tau_d 차이 %.2e N (|tau_d|max %.1f N, "
            "상대 %.1e, 상한 %.0e) -> 보수적"
            % (d, scale, d / max(scale, 1e-12), Q32_TAU_TOL_N))


def test_a_prev_evolves_identically():
    """정책이 들고 가는 유일한 상태 `a_prev` 도 두 경로에서 같아야 한다.

    ★ 관측 19차에 `a_prev` 가 들어 있어 **정책은 상태를 가진다.** 한 스텝만
    어긋나도 이후 전부 갈라지므로, 위 시험이 0 이라는 것은 `a_prev` 궤적이
    같다는 뜻이다. 여기서는 **제어기 직접호출 대 `rclpy` 노드**로 그 값을
    비교해 "노드가 끼어들어도 상태가 같다"를 따로 고정한다.
    """
    seq = _states(12)
    r1, r2 = _nominal_attach()
    ctl_a = NucController()
    ctl_a.set_goal(GOAL)
    a_hist = []
    for s6 in seq:
        ctl_a.policy_step(s6[0:3], s6[3:6], r1, r2)
        a_hist.append(np.asarray(ctl_a.a_prev, dtype=np.float64).copy())

    rclpy.init()
    ctl_b = NucController()
    node = RlPolicyNode(ctl_b)
    b_hist = []
    try:
        for i, s6 in enumerate(seq):
            st = MothershipState()
            st.timestamp = i * 100000
            st.u, st.v, st.r = float(s6[0]), float(s6[1]), float(s6[2])
            st.x, st.y, st.psi = float(s6[3]), float(s6[4]), float(s6[5])
            st.x_ref, st.y_ref, st.psi_ref = GOAL
            st.scenario_mode, st.rc_on_off, st.on_off = 3, 0, 1
            node._on_state(st)
            node._on_tick()
            b_hist.append(np.asarray(ctl_b.a_prev, dtype=np.float64).copy())
    finally:
        try:
            node.destroy_node()
        except Exception:                                       # noqa: BLE001
            pass

    d = float(np.abs(np.asarray(a_hist) - np.asarray(b_hist)).max())
    assert d == 0.0, "a_prev 궤적이 갈라졌다 (max %.3e)" % d
    return "a_prev %d 스텝 동일 (차이 정확히 0)" % len(seq)
