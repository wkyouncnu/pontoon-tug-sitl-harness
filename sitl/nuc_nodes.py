# -*- coding: utf-8 -*-
"""harness_sitl/sitl/nuc_nodes.py — 실물 NUC 의 ROS 2 노드 **그 자체**.

이 파일의 지위 — ★ 다른 어댑터와 다르다
---------------------------------------
`nodes/node_c_nuc.py` 는 **하네스 전송(`transport.latest/publish`)** 어댑터고,
이 파일은 **진짜 `rclpy` 어댑터**다. 둘은 같은 `NucController` 를 부른다.

| 경로 | 어댑터 | 전송 |
|---|---|---|
| inproc 하네스 | `nodes/node_c_nuc.py::NucNode` | 딕셔너리 토픽 |
| **SITL / 실물** | **이 파일** | **`rclpy` + UDP(MAVLink)** |

→ **같은 제어기, 다른 어댑터, 같은 숫자.** 그것을 `tests/test_sitl_parity.py`
가 재고, 그 등식이 이 하네스의 존재 이유다.

왜 `nuc/` 안에 두지 않는가
--------------------------
`tests/test_same_code_both_paths.py::FORBIDDEN` 에 **`rclpy` 가 들어 있다.**
`nuc/` 가 `rclpy` 를 import 하면 "`nuc/` 는 전송을 모른다"가 거짓이 되고 그
시험이 깨진다. 그래서 **어댑터는 `sitl/`, 수식은 `nuc/`** 로 갈라 둔다.
실물 배치에서는 이 파일이 ROS 2 패키지의 노드 소스가 된다
(`setup.py` 의 `console_scripts`) — `nuc/` 는 그대로 따라간다.

노드 2개로 쪼갠 이유
--------------------
실물은 `rl_node`(10 Hz) 와 `latch_qp_node`(50 Hz) 가 **분리된 두 노드**다.
그래서 τ_d 가 토픽을 건넌다. 여기서도 그렇게 둔다.

★ **τ_d 는 와이어로 나가지 않는다.** NUC 내부 ROS 토픽이고 `.msg` 가
**float64** 다 → **양자화가 없다.** inproc 하네스는 같은 자리에서 float32 로
자르는데(`MothershipData.from_tau`), 그것은 실물에 **없는 오차를 더한
보수적 설정**이다 (실측 ≤ 7.38e-06 N). 기준선을 무효화하지 않기 위해
inproc 쪽은 그대로 두고, 이 경로가 진실을 보인다. → `docs/SITL.md`.

제어기 인스턴스를 공유하는 이유
-------------------------------
두 노드가 **한 `NucController` 를 공유**한다. 실물에서 두 프로세스로 갈라도
**숫자는 같다**: 정책이 들고 가는 상태는 `a_prev` 뿐이고 QP 가 들고 가는
상태는 `res`·`marg`(출력) 뿐이라 **서로 읽지 않는다.** 한 인스턴스로 두면
하네스가 ONNX 세션을 두 번 띄우지 않아도 된다.

안전 — 새 장치를 만들지 않는다
------------------------------
상태가 끊기면 **발행을 멈춘다.** 그러면 `tug_tx.py` 의 `input_timeout_s`
(0.5 s)가 깨어나 **τ=0** 을 보낸다. 그것이 체인의 **현재 유일한 안전장치**
이므로(`CLAUDE.md` §9 2026-11-22 오후) 하네스가 다른 장치를 끼워 넣어
그 경로를 가려서는 안 된다. → `rc_on_off`/`on_off` 수동개입은 제어기
안(`policy_step`)에서 이미 τ=0 으로 처리된다.
"""
from __future__ import annotations

import time
from typing import Any, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tug_interfaces.msg import (AllocationDebug, MothershipData,
                                MothershipState, TugCommand)

TOPIC_STATE_IN = "/mothership/state"      # vendor/mothership_rx 가 발행 (52501)
TOPIC_TAU_D = "/rl/tau_d"                 # NUC 내부. 와이어가 아니다
TOPIC_CMD_OUT = "/tug_command"            # vendor/tug_tx 가 구독 (→ 52503)
TOPIC_DBG_OUT = "/allocation_debug"

POLICY_RATE_HZ = 10.0
QP_RATE_HZ = 50.0
STATE_TIMEOUT_S = 0.5                     # tug_tx.input_timeout_s 와 같은 값


def _qos(depth: int = 10) -> QoSProfile:
    """`best_effort` — 벤더 노드와 같은 설정 (`tug_bridge.yaml`)."""
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)


class RlPolicyNode(Node):
    """10 Hz 정책 노드. `/mothership/state` → `/rl/tau_d`."""

    def __init__(self, ctl: Any, rate_hz: float = POLICY_RATE_HZ,
                 **kwargs) -> None:
        super().__init__("rl_policy", **kwargs)
        self.ctl = ctl
        # ★ 52501 에는 부착점이 없다 → **설정의 공칭값**을 쓴다.
        #   실물에서도 이 값이다. inproc 하네스만 난수화된 참값을 넘긴다.
        ca = self.ctl.c["attach"]
        self.r1 = np.asarray(ca["r1"], dtype=np.float64)
        self.r2 = np.asarray(ca["r2"], dtype=np.float64)

        self.latest: Optional[MothershipState] = None
        self.latest_mono = 0.0
        self.n_state = 0
        self.n_pub = 0
        self.n_stale = 0
        self.n_override = 0
        self._stale = False

        self.pub = self.create_publisher(MothershipData, TOPIC_TAU_D, _qos())
        self.create_subscription(MothershipState, TOPIC_STATE_IN,
                                 self._on_state, _qos())
        self.create_timer(1.0 / float(rate_hz), self._on_tick)

    # -- 구독: 메시지에서 숫자 꺼내기만 한다 --------------------------------
    def _on_state(self, msg: MothershipState) -> None:
        self.latest = msg
        self.latest_mono = time.monotonic()
        self.n_state += 1

    # -- 10 Hz 틱 -----------------------------------------------------------
    def _on_tick(self) -> None:
        st = self.latest
        if st is None:
            return
        if time.monotonic() - self.latest_mono > STATE_TIMEOUT_S:
            # 발행을 멈춘다 → tug_tx 의 input_timeout 이 τ=0 을 보낸다
            self.n_stale += 1
            if not self._stale:
                self._stale = True
                self.get_logger().warn(
                    "/mothership/state 끊김 → 발행 중단 (tug_tx 가 τ=0 을 보낸다)")
            return
        if self._stale:
            self._stale = False
            self.get_logger().info("/mothership/state 복구 → 발행 재개")

        # 목표자세는 **메시지에서 받는다** (하드코딩 금지, 사용자 확정)
        self.ctl.set_goal((st.x_ref, st.y_ref, st.psi_ref))

        if int(st.rc_on_off) != 0 or int(st.on_off) == 0:
            self.n_override += 1

        nu = np.array([st.u, st.v, st.r], dtype=np.float64)
        eta = np.array([st.x, st.y, st.psi], dtype=np.float64)
        tau_d, _a = self.ctl.policy_step(
            nu, eta, self.r1, self.r2,
            rc_on_off=int(st.rc_on_off), on_off=int(st.on_off))

        out = MothershipData()
        out.timestamp = int(st.timestamp)
        out.tau_x = float(tau_d[0])      # float64 — 양자화 없음
        out.tau_y = float(tau_d[1])
        out.tau_n = float(tau_d[2])
        self.pub.publish(out)
        self.n_pub += 1


class LatchQpNode(Node):
    """50 Hz Latch-aware QP 노드. `/rl/tau_d` → `/tug_command`.

    ⚠️ **QP 는 RL 로 대체하지 않는다** (사용자 확정). 전자석 이탈을 막는
    유일한 하드 보장이다.
    """

    def __init__(self, ctl: Any, rate_hz: float = QP_RATE_HZ,
                 **kwargs) -> None:
        super().__init__("latch_qp", **kwargs)
        self.ctl = ctl
        self.latest: Optional[MothershipData] = None
        self.latest_mono = 0.0
        self.n_tau = 0
        self.n_pub = 0
        self.n_stale = 0
        self._stale = False

        self.pub = self.create_publisher(TugCommand, TOPIC_CMD_OUT, _qos())
        self.dbg = self.create_publisher(AllocationDebug, TOPIC_DBG_OUT, _qos())
        self.create_subscription(MothershipData, TOPIC_TAU_D,
                                 self._on_tau, _qos())
        self.create_timer(1.0 / float(rate_hz), self._on_tick)

    def _on_tau(self, msg: MothershipData) -> None:
        self.latest = msg
        self.latest_mono = time.monotonic()
        self.n_tau += 1

    def _on_tick(self) -> None:
        m = self.latest
        if m is None:
            return
        if time.monotonic() - self.latest_mono > STATE_TIMEOUT_S:
            self.n_stale += 1
            if not self._stale:
                self._stale = True
                self.get_logger().warn(
                    "/rl/tau_d 끊김 → 발행 중단 (tug_tx 가 τ=0 을 보낸다)")
            return
        if self._stale:
            self._stale = False
            self.get_logger().info("/rl/tau_d 복구 → 발행 재개")

        tau_d = np.array([m.tau_x, m.tau_y, m.tau_n], dtype=np.float64)
        u6, tau_a = self.ctl.allocate(tau_d)
        res, marg = self.ctl.res, self.ctl.marg

        cmd = TugCommand()
        cmd.timestamp = int(m.timestamp)
        (cmd.tau_x_one, cmd.tau_y_one, cmd.tau_n_one,
         cmd.tau_x_two, cmd.tau_y_two, cmd.tau_n_two) = (
            float(u6[0]), float(u6[1]), float(u6[2]),
            float(u6[3]), float(u6[4]), float(u6[5]))
        self.pub.publish(cmd)
        self.n_pub += 1

        d = AllocationDebug()
        d.timestamp = int(m.timestamp)
        d.tau_x, d.tau_y, d.tau_n = float(tau_d[0]), float(tau_d[1]), float(tau_d[2])
        d.tau_real_x, d.tau_real_y, d.tau_real_n = (
            float(tau_a[0]), float(tau_a[1]), float(tau_a[2]))
        d.error_x, d.error_y, d.error_n = (
            float(res[0]), float(res[1]), float(res[2]))
        d.ok = True
        d.saturated = bool(self.ctl.saturated)
        d.solve_time_us = float(self.ctl.qp_us)
        d.load_x_one, d.load_y_one = float(u6[0]), float(u6[1])
        d.load_x_two, d.load_y_two = float(u6[3]), float(u6[4])
        self.dbg.publish(d)


def spin_in_thread(node: Node) -> Any:
    """노드 하나를 자기 스레드에서 돈다 — shim 실행기는 **노드별**이다."""
    import threading
    th = threading.Thread(target=_spin_quiet, args=(node,), daemon=True)
    th.start()
    return th


def _spin_quiet(node: Node) -> None:
    try:
        rclpy.spin(node)
    except Exception:
        # 종료 시 context 가 먼저 내려가면 조용히 끝낸다 (벤더 main 과 같은 처리)
        pass
