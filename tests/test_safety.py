# -*- coding: utf-8 -*-
"""harness_sitl/tests/test_safety.py — S5③ 안전거동 시험 (**실제 전송경로**).

무엇을 재는가
-------------
`CLAUDE.md` §9 2026-11-22 오후: **터그 Pixhawk 에는 수신 타임아웃이 없어
마지막 값을 유지한다.** 그래서 체인의 **현재 유일한 안전장치**는
`vendor/tug_bridge/tug_tx.py` 가 τ=0 을 보내는 것이다. 이 시험은 그 장치가
**정말 발동하는지**를 와이어에서(`52503` 바이트) 확인한다.

| 사례 | 무엇이 발동해야 하는가 | 함수 |
|---|---|---|
| ① `rc_on_off = 1` | 제어기가 **정책을 돌리지 않고** τ=0 | `test_override_rc_on_off` |
| ② `on_off = 0` | 같음 + `a_prev` 초기화 | `test_override_on_off_resets_a_prev` |
| ③ 입력 0.5 s 단절 | `tug_tx` 가 τ=0 을 **계속** 송신 | `test_input_timeout_sends_zero` |
| ④ 한 필드 NaN/inf | **메시지 전체**를 τ=0 으로 대체 | `test_nonfinite_replaces_whole_message` |
| ⑤ 종료 | τ=0 을 `shutdown_zero_count`(10) 회 | `test_shutdown_burst_ten_zeros` |
| ⑥ 신호 중복 | 중복 SIGINT/SIGTERM 에도 버스트 1회 | `test_duplicate_signals_single_burst` |
| ⑦ 상태 단절 | NUC 노드가 **발행을 멈춘다**(새 장치 금지) | `test_nuc_nodes_stop_publishing_when_stale` |

★ ⑥ 의 `SIGHUP` 은 **진짜 전달이 아니다**
-------------------------------------------
Windows 는 `SIGHUP` 을 보낼 수 없다. `sitl/bootstrap.py::raise_emulated` 는
**등록된 핸들러를 직접 호출**한다. 따라서 검증되는 것은 "τ=0 ×10 송신 로직"
이고, **"SSH 끊김이 그 로직을 실제로 깨우는가"는 가상으로 검증되지 않는다.**
보고서(§안전)에 그대로 적는다 — 통과를 과장하지 않는다.
"""
from __future__ import annotations

import io
import math
import os
import sys
import time

import numpy as np

import _common  # noqa: F401  (sys.path)

HARNESS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.join(HARNESS, "sitl") not in sys.path:
    sys.path.insert(0, os.path.join(HARNESS, "sitl"))

import bootstrap                                      # noqa: E402
bootstrap.setup()                                     # 신호 대역 + 경로

import signal                                         # noqa: E402

import rclpy                                          # noqa: E402
from rclpy.node import Node                           # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy   # noqa: E402

import yaml                                           # noqa: E402

from tug_bridge.tug_tx import TugTx                   # noqa: E402
from tug_interfaces.msg import MothershipData, TugCommand  # noqa: E402

from sitl.wire import TugRx                           # noqa: E402

CFG = os.path.join(HARNESS, "config", "tug_bridge_loopback.yaml")

# 시험 전용 포트 — SITL(14601/14602/14651/14652)과 겹치지 않게 띄워 둔다
TX_BIND_PORT = 14691
TUG_PORT = 14692
TOPIC = "/tug_command_test"


def _overrides(**patch):
    """`tug1_tx` 설정을 읽어 시험용 포트·토픽만 바꿔 끼운다."""
    with io.open(CFG, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    params = dict(doc["tug1_tx"]["ros__parameters"])
    params.update({"topic": TOPIC, "bind_port": TX_BIND_PORT,
                   "remote_ip": "127.0.0.1", "remote_port": TUG_PORT})
    params.update(patch)
    return [(k, v) for k, v in params.items()]


class _CmdPub(Node):
    """`/tug_command_test` 발행자. 하네스쪽 가짜 `latch_qp` 다."""

    def __init__(self):
        super().__init__("cmd_pub_test")
        self.pub = self.create_publisher(
            TugCommand, TOPIC,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def send(self, six, t_us=1):
        m = TugCommand()
        m.timestamp = int(t_us)
        (m.tau_x_one, m.tau_y_one, m.tau_n_one,
         m.tau_x_two, m.tau_y_two, m.tau_n_two) = [float(v) for v in six]
        self.pub.publish(m)


class _Rig(object):
    """`TugTx` 1대 + 터그 수신기 1대. `with` 로 쓴다."""

    def __init__(self, **patch):
        rclpy.init(args=None)
        self.rx = TugRx(TUG_PORT, which=1, accepted_sysid=250)
        self.tx = TugTx(parameter_overrides=_overrides(**patch))
        self.pub = _CmdPub()
        from sitl.nuc_nodes import spin_in_thread
        self.th = [spin_in_thread(self.tx), spin_in_thread(self.pub)]
        time.sleep(0.2)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        try:
            self.tx.destroy_node()
        except Exception:
            pass
        try:
            self.pub.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
        self.rx.close()
        return False

    def wait_tau(self, pred, timeout_s=2.0, period_s=0.01):
        """조건을 만족하는 τ 가 **와이어로** 올 때까지 기다린다."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            self.rx.drain()
            if pred(self.rx.tau()):
                return True, self.rx.tau()
            time.sleep(period_s)
        self.rx.drain()
        return False, self.rx.tau()


def _is_zero(tau, tol=0.0):
    return all(abs(float(v)) <= tol for v in tau)


# ===== ①② 수동개입 — production 제어기 안의 분기 ==========================

def test_override_rc_on_off():
    """`rc_on_off = 1` → 정책을 돌리지 않고 τ=0. ★ 새 코드가 아니다."""
    from nuc.controller import NucController
    ctl = NucController()
    nu = np.array([0.2, -0.05, 0.03])
    eta = np.array([1.0, -0.5, 0.2])
    r1 = np.asarray(ctl.c["attach"]["r1"], dtype=float)
    r2 = np.asarray(ctl.c["attach"]["r2"], dtype=float)

    tau_n, _ = ctl.policy_step(nu, eta, r1, r2, rc_on_off=0, on_off=1)
    if _is_zero(tau_n):
        raise AssertionError("공칭에서 τ 가 0 이면 시험이 의미없다")
    n0 = int(ctl.n_override)

    tau_o, a_o = ctl.policy_step(nu, eta, r1, r2, rc_on_off=1, on_off=1)
    if not _is_zero(tau_o):
        raise AssertionError("rc_on_off=1 인데 τ 가 0 이 아니다: %s" % (tau_o,))
    if not _is_zero(a_o):
        raise AssertionError("rc_on_off=1 인데 a_prev 가 0 이 아니다")
    if ctl.last_obs is not None:
        raise AssertionError("수동개입인데 관측을 만들었다 (정책이 돌았다)")
    if int(ctl.n_override) != n0 + 1:
        raise AssertionError("n_override 가 증가하지 않았다")
    return "공칭 |τ|=%.3f → 개입 τ=0, 정책 미실행" % float(np.linalg.norm(tau_n))


def test_override_on_off_resets_a_prev():
    """`on_off = 0`(자율해제) → τ=0 + `a_prev` 초기화."""
    from nuc.controller import NucController
    ctl = NucController()
    nu = np.array([0.3, 0.1, -0.02])
    eta = np.array([-0.8, 0.4, -0.1])
    r1 = np.asarray(ctl.c["attach"]["r1"], dtype=float)
    r2 = np.asarray(ctl.c["attach"]["r2"], dtype=float)

    for _ in range(3):
        ctl.policy_step(nu, eta, r1, r2)
    if _is_zero(ctl.a_prev):
        raise AssertionError("세 스텝을 돌렸는데 a_prev 가 0 이다")

    tau, a = ctl.policy_step(nu, eta, r1, r2, on_off=0)
    if not (_is_zero(tau) and _is_zero(a) and _is_zero(ctl.a_prev)):
        raise AssertionError("on_off=0 인데 τ·a_prev 가 0 이 아니다")
    return "a_prev 초기화 확인 (에피소드 경계 경로와 동일)"


# ===== ③ 입력 단절 → τ=0 (와이어에서 확인) ================================

def test_input_timeout_sends_zero():
    """0.5 s 입력단절 → `tug_tx` 가 τ=0 을 **계속** 보낸다."""
    with _Rig() as rig:
        for _ in range(12):                     # 정상값 공급
            rig.pub.send([3.0, -2.0, 0.5, 1.5, 2.5, -0.5])
            time.sleep(0.02)
        ok, tau = rig.wait_tau(lambda t: abs(t[0] - 3.0) < 1e-6, 2.0)
        if not ok:
            raise AssertionError("정상 τ 가 와이어에 오지 않았다: %s" % (tau,))
        if rig.tx.zero_mode:
            raise AssertionError("정상 입력인데 zero_mode 다")

        time.sleep(0.8)                         # 발행을 끊는다 (> 0.5 s)
        ok, tau = rig.wait_tau(lambda t: _is_zero(t), 2.0)
        if not ok:
            raise AssertionError("단절 후에도 τ=0 이 오지 않았다: %s" % (tau,))
        if not rig.tx.zero_mode:
            raise AssertionError("단절인데 zero_mode 가 아니다")

        n0 = rig.rx.n_rx                        # 유지가 아니라 **계속** 송신인가
        time.sleep(0.3)
        rig.rx.drain()
        if rig.rx.n_rx - n0 < 5:
            raise AssertionError("τ=0 이 계속 오지 않는다 (%d 개)"
                                 % (rig.rx.n_rx - n0))
        if not _is_zero(rig.rx.tau()):
            raise AssertionError("계속 송신되는 값이 0 이 아니다")
    return "정상→단절→τ=0 지속 (0.3 s 에 %d 개)" % (rig.rx.n_rx - n0)


# ===== ④ NaN/inf → 메시지 전체 τ=0 ========================================

def test_nonfinite_replaces_whole_message():
    """한 필드만 NaN 이어도 **6 필드 전부** 0 이 되어야 한다."""
    cases = [("tau_x_one", float("nan")), ("tau_n_two", float("inf")),
             ("tau_y_two", -float("inf"))]
    names = ["tau_x_one", "tau_y_one", "tau_n_one",
             "tau_x_two", "tau_y_two", "tau_n_two"]
    done = []
    with _Rig() as rig:
        for field, bad in cases:
            # ★ 먼저 **비영**을 와이어에 세운다. `TugRx.last_tau` 는 (0,0,0)
            #   으로 시작하고 `tug_tx` 도 초기에 zero_mode 라, 이 단계를
            #   빼면 "τ=0 이다"가 **처음부터 참**이어서 시험이 무의미해진다.
            for _ in range(10):
                rig.pub.send([3.0, -2.0, 0.5, 1.5, 2.5, -0.5])
                time.sleep(0.02)
            ok, tau = rig.wait_tau(lambda t: abs(t[0] - 3.0) < 1e-6, 1.5)
            if not ok:
                raise AssertionError("%s: 비영 기준선을 세우지 못했다: %s"
                                     % (field, tau))

            six = [3.0, -2.0, 0.5, 1.5, 2.5, -0.5]
            six[names.index(field)] = bad
            n0 = int(rig.tx.nonfinite_total)
            for _ in range(10):
                rig.pub.send(six)
                time.sleep(0.02)
            ok, tau = rig.wait_tau(lambda t: _is_zero(t), 1.5)
            if not ok:
                raise AssertionError("%s=%r 인데 τ=0 이 아니다: %s"
                                     % (field, bad, tau))
            if int(rig.tx.nonfinite_total) <= n0:
                raise AssertionError("%s: nonfinite_total 이 늘지 않았다" % field)
            done.append(field)
    return "3 사례(%s) 전부 6 필드 τ=0" % ", ".join(done)


# ===== ⑤⑥ 종료 버스트 ====================================================

def test_shutdown_burst_ten_zeros():
    """`destroy_node()` → τ=0 을 `shutdown_zero_count` 회 보낸다."""
    rig = _Rig()
    try:
        for _ in range(10):
            rig.pub.send([4.0, 1.0, -1.0, 2.0, -2.0, 0.25])
            time.sleep(0.02)
        rig.wait_tau(lambda t: abs(t[0] - 4.0) < 1e-6, 1.5)
        # ★ 계측점은 **와이어**다. `send_shutdown_zero()` 는 `_send_values`
        #   를 우회해 `link.sendto` 를 직접 부르므로 `sent_total` 이 움직이지
        #   않는다 (실측으로 확인). 터그가 실제로 받은 것을 센다.
        want = int(rig.tx.shutdown_zero_count)
        rig.rx.drain()
        n0 = int(rig.rx.n_rx)
        rig.tx.send_shutdown_zero()
        time.sleep(0.2)
        rig.rx.drain()
        got = int(rig.rx.n_rx) - n0
        if got < want:
            raise AssertionError("터그가 받은 종료 프레임 %d 개 (기대 ≥ %d)"
                                 % (got, want))
        if not _is_zero(rig.rx.tau()):
            raise AssertionError("종료 후 와이어의 마지막 τ 가 0 이 아니다: %s"
                                 % (rig.rx.tau(),))
        if not rig.tx._closing:
            raise AssertionError("_closing 이 서지 않았다 (타이머가 계속 송신한다)")
    finally:
        try:
            rig.pub.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
        rig.rx.close()
    return ("와이어 수신 %d 개 (버스트 %d + 진행중 타이머), "
            "마지막 τ=0, _closing 확인" % (got, want))


def test_duplicate_signals_single_burst():
    """중복 신호가 τ=0 버스트를 **끊을 수 없다**.

    벤더 `install_shutdown_signal_handlers()` 는 인수를 받지 않고, 첫 신호에서
    `ignore_termination_signals()` 로 **세 신호 전부를 `SIG_IGN`** 으로 바꾼 뒤
    `KeyboardInterrupt` 를 올린다. 그래서 두 번째 신호는 **아무 일도 하지
    않는다** — 이것이 버스트가 보장되는 기전이다. 그 기전을 직접 확인한다.

    ★ `SIGHUP` 은 Windows 가 보낼 수 없어 `bootstrap.raise_emulated` 로
    **핸들러를 호출**한다. 검증되는 것은 로직이고, **실제 전달은 아니다.**
    """
    import tug_bridge.relay_common as rc

    sig_names = [n for n in ("SIGINT", "SIGTERM", "SIGHUP")
                 if getattr(signal, n, None) is not None]
    saved = [(n, signal.getsignal(getattr(signal, n))) for n in sig_names]

    rig = _Rig()
    try:
        for _ in range(10):
            rig.pub.send([2.0, 0.5, 0.1, 1.0, -1.0, 0.2])
            time.sleep(0.02)
        ok, tau = rig.wait_tau(lambda t: abs(t[0] - 2.0) < 1e-6, 1.5)
        if not ok:
            raise AssertionError("비영 기준선을 세우지 못했다: %s" % (tau,))

        rc.install_shutdown_signal_handlers()

        # ① 첫 신호 — KeyboardInterrupt 가 올라와야 한다
        first = sig_names[0]
        hit = False
        try:
            if not bootstrap.raise_emulated(getattr(signal, first)):
                signal.getsignal(getattr(signal, first))(
                    getattr(signal, first), None)
        except KeyboardInterrupt:
            hit = True
        if not hit:
            raise AssertionError("%s 에 KeyboardInterrupt 가 올라오지 않았다"
                                 % first)

        # ② 이후 신호는 전부 SIG_IGN 이어야 한다
        for n in sig_names:
            num = getattr(signal, n)
            d = signal.getsignal(num)
            if d is None:
                d = bootstrap.emulated_disposition(num)
            if d is not signal.SIG_IGN:
                raise AssertionError("%s 의 처리방식이 SIG_IGN 이 아니다: %r"
                                     % (n, d))

        # ③ 중복 신호를 더 넣어도 아무 일이 없어야 한다
        for n in sig_names[1:]:
            num = getattr(signal, n)
            if bootstrap.raise_emulated(num):
                raise AssertionError("%s 가 무시되지 않고 핸들러를 불렀다" % n)

        # ④ 그 상태에서 종료 버스트가 온전히 나간다
        want = int(rig.tx.shutdown_zero_count)
        rig.rx.drain()
        n0 = int(rig.rx.n_rx)
        rig.tx.send_shutdown_zero()
        time.sleep(0.2)
        rig.rx.drain()
        got = int(rig.rx.n_rx) - n0
        if got < want:
            raise AssertionError("버스트가 끊겼다: 와이어 %d 개 (기대 ≥ %d)"
                                 % (got, want))
        if not _is_zero(rig.rx.tau()):
            raise AssertionError("와이어 마지막 τ 가 0 이 아니다: %s"
                                 % (rig.rx.tau(),))
    finally:
        for n, h in saved:
            try:
                signal.signal(getattr(signal, n), h)
            except Exception:
                pass
        try:
            rig.pub.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
        rig.rx.close()
    return ("%s → 첫 신호 KeyboardInterrupt, 나머지 SIG_IGN, "
            "τ=0 ×%d 온전. ⚠️ SIGHUP 은 흉내낸 핸들러 호출"
            % ("/".join(sig_names), want))


# ===== ⑦ 상태 단절 → NUC 노드는 **발행을 멈춘다** =========================

def test_nuc_nodes_stop_publishing_when_stale():
    """새 안전장치를 만들지 않는다 — 멈추면 `tug_tx` 가 깨어난다."""
    from sitl.nuc_nodes import LatchQpNode, STATE_TIMEOUT_S
    from nuc.controller import NucController

    rclpy.init(args=None)
    try:
        ctl = NucController()
        qp = LatchQpNode(ctl, rate_hz=50.0)
        m = MothershipData()
        m.timestamp = 1
        m.tau_x, m.tau_y, m.tau_n = 5.0, -3.0, 1.0
        qp._on_tau(m)

        qp._on_tick()
        if qp.n_pub != 1:
            raise AssertionError("신선한 τ_d 인데 발행하지 않았다")

        qp.latest_mono = time.monotonic() - (STATE_TIMEOUT_S + 0.2)
        before = int(qp.n_pub)
        for _ in range(5):
            qp._on_tick()
        if int(qp.n_pub) != before:
            raise AssertionError("단절인데 %d 회 더 발행했다"
                                 % (int(qp.n_pub) - before))
        if int(qp.n_stale) != 5:
            raise AssertionError("n_stale=%d (기대 5)" % int(qp.n_stale))

        qp.latest_mono = time.monotonic()        # 복구
        qp._on_tick()
        if int(qp.n_pub) != before + 1:
            raise AssertionError("복구 후 발행이 재개되지 않았다")
        qp.destroy_node()
    finally:
        rclpy.try_shutdown()
    return ("단절 5 틱 발행 0 / n_stale 5 / 복구 후 재개 "
            "→ τ=0 은 `tug_tx` 가 만든다 (장치 중복 없음)")
