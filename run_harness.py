# -*- coding: utf-8 -*-
"""harness_sitl/run_harness.py — A to B to C to D to A 폐루프 스케줄러.

실행
----
    ../plan_pontoon_rl/.venv/Scripts/python.exe run_harness.py --episodes 20

틱 스케줄 (실물 레이트를 실제로 재현한다)
-----------------------------------------
기준 틱 = 물리 50 Hz (hsub = 0.02 s). 제어는 10 Hz 이므로 5 틱마다 한 번만
정책이 돈다. 나머지 4 틱 동안 터그 명령은 sample-and-hold 된다 (= 실물
Pixhawk 동작. iface/transport.py 주석 참조).

    틱 i:
        B.tick()                      50 Hz  플랜트 상태 중계 (float32 양자화)
        if i % 5 == 0: C.tick_policy()  10 Hz  ONNX 정책 -> tau_d -> /mothership_data
        C.tick_qp()                   50 Hz  Latch-aware QP -> /tug_command
        D.tick()                      50 Hz  MAVLink 왕복 -> 하위 역행렬 -> /tug/thrust
        A.tick()                      50 Hz  RK4 서브스텝 1회 -> /plant/state
        transport.tick()                     지연 큐 전진

pontoon_rl/train/eval_policy.py 와의 등가성
-------------------------------------------
env.step(a) 은 (1) 정책 (2) QP (3) RK4 x5 (4) 새 obs 순서다. 새 obs 는
직전 스텝의 res/marg/a_prev 를 담는다. 위 스케줄은 tick_policy 를 tick_qp
보다 먼저 호출하므로 그 순서가 그대로 보존된다.

의도된 차이 (실물 경로이기 때문에 남겨 둔 것)
----------------------------------------------
1. /pontoon/state 는 float32 (PX4 uORB) -> 상태 관측에 양자화가 있다.
2. /mothership_data 는 float32 (<Qfff) -> tau_d 에 양자화가 있다.
3. /tug_command 는 float64 -> 양자화 없음. MAVLink CRC 왕복만 통과한다.
4. QP 가 50 Hz 로 돈다 (eval_policy 는 제어 스텝당 1회). LatchQP.solve 가
   결정론이라 같은 tau_d 면 같은 해 -> 수치적으로 동일하다.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, os.path.join(_ROOT, "pontoon_rl")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _console import utf8_stdout                                   # noqa: E402
from iface.transport import LinkConfig, make_transport             # noqa: E402
from nodes.node_a_plant import PlantNode                           # noqa: E402
from nodes.node_b_mothership import MothershipBypassNode           # noqa: E402
from nodes.node_c_nuc import NucNode                               # noqa: E402
from nodes.node_d_esp32 import Esp32LinkNode                       # noqa: E402
from nodes.topics import (TOPIC_CMD, TOPIC_PLANT, TOPIC_STATE,     # noqa: E402
                          TOPIC_TAU_D, TOPIC_THRUST)

OUT_DIR = os.path.join(_HERE, "out")

# 성공 판정은 pontoon_rl/train/eval_policy.py 와 같은 기준으로 고정한다.
POS_TOL_OK = 0.30             # m
YAW_TOL_OK = np.deg2rad(5.0)  # rad
ACCEPT_MEAN_POS = 0.01        # m — 과제 합격 기준


def default_links(delay_state=0, delay_cmd=0, drop=0.0, serialize=True):
    """토픽별 전송 특성.

    /plant/state 는 물리적 계측이지 전송선이 아니다 -> 직렬화하지 않는다.
    /pontoon/state 가 실물 uORB/DDS 경계이고 여기서 float32 로 깎인다.
    """
    return {
        TOPIC_PLANT:  LinkConfig(0, 0.0, serialize=False),
        TOPIC_STATE:  LinkConfig(delay_state, drop, serialize=serialize),
        TOPIC_TAU_D:  LinkConfig(0, 0.0, serialize=serialize),
        TOPIC_CMD:    LinkConfig(delay_cmd, drop, serialize=serialize),
        TOPIC_THRUST: LinkConfig(0, 0.0, serialize=serialize),
    }


class Harness(object):
    """네 노드를 하나의 틱 루프로 묶는다. 노드 본체는 전혀 고치지 않는다."""

    def __init__(self, transport="inproc", onnx_path=None, cfg=None, dr=True,
                 delay_state=0, delay_cmd=0, drop=0.0, link_seed=0,
                 mavlink=True, pwm_quantize=False, serialize=True,
                 tau_from_topic=True):
        self.tp = make_transport(
            transport,
            links=default_links(delay_state, delay_cmd, drop, serialize),
            seed=link_seed)
        self.A = PlantNode(self.tp, cfg=cfg, dr=dr)
        self.B = MothershipBypassNode(self.tp)
        self.C = NucNode(self.tp, onnx_path=onnx_path, cfg=cfg)
        self.D = Esp32LinkNode(self.tp, cfg=cfg, mavlink=mavlink,
                               pwm_quantize=pwm_quantize)
        self.nsub = self.A.nsub
        self.hsub = self.A.hsub
        self.dt_ctrl = self.A.dt_ctrl
        self.N = self.A.N
        self.tau_from_topic = bool(tau_from_topic)

    # -- 1 에피소드 ---------------------------------------------------------
    def episode(self, seed):
        self.tp.reset()
        self.B.reset()
        self.C.reset()
        self.D.reset()
        self.A.reset(seed=seed)          # 초기 상태 1회 발행

        S, A, U, M = [], [], [], []
        a_now = np.zeros(3)
        term = trunc = False
        k = i = 0
        while True:
            t_us = int(round(i * self.hsub * 1e6))
            self.B.tick()
            if i % self.nsub == 0:
                a = self.C.tick_policy(t_us)
                if a is not None:
                    a_now = np.asarray(a, dtype=np.float64)
            self.C.tick_qp(t_us, from_topic=self.tau_from_topic)
            self.D.tick(t_us)
            self.A.tick()
            self.tp.tick()
            i += 1

            if i % self.nsub == 0:       # 제어 스텝 1회 완료
                k += 1
                S.append(self.A.env.s.copy())
                A.append(a_now.copy())
                U.append(self.C.u6.copy())
                M.append(self.C.marg.copy())
                if (not self.A.finite()) or self.A.dpos() > 12.0:
                    term = True
                if k >= self.N:
                    trunc = True
                if term or trunc:
                    break

        return dict(S=np.array(S), A=np.array(A), U=np.array(U), M=np.array(M),
                    r1=self.A.env.r1.copy(), r2=self.A.env.r2.copy(),
                    goal=self.A.env.goal.copy(), dt=self.dt_ctrl,
                    dpos=self.A.dpos(), dyaw=self.A.dyaw(),
                    term=bool(term), trunc=bool(trunc), k=int(k))


def run(args):
    utf8_stdout()
    os.makedirs(OUT_DIR, exist_ok=True)
    h = Harness(transport=args.transport, onnx_path=args.onnx,
                dr=not args.no_dr,
                delay_state=args.delay_state, delay_cmd=args.delay_cmd,
                drop=args.drop, link_seed=args.link_seed,
                mavlink=not args.no_mavlink, pwm_quantize=args.pwm,
                serialize=not args.no_serialize,
                tau_from_topic=not args.tau_direct)

    print("harness_sitl - A->B->C->D->A 폐루프")
    print("  transport      : %s" % args.transport)
    print("  정책           : %s" % (args.onnx or "out/ppo_sc3_final.onnx (기본)"))
    print("  레이트         : 물리 %.0f Hz / 제어 %.0f Hz (nsub=%d, N=%d)"
          % (1.0 / h.hsub, 1.0 / h.dt_ctrl, h.nsub, h.N))
    print("  지연/드랍      : state %d틱, cmd %d틱, drop %.3f"
          % (args.delay_state, args.delay_cmd, args.drop))
    print("  MAVLink 왕복   : %s / 직렬화: %s / PWM 양자화: %s"
          % ("ON" if not args.no_mavlink else "OFF",
             "OFF" if args.no_serialize else "ON",
             "ON(확인 필요)" if args.pwm else "OFF"))
    print("")

    dps, dys, oks = [], [], 0
    t0 = time.perf_counter()
    for e in range(args.episodes):
        seed = args.seed0 + e
        r = h.episode(seed)
        ok = (r["dpos"] < POS_TOL_OK) and (r["dyaw"] < YAW_TOL_OK) and not r["term"]
        oks += int(ok)
        dps.append(r["dpos"])
        dys.append(r["dyaw"])
        print("  ep%02d seed%d  dpos %.4f m  dyaw %.3f deg  %s%s"
              % (e, seed, r["dpos"], np.degrees(r["dyaw"]),
                 "OK" if ok else "FAIL", "  [terminated]" if r["term"] else ""))
        if e == 0 and not args.no_save:
            p = os.path.join(OUT_DIR, "traj_harness.npz")
            np.savez(p, S=r["S"], A=r["A"], U=r["U"], M=r["M"],
                     r1=r["r1"], r2=r["r2"], goal=r["goal"], dt=r["dt"])
            print("       궤적 저장: %s  (make_video.py 호환)" % p)
    el = time.perf_counter() - t0

    dps = np.asarray(dps)
    dys = np.asarray(dys)
    passed = (oks == args.episodes) and (float(dps.mean()) <= ACCEPT_MEAN_POS)
    print("")
    print("  성공(<%.2f m, <5 deg) : %d/%d" % (POS_TOL_OK, oks, args.episodes))
    print("  평균 위치오차         : %.5f m   (최대 %.5f m)"
          % (dps.mean(), dps.max()))
    print("  평균 방위오차         : %.4f deg (최대 %.4f deg)"
          % (np.degrees(dys.mean()), np.degrees(dys.max())))
    print("  소요                  : %.1f s (%.2f s/ep)"
          % (el, el / max(args.episodes, 1)))
    print("")
    print("  합격 기준 %d/%d · 평균 <= %.3f m  ->  %s"
          % (args.episodes, args.episodes, ACCEPT_MEAN_POS,
             "PASS" if passed else "FAIL"))
    print("")
    print("  전송 통계 (pub/rx/drop):")
    for t, s in h.tp.stats().items():
        print("    %-20s pub %6d  rx %6d  drop %4d"
              % (t, s["pub"], s["rx"], s["drop"]))
    return oks, dps, dys, passed


def build_parser():
    ap = argparse.ArgumentParser(description="harness_sitl 폐루프 실행")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--transport", default="inproc", choices=["inproc", "ros2"])
    ap.add_argument("--onnx", default=None)
    ap.add_argument("--delay-state", type=int, default=0, dest="delay_state",
                    help="/pontoon/state 지연 (물리 틱, 1틱=20 ms). 기본 0")
    ap.add_argument("--delay-cmd", type=int, default=0, dest="delay_cmd",
                    help="/tug_command 지연 (물리 틱). 기본 0")
    ap.add_argument("--drop", type=float, default=0.0,
                    help="패킷 손실 확률. 기본 0")
    ap.add_argument("--link-seed", type=int, default=0, dest="link_seed")
    ap.add_argument("--no-mavlink", action="store_true",
                    help="MAVLink 프레이밍 생략 (진단)")
    ap.add_argument("--no-serialize", action="store_true",
                    help="모든 wire 양자화 끄기 (진단)")
    ap.add_argument("--tau-direct", action="store_true",
                    help="tau_d 를 토픽 대신 내부 float64 로 전달 (진단)")
    ap.add_argument("--pwm", action="store_true",
                    help="PWM 양자화 ON (확인 필요 - 실측 곡선 없음)")
    ap.add_argument("--no-dr", action="store_true", help="도메인 랜덤화 끄기")
    ap.add_argument("--no-save", action="store_true")
    return ap


def main():
    a = build_parser().parse_args()
    _, _, _, passed = run(a)
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
