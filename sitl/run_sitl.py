# -*- coding: utf-8 -*-
"""harness_sitl/sitl/run_sitl.py — SITL 러너. **실제 전송계층을 통과하는 루프**.

inproc 하네스와 무엇이 다른가
-----------------------------
| | `run_harness.py` (inproc) | **이 파일 (SITL)** |
|---|---|---|
| 루프 | **스텝 구동.** 벽시계 없음 | **벽시계.** 50 Hz 데드라인 |
| 노드 간 | 딕셔너리 토픽 | **shim `rclpy`** 브로커 (스레드 5개) |
| NUC ↔ 외부 | 같은 함수호출 | **UDP 소켓 + MAVLink v2** |
| 벤더 코드 | 안 쓴다 | **`mothership_rx` ×1 + `tug_tx` ×2 를 그대로 실행** |
| 상태 전달 | `PontoonState` 객체 | **52501** (13필드, float32) |
| 명령 전달 | `TugCommand` 객체 | **52503** (float64) |

프로세스 구성 (스레드 6개)
--------------------------
    [플랜트 — main 스레드]  50 Hz 데드라인 루프
        │ 52501·52502·HB (UDP :14600)         ▲ 52503 (UDP :14651/:14652)
        ▼                                     │
    [mothership_rx]  ─/mothership/state─▶ [rl_policy 10 Hz]
                                              │ /rl/tau_d (float64)
                                              ▼
                                         [latch_qp 50 Hz]
                                              │ /tug_command
                                              ├──▶ [tug1_tx 50 Hz] ─┐
                                              └──▶ [tug2_tx 50 Hz] ─┘

⚠️ `--speed` — 성능평가에 쓸 수 없다 (실측으로 확인)
----------------------------------------------------
`--speed s` 는 **플랜트의 벽시계만** s 배로 당긴다. 그런데 **NUC 노드의
타이머는 진짜 벽시계**(`rclpy` 타이머)다 → 플랜트가 s 배 빨라지면 정책이
**물리시간당 1/s 번만** 돌아 제어주기가 사실상 s 배 느려진다.
실측(`--speed 8 --steps 40`): 제어 40 스텝 구간에서 정책이 **5 회만** 돌았고
(`ctl_policy = 5`) 오차가 0.927 m 였다. **지연이 늘어난 게 아니라 제어율이
바뀐 것**이므로 보수적인 시험이 아니라 **다른 시스템**이다.

→ **성능(20/20·≤0.01 m) 판정은 `--speed 1` 에서만 유효하다.**
`--speed > 1` 은 **배선 연기시험(smoke)** 전용 — 메시지가 흐르는지만 본다.
- `--speed 1` : 1 에피소드 = **60 s**. 20 에피소드 ≈ **20 분** → 배경실행.

⚠️ 기본 `--dr off` 인 이유
--------------------------
**52501 에는 부착점 `r1`·`r2` 가 없다.** 그래서 실물 NUC 은 설정의 공칭값을
쓰고, inproc 하네스는 난수화된 **참값**을 정책에 넘긴다 → 같은 조건이 아니다.
`--dr off` 면 공칭 = 참값이라 두 경로를 공정하게 비교할 수 있다. 난수화를
켠 경우(`--dr on`)는 **별도 측정**으로 보고한다. → `docs/SITL.md`
"""
from __future__ import annotations

import argparse
import io
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bootstrap                                                   # noqa: E402

bootstrap.setup()

import numpy as np                                                 # noqa: E402

import rclpy                                                       # noqa: E402
from rclpy.signals import SignalHandlerOptions                     # noqa: E402

import yaml                                                        # noqa: E402

from tug_bridge.mothership_rx import MothershipRx                  # noqa: E402
from tug_bridge.tug_tx import TugTx                                # noqa: E402

from nuc.controller import NucController                           # noqa: E402

from sitl.nuc_nodes import (LatchQpNode, RlPolicyNode,             # noqa: E402
                            spin_in_thread)
from sitl.plant_proc import PlantWorld                             # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.dirname(HERE)
OUT_DIR = os.path.join(HARNESS, "out")
CFG_YAML = os.path.join(HARNESS, "config", "tug_bridge_loopback.yaml")

ACCEPT_MEAN_POS = 0.01        # run_harness.py 와 같은 기준 [m]
POS_TOK_OK = 0.30             # 에피소드 성공 판정 [m]
YAW_TOL_OK = np.deg2rad(5.0)


# ------------------------------------------------------------------ 파라미터
def load_overrides(path: str = CFG_YAML):
    """ROS 2 파라미터 yaml → 노드별 `(이름, 값)` 목록.

    ⚠️ 값을 **손으로 고치지 않는다.** `tug_bridge_loopback.yaml` 이 유일한
    출처여야 하네스와 실기체가 같은 설정을 본다. yaml 의 `*_s`/`*_hz` 가
    float 이어야 노드가 뜨는 함정도 그대로 재현된다 (`Parameter.Type.DOUBLE`
    는 int 를 거부한다).
    """
    with io.open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    out = {}
    for node, body in (doc or {}).items():
        params = (body or {}).get("ros__parameters", {}) or {}
        out[node] = [(k, v) for k, v in params.items()]
    return out


# ------------------------------------------------------------------ 구성
class SitlRig(object):
    """벤더 노드 3개 + NUC 노드 2개 + 플랜트를 띄우고 내린다."""

    def __init__(self, onnx_path=None, dr=False, seed=None,
                 cfg_yaml=CFG_YAML, quiet=True, send_data_52502=True,
                 qp_hz=None, policy_hz=None, state_src="52501",
                 geometry=None):
        ov = load_overrides(cfg_yaml)
        for need in ("mothership_rx", "tug1_tx", "tug2_tx"):
            if need not in ov:
                raise KeyError("%s 에 `%s` 설정이 없다" % (cfg_yaml, need))

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

        # ① 벤더 노드 — **한 글자도 고치지 않은 것들** (diff 0 / 가산만)
        self.state_src = str(state_src)
        if self.state_src == "px4std":
            # PX4 표준 경로 — 52501 도 다이얼렉트도 마더십도 바꾸지 않는다.
            # **받는 노드만** 갈아 끼운다: `MothershipRx` → `Px4StdStateNode`.
            from sitl.px4_state_node import Px4StdStateNode
            pm = dict(ov["mothership_rx"])
            self.rx = Px4StdStateNode(
                bind_ip=str(pm.get("bind_ip", "127.0.0.1")),
                bind_port=int(pm.get("bind_port", 14600)),
                allowed_src_ip=str(pm.get("allowed_src_ip", "")),
                sysid=int(pm.get("filter_sysid", 1)),
                compid=int(pm.get("filter_compid", 0)))
        else:
            self.rx = MothershipRx(parameter_overrides=ov["mothership_rx"])
        self.tx1 = TugTx(parameter_overrides=ov["tug1_tx"])
        self.tx2 = TugTx(parameter_overrides=ov["tug2_tx"])

        # ② NUC production 제어기 — 하네스와 **같은 파일**
        # 부착기하는 `config/geometry.yaml` 에서 오고, 제어기·QP·플랜트가 같은 것을 쓴다.
        self.geometry = geometry
        self.ctl = NucController(onnx_path=onnx_path, geometry=geometry)
        from sitl.nuc_nodes import POLICY_RATE_HZ, QP_RATE_HZ
        self.rl = RlPolicyNode(self.ctl,
                               rate_hz=policy_hz or POLICY_RATE_HZ)
        self.qp = LatchQpNode(self.ctl, rate_hz=qp_hz or QP_RATE_HZ)

        # ③ 외부세계
        self.world = PlantWorld(dr=dr, seed=seed,
                                send_data_52502=send_data_52502,
                                state_src=self.state_src,
                                geometry=geometry)

        self.nodes = [self.rx, self.tx1, self.tx2, self.rl, self.qp]
        self.threads = [spin_in_thread(n) for n in self.nodes]
        time.sleep(0.3)        # 노드가 소켓·타이머를 올릴 틈

    def close(self):
        # ★ `destroy_node()` 가 τ=0 ×`shutdown_zero_count` 를 보낸다 (안전경로)
        for n in self.nodes:
            try:
                n.destroy_node()
            except Exception as e:
                print("destroy_node 경고: %s" % e)
        rclpy.try_shutdown()
        self.world.close()

    def tx_stats(self):
        lk = getattr(self.rx, "link", None)
        return {"rx_pkt": int(getattr(lk, "rx_packets", -1)),
                "rx_reopen": int(getattr(lk, "reopen_count", -1)),
                "rx_dropped_ip": int(getattr(lk, "dropped_ip", -1)),
                "rx_parse_err": int(getattr(self.rx, "parse_errors", -1)),
                "rx_drop_msgid": int(getattr(self.rx, "drop_msgid", -1)),
                "rx_drop_sysid": int(getattr(self.rx, "drop_sysid", -1)),
                "rx_state": int(getattr(self.rx, "state_total",
                                      getattr(self.rx, "n_pub", 0))),
                "rx_data": int(getattr(self.rx, "rx_total", 0)),
                "rl_state": self.rl.n_state, "rl_pub": self.rl.n_pub,
                "rl_stale": self.rl.n_stale, "rl_override": self.rl.n_override,
                "qp_tau": self.qp.n_tau, "qp_pub": self.qp.n_pub,
                "qp_stale": self.qp.n_stale,
                "ctl_policy": int(self.ctl.n_policy),
                "ctl_qp": int(self.ctl.n_qp),
                "ctl_override": int(self.ctl.n_override)}


# ------------------------------------------------------------------ 에피소드
def run_episode(rig: SitlRig, seed: int, n_ctrl: int, speed: float,
                hold_s: float = 0.6, rec=None):
    """에피소드 1회. 반환 지표 딕셔너리.

    ★ 에피소드 초기화는 **production 경로**로 한다: `on_off = 0` 을 몇 틱
    보내면 `NucController.policy_step` 의 수동개입 분기가 `a_prev` 를 0 으로
    되돌린다. 하네스가 제어기 내부를 건드리지 않는다.
    """
    w = rig.world
    period = (w.hsub) / float(speed)
    n_tick = int(n_ctrl * w.A.nsub)

    # --- ① 자율해제 구간 (a_prev 초기화 + 수동개입 경로 시험) ---
    w.reset(seed=seed)
    w.on_off = 0
    t0 = time.monotonic()
    n_hold = max(1, int(hold_s / period))
    for i in range(n_hold):
        w.tick()
        _sleep_until(t0 + (i + 1) * period)

    # --- ② 본 구간 ---
    w.on_off = 1
    w.reset(seed=seed)          # 초기상태를 다시 깔끔하게 (제어기는 이미 0)
    t0 = time.monotonic()
    overrun = 0
    worst_lag = 0.0
    term = False
    k = 0
    for i in range(n_tick):
        w.tick()
        k = i + 1
        if rec is not None:
            # ★ 영상·그림용 궤적 기록. 플랜트와 제어기를 **읽기만** 한다
            c = rig.ctl
            rec["S"].append(np.asarray(w.A.env.s, dtype=np.float64).copy())
            rec["A"].append(np.asarray(c.a_prev, dtype=np.float64).copy())
            rec["U"].append(np.asarray(c.u6, dtype=np.float64).copy())
            rec["M"].append(np.asarray(c.marg, dtype=np.float64).copy())
            st = rig.tx_stats()
            rec["C"].append([st["rx_state"], st["rx_data"], st["rl_pub"],
                             st["qp_pub"], st["rx_reopen"]])
        if not w.finite() or w.dpos() > 12.0:
            term = True
            break
        lag = _sleep_until(t0 + (i + 1) * period)
        if rec is not None:
            rec["L"].append(lag * 1e3)
        if lag > period:
            overrun += 1
        if lag > worst_lag:
            worst_lag = lag
    wall = time.monotonic() - t0

    return {"seed": int(seed), "k": k, "n_tick": n_tick,
            "dpos": w.dpos(), "dyaw": w.dyaw(), "term": bool(term),
            "wall_s": wall, "overrun": overrun, "worst_lag_ms": worst_lag * 1e3,
            "plant": w.stats(), "nuc": rig.tx_stats()}


def _sleep_until(deadline: float) -> float:
    """데드라인까지 잔다. 반환: **지연량**(이미 지났으면 초과분)."""
    now = time.monotonic()
    if now >= deadline:
        return now - deadline
    time.sleep(deadline - now)
    return max(0.0, time.monotonic() - deadline)


# ------------------------------------------------------------------ main
def main(argv=None):
    ap = argparse.ArgumentParser(description="SITL 러너 (실제 전송계층)")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--speed", type=float, default=1.0,
                    help="벽시계 배속. ⚠️ >1 은 제어율을 바꾼다 → 연기시험 전용")
    ap.add_argument("--dr", choices=["off", "on"], default="off")
    ap.add_argument("--steps", type=int, default=0,
                    help="제어스텝 수. 0 이면 환경의 N (600)")
    ap.add_argument("--onnx", default=None)
    ap.add_argument("--out", default=os.path.join(OUT_DIR, "SITL_RUN.json"))
    ap.add_argument("--no-52502", action="store_true",
                    help="기존 τ 경로(52502) 송신을 끈다 — 와이어 부하 절반 확인용")
    ap.add_argument("--qp-hz", type=float, default=None,
                    help="진단용. QP 노드 주기를 바꾼다 (기본 50)")
    ap.add_argument("--policy-hz", type=float, default=None,
                    help="진단용. 정책 노드 주기를 바꾼다 (기본 10)")
    ap.add_argument("--traj", default=None,
                    help="궤적 npz 저장경로. 영상·그림용 (성능판정과 무관)")
    ap.add_argument("--traj-ep", type=int, default=0,
                    help="--traj 로 기록할 에피소드 번호 (기본 0)")
    ap.add_argument("--geometry", default=None,
                    help="부착기하 yaml 경로. 기본은 config/geometry.yaml "
                         "(docs/GEOMETRY.md). off 를 주면 학습 당시 값 그대로")
    ap.add_argument("--state-src", choices=["52501", "px4std"],
                    default="52501",
                    help="상태 입력경로. 52501=신설 메시지 / "
                         "px4std=PX4 표준 ATTITUDE+LOCAL_POSITION_NED+"
                         "POSITION_TARGET_LOCAL_NED (docs/INPUT_PATHS.md)")
    a = ap.parse_args(argv)

    rig = SitlRig(onnx_path=a.onnx, dr=(a.dr == "on"), seed=a.seed0,
                  send_data_52502=(not a.no_52502),
                  qp_hz=a.qp_hz, policy_hz=a.policy_hz,
                  state_src=a.state_src,
                   geometry=(False if str(a.geometry).lower() == "off"
                             else a.geometry))
    n_ctrl = a.steps if a.steps > 0 else int(rig.world.A.N)
    rows = []
    try:
        print("SITL: episodes=%d steps=%d speed=%.2f dr=%s  (1ep = %.1f s)"
              % (a.episodes, n_ctrl, a.speed, a.dr,
                 n_ctrl * rig.world.A.nsub * rig.world.hsub / a.speed))
        from nuc.cfg import geometry_summary
        print("  " + geometry_summary(rig.ctl.c))
        if a.speed != 1.0:
            print("  ⚠️ speed != 1 → 제어율이 바뀐다. 성능판정에 쓰지 말 것 "
                  "(배선 연기시험 전용)")
        traj_rec = None
        traj_seed = -1
        traj_env = None
        for i in range(a.episodes):
            rec = None
            if a.traj and i == a.traj_ep:
                rec = {k: [] for k in ("S", "A", "U", "M", "C", "L")}
            r = run_episode(rig, a.seed0 + i, n_ctrl, a.speed, rec=rec)
            if rec is not None:
                traj_rec, traj_seed = rec, a.seed0 + i
                w = rig.world
                traj_env = {"r1": np.asarray(w.A.env.r1, dtype=np.float64),
                            "r2": np.asarray(w.A.env.r2, dtype=np.float64),
                            "goal": np.asarray(w.goal, dtype=np.float64),
                            "dt": np.float64(w.hsub)}
            rows.append(r)
            print("  ep %2d seed %d  dpos %.4f m  dyaw %.3f deg  "
                  "k %d/%d  overrun %d  worst %.1f ms%s"
                  % (i, r["seed"], r["dpos"], np.rad2deg(r["dyaw"]),
                     r["k"], r["n_tick"], r["overrun"], r["worst_lag_ms"],
                     "  TERM" if r["term"] else ""))
            sys.stdout.flush()
    finally:
        rig.close()

    dp = np.array([r["dpos"] for r in rows], dtype=np.float64)
    dy = np.array([r["dyaw"] for r in rows], dtype=np.float64)
    ok = int(np.sum((dp <= POS_TOK_OK) & (dy <= YAW_TOL_OK)
                    & np.array([not r["term"] for r in rows])))
    summary = {"episodes": len(rows), "ok": ok,
               "mean_pos": float(dp.mean()) if len(dp) else float("nan"),
               "max_pos": float(dp.max()) if len(dp) else float("nan"),
               "mean_yaw_deg": float(np.rad2deg(dy.mean())) if len(dy) else float("nan"),
               "speed": a.speed, "dr": a.dr, "steps": n_ctrl,
               "state_src": a.state_src,
        "geometry": a.geometry or "config/geometry.yaml",
               "accept_mean_pos": ACCEPT_MEAN_POS,
               "pass": bool(len(rows) and ok == len(rows)
                            and dp.mean() <= ACCEPT_MEAN_POS
                            and a.speed == 1.0),
               "perf_valid": bool(a.speed == 1.0),
               "overrun_total": int(sum(r["overrun"] for r in rows)),
               "nuc_last": rows[-1]["nuc"] if rows else {},
               "plant_last": rows[-1]["plant"] if rows else {}}

    if not os.path.isdir(OUT_DIR):
        os.makedirs(OUT_DIR)
    with io.open(a.out, "w", encoding="utf-8") as f:
        f.write(json.dumps({"summary": summary, "episodes": rows},
                           ensure_ascii=False, indent=1))

    if a.traj and traj_rec is not None:
        env = traj_env
        np.savez(a.traj,
                 S=np.array(traj_rec["S"]), A=np.array(traj_rec["A"]),
                 U=np.array(traj_rec["U"]), M=np.array(traj_rec["M"]),
                 C=np.array(traj_rec["C"], dtype=np.int64),
                 L=np.array(traj_rec["L"], dtype=np.float64),
                 r1=env["r1"], r2=env["r2"], goal=env["goal"],
                 dt=env["dt"], seed=np.int64(traj_seed))
        print("궤적 저장: %s  (%d 틱, seed %d)"
              % (a.traj, len(traj_rec["S"]), traj_seed))

    print("")
    print("합계: %d/%d  평균 %.5f m (최대 %.5f)  yaw 평균 %.4f deg  "
          "overrun %d  →  %s"
          % (ok, len(rows), summary["mean_pos"], summary["max_pos"],
             summary["mean_yaw_deg"], summary["overrun_total"],
             "PASS" if summary["pass"] else "FAIL"))
    print("기록: %s" % a.out)
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
