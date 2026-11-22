# -*- coding: utf-8 -*-
"""harness_sitl/sitl/plant_proc.py — SITL 외부세계 (**가상 전용**).

무엇을 대역하는가
-----------------
    ┌──────────────── 이 파일 ────────────────┐
    │ 폰툰 + 물  (PontoonSc3Env, 그대로 재사용) │
    │ 마더십 Pixhawk/Simulink → 52501·52502     │
    │ 터그 ESP32 ×2 + 터그 Pixhawk ←  52503     │
    │   └─ 하위 역행렬 (TugLowerAllocator, 그대로)│
    └───────────────────────────────────────────┘
               ▲ UDP (127.0.0.1)          │
               │                           ▼
          `vendor/tug_bridge/tug_tx` ×2   `vendor/tug_bridge/mothership_rx`

**NUC 안쪽은 이 파일을 모른다.** 반대로 이 파일도 `nuc/` 를 모른다
(import 하지 않는다). 둘을 잇는 것은 **UDP 소켓과 MAVLink 프레임뿐**이다
→ 그것이 "가상과 실물의 차이는 전송계층과 플랜트뿐"이라는 주장의 실증이다.

재사용 — 새로 만든 것이 없다
----------------------------
| 쓰는 것 | 출처 | 수정 |
|---|---|---|
| 폰툰 운동모델·보상·난수화 | `pontoon_rl/env/pontoon_env.py` | **없음** |
| 플랜트 적분 루프 | `nodes/node_a_plant.py::PlantNode` | **없음** |
| 터그 하위 역행렬·포화 | `plant/tug_lower.py::TugLowerAllocator` | **없음** |
| MAVLink 프레이밍 | `sitl/wire.py` (dialect + 벤더 `pack_and_advance`) | — |

`PlantNode` 는 자기 전용 `InprocTransport` **인스턴스**를 받는다
(`make_transport` 는 호출마다 새 인스턴스다). 그래서 shim `rclpy` 브로커와
완전히 분리된다 — 플랜트 내부 배선(`/plant/state`, `/tug/thrust`)이 NUC 쪽
토픽과 섞일 길이 없다.

에피소드 사이 초기화 — ★ production 경로를 쓴다
-----------------------------------------------
NUC 노드는 실물처럼 **계속 떠 있다.** 그래서 `a_prev` 를 하네스가 직접
건드릴 수 없다(건드리면 "같은 코드"가 깨진다). 대신 `52501` 의
**`on_off = 0`** 을 몇 틱 보낸다 → `NucController.policy_step` 의
수동개입 분기가 `a_prev`·`tau_d` 를 0 으로 되돌린다. **실물 자율해제와
같은 코드 경로**이고, 덤으로 매 에피소드 그 경로를 시험하게 된다.
"""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from iface.messages import TugThrust
from iface.transport import make_transport
from nodes.node_a_plant import PlantNode
from nodes.topics import TOPIC_THRUST
from plant.tug_lower import TugLowerAllocator

from sitl.wire import MothershipTx, TugRx

TUG1_PORT = 14651          # config/tug_bridge_loopback.yaml: tug1_tx.remote_port
TUG2_PORT = 14652          # 같은 파일: tug2_tx.remote_port
NUC_RX_PORT = 14600        # 같은 파일: mothership_rx.bind_port
HB_PERIOD_S = 1.0


class PlantWorld(object):
    """폰툰·물·마더십송신기·터그2대를 한 덩어리로 들고 50 Hz 로 돈다."""

    def __init__(self, cfg=None, dr: bool = False, seed: Optional[int] = None,
                 nuc_ip: str = "127.0.0.1", nuc_port: int = NUC_RX_PORT,
                 tug1_port: int = TUG1_PORT, tug2_port: int = TUG2_PORT,
                 send_data_52502: bool = True,
                 scenario_mode: int = 3, state_src: str = "52501",
                 geometry=None) -> None:
        self.tp = make_transport("inproc")          # 플랜트 전용 인스턴스
        self.A = PlantNode(self.tp, cfg=cfg, dr=dr, seed=seed,
                           geometry=geometry)   # config/geometry.yaml
        self.alloc = TugLowerAllocator.from_cfg(self.A.env.c)
        self.tx = MothershipTx(nuc_ip, nuc_port)
        self.rx = [TugRx(tug1_port, which=1, accepted_sysid=250),
                   TugRx(tug2_port, which=2, accepted_sysid=250)]
        self.send_data_52502 = bool(send_data_52502)
        self.scenario_mode = int(scenario_mode)
        # 상태 송신 경로. "52501" = 신설 메시지 1 개,
        # "px4std" = PX4 표준 3 개 (ATTITUDE · LOCAL_POSITION_NED ·
        # POSITION_TARGET_LOCAL_NED). 플랜트(=마더십 모사)만 바뀌고
        # 정책·QP·터그 경로는 **그대로**다. 근거: docs/INPUT_PATHS.md
        self.state_src = str(state_src)

        # 수동개입 플래그. 하네스가 외부에서 바꿔 S5③ 안전거동을 시험한다.
        self.rc_on_off = 0
        self.on_off = 1

        self.hsub = float(self.A.hsub)
        self.goal = np.asarray(getattr(self.A.env, "goal", np.zeros(3)),
                               dtype=np.float64)
        self._t0_mono = time.monotonic()
        self._last_hb = -1e9
        self.k = 0
        self.n_cmd_rx = 0

    # -- 수명주기 -----------------------------------------------------------
    def reset(self, seed: Optional[int] = None) -> None:
        self.A.reset(seed=seed)
        self.alloc.reset()
        self.goal = np.asarray(getattr(self.A.env, "goal", np.zeros(3)),
                               dtype=np.float64)
        self.k = 0
        # ⚠️ `last_tau` 는 지우지 않는다 — 터그 Pixhawk 가 마지막 값을
        # 유지하는 성질을 모사하는 것이 이 하네스의 목적이다.

    def close(self) -> None:
        self.tx.close()
        for r in self.rx:
            r.close()

    # -- 상태 꺼내기 --------------------------------------------------------
    def _eta_nu(self) -> Tuple[np.ndarray, np.ndarray]:
        s = np.asarray(self.A.env.s, dtype=np.float64)
        return s[3:6], s[0:3]          # (x,y,psi), (u,v,r)

    # -- 한 틱 (= 물리 20 ms) ----------------------------------------------
    def tick(self) -> None:
        """수신 → 하위배분 → 적분 → 송신. **순서가 실물과 같다.**"""
        # ① 터그 2대가 각자 52503 을 받아 자기 몫을 고른다
        got = sum(r.drain() for r in self.rx)
        self.n_cmd_rx += got

        # ② 터그 Pixhawk 내부 하위 역행렬 (우리가 배포하지 않는 코드)
        f1, f2, ta1, ta2 = self.alloc.allocate_pair(
            np.asarray(self.rx[0].tau(), dtype=np.float64),
            np.asarray(self.rx[1].tau(), dtype=np.float64))
        self.tp.publish(TOPIC_THRUST, TugThrust(
            t_us=int(self.A.t_us), f_one=f1, f_two=f2,
            tau_one=ta1, tau_two=ta2))

        # ③ 물 — `PlantNode` 가 `/tug/thrust` 를 래치해 RK4 적분한다
        self.A.tick()
        self.k += 1

        # ④ 마더십이 상태를 올린다 (50 Hz)
        eta, nu = self._eta_nu()
        t_us = int(self.A.t_us)
        if self.state_src == "px4std":
            self.tx.state_px4std(t_us, eta, nu, self.goal)
        else:
            self.tx.state(t_us, eta, nu, self.goal,
                          scenario_mode=self.scenario_mode,
                          rc_on_off=self.rc_on_off, on_off=self.on_off)
        if self.send_data_52502:
            # 기존 LOS/PID 경로가 살아 있음을 보이는 공존 확인용 (RL 은 안 쓴다)
            self.tx.data(t_us, (0.0, 0.0, 0.0))

        # ⑤ 마더십 ESP32 가 NUC HEARTBEAT 를 듣는다 — 비우지 않으면 ICMP
        #    port-unreachable 이 NUC 수신소켓을 재바인드시킨다 (`sitl/wire.py`)
        self.tx.drain_hb()

        now = time.monotonic()
        if now - self._last_hb >= HB_PERIOD_S:
            self.tx.heartbeat()
            self._last_hb = now

    # -- 지표 (하네스와 같은 정의) ------------------------------------------
    def dpos(self) -> float:
        return float(self.A.dpos())

    def dyaw(self) -> float:
        return float(self.A.dyaw())

    def finite(self) -> bool:
        return bool(self.A.finite())

    def stats(self) -> Dict[str, Any]:
        return {"k": self.k,
                "cmd_rx": self.n_cmd_rx,
                "tug1_rx": self.rx[0].n_rx,
                "tug2_rx": self.rx[1].n_rx,
                "tug1_drop_sysid": self.rx[0].n_drop_sysid,
                "tug2_drop_sysid": self.rx[1].n_drop_sysid,
                "tx_state": self.tx.n_state,
                "tx_data": self.tx.n_data,
                "tx_hb": self.tx.n_hb,
                "hb_rx": self.tx.n_hb_rx,
                "tx_err": self.tx.n_err,
                "n_sat": int(self.alloc.n_sat)}
