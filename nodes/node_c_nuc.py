# -*- coding: utf-8 -*-
"""harness_sitl/nodes/node_c_nuc.py — 노드 C: 마더십 NUC 전송 어댑터 (**가상 전용**).

이 파일의 지위 — ★ S4 에서 바뀠다
----------------------------------
**제어 수식은 이 파일에 없다.** 전부 `nuc/controller.py::NucController` 에 있다.

| 파일 | 지위 | 실물에서 |
|---|---|---|
| `nuc/controller.py` + `nuc/onnx_policy.py` | **production** | **같은 파일이 NUC 에서 돈다** |
| **이 파일** | **전송 어댑터. 가상 전용** | **ROS 2 노드가 대신한다** |

즉 이 파일이 하는 일은 **세 가지뿐**이다:
① 토픽 구독 → 메시지에서 숫자 꺼내기, ② 제어기 호출, ③ 결과를 토픽으로 발행.
실물에서는 이 세 줄이 `rclpy` 노드(`create_subscription` / `create_publisher`)로
바뀐다. **`NucController` 호출부는 한 글자도 바뀌지 않는다** → 그것이 하네스의
존재 이유다. 교체표는 `docs/FILE_SWAP.md`.

왜 분리했나
-----------
S3 까지는 이 파일이 `LatchQP` 와 `OnnxPolicy` 를 직접 들고 관측 19차도 직접
조립했다. 그러면 "실물 NUC 에 무엇을 배포하는가"의 답이 **이 파일**이 되는데,
이 파일은 하네스 전송(`transport.latest/publish`)에 묶여 있어 그대로 배포할 수
없다. 수식을 `nuc/` 로 옮겨 **배포 단위와 전송 단위를 파일로 분리**했다.

데이터 흐름 (실물 대응 — 사용자 확정 구조)
------------------------------------------
    /pontoon/state (50 Hz)
         └─ NucController.policy_step (10 Hz) ─ τ_d
                                   └─ /mothership_data  ★ float32 양자화 지점
                                   └─ NucController.allocate (50 Hz) ─ u6
                                              └─ /tug_command (50 Hz) → 노드 D
                                              └─ /allocation_debug (로깅)

* **RL 은 NUC 에만 있다.** Pixhawk 에는 없다. 정책은 **ONNX** 배포(재학습 =
  파일 교체) → `stable_baselines3` 를 import 하지 않는다.
* **Latch-aware QP 는 RL 로 대체하지 않는다** (사용자 확정).

QP 를 50 Hz 로 도는 이유
------------------------
실물 할당 노드가 50 Hz 다. `τ_d` 는 10 Hz 로만 바뀌고 `LatchQP.solve` 는
결정론이므로 **수치결과는 10 Hz 로 도는 것과 같다.** 실제 레이트를 재현해 두면
지연·드랍 주입 실험에서 레이트 불일치가 드러난다.

호환성
------
외부에서 쓰는 이름(`NucNode`, `reset`, `tick_policy`, `tick_qp`, `u6`, `marg`,
`n_miss`, `build_obs`)은 **S3 과 동일하게 유지한다.** `run_harness.py` ·
`tools_sweep_comm.py` · `tests/` 를 고치지 않기 위해서다. 상태값은 제어기가
들고 있고 여기서는 property 로 비쳐 보낸다.
"""
from __future__ import annotations

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, os.path.join(_ROOT, "pontoon_rl")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from iface.messages import (AllocationDebug, MothershipData,   # noqa: E402
                            PontoonState, TugCommand)
from nodes.topics import (TOPIC_CMD, TOPIC_DBG, TOPIC_STATE,   # noqa: E402
                          TOPIC_TAU_D)
from nuc.controller import NucController                       # noqa: E402


class NucNode(object):
    """`NucController` 의 전송 어댑터. 정책 10 Hz + QP 50 Hz."""

    def __init__(self, transport, onnx_path=None, cfg=None, goal=None):
        self.tp = transport
        # ★ 제어 수식 전부를 production 제어기에 위임한다.
        self.ctl = NucController(onnx_path=onnx_path, cfg=cfg, goal=goal)
        self.n_miss = 0

    # -- 수명주기 -----------------------------------------------------------
    def reset(self):
        self.ctl.reset()
        self.n_miss = 0            # 전송 통계는 어댑터 소유 (제어기와 무관)

    def set_goal(self, goal):
        self.ctl.set_goal(goal)

    # -- 제어기 상태 비쳐보내기 (하위호환) -----------------------------------
    # 전부 읽기 전용이다. 쓰는 쪽은 제어기뿐이다.
    @property
    def c(self):          return self.ctl.c

    @property
    def qp(self):         return self.ctl.qp

    @property
    def policy(self):     return self.ctl.policy

    @property
    def tau_max(self):    return self.ctl.tau_max

    @property
    def goal(self):       return self.ctl.goal

    @property
    def a_prev(self):     return self.ctl.a_prev

    @property
    def res(self):        return self.ctl.res

    @property
    def marg(self):       return self.ctl.marg

    @property
    def tau_d(self):      return self.ctl.tau_d

    @property
    def u6(self):         return self.ctl.u6

    @property
    def last_obs(self):   return self.ctl.last_obs

    @property
    def n_policy(self):   return self.ctl.n_policy

    @property
    def n_qp(self):       return self.ctl.n_qp

    @property
    def qp_us(self):      return self.ctl.qp_us

    # -- 관측 조립: 메시지 → 숫자 (전송 어댑터의 일) ------------------------
    def build_obs(self, st: PontoonState) -> np.ndarray:
        """`PontoonState` 를 풀어서 제어기에 넘긴다. 수식은 제어기에 있다."""
        return self.ctl.build_obs(st.nu(), st.eta(), st.r1(), st.r2())

    # -- 10 Hz: 정책 --------------------------------------------------------
    def tick_policy(self, t_us: int):
        st = self.tp.latest(TOPIC_STATE)
        if not isinstance(st, PontoonState):
            self.n_miss += 1
            return None
        tau_d, a = self.ctl.policy_step(st.nu(), st.eta(), st.r1(), st.r2())
        self.tp.publish(TOPIC_TAU_D, MothershipData.from_tau(t_us, tau_d))
        return a

    # -- 50 Hz: 추력배분 QP -------------------------------------------------
    def tick_qp(self, t_us: int, from_topic: bool = True):
        """`τ_d` 를 `/mothership_data` 에서 받아 QP 를 푼다.

        실물 NUC 는 rl_node 와 latch_qp_node 가 분리된 두 프로세스다. 그래서
        `τ_d` 는 토픽을 거쳐 오며 그 payload 가 **float32** (`<Qfff`) 다
        → ★ **여기가 양자화 지점이다.** `from_topic=False` 면 제어기 내부
        float64 를 바로 쓴다 (`pontoon_rl` 직접 평가와 같은 경로 = 진단 비교군).
        """
        if from_topic:
            m = self.tp.latest(TOPIC_TAU_D)
            tau_d = (np.asarray(m.tau(), dtype=np.float64)
                     if isinstance(m, MothershipData) else np.zeros(3))
        else:
            tau_d = self.ctl.tau_d

        u6, tau_a = self.ctl.allocate(tau_d)
        res, marg = self.ctl.res, self.ctl.marg

        self.tp.publish(TOPIC_CMD, TugCommand.from_tau6(t_us, u6))
        self.tp.publish(TOPIC_DBG, AllocationDebug(
            t_us=t_us,
            tau_x=tau_d[0], tau_y=tau_d[1], tau_n=tau_d[2],
            tau_real_x=tau_a[0], tau_real_y=tau_a[1], tau_real_n=tau_a[2],
            error_x=res[0], error_y=res[1], error_n=res[2],
            ok=1, saturated=self.ctl.saturated,
            solve_time_us=self.ctl.qp_us,
            marg_one=float(marg[0]), marg_two=float(marg[1])))
        return u6
