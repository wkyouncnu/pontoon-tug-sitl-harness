# -*- coding: utf-8 -*-
"""harness_sitl/nodes/node_d_esp32.py — 노드 D: 터그 링크 어댑터 (**가상 전용**).

이 파일의 지위 — ★ S4 에서 바뀠다
----------------------------------
**하위 추력배분 수식은 이 파일에 없다.** `plant/tug_lower.py::TugLowerAllocator`
로 옮겼다. 두 가지가 섞여 있었기 때문이다:

| 내용 | 실물에서 어디 |
|---|---|
| MAVLink v2 프레이밍 (msgid 52503) | **실제 ESP32 / Wi-Fi UDP 가 대신한다** |
| 터그 하위 역행렬 `f = B_F⁻¹ τ` + 포화 | **터그 Pixhawk 안에서 돈다** (우리가 배포하지 않는다) |

둘 다 실물에서는 사라지지만 **사라지는 곳이 다르다.** 그래서 전송은 여기
(`nodes/`), 수식은 `plant/` 로 갈랐다. → `docs/FILE_SWAP.md` §2.

역할 (실물 대응)
----------------
    NUC  ──/tug_command (ROS 2)──▶ tug_bridge/tug_tx.py
         ──MAVLink v2 msgid 52503 (UDP)──▶ ESP32 ──▶ 터그 Pixhawk
                                                  └─ 하위 역행렬 → 추진기 3기

* 입력: `/tug_command` (TugCommand) — 터그별 일반화력 `τ_i = [X_i, Y_i, N_i]`
* 출력: `/tug/thrust` (TugThrust) — 추진기가 실제로 낸 힘과 그 합력
* **실물 전송형식을 그대로 통과시킨다**: MAVLink v2 프레이밍(msgid 52503,
  crc_extra 16, payload `<Qdddddd`, trailing-zero truncation)으로 pack → unpack.
  payload 가 **float64 라 이 경로에는 양자화가 없다** (실측 왕복오차 0).
  → 양자화가 일어나는 곳은 상행 `/mothership_data` (`<Qfff`, float32) 뿐이다.

⚠️ **터그 쪽은 전혀 바뀌지 않는다.** 52503 은 S5 에서도 손대지 않는다
(새 메시지는 상행 52501 뿐). `tug_tx.py` diff 0 이 그 근거다.

호환성
------
외부에서 쓰는 이름(`Esp32LinkNode`, `reset`, `tick`, `_allocate`, `n_rx`,
`n_miss`, `n_sat`, `body_force_matrix`)은 **S3 과 동일하게 유지한다.**
`tests/test_acceptance.py::test_lower_allocation_roundtrip` 이 `_allocate` 를
직접 부르므로 그 이름을 지우지 않는다.
"""
from __future__ import annotations

import os
import sys

import numpy as np   # noqa: F401  (하위호환 import 유지)

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, os.path.join(_ROOT, "pontoon_rl")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from iface.mavlink_wire import (MSG_TUG_COMMAND, MavSeq,          # noqa: E402
                                pack_frame, unpack_frame)
from iface.messages import TugCommand, TugThrust                  # noqa: E402
from nodes.topics import TOPIC_CMD, TOPIC_THRUST                  # noqa: E402
from plant.tug_lower import (TugLowerAllocator,                   # noqa: E402,F401
                             body_force_matrix)

from env.pontoon_env import _load, CFG_DEFAULT                    # noqa: E402


class Esp32LinkNode(object):
    """MAVLink 왕복(전송) + `TugLowerAllocator` 호출(플랜트)."""

    def __init__(self, transport, cfg=None, mavlink=True,
                 pwm_quantize=False, pwm_step_us=1.0, k_force_per_us=None):
        self.tp = transport
        self.c = _load(cfg or CFG_DEFAULT) if not isinstance(cfg, dict) else cfg
        # ★ 하위 역행렬·포화·PWM 양자화는 전부 plant/ 로 위임한다.
        self.alloc = TugLowerAllocator.from_cfg(
            self.c, pwm_quantize=pwm_quantize, pwm_step_us=pwm_step_us,
            k_force_per_us=k_force_per_us)
        self.mavlink = bool(mavlink)
        self.seq = MavSeq()
        self.reset()

    def reset(self):
        self.n_rx = 0
        self.n_miss = 0
        self.alloc.reset()         # n_sat 은 배분기가 센다
        self.seq = MavSeq()
        self.last = None

    # -- 배분기 상태 비쳐보내기 (하위호환) -----------------------------------
    @property
    def n_sat(self):      return self.alloc.n_sat

    @property
    def B(self):          return self.alloc.B

    @property
    def Binv(self):       return self.alloc.Binv

    @property
    def f_max(self):      return self.alloc.f_max

    @property
    def f_min(self):      return self.alloc.f_min

    @property
    def pwm_quantize(self):  return self.alloc.pwm_quantize

    # -- 실물 전송형식 왕복 (이 파일의 본업) ---------------------------------
    def _wire_roundtrip(self, cmd: TugCommand) -> TugCommand:
        if not self.mavlink:
            return cmd
        frame = pack_frame(MSG_TUG_COMMAND, cmd.pack_mavlink_payload(),
                           self.seq.advance())
        mid, payload, _ = unpack_frame(frame, expect_len=56)
        assert mid == MSG_TUG_COMMAND
        return TugCommand.unpack_mavlink_payload(payload)

    # -- 하위 할당: plant/ 로 위임 ------------------------------------------
    def _allocate(self, tau):
        """하위호환 별칭. 실제 계산은 `plant/tug_lower.py` 에 있다."""
        return self.alloc.allocate(tau)

    # -- 50 Hz 틱 -----------------------------------------------------------
    def tick(self, t_us: int):
        cmd = self.tp.latest(TOPIC_CMD)
        if not isinstance(cmd, TugCommand):
            self.n_miss += 1
            return None
        cmd = self._wire_roundtrip(cmd)
        self.n_rx += 1
        f1, f2, ta1, ta2 = self.alloc.allocate_pair(cmd.tau_one(), cmd.tau_two())
        out = TugThrust(t_us=int(t_us), f_one=f1, f_two=f2,
                        tau_one=ta1, tau_two=ta2)
        self.last = out
        self.tp.publish(TOPIC_THRUST, out)
        return out
