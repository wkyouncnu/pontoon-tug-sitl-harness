# -*- coding: utf-8 -*-
"""harness_sitl/nodes/node_b_mothership.py — 노드 B: 마더십 Pixhawk 바이패스.

역할 (실물 대응)
----------------
* 실물: 마더십 Pixhawk 의 EKF2 출력(uORB)을 **그대로 NUC 로 흘려보내는**
  Simulink 바이패스 앱. RL 은 Pixhawk 에 없다 (사용자 확정).
* 입력: 플랜트 상태 (실물에서는 센서 → EKF2)
      /fmu/out/vehicle_local_position    px4_msgs/msg/VehicleLocalPosition
      /fmu/out/vehicle_attitude          px4_msgs/msg/VehicleAttitude
      /fmu/out/vehicle_angular_velocity  px4_msgs/msg/VehicleAngularVelocity
      /fmu/out/vehicle_status            px4_msgs/msg/VehicleStatus
* 출력: `/pontoon/state` (PontoonState, 50 Hz, **float32**)

왜 이 노드가 별도로 있어야 하는가
----------------------------------
1. **양자화 지점이 여기다.** uORB/DDS 는 float32 다. 노드 A→B 는 물리 측정이라
   양자화가 없고(`serialize=False`), B→C 가 실제 전송선이다(`serialize=True`).
2. **좌표계 책임이 여기서 끝난다.** PX4 = NED 월드 + FRD 바디 = Fossen/SNAME 과
   동일하므로 **부호 반전이 없다** (CLAUDE.md §2.5.0). 월드 속도를 바디로 돌리는
   변환만 Pixhawk 안에서 일어나고, 여기서는 이미 바디 `(u,v,r)` 이다.
3. **모드 게이트.** MODE_MANUAL 이면 RL 경로를 끊는다 (실물 안전장치).

⚠️ 확인 필요
------------
* `/pontoon/state` 에 대응하는 실물 `.msg` 가 아직 없다. BEST_PLAN §16.1 규약만
  존재한다 → `iface/ros2_stub.py` TOPIC_MAP 에 `confirmed=False` 로 표시했다.
* 모멘트암 `r1/r2` 를 상태에 실어 보내는 것은 **학습된 정책의 관측 규약**이다.
  실물에서는 이 값을 측정해 넣어야 한다(부착위치 추정 S1~S3). 현재는 플랜트의
  **지터된 참값**이 그대로 들어간다 → 문서의 한계 목록에 명시.
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from iface.messages import MODE_MANUAL, MODE_RL, PontoonState   # noqa: E402
from nodes.topics import TOPIC_PLANT, TOPIC_STATE               # noqa: E402


class MothershipBypassNode(object):
    """Pixhawk 바이패스. 상태를 받아 NUC 로 재발행한다."""

    def __init__(self, transport, mode: int = MODE_RL):
        self.tp = transport
        self.mode = int(mode)
        self.n_fwd = 0
        self.n_miss = 0
        self.last = None

    def reset(self) -> None:
        self.n_fwd = 0
        self.n_miss = 0
        self.last = None

    def set_mode(self, mode: int) -> None:
        self.mode = int(mode)

    def tick(self):
        """플랜트 상태를 1회 중계한다. 50 Hz."""
        m = self.tp.latest(TOPIC_PLANT)
        if not isinstance(m, PontoonState):
            self.n_miss += 1
            return None
        if self.mode == MODE_MANUAL:
            # 수동 모드: RL 경로를 끊는다. 상태는 계속 올려보낸다(모니터링용).
            m = PontoonState(**{**m.__dict__, "valid": 0})
        self.last = m
        self.tp.publish(TOPIC_STATE, m)
        self.n_fwd += 1
        return m
