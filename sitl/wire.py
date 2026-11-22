# -*- coding: utf-8 -*-
"""harness_sitl/sitl/wire.py — SITL 플랜트쪽 MAVLink 송수신 (**가상 전용**).

이 파일이 서는 자리
-------------------
이 파일은 **우리 코드가 아닌 것들을 대역한다**:

| 대역 대상 | 실물에서 누구 | 여기서 하는 일 |
|---|---|---|
| 마더십 Simulink + 마더십 ESP32 | 삼성중공업 과제측 | `52501`·`52502`·HEARTBEAT 를 UDP 로 **송신** |
| 터그 ESP32 + 터그 Pixhawk | 과제측 | `52503` 을 UDP 로 **수신** |

즉 **NUC 밖의 세계**다. `nuc/` 와 `vendor/tug_bridge/` 는 이 파일을 모른다.

왜 패킷을 직접 만들지 않는가
----------------------------
사용자 지시: **자체 패킷 구현 금지.** 그래서 두 가지를 그대로 쓴다.

1. **dialect**: `mavlink/gen/usv_tug.py` (우리가 `gen_dialect.py` 로 생성한 것).
   `mothership_state_encode` 등 인코더가 이미 있다.
2. **seq 증가**: `vendor/tug_bridge/mav_link.py::pack_and_advance`.
   pymavlink 의 `msg.pack(mav)` 는 **seq 를 올리지 않는다** — 그대로 쓰면
   수신측 `SeqTracker` 가 손실률 235.9 % 를 보고한다(실측). 벤더가 이미
   고쳐 둔 함수를 import 해서 그 버그를 되풀이하지 않는다.

⚠️ `relay_common.py::MavEncoder` 에는 **`mothership_state` 인코더가 없다.**
`relay_common.py` 는 diff 0 을 지켜야 해서 고칠 수 없으므로, 송신측 인코더만
이 파일에서 새로 만든다. **수신측(NUC)은 벤더 코드가 그대로 처리한다.**

소켓은 `socket` 표준모듈을 직접 쓴다. 벤더의 `MavUdpLink` 는 NUC 노드용
(재바인드·백오프·IP 필터)이고, 이쪽은 과제측 장비 대역이라 그 정책을
흉내낼 이유가 없다. **"패킷 금지"는 프레이밍 얘기이고 소켓은 전송이다.**
"""
from __future__ import annotations

import math
import socket
from typing import Any, List, Optional, Tuple

from mavlink.gen import usv_tug as dialect          # 우리 dialect (하네스 소유)
from tug_bridge.mav_link import pack_and_advance    # 벤더 (seq 증가 보정)

MSG_ID_HEARTBEAT = 0
MSG_ID_MOTHERSHIP_STATE = getattr(
    dialect, "MAVLINK_MSG_ID_MOTHERSHIP_STATE", 52501)
MSG_ID_MOTHERSHIP_DATA = getattr(
    dialect, "MAVLINK_MSG_ID_MOTHERSHIP_DATA", 52502)
MSG_ID_TUG_COMMAND = getattr(
    dialect, "MAVLINK_MSG_ID_TUG_COMMAND", 52503)

# 마더십 Pixhawk 의 식별자. `tug_bridge.yaml` 의 `filter_sysid: 1` 과 맞는다.
MOTHERSHIP_SYSID = 1
MOTHERSHIP_COMPID = 1

# ★ NUC → 마더십 HEARTBEAT 목적지 (`tug_bridge.yaml` 의 heartbeat_target_port).
#   실물에서는 **마더십 ESP32 가 여기를 듣는다.** 하네스에서 이 포트를 비워 두면
#   Windows 가 ICMP port-unreachable 을 **같은 UDP 소켓의 recvfrom** 에
#   WSAECONNRESET(10054) 로 올려 준다 → 벤더 `MavUdpLink` 가 소켓을 닫고
#   재바인드하고(`reopen_count`), 닫힌 동안 도착한 52501/52502 가 전부 사라진다.
#   실측: HB 1 Hz → 10 s 에 재바인드 10 회, 상행 손실 **약 50 %** (송신율과 무관).
#   → 그래서 "듣기만 하고 버리는" 소켓을 반드시 열어 둔다. **벤더 코드는 고치지 않는다.**
HB_LISTEN_PORT = 14660


class MothershipTx(object):
    """마더십(sysid 1 / compid 1) → NUC `:14600` 송신기.

    `52501`(상태) 과 `52502`(τ) 를 **같은 포트로** 보낸다 — 사용자 결정
    (`CLAUDE.md` §9 2026-11-22 오후). 그래서 NUC 쪽 파서·`allowed_src_ip`
    설정이 하나로 유지된다.
    """

    def __init__(self, dst_ip: str = "127.0.0.1", dst_port: int = 14600,
                 sysid: int = MOTHERSHIP_SYSID,
                 compid: int = MOTHERSHIP_COMPID,
                 hb_listen_port: int = HB_LISTEN_PORT) -> None:
        self.dst = (dst_ip, int(dst_port))
        self.mav = dialect.MAVLink(None, srcSystem=int(sysid),
                                   srcComponent=int(compid))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.n_state = 0
        self.n_px4std = 0
        self.n_data = 0
        self.n_hb = 0
        self.n_err = 0
        # ★ NUC 의 HEARTBEAT 수신구 — 반드시 열어 둬야 한다 (아래 주석 참조)
        self.hb_sock = None
        self.n_hb_rx = 0
        if hb_listen_port:
            try:
                hs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                hs.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                hs.bind(("127.0.0.1", int(hb_listen_port)))
                hs.setblocking(False)
                self.hb_sock = hs
            except OSError:
                self.hb_sock = None

    def drain_hb(self, limit: int = 64) -> int:
        """NUC HEARTBEAT 를 비운다. **읽는 것이 목적이 아니라 버리지 않는 것이 목적**이다."""
        if self.hb_sock is None:
            return 0
        got = 0
        for _ in range(int(limit)):
            try:
                if not self.hb_sock.recv(2048):
                    break
            except (BlockingIOError, OSError):
                break
            got += 1
        self.n_hb_rx += got
        return got

    # -- 프레이밍 -----------------------------------------------------------
    def _send(self, msg: Any) -> int:
        buf = pack_and_advance(self.mav, msg)
        try:
            self.sock.sendto(buf, self.dst)
        except OSError:
            self.n_err += 1
            return 0
        return len(buf)

    # -- 52501: 상태 (★ S5 신설) --------------------------------------------
    def state(self, t_us: int, eta: Any, nu: Any, goal: Any,
              scenario_mode: int = 3, rc_on_off: int = 0,
              on_off: int = 1) -> int:
        """`MOTHERSHIP_STATE`. `eta=(x,y,psi)` NED, `nu=(u,v,r)` 바디 FRD.

        ⚠️ **부착점 `r1`·`r2` 는 이 메시지에 없다** (사용자 확정 필드집합).
        실물 NUC 은 설정의 공칭값을 쓴다 → `docs/SITL.md` 의 r1/r2 항목.
        """
        msg = self.mav.mothership_state_encode(
            int(t_us),
            float(eta[0]), float(eta[1]), float(eta[2]),
            float(nu[0]), float(nu[1]), float(nu[2]),
            float(goal[0]), float(goal[1]), float(goal[2]),
            int(scenario_mode), int(rc_on_off), int(on_off))
        self.n_state += 1
        return self._send(msg)

    # -- PX4 표준 3종 (★ 52501 을 쓰지 않는 입력경로) -------------------------
    def state_px4std(self, t_us: int, eta: Any, nu: Any, goal: Any,
                     send_ref: bool = True) -> int:
        """`ATTITUDE` + `LOCAL_POSITION_NED` + `POSITION_TARGET_LOCAL_NED`.

        **다이얼렉트를 바꾸지 않는다** — 세 메시지는 기존 `usv_tug.xml`
        (`<include>development.xml</include>`) 에 이미 들어 있다. 마더십
        Simulink 도 바꾸지 않는다: PX4 `mavlink` 모듈이 원래 내보내는 것이다.
        ⚠️ 실기체가 **실제로 이 셋을 내보내는지**는 확인 필요
        (`tools/sniff_mothership.py` 가 현장에서 1 분에 답한다).

        `nu` 는 바디속도이므로 여기서 **NED 로 되돌려** 보낸다 — 실물 PX4 가
        NED 속도를 내보내기 때문이다. NUC 쪽 `Px4StateAssembler` 가 다시
        바디로 돌린다. 왕복이 항등인지는 `tests/test_px4_state.py` 가 잰다.
        """
        t_ms = int(t_us // 1000) & 0xFFFFFFFF
        psi = float(eta[2])
        c, s = math.cos(psi), math.sin(psi)
        vx = c * float(nu[0]) - s * float(nu[1])
        vy = s * float(nu[0]) + c * float(nu[1])

        n = 0
        n += self._send(self.mav.attitude_encode(
            t_ms, 0.0, 0.0, psi, 0.0, 0.0, float(nu[2])))
        n += self._send(self.mav.local_position_ned_encode(
            t_ms, float(eta[0]), float(eta[1]), 0.0, vx, vy, 0.0))
        if send_ref:
            n += self._send(self.mav.position_target_local_ned_encode(
                t_ms, dialect.MAV_FRAME_LOCAL_NED, 0,
                float(goal[0]), float(goal[1]), 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                float(goal[2]), 0.0))
        self.n_px4std += 1
        return n

    # -- 52502: τ (기존 경로. 손대지 않는다) ---------------------------------
    def data(self, t_us: int, tau: Any) -> int:
        """`MOTHERSHIP_DATA`. 기존 LOS/PID 경로가 살아 있음을 보이기 위한 것.

        RL 경로는 이 값을 쓰지 않는다. **두 메시지가 같은 포트에서 공존하는지**
        가 S5 의 검수항목이므로 공칭에서도 계속 보낸다.
        """
        msg = self.mav.mothership_data_encode(
            int(t_us), float(tau[0]), float(tau[1]), float(tau[2]))
        self.n_data += 1
        return self._send(msg)

    def heartbeat(self) -> int:
        msg = self.mav.heartbeat_encode(
            dialect.MAV_TYPE_ONBOARD_CONTROLLER,
            dialect.MAV_AUTOPILOT_INVALID, 0, 0, dialect.MAV_STATE_ACTIVE)
        self.n_hb += 1
        return self._send(msg)

    def close(self) -> None:
        for sk in (self.sock, self.hb_sock):
            if sk is None:
                continue
            try:
                sk.close()
            except OSError:
                pass


class TugRx(object):
    """터그 1대(ESP32 + Pixhawk) 대역. `52503` 을 받아 **자기 몫**만 꺼낸다.

    ★ 와이어에는 터그 2대 값이 **한 메시지**에 실리고, 어느 쪽을 쓸지는
    각 터그의 Simulink 가 고른다 (`CLAUDE.md` §9 2026-11-22 오후).
    `which=1` 이면 `tau_x_one/..`, `which=2` 면 `tau_x_two/..` 를 쓴다.
    여기서 그 선택을 재현한다 — 두 터그가 같은 포트를 공유하지 않고
    각자 `:14651` / `:14652` 를 바인드하는 것도 실제 구성과 같다.
    """

    def __init__(self, bind_port: int, which: int,
                 bind_ip: str = "127.0.0.1",
                 accepted_sysid: int = 250) -> None:
        if int(which) not in (1, 2):
            raise ValueError("which 는 1 또는 2 여야 한다 (터그 번호)")
        self.which = int(which)
        self.accepted_sysid = int(accepted_sysid)
        self.mav = dialect.MAVLink(None, srcSystem=0, srcComponent=0)
        self.mav.robust_parsing = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, int(bind_port)))
        self.sock.setblocking(False)
        self.n_rx = 0
        self.n_drop_sysid = 0
        self.n_hb = 0
        self.last_tau = (0.0, 0.0, 0.0)
        self.last_t_us = 0

    def drain(self, limit: int = 256) -> int:
        """도착한 데이터그램을 전부 읽고 **마지막 τ** 를 남긴다.

        터그 Pixhawk 는 **수신 타임아웃이 없어 마지막 값을 유지한다**
        (`CLAUDE.md` §9). 그 위험한 성질을 그대로 모사하려면 여기서
        값을 지우지 않아야 한다 — `last_tau` 는 덮어쓸 뿐 0 으로
        되돌리지 않는다.
        """
        got = 0
        for _ in range(int(limit)):
            try:
                buf = self.sock.recv(65535)
            except (BlockingIOError, OSError):
                break
            if not buf:
                break
            try:
                msgs = self.mav.parse_buffer(buf) or []
            except Exception:
                continue
            for m in msgs:
                mid = m.get_msgId()
                if mid == MSG_ID_HEARTBEAT:
                    self.n_hb += 1
                    continue
                if mid != MSG_ID_TUG_COMMAND:
                    continue
                if m.get_srcSystem() != self.accepted_sysid:
                    self.n_drop_sysid += 1
                    continue
                if self.which == 1:
                    self.last_tau = (float(m.tau_x_one), float(m.tau_y_one),
                                     float(m.tau_n_one))
                else:
                    self.last_tau = (float(m.tau_x_two), float(m.tau_y_two),
                                     float(m.tau_n_two))
                self.last_t_us = int(m.timestamp)
                self.n_rx += 1
                got += 1
        return got

    def tau(self) -> Tuple[float, float, float]:
        return self.last_tau

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


def drain_all(rxs: List[TugRx]) -> int:
    return sum(r.drain() for r in rxs)
