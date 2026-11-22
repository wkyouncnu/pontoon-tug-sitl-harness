# -*- coding: utf-8 -*-
"""harness_sitl/sitl/px4_state_node.py — PX4 표준 상태를 `/mothership/state` 로 올린다.

이 파일의 지위
--------------
`RlPolicyNode` 는 `/mothership/state` 하나만 본다. 그 토픽을 **누가 채우는가**
만 바꾸는 것이 이 파일이다 → 정책·QP·터그 경로의 diff 가 **0** 이다.

| 입력경로 | `/mothership/state` 발행자 | 마더십 변경 | 다이얼렉트 변경 | 벤더 코드 변경 |
|---|---|---|---|---|
| 신설 52501 | `vendor/mothership_rx.py` (가산 패치) | **필요** | 필요 | 가산 |
| **PX4 표준** | **이 파일** | **없음** | **없음** | **없음** |

표준경로가 쓰는 세 메시지(`ATTITUDE` · `LOCAL_POSITION_NED` ·
`POSITION_TARGET_LOCAL_NED`)는 기존 `usv_tug.xml` 의
`<include>development.xml</include>` 에 **이미 들어 있다**(실측: 생성 다이얼렉트
224 종에 전부 포함). 참조 구현은 캡스톤 UGV
`auto_usv_pkg/coordinate_transformer_node.py` 다 — 그 노드가 PX4 상태를
받아 `/usv/state/*` 로 다시 올리는 자리에, 이 노드가 `/mothership/state` 를 올린다.

⚠️ **확인 필요** — 마더십 Pixhawk 가 이 세 메시지를 실제로 내보내는지는
측정하지 않았다. `tools/sniff_mothership.py` 가 현장에서 1 분에 답한다.
수신 필터·재바인드·시퀀스 처리는 `vendor/tug_bridge/mothership_rx.py:129–175`
와 같은 순서로 둔다 (벤더 코드를 고치지 않고 **같은 방식**을 쓴다).
"""
from __future__ import annotations

import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tug_interfaces.msg import MothershipState

from nuc.px4_state import LlaOrigin, Px4StateAssembler

TOPIC_STATE_OUT = "/mothership/state"
RX_POLL_S = 0.02
MOTHERSHIP_SYSID = 1
MOTHERSHIP_COMPID = 1


def _qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)


class Px4StdStateNode(Node):
    """MAVLink 표준 텔레메트리(UDP) → `MothershipState`.

    전송은 **벤더 `UdpLink` 를 그대로 쓴다** (재바인드·IP 필터·백오프가 이미
    현장에서 돈 코드다). 조립은 `nuc/px4_state.py` 가 한다 — 전송을 모른다.
    """

    def __init__(self, bind_ip: str = "127.0.0.1", bind_port: int = 14600,
                 allowed_src_ip: str = "",
                 sysid: int = MOTHERSHIP_SYSID,
                 compid: int = MOTHERSHIP_COMPID,
                 stale_s: float = 0.5,
                 origin: Optional[LlaOrigin] = None,
                 psi_ref_from_los: bool = False,
                 forward_addr: Optional[tuple] = None,
                 **kwargs) -> None:
        super().__init__("px4_state", **kwargs)
        from tug_bridge.mav_link import MavUdpLink       # 벤더 전송
        from tug_bridge.relay_common import is_verified  # 벤더 검증

        self._is_verified = is_verified
        self.filter_sysid = int(sysid)
        self.filter_compid = int(compid)
        self.asm = Px4StateAssembler(stale_s=stale_s, origin=origin,
                                     psi_ref_from_los=psi_ref_from_los)
        self.forward_addr = forward_addr

        self.n_pub = 0
        self.n_drop_sysid = 0
        self.n_drop_msgid = 0
        self.n_bad = 0
        self.n_incomplete = 0
        self._warned = False

        self.pub = self.create_publisher(MothershipState, TOPIC_STATE_OUT,
                                         _qos())
        self.link = MavUdpLink(bind_ip, int(bind_port), self.get_logger(),
                               allowed_src_ip=allowed_src_ip)
        self.link.open()
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._rx_loop, daemon=True)
        self._thr.start()

    # -- 수신 스레드 (벤더와 같은 순서) --------------------------------------
    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self.link.service()
                self.link.wait_readable(RX_POLL_S)
                for msg, addr in self.link.recv_messages():
                    self._handle(msg, addr)
            except Exception:                      # 수신 스레드는 죽지 않는다
                time.sleep(RX_POLL_S)

    def _handle(self, msg: Any, addr: Any) -> None:
        t = msg.get_type()
        if t == "BAD_DATA":
            self.n_bad += 1
            return
        if not self._is_verified(msg):
            self.n_drop_msgid += 1
            return
        if msg.get_srcSystem() != self.filter_sysid or (
                self.filter_compid != 0
                and msg.get_srcComponent() != self.filter_compid):
            self.n_drop_sysid += 1
            return
        if self.forward_addr is not None:
            # 기존 52502 배분경로를 살려 두려면 같은 패킷을 그 노드로 넘긴다.
            # 벤더 코드는 그대로 두고 **포트만** 바꾸면 된다 (yaml 설정).
            try:
                self.link.sendto(msg.get_msgbuf(), self.forward_addr)
            except Exception:
                pass

        now = time.monotonic()
        if t == "ATTITUDE":
            self.asm.on_attitude(now, msg.yaw, msg.yawspeed,
                                 t_us=int(msg.time_boot_ms) * 1000)
        elif t == "LOCAL_POSITION_NED":
            self.asm.on_local_position(now, msg.x, msg.y, msg.vx, msg.vy,
                                       t_us=int(msg.time_boot_ms) * 1000)
            self._emit(now)
        elif t == "GLOBAL_POSITION_INT":
            self.asm.on_global_position(
                now, msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3,
                msg.vx * 1e-2, msg.vy * 1e-2,
                t_us=int(msg.time_boot_ms) * 1000)
            self._emit(now)
        elif t == "POSITION_TARGET_LOCAL_NED":
            self.asm.on_position_target(now, msg.x, msg.y, msg.yaw,
                                        t_us=int(msg.time_boot_ms) * 1000)

    # -- 발행 ---------------------------------------------------------------
    def _emit(self, now: float) -> None:
        st = self.asm.state(now)
        if st is None:
            self.n_incomplete += 1
            if not self._warned and self.asm.n_pos > 25:
                self._warned = True
                self.get_logger().warn(
                    "PX4 표준 상태가 모자라다: %s" % (self.asm.missing(now),))
            return
        m = MothershipState()
        m.timestamp = int(st.t_us)
        m.x, m.y, m.psi = st.x, st.y, st.psi
        m.u, m.v, m.r = st.u, st.v, st.r
        m.x_ref, m.y_ref, m.psi_ref = st.x_ref, st.y_ref, st.psi_ref
        m.scenario_mode = 3
        m.rc_on_off = int(st.rc_on_off)
        m.on_off = int(st.on_off)
        self.pub.publish(m)
        self.n_pub += 1

    def stats(self) -> dict:
        d = {"n_pub": self.n_pub, "n_bad": self.n_bad,
             "n_drop_sysid": self.n_drop_sysid,
             "n_drop_msgid": self.n_drop_msgid,
             "n_incomplete": self.n_incomplete}
        d.update(self.asm.stats())
        return d

    def destroy_node(self) -> bool:
        self._stop.set()
        try:
            self.link.close()
        except Exception:
            pass
        return super().destroy_node()


class Px4DdsStateNode(Node):
    """uXRCE-DDS(`px4_msgs`) 경로 — 캡스톤 UGV 노드와 **같은 구독 집합**.

    ⚠️ **이 환경에서 실행하지 못했다.** Windows 에 ROS 2·`px4_msgs` 가 없고
    하네스는 `shim/rclpy` 로 돈다. 조립식(`Px4StateAssembler`)은 MAVLink 경로와
    **같은 코드**라 시험되지만, 아래 구독 배선 자체는 **미검증**이다.
    참조: `coordinate_transformer_node.py:133–146` (토픽·QoS),
    `px4_mission_msg_pub_node.py:21–70` (같은 QoS 를 `/fmu/in/*` 에 쓰는 예).
    """

    def __init__(self, stale_s: float = 0.5,
                 psi_ref_from_los: bool = False, **kwargs) -> None:
        super().__init__("px4_dds_state", **kwargs)
        from px4_msgs.msg import (PositionSetpointTriplet, VehicleAngularVelocity,
                                  VehicleAttitude, VehicleLocalPosition)
        from rclpy.qos import DurabilityPolicy, HistoryPolicy

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.asm = Px4StateAssembler(stale_s=stale_s,
                                     psi_ref_from_los=psi_ref_from_los)
        self.n_pub = 0
        self._q = None
        self._r = 0.0

        self.pub = self.create_publisher(MothershipState, TOPIC_STATE_OUT,
                                         _qos())
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude",
                                 self._on_att, px4_qos)
        self.create_subscription(VehicleAngularVelocity,
                                 "/fmu/out/vehicle_angular_velocity",
                                 self._on_rate, px4_qos)
        self.create_subscription(VehicleLocalPosition,
                                 "/fmu/out/vehicle_local_position",
                                 self._on_pos, px4_qos)
        self.create_subscription(PositionSetpointTriplet,
                                 "/fmu/out/position_setpoint_triplet",
                                 self._on_ref, px4_qos)

    def _on_rate(self, msg) -> None:
        self._r = float(msg.xyz[2])            # 선체고정 z 각속도 = r

    def _on_att(self, msg) -> None:
        # `VehicleAttitude.q` 는 (w,x,y,z). UGV 노드와 같은 유효성 판정.
        self.asm.on_attitude_quat(time.monotonic(), list(msg.q), self._r,
                                  t_us=int(msg.timestamp))

    def _on_pos(self, msg) -> None:
        self.asm.on_local_position(time.monotonic(), msg.x, msg.y,
                                   msg.vx, msg.vy, t_us=int(msg.timestamp))
        self._emit()

    def _on_ref(self, msg) -> None:
        cur = msg.current
        yaw = getattr(cur, "yaw", None)
        self.asm.on_position_target(time.monotonic(), cur.x, cur.y, yaw)

    def _emit(self) -> None:
        st = self.asm.state(time.monotonic())
        if st is None:
            return
        m = MothershipState()
        m.timestamp = int(st.t_us)
        m.x, m.y, m.psi = st.x, st.y, st.psi
        m.u, m.v, m.r = st.u, st.v, st.r
        m.x_ref, m.y_ref, m.psi_ref = st.x_ref, st.y_ref, st.psi_ref
        m.scenario_mode = 3
        m.rc_on_off = int(st.rc_on_off)
        m.on_off = int(st.on_off)
        self.pub.publish(m)
        self.n_pub += 1


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = Px4StdStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
