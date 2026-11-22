# -*- coding: utf-8 -*-
"""harness_sitl/iface/messages.py — 하네스 메시지 dataclass + 직렬화.

`schema.yaml` 이 단일 정의이고, 이 파일은 그것을 코드로 반영한 것이다.
둘이 어긋나면 `tests/test_schema_sync.py` 가 실패한다.

설계 의도
---------
* 노드 A/B/C/D 는 **이 dataclass 만** 주고받는다. 전송계층(inproc / ros2 / mavlink)이
  바뀌어도 노드 본체는 고치지 않는다.
* `wire_dtype` 이 float32 인 메시지는 `to_wire()` 에서 실제로 float32 로 깎는다
  (PX4 uORB 가 float32 라서 실물에 존재하는 양자화를 하네스가 재현한다).
* `pack()` / `unpack()` 은 struct 바이트열이다. MAVLink 프레이밍은
  `mavlink_wire.py` 가 이 payload 를 감싼다.
"""
from __future__ import annotations

import os
import struct
from dataclasses import dataclass, field, fields as dc_fields
from typing import List, Tuple

import numpy as np

SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "schema.yaml")
SCHEMA_VERSION = 1

F32 = np.float32


def _f32(x: float) -> float:
    """float32 로 깎아 다시 python float 로. 실물 uORB 양자화 재현."""
    return float(np.float32(x))


# ─────────────────────────────────────────────────────────────────────────────
# A → B → C : /pontoon/state   (50 Hz, wire float32)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class PontoonState:
    t_us: int = 0
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    u: float = 0.0
    v: float = 0.0
    r: float = 0.0
    r1x: float = 0.0
    r1y: float = 0.0
    r2x: float = 0.0
    r2y: float = 0.0
    valid: int = 1

    STRUCT = "<Q10fB"        # uint64 + 10 x float32 + uint8
    WIRE_DTYPE = "float32"

    # -- 편의 ---------------------------------------------------------------
    @staticmethod
    def from_plant(t_us: int, s6: np.ndarray, r1: np.ndarray, r2: np.ndarray,
                   valid: int = 1) -> "PontoonState":
        """s6 = [u, v, r, x, y, psi]  (pontoon_env 의 상태 순서)."""
        return PontoonState(t_us=int(t_us),
                            x=float(s6[3]), y=float(s6[4]), psi=float(s6[5]),
                            u=float(s6[0]), v=float(s6[1]), r=float(s6[2]),
                            r1x=float(r1[0]), r1y=float(r1[1]),
                            r2x=float(r2[0]), r2y=float(r2[1]),
                            valid=int(valid))

    def nu(self) -> np.ndarray:
        return np.array([self.u, self.v, self.r], dtype=np.float64)

    def eta(self) -> np.ndarray:
        return np.array([self.x, self.y, self.psi], dtype=np.float64)

    def r1(self) -> np.ndarray:
        return np.array([self.r1x, self.r1y], dtype=np.float64)

    def r2(self) -> np.ndarray:
        return np.array([self.r2x, self.r2y], dtype=np.float64)

    # -- 직렬화 -------------------------------------------------------------
    def to_wire(self) -> "PontoonState":
        """float32 양자화를 적용한 사본."""
        return PontoonState(self.t_us,
                            _f32(self.x), _f32(self.y), _f32(self.psi),
                            _f32(self.u), _f32(self.v), _f32(self.r),
                            _f32(self.r1x), _f32(self.r1y),
                            _f32(self.r2x), _f32(self.r2y), self.valid)

    def pack(self) -> bytes:
        return struct.pack("<Q10fB", self.t_us, self.x, self.y, self.psi,
                           self.u, self.v, self.r,
                           self.r1x, self.r1y, self.r2x, self.r2y, self.valid)

    @staticmethod
    def unpack(b: bytes) -> "PontoonState":
        t = struct.unpack("<Q10fB", b)
        return PontoonState(t[0], *[float(x) for x in t[1:11]], valid=int(t[11]))


# ─────────────────────────────────────────────────────────────────────────────
# mission → C : /pontoon/goal   (1 Hz, wire float32)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class PontoonGoal:
    t_us: int = 0
    gx: float = 0.0
    gy: float = 0.0
    gpsi: float = 0.0
    scenario: int = 3

    WIRE_DTYPE = "float32"

    def goal(self) -> np.ndarray:
        return np.array([self.gx, self.gy, self.gpsi], dtype=np.float64)

    def to_wire(self) -> "PontoonGoal":
        return PontoonGoal(self.t_us, _f32(self.gx), _f32(self.gy),
                           _f32(self.gpsi), self.scenario)

    def pack(self) -> bytes:
        return struct.pack("<Q3fB", self.t_us, self.gx, self.gy, self.gpsi,
                           self.scenario)

    @staticmethod
    def unpack(b: bytes) -> "PontoonGoal":
        t = struct.unpack("<Q3fB", b)
        return PontoonGoal(t[0], float(t[1]), float(t[2]), float(t[3]), int(t[4]))


# ─────────────────────────────────────────────────────────────────────────────
# C 내부 : /mothership_data   (10 Hz)  — 실물 tug_interfaces/msg/MothershipData
#   MAVLink MOTHERSHIP_DATA id 52502 / crc_extra 123 / payload "<Qfff"
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class MothershipData:
    t_us: int = 0
    tau_x: float = 0.0
    tau_y: float = 0.0
    tau_n: float = 0.0

    MAVLINK_ID = 52502
    MAVLINK_CRC_EXTRA = 123
    MAVLINK_PAYLOAD = "<Qfff"      # ★ float32 (usv_tug.xml)
    WIRE_DTYPE = "float64"         # ROS 2 msg 는 float64

    def tau(self) -> np.ndarray:
        return np.array([self.tau_x, self.tau_y, self.tau_n], dtype=np.float64)

    @staticmethod
    def from_tau(t_us: int, tau: np.ndarray) -> "MothershipData":
        return MothershipData(int(t_us), float(tau[0]), float(tau[1]), float(tau[2]))

    def to_wire(self) -> "MothershipData":
        return MothershipData(self.t_us, *(float(x) for x in self.tau()))

    def pack(self) -> bytes:
        """ROS 2 경로 (float64)."""
        return struct.pack("<Q3d", self.t_us, self.tau_x, self.tau_y, self.tau_n)

    @staticmethod
    def unpack(b: bytes) -> "MothershipData":
        t = struct.unpack("<Q3d", b)
        return MothershipData(t[0], t[1], t[2], t[3])

    def pack_mavlink_payload(self) -> bytes:
        """MAVLink MOTHERSHIP_DATA payload — ★ float32 로 깎인다."""
        return struct.pack(self.MAVLINK_PAYLOAD, self.t_us,
                           self.tau_x, self.tau_y, self.tau_n)

    @staticmethod
    def unpack_mavlink_payload(b: bytes) -> "MothershipData":
        t = struct.unpack(MothershipData.MAVLINK_PAYLOAD, b)
        return MothershipData(t[0], float(t[1]), float(t[2]), float(t[3]))


# ─────────────────────────────────────────────────────────────────────────────
# C → D : /tug_command   (50 Hz)  — 실물 tug_interfaces/msg/TugCommand
#   MAVLink TUG_COMMAND id 52503 / crc_extra 16 / payload "<Qdddddd"  (양자화 없음)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TugCommand:
    t_us: int = 0
    tau_x_one: float = 0.0
    tau_y_one: float = 0.0
    tau_n_one: float = 0.0
    tau_x_two: float = 0.0
    tau_y_two: float = 0.0
    tau_n_two: float = 0.0

    MAVLINK_ID = 52503
    MAVLINK_CRC_EXTRA = 16
    MAVLINK_PAYLOAD = "<Qdddddd"   # ★ float64 → 손실 없음
    WIRE_DTYPE = "float64"

    @staticmethod
    def from_tau6(t_us: int, tau6: np.ndarray) -> "TugCommand":
        """tau6 = [X1, Y1, N1, X2, Y2, N2]."""
        a = np.asarray(tau6, dtype=np.float64).ravel()
        return TugCommand(int(t_us), *(float(x) for x in a))

    def tau_one(self) -> np.ndarray:
        return np.array([self.tau_x_one, self.tau_y_one, self.tau_n_one])

    def tau_two(self) -> np.ndarray:
        return np.array([self.tau_x_two, self.tau_y_two, self.tau_n_two])

    def to_wire(self) -> "TugCommand":
        return TugCommand(self.t_us, self.tau_x_one, self.tau_y_one, self.tau_n_one,
                          self.tau_x_two, self.tau_y_two, self.tau_n_two)

    def pack(self) -> bytes:
        return struct.pack(self.MAVLINK_PAYLOAD, self.t_us,
                           self.tau_x_one, self.tau_y_one, self.tau_n_one,
                           self.tau_x_two, self.tau_y_two, self.tau_n_two)

    @staticmethod
    def unpack(b: bytes) -> "TugCommand":
        t = struct.unpack(TugCommand.MAVLINK_PAYLOAD, b)
        return TugCommand(t[0], *(float(x) for x in t[1:]))

    pack_mavlink_payload = pack

    @staticmethod
    def unpack_mavlink_payload(b: bytes) -> "TugCommand":
        return TugCommand.unpack(b)


# ─────────────────────────────────────────────────────────────────────────────
# C → 로그 : /allocation_debug  — 실물 tug_interfaces/msg/AllocationDebug
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class AllocationDebug:
    t_us: int = 0
    tau_x: float = 0.0
    tau_y: float = 0.0
    tau_n: float = 0.0
    tau_real_x: float = 0.0
    tau_real_y: float = 0.0
    tau_real_n: float = 0.0
    error_x: float = 0.0
    error_y: float = 0.0
    error_n: float = 0.0
    ok: int = 1
    saturated: int = 0
    solve_time_us: float = 0.0
    marg_one: float = 1.0
    marg_two: float = 1.0

    WIRE_DTYPE = "float64"

    def pack(self) -> bytes:
        return struct.pack("<Q9dBBd2d", self.t_us,
                           self.tau_x, self.tau_y, self.tau_n,
                           self.tau_real_x, self.tau_real_y, self.tau_real_n,
                           self.error_x, self.error_y, self.error_n,
                           self.ok, self.saturated, self.solve_time_us,
                           self.marg_one, self.marg_two)

    @staticmethod
    def unpack(b: bytes) -> "AllocationDebug":
        t = struct.unpack("<Q9dBBd2d", b)
        return AllocationDebug(t[0], *t[1:10], ok=int(t[10]), saturated=int(t[11]),
                               solve_time_us=t[12], marg_one=t[13], marg_two=t[14])


# ─────────────────────────────────────────────────────────────────────────────
# D → A : 터그 추진기 실제 발생 힘
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TugThrust:
    t_us: int = 0
    f_one: np.ndarray = field(default_factory=lambda: np.zeros(3))
    f_two: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tau_one: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tau_two: np.ndarray = field(default_factory=lambda: np.zeros(3))

    WIRE_DTYPE = "float64"

    def tau6(self) -> np.ndarray:
        """플랜트가 받는 u6 = [X1,Y1,N1,X2,Y2,N2]."""
        return np.concatenate([self.tau_one, self.tau_two])

    def to_wire(self) -> "TugThrust":
        return self

    def pack(self) -> bytes:
        return struct.pack("<Q12d", self.t_us,
                           *np.concatenate([self.f_one, self.f_two,
                                            self.tau_one, self.tau_two]))

    @staticmethod
    def unpack(b: bytes) -> "TugThrust":
        t = struct.unpack("<Q12d", b)
        a = np.array(t[1:], dtype=np.float64)
        return TugThrust(t[0], a[0:3], a[3:6], a[6:9], a[9:12])


# ─────────────────────────────────────────────────────────────────────────────
# NUC → 마더십 Pixhawk : mode flag
# ─────────────────────────────────────────────────────────────────────────────
MODE_MANUAL = 0
MODE_RL = 1


@dataclass
class MothershipMode:
    t_us: int = 0
    mode: int = MODE_RL

    WIRE_DTYPE = "uint8"

    def to_wire(self) -> "MothershipMode":
        return self

    def pack(self) -> bytes:
        return struct.pack("<QB", self.t_us, self.mode)

    @staticmethod
    def unpack(b: bytes) -> "MothershipMode":
        t = struct.unpack("<QB", b)
        return MothershipMode(t[0], int(t[1]))


# ─────────────────────────────────────────────────────────────────────────────
# 토픽 이름 ↔ 타입 등록부 (전송계층이 쓴다)
# ─────────────────────────────────────────────────────────────────────────────
TOPICS = {
    "/pontoon/state":    PontoonState,
    "/pontoon/goal":     PontoonGoal,
    "/mothership_data":  MothershipData,
    "/tug_command":      TugCommand,
    "/allocation_debug": AllocationDebug,
    "/tug/thrust":       TugThrust,
    "/fmu/in/mothership_mode": MothershipMode,
}

# ROS 2 QoS (schema.yaml 과 동일. ros2_stub.py 가 쓴다)
QOS = {
    "/pontoon/state":          dict(reliability="RELIABLE",    durability="VOLATILE",        depth=1),
    "/pontoon/goal":           dict(reliability="RELIABLE",    durability="VOLATILE",        depth=1),
    "/mothership_data":        dict(reliability="BEST_EFFORT", durability="VOLATILE",        depth=1),
    "/tug_command":            dict(reliability="BEST_EFFORT", durability="VOLATILE",        depth=1),
    "/allocation_debug":       dict(reliability="BEST_EFFORT", durability="VOLATILE",        depth=1),
    "/tug/thrust":             dict(reliability="BEST_EFFORT", durability="VOLATILE",        depth=1),
    "/fmu/in/mothership_mode": dict(reliability="BEST_EFFORT", durability="TRANSIENT_LOCAL", depth=1),
}


def field_names(cls) -> List[str]:
    return [f.name for f in dc_fields(cls)]
