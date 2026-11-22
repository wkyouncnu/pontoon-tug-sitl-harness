# -*- coding: utf-8 -*-
"""harness_sitl/iface/mavlink_wire.py — MAVLink v2 프레이밍 (직접 구현).

왜 직접 구현했는가
------------------
* `pymavlink` 이 `plan_pontoon_rl/.venv` 에 설치되어 있지 않다.
* 실물 생성 다이얼렉트
  `cooperative-tug-experiment-main/experiment/nuc 코드/nuc_ws/src/tug_bridge/
   tug_bridge/mavlink_dialect/usv_tug.py`
  는 **읽기전용 폴더**에 있고 `pymavlink` 에 의존한다.
* 따라서 프레이밍(CRC-16/MCRF4XX + crc_extra)만 직접 구현하고,
  **메시지 id 와 crc_extra 상수는 그 파일에서 인용**했다:
      MAVLINK_MSG_ID_MOTHERSHIP_DATA = 52502,  crc_extra = 123
      MAVLINK_MSG_ID_TUG_COMMAND     = 52503,  crc_extra = 16
  (usv_tug.py L6539/L6540, L6777, L6819)

프레임 구조 (MAVLink v2)
------------------------
    0xFD | len | incompat | compat | seq | sysid | compid | msgid(3,LE) | payload | crc(2,LE)
  CRC 는 start byte 를 제외한 헤더 + payload + crc_extra 에 대해 계산한다.
  v2 는 payload 뒤쪽 0 바이트를 잘라 보낸다(trailing-zero truncation) →
  수신측이 0 으로 복원한다. 본 구현도 그대로 재현한다.

실물 송신 파라미터 (tug_tx.py)
------------------------------
    own_sysid 250, own_compid 191, send_rate_hz 50, input_timeout_s 0.5
    ★ `pack_and_advance()` 가 seq 를 직접 올려야 한다 — 안 올리면 수신측이
      "rx loss 235.9%" 를 보고한다 (mav_link.py 주석).
"""
from __future__ import annotations

import struct
from typing import Tuple

STX_V2 = 0xFD
OWN_SYSID = 250       # tug_tx.py 기본값
OWN_COMPID = 191      # tug_tx.py 기본값 (MAV_COMP_ID_ONBOARD_COMPUTER)

# ── usv_tug.py 에서 인용한 상수 ──────────────────────────────────────────────
MSG_MOTHERSHIP_DATA = 52502
CRC_EXTRA_MOTHERSHIP_DATA = 123
MSG_TUG_COMMAND = 52503
CRC_EXTRA_TUG_COMMAND = 16

CRC_EXTRA = {
    MSG_MOTHERSHIP_DATA: CRC_EXTRA_MOTHERSHIP_DATA,
    MSG_TUG_COMMAND: CRC_EXTRA_TUG_COMMAND,
}


# ── CRC-16/MCRF4XX (MAVLink X.25) ───────────────────────────────────────────
def crc16_mcrf4xx(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def _truncate(payload: bytes) -> bytes:
    """MAVLink v2 trailing-zero truncation. 최소 1 바이트는 남긴다."""
    n = len(payload)
    while n > 1 and payload[n - 1] == 0:
        n -= 1
    return payload[:n]


class MavSeq:
    """송신 시퀀스 카운터. tug_tx.py 와 같은 역할."""

    def __init__(self, start: int = 0):
        self.seq = start & 0xFF

    def advance(self) -> int:
        s = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return s


def pack_frame(msgid: int, payload: bytes, seq: int,
               sysid: int = OWN_SYSID, compid: int = OWN_COMPID) -> bytes:
    """payload 를 MAVLink v2 프레임으로 감싼다."""
    if msgid not in CRC_EXTRA:
        raise KeyError("crc_extra 미등록 msgid=%d" % msgid)
    p = _truncate(payload)
    hdr = struct.pack("<BBBBBB", len(p), 0, 0, seq & 0xFF, sysid, compid)
    mid = struct.pack("<I", msgid)[:3]
    body = hdr + mid + p
    crc = crc16_mcrf4xx(body)
    crc = crc16_mcrf4xx(bytes([CRC_EXTRA[msgid]]), crc)
    return bytes([STX_V2]) + body + struct.pack("<H", crc)


def unpack_frame(frame: bytes, expect_len: int = None) -> Tuple[int, bytes, int]:
    """프레임 → (msgid, payload, seq). CRC 불일치면 ValueError."""
    if len(frame) < 12 or frame[0] != STX_V2:
        raise ValueError("MAVLink v2 프레임이 아니다")
    plen, incompat, compat, seq, sysid, compid = struct.unpack("<BBBBBB", frame[1:7])
    msgid = struct.unpack("<I", frame[7:10] + b"\x00")[0]
    p = frame[10:10 + plen]
    rx_crc = struct.unpack("<H", frame[10 + plen:12 + plen])[0]
    crc = crc16_mcrf4xx(frame[1:10 + plen])
    if msgid not in CRC_EXTRA:
        raise KeyError("crc_extra 미등록 msgid=%d" % msgid)
    crc = crc16_mcrf4xx(bytes([CRC_EXTRA[msgid]]), crc)
    if crc != rx_crc:
        raise ValueError("CRC 불일치: rx=0x%04X calc=0x%04X (msgid=%d)"
                         % (rx_crc, crc, msgid))
    if expect_len is not None and len(p) < expect_len:
        p = p + b"\x00" * (expect_len - len(p))   # truncation 복원
    return msgid, p, seq


def frame_len(payload_len: int) -> int:
    """truncation 전 기준 최대 프레임 길이."""
    return 1 + 6 + 3 + payload_len + 2
