# -*- coding: utf-8 -*-
"""전송계층 왕복 시험 — 어디서 양자화가 일어나는지 수치로 고정한다.

이 시험이 존재하는 이유
-----------------------
합격 기준(20/20, 0.01 m)이 깨질 때 원인 후보가 3개다: ONNX, 지연, 직렬화
양자화. 이 파일은 **세 번째 후보를 수치로 닫는다** — 각 토픽의 왕복 오차가
여기 기록된 값보다 크면 메시지 정의가 바뀐 것이다.
"""
from __future__ import annotations

import numpy as np

import _common  # noqa: F401  (sys.path 설정)

from iface.mavlink_wire import (CRC_EXTRA, MSG_MOTHERSHIP_DATA,
                                MSG_TUG_COMMAND, MavSeq, crc16_mcrf4xx,
                                frame_len, pack_frame, unpack_frame)
from iface.messages import (AllocationDebug, MothershipData, PontoonState,
                            TugCommand, TugThrust)

# 왕복 허용오차 — 메시지 정의로부터 결정된 값이다 (float32 eps = 1.19e-7)
TOL_F32_REL = 2.0e-7
TOL_F64 = 0.0


def test_pontoon_state_float32():
    """/pontoon/state 는 <Q10fB 이므로 float32 상대오차까지만 보존된다."""
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        s6 = rng.uniform(-3.0, 3.0, 6)
        r1 = rng.uniform(-2.0, 2.0, 2)
        r2 = rng.uniform(-2.0, 2.0, 2)
        m = PontoonState.from_plant(123456, s6, r1, r2)
        back = PontoonState.unpack(m.to_wire().pack())
        got = np.concatenate([back.nu(), back.eta(), back.r1(), back.r2()])
        exp = np.concatenate([s6[:3], s6[3:], r1, r2])
        rel = np.abs(got - exp) / np.maximum(np.abs(exp), 1e-9)
        worst = max(worst, float(rel.max()))
        assert back.t_us == 123456
        assert back.valid == 1
    assert worst < TOL_F32_REL, "float32 왕복 상대오차 %.3e" % worst
    return "float32 상대오차 max %.3e (허용 %.1e)" % (worst, TOL_F32_REL)


def test_mothership_data_float32():
    """/mothership_data 는 <Qfff. tau_d 60 N 기준 절대오차 ~1e-5 N."""
    worst = 0.0
    for tau in ([60.0, -60.0, 60.0], [0.0, 0.0, 0.0], [-13.98963243, 1.0, -7.5]):
        m = MothershipData.from_tau(1000, np.asarray(tau))
        back = MothershipData.unpack(m.to_wire().pack())
        worst = max(worst, float(np.abs(back.tau() - np.asarray(tau)).max()))
    assert worst < 1e-4, "tau_d 왕복 절대오차 %.3e N" % worst
    return "tau_d 절대오차 max %.3e N (60 N 기준)" % worst


def test_tug_command_float64_exact():
    """/tug_command 는 <Qdddddd. **양자화가 없어야 한다** (오차 0)."""
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(200):
        u6 = rng.uniform(-30.0, 36.0, 6)
        m = TugCommand.from_tau6(7, u6)
        back = TugCommand.unpack(m.to_wire().pack())
        got = np.concatenate([back.tau_one(), back.tau_two()])
        worst = max(worst, float(np.abs(got - u6).max()))
    assert worst == TOL_F64, "float64 경로에 양자화가 생겼다: %.3e" % worst
    return "float64 왕복 오차 %.1e (정확히 0)" % worst


def test_tug_thrust_float64_exact():
    rng = np.random.default_rng(2)
    f1 = rng.uniform(-28.0, 35.0, 3)
    f2 = rng.uniform(-28.0, 35.0, 3)
    t1 = rng.uniform(-60.0, 60.0, 3)
    t2 = rng.uniform(-60.0, 60.0, 3)
    m = TugThrust(t_us=9, f_one=f1, f_two=f2, tau_one=t1, tau_two=t2)
    back = TugThrust.unpack(m.to_wire().pack())
    err = max(float(np.abs(back.f_one - f1).max()),
              float(np.abs(back.tau6() - np.concatenate([t1, t2])).max()))
    assert err == 0.0
    return "오차 %.1e" % err


def test_allocation_debug_roundtrip():
    m = AllocationDebug(t_us=11, tau_x=1.0, tau_y=2.0, tau_n=3.0,
                        tau_real_x=1.1, tau_real_y=2.2, tau_real_n=3.3,
                        error_x=0.1, error_y=0.2, error_n=0.3,
                        ok=1, saturated=0, solve_time_us=123.4,
                        marg_one=0.25, marg_two=0.26)
    back = AllocationDebug.unpack(m.pack())
    assert back.ok == 1 and back.saturated == 0
    assert abs(back.solve_time_us - 123.4) == 0.0
    assert abs(back.marg_two - 0.26) == 0.0
    return "15 필드 왕복 OK"


def test_mavlink_crc_known_vector():
    """CRC-16/MCRF4XX 자체 검증. '123456789' -> 0x6F91 (표준 시험벡터)."""
    got = crc16_mcrf4xx(b"123456789")
    assert got == 0x6F91, "CRC 구현이 틀렸다: 0x%04X" % got
    return "0x%04X (표준벡터 일치)" % got


def test_mavlink_frame_roundtrip():
    """MAVLink v2 프레이밍 + trailing-zero truncation 복원."""
    seq = MavSeq()
    cmd = TugCommand.from_tau6(0, np.array([5.0, -3.0, 2.0, -4.0, 6.0, -1.0]))
    payload = cmd.pack_mavlink_payload()
    assert len(payload) == 56, "payload 길이 %d (<Qdddddd = 56)" % len(payload)
    frame = pack_frame(MSG_TUG_COMMAND, payload, seq.advance())
    assert frame[0] == 0xFD
    assert len(frame) <= frame_len(56)
    mid, p, s = unpack_frame(frame, expect_len=56)
    assert mid == MSG_TUG_COMMAND and s == 0
    back = TugCommand.unpack_mavlink_payload(p)
    err = float(np.abs(np.concatenate([back.tau_one(), back.tau_two()])
                       - np.array([5.0, -3.0, 2.0, -4.0, 6.0, -1.0])).max())
    assert err < 1e-12
    return "frame %d B, 왕복 오차 %.1e" % (len(frame), err)


def test_mavlink_crc_detects_corruption():
    seq = MavSeq()
    m = MothershipData.from_tau(0, np.array([10.0, -20.0, 30.0]))
    frame = bytearray(pack_frame(MSG_MOTHERSHIP_DATA,
                                 m.pack_mavlink_payload(), seq.advance()))
    frame[12] ^= 0x01          # payload 1 비트 손상
    try:
        unpack_frame(bytes(frame))
    except ValueError:
        return "손상 프레임을 CRC 로 거부함 (정상)"
    raise AssertionError("CRC 가 손상을 잡지 못했다")


def test_mavlink_seq_advances():
    """tug_tx.py 주석: seq 를 안 올리면 수신측이 'rx loss 235.9%' 를 본다."""
    seq = MavSeq()
    got = [seq.advance() for _ in range(300)]
    assert got[:3] == [0, 1, 2]
    assert got[256] == 0, "seq 가 255 에서 wrap 하지 않는다"
    return "0..255 wrap 확인"


def test_crc_extra_constants():
    """실물 다이얼렉트에서 인용한 상수가 바뀌지 않았는지 고정한다."""
    assert CRC_EXTRA[MSG_MOTHERSHIP_DATA] == 123
    assert CRC_EXTRA[MSG_TUG_COMMAND] == 16
    return "52502/123, 52503/16 (usv_tug.py 인용값)"

# ---------------------------------------------------------------------------
# 52501 MOTHERSHIP_STATE — ★ S5 에서 신설한 메시지 (사용자 결정 2026-11-22 오후)
#
# 위의 시험들은 `iface/mavlink_wire.py`(inproc 경로의 자체 프레이밍)를 재고,
# 아래는 **SITL/실물이 실제로 쓰는 경로**를 잰다:
#   생성 dialect `mavlink/gen/usv_tug.py` + 벤더 `pack_and_advance`.
# 두 경로가 따로 있으므로 **둘 다 재야** 52501 이 와이어를 건너는 것이 증명된다.
# ---------------------------------------------------------------------------

STATE_FIELDS = ("timestamp", "x", "y", "psi", "u", "v", "r",
                "x_ref", "y_ref", "psi_ref",
                "scenario_mode", "rc_on_off", "on_off")


def _state_wire_roundtrip(vals):
    """13 필드를 dialect 로 인코딩 → 프레임 → 파싱해서 되돌려 받는다."""
    from sitl import bootstrap
    bootstrap.setup()                      # vendor/ · shim/ 을 sys.path 에 올린다
    from mavlink.gen import usv_tug as dialect
    from tug_bridge.mav_link import pack_and_advance

    tx = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    rx = dialect.MAVLink(None, srcSystem=0, srcComponent=0)
    rx.robust_parsing = True
    msg = tx.mothership_state_encode(*vals)
    buf = pack_and_advance(tx, msg)        # ⚠️ msg.pack() 은 seq 를 안 올린다
    got = [m for m in (rx.parse_buffer(buf) or [])
           if m.get_msgId() == dialect.MAVLINK_MSG_ID_MOTHERSHIP_STATE]
    assert len(got) == 1, "52501 프레임이 1개로 파싱되지 않았다 (%d)" % len(got)
    return got[0], buf, dialect


def test_mothership_state_52501_roundtrip():
    """13 필드 전부 왕복. float 9개는 **float32** 라 그 양자화까지만 보존된다.

    payload 47 B = uint64(8) + float32×9(36) + uint8×3(3).
    """
    import numpy as _np
    vals = [1234567890123456,
            -12.3456789, 7.65432109, -3.0415926,       # x, y, psi
            1.2345678, -0.87654321, 0.54321098,        # u, v, r
            0.5, -0.25, 1.5707963,                     # x_ref, y_ref, psi_ref
            3, 1, 1]                                   # scenario_mode, rc, on
    m, buf, dl = _state_wire_roundtrip(vals)
    assert m.get_srcSystem() == 1 and m.get_srcComponent() == 1,         "sysid/compid 가 마더십(1/1)이 아니다"
    assert int(m.timestamp) == vals[0], "uint64 timestamp 가 깨졌다"
    worst = 0.0
    for i, name in enumerate(STATE_FIELDS[1:10], start=1):
        want = float(_np.float32(vals[i]))             # float32 양자화 기대값
        got = float(getattr(m, name))
        assert got == want, "%s: %r != %r (float32 기대값)" % (name, got, want)
        worst = max(worst, abs(got - vals[i]))
    for i, name in enumerate(STATE_FIELDS[10:], start=10):
        assert int(getattr(m, name)) == vals[i], "%s 가 깨졌다" % name
    return "13 필드 왕복 OK, frame %d B, float32 최대오차 %.2e" % (len(buf), worst)


def test_mothership_state_52501_zero_truncation():
    """MAVLink v2 는 **뒤쪽 0 바이트를 잘라 보낸다** → 복원되는지 본다.

    `scenario_mode=rc_on_off=on_off=0` 이고 목표자세도 0 이면 payload 뒤가
    전부 0 이라 프레임이 47 B 보다 짧아진다. 수신측이 0 으로 되채우지 않으면
    `on_off` 가 쓰레기값이 되고 **정책이 수동개입으로 오인**된다.
    """
    vals = [0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0]
    m, buf, dl = _state_wire_roundtrip(vals)
    for name in STATE_FIELDS:
        if name == "x":
            continue
        assert float(getattr(m, name)) == 0.0, "%s 가 0 으로 복원되지 않았다" % name
    assert float(m.x) == 1.0
    assert len(buf) < 12 + 47 + 2, "뒤쪽 0 이 잘리지 않았다 (%d B)" % len(buf)
    return "truncation 후 복원 OK, frame %d B (< 61 B)" % len(buf)


def test_mothership_state_52501_crc_extra():
    """`CRC_EXTRA` 는 필드집합의 지문이다 — 바뀌면 와이어가 조용히 끊긴다.

    ⚠️ 52501 을 추가해도 `52502/52503` 의 `CRC_EXTRA` 는 **불변**이어야 한다
    (하위호환 근거. `CLAUDE.md` §9 2026-11-22 오후). 그래서 세 값을 함께 고정한다.
    """
    from sitl import bootstrap
    bootstrap.setup()
    from mavlink.gen import usv_tug as dialect
    mm = dialect.mavlink_map
    got = {mid: mm[mid].crc_extra for mid in (52501, 52502, 52503)}
    assert got == {52501: 119, 52502: 123, 52503: 16},         "CRC_EXTRA 가 바뀠다: %r" % got
    return "52501/119 (신설), 52502/123 · 52503/16 불변"
