# -*- coding: utf-8 -*-
"""mavlink/gen_dialect.py — `mavlink/usv_tug.xml` 로 dialect 를 생성하고 검사한다.

원본: `cooperative-tug-experiment/experiment/nuc 코드/nuc_ws/src/tug_bridge/tools/gen_dialect.sh`
(타인 저장소. 읽기만 했다). 이 파일은 그 bash 스크립트의 **윈도우/venv 이식본**이다.
바뀐 점은 두 가지뿐이다: ① `python3` 대신 현재 인터프리터 ② `EXPECTED` 에 **52501 행 추가**.

검사 목적: PX4 와 NUC 가 서로 다른 `CRC_EXTRA` 를 쓰면 프레임이 **전부 조용히 버려진다.**
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "gen")
HEADER_AND_CRC = 12          # MAVLink v2 헤더 10 B + CRC 2 B (서명 없음)

# (이름, msgid, CRC_EXTRA, struct 포맷, payload B, 최대 프레임 B)
#   52502 / 52503 은 원본 스크립트의 값을 그대로 둔다 (회귀검사).
#   52501 의 CRC_EXTRA 는 **측정값**이다 — 손으로 적지 않는다.
EXPECTED = (
    ("MOTHERSHIP_DATA", 52502, 123, "<Qfff", 20, 32),
    ("TUG_COMMAND", 52503, 16, "<Qdddddd", 56, 68),
)


def generate():
    import pymavlink
    pdir = os.path.dirname(pymavlink.__file__)
    tmp = tempfile.mkdtemp()
    try:
        shutil.copy(os.path.join(HERE, "usv_tug.xml"), tmp)
        for x in ("development", "common", "standard", "minimal"):
            shutil.copy(os.path.join(pdir, "dialects", "v20", x + ".xml"), tmp)
        if not os.path.isdir(OUT):
            os.makedirs(OUT)
        open(os.path.join(OUT, "__init__.py"), "w").close()
        subprocess.check_call(
            [sys.executable, "-m", "pymavlink.tools.mavgen", "--lang=Python3",
             "--wire-protocol=2.0", "--output=" + os.path.join(OUT, "usv_tug.py"),
             os.path.join(tmp, "usv_tug.xml")])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def check():
    sys.path.insert(0, OUT)
    import usv_tug as m
    rows = []
    for name, msg_id, crc, fmt, payload, frame in EXPECTED:
        c = getattr(m, "MAVLink_" + name.lower() + "_message")
        assert getattr(m, "MAVLINK_MSG_ID_" + name) == msg_id, (name, "ID")
        assert c.crc_extra == crc, (name, "CRC_EXTRA", c.crc_extra)
        assert c.unpacker.format == fmt, (name, "format", c.unpacker.format)
        assert c.unpacker.size == payload, (name, "payload", c.unpacker.size)
        assert c.unpacker.size + HEADER_AND_CRC == frame, (name, "frame")
        rows.append((name, msg_id, crc, fmt, payload, frame, "회귀 OK"))

    # 52501 은 신설이므로 기대값을 모른다 → 측정해서 출력한다.
    c = m.MAVLink_mothership_state_message
    rows.append(("MOTHERSHIP_STATE", m.MAVLINK_MSG_ID_MOTHERSHIP_STATE,
                 c.crc_extra, c.unpacker.format, c.unpacker.size,
                 c.unpacker.size + HEADER_AND_CRC, "측정값(신설)"))
    for r in rows:
        print("%-18s id %5d  CRC_EXTRA %3d  %-14s payload %2d B  프레임 %2d B  %s" % r)
    print("필드순서 :", ", ".join(c.fieldnames))
    return rows


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(HERE))
    try:
        from _console import utf8_stdout
        utf8_stdout()
    except Exception:
        pass
    generate()
    check()
