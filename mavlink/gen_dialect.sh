#!/usr/bin/env bash
# ⚠️ 이 파일은 사본이다. 원본: cooperative-tug-experiment/experiment/nuc 코드/nuc_ws/src/
#    tug_bridge/tools/gen_dialect.sh:20 (타인 저장소 — 읽기만 했고 수정·푸시하지 않았다).
#    바꾼 것은 EXPECTED 에 52501 한 행을 **추가**한 것뿐이다 (S10 패치에 포함).
#    윈도우/venv 에서는 이 bash 대신 mavlink/gen_dialect.py 를 쓴다 (python3 가 없다).
# tools/gen_dialect.sh: usv_tug.xml로 dialect를 생성하고 ID, CRC_EXTRA, 길이를 검사한다.
# 하나라도 다르면 0이 아닌 값으로 끝난다. PX4와 NUC가 서로 다른 값을 쓰면 프레임이 전부 버려진다.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
PYMAV_DIR="$(python3 -c 'import pymavlink,os;print(os.path.dirname(pymavlink.__file__))')"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
cp "$HERE/mavlink/usv_tug.xml" "$TMP/"
for x in development common standard minimal; do cp "$PYMAV_DIR/dialects/v20/$x.xml" "$TMP/"; done
OUT="$HERE/tug_bridge/mavlink_dialect"
mkdir -p "$OUT"
touch "$OUT/__init__.py"
python3 -m pymavlink.tools.mavgen --lang=Python3 --wire-protocol=2.0 \
    --output="$OUT/usv_tug.py" "$TMP/usv_tug.xml"
python3 - "$OUT" <<'EOF'
import sys; sys.path.insert(0, sys.argv[1])
import usv_tug as m
HEADER_AND_CRC = 12   # MAVLink v2 헤더 10 B + CRC 2 B (서명 없음)
EXPECTED = (
    # 52501 은 2026-11-22 신설. CRC_EXTRA 119 는 mavgen 산출물에서 측정한 값이다.
    ('MOTHERSHIP_STATE', m.MAVLink_mothership_state_message, 52501, 119, '<QfffffffffBBB', 47, 59),
    ('MOTHERSHIP_DATA', m.MAVLink_mothership_data_message, 52502, 123, '<Qfff', 20, 32),
    ('TUG_COMMAND', m.MAVLink_tug_command_message, 52503, 16, '<Qdddddd', 56, 68),
)
for name, c, msg_id, crc, fmt, payload, frame in EXPECTED:
    assert getattr(m, 'MAVLINK_MSG_ID_' + name) == msg_id, (name, 'ID', getattr(m, 'MAVLINK_MSG_ID_' + name))
    assert c.crc_extra == crc, (name, 'CRC_EXTRA', c.crc_extra)
    assert c.unpacker.format == fmt, (name, 'format', c.unpacker.format)
    assert c.unpacker.size == payload, (name, 'payload', c.unpacker.size)
    assert c.unpacker.size + HEADER_AND_CRC == frame, (name, 'frame', c.unpacker.size + HEADER_AND_CRC)
    print(f'OK: {name} ID {msg_id}, CRC_EXTRA {crc}, payload {payload} B ({fmt}), 최대 프레임 {frame} B')
EOF
